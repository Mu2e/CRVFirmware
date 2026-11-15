#!/bin/sh
# Runs inside the staging directory prepared by run_ghdl.sh:
#   src/  current sources      old/  top level before the change
set -e
FLAGS="--std=08 --ieee=synopsys -fexplicit -frelaxed"
TOP=$PWD
COMMON="CRC32_D4.vhd FM_Tx_Rx.vhd"

build() { # directory with the sources
    (cd "$1" && ghdl -a $FLAGS --work=unisim "$TOP/src/sim/unisim_stub.vhd" \
        && ghdl -a $FLAGS Proj_Defs.vhd "$TOP/src/sim/core_models.vhd" $COMMON $2 Controller_FPGA2.vhd \
                   "$TOP/src/PortMatch_tb.vhd" \
        && ghdl -e $FLAGS PortMatch_tb)
}
sim() { # directory, generics
    d=$1; shift
    (cd "$d" && ghdl -r $FLAGS PortMatch_tb --ieee-asserts=disable "$@" 2>&1 || true) \
        | grep -v 'END OF SIMULATION\|simulation failed\|error: ' | sed 's/^.*:(report note): //'
}

build "$TOP/src" PortMatch.vhd
build "$TOP/old" ""

echo "##### port match cases (current sources)"
sim "$TOP/src" -gMODE=full | tee full.log
[ "$1" = full ] && exit 0

for m in reg reg2; do
    sim "$TOP/old" -gMODE=$m -gPMEN=0 > $m.old
    sim "$TOP/src" -gMODE=$m -gPMEN=0 > $m.off
    echo "##### $m: $(grep -c '^EV' $m.old) events before the change"
    grep '^EV' $m.old | cut -c1-60
    if cmp -s $m.old $m.off; then echo "$m, port match off: identical"; else echo "$m, port match off: DIFFERENT"; diff $m.old $m.off | head -20; fi
done
sim "$TOP/src" -gMODE=reg -gPMEN=1 > reg.on
if cmp -s reg.old reg.on; then echo "reg, port match on: identical"; else echo "reg, port match on: DIFFERENT"; diff reg.old reg.on | head -20; fi
