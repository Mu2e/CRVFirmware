#!/bin/sh
# Runs inside the staging directory prepared by run_ghdl.sh:
#   src/  current sources      old/  EventBuilder / PacketFormer before the change
# Argument "full": only the full test on the current sources; further
# arguments go to the simulation (for example -gVERBOSE=true).
set -e
ONLY=$1
[ $# -gt 0 ] && shift
FLAGS="--std=93c --ieee=synopsys -fexplicit"
TOP=$PWD

build() { # work directory, files
    d=$1; shift
    mkdir -p "$d"
    (cd "$d" && ghdl -a $FLAGS --work=unisim "$TOP/src/sim/unisim_stub.vhd" \
             && ghdl -a $FLAGS "$@" && ghdl -e $FLAGS EwtMatch_tb)
}
sim() { # work directory, generics
    d=$1; shift
    (cd "$d" && ghdl -r $FLAGS EwtMatch_tb --ieee-asserts=disable-at-0 "$@" 2>&1)
}
COMMON="$TOP/src/Project_Defs.vhd $TOP/src/Project_Components.vhd $TOP/src/crc.vhd $TOP/src/sim/fifo_models.vhd"

echo "##### marker sampling"
mkdir -p "$TOP/ms"
(cd "$TOP/ms" && ghdl -a $FLAGS "$TOP/src/MarkerSample.vhd" "$TOP/src/MarkerSample_tb.vhd" \
              && ghdl -e $FLAGS MarkerSample_tb \
              && ghdl -r $FLAGS MarkerSample_tb --ieee-asserts=disable-at-0 2>&1) | sed 's/^.*:(report note): //'

echo "##### current sources, all cases"
build "$TOP/new" $COMMON "$TOP/src/EventBuilder.vhd" "$TOP/src/PacketFormer.vhd" \
      "$TOP/src/DRHandler.vhd" "$TOP/src/LinkFlushCtrl.vhd" "$TOP/src/EwtMatch_tb.vhd"
sim "$TOP/new" -gMODE=full "$@" | sed 's/^.*:(report /(/'   # a later -gMODE=... in the arguments wins
[ "$ONLY" = full ] && exit 0

echo "##### current sources, regression stimulus (normal events, DCS, GR)"
sim "$TOP/new" -gMODE=regress -gSTREAM_FILE=stream.txt | grep -i "passed\|failed\|error" || true

echo "##### firmware before the change, same stimulus"
build "$TOP/oldw" $COMMON "$TOP/old/EventBuilder_orig.vhd" "$TOP/old/PacketFormer_orig.vhd" \
      "$TOP/src/sim/legacy_wrap.vhd" "$TOP/src/LinkFlushCtrl.vhd" "$TOP/src/EwtMatch_tb.vhd"
sim "$TOP/oldw" -gLEGACY=true -gMODE=regress -gSTREAM_FILE=stream.txt | grep -i "passed\|failed\|error" || true

echo "##### transmitted words before vs. after the change"
if cmp "$TOP/oldw/stream.txt" "$TOP/new/stream.txt"; then
    echo "identical: $(wc -l < "$TOP/new/stream.txt") words"
else
    echo "DIFFERENT"
    diff "$TOP/oldw/stream.txt" "$TOP/new/stream.txt" | head -40
fi

echo "##### firmware before the change, one request without event (log only)"
sim "$TOP/oldw" -gLEGACY=true -gMODE=skip | sed 's/^.*:(report note): //'
