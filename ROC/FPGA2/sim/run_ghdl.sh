#!/bin/bash
# Simulate FPGA2 (PortMatch_tb.vhd, complete top level) with GHDL.
#
#   sim/run_ghdl.sh         port match cases on the current sources, then the
#                           same stimulus on the firmware before the change and
#                           a comparison of the events sent to FPGA 1
#   sim/run_ghdl.sh full    port match cases only
#   BASE=<git ref> sim/run_ghdl.sh    reference firmware (default: 513c2ce, the last
#                                     commit before the EWT match and the port match)
#
# Uses a local ghdl if there is one, otherwise the ghdl/ghdl container image
# with podman or docker. The Xilinx cores are replaced by sim/core_models.vhd.
set -e
cd "$(dirname "$0")/.."
BASE=${BASE:-513c2cedbdd0e5d0d10835e67ab49759707f4668}
IMAGE=${GHDL_IMAGE:-docker.io/ghdl/ghdl:buster-mcode}
STAGE=${STAGE:-sim/build}

rm -rf "$STAGE"
mkdir -p "$STAGE/src/sim" "$STAGE/old"
cp Proj_Defs.vhd CRC32_D4.vhd FM_Tx_Rx.vhd Controller_FPGA2.vhd PortMatch.vhd PortMatch_tb.vhd "$STAGE/src/"
cp sim/unisim_stub.vhd sim/core_models.vhd "$STAGE/src/sim/"
cp sim/ghdl_inner.sh "$STAGE/"
for f in Proj_Defs CRC32_D4 FM_Tx_Rx Controller_FPGA2; do
    git show "$BASE:./$f.vhd" > "$STAGE/old/$f.vhd"
done

if command -v ghdl >/dev/null; then
    (cd "$STAGE" && sh ghdl_inner.sh "$@")
else
    CT=$(command -v podman || command -v docker)
    tar -C "$STAGE" -c . | "$CT" run --rm -i "$IMAGE" \
        sh -c 'mkdir /w && cd /w && tar x && sh ghdl_inner.sh "$@"' sh "$@"
fi
