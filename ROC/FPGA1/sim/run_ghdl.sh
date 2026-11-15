#!/bin/bash
# Simulate the data request path of FPGA1 (EwtMatch_tb.vhd) with GHDL.
#
#   sim/run_ghdl.sh         all cases on the current sources, then the same
#                           normal / DCS / GR stimulus on the firmware before
#                           the change and a comparison of the transmitted words
#   sim/run_ghdl.sh full    all cases on the current sources only
#   BASE=<git ref> sim/run_ghdl.sh    reference firmware (default: 513c2ce, the last
#                                     commit before the EWT match and the port match)
#
# Uses a local ghdl if there is one, otherwise the ghdl/ghdl container image
# with podman or docker (the files are piped into the container, nothing is
# mounted). The Xilinx FIFO cores are replaced by sim/fifo_models.vhd.
set -e
cd "$(dirname "$0")/.."
BASE=${BASE:-513c2cedbdd0e5d0d10835e67ab49759707f4668}
IMAGE=${GHDL_IMAGE:-docker.io/ghdl/ghdl:buster-mcode}
STAGE=${STAGE:-sim/build}

rm -rf "$STAGE"
mkdir -p "$STAGE/src/sim" "$STAGE/old"
cp Project_Defs.vhd Project_Components.vhd crc.vhd EventBuilder.vhd PacketFormer.vhd \
   DRHandler.vhd LinkFlushCtrl.vhd EwtMatch_tb.vhd "$STAGE/src/"
cp sim/unisim_stub.vhd sim/fifo_models.vhd sim/legacy_wrap.vhd "$STAGE/src/sim/"
cp sim/ghdl_inner.sh "$STAGE/"
for f in EventBuilder PacketFormer; do
    git show "$BASE:./$f.vhd" | sed "s/\b$f\b/${f}_orig/g" > "$STAGE/old/${f}_orig.vhd"
done
# The old PacketFormer does not have DCSBuffRdCnt in the sensitivity list of its
# next state process. That has no effect on the hardware, but in simulation a
# DCS packet then only starts with the next data request. Complete the list.
sed -i 's/FormRst, EventBuffErr, Pkt_Timer, MarkerSync)/FormRst, EventBuffErr, Pkt_Timer, MarkerSync, DCSBuffRdCnt)/' \
    "$STAGE/old/PacketFormer_orig.vhd"

if command -v ghdl >/dev/null; then
    (cd "$STAGE" && sh ghdl_inner.sh "$@")
else
    CT=$(command -v podman || command -v docker)
    tar -C "$STAGE" -c . | "$CT" run --rm -i "$IMAGE" \
        sh -c 'mkdir /w && cd /w && tar x && sh ghdl_inner.sh "$@"' sh "$@"
fi
