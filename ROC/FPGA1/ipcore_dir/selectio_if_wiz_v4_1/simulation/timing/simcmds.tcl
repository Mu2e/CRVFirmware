# file: simcmds.tcl

# create the simulation script
vcd dumpfile isim.vcd
vcd dumpvars -m /selectio_if_wiz_v4_1_tb -l 0
wave add /
run 50000ns
quit

