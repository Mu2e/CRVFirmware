setMode -pff
setMode -pff
addConfigDevice  -name "ROC_FPGA1_20240224_newCntResets" -path "/home/scorrodi/Documents/mu2e-daq-firmware-crv/ROC/FPGA1"
setSubmode -pffparallel
setAttribute -configdevice -attr multibootBpiType -value ""
addDesign -version 0 -name "0"
setAttribute -configdevice -attr compressed -value "FALSE"
setAttribute -configdevice -attr compressed -value "FALSE"
setAttribute -configdevice -attr autoSize -value "FALSE"
setAttribute -configdevice -attr fileFormat -value "bin"
setAttribute -configdevice -attr fillValue -value "FF"
setAttribute -configdevice -attr swapBit -value "TRUE"
setAttribute -configdevice -attr dir -value "UP"
setAttribute -configdevice -attr multiboot -value "FALSE"
setAttribute -configdevice -attr multiboot -value "FALSE"
setAttribute -configdevice -attr spiSelected -value "FALSE"
setAttribute -configdevice -attr spiSelected -value "FALSE"
addPromDevice -p 1 -size 1024 -name 1M
setMode -pff
setMode -pff
setMode -pff
setSubmode -pffparallel
setMode -pff
addDeviceChain -index 0
setMode -pff
addDeviceChain -index 0
setMode -pff
setSubmode -pffparallel
setMode -pff
setMode -pff
setMode -pff
addDeviceChain -index 0
addDevice -p 1 -file "/home/scorrodi/Documents/mu2e-daq-firmware-crv/ROC/FPGA1/ControllerFPGA_1.bit"
setAttribute -design -attr name -value "0"
setAttribute -design -attr endAddress -value "c3ce2"
setAttribute -design -attr endAddress -value "c3ce2"
setMode -pff
setSubmode -pffparallel
setAttribute -configdevice -attr swapBit -value "TRUE"
generate
setCurrentDesign -version 0
setMode -pff
setMode -bs
setMode -ss
setMode -sm
setMode -hw140
setMode -spi
setMode -acecf
setMode -acempm
setMode -pff
setMode -pff
saveProjectFile -file "/home/scorrodi/Documents/CRVFirmwareProdRoc/ROC/FPGA1//auto_project.ipf"
setMode -pff
setMode -bs
setMode -ss
setMode -sm
setMode -hw140
setMode -spi
setMode -acecf
setMode -acempm
setMode -pff
