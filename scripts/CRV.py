import serial
import time
import colorama
from colorama import Fore, Back, Style 
from math import ceil, floor
import re
import math

class CRV:
    def __init__(self, port='/dev/ttyUSB0', verbose=False):
        self.ser = serial.Serial(port, 460800, timeout=0.2)
        self.verbose = verbose
        self.page_size = 0x400
        self.n_sample = 8
        self.rocFPGA = {1 : "4", 2:"8", 3:"C"}
        self.febFPGA = {0 : "0", 1 : "4", 2 : "8", 3 : "C"}

    def close(self):
        self.ser.close()

    def __del__(self):
        self.close()

    def cmd(self, cmd):
        self.ser.write((cmd+"\r").encode())
        #time.sleep(0.5)        

    def write(self, reg, value, lc=False):
        cmd = ""
        if lc:
            cmd += "LC "
        cmd += "WR "+reg+" "+value
        if self.verbose:
            print("send: ", cmd)
        self.cmd(cmd)
        if self.verbose:
            print("readback: ", self.ser.readline())
        self.ser.flushInput()

    def read(self, reg, n=1, lc=False):
        self.ser.flushInput()
        cmd = ""
        if lc:
            cmd += "LC "
        if n == 1:
            cmd += "RD "+reg
        else:
            cmd += "RDI "+reg+" "+str(n)
        if self.verbose:
            print("send: ", cmd)
        self.cmd(cmd)
        if self.verbose:
            print("readback: ", self.ser.readline())
        else:
            self.ser.readline()
        if lc:
            time.sleep(0.03)
        else:
            pass
            #time.sleep(0.2)
        raw = self.ser.read(self.ser.in_waiting)
        if self.verbose:
            print("read :", raw)
        return self.parse(raw).split()

    def parse(self, response): 
        return response.decode('utf-8').replace('Socket?>','').replace('P1>','').replace('\x00\x00\x00','').replace('\x00', ' ').replace('\n', ' ').replace('\r','').replace('>','')

    def readm(self, reg, n,  lc=False):
        nn = 1024
        if n > nn:
            out = self.readm(reg=reg, n=nn,   lc=lc) + self.readm(reg=reg, n=n-nn, lc=lc)

        self.ser.flushInput()
        cmd = ""
        if lc:
            cmd += "LC "
        if lc:
            cmd += "RDM "+reg+" "+hex(n)[2:]+" 20"
        else:
            cmd += "RDM "+reg+" "+str(n)+" 10"
        if self.verbose:
            print("send: ", cmd)
        self.cmd(cmd)
        if self.verbose:
            print("readback: ", self.ser.readline())
        else:
             self.ser.readline()
        out = []
        if lc:
            time.sleep(0.2)
        else:
            time.sleep(0.0004*n)
        while self.ser.in_waiting>0:
            raw = self.ser.read(self.ser.in_waiting)
            if self.verbose:
                print("read :", raw)
                print(" ")
                print(" ")
            out = out + self.parse(raw).split()
        return out

    def reset(self, lc=False):
        cmd = ""
        if lc:
            cmd += "LC "
        cmd += "RESET"
        self.cmd(cmd)
        if self.verbose:
            print("readback: ", self.ser.readline())

    def readOutput(self, n=1):
        data = self.readm("21", 30*n, lc=False)
        if self.verbose:
            for n_ in range(n):
                print(data[n_*30   :n_*30+10])
                print(data[n_*30+10:n_*30+20])
                print(data[n_*30+20:n_*30+30])
        return data

    def readOutTrace(self,n=1):
        data = self.readm("85", 10*n, lc=False)
        if self.verbose:
            for n_ in range(n):
                for d in data[n_*10   :n_*10+10]:
                    if d == "a050":
                        print(Fore.RED, d, Style.RESET_ALL, end=" ")
                    else:
                        print(d, end=" ")
                print(" ")
                #print(data[n_*10   :n_*10+10])
                #print(data[n_*10+10:n_*10+20])

    def readDReqTrace(self,n=1):
        data = self.readm("86", 9*n, lc=False)
        if self.verbose:
            for n_ in range(n):
                print(data[n_*9   :n_*9+9])
                #print(data[n_*9+10:n_*9+20])

    def readTriggers(self,n=1):
        data = self.readm("20", 20*n, lc=False)
        if self.verbose:
            for n_ in range(n):
                print(data[n_*20   :n_*20+10])
                print(data[n_*20+10:n_*20+20])

    def rocCnters(self):
        markers = self.read("41")
        heartb  = self.read("42")
        last    = self.read("43")
        buffer_ = self.read("3D")
        markers2 = self.read("46")
        markers3 = self.read("48")
        if not hasattr(self, 'markers_'):
            self.markers_ = 0 
        if not hasattr(self, 'heartb_'):
            self.heartb_ = 0
        print("words in buffer (9 per hb): ", buffer_)
        print("heart beats counter (sent, read): ", heartb, int(heartb[0][2:],16)-self.heartb_)
        print("marker count: ", markers, int(markers[0][2:],16)-self.markers_)
        print("marker count 3/2 ", markers2)
        print("marker count 4/5 ", markers3)
        print("time of last event window:", last)
        #print(markers, heartb, last)
        self.markers_ = int(markers[0][2:],16)
        self.heartb_ = int(heartb[0][2:],16)

    def rocDebBuffStatus(self):
       d = self.read("4C")[0]
       pattern = self.read("4E")
       mask    = self.read("4D")
       print("Patten: %s" % pattern[0])
       print("Mask: %s" % mask[0])
       full_  = (int(d[0],16) & 0x8) > 0
       empty_ = (int(d[0],16) & 0x4) > 0
       sel_   = (int(d[1],16) & 0x1) > 0
       reset_ = (int(d[0],16) & 0x1) > 0
       print("Sel:   %i" % sel_)
       print("Rst:   %s" % reset_)
       print("Empty: %s" % empty_)
       print("Full:  %s" % full_)

    def rocDebBuffTrig(self, pattern, ch="0"):
       self.write("4C", "1"+ch+pattern)
       time.sleep(0.1)
       self.write("4C", "0"+ch+pattern)

    def rocDebBuffGet(self, n=4):
        data = self.readm("4B", 20*n, lc=False)
        if self.verbose:
            for n_ in range(n):
                print(data[n_*20   :n_*20+10])
                print(data[n_*20+10:n_*20+20])


    def rocDuty(self):
       d = self.read("49")[0]
       print("Input 1: %.2f" % (int(d[:2],16)/0xff))
       print("Input 2: %.2f" % (int(d[2:],16)/0xff))

    def rocLock(self):
        self.write("17","12")
        self.write("18","12")
        return int(self.read("19")[0][2])

    def rocRx(self, n=1):
        data = self.readm("20", 20*n, lc=False)
        if self.verbose:
            for n_ in range(n):
                print(data[n_*20   :n_*20+10])
                print(data[n_*20+10:n_*20+20])

    def rocTx(self,n=1):
        return self.readOutTrace(n=2*n)

    def getTriggers(self, n=1):
        if self.verbose:
            print("send: UB2 "+str(n))
        self.cmd("UB2 "+str(n))
        if self.verbose:
            print("readback ", self.ser.readline())
        else:
            self.ser.readline()
        time.sleep(0.001)
        data = self.readm("20", 20*n, lc=False)
        if self.verbose:
            for n_ in range(n):
                print(data[n_*20   :n_*20+10])
                print(data[n_*20+10:n_*20+20])
        muB = []
        for n_ in range(n):
            muB.append(data[n_*20+3])
        return muB
    
    def rocDdrStatus(self, fpga=1):
        data = self.read(self.rocFPGA[fpga]+"02",4, lc=False)
        print("ROC DDR write ("+self.rocFPGA[fpga]+"02, "+self.rocFPGA[fpga]+"03): ", data[0], data[1])
        print("ROC DDR read ("+self.rocFPGA[fpga]+"04, "+self.rocFPGA[fpga]+"05):  ", data[2], data[3])

    def rocDdrRead(self, hi="0", lw="0", n=16, fpga=1):
        print("Reading ROC DDR ("+str(fpga)+") at ", hi, lw)
        self.write(self.rocFPGA[fpga]+"04", hi)
        self.write(self.rocFPGA[fpga]+"05", lw)
        data = self.readm(self.rocFPGA[fpga]+"07", n, lc=False)
        for n_ in range(n//16):
            print(data[n_*16:n_*16+16])
        return data

    def rocDdrReadEv(self, add0=True, fpga=1, getStatus=False):
        if add0:
            if self.verbose:
                 print("Reset ROC DDR ("+str(fpga)+") Read address")
                 self.write(self.rocFPGA[fpga] +"04", "0")
                 self.write(self.rocFPGA[fpga]+"05", "0")
        read_add = self.read(self.rocFPGA[fpga]+"04", 2, lc=False)
        n = int(self.read(self.rocFPGA[fpga]+"07", 1, lc=False)[0], 16)
        if n > 1024+4:
            data = self.readm(self.rocFPGA[fpga]+"07", 4, lc=False)
            print("CORRUPT  EVENT?", data)
            return
        n16 = math.ceil(n/16)*16 # the next event always starts at a full 16 
        if n%16 == 0: # needed because we get an additional 16 woords if we have an event that would exactly fit into 16 words
            n16 = n16 + 16
        print("Read event with %i words, read total %i words" % (n, n16))
        data = self.readm(self.rocFPGA[fpga]+"07", n16-1, lc=False)
        print("words: ", n ,"status: ", data[0], ", uB: ", data[1], data[2], "(at read add: ", read_add,")")
        ev = (n - 4)//(self.n_sample + 2)
        out_data = []
        for ev_ in range(ev):
            print("%i)" % (ev_ + 1), data[ev_*(self.n_sample + 2)+3:(ev_+1)*(self.n_sample + 2)+3])
            out_data.append(data[ev_*(self.n_sample + 2)+3:(ev_+1)*(self.n_sample + 2)+3])
        if getStatus:
            return data[0], data[1] + data[2]
        if ev > 0:
            print("filler: ", data[(ev_+1)*(self.n_sample + 2)+3:])
        else:
            print("filler: ", data[4:])
        return out_data

    def rocDdrReset(self, fpga=1):
        # Don't read out the DDR
        #print("Disable the DDR read sequence")
        self.write(self.rocFPGA[fpga]+"00", "A8", lc=False) # default A8
        print("Reset DDR write address")
        self.write(self.rocFPGA[fpga]+"02","0", lc=False)
        self.write(self.rocFPGA[fpga]+"03","0", lc=False)
        self.write(self.rocFPGA[fpga]+"04","0", lc=False)
        self.write(self.rocFPGA[fpga]+"05","0", lc=False)
        if self.verbose:
            print("The read/write points should now point to 0 0 0 20")
            self.rocDdrStatus(fpga=fpga)

    def febDdrStatus(self, fpga=0):
        data = self.read(self.febFPGA[fpga]+"02",4, lc=True)
        print("FEB DDR write ("+self.febFPGA[fpga]+"02, "+self.febFPGA[fpga]+"03): ", data[0], data[1], "->", hex((0x10000 * int(data[0],16) + int(data[1],16))//1024))
        print("FEB DDR read  ("+self.febFPGA[fpga]+"04, "+self.febFPGA[fpga]+"05): ", data[2], data[3])

    def febPageClear(self):
        for fpga in range(4):
            n_str = self.read("%x" % (fpga*4)+"0D", lc=True)[0]
            self.readm("%x" % (fpga*4)+"0C", n=int(n_str,16), lc=True)
            return self.read("317", lc=True)

    def febPageRead(self, hi="0", lw="0", fpga=0):
        print("Read FEB ("+str(fpga)+") Page ", hi, lw, end=" ")
        self.write("312", hi, lc=True)
        self.write("313", lw, lc=True)
        time.sleep(0.1)
        n = self.read(self.febFPGA[fpga]+"0D", lc=True)[0]
        if int(n, 16) == 1026:
            print("with NO (",int(n, 16),") events")
        else:
            print("with ", n, " (%i, ev: %i) words" % (int(n, 16), (int(n, 16)-3)/(2+self.n_sample)))
        data = self.readm(self.febFPGA[fpga]+"0C", n=int(n, 16), lc=True)
        ev = (int(n,16)-3)//(self.n_sample+2)
        #print("DEBUG", ev)
        print(data)
        for ev_ in range(ev):
            print("%02i) " % (ev_+1), data[ev_*(self.n_sample+2)+3:ev_*(self.n_sample+2)+3+(self.n_sample+2)])
        

    def febPageHeader(self, hi="0", lw="0", fpga=0):
        addr = int(hi + "%04x" % int(lw,16), 16) * self.page_size
        addr_str = "%08x" % addr
        addr_hi = addr_str[:4]
        addr_lw = addr_str[4:]
        print("Read FEB  DDR ("+str(fpga)+") for uB", hi, lw, " at ", addr_hi, addr_lw)
        self.write(self.febFPGA[fpga]+"04", addr_hi, lc=True)
        self.write(self.febFPGA[fpga]+"05", addr_lw, lc=True)
        data = self.readm(self.febFPGA[fpga]+"07", n=3, lc=True)
        if int(data[0],16)&(0x8000)>0:
            print("words (overflow!): ", data[0] ,"(%i, ev: %i)" % (int(data[0], 16)-0x8000, (int(data[0], 16)-0x8000-3)/(2+self.n_sample)), "uB: ", data[1], data[2])
        else:
            print("words: ", data[0] ,"(%i, ev: %i)" % (int(data[0], 16), (int(data[0], 16)-3)/(2+self.n_sample)), "uB: ", data[1], data[2])

    def febReset(self):
        self.write("316","20",lc=True)

    def febPwrCycle(self):
        print("Power cycling all FEB ports")
        self.cmd("PWRRST 25")
        if self.verbose:
            print("readback: ", self.ser.readline())

    def febGetSample(self):
        data = self.read("30C", lc=True)
        return data

    def febSetup(self, n_sample=10, port="1"):
        self.n_sample = n_sample
        print("Set external trigger to RJ45")
        self.cmd("LC TRIG 0")
        if self.verbose:
            print("readback: ", self.ser.readline())
        print("Set the port the FEB is connected to: ", port)
        self.write("314",port,lc=True)
        print("Take new pedestral")
        self.write("316","100",lc=True)
        print("Enable self-triggering on spill gate")
        self.write("30E","2", lc=True)
        print("Set spill gate")
        print("    On-spill 0x70 @ 80MHZ (Default is 0x800)")
        self.write("305", "70", lc=True)
        print("    Off-spill 0x800 @ 80Mhz")
        self.write("304","05",lc=True) 
        print("Set pipeline delay to 0x05: trigger between  2nd and 3th sample")
        #self.write("306", "800", lc=True)
        self.write("306", "100", lc=True) # USED in Vertical Slice Test, 400 works, 600 doesn't
        print("Number of ADC samples: ", n_sample)
        self.write("30C", hex(n_sample)[2:], lc=True)
        print("Reset DDR write/read pointers")
        self.write("310","0",lc=True)
        self.write("311","0",lc=True)

    def febSetGate(self, a="fff"):
        self.write("306",a, lc=True)

    def rocDdrReset(self):
        print("Reset DDR read/write addresses")
        self.write("402","0")
        self.write("403","0")
        self.write("404","0")
        self.write("405","0")

    def rocMarkerBits(self):
        print(bin(int(self.read("77")[0],16)))

    def rocSetup(self, tdaq=True, tdaq_timing=True, uB_offset="a"):
        print("Test Fiber Connection")
        self.write("0", "8") # reset it
        print(self.read("1"))
        print("Enable package forwarding to the FEB")
        if tdaq_timing:
            print("Enable Marker Sync, timing from TDAQ")
            self.write("0", hex(2**5)[2:])
            #self.write("0", hex(2**5+2**4)[2:])
            print("Enable Onboard PLL for external clock")
            self.write("19","0") # power down PLL SIMON TESTED CRUCIAL! 0 -> PLL enabled
        else:
            self.write("0", "0")
            self.write("19","1")
        print("set CSR")
        self.write("400", "A8")
        print("Reset GTP FIFO")
        self.write("2","1")
        print("Reset link reciver FIFO")
        self.write("27", "300")
        self.rocDdrReset()
        if tdaq:
            print("Set TRIG 1")
            self.cmd("TRIG 1")
            if self.verbose:
                print("readback: ", self.ser.readline())
                print("readback: ", self.ser.readline())
        print("Add uB offset of %i" % int(uB_offset,16))
        self.write("81", uB_offset)

    def rocSetPLLMon(self):
        self.write("17","12")
        self.write("18","12") 
        # default is 42 which means r-divider on the output
        # 0x17 0x12, 0x18 42: r divider
        # 0x17 0x12, 0x18 12: digital lock
        # 0x17 0x12, 0x18 32: outut on vdd to test functionality

    def febPedestal(self, fpga=0):
        print("Take pedestals...")
        self.write("316","100",lc=True)
        self.ser.flushInput()
        data = self.read(self.febFPGA[fpga]+"80", "10", lc=True)
        print(data)
        return data

    def febGetThr(self, fpga=0):
        data = self.read(self.febFPGA[fpga]+"90", "10", lc=True)
        print(data)

    def febSetThr(self, ch, th, fpga=0):
        if ch == -1:
            for ch_ in range(16):
                self.febSetThr(ch_, th, fpga=fpga)
        add = "%x" % (0x90 + ch + int(self.febFPGA[fpga]+"00",16))
        print("Set threshold of fpga %i, channel %i (add %s) to %s" % (fpga, ch, add, th) )
        self.write(add, th, lc=True)
        self.febGetThr(fpga=fpga)
        #data = self.read(self.febFPGA[fpga]+"90", "10", lc=True)
        #print(data)

    def rocUBunchCnt(self):
        return self.read("36", 3)

    def febAFEHighGain(self, gain=31, afe=1, fpga=0):
        afe_ = hex(afe + int(self.febFPGA[fpga], 16))[2:]
        #print("Increase PGA gain from 24dB to 30dB")
        #self.write("133", hex(2**13)[2:], lc=True)
        #self.write("133", "2004", lc=True) # default 2004 30dB, 20Mhz
        
        #print("Increase LNA gain from 12dB to 24dB")
        #self.write("134", "2140", lc=True)
        #self.write("134", "4140", lc=True) # default 4140, active terminatin, 100Ohm, 12dB  
        if gain == 0:
            print("Disable digital gain, set LNA gain to nominal 12dB")
            self.write(afe_+"34", "4140", lc=True)
            self.write(afe_+"03", "0", lc=True)
        else:
            print("Enable digital gain, set to %.1fdB" % (gain*0.2))
            self.write(afe_+"03", hex(2**12)[2:],lc=True)
            print("Increase LNA gain from 12dB to 24dB")
            self.write(afe_+"34", "2140", lc=True)
        val = hex(gain * 2**11)[2:]
        self.write(afe_+"0D", val, lc=True) # ch1
        self.write(afe_+"0F", val, lc=True) # ch2
        self.write(afe_+"11", val, lc=True) # ch3
        self.write(afe_+"13", val, lc=True) # ch4
        self.write(afe_+"19", val, lc=True) # ch8
        self.write(afe_+"1b", val, lc=True) # ch7
        self.write(afe_+"1d", val,lc=True) # ch6
        self.write(afe_+"1f", val, lc=True) # ch5

    def rocUBCnt(self):
        data = self.read("36", n=3)
        print(data)

    def resetLink(self):
        self.write("0", "8")
        print(self.read("1"))

    def dcs(self):
        while True:
            if int(self.read("51")[0])>0:
                #return self.cmd("RD2 1")
                adr = self.readDCS()[3]
                print("Return ", adr)
                self.sendDCS(adr, "cafe")
                break

    def readDCS(self):
        n = int(self.read("51")[0])
        if n < 9:
            print("Only %i words are in the buffer (0x50)." % n)
            return []
        else:
            return self.readm("50",9)
       
    def sendDCS(self, add, data):
        # header
        dd = ["0010", "8040", "0008", add, data, "0000", "0000", "0000"]
        self.write("1a", "4") # header K28.0 D4.Y
        self.write("1c", "0010") # package size, not used
        self.write("1c", "8040") # valdi, ROC no, type, hops
        self.write("1c", "0008") # block seq (not used), status, option 0 := Read(s), 1:= Write(s), 2:= Block Read, 3:= Block Write
        self.write("1c", add)    # op 0: address
        self.write("1c", data)   # op 0: data   
        self.write("1c", "0000") # reserved
        self.write("1c", "0000") # reserved
        self.write("1c", "0000") # reserved
        self.write("1e", "0001") # generate CRC and send
        print(dd)

        self.write("1b", "4") # header K28.0 D4.Y
        self.write("1d", "0010") # package size, not used
        self.write("1d", "8040") # valdi, ROC no, type, hops
        self.write("1d", "0000") # block seq (not used), status, option 0 := Read(s), 1:= Write(s), 2:= Block Read, 3:= Block Write
        self.write("1d", add)    # op 0: address
        self.write("1d", data)   # op 0: data   
        self.write("1d", "0000") # reserved
        self.write("1d", "0000") # reserved
        self.write("1d", "0000") # reserved
        self.write("1f", "0001") # generate CRC and send

    def LP(self, port="1"):
        cmd = "LP "+port
        self.cmd(cmd)
        if self.verbose:
            print("readback: ", self.ser.readline())
            print("readback: ", self.ser.readline())

    def setupData(self, n_sample=8,th="C"):
        self.rocSetup(tdaq=True, tdaq_timing=True, uB_offset="a")
        self.rocDdrReset() # I don't think this is really needed?
        for np, port in enumerate(["1"]):
            self.LP(port)
            self.febSetup(n_sample=n_sample, port=port)
            for fpga in range(4):
                self.febSetThr(-1, th, fpga=fpga)

            #for fpga in range(nfpga):
            #    for afe in range(1, nafe):
            #       print("DEBUG", afe, fpga)
            #       self.febAFEHighGain(gains[np],afe=afe,fpga=fpga)
        self.write("401","0") # switch off the active numbers

    def febSetAFEDAC(self, val="384"):
        for fpga in range(4):
            add1 = hex(fpga << 2)[2:]+"46"
            add2 = hex(fpga << 2)[2:]+"47"
            self.write(add1, val, lc=True)
            self.write(add2, val, lc=True)

    def rocReset(self):
        self.rocSetup(tdaq=True, tdaq_timing=True, uB_offset="a")
        self.rocDdrReset() 

    def setupGr(self, gr="1"):
        self.rocSetup(tdaq=True, tdaq_timing=True, uB_offset="0")
        self.rocDdrReset()
        self.write("401","0")
        self.write("58", gr)
        self.write("45","1")
        #self.write("45","1") # enable the 80MHz clock alignment
        self.write("0a","0") # set the ROC ID, needs to match DTC channel
        self.write("4","2")  # switch off event builder from data

    def setup(self, n_sample=8, tdaq=True, tdaq_timing=True, gr=True, gains=[0], nfpga=1, nafe=2, ports=["1"], uB_offset="0", highGain=False):
        self.rocSetup(tdaq=tdaq, tdaq_timing=tdaq_timing, uB_offset=uB_offset)
        self.rocDdrReset()
        for np, port in enumerate(ports):
            self.LP(port)
            self.febSetup(n_sample=n_sample, port=port)
            if highGain:
                for fpga in range(nfpga):
                    for afe in range(1, nafe):
                        print("DEBUG", afe, fpga)
                        self.febAFEHighGain(gains[np],afe=afe,fpga=fpga)
        self.write("401","0") # switch off the active numbers
        if gr:
            self.write("58","1")

    def markerBits(self):
        data = self.read("77", lc=False)
        return "{0:b}".format(int(data[0],16))

    def single(self):
        self.rocDdrReset()
        time.sleep(0.1)
        self.getTriggers()
        return self.rocDdrReadEv()

    def rocHeartbeat(self):
        data = self.read("3D")
        return data

    def rocTriggerReq(self):
        data = self.read("3C")
        return data

    def setPllMon(self,m="000"):
        self.write("17", "0")
        self.write("18", "0050") # div by 20 addr 0
        self.write("18", "5001") # div by 80 addr 
       
        self.write("17","12")
        self.write("18","42")
        # 000: THREE-STATE OUTPUT
        # 001: DIGITAL LOCK DETECT
        # 010: N DIVIDER OUTPUT
        # 011: AVDD
        # 100: R DIVIDER OUTPUT
        # 101: N-CHANNEL OPEN-DRAIN, LOCK DETECT
        # 110: SERIAL DATA OUTPUT
        # 111: DGND

        bits = "00"+"0"+"100"+"100"+"0000"+"0"+"0"+"0"+"0"+m+"00"+"10"
        bits_ = bits

        print("bit pattern:", bits_,)
        reg = "%06x" % int(bits_,2)
        print("0x",reg)
        self.write("17", "00"+reg[:2])
        self.write("18", reg[2:])


    def febSetBias(self,a="aac", idx=-1):
        addresses = ["44","45","444","445","844","845","c44","c45"]
        if idx == -1:
            for ad in addresses:
               self.write(ad, a, lc=True)
        else:
            self.write(addresses[idx-1], a, lc=True)
        #self.write("45", a, lc=True)

    def febBias(self):
       self.cmd("LC ADC")
       raw = self.ser.read(self.ser.in_waiting)
       return raw

    def lastUBs(self):
        rocSent  = self.read("70")
        febRec   = self.read("65", lc=True)
        febBuff  = self.read("67", lc=True)
        print("rocSent:", rocSent)
        print("febRec:", febRec)
        print("febBuff:", febBuff)

    def constructMask(self, channel): 
        return 0xFFFF & ~(1<<channel) # Clear the bit


    def febChannels(self, mask=0xFFFF, FPGA=-1):
        # Enable FEB stuff
        self.cmd("LC ADC")
        # Input mask register
        input_mask_reg = 0x21

        # Reset (make sure old settings don't hang around)
        def _reset():
            print("\n---> Resetting FEB FPGA channels")
            for reg_digit in self.febFPGA.values():
                reg = int(reg_digit, 16) << 8
                self.write(
                    format(reg | input_mask_reg, 'X'),
                    format(0xFFFF, "X"),
                    lc=True
                )        
        _reset()

        # Set channels
        print("\n---> Setting FEB FPGA channels")
        if FPGA < 0: # Do them all
            for reg_digit in self.febFPGA.values():
                # Convert hex string to int and shift left by 8 bits
                reg = int(reg_digit, 16) << 8
                # Write to input mask register
                self.write(
                    format(reg | input_mask_reg, 'X'),
                    format(mask, "X"),
                    lc=True
                )
        else: # Do the specified FPGA
            reg_digit = self.febFPGA[FPGA] 
            reg = int(reg_digit, 16) << 8
            self.write(
                    format(reg | input_mask_reg, 'X'),
                    format(mask, "X"),
                    lc=True
                )