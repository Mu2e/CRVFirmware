import struct
import numpy as np
import ctypes
PATH = "/scratch/mu2e/mu2ecrv_crv_scorrodi_v3_03_00/OutputData/"

class subevent:
    def __init__(self, data):
        words = struct.unpack('<IIIIIIIIIIII', data)
        self.byte_count = words[0]
        self.EWT = words[1] + ((words[2]&0xffff) << 32)
        self.nROCs = (words[2]&0xff0000) >> 16 # is always 6
        self.evMode = (words[3] << 8) + ((words[2]&0xff000000) >> 24)
        #words[4]: DTC MAC, EVB Mode, Data Source
        #words[5]: Reserved
        self.link0 = (words[6]&0xff) # ROC0
        self.link1 = (words[6]&0xff00) >> 8
        self.link2 = (words[6]&0xff0000) >> 16
        self.link3 = (words[6]&0xff000000) >> 24
        self.link4 = (words[7]&0xff)
        self.link5 = (words[7]&0xff00) >> 8
    def print(self):
        print("Subevent    %0.4x %0.4x %0.4x,% 4i bytes, stat0: %02x" % (
              (self.EWT>>16)&0xFFFF, (self.EWT>>8)&0xFFFF, (self.EWT>>0)&0xFFFF,
               self.byte_count, self.link0), end="   ")
        if self.link0&0x1 > 0:
            print("timeout", end=" ")
        if self.link0&0x8 > 0:
            print("CRC", end=" ")
        if self.link0&0x20 > 0:
            print("ROC", end=" ")
        if self.link0&0x40 > 0:
            print("DTC", end=" ")
        if self.link0&0x80 > 0:
            print("any", end=" ")
        print(" ")

class header:
    def __init__(self, data):
        words = struct.unpack('<HHHHHHHH', data)
        if words[1]&0xff != 0x50:
            raise Exception("Expected a data header, but got 0x%04x abort!" % words[1])
        self.valid      = (words[1]&0x8000) >> 15
        self.dtc_errors = (words[1]&0x7800) >> 11
        self.roc_id     = (words[1]&0x0700) >> 8
        self.subsystem  = (words[2]&0xe000) >> 13
        self.n          = words[2]&0x7ff
        self.EWT        = words[3] + (words[4] << 16) + (words[5] << 32)
        self.status     = words[6]&0xff
    def print(self):
        print("Header %0.4x %0.4x %0.4x,% 4i pack., stat.: %02x" % (
              (self.EWT>>16)&0xFFFF, (self.EWT>>8)&0xFFFF, (self.EWT>>0)&0xFFFF,
               self.n, self.status), end="   ")
        if self.status&0x8 > 0:
            print("roc-timeout", end=" ")
        if self.status&0x4 > 0:
            print("corrupt", end=" ")
        print(" ")

class status:
    def __init__(self, data):
        words = struct.unpack('<HHHHHHHH', data)
        if words[0]&0xfff0 != 0x0060:
            raise Exception("Expected 006X, but got 0x%04x abort!" % words[0])
        self.id     = words[0]&0xf
        self.byte   = words[1]
        self.active = words[2] + (words[3]<<16)
        self.dr_cnt = words[4]
        self.EWT    = words[5] + (words[6] << 16) + (words[7] << 32)
    def print(self):
        print("Cntrl   %0.4x %0.4x %0.4x,% 4i bytes., dr_cnt: % 6i" % (
              (self.EWT>>16)&0xFFFF, (self.EWT>>8)&0xFFFF, (self.EWT>>0)&0xFFFF,
               self.byte, self.dr_cnt), end="   ")
        print(" ")

class gr:
    def __init__(self, data):
        words = struct.unpack('<HHHHHHHH', data)
        if (words[0] != 0xcafe) or (words[7] != 0xbeef):
            raise Exception("Expected cafe and beef, but got %04x and %04x abort!" % (words[0], words[7]))
        self.n_ewt           = words[1]
        self.n_marker        = words[2]
        self.LastWindow      = words[3]
        self.CRC             = (words[4]>>8)&0xff
        self.PLL             = (words[4]>>4)&0xf
        self.Lock            = words[4]&0x1
        self.InjectionTs     = words[5]
        self.InjectionWindow = words[6]
    def print(self):
        print("ewt/mrk: %i/%i, crc %i, loss %i, pll %i, %04x %04x %04x" % (
             self.n_ewt, self.n_marker,\
             self.CRC, self.Lock, (1 - self.PLL),\
              self.LastWindow, self.InjectionWindow, self.InjectionTs), end="")
        print(" ")

class hit:
    def __init__(self, data):
        n_words = len(data) // 2
        format_string = '<' + 'H' * n_words
        words = struct.unpack(format_string, data)
        self.channel = words[0]
        self.time    = words[1]
        self.samples = words[2:]
    def print(self):
        print("ch-%04x at %04x" % (
            self.channel,
            self.time
        ), end=": ")
        for h in self.samples:
            print("%04x" % (h), end=" ")
        print("")
        
class reader:
    def __init__(self, fname, path=PATH, n_samples=8):
        self.fname = path+fname
        self.file = open(self.fname, 'rb')
        self.isGr = False
        self.n_samples = n_samples
        self.raw = False

    def getSubevent(self):
        data = self.file.read(48)
        if not data:
            return None
        return subevent(data)

    def getHeader(self):
        data = self.file.read(16)
        if not data:
            return None
        return header(data)

    def getStatus(self):
        data = self.file.read(16)
        if not data:
            return None
        return status(data)

    def getGR(self):
        data = self.file.read(16)
        if not data:
            return None
        return gr(data)

    def getHit(self):
        data = self.file.read((2+self.n_samples)*2)
        if not data:
            return None
        return hit(data)
        

    def get(self, verbose=0, writer=None, ewt_writer=None, n_max=None):
        i = 0
        while True:
            subevent = self.getSubevent()
            if n_max:
                if i >= n_max:
                    break
            i = i + 1
            if subevent is None:
                break
            if verbose > 0:
                print("% 3i)" % i, end=" ")
                subevent.print()
            for rocn in range(6):
                header = self.getHeader()
                if rocn in [0]:
                    if verbose > 1:
                        print("     ROC%i" % rocn, end="-")
                        header.print()
                    if ewt_writer:
                        ewt_writer(header)
                if self.isGr:
                    for n in range(header.n):
                        if n in [0]:
                            status = self.getStatus()
                            if verbose > 1:
                                print("     CRV-" , end="")
                                status.print()
                        elif n in [1]:
                            gr = self.getGR()
                            if verbose > 1:
                                print("     " , end="")
                                gr.print()
                        else:
                            data = self.file.read(16)
                            words = struct.unpack('<HHHHHHHH', data) # in words
                            print("        ", end=" ")
                            for k in range(8):
                                print("%04x" % words[k], end=' ')
                            print(" ")
                elif self.raw:
                    for n in range(header.n):
                        data = self.file.read(16)
                        words = struct.unpack('<HHHHHHHH', data) # in words
                        print("        ", end=" ")
                        for k in range(8):
                            print("%04x" % words[k], end=' ')
                        print(" ")
                        
                else: 
                    if header.n > 0:
                        status = self.getStatus()
                        if verbose > 1:
                            print("     CRV-" , end="")
                            status.print()
                        n_hits =  (status.byte-8) // ((2+self.n_samples))
                        #print("n_hits",n_hits)
                        for hit_idx in range(n_hits):
                            hit = self.getHit()
                            if verbose > 2:
                                print("        ", end=" ")
                                hit.print()
                            if writer:
                                writer(header.EWT, hit)
                        padding = (header.n-1)*8 - n_hits * (2+self.n_samples)
                        #print("padding",padding)
                        self.file.read(padding*2)
                            
                        

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.file.close()

    def __delete__(self):
        self.file.close()

class EWTWriter:
    def __init__(self):
        self.idx = 0
        self.max_hits = 100
        self.ewts       = np.zeros([self.max_hits], dtype=np.uint64)
        self.n_packages = np.zeros([self.max_hits], dtype=np.uint16)
        self.status     = np.zeros([self.max_hits], dtype=np.uint16)

    def _reshape(self): 
        self.max_hits *= 2
        self.ewts       .resize([self.max_hits])
        self.n_packages .resize([self.max_hits])
        self.status     .resize([self.max_hits])
    
    def __call__(self, header, status=None):
        if self.idx == self.max_hits:
            self._reshape()
        self.ewts[self.idx]        = header.EWT
        self.n_packages[self.idx]  = header.n 
        self.status[self.idx]      = header.status
        self.idx += 1

    def get(self):
        import pandas as pd
        return pd.DataFrame(np.concatenate([
                             self.ewts[:self.idx][None,:],
                             self.n_packages[:self.idx][None,:],
                             self.status[:self.idx][None,:]
                             ], dtype='int').T,
                           columns = ["EWT","packages","status"])

class HitWriter:
    def __init__(self, n_samples):
        self.max_hits = 100
        self.n_samples  = n_samples
        self.ewts       = np.zeros([self.max_hits], dtype=np.uint64)
        self.channels   = np.zeros([self.max_hits], dtype=np.uint32)
        self.time       = np.zeros([self.max_hits], dtype=np.uint16)
        self.nsamples   = np.zeros([self.max_hits], dtype=np.uint8)
        self.wf         = np.zeros([self.max_hits, self.n_samples], dtype=np.int16)
        self.idx = 0

    def _reshape(self): 
        self.max_hits *= 2
        self.ewts       .resize([self.max_hits])
        self.channels   .resize([self.max_hits])
        self.time       .resize([self.max_hits])
        self.nsamples   .resize([self.max_hits])
        self.wf         .resize([self.max_hits, self.wf.shape[1]])
        
    def __call__(self, ewt, hit):
        if self.idx == self.max_hits:
            self._reshape()
        self.ewts[self.idx]     = ewt
        self.channels[self.idx] = hit.channel # controller - port - GA - channel
        self.time[self.idx]     = hit.time & 0xfff
        self.nsamples[self.idx] = hit.time >> 12
        self.wf[self.idx,:]     = ((np.array(hit.samples) & 0xFFF) ^ 0x800) - 0x800
                                    #np.frombuffer(np.array(hit.samples, dtype=np.uint16).tobytes(), dtype=np.int16) 
                                     #[ctypes.c_int16(h).value for h in hit.samples]
        self.idx += 1
        #print(self.idx)

    def test(self):
        return np.concatenate([self.ewts[:self.idx][None,:],
                             self.channels[:self.idx][None,:],
                             self.time[:self.idx][None,:],
                             self.wf[:self.idx,:].T]).shape
        
    def get(self):
        import pandas as pd
        return pd.DataFrame(np.concatenate([self.ewts[:self.idx][None,:],
                             self.channels[:self.idx][None,:],
                             self.time[:self.idx][None,:] & 0xfff,
                             self.nsamples[:self.idx][None,:],
                             self.wf[:self.idx,:].T], dtype='int').T,
                           columns = ["EWT","channel","time","nsamples"]+["s%i" % s for s in range(self.n_samples)])
        
        