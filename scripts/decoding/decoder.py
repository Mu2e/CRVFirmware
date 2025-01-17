import struct
import numpy as np
import ctypes
from tqdm import tqdm
import os
import pandas as pd

PATH = "/scratch/mu2e/mu2ecrv_crv_scorrodi_v3_03_00/OutputData/"

class subevent:
    """
    Decode a subevent.
    
    Attributes:
        byte_count (int): Total number of bytes in the subevent, should be 48
        EWT (int): Event Window Tag 
        nROCs (int): Number of ROCs (always 6, why?)
        evMode (int): Event mode??
        link0-link5 (int): Status bytes for ROC links (1-5 unused)
    """
    def __init__(self, data):
        """Initialise"""
        # I: unsigned int (4 bytes/32 bits)
        # Unpack 12 32-bit words 
        words = struct.unpack('<IIIIIIIIIIII', data)
        self.byte_count = words[0] # Total bytes in subevent
        self.EWT = words[1] + ((words[2]&0xffff) << 32) # ???
        self.nROCs = (words[2]&0xff0000) >> 16 # bits 16-23 of word[2]
        self.evMode = (words[3] << 8) + ((words[2]&0xff000000) >> 24) # ???
        #words[4]: DTC MAC, EVB Mode, Data Source
        #words[5]: Reserved
        self.link0 = (words[6]&0xff)             # bits 7-0 (word[6])
        self.link1 = (words[6]&0xff00) >> 8      # bits 15-8 ""
        self.link2 = (words[6]&0xff0000) >> 16   # bits 23-16 ""
        self.link3 = (words[6]&0xff000000) >> 24 # bits 31-24 ""
        self.link4 = (words[7]&0xff)             # bits 7-0 (word[7])
        self.link5 = (words[7]&0xff00) >> 8      # bits 15-8 ""
    def print(self):
        """Print statuses"""
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
    """
    Decode a header.
    
    Attributes:
        valid (bool): Header validation flag
        dtc_errors (int): DTC error flags
        roc_id (int): ROC ID
        subsystem (int): Subsystem ID
        n (int): Number of packets
        EWT (int): Event window tag
        status (int): Status byte 
    """
    def __init__(self, data):
        """Initialise"""
        # H : unsigned short integer (2 bytes/16 bits)
        # Unpack 8 16 bit words 
        words = struct.unpack('<HHHHHHHH', data)
        if words[1]&0xff != 0x50:
            raise Exception("Expected a data header, but got 0x%04x abort!" % words[1])
        # word[1]:
        self.valid      = (words[1]&0x8000) >> 15 # bit 15
        self.dtc_errors = (words[1]&0x7800) >> 11 # bits 14-11
        self.roc_id     = (words[1]&0x0700) >> 8  # bits 10-8
        # word[2]:
        self.subsystem  = (words[2]&0xe000) >> 13 # bits 15-13
        self.n          = words[2]&0x7ff          # bits 10-0 
        # Construct 48-bit EWT from three 16-bit words
        # word[3] bits 15-0 , word[4] bits 31-16, word[5] bits 47-32
        self.EWT        = words[3] + (words[4] << 16) + (words[5] << 32)
        # Extract status from lower byte of word[6]
        self.status     = words[6]&0xff           # bits 7-0
    def print(self):
        """
        Print header info
        """
        print("Header %0.4x %0.4x %0.4x,% 4i pack., stat.: %02x" % (
              (self.EWT>>16)&0xFFFF, (self.EWT>>8)&0xFFFF, (self.EWT>>0)&0xFFFF,
               self.n, self.status), end="   ")
        if self.status&0x8 > 0:
            print("roc-timeout", end=" ")
        if self.status&0x4 > 0:
            print("corrupt", end=" ")
        print(" ")

class status:
    """
    Decode status info
    
    Attributes:
        id (int): Status ID (4 bits)
        byte (int): Byte count
        active (int): Active channels status (32 bits)
        dr_cnt (int): Data readout count??
        EWT (int): Event Window Tag (48 bits) 
    """
    def __init__(self, data):
        # H: unsigned short integer (2 bytes/16 bits each)
        # Unpack 8 16-bit words
        words = struct.unpack('<HHHHHHHH', data)
        # Check for status marker 0x0060 in bits 15-4 of word[0]
        if words[0]&0xfff0 != 0x0060:
            raise Exception("Expected 006X, but got 0x%04x abort!" % words[0])
        # Extract fields:
        self.id     = words[0]&0xf      # bits 3-0 of word[0]
        self.byte   = words[1]          # full word[1]: byte count
        # Construct 32-bit active channels status from two 16-bit words
        # bits 15-0  (word2), bits 31-16 (word3)
        self.active = words[2] + (words[3]<<16)     
        self.dr_cnt = words[4]          # full word[4]: data readout counter
        # Construct 48-bit EWT from three 16-bit words
        # bits 15-0 (word5), bits 31-16 (word6), bits 47-32 (word7)
        self.EWT    = words[5] + (words[6] << 16) + (words[7] << 32) 

    def print(self):
        """
        Print status info:
        - EWT (Event Window Tag) in three 16-bit segments
        - Byte count
        - Data readout count??
        """
        print("Cntrl   %0.4x %0.4x %0.4x,% 4i bytes., dr_cnt: % 6i" % (
              (self.EWT>>16)&0xFFFF,   # Middle 16 bits of EWT
              (self.EWT>>8)&0xFFFF,    # Next 16 bits
              (self.EWT>>0)&0xFFFF,    # Lowest 16 bits
              self.byte,               # Byte count
              self.dr_cnt),            # Data readout count ?? 
              end="   ")
        print(" ")

class gr:
   """ 
   Decode Global Run info
   
   Attributes:
       n_ewt (int): Number of event window tags
       n_marker (int): Number of event window markers
       LastWindow (int): Last EWT
       CRC (int): (cyclic reduncancy check??) check value (8 bits)
       PLL (int): Phase-Locked Loop status (4 bits)
       Lock (int): Lock status bit
       InjectionTs (int): Injection timestamp
       InjectionWindow (int): Injection window number
   """
   def __init__(self, data):
       """Initialise"""
       # H: unsigned short integer (2 bytes/16 bits each)
       # Unpack 8 16-bit words
       words = struct.unpack('<HHHHHHHH', data)
       # Check for injected magic numbers at start and end
       if (words[0] != 0xcafe) or (words[7] != 0xbeef):
           raise Exception("Expected cafe and beef, but got %04x and %04x abort!" % (words[0], words[7]))
       # Extract fields:
       self.n_ewt = words[1]              # full word[1]: EWT count
       self.n_marker = words[2]           # full word[2]: marker count
       self.LastWindow = words[3]         # full word[3]: last window number
       # Extract status bits from word[4]:
       self.CRC = (words[4]>>8)&0xff      # bits 15-8: CRC check
       self.PLL = (words[4]>>4)&0xf       # bits 7-4: PLL status
       self.Lock = words[4]&0x1           # bit 0: Lock status
       # Injection timing information:
       self.InjectionTs = words[5]        # full word[5]: injection timestamp
       self.InjectionWindow = words[6]    # full word[6]: injection window
       
   def print(self):
       """
       Print Global Run info:
       - EWT and marker counts
       - CRC, Lock status, and PLL status
       - Window and timing information
       """
       print("ewt/mrk: %i/%i, crc %i, loss %i, pll %i, %04x %04x %04x" % (
            self.n_ewt, self.n_marker,    # EWT and marker counts
            self.CRC, self.Lock,          # CRC and lock status
            (1 - self.PLL),               # Inverted PLL status
            self.LastWindow,              # Last window number
            self.InjectionWindow,         # Injection window
            self.InjectionTs),            # Injection timestamp
            end="")
       print(" ")

class hit:
    """
    Decode a single hit with timing and waveform information.
    
    Attributes:
        channel (int): Channel number
        time (int): Hit timestamp
        samples (list): List of digitized waveform samples
    """
    def __init__(self, data):
        """Initiliase"""
        n_words = len(data) // 2 # Why // 2 ?? 
        format_string = '<' + 'H' * n_words  # Construct format string of 16 bit words
        words = struct.unpack(format_string, data) # Unpack it
        self.channel = words[0]  # full word[1]: channel number
        self.time    = words[1]  # full word[2]: timestamp
        self.samples = words[2:] # full word[3 and up]: samples
    def print(self):
        """Print hit info"""
        print("ch-%04x at %04x" % (
            self.channel,
            self.time
        ), end=": ")
        for h in self.samples:
            print("%04x" % (h), end=" ")
        print("")
        
class reader:
    """
    Read file and configure setup
    
    Arguments:
        fname (str): Input file name
        path (str): File path
        n_samples (int): Number of samples per hit
        
    Attributes:
        isGr (bool): global run flag
        raw (bool): raw data flag 
    """
    def __init__(self, fname, path=PATH, n_samples=8):
        """Initialise"""
        self.fname = path+fname # full file path
        self.file = open(self.fname, 'rb') # read binary file
        self.pbar = None # Init progress bar (None, otherwise it starts when you init the class)
        self.isGr = False # GR flag
        self.n_samples = n_samples # number of samples / hit
        self.raw = False # raw data flag

    def _read(self, b=16):
        """
        Internal function to update progress bar when reading 
        Returns:
            Execute read of b bytes from file
        """
        if self.pbar is None: # Init progress bar
            self.pbar = tqdm(total=os.path.getsize(self.fname), unit='B', unit_scale=True, desc=self.fname) 
        self.pbar.update(b) # Update status bar
        return self.file.read(b) # Read
        
    def getSubevent(self):
        """
        Read the next subevent from binary file
        Reads 48 bytes because a subevent is:
        - 12 words × 4 bytes per word = 48 bytes 
        Returns:
           subevent: Decoded subevent object or None  
        """
        data = self._read(48) # Read 48 bytes (one subevent) 
        if not data: # Return None if EOF or empty 
            return None
        return subevent(data) # Return decoded subevent object 

    def getHeader(self):
        """
        Read the next subevent header from binary file
        Reads 16 bytes because a header is:
        - 8 words x 2 bytes per word = 16 bytes 
        Returns:
           subevent: Decoded header object or None 
        """
        data = self._read(16) # Read 16 bytes (one header) 
        if not data: # Return None if EOF or empty 
            return None
        return header(data) # Return decoded header object 

    def getStatus(self):
        """
        Read the next subevent ROC ?? status (from header) from binary file
        Reads 16 bytes because a header is:
        - 8 words x 2 bytes per word = 16 bytes 
        Returns:
            subevent: Decoded status object or None 
        """
        data = self._read(16) # Read 16 bytes (one header)
        if not data: # Return None if EOF or empty 
            return None
        return status(data) # Return decoded status object 

    def getGR(self):
        """
        Read the next subevent GR info (from header) from binary file
        Reads 16 bytes because a header is:
        - 8 words x 2 bytes per word = 16 bytes 
        Returns:
            subevent: Decoded status object or None 
        """
        data = self._read(16) # Read 16 bytes (one header)
        if not data: # Return None if EOF or empty 
            return None 
        return gr(data) # Return decoded GR status object 

    def getHit(self):
        """
        Read the next hit from binary file
        Reads 20 bytes because: ?? 
        - of unknown reasons 
        Returns:
            subevent: Decoded hit object or None 
        """
        data = self._read((2+self.n_samples)*2) # Read 2*(2+n_samples) bytes 
        if not data: # Return None if EOF or empty 
            return None
        return hit(data) # Return decoded hit object 
        
    def get(self, verbose=0, writer=None):
        """
        Main function to read and process all data from file
        Args:
            verbose (int): Detail level for printing (0-3)
            writer: Optional callback to process hit data
        """
        i = 0 # Subevent counter
        while True: # Iterate until break (EOF)
            subevent = self.getSubevent() # get subevent
            i += 1 # Increase counter
            if subevent is None: # EOF
                break # Exit
            if verbose > 0: # Verbose printout > 0
                print("% 3i)" % i, end=" ") # Print subevent counter as 3 digit int
                subevent.print() # Print decoded subevent info
            for rocn in range(6): # Loop thro' 6 ROCs (why always 6??)
                header = self.getHeader() # Get decoded header
                if verbose > 1: # Verbose printout > 1
                    if rocn in [0]: # Just ROC0 
                        print("     ROC%i" % rocn, end="-") # Print ROC number
                        header.print() # Print ROC header
                if self.isGr: # Check global run flag
                    for n in range(header.n): # Loop thro' headers 
                        if n in [0]: # ROC 0 only ??
                            status = self.getStatus() # Get ROC status
                            if verbose > 1: # Verbose printout > 1
                                print("     CRV-" , end="")
                                status.print() # Print ROC status
                        elif n in [1]: # ROC 1 ?? 
                            gr = self.getGR() # Same as previous "if"
                            if verbose > 1:
                                print("     " , end="")
                                gr.print()
                        else: # ROCs above 1
                            data = self.file.read(16) # Get subevent header
                            words = struct.unpack('<HHHHHHHH', data) # unpack 8 16 bit words
                            print("        ", end=" ")
                            for k in range(8): # Loop thro' words 
                                print("%04x" % words[k], end=' ') # Print word in 4 digit hex
                            print(" ")
                elif self.raw: # Handle raw data
                    for n in range(header.n): # Loop thro' raw data
                        data = self.file.read(16) # Read subevent header
                        words = struct.unpack('<HHHHHHHH', data) # unpack 8 16 bit words
                        print("        ", end=" ")
                        for k in range(8): # Loop thro' words 
                            print("%04x" % words[k], end=' ') # Print word in 4 digit hex
                        print(" ")
                        
                else:  # Normal hit processing mode (not raw, not GR)
                    if header.n > 0: # Check header > 0
                        status = self.getStatus() # Get decoded status object
                        if verbose > 1: # Verbose printout > 1
                            print("     CRV-" , end="") # Prepend CRV-
                            status.print() # Print status
                        # Calculate number of hits in subevent
                        # status.byte contains total bytes
                        # Subtract 8 bytes of header
                        # Divide by (2 + n_samples) which is size of each hit
                        n_hits =  (status.byte-8) // ((2+self.n_samples))
                        #print("n_hits",n_hits)
                        for hit_idx in range(n_hits): # Itererate through hits
                            hit = self.getHit() # get hit
                            if verbose > 2: # verbose > 2 
                                print("        ", end=" ") # Print hit
                                hit.print() 
                            if writer: # Check write flag 
                                writer(header.EWT, hit) # Write header
                        padding = (header.n-1)*8 - n_hits * (2+self.n_samples) # Add padding 
                        #print("padding",padding)
                        self.file.read(padding*2)
                        
        if self.pbar is not None:
            self.pbar.close() # Close progress bar 
                            
    def __enter__(self):
        """
        Enter method for context manager ('with' statement)
        Allows the class to be used with 'with' statements
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Exit method for context manager
        Ensures file is closed when leaving 'with' block
        even if an error occurred
        """
        self.file.close()

    def __delete__(self):
        """
        Destructor method
        Ensures file is closed when object is deleted
        """
        self.file.close() # Close file

class EWTWriter:
    """
    Collects and stores EWT info.
    Accumulates headers and can convert to pandas DataFrame.
    
    Attributes:
    idx (int): Current number of stored hits
    max_hits (int): Maximum hits (auto-expands if needed)
    ewts (ndarray): Event Window Tags
    n_packages (ndarray): Number of packages per subevent
    status (ndarray): Status flags
    """
    def __init__(self):
        """Initialise"""
        self.idx = 0
        self.max_hits = 100  # Initial capacity
        
        # Initialize arrays for storing data
        self.ewts = np.zeros([self.max_hits], dtype=np.uint64)        # 64-bit for EWT
        self.n_packages = np.zeros([self.max_hits], dtype=np.uint16)  # 16-bit for package count
        self.status = np.zeros([self.max_hits], dtype=np.uint16)      # 16-bit for status flags

    def _reshape(self):
        """
        Double the size of all arrays when capacity is reached
        """
        self.max_hits *= 2  # Double capacity
        self.ewts.resize([self.max_hits])
        self.n_packages.resize([self.max_hits])
        self.status.resize([self.max_hits])
   
    def __call__(self, header, status=None):
        """
        Add a new header's information to storage
        __call__ allows class instance to be used like a function
        
        Args:
           header: Header object containing EWT, package count, and status
           status: Optional status information (unused)
        """
        # Expand arrays if needed
        if self.idx == self.max_hits:
            self._reshape()
           
        # Store header information
        self.ewts[self.idx] = header.EWT
        self.n_packages[self.idx] = header.n
        self.status[self.idx] = header.status
        self.idx += 1

    def get(self):
        """
        Convert stored data to DataFrame
        
        Returns:
        pandas.DataFrame: Contains EWT, package count, and status for each event
        """        
        # Concatenate arrays and transpose:
        # 1. Slice each array to include only valid entries [:self.idx]
        # 2. Add dimension with [None,:] for concatenation
        # 3. Stack vertically, then transpose for DataFrame format
        return pd.DataFrame(
            np.concatenate([
                self.ewts[:self.idx][None,:],        # Event Window Tags
                self.n_packages[:self.idx][None,:],  # Package counts
                self.status[:self.idx][None,:]       # Status flags
            ], dtype='int').T,
            columns=["EWT", "packages", "status"]
        )
       
class HitWriter:
    """
    Collects and stores waveform hit data.
    Accumulates hits with their waveforms and can convert to DataFrame.
    
    Args:
    n_samples (int): Number of samples per waveform
    
    Attributes:
    max_hits (int): Maximum hits (auto-expands if needed)
    n_samples (int): Samples per waveform
    ewts (ndarray): Event Window Tags, uint64
    channels (ndarray): Channel numbers, uint32
    time (ndarray): Hit timestamps, uint16
    wf (ndarray): Waveform samples, int16 [hits, samples]
    idx (int): Current number of stored hits
    """
    def __init__(self, n_samples):       
        """Initiliase"""
        self.max_hits = 100  # Initial capacity
        self.n_samples = n_samples 
        
        # Initialise arrays for storing hit data
        self.ewts = np.zeros([self.max_hits], dtype=np.uint64)      # 64-bit for EWT
        self.channels = np.zeros([self.max_hits], dtype=np.uint32)  # 32-bit for channel
        self.time = np.zeros([self.max_hits], dtype=np.uint16)      # 16-bit for time
        self.wf = np.zeros([self.max_hits, self.n_samples], dtype=np.int16)  # 16-bit signed for waveforms
        self.idx = 0

    def _reshape(self):
        """
        Internal function to double the size of all arrays when capacity is reached
        """
        self.max_hits *= 2  # Double capacity
        self.ewts.resize([self.max_hits])
        self.channels.resize([self.max_hits])
        self.time.resize([self.max_hits])
        self.wf.resize([self.max_hits, self.wf.shape[1]])
       
    def __call__(self, ewt, hit):
        """
        Add a new hit's data to storage
        
        Args:
           ewt: Event Window Tag (timestamp)
           hit: Hit object containing channel, time, and waveform samples
        """
        # Expand arrays if needed
        if self.idx == self.max_hits:
            self._reshape()
           
        # Store hit information
        self.ewts[self.idx] = ewt
        self.channels[self.idx] = hit.channel  # Includes controller, port, GA, channel info
        self.time[self.idx] = hit.time & 0xfff  # Mask to 12 bits
        
        # Convert waveform samples from offset binary to signed int
        # 1. Mask to 12 bits (&0xFFF)
        # 2. XOR with 0x800 and subtract 0x800 to convert from offset binary
        self.wf[self.idx,:] = ((np.array(hit.samples) & 0xFFF) ^ 0x800) - 0x800
        
        self.idx += 1

    def test(self):
        """
        Return shape of concatenated data arrays
        Used for testing/debugging array dimensions
        
        Returns:
           tuple: Shape of concatenated array
        """
        return np.concatenate([
            self.ewts[:self.idx][None,:],     # Add dimension for concatenation
            self.channels[:self.idx][None,:],
            self.time[:self.idx][None,:],
            self.wf[:self.idx,:].T            # Transpose waveforms for concatenation
        ]).shape
       
    def get(self):
        """
        Convert stored data to pandas DataFrame
        
        Returns:
           pandas.DataFrame: Contains EWT, channel, time, and waveform samples
                                Waveform samples are in columns s0, s1, etc.
        """
        return pd.DataFrame(
            # Concatenate arrays vertically, then transpose
            np.concatenate([
                self.ewts[:self.idx][None,:],          # Event Window Tags
                self.channels[:self.idx][None,:],      # Channel numbers
                self.time[:self.idx][None,:] & 0xfff,  # Time (12 bits)
                self.wf[:self.idx,:].T                 # Waveform samples
            ], dtype='int').T,
            # Column names: EWT, channel, time, s0, s1, ...
            columns=["EWT", "channel", "time"] + [f"s{s}" for s in range(self.n_samples)]
        )