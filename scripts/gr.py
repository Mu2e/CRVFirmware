import struct
path = "/scratch1/mu2e/mu2ecrv_crv__v3/OutputData/"

def get(fname, verbose=0):
    with open(fname, 'rb') as f:
        event = []
        CRCErrCnt = []
        LosCounter = []
        PLLStat = []
        uB = []
        BeamOn = []
        LastWindow = []
        MarkerCnt = []
        HeartBeatCnt = []
        InjectionTs = []
        while True:
            data = f.read(16)
            if not data:
                break
            words = struct.unpack('<HHHHHHHH', data)
        
            if False:
                for k in range(8):
                    print("%04x" % words[k], end= ' ')
                print("")
        
            # find headers
            if words[0] == 0x0020 and words[1] == 0x8050:
                n   = words[2]
                EVT = words[3] + (words[4] << 16) + (words[5] << 32) +  (words[6] << 48)
                event.append(EVT)
                #ns.append(n)
                if verbose>0:
                    print("Event 0x%08x (n=%i)" % (EVT, n))
            
                # read data
                data = f.read(16)
                if verbose>1:
                    words = struct.unpack('<HHHHHHHH', data) # in words
                    for k in range(8):
                       print("%04x" % words[k], end=' ')
                    print("")
                unpacked         = struct.unpack('<BBHHBBHHHH', data)
                CRCErrCnt.append(       unpacked[1])
                LosCounter.append(      unpacked[0] >> 4)
                PLLStat.append(         unpacked[0] & 0x1)
                uB.append(              unpacked[2] + (unpacked[3] << 16) )
                BeamOn.append(          (unpacked[4] >> 4) & 0x1)
                LastWindow.append(      unpacked[6])
                MarkerCnt.append(       unpacked[7])
                HeartBeatCnt.append(    unpacked[8])
                InjectionTs.append(     unpacked[9])
            
    import pandas as pd
    return pd.DataFrame({
        'event': event,
        'CRCErrCnt': CRCErrCnt,
        'LosCounter': LosCounter, 
        'PLLStat': PLLStat,
        'uB': uB,
        'BeamOn': BeamOn,
        'LastWindow': LastWindow,
        'MarkerCnt': MarkerCnt,
        'HeartBeatCnt': HeartBeatCnt,
        'InjectionTs': InjectionTs
    }).set_index("event")
