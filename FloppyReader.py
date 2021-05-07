from struct import pack, iter_unpack
import time
import os
import tempfile
import numpy as np
import saleae
import serial

# Image settings
HEADS = 2
CYLINDERS = 83
STARTING_CYL = 0
TRACKSKIP = 0

OUTPUT_IMAGE_FILE = "floppy2.scp"

# Flags for the drive and disk used.
flag_48tpi          = 0x00
flag_96tpi          = 0x02
flag_300rpm         = 0x00
flag_360rpm         = 0x04

CONFIG_FLAGS = flag_48tpi + flag_300rpm

# Settings related to the analyzer setup
ANALYZER_PRESET_FILE    = "setup8m.logicsettings"   # Name of preset-file
SAMPLE_RATE             = 8e6                       # Bitrate sampled by the analyzer. 40e6 needs to be an integer-multiple of this.
REVOLUTIONS             = 2                         # Revolutions per track. Sampling-time needs to be long enough to cover these.

#
# Disk-format parameters:
#
man_CBM         = 0x0
man_PC          = 0x3
man_Other       = 0x8

# man_PC
disk_PC360K     = 0x00
disk_PC720K     = 0x01
disk_PC12M      = 0x02
disk_PC144M     = 0x03

# man_Other
disk_360        = 0x00
disk_12M        = 0x01
disk_Rrsvd1     = 0x02
disk_Rsrvd2     = 0x03
disk_720        = 0x04
disk_144M       = 0x05

MANUFACTURE     = man_Other
DISK_TYPE       = disk_360

class SCPWriter:

    def __init__(self, imagetype):
        self.__trackdata = dict()
        self.__trackduration = np.zeros((HEADS*CYLINDERS, REVOLUTIONS))
        self.__tracklen = np.zeros((HEADS*CYLINDERS, REVOLUTIONS))
        self.__imagetype = imagetype

    def fileheader(self):
        # SCP header
        scp_magic = b"SCP"
        scp_vers = 0x22 # version 2.2
        scp_type = self.__imagetype
        scp_nrev = REVOLUTIONS # number of revolutions
        scp_starttrack = 0 # always start at track 0  not at min(self.trackdata.keys())
        scp_endtrack = max(self.__trackdata.keys())
        scp_flags = 1+CONFIG_FLAGS # flux data starts at index
        scp_width = 0 # 16 bits
        scp_heads = [-1, TRACKSKIP+1, 0][HEADS]
        scp_res = (40e6/SAMPLE_RATE) - 1
        scp_checksum = 0 # TODO

        scp_header = pack("<3sBBBBBBBBBL", scp_magic, scp_vers, scp_type, scp_nrev, scp_starttrack, scp_endtrack, scp_flags, scp_width, scp_heads, scp_res, scp_checksum)
        return scp_header

    def trackoffsettable(self):
        # SCP track table
        scp_tracklist = HEADS*CYLINDERS*[0]
        offs = len(self.fileheader()) + HEADS*CYLINDERS*4 # 1 long word for each track
        for k in range(len(scp_tracklist)):
            try:
                scp_tlen = len(self.trackdata(k)) + len(self.trackheader(k)) # will raise KeyError if track does not exist
                scp_tracklist[k] = offs
                offs = offs + scp_tlen 
            except:
                scp_tracklist[k] = 0  # skip track
        scp_trackoffsets = pack("<%dL" % len(scp_tracklist), *scp_tracklist)
        return scp_trackoffsets

    def trackheader(self, num): # will raise KeyError if track does not exist
        # SCP track header
        scp_tmagic = b"TRK"
        scp_trackno = num
        scp_trkhead = pack("<3sB", scp_tmagic, scp_trackno)
        scp_tstart = 4 + 12*REVOLUTIONS # first revolution starts after this header
        for k in range(REVOLUTIONS):
            scp_tduration = round(self.__trackduration[num,k]*SAMPLE_RATE) 
            scp_tlen = self.__tracklen[num,k] # in bitcells, not in bytes!
            scp_trkhead = scp_trkhead + pack("<LLL", int(scp_tduration), int(scp_tlen), scp_tstart)
            scp_tstart = scp_tstart + 2 * int(scp_tlen)  # start of next revolution
        return scp_trkhead

    def trackdata(self, num): # will raise KeyError if track does not exist
        # round to sample-rate precision
        bitcells = np.round(self.__trackdata[num]*SAMPLE_RATE)
        # pack into output data
        bitdata = pack(">%dH" % len(bitcells), *bitcells.astype('int'))
        return bitdata

    def loadtrack(self, num, filename):
        # load and process one track
        # tr = np.recfromcsv(filename)
        
        # faster binary file parser: assumes index in channel 1, read_data in channel 3
        temp = iter_unpack("=qQ", open(filename,"rb").read())
        tr = np.rec.array([(k[0]/SAMPLE_RATE,(k[1]&2) >> 1,(k[1]&8) >> 3) for k in temp], dtype=[('times','f8'),('index','i4'),('read_data','i4')])

        indexpulse = np.where(np.diff(tr.index) == -1)[0]+1 # index transitions from high->low
        if len(indexpulse) != REVOLUTIONS+1:
            print("Not enough index pulses found")

        for k in range(REVOLUTIONS):
            # one revolution
            tr_rev = tr[indexpulse[k]:indexpulse[k+1]+1] # one sample overlap
            self.__trackduration[num,k] = max(tr_rev.times) - min(tr_rev.times)
            self.__tracklen[num,k] = np.count_nonzero(np.diff(tr_rev.read_data) == -1) # count bitcells, aka transitions from high->low

        # shorten first revolution by 1 bitcell to keep Aufit happy
        self.__tracklen[num,0] = self.__tracklen[num,0]-1
        # now extract all data between first and last index pulses
        trkstart = indexpulse[0]
        trkstop = indexpulse[-1]
        tr = tr[trkstart:trkstop+1]
        fluxchg = np.where(np.diff(tr.read_data) == -1)[0] # transitions from high->low
        fluxchg = fluxchg+1 # +1 so we get the indices where read_data is low
        self.__trackdata[num] = np.diff(tr.times[fluxchg])


    def saveimage(self, filename):
        with open(filename, "wb") as f:
            f.write(self.fileheader())
            f.write(self.trackoffsettable())
            for k in range(max(self.__trackdata.keys())+1):
                try:
                    f.write(self.trackheader(k))
                    f.write(self.trackdata(k))
                except:
                    pass
            f.write(time.asctime().encode("latin1"))

class LogicAnalyzer:
    def __init__(self):
        self.s = saleae.Saleae()

    def setup(self):
        # not all setup options are exposed via the Socket API, unfortunately.
        # therefore load "canned" settings from file
        self.s.load_from_file(os.path.join(os.getcwd(),ANALYZER_PRESET_FILE))

    def captureandsave(self, filename):
        self.s.close_all_tabs()
        self.s.capture_start_and_wait_until_finished()
        # faster binary export
        #self.s.export_data2(filename, format="csv", display_base="separate")
        self.s.export_data2(filename, format="binary", each_sample=False, word_size=64)
        while (not self.s.is_processing_complete()):
            time.sleep(.25)

class FloppyDrive:
    # connections between serial cable and floppy:
    # RTS = direction select = green, pin 18
    # DTR = side select = grey, 32
    # TX = step = orange, 20
    # CTS = track 0 = brown, 26
    # GND = GND = black, odd pin
    def __init__(self, portname):
        self.p = serial.Serial(port=portname, baudrate=115200)
        self.track = 0
        self.rezero()
        self.sideselect(0)

    def rezero(self):
        while (not self.p.cts): # track 0 sensor
            self.step("out", settle=False)
        self.track = 0

    # dirstr = "in" (i.e. towards track 79) or "out" (i.e. towards track 0)
    def step(self, dirstr, settle=True):
        if dirstr == "out":
            direction = False
        else:
            direction = True
        self.p.rts = direction
        time.sleep(1e-6) # setup time for direction signal
        # will output one low pulse for the start bit, duration 1/115200 = 8.7µs
        self.p.write(b"\xff")
        self.p.flush()
        if settle:
            time.sleep(20e-3) # head settling time
        else:
            time.sleep(3e-3) # minimum step rate
        if direction:
            if self.track > 0:
                self.track = self.track - 1
        else:
            self.track = self.track + 1

    def sideselect(self, side):
        if side == 1:
            self.p.dtr = True
        else:
            self.p.dtr = False

s = SCPWriter((MANUFACTURE<<4) + DISK_TYPE)
l = LogicAnalyzer()
l.setup()
f = FloppyDrive("com3")
f.rezero()

with tempfile.TemporaryDirectory() as tmpdirname:
    fname = os.path.join(tmpdirname,"export.bin")
    for _ in range(STARTING_CYL):
        f.step("in")
    for trackno in range(CYLINDERS-STARTING_CYL):
        for headno in range(HEADS):
            print("T%dS%d" % (trackno, headno))
            f.sideselect(headno)
            l.captureandsave(fname)
            s.loadtrack(HEADS*trackno + headno, fname)
        f.step("in")
        for _ in range(TRACKSKIP):
            f.step("in")

s.saveimage(OUTPUT_IMAGE_FILE)
