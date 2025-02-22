from struct import pack, iter_unpack
import time
import os
import tempfile
import numpy as np
import saleae
import serial

CONTROLLER_COM_PORT = "COM4"
CHANNEL_INDEX = 1
CHANNEL_DATA = 3

# Image settings
HEADS = 2
CYLINDERS = 83
STARTING_CYL = 0
TRACKSKIP = 0
FLUX_OFFSET = 50
OVERLAP = 1

OUTPUT_IMAGE_FILE = "floppy2.scp"

# Flags for the drive and disk used.
flag_48tpi          = 0x00
flag_96tpi          = 0x02
flag_300rpm         = 0x00
flag_360rpm         = 0x04

CONFIG_FLAGS = flag_96tpi + flag_360rpm

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

# CBM DISK TYPES
disk_C64        = 0x00
disk_Amiga      = 0x04

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

MANUFACTURE     = man_PC
DISK_TYPE       = disk_PC12M

class SCPWriter:

    def __init__(self):
        self.data = dict()
        self.trackduration = np.zeros((HEADS*(STARTING_CYL+CYLINDERS), REVOLUTIONS))
        self.tracklen = np.zeros((HEADS*(STARTING_CYL+CYLINDERS), REVOLUTIONS))

    def fileheader(self):
        # SCP header
        scp_magic       = b"SCP"
        scp_vers        = 0x22                      # version 2.2
        scp_type        = (MANUFACTURE<<4) + DISK_TYPE
        scp_nrev        = REVOLUTIONS               # number of revolutions
        scp_starttrack  = 0                         # always start at track 0  not at min(self.data.keys())
        scp_endtrack    = max(self.data.keys())
        scp_flags       = 1+CONFIG_FLAGS            # flux data starts at index
        scp_width       = 0                         # 16 bits
        scp_heads       = [-1, 1, 0][HEADS]
        scp_res         = int(40e6/SAMPLE_RATE) - 1
        scp_checksum    = 0                         # TODO
        scp_header = pack("<3sBBBBBBBBBL", scp_magic, scp_vers, scp_type, scp_nrev, scp_starttrack, scp_endtrack, scp_flags, scp_width, scp_heads, scp_res, scp_checksum)

        # SCP track table
        offs = len(scp_header) + (STARTING_CYL+CYLINDERS)*8                     # 2 long word for each cylinder
        for k in range(HEADS*(STARTING_CYL+CYLINDERS)):
            try:
                print("saving track {} head {}".format(int(k/HEADS), k%HEADS))
                scp_tlen = len(self.trackdata(k)) + len(self.trackheader(k))    # will raise KeyError if track does not exist
                scp_header += pack("<L", offs)
                offs += scp_tlen
            except:
                print("can't find track!")
                scp_header += pack("<L", 0)                                     # skip track
            if HEADS == 1:
                scp_header += pack("<L", 0) 

        return scp_header

    def trackheader(self, num): # will raise KeyError if track does not exist
        # SCP track header
        scp_tmagic = b"TRK"
        scp_trackno = num
        scp_trkhead = pack("<3sB", scp_tmagic, scp_trackno)
        scp_tstart = 4 + 12*REVOLUTIONS # first revolution starts after this header
        for k in range(REVOLUTIONS):
            scp_tduration = round(self.trackduration[num,k]*SAMPLE_RATE) 
            scp_tlen = self.tracklen[num,k] # in bitcells, not in bytes!
            scp_trkhead = scp_trkhead + pack("<LLL", int(scp_tduration), int(scp_tlen), scp_tstart)
            scp_tstart = scp_tstart + 2 * int(scp_tlen)  # start of next revolution
        return scp_trkhead

    def trackdata(self, num): # will raise KeyError if track does not exist
        # round to sample-rate precision
        bitcells = np.round(self.data[num]*SAMPLE_RATE)
        # pack into output data
        bitdata = pack(">%dH" % len(bitcells), *bitcells.astype('int'))
        return bitdata

    def loadtrack(self, num, filename):
        # load and process one track
        # trackdata = np.recfromcsv(filename)
        
        # faster binary file parser: assumes index in channel 1, read_data in channel 3
        file = iter_unpack("=qQ", open(filename,"rb").read())
        trackdata = np.rec.array([(point[0]/SAMPLE_RATE, (point[1] >> CHANNEL_INDEX) & 1, (point[1] >> CHANNEL_DATA) & 1) for point in file],
                                 dtype = [('times','f8'), ('index','i4'), ('read_data','i4')])

        indexpulse = np.where(np.diff(trackdata.index) == -1)[0]+1 # index transitions from high->low
        if len(indexpulse) != REVOLUTIONS+1:
            print("Not enough index pulses found")

        for revolution in range(REVOLUTIONS):
            # one revolution
            trackdata_rev = trackdata[indexpulse[revolution]:indexpulse[revolution+1]+OVERLAP] # one sample overlap
            self.trackduration[num,revolution] = max(trackdata_rev.times) - min(trackdata_rev.times)
            self.tracklen[num,revolution] = np.count_nonzero(np.diff(trackdata_rev.read_data) == -1) # count bitcells, aka transitions from high->low

        # shorten first revolution by 1 bitcell to keep Aufit happy
        for revolution in range(REVOLUTIONS-1):
            self.tracklen[num,revolution] = self.tracklen[num,revolution]-OVERLAP

        # now extract all data between first and last index pulses
        trkstart = indexpulse[0]+FLUX_OFFSET
        trkstop = indexpulse[-1]+FLUX_OFFSET
        trackdata = trackdata[trkstart:trkstop+1]
        fluxchg = np.where(np.diff(trackdata.read_data) == -1)[0]+1 # transitions from high->low
        self.data[num] = np.diff(trackdata.times[fluxchg])


    def saveimage(self, filename):
        with open(filename, "wb") as f:
            f.write(self.fileheader())
            for k in range(max(self.data.keys())+1):
                try:
                    t_dat = self.trackdata(k)
                    f.write(self.trackheader(k))
                    f.write(t_dat)
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

    def wait_for_analyzer(self):
        starttime = time.time()
        while not self.s.is_processing_complete():
            time.sleep(.1)
            if time.time() > starttime + 5:
                raise Exception("Timeout!")

    def captureandsave(self, filename):
        self.s.close_all_tabs()
        self.s.capture_start()
        self.wait_for_analyzer()
        self.s.export_data2(filename, format="binary", each_sample=False, word_size=64)
        self.wait_for_analyzer()

class FloppyDrive:
    # For controlling the floppy-drive automation controller
    def __init__(self, portname):
        self.s = serial.Serial(port=portname, baudrate=115200)
        time.sleep(3)
        self.start()
        time.sleep(3)
        self.rezero()
        self.sideselect(0)

    def _write(self, txt):
        self.s.write(txt.encode())
        self.s.write(0xD)
        self.s.write(0xA)
        while self.s.readline() != b'OK\r\n':
            pass

    def start(self):
        self._write('start')
        time.sleep(3)

    def stop(self):
        self._write('stop')

    def rezero(self):
        print("stepping home")
        self._write('step 0')

    # dirstr = "in" (i.e. towards track 79) or "out" (i.e. towards track 0)
    def step(self, dirstr):
        if dirstr == "in":
            print("stepping +")
            self._write('step +')
        elif dirstr == "out":
            print("stepping -")
            self._write('step -')

    def sideselect(self, side):
        self._write('head %d' % side)



s = SCPWriter()
l = LogicAnalyzer()
l.setup()
f = FloppyDrive(CONTROLLER_COM_PORT)

with tempfile.TemporaryDirectory() as tmpdirname:
    fname = os.path.join(tmpdirname,"export.bin")
    for _ in range(STARTING_CYL):
        f.step("in")
    for trackno in range(STARTING_CYL, STARTING_CYL+CYLINDERS):
        for headno in range(HEADS):
            print("T%dS%d" % (trackno, headno))
            f.sideselect(headno)
            while True:
                try:
                    l.captureandsave(fname)
                    s.loadtrack(HEADS*trackno + headno, fname)
                    break
                except Exception as e:
                    continue
        f.sideselect(0)
        time.sleep(0.2)
        f.step("in")
        for _ in range(TRACKSKIP):
            f.step("in")
time.sleep(2)
f.rezero()
f.stop()

s.saveimage(OUTPUT_IMAGE_FILE)
