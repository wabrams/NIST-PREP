import numpy as np
import sys
import scipy
import scipy.signal
import time
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import threading
import serial
import serial.tools.list_ports
import platform
import struct
import datetime

class dev:
    def __init__(self, port):
        self.port = serial.Serial(port)
        self.port.baudrate = 115200
        self.port.bytesize = serial.EIGHTBITS
        self.port.parity = serial.PARITY_NONE
        self.port.stopbits = serial.STOPBITS_ONE
        self.port.flush()

    def set_chirp_length(self, t=1e-3):
        self.pulse_width = t
        ba = bytearray(struct.pack("f", t))
        msg = b'0'+bytes(ba)
        self.port.write(msg)
        done = False
        while not done:
            msg = self.port.readline()
            print(msg)
            if msg.startswith(b'ack'):
                done = True

    def set_start(self, f=20000):
        self.freq_start = f
        msg = b'1'+(f).to_bytes(2, byteorder='little')
        self.port.write(msg)
        done = False
        while not done:
            msg = self.port.readline()
            print(msg)
            if msg.startswith(b'ack'):
                done = True

    def set_stop(self, f=20000):
        self.freq_stop = f
        msg = b'2'+(f).to_bytes(2, byteorder='little')
        self.port.write(msg)
        done = False
        while not done:
            msg = self.port.readline()
            print(msg)
            if msg.startswith(b'ack'):
                done = True

    def get_size(self, string):
        done = False
        while not done:
            msg = self.port.readline()
            # print(msg)
            if msg.startswith(string.encode()):
                done =  True
        size = msg.decode().split(':')[-1]
        return int(size)

    def get_mic(self):
        size = self.get_size('left')
        l = self.port.read((size<<1))
        size = self.get_size('right')
        r = self.port.read((size<<1))
        msg = self.port.readline()
        if len(msg)!=2:
            print("problem")
        self.l = np.frombuffer(l, dtype=np.int16)
        self.r = np.frombuffer(r, dtype=np.int16)

    def flush_serial(self):
        while True:
            print(self.port.readline())

    def cmd_a(self):
        self.port.write(b'a')

    def cmd_l(self):
        self.port.write(b'l')

    def cmd_t(self):
        self.port.write(b't')

def estimate_chirp(freq_start, freq_stop, pulse_width, offset=0):
    timerFreq = 19e6
    top_start = int(timerFreq / freq_start);  # period of start frequency
    top_stop = int(timerFreq / freq_stop);  # period of stop frequency
    N_waves = int(pulse_width * (freq_stop+freq_start)/2);
    top_step = (freq_start - freq_stop) / (freq_start * freq_stop) * timerFreq / (N_waves - 1);
    print(top_start, top_stop, top_step, N_waves)
    list_periods = []
    for i in range(N_waves):
        period = top_start + int(i*top_step)
        list_periods.append(period)
    list_periods =  np.array(list_periods)
    # list_periods
    chirp = []
    for period in list_periods:
        chirp = np.hstack([chirp, np.ones(int(period/2)), np.zeros(period-int(period/2))])
    # offset = 0
    chirp_pdm = chirp[offset::(32*6)]
    return chirp_pdm

app = QtGui.QApplication([])
PAUSED = False
UPDATE_PARAMS = False

Fs= 19e6/32/6
RATE = Fs
CHUNK=1<<11

def update_params_on_device(freq_start, freq_stop, pulse_width):
    global chirp
    s1.set_chirp_length(pulse_width)
    s1.set_start(f=freq_start)
    s1.set_stop(f=freq_stop)

    chirp = estimate_chirp(freq_start, freq_stop, pulse_width)

def list_ports():
    devOS = platform.system()
    print("Detected", devOS, "Computer")
    ports = []
    if devOS == "Windows":
        ports = serial.tools.list_ports.comports()
    print("Listing all Serial Ports:")
    for p in ports:
        print(p)

# initialize audio, microphone, speaker, pass callback as a parameter
def init_serial():
    global s1, s2, chirp

    list_ports()

    port = input("enter port:")
    s1 = dev(port)
    port = input("enter port:")
    s2 = dev(port)

    freq_start =  20000
    freq_stop = 20000
    pulse_width = 5e-3
    update_params_on_device(freq_start, freq_stop, pulse_width)

try:
    init_serial()
except Exception as e:
    print(e)

def play():
    global s1, s2, win, helper, UPDATE_PARAMS
    while(win.isVisible()):
        while ((not PAUSED) and win.isVisible()):
            s1.cmd_a()
            s2.cmd_l()
            s1.get_mic()
            s2.cmd_t()
            s2.get_mic()
            # s2.l()
            helper.update.emit()
            if UPDATE_PARAMS:
                UPDATE_PARAMS = False
                start = int(start_box.text())
                stop = int(stop_box.text())
                pulse_width = float(pulse_width_box.text())
                update_params_on_device(start, stop, pulse_width)
        time.sleep(1)  # sleep briefly while not chirping


w = QtGui.QWidget()

## Create some widgets to be placed inside
btn = QtGui.QPushButton('pause')
btn_save = QtGui.QPushButton('save')
btn_1x = QtGui.QPushButton('chirp 1x')
# text = QtGui.QLineEdit('enter text')
start_box = QtGui.QLineEdit()
start_box.setValidator(QtGui.QIntValidator())
stop_box = QtGui.QLineEdit()
stop_box.setValidator(QtGui.QIntValidator())
pulse_width_box = QtGui.QLineEdit()
pulse_width_box.setValidator(QtGui.QDoubleValidator())
try:
    start_box.setText(f'{s1.freq_start}')
    stop_box.setText(f'{s1.freq_stop}')
    pulse_width_box.setText(f'{s1.pulse_width}')
except Exception as e:
    print("Can't enter starting parameters")
    print(e)

listw = QtGui.QListWidget()
flo = QtGui.QFormLayout()
flo.addRow("start_freq", start_box)
flo.addRow("stop_freq", stop_box)
flo.addRow("pulse_width", pulse_width_box)
params = QtGui.QWidget()
params.setLayout(flo)

win = pg.GraphicsLayoutWidget(show=True, title=f"Pyaudio+pyqtgraph, fs={Fs}")

## Create a grid layout to manage the widgets size and position
layout = QtGui.QGridLayout()
w.setLayout(layout)

## Add widgets to the layout in their proper positions
layout.addWidget(btn, 0, 0)   # button goes in upper-left
layout.addWidget(btn_save, 1, 0)   # button goes in upper-left
layout.addWidget(btn_1x, 2, 0)   # button goes in upper-left
layout.addWidget(params, 3, 0)
# layout.addWidget(text, 1, 0)   # text edit goes in middle-left
layout.addWidget(listw, 4, 0)  # list widget goes in bottom-left
layout.addWidget(win, 0, 1, 5, 5)  # plot goes on right side, spanning 3 rows

## Display the widget as a new window
w.show()


def pause(evt):
    global PAUSED
    PAUSED = not PAUSED
    if PAUSED:
        btn.setText('Unpause')
    else:
        btn.setText('Pause')

def save_clicked(evt):
    date_suffix = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = 'mic_pwm_data_'+date_suffix
    print("Save data, filename", filename)
    params = np.array([s1.freq_start, s1.freq_stop, s1.pulse_width])
    np.savez(filename, s1.l, s1.r, s2.l, s2.r, params)

def play_1x(evt):
    global helper, s1, s2, win
    s1.cmd_a()
    s1.get_mic()
    # print(s1.l, s1.r)
    helper.update.emit()

def params_changed():
    global UPDATE_PARAMS
    print('start', start_box.text())
    print('stop', stop_box.text())
    print('pulse_width', pulse_width_box.text())
    UPDATE_PARAMS = True

btn.clicked.connect(pause)
btn_save.clicked.connect(save_clicked)
btn_1x.clicked.connect(play_1x)
start_box.selectionChanged.connect(params_changed)
stop_box.selectionChanged.connect(params_changed)
pulse_width_box.selectionChanged.connect(params_changed)
#  Setup plotting window
# win = pg.GraphicsLayoutWidget(show=True, title=f"Pyaudio+pyqtgraph, fs={Fs}")
n_curves = 4
win.resize(500,500)
p = win.addPlot()
p.setLabel('bottom', 'time', units='sec')
p.setRange(QtCore.QRectF(0, -1000, CHUNK/RATE, 2000))
p_curves = []
for i in range(n_curves):
    p_curves.append(p.plot())
win.nextRow()
p2 = win.addPlot()
p2.setLabel('bottom', 'frequency', units='Hz')
p2.setLogMode(False, True)
p2.setRange(QtCore.QRectF(0, 0, RATE/2, 5))
p2_curves = []
for i in range(n_curves):
    p2_curves.append(p2.plot())
win.nextRow()
p3 = win.addPlot()
p3.setLabel('bottom', 'time', units='sec')
p3.setRange(QtCore.QRectF(0, -1000, CHUNK/RATE, 2000))
p3_curves = []
for i in range(n_curves):
    p3_curves.append(p3.plot())
win.nextRow()
p4 = win.addPlot()
p4_curves = []
for i in range(n_curves):
    p4_curves.append(p4.plot())

# setup filter parameters
data = []
b, a = scipy.signal.butter(4, 0.1, btype='high')
# use zi to eliminate transients on successive filter of blocks of data
zi = np.zeros(max(len(a), len(b))-1)

def calc_corr(f, c, t):
    ret = np.correlate(f, c, 'full')
    ret[:len(t)] += t
    new_tail = ret[-len(t):]
    return ret[:len(f)], new_tail


screen_update_count = 0

#  This is called by qt slot/signal to update the graph
def update():
    global PAUSED
    if not PAUSED:
        global p_curves, chirp, s1, s2, fft1, fft2, xcorr1, xcorr2, screen_update_count
        print("updated:", screen_update_count)
        screen_update_count += 1
        
        x = np.arange(len(s1.l))/RATE
        p_curves[0].setData(x, s1.l, pen='y')
        p_curves[1].setData(x, s1.r, pen='r')
        p_curves[2].setData(x, s2.l, pen='b')
        p_curves[3].setData(x, s2.r, pen='g')
        # compute and plot fft's of raw and filtered data
        window = np.blackman(len(s1.l))
        for (curve, data, pen_color) in zip(p2_curves,
                                             [s1.l, s1.r, s2.l, s2.r],
                                             ['y', 'r', 'b', 'g']):
            D = np.abs(np.fft.rfft(data*window))
            freq = np.arange(len(D))*RATE/CHUNK
            curve.setData(freq, D, pen=pen_color)
        xcorr1 = np.correlate(s1.l, chirp, 'valid')
        xcorr2 = np.correlate(s1.r, chirp, 'valid')
        xcorr3 = np.correlate(s2.l, chirp, 'valid')
        xcorr4 = np.correlate(s2.r, chirp, 'valid')
        x2 = np.arange(len(xcorr1))/RATE
        bhigh, ahigh = scipy.signal.butter(4, 0.1, btype='high')
        xcorr1 = scipy.signal.lfilter(bhigh, ahigh, (xcorr1))
        xcorr2 = scipy.signal.lfilter(bhigh, ahigh, (xcorr2))
        xcorr3 = scipy.signal.lfilter(bhigh, ahigh, (xcorr3))
        xcorr4 = scipy.signal.lfilter(bhigh, ahigh, (xcorr4))
        p3_curves[0].setData(x2, np.abs(xcorr1), pen='y')
        p3_curves[1].setData(x2, np.abs(xcorr2), pen='r')
        p3_curves[2].setData(x2, np.abs(xcorr3), pen='b')
        p3_curves[3].setData(x2, np.abs(xcorr4), pen='g')

        for (curve, xcorr, pen_color) in zip(p4_curves,
                                             [xcorr1, xcorr2, xcorr3, xcorr4],
                                             ['y', 'r', 'b', 'g']):
            envelope = np.abs(xcorr).cumsum()
            navg = 100
            havg = (envelope[navg:] - envelope[:-navg])/navg
            curve.setData(x2[navg//2:-navg//2], havg, pen=pen_color)

        # peaks, props = scipy.signal.find_peaks(havg, prominence=5,
        #                                        height=5000)
        # # print(peaks/Fs*1000, props)
        # peaks = peaks.tolist()
        # items = []
        # for i in range(len(peaks)):
        #     items.append(f'{peaks[i]/Fs*1000:8.2f}: {props["peak_heights"][i]:8.2f}')
        # # peaks = [str(p) for p in peaks]
        # listw.clear()
        # listw.addItems(items)
# This is a class to signal that data is ready to be plotted...Interface to QT
# objects
#   Could not get the helper to pass parameters to update...used globals
class Helper(QtCore.QObject):
    update = QtCore.pyqtSignal()


helper = Helper()
helper.update.connect(update)


thread_play = threading.Thread(target=play)
thread_play.start()

## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    # import sys
    # if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
    #     QtGui.QApplication.instance().exec_()
    app.exec_()
