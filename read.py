from datetime import datetime
import serial
import serial.tools.list_ports
import platform
import matplotlib.pyplot as plt
import numpy as np
import csv

plt.style.use('dark_background')

def tprint(*args, **kwargs):
    stamp = str(datetime.now())
    print("[" + stamp + "]", *args)
# determine which OS
devOS = platform.system()
tprint("detected", devOS, "device")

# list all serial ports
ports = []
tprint("listing all serial ports:")
if devOS == "Windows":
    ports = serial.tools.list_ports.comports()
for p in ports:
    print(p)

# 115200 bps, 8 bits, no parity, and 1 stop bit
while True:
    try:
        port = input("enter port:")
        s = serial.Serial(port)
        break
    except serial.SerialException:
        tprint("invalid serial port, please try again!")
        continue

tprint("setting up", port, "for reading BG22")
s.baudrate = 115200
s.bytesize = serial.EIGHTBITS
s.parity = serial.PARITY_NONE
s.stopbits = serial.STOPBITS_ONE

samp_len = 1024
larr = np.zeros(samp_len)
rarr = np.zeros(samp_len)
# fsamp = (19e6 / (32*6))
n = np.linspace(0, 10, samp_len)
# 19MHz / (32 * 6) =

plt.ion()
fig, ax = plt.subplots()
lineL, lineR = ax.plot(n, larr, 'r-', n, rarr, 'b-')
ax.set_xlabel("Time (mS)")
ax.set_ylabel("Amplitude")
ax.set_ylim(-1000, 1000)
ax.set_title("PDM Microphone Data")
fig.canvas.draw()

tprint("Opening CSV")
filename = "listen.csv"
csvfile = open(filename, "w", newline='') #windows translates \r\n into \r\r\n
csvwriter = csv.writer(csvfile)

while True:
    s.write(b"r")

    larr_raw = []
    rarr_raw = []

    res = s.readline().decode().strip()
    if res == "left":
        larr_raw = s.readline().decode().strip().split(" ") # tprint(larr_raw)
    else:
        # todo: throw error / warning
        tprint("not left!")

    res = s.readline().decode().strip()
    if res == "right":
        rarr_raw = s.readline().decode().strip().split(" ") # tprint(rarr_raw)
    else:
        tprint("not right!")

    larr = np.array(list(map(int, larr_raw)))
    rarr = np.array(list(map(int, rarr_raw)))

    tprint("writing to CSV")
    csvwriter.writerow(larr)
    csvwriter.writerow(rarr)

    lineL.set_ydata(larr)
    lineR.set_ydata(rarr)

    # tprint(len(larr), len(rarr), len(n))

    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(0.01)
