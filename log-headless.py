from datetime import datetime
import serial
import serial.tools.list_ports
import platform
import numpy as np
import csv
from progress.bar import Bar

def printLog(*args, **kwargs):
    stamp = str(datetime.now())
    print("[" + stamp + "]", *args)

def errorLog(*args, **kwargs):
    stamp = str(datetime.now())
    print("[" + stamp + "]", *args)

def listPorts():
    devOS = platform.system()
    printLog("Detected", devOS, "Computer")
    ports = []
    if devOS == "Windows":
        ports = serial.tools.list_ports.comports()
    printLog("Listing all Serial Ports:")
    for p in ports:
        printLog(p)

def setupSerial():
    listPorts()
    while True:
        try:
            port = input("enter port:")
            s = serial.Serial(port)
            s.baudrate = 115200
            s.bytesize = serial.EIGHTBITS
            s.parity = serial.PARITY_NONE
            s.stopbits = serial.STOPBITS_ONE
            printLog("Serial Port open on", port.upper(), "for EFR32BG22")
            return s
        except serial.SerialException:
            errorLog("Invalid Serial Port, please try again!")
            continue

def serialQuerySamples(conn):
    conn.write(b"s")
    resp = conn.readline().decode().strip()
    if resp == "samp":
        resp = int(conn.readline().decode().strip())
        printLog("EFR32BG22 is sampling", resp, "points of PDM data")
        return resp

def setupDataFile():
    filename = "listen.csv"
    csvFile = open(filename, "w", newline='') # b/c windows translates \r\n into \r\r\n
    csvStream = csv.writer(csvFile)
    printLog("Storing data to:", filename)
    return csvFile, csvStream

ser = setupSerial()
samp = serialQuerySamples(ser)
dataFile, dataCSV = setupDataFile()
data_samples = int(input("enter number of trials to record:"))

larr = np.zeros(samp)
rarr = np.zeros(samp)

with Bar('Processing', max=data_samples) as bar:
    for i in range(data_samples):
        ser.write(b"a")

        larr_raw = []
        rarr_raw = []

        res = ser.readline().decode().strip()
        if res == "left":
            larr_raw = ser.readline().decode().strip().split(" ") # printLog(larr_raw)
        else:
            # todo: throw error / warning
            errorLog("not left!")

        res = ser.readline().decode().strip()
        if res == "right":
            rarr_raw = ser.readline().decode().strip().split(" ") # printLog(rarr_raw)
        else:
            errorLog("not right!")

        larr = np.array(list(map(int, larr_raw)))
        rarr = np.array(list(map(int, rarr_raw)))

        # printLog("Writing to CSV", i)
        dataCSV.writerow(larr)
        dataCSV.writerow(rarr)
        bar.next()
