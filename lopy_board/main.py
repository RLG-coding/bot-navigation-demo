import time
import pycom
import gc

from pytrack import Pytrack
from network import LoRa
from L76GNSS import L76GNSS
from LIS2HH12 import LIS2HH12

from CayenneLPP import CayenneLPP

py = Pytrack()
fw_version = py.read_fw_version()
print("Pytrack firmware version: " + str(fw_version))
# Pytrack GPS
time.sleep(2)
# enable garbage collector
gc.enable()

coord = None
dl_data = ""
counter = 0
mvt = False

# Init GPS
gps = L76GNSS(py, timeout=10)
coord = None
lpp = CayenneLPP()

acc = LIS2HH12()
# enable the activity/inactivity interrupts
# set the accelereation threshold to 2000mG (2G) and the min duration to 200ms

pycom.rgbled(0x001400)
lpp.reset()
lpp.add_digital_output(0, 123)
print('Sending data (uplink)...')
s.setblocking(True)
s.send(bytes(lpp.get_buffer()))
s.setblocking(False)
pycom.rgbled(0x000000)

def handler_acc(event):
    global mvt
    print('Movement')
    mvt = True

acc.enable_activity_interrupt(200, 100, handler_acc)
pycom.rgbled(0x000000)

while True:
    # Acquire GPS
    print("--LOOP")
    coord = gps.coordinates()
    battery = py.read_battery_voltage()
    print("Battery voltage: " + str(battery))
    if mvt:
        pycom.rgbled(0x001400)
        lpp.reset()
        lpp.add_voltage(5, py.read_battery_voltage())
        print('Sending data (uplink)...')
        s.setblocking(True)
        s.send(bytes(lpp.get_buffer()))
        s.setblocking(False)
        pycom.rgbled(0x000000)
        dl_data = s.recv(64)
        if dl_data != "":
            print('Received data (downlink)', dl_data)
            print(lora.stats())
        counter = counter + 1
        print('Counter', counter)
        mvt = False
    else:
        print("No movement")
    time.sleep(2)
