import socket
import binascii
import pycom
import time
import os
import machine
from machine import UART
from network import WLAN
from network import LoRa

uart = UART(0, baudrate=115200)
os.dupterm(uart)

wlan = WLAN()
wlan.deinit()

# Disable heartbeat LED
pycom.heartbeat(False)

# Initialize LoRa in LORAWAN mode.
lora = LoRa(mode=LoRa.LORAWAN, adr=True)

# create an OTAA authentication parameters
app_eui = binascii.unhexlify('1234567890123456')
app_key = binascii.unhexlify('47900F2BDED548CB345C658D3175EB98')

print(os.uname())
print("DevEUI: %s" % (binascii.hexlify(lora.mac())))
print("AppEUI: %s" % (binascii.hexlify(app_eui)))
print("AppKey: %s" % (binascii.hexlify(app_key)))

# join a network using OTAA (Over the Air Activation)
lora.join(activation=LoRa.OTAA, auth=(app_eui, app_key), timeout=0, dr=0)

# wait until the module has joined the network
while not lora.has_joined():
    pycom.rgbled(0x140000)
    time.sleep(2.5)
    pycom.rgbled(0x000000)
    time.sleep(1.0)
    print('Not yet joined...')

print('OTAA joined')
pycom.rgbled(0x001400)

# create a LoRa socket
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)

# set the LoRaWAN data rate
s.setsockopt(socket.SOL_LORA, socket.SO_DR, 5)

# battery info for MAC Commands
lora.set_battery_level(254)

machine.main('main.py')
