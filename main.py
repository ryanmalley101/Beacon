# SPDX-FileCopyrightText: 2018 Brent Rubell for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
Example for using the RFM69HCW Radio with Raspberry Pi.
Learn Guide: https://learn.adafruit.com/lora-and-lorawan-for-raspberry-pi
Author: Brent Rubell for Adafruit Industries
"""
# Import Python System Libraries
import time
# Import Blinka Libraries
import busio
from digitalio import DigitalInOut, Direction, Pull
import board
# Import the SSD1306 module.
import adafruit_ssd1306
# Import the RFM69 radio module.
import adafruit_rfm69

import pynmea2

import serial

import RPi.GPIO as GPIO
from mfrc522 import MFRC522, SimpleMFRC522, BasicMFRC522


class PatchedSimpleMFRC522(SimpleMFRC522):
    def __init__(self, **kwargs):
        self.READER = MFRC522(**kwargs)
        self.KEY = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
        self.TRAILER_BLOCK = 11
        self.BasicMFRC522 = BasicMFRC522()

try:
    # Button A
    btnA = DigitalInOut(board.D5)
    btnA.direction = Direction.INPUT
    btnA.pull = Pull.UP
    # Button B
    btnB = DigitalInOut(board.D6)
    btnB.direction = Direction.INPUT
    btnB.pull = Pull.UP
    # Button C
    btnC = DigitalInOut(board.D12)
    btnC.direction = Direction.INPUT
    btnC.pull = Pull.UP
    # Create the I2C interface.
    i2c = busio.I2C(board.SCL, board.SDA)
    # 128x32 OLED Display
    reset_pin = DigitalInOut(board.D4)
    display = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c, reset=reset_pin)
    # Clear the display.
    display.fill(0)
    display.show()
    width = display.width
    height = display.height
    # Configure Packet Radio
    CS = DigitalInOut(board.CE1)
    RESET = DigitalInOut(board.D25)
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    rfm69 = adafruit_rfm69.RFM69(spi, CS, RESET, 915.0)
    prev_packet = None
    # Optionally set an encryption key (16 byte AES key). MUST match both
    # on the transmitter and receiver (or be set to None to disable/the default).
    rfm69.encryption_key = b'\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08'

    reader = PatchedSimpleMFRC522(pin_rst=17)

    port = "/dev/ttyAMA0"
    ser = serial.Serial(port, baudrate=9600, timeout=0.5)
    dataout = pynmea2.NMEAStreamReader()

    while True:
        print("Starting Loop")
        packet = None
        # draw a box to clear the image
        display.fill(0)
        display.text('RasPi Radio', 35, 0, 1)
        # check for packet rx
        packet = rfm69.receive()
        if packet is None:
            display.show()
            line = ser.readline()
            if line:
                line = line.decode('latin-1')
                # print(line)
                newdata = line
                if newdata[0:6] == "$GNRMC":
                    newmsg = pynmea2.parse(newdata)
                    lat = newmsg.latitude
                    lng = newmsg.longitude
                    gps = "Latitude=" + str(lat) + "and Longitude=" + str(lng)
                    display.text(gps, 15, 20, 1)
        else:
            # Display the packet text and rssi
            display.fill(0)
            prev_packet = packet
            packet_text = str(prev_packet, "utf-8")
            display.text('RX: ', 0, 0, 1)
            display.text(packet_text, 25, 0, 1)
            time.sleep(1)
        if not btnA.value:
            # Send Button A
            display.fill(0)
            button_a_data = bytes("Button A!\r\n", "utf-8")
            rfm69.send(button_a_data)
            display.text('Sent Button A!', 25, 15, 1)
            print("Sent Button A!")
        elif not btnB.value:
            # Send Button B
            display.fill(0)
            button_b_data = bytes("Button B!\r\n", "utf-8")
            rfm69.send(button_b_data)
            display.text('Sent Button B!', 25, 15, 1)
            print("Sent Button B!")
        elif not btnC.value:
            # Send Button C
            display.fill(0)
            id, text = reader.read()
            print(id)
            print(text)
            display.text(f'Read {text}', 25, 15, 1)
        display.show()
        time.sleep(1)
finally:
    GPIO.cleanup()
