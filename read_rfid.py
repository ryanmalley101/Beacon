import RPi.GPIO as GPIO
from mfrc522 import MFRC522, SimpleMFRC522, BasicMFRC522


class PatchedSimpleMFRC522(SimpleMFRC522):
    def __init__(self, **kwargs):
        self.READER = MFRC522(**kwargs)
        self.KEY = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
        self.TRAILER_BLOCK = 11
        self.BasicMFRC522 = BasicMFRC522()


reader = PatchedSimpleMFRC522(pin_rst=17,gpio_mode=GPIO.BCM)

try:
    id, text = reader.read()
    print(id)
    print(text)
finally:
    GPIO.cleanup()
