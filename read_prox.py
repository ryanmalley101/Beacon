import RPi.GPIO as GPIO
import time

# Pin setup for Wiegand lines
D0_PIN = 20  # GPIO pin connected to Data 0
D1_PIN = 21  # GPIO pin connected to Data 1

# Variables to store Wiegand data
bit_count = 0
data = []


# Callback for Data 0
def d0_callback(channel):
    global bit_count, data
    bit_count += 1
    data.append(0)


# Callback for Data 1
def d1_callback(channel):
    global bit_count, data
    bit_count += 1
    data.append(1)


def main():
    global bit_count, data

    # GPIO setup
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(D0_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(D1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Add event detection for Wiegand signals
    GPIO.add_event_detect(D0_PIN, GPIO.FALLING, callback=d0_callback)
    GPIO.add_event_detect(D1_PIN, GPIO.FALLING, callback=d1_callback)

    print("Waiting for Wiegand data...")

    try:
        while True:
            time.sleep(0.1)

            # If we receive data
            if bit_count > 0:
                print(f"Bits received: {bit_count}")
                print(f"Data: {''.join(map(str, data))}")

                # Process data (e.g., parse facility code and card number)
                if bit_count == 26:  # Example for 26-bit Wiegand
                    print("26-bit Wiegand detected")
                    facility_code = int(''.join(map(str, data[1:9])), 2)
                    card_number = int(''.join(map(str, data[9:-1])), 2)
                    print(f"Facility Code: {facility_code}, Card Number: {card_number}")

                # Reset for next read
                bit_count = 0
                data = []
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
