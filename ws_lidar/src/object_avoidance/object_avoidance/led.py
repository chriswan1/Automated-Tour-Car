import time
import pigpio

LED_PIN = 18  # Adjust GPIO pin as needed

# Initialize pigpio
pi = pigpio.pi()

if not pi.connected:
    print("Failed to connect to pigpio daemon. Is it running?")
    exit(1)

# Set GPIO mode
pi.set_mode(LED_PIN, pigpio.OUTPUT)

# Turn on LED
pi.write(LED_PIN, 1)
print("LEDs on for 5 seconds...")

time.sleep(5)

# Turn off LED
pi.write(LED_PIN, 0)
print("LEDs off.")

# Stop pigpio
pi.stop()
