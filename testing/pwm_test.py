#  Control a 5V PWM fan speed with the lgpio library
#  Uses lgpio library, compatible with kernel 5.11
#  Author: William 'jawn-smith' Wilson

import lgpio
import time

# Configuration
FREQ = 10000
ENA = 23
ENB = 18
IN1 = 27
IN2 = 22
IN3 = 25
IN4 = 24


h = lgpio.gpiochip_open(0)

try:
    while True:
        # Turn the fan off
        lgpio.tx_pwm(h, IN1, FREQ, 0)
        time.sleep(10)

        # Turn the fan to medium speed
        lgpio.tx_pwm(h, FAN, FREQ, 50)
        time.sleep(10)

        # Turn the fan to max speed
        lgpio.tx_pwm(h, FAN, FREQ, 100)
        time.sleep(10)

except KeyboardInterrupt:
    # Turn the fan to medium speed
    lgpio.tx_pwm(h, FAN, FREQ, 50)
    lgpio.gpiochip_close(h)
