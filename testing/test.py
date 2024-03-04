#  Blink an LED with the LGPIO library
#  Uses lgpio library, compatible with kernel 5.11
#  Author: William 'jawn-smith' Wilson

import time
import lgpio

ENA = 23
ENB = 18
IN1 = 27
IN2 = 22
IN3 = 25
IN4 = 24

# open the gpio chip and set the LED pin as output
h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, IN1)
lgpio.gpio_claim_output(h, IN2)
lgpio.gpio_claim_output(h, ENA)


try:
    while True:
        print("going")
        # Turn the GPIO pin on
        lgpio.gpio_write(h, IN1, 1)
        lgpio.gpio_write(h, IN2, 0)

        time.sleep(0.5)

except KeyboardInterrupt:
    print("failed")
    lgpio.gpio_write(h, IN1, 0)
    lgpio.gpio_write(h, IN2, 0)

    lgpio.gpiochip_close(h)
