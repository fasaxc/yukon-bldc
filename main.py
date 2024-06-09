import time
from machine import Pin
import micropython
import rp2
import gc
from pimoroni_yukon import Yukon


@rp2.asm_pio()
def measure_high_time():
    wrap_target()
    # x = 0xffffffff
    mov(x, invert(null))

    # Wait for a HIGH.
    wait(1, pin, 0)

    label("high_loop")
    jmp(x_dec, "cont_high_loop")
    label("cont_high_loop")
    nop()
    jmp(pin, "high_loop")

    mov(isr, x)
    push(noblock)

    wrap()

@rp2.asm_pio()
def measure_interval():
    wrap_target()

    # x = 0xffffffff
    mov(x, invert(null))

    # Wait for LOW->HIGH transition.
    wait(0, pin, 0)
    wait(1, pin, 0)

    label("loop_while_high")
    jmp(x_dec, "cont_loop_while_high")
    label("cont_loop_while_high")
    nop()
    jmp(pin, "loop_while_high")

    label("loop_while_low")
    jmp(x_dec, "cont_loop_while_low")
    label("cont_loop_while_low")
    jmp(pin, "send")
    jmp("loop_while_low")

    label("send")
    mov(isr, x)
    push(noblock)

    wrap()


pin_sensor = Pin(23, Pin.IN, Pin.PULL_UP)
sm0 = rp2.StateMachine(0, measure_high_time, freq=125_000_000,  in_base=pin_sensor, jmp_pin=pin_sensor)
sm1 = rp2.StateMachine(4, measure_interval, freq=125_000_000, in_base=pin_sensor, jmp_pin=pin_sensor)

# Start the StateMachine's running.
sm0.active(1)
sm1.active(1)


pin_debug = Pin(26, Pin.OUT)


# Variables
yukon = Yukon()     # A new Yukon object
led_state = False   # The state of the LED

# Wrap the code in a try block, to catch any exceptions (including KeyboardInterrupt)
try:
    # Loop until the BOOT/USER button is pressed
    while not yukon.is_boot_pressed():
        pin_debug.high()
        while 1:
            high = 0xffffffff - sm0.get()
            if sm0.rx_fifo() == 0:
                break
        while sm1.rx_fifo() > 0:
            ivl = 0xffffffff - sm1.get()

        duty = high*4119//ivl - 15
        if duty < 0:
            duty = 0
        if duty > 4095:
            duty = 0
        
        pin_debug.low()
        gc.collect()
        #print("{:6} {:6} {:4} {:.2f}".format(ivl, high, duty, duty*360/4096))

finally:
    # Put the board back into a safe state, regardless of how the program may have ended
    yukon.reset()
