import micropython
micropython.alloc_emergency_exception_buf(100)

import array
import math
import time
import machine
from machine import Pin, PWM
import random
import gc
import rp2
from pimoroni_yukon import Yukon


@rp2.asm_pio()
def measure_high_time():
    # Wait for a LOW.
    wait(0, pin, 0)
    # Wait for a HIGH.
    wait(1, pin, 0)

    wrap_target()
    # x = 0xffffffff
    mov(x, invert(null))

    label("high_loop")
    jmp(x_dec, "cont_high_loop")
    label("cont_high_loop")
    nop()
    jmp(pin, "high_loop")

    mov(isr, x)
    # Wait for a HIGH before we push to avoid jitter.
    wait(1, pin, 0)
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



pwms = [PWM(Pin(n)) for n in [20, 21, 22]]

PWM_BASE = 0x40050000
PWM_EN = PWM_BASE + 0xa0
PWM_INTR = PWM_BASE + 0xa4

PWM_REGS = {}

for bank in range(8):
    bank_base = PWM_BASE + 0x14*bank
    PWM_REGS[bank] = {
        "CSR": bank_base,
        "CTR": bank_base + 0x08,    
    }

PWM_BANK_LO = 2
PWM_BANK_HI = 3

# Set the PWM frequency.
f = 40000
ivl = 0.99/f
for pwm in pwms:
    pwm.freq(f*2)  # *2 for phase accurate PWM

# Sync PWMs. EN register mirrors enable bits for individual PWMs.
machine.mem32[PWM_EN] = 0
machine.mem32[PWM_REGS[PWM_BANK_LO]["CSR"]] = machine.mem32[PWM_REGS[PWM_BANK_LO]["CSR"]] | 0x2
machine.mem32[PWM_REGS[PWM_BANK_HI]["CSR"]] = machine.mem32[PWM_REGS[PWM_BANK_HI]["CSR"]] | 0x2
machine.mem32[PWM_REGS[PWM_BANK_LO]["CTR"]] = 0
machine.mem32[PWM_REGS[PWM_BANK_HI]["CTR"]] = 0
machine.mem32[PWM_EN] = (1<<2) | (1 <<3)

lut_len : int = 4096
third = 4096 // 3
offset = 0.016
lut = array.array('H', [
    min(
        int(((math.sin(i * 2 * math.pi / lut_len) + 1)/2 + offset) / (1+offset) * 65535),
        65535
    ) for i in range(lut_len)])


def read_pwm_slow():
    sm_buf = array.array('H',[0])
    while 1:
        sm0.get(sm_buf)
        if sm0.rx_fifo() == 0:
            sm0.get(sm_buf)
            break
    high = 0xffff - sm_buf[0] 

    while 1:
        sm1.get(sm_buf)
        if sm1.rx_fifo() == 0:
            sm1.get(sm_buf)
            break
    invl = 0xffff - sm_buf[0]
    return high, invl


@micropython.viper
def set_duty(angle : int, power : int):
    angle = angle % 4096
    pwms[0].duty_u16(int(lut[angle]) * power // 4096)
    angle2 = (angle+(4096 // 3))%4096
    pwms[1].duty_u16(int(lut[angle2]) * power // 4096)
    pwms[2].duty_u16(int(lut[(angle+(2 *4096 // 3))%4096]) * power // 4096)


def clamp_angle(angle):
    angle = angle % 4096
    while angle >= 2048:
        angle -= 4096
    while angle < -2048:
        angle += 4096
    return angle


def full_calibration():
    # Calibration: rotate by a quarter phase to capture the rotor.
    # Seems to be a good angle to read off the angle offset.  Maybe
    # because the first duty is at 100%?
    for phase_angle in range(lut_len//4):
        time.sleep_us(100)
        pin_debug.on()
        machine.mem32[PWM_INTR] = 0x7f
        set_duty(phase_angle, 1024)
        pin_debug.off()
        
    time.sleep_ms(500) # Let rotor settle.
    high, invl = read_pwm_slow()
    angle1 = clamp_angle(high*4096//invl)
    print("Calibration interval A = {} / {} = {} = {}".format(high, invl, angle1, angle1*360/4096))

    for phase_angle in range(lut_len//4, lut_len*9//4):
        time.sleep_us(100)
        pin_debug.on()
        machine.mem32[PWM_INTR] = 0x7f
        set_duty(phase_angle, 1024)
        pin_debug.off()
        
    time.sleep_ms(500) # Let rotor settle.
    high2, invl2 = read_pwm_slow()
    angle2 = clamp_angle(high2*4096//invl2)
    print("Calibration interval B = {} / {} = {} = {}".format(high2, invl2, angle2, angle2*360/4096))
    delta = clamp_angle(angle2-angle1)
    print("Delta = {} = {}".format(delta, delta*360/4096))
    num_pole_pairs = (4096*2*32//abs(delta) + 15) // 32
    print("Pole pairs = {}".format(num_pole_pairs))

    # Now we've got the number of poles, we can work out the phase offset.
    pole_angle = 4096//4 # Angle we set when we did the measurement
    meas_pole_angle = clamp_angle(angle1 * num_pole_pairs)
    angle_offset = clamp_angle(pole_angle - meas_pole_angle)
    print("Angle offset = {}".format(angle_offset))

    for pwm in pwms:
        pwm.duty_u16(0)

    return angle_offset, num_pole_pairs


stop = False


class Motor:
    def __init__(self) -> None:
        # Using a 16-bit array so that StateMachine.get() discards the 
        # top 16 bits, which prevents our value from spilling to the 
        # heap.
        self.sm_buf = array.array('H',[0])
        self.angle_offset : int = 0
        self.num_pole_pairs : int = 0

        self.drive_offset :int = 1024
        self.a_was_down : bool = False

        self.invl : int = 0

    def calibrate(self):
        self.angle_offset, self.num_pole_pairs = full_calibration()
        sm1.get(self.sm_buf)
        invl = 0xffff - self.sm_buf[0]

    @micropython.native
    def update(self, pio):
        try:
            pin_debug.on()
            sm0.get(self.sm_buf)
            high = 0xffff - self.sm_buf[0] 

            if sm1.rx_fifo() > 0:
                sm1.get(self.sm_buf)
                self.invl = 0xffff - self.sm_buf[0]

            duty = high*4119//self.invl - 15
            # TODO: <16 means "error"
            if duty < 0:
                duty = 0
            if duty > 4095:
                duty = 0

            pole_angle = clamp_angle(duty * self.num_pole_pairs + self.angle_offset)
            
            drive_angle = pole_angle + self.drive_offset
            if drive_angle < 0:
                drive_angle += 4096
            drive_angle = drive_angle % 4096
            set_duty(drive_angle, 1024)
        except BaseException:
            global stop 
            stop = True
        finally:
            pin_debug.off()


@micropython.native
def run():
    yukon.enable_main_output()
    
    motor = Motor()
    motor.calibrate()

    sm_buf = array.array('H',[0])

    drive_offset = 1024
    a_was_down = False

    try:
        INTR_SM0_RXNEMPTY = 0x001
        rp2.PIO(0).irq(motor.update, trigger=INTR_SM0_RXNEMPTY, hard=True)
        
        while True:
            # yukon.is_boot_pressed() gives spurious readings while
            # the interrupt handler is running.  Need to debug!
            #s = machine.disable_irq()
            bp = yukon.is_boot_pressed()
            ap = yukon.is_pressed("A")
            #machine.enable_irq(s)

            if ap and not a_was_down:
                motor.drive_offset = 4096 - motor.drive_offset
                a_was_down = True
            elif not ap:
                a_was_down = False
            if bp:
                print("Boot pressed, stopping")
                break
            if stop:
                print("Interrupt in IRQ, stopping")
                break
            print("Random:", random.random())
            print("Alloc:", gc.mem_alloc())
            gc.collect()
    finally:
        print("Shutting down")
        rp2.PIO(0).irq(None)

try:
    run()

finally:
    # Put the board back into a safe state, regardless of how the program may have ended
    for pwm in pwms:
        pwm.duty_u16(0)
    yukon.reset()
