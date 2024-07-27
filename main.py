import micropython
micropython.alloc_emergency_exception_buf(1000)

import array
import math
import time
import machine
from machine import Pin, PWM
import random
import gc
import rp2
from pimoroni_yukon import Yukon


# PIO program to read a PWM signal on its input pin
# Requires the single input pin to be mapped to both 
# its IN pin and its JMP pin.
#
# Output is 32 bits encoding the high time and the 
# interval.  The high time is in the most significant
# 16 bits; the interval is in the least significant 16
# bits.  The values are encoded as 0xffff - value.
@rp2.asm_pio(push_thresh=32, autopush=True)
def pio_read_pwm():
    wrap_target()                # while (1) {             # type: ignore

    mov(x, invert(null))         #   x = 0xffffffff        # type: ignore
    
    label("loop_high")           #   do {                  # type: ignore
    jmp(x_dec, "xdec1")          #     x--                 # type: ignore
    label("xdec1")               #                         # type: ignore
    nop()                        #                         # type: ignore
    jmp(pin, "loop_high")        #   } while (pin high) # type: ignore
    
    in_(x, 16)                   #   ISR = (ISR << 16) | (X & 0xffff) # type: ignore

    label("loop_low")            #   do {                  # type:ignore 
    jmp(x_dec, "xdec2")          #     x--                 # type:ignore
    label("xdec2")               #                         # type:ignore
    jmp(pin, "exit_loop_low")    #     if (pin high) break # type:ignore
    jmp("loop_low")              #   } while (1)           # type:ignore
    label("exit_loop_low")       #                         # type:ignore

    in_(x, 16)                   #   ISR = (ISR << 16) | (X & 0xffff) # type: ignore
    
    wrap()                       # }                       # type:ignore           


pin_sensor = Pin(23, Pin.IN, Pin.PULL_UP)
pwm_sm = rp2.StateMachine(
    0, 
    pio_read_pwm, 
    freq=125_000_000,  
    in_base=pin_sensor, 
    jmp_pin=pin_sensor,
)

# Start the StateMachine's running.
pwm_sm.active(1)

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

_read_pwm_block_read_buf=array.array("i", [0])
@micropython.viper
def read_pwm_block(pwm_sm, out_buf:ptr32): # type:ignore
    """Read a fresh value from the PWM FIFO.

    Detects FIFO overflow and resyncs with the PWM signal 
    if needed (this takes 2-3 PWM cycles).
    
    To avoid allocations, the values are returned via
    the out_buf in the order high, interval.
    """

    # Check if we need to handle a FIFO overflow.
    overflow = int(pwm_sm.rx_fifo()) == 4
    # Drain the FIFO.
    while 1:
        pwm_sm.get(_read_pwm_block_read_buf)
        if int(pwm_sm.rx_fifo()) == 0:
            break
    if overflow:
        # If FIFO overflowed, then the next value will
        # be garbage because the PIO program blocks and 
        # restarts at a random point of a PWM cycle.
        # Read another couple of values to get back in 
        # sync.
        pwm_sm.get(_read_pwm_block_read_buf)
        pwm_sm.get(_read_pwm_block_read_buf)

    buf_ptr = ptr16(_read_pwm_block_read_buf)  # type: ignore
    high = 0xffff - buf_ptr[1]
    invl = 0xffff - buf_ptr[0]
    # Return via the provided buffer because returning 32
    # bits or a tuple would allocate.
    out_buf[0] = high
    out_buf[1] = invl


@micropython.viper
def set_duty(angle : int, power : int):
    lut_ptr = ptr16(lut)
    angle = angle & 0xfff
    pwms[0].duty_u16((lut_ptr[angle] * power) >> 12)
    angle2 = (angle+1365) & 0xfff
    pwms[1].duty_u16((lut_ptr[angle2] * power) >> 12)
    angle3 = (angle+2731) & 0xfff
    pwms[2].duty_u16((lut_ptr[angle3] * power) >> 12)


@micropython.viper
def clamp_angle(angle:int) -> int:
    angle = angle % 4096
    if angle >= 2048:
        angle -= 4096
    elif angle < -2048:
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

    pwm_buf = array.array("I", [0] * 2)
    read_pwm_block(pwm_sm, pwm_buf)
    high = pwm_buf[0]
    invl = pwm_buf[1]
    angle1 = clamp_angle(high*4096//invl)
    print("Calibration interval A = {} / {} = {} = {}".format(high, invl, angle1, angle1*360/4096))

    for phase_angle in range(lut_len//4, lut_len*9//4):
        time.sleep_us(100)
        pin_debug.on()
        machine.mem32[PWM_INTR] = 0x7f
        set_duty(phase_angle, 1024)
        pin_debug.off()
        
    time.sleep_ms(500) # Let rotor settle.
    read_pwm_block(pwm_sm, pwm_buf)
    high2 = pwm_buf[0]
    invl2 = pwm_buf[1]
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
        # Pre-allocated buffer for use by ISR.
        self.sm_buf = array.array('i',[0])

        self.angle_offset : int = 0
        self.num_pole_pairs : int = 0

        self.drive_offset :int = 1024
        self.a_was_pressed : bool = False

    def calibrate(self):
        self.angle_offset, self.num_pole_pairs = full_calibration()

    @micropython.viper
    def _read_pwm(self) -> int:
        """Read next value from the PWM FIFO.
        
        Returns the wheel angle in 4096ths.
        """
        # FIXME Hard coded to read SM0's FIFO
        fifo = ptr32(0x50200020)    
    
        # Data is actually two 16-bit counters packed into
        # a 32-bit word.
        packed_value = fifo[0]
        raw_invl = packed_value & 0xffff
        raw_high = (packed_value >> 16) & 0xffff
        invl = 0xffff - raw_invl
        high = 0xffff - raw_high
        angle = high*4119//invl - 15
        # TODO: <16 means "error"
        if angle < 0:
            angle = 0
        if angle > 4095:
            angle = 0
        return angle

    @micropython.viper
    def update(self, pio):
        pin_debug.on()
        try:
            # FIXME Hard coded to read SM0's FIFO
            fifo = ptr32(0x50200020)    
        
            # Data is actually two 16-bit counters packed into
            # a 32-bit word.
            packed_value = fifo[0]
            raw_invl = packed_value & 0xffff
            raw_high = (packed_value >> 16) & 0xffff
            invl = 0xffff - raw_invl
            high = 0xffff - raw_high
            angle = high*4119//invl - 15
            # TODO: <16 means "error"
            if angle < 0:
                angle = 0
            if angle > 4095:
                angle = 0
            
            pole_angle : int = angle * int(self.num_pole_pairs) + int(self.angle_offset)
            drive_angle : int = pole_angle + int(self.drive_offset)

            power = 1024
            lut_ptr = ptr16(lut)
            tap1 = drive_angle & 0xfff
            tap2 = (drive_angle+1365) & 0xfff
            tap3 = (drive_angle+2731) & 0xfff
            duty1 = (lut_ptr[tap1] * power) >> 18 # Extra shift of 6 for PWM TOP scale.
            duty2 = (lut_ptr[tap2] * power) >> 18
            duty3 = (lut_ptr[tap3] * power) >> 18

            # Want to do this but each method call costs 10us, which
            # is far too much.
            # pwms[0].duty_u16(duty1)
            # pwms[1].duty_u16(duty2)
            # pwms[2].duty_u16(duty3)

            # 20 = 2A 0x40050034 LSB
            # 21 = 2B            MSB
            # 22 = 3A 0x40050048
            ptr32(0x40050034)[0] = duty1 | (duty2<<16)
            ptr32(0x40050048)[0] = duty3
        except BaseException:
            global stop 
            stop = True
        pin_debug.off()


@micropython.native
def run():
    yukon.enable_main_output()
    
    motor = Motor()
    motor.calibrate()

    drive_offset = 1024
    a_was_pressed = False

    try:
        # Attach interrupt handler to the "FIFO has data" interrupt. 
        INTR_SM0_RXNEMPTY = 0x001
        rp2.PIO(0).irq(motor.update, trigger=INTR_SM0_RXNEMPTY, hard=True)
        
        while True:
            if stop:
                print("Interrupt in IRQ, stopping")
                break

            boot_pressed = yukon.is_boot_pressed()
            if boot_pressed:
                print("Boot pressed, stopping")
                break

            a_pressed = yukon.is_pressed("A")
            if a_pressed and not a_was_pressed:
                motor.drive_offset = 4096 - motor.drive_offset
                a_was_pressed = True
            elif not a_pressed:
                a_was_pressed = False
                
            # Do some float operations and GC to prove that the IRQ is
            # decoupled from the main loop.
            #print("Random:", random.random())
            #print("Alloc:", gc.mem_alloc())
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
    gc.collect()
