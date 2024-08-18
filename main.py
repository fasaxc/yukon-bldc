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

pin_debug = Pin(26, Pin.OUT)


# Variables
yukon = Yukon()  # A new Yukon object
led_state = False  # The state of the LED

# Wrap the code in a try block, to catch any exceptions (including KeyboardInterrupt)


# Hack to make VSCode happy with viper pointers.
ptr32 = ptr16 = lambda x: x

stop = False


class Motor:
    _lut_len: int = 4096
    _lut = None

    def __init__(
        self,
        pio_sm_num: int,
        slot_num: int,
        pwm_freq: int = 40000,
    ) -> None:
        """
        Parameters:
          pio_sm_num: PIO StateMachine number to use.
          slot_num:   Yukon slot number (used to determine which
                      group of GPIO pins/PWMs to use).

                      slot 1 -> GPIO 0-3,   PWM 0-1
                      slot 2 -> GPIO 4-7,   PWM 2-3
                      slot 3 -> GPIO 8-11,  PWM 4-5
                      slot 4 -> GPIO 12-15, PWM 6-7
                      slot 5 -> GPIO 16-19, PWM 0-1 (clash with slot 1)
                      slot 6 -> GPIO 20-23, PWM 2-3 (clash with slot 2)
        """
        # One-off look-up-table initialisation.
        self._init_lut()

        # We use the first three GPIOs in the block for the motor outputs.
        first_gpio_num = (slot_num - 1) * 4
        self._motor_pwms = [
            PWM(Pin(n)) for n in range(first_gpio_num, first_gpio_num + 3)
        ]
        self._pwm_banks = [((slot_num - 1) * 2) % 8, (((slot_num - 1) * 2) + 1) % 8]
        self._pwm_duty_addrs = array.array("I")
        self._configure_pwms(pwm_freq)

        # We use the final GPIO in the slot for the sensor input. Configure it
        # with a PIO program to read the PWM signal.
        sensor_pin_num = first_gpio_num + 3
        self._sensor_pin = Pin(sensor_pin_num, Pin.IN, Pin.PULL_UP)
        self._sensor_sm_num = pio_sm_num
        if pio_sm_num < 4:
            pio_base = 0x50200000
        else:
            pio_base = 0x50300000
        self._sensor_fifo_addr = pio_base + 0x20 + (pio_sm_num % 4)
        self._sensor_debug_addr = pio_base + 0x8
        self._sensor_rx_stall_mask = 1 << (pio_sm_num % 4)

        self._sensor_sm = rp2.StateMachine(
            pio_sm_num,
            pio_read_pwm,
            in_base=self._sensor_pin,
            jmp_pin=self._sensor_pin,
        )
        self._sensor_sm.active(1)

        # Calibration values.
        self.angle_offset: int = 0
        self.num_pole_pairs: int = 0

        # Pole angle at which we drive the coils vs the rotor position.
        # 1024 = +90 degrees
        # 3072 = -90 degrees
        self.drive_offset: int = 1024

    @classmethod
    def _init_lut(cls):
        if cls._lut is not None:
            return
        third = 4096 // 3
        # This fraction of the PWM duty cycle should match the switching
        # delay of the motor driver.
        min_on_fraction = 0.016
        scale_factor = 65535 * (1 - min_on_fraction) / 2
        two_pi = 2 * math.pi
        cls._lut = array.array("H")
        for i in range(cls._lut_len):
            sin = math.sin(i * two_pi / cls._lut_len)
            sin_scaled = (sin + 1) * scale_factor
            sin_offset = sin_scaled + min_on_fraction
            cls._lut.append(int(sin_offset))

    def _configure_pwms(self, f):
        # Start with the basic configuration via the library.
        for pwm in self._motor_pwms:
            pwm.freq(f * 2)  # factor of 2 due to phase accurate mode.

        # Now tweak some settings via registers because the library
        # lacks support...
        pwm_base = 0x40050000

        # Look up the PWM_EN register, which allows us to enable/disable
        # both banks together.  This lets us sync the PWMs.
        pwm_en = pwm_base + 0xA0
        set_offset = 0x2000
        clr_offset = 0x3000
        pwm_en_set = pwm_en + set_offset
        pwm_en_clr = pwm_en + clr_offset

        # Calculate mask for our banks.
        pwm_en_mask = 0
        for bank in self._pwm_banks:
            pwm_en_mask |= 1 << bank

        # Disable our PWM banks.  Whiting to the "clear" version of the
        # register clears the bits in the mask.
        machine.mem32[pwm_en_clr] = pwm_en_mask

        for bank in self._pwm_banks:
            # Store off the address for the ISR.
            bank_base = pwm_base + 0x14 * bank
            self._pwm_duty_addrs.append(bank_base + 0x0C)
            # Enable phase-accurate mode
            bank_csr_set = bank_base + set_offset
            machine.mem32[bank_csr_set] = 0x2
            bank_ctr = bank_base + 0x8
            # Reset the counter so that both banks start together.
            machine.mem32[bank_ctr] = 0

        # Start both PWMs together.
        machine.mem32[pwm_en_set] = pwm_en_mask

    def calibrate(self):
        # Calibration: rotate by a quarter phase to capture the rotor.
        # Seems to be a good angle to read off the angle offset.  Maybe
        # because the first duty is at 100%?
        for phase_angle in range(self._lut_len // 4):
            time.sleep_us(100)
            pin_debug.on()
            self._set_duty(phase_angle, 1024)
            pin_debug.off()

        time.sleep_ms(500)  # Let rotor settle.

        angle1 = self._read_pwm_slow()
        print(
            "Calibration interval A = {} = {} deg".format(angle1, angle1 * 360 / 4096)
        )

        for phase_angle in range(self._lut_len // 4, self._lut_len * 9 // 4):
            time.sleep_us(100)
            pin_debug.on()
            self._set_duty(phase_angle, 1024)
            pin_debug.off()

        time.sleep_ms(500)  # Let rotor settle.
        angle2 = self._read_pwm_slow()
        print(
            "Calibration interval B = {} = {} deg".format(angle2, angle2 * 360 / 4096)
        )
        delta = clamp_angle(angle2 - angle1)
        print("Delta = {} = {}".format(delta, delta * 360 / 4096))
        num_pole_pairs = (4096 * 2 * 32 // abs(delta) + 15) // 32
        print("Pole pairs = {}".format(num_pole_pairs))

        # Now we've got the number of poles, we can work out the phase offset.
        pole_angle = 4096 // 4  # Angle we set when we did the measurement
        meas_pole_angle = clamp_angle(angle1 * num_pole_pairs)
        angle_offset = clamp_angle(pole_angle - meas_pole_angle)
        print("Angle offset = {}".format(angle_offset))

        self._set_duty(0, 0)

        self.angle_offset, self.num_pole_pairs = angle_offset, num_pole_pairs

    @micropython.viper
    def _set_duty(self, angle: int, power: int):
        lut_ptr = ptr16(self._lut)
        angle = angle & 0xFFF
        self._motor_pwms[0].duty_u16((lut_ptr[angle] * power) >> 12)
        angle2 = (angle + 1365) & 0xFFF
        self._motor_pwms[1].duty_u16((lut_ptr[angle2] * power) >> 12)
        angle3 = (angle + 2731) & 0xFFF
        self._motor_pwms[2].duty_u16((lut_ptr[angle3] * power) >> 12)

    def _read_pwm_slow(self) -> int:
        """Read next value from the PWM FIFO.

        Detects FIFO overflow and resyncs with the PWM signal
        if needed (this takes 2-3 PWM cycles).

        Returns the wheel angle in 4096ths.
        """
        # Drain the FIFO.
        while self._sensor_sm.rx_fifo() > 0:
            _ = self._sensor_sm.get()

        # Then, block waiting for the next value...
        reads_needed = 1
        while reads_needed > 0:
            packed_value = self._sensor_sm.get()
            reads_needed -= 1

            # Check for an earlier FIFO stall.  If there was a stall, the value
            # we read might be junk.
            if machine.mem32[self._sensor_debug_addr] & self._sensor_rx_stall_mask:
                print("FIFO stall")
                # Do two more reads plus the FIFO length, in case of a long GC stall.
                reads_needed = 2 + self._sensor_sm.rx_fifo()
                machine.mem32[self._sensor_debug_addr + 0x3000] = (
                    self._sensor_rx_stall_mask
                )

        # Data is actually two 16-bit counters packed into
        # a 32-bit word.
        raw_invl = packed_value & 0xFFFF
        raw_high = (packed_value >> 16) & 0xFFFF
        invl = 0xFFFF - raw_invl
        high = 0xFFFF - raw_high
        angle = high * 4119 // invl - 15
        # TODO: <16 means "error"
        if angle < 0:
            angle = 0
        if angle > 4095:
            angle = 0
        return angle

    def start(self):
        self._sensor_sm.irq(
            self.update, trigger=rp2.StateMachine.IRQ_RXNEMPTY, hard=True
        )

    def stop(self):
        self._sensor_sm.irq(None)
        self._set_duty(0, 0)

    @micropython.viper
    def update(self, pio):
        pin_debug.on()
        try:
            fifo = ptr32(self._sensor_fifo_addr)

            # Data is actually two 16-bit counters packed into
            # a 32-bit word.
            packed_value = fifo[0]
            raw_invl = packed_value & 0xFFFF
            raw_high = (packed_value >> 16) & 0xFFFF
            invl = 0xFFFF - raw_invl
            high = 0xFFFF - raw_high
            angle = high * 4119 // invl - 15
            # TODO: <16 means "error"
            if angle < 0:
                angle = 0
            if angle > 4095:
                angle = 0

            pole_angle: int = angle * int(self.num_pole_pairs) + int(self.angle_offset)
            drive_angle: int = pole_angle + int(self.drive_offset)

            power = 1024
            lut_ptr = ptr16(self._lut)
            tap1 = drive_angle & 0xFFF
            tap2 = (drive_angle + 1365) & 0xFFF
            tap3 = (drive_angle + 2731) & 0xFFF
            duty1 = (lut_ptr[tap1] * power) >> 18  # Extra shift of 6 for PWM TOP scale.
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
            duty_addrs = ptr32(self._pwm_duty_addrs)
            ptr32(duty_addrs[0])[0] = duty1 | (duty2 << 16)
            ptr32(duty_addrs[1])[0] = duty3
        except BaseException:
            global stop
            stop = True
        pin_debug.off()


# Clamp and angle into [-2048, 2048)
@micropython.viper
def clamp_angle(angle: int) -> int:
    angle = angle % 4096
    if angle >= 2048:
        angle -= 4096
    elif angle < -2048:
        angle += 4096
    return angle


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
    wrap_target()  # while (1) {             # type: ignore

    mov(x, invert(null))  #   x = 0xffffffff        # type: ignore

    label("loop_high")  #   do {                  # type: ignore
    jmp(x_dec, "xdec1")  #     x--                 # type: ignore
    label("xdec1")  #                         # type: ignore
    nop()  #                         # type: ignore
    jmp(pin, "loop_high")  #   } while (pin high) # type: ignore

    in_(x, 16)  #   ISR = (ISR << 16) | (X & 0xffff) # type: ignore

    label("loop_low")  #   do {                  # type:ignore
    jmp(x_dec, "xdec2")  #     x--                 # type:ignore
    label("xdec2")  #                         # type:ignore
    jmp(pin, "exit_loop_low")  #     if (pin high) break # type:ignore
    jmp("loop_low")  #   } while (1)           # type:ignore
    label("exit_loop_low")  #                         # type:ignore

    in_(x, 16)  #   ISR = (ISR << 16) | (X & 0xffff) # type: ignore

    wrap()  # }                       # type:ignore


@micropython.native
def run():
    try:
        yukon.enable_main_output()

        motor = Motor(0, 6)
        motor.calibrate()

        motor.start()

        drive_offset = 1024
        a_was_pressed = False

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
    finally:
        print("Shutting down")
        motor.stop()


try:
    run()

finally:
    # Put the board back into a safe state, regardless of how the program may have ended
    gc.collect()
