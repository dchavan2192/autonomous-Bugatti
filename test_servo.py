import os
import time

class Servo:
    CHIP = "/sys/class/pwm/pwmchip0"
    CHANNEL = 0
    PERIOD_NS = 20_000_000

    def __init__(self):
        pwm_dir = f"{self.CHIP}/pwm{self.CHANNEL}"
        if not os.path.exists(pwm_dir):
            self._write(f"{self.CHIP}/export", self.CHANNEL)
        self._write(f"{pwm_dir}/period", self.PERIOD_NS)
        self._write(f"{pwm_dir}/enable", 1)
        self.pwm_dir = pwm_dir

    def _write(self, path, value):
        with open(path, "w") as f:
            f.write(str(value))

    def set_angle(self, deg):
        deg = max(-90, min(90, deg))
        pulse_us = 1500 + (deg / 90) * 500
        self._write(f"{self.pwm_dir}/duty_cycle", int(pulse_us * 1000))

    def stop(self):
        self._write(f"{self.pwm_dir}/enable", 0)

servo = Servo()
print("Center")
servo.set_angle(0)
time.sleep(1)
print("Left")
servo.set_angle(-90)
time.sleep(1)
print("Right")
servo.set_angle(90)
time.sleep(1)
print("Center")
servo.set_angle(0)
