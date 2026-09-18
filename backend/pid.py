
import time


class PID:

    def __init__(
        self,
        kp=1.5,
        ki=0.0,
        kd=0.02,
        output_min=-255,
        output_max=255,
        time_fn=None,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None

        self._time_fn = time_fn or time.perf_counter

    def update(self, target, current):

        now = self._time_fn()

        error = target - current

        if self.previous_time is None:
            dt = 0.01
        else:
            dt = now - self.previous_time

            if dt <= 0:
                dt = 0.01

        self.integral += error * dt

        self.integral = max(
            -100,
            min(100, self.integral)
        )

        derivative = (
            error - self.previous_error
        ) / dt


        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        output = max(
            self.output_min,
            min(self.output_max, output)
        )

        self.previous_error = error
        self.previous_time = now

        return int(output)

    def reset(self):

        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None
