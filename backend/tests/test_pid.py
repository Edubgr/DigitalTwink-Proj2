import sys
import os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pid import PID


class FakeClock:
    """Relógio fictício para testes determinísticos."""

    def __init__(self, start=0.0):
        self._now = start

    def __call__(self):
        return self._now

    def advance(self, dt):
        self._now += dt


# ============================================================
# INIT
# ============================================================


def test_default_gains():
    pid = PID()
    assert pid.kp == 1.5
    assert pid.ki == 0.0
    assert pid.kd == 0.02


def test_custom_gains():
    pid = PID(kp=3.0, ki=0.5, kd=0.1)
    assert pid.kp == 3.0
    assert pid.ki == 0.5
    assert pid.kd == 0.1


def test_initial_state():
    pid = PID()
    assert pid.integral == 0.0
    assert pid.previous_error == 0.0
    assert pid.previous_time is None


# ============================================================
# UPDATE
# ============================================================


def test_first_update_uses_default_dt():
    clock = FakeClock(0.0)
    pid = PID(kp=1.0, ki=0.0, kd=0.0, time_fn=clock)

    output = pid.update(target=10.0, current=0.0)

    assert output == 10


def test_proportional_only():
    clock = FakeClock(0.0)
    pid = PID(kp=2.0, ki=0.0, kd=0.0, time_fn=clock)

    clock.advance(0.01)
    out1 = pid.update(target=50.0, current=0.0)

    assert out1 == 100


def test_integral_accumulates():
    clock = FakeClock(0.0)
    pid = PID(kp=0.0, ki=1.0, kd=0.0, time_fn=clock)

    clock.advance(0.01)
    pid.update(target=10.0, current=0.0)

    clock.advance(0.01)
    pid.update(target=10.0, current=0.0)

    assert pid.integral == pytest.approx(0.2, rel=1e-6)


def test_derivative_on_constant_error():
    clock = FakeClock(0.0)
    pid = PID(kp=0.0, ki=0.0, kd=1.0, derivative_filter=1.0, time_fn=clock)

    clock.advance(0.01)
    pid.update(target=10.0, current=0.0)

    clock.advance(0.01)
    output = pid.update(target=10.0, current=0.0)

    assert output == 0


def test_derivative_on_changing_error():
    clock = FakeClock(0.0)
    pid = PID(kp=0.0, ki=0.0, kd=1.0, output_min=-1000, derivative_filter=1.0, time_fn=clock)

    clock.advance(0.01)
    pid.update(target=10.0, current=0.0)

    clock.advance(0.01)
    output = pid.update(target=10.0, current=5.0)

    assert output == -500


def test_derivative_filter_smooths():
    clock = FakeClock(0.0)
    pid = PID(
        kp=0.0, ki=0.0, kd=1.0,
        derivative_filter=0.2,
        output_min=-1000, time_fn=clock,
    )

    clock.advance(0.01)
    pid.update(target=10.0, current=0.0)

    clock.advance(0.01)
    out1 = pid.update(target=10.0, current=5.0)

    clock.advance(0.01)
    out2 = pid.update(target=10.0, current=5.0)

    assert abs(out2) < abs(out1)


# ============================================================
# CLAMPING
# ============================================================


def test_output_clamped_to_max():
    clock = FakeClock(0.0)
    pid = PID(kp=100.0, ki=0.0, kd=0.0, output_max=50, time_fn=clock)

    clock.advance(0.01)
    output = pid.update(target=100.0, current=0.0)

    assert output == 50


def test_output_clamped_to_min():
    clock = FakeClock(0.0)
    pid = PID(kp=100.0, ki=0.0, kd=0.0, output_min=-50, time_fn=clock)

    clock.advance(0.01)
    output = pid.update(target=-100.0, current=0.0)

    assert output == -50


def test_integral_anti_windup():
    clock = FakeClock(0.0)
    pid = PID(kp=0.0, ki=1000.0, kd=0.0, time_fn=clock)

    for _ in range(100):
        clock.advance(0.01)
        pid.update(target=100.0, current=0.0)

    assert -100 <= pid.integral <= 100


# ============================================================
# DT EDGE CASES
# ============================================================


def test_dt_zero_uses_fallback():
    clock = FakeClock(0.0)
    pid = PID(kp=0.0, ki=0.0, kd=1.0, time_fn=clock)

    clock.advance(0.01)
    pid.update(target=10.0, current=0.0)

    clock.advance(0.0)
    output = pid.update(target=10.0, current=5.0)

    assert output != 0


def test_negative_dt_uses_fallback():
    clock = FakeClock(10.0)
    pid = PID(kp=0.0, ki=0.0, kd=1.0, time_fn=clock)

    pid.update(target=10.0, current=0.0)

    clock._now = 5.0
    output = pid.update(target=10.0, current=5.0)

    assert output != 0


# ============================================================
# RESET
# ============================================================


def test_reset():
    clock = FakeClock(0.0)
    pid = PID(kp=1.0, ki=1.0, kd=1.0, time_fn=clock)

    clock.advance(0.01)
    pid.update(target=10.0, current=0.0)

    pid.reset()

    assert pid.integral == 0.0
    assert pid.previous_error == 0.0
    assert pid.previous_time is None


def test_reset_after_accumulation():
    clock = FakeClock(0.0)
    pid = PID(kp=0.0, ki=1.0, kd=0.0, time_fn=clock)

    for _ in range(10):
        clock.advance(0.01)
        pid.update(target=10.0, current=0.0)

    assert pid.integral > 0.0

    pid.reset()

    assert pid.integral == 0.0
    assert pid.previous_time is None


# ============================================================
# RETORNO INTEIRO
# ============================================================


def test_returns_int():
    clock = FakeClock(0.0)
    pid = PID(kp=1.5, ki=0.0, kd=0.0, time_fn=clock)

    clock.advance(0.01)
    output = pid.update(target=7.0, current=0.0)

    assert isinstance(output, int)
