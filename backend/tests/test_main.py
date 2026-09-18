import sys
import os
import json
import tempfile
import pytest
from unittest.mock import MagicMock, patch, AsyncMock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from main import adc_to_degree, pid, load_pid_config, save_pid_config, PID_CONFIG_PATH


# ============================================================
# ADC_TO_DEGREE
# ============================================================


def test_adc_max_gives_zero_degrees():
    assert adc_to_degree(4095) == 0.0


def test_adc_min_gives_180_degrees():
    assert adc_to_degree(0) == 180.0


def test_adc_midpoint():
    result = adc_to_degree(2047)
    assert 89.0 < result < 91.0


def test_adc_zero_clamped():
    assert adc_to_degree(-100) == 180.0


def test_adc_4095_clamped():
    assert adc_to_degree(5000) == 0.0


def test_adc_linear_mapping():
    adc_25 = adc_to_degree(1023)
    assert 134.0 < adc_25 < 136.0

    adc_75 = adc_to_degree(3071)
    assert 44.0 < adc_75 < 46.0


def test_adc_negative_clamped():
    assert adc_to_degree(-1) == 180.0


def test_adc_above_max_clamped():
    assert adc_to_degree(4096) == 0.0


# ============================================================
# PID PERSISTENCE
# ============================================================


def test_save_and_load_pid_config(tmp_path):
    config_path = tmp_path / "pid_config.json"

    with patch("main.PID_CONFIG_PATH", config_path):

        pid[0].kp = 3.5
        pid[0].ki = 0.1
        pid[0].kd = 0.05

        pid[1].kp = 4.0
        pid[1].ki = 0.2
        pid[1].kd = 0.03

        pid[2].kp = 2.5
        pid[2].ki = 0.0
        pid[2].kd = 0.01

        save_pid_config()

        assert config_path.exists()

        pid[0].kp = 1.0
        pid[0].ki = 0.0
        pid[0].kd = 0.0

        load_pid_config()

        assert pid[0].kp == 3.5
        assert pid[0].ki == 0.1
        assert pid[0].kd == 0.05

        assert pid[1].kp == 4.0
        assert pid[1].ki == 0.2
        assert pid[1].kd == 0.03

        assert pid[2].kp == 2.5
        assert pid[2].ki == 0.0
        assert pid[2].kd == 0.01


def test_load_pid_config_no_file(tmp_path):
    config_path = tmp_path / "nonexistent.json"

    with patch("main.PID_CONFIG_PATH", config_path):

        original_kp = pid[0].kp

        load_pid_config()

        assert pid[0].kp == original_kp


def test_load_pid_config_corrupt_file(tmp_path):
    config_path = tmp_path / "corrupt.json"
    config_path.write_text("not valid json {{{")

    with patch("main.PID_CONFIG_PATH", config_path):

        original_kp = pid[0].kp

        load_pid_config()

        assert pid[0].kp == original_kp


def test_save_pid_config_format(tmp_path):
    config_path = tmp_path / "pid_config.json"

    with patch("main.PID_CONFIG_PATH", config_path):

        pid[0].kp = 1.5
        pid[0].ki = 0.0
        pid[0].kd = 0.02

        pid[1].kp = 2.0
        pid[1].ki = 0.1
        pid[1].kd = 0.03

        pid[2].kp = 3.0
        pid[2].ki = 0.2
        pid[2].kd = 0.04

        save_pid_config()

        with open(config_path) as f:
            data = json.load(f)

        assert "motor_0" in data
        assert "motor_1" in data
        assert "motor_2" in data

        assert data["motor_0"]["kp"] == 1.5
        assert data["motor_1"]["ki"] == 0.1
        assert data["motor_2"]["kd"] == 0.04
