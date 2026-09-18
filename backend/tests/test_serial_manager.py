import sys
import os
import threading
import pytest
from unittest.mock import MagicMock, patch, PropertyMock

import serial

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from serial_manager import ESP32Serial, MovingAverage


# ============================================================
# MOVING AVERAGE
# ============================================================


def test_moving_average_single_value():
    ma = MovingAverage(window=5)
    assert ma.update(100) == 100.0


def test_moving_average_constant_values():
    ma = MovingAverage(window=5)
    for _ in range(10):
        result = ma.update(200)
    assert result == 200.0


def test_moving_average_smoothing():
    ma = MovingAverage(window=4)
    values = [100, 110, 100, 110]
    results = [ma.update(v) for v in values]

    assert results[0] == 100.0
    assert results[1] == 105.0
    assert results[2] == pytest.approx(103.33, rel=1e-2)
    assert results[3] == 105.0


def test_moving_average_window_limit():
    ma = MovingAverage(window=3)
    ma.update(10)
    ma.update(20)
    ma.update(30)
    result = ma.update(40)

    assert result == 30.0


def test_moving_average_reset():
    ma = MovingAverage(window=5)
    ma.update(100)
    ma.update(200)
    ma.reset()
    result = ma.update(50)

    assert result == 50.0


def test_moving_average_partial_window():
    ma = MovingAverage(window=10)
    ma.update(100)
    ma.update(200)

    assert ma.update(300) == 200.0


# ============================================================
# INIT
# ============================================================


def test_default_params():
    esp = ESP32Serial()
    assert esp.port == "COM7"
    assert esp.baudrate == 115200
    assert esp.serial is None
    assert esp.running is False


def test_custom_params():
    esp = ESP32Serial(port="COM3", baudrate=9600)
    assert esp.port == "COM3"
    assert esp.baudrate == 9600


def test_initial_state():
    esp = ESP32Serial()
    assert esp.position == [0, 0, 0]
    assert esp.command == [0, 0, 0]


# ============================================================
# GET_FEEDBACK
# ============================================================


def test_get_feedback_returns_copies():
    esp = ESP32Serial()
    fb1 = esp.get_feedback()
    fb2 = esp.get_feedback()

    assert fb1 is not fb2
    assert fb1["position"] == fb2["position"]
    assert fb1["command"] == fb2["command"]


def test_get_feedback_thread_safe():
    esp = ESP32Serial()
    results = []

    def reader():
        for _ in range(100):
            results.append(esp.get_feedback())

    threads = [threading.Thread(target=reader) for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert len(results) == 400


# ============================================================
# SEND_COMMAND
# ============================================================


@patch("serial_manager.serial.Serial")
def test_send_command_no_serial(mock_serial_class):
    esp = ESP32Serial()
    esp.send_command(100, -50, 200)

    assert esp.command == [0, 0, 0]


@patch("serial_manager.serial.Serial")
def test_send_command_clamps_values(mock_serial_class):
    esp = ESP32Serial()
    mock_serial = MagicMock()
    esp.serial = mock_serial

    esp.send_command(300, -300, 0)

    mock_serial.write.assert_called_once()
    sent = mock_serial.write.call_args[0][0]
    assert sent == b"CMD,255,-255,0\n"


@patch("serial_manager.serial.Serial")
def test_send_command_stores_commands(mock_serial_class):
    esp = ESP32Serial()
    esp.serial = MagicMock()

    esp.send_command(100, -50, 200)

    assert esp.command == [100, -50, 200]


@patch("serial_manager.serial.Serial")
def test_send_command_converts_to_int(mock_serial_class):
    esp = ESP32Serial()
    esp.serial = MagicMock()

    esp.send_command(100.7, -50.3, 200.9)

    assert esp.command == [100, -50, 200]


# ============================================================
# STOP
# ============================================================


@patch("serial_manager.serial.Serial")
def test_stop_sends_zero_command(mock_serial_class):
    esp = ESP32Serial()
    esp.serial = MagicMock()

    esp.stop()

    esp.serial.write.assert_called_once_with(b"CMD,0,0,0\n")
    assert esp.command == [0, 0, 0]


@patch("serial_manager.serial.Serial")
def test_stop_no_serial(mock_serial_class):
    esp = ESP32Serial()
    esp.stop()

    assert esp.command == [0, 0, 0]


# ============================================================
# PROCESS_DATA
# ============================================================


def test_process_data_valid_feedback():
    esp = ESP32Serial()
    esp._process_data("FB,1024,2048,3072")

    assert esp.position == [1024, 2048, 3072]


def test_process_data_ignores_non_fb():
    esp = ESP32Serial()
    esp._process_data("CMD,100,200,300")

    assert esp.position == [0, 0, 0]


def test_process_data_wrong_length():
    esp = ESP32Serial()
    esp._process_data("FB,1024,2048")

    assert esp.position == [0, 0, 0]


def test_process_data_invalid_int():
    esp = ESP32Serial()
    esp._process_data("FB,abc,2048,3072")

    assert esp.position == [0, 0, 0]


def test_process_data_empty_line():
    esp = ESP32Serial()
    esp._process_data("")

    assert esp.position == [0, 0, 0]


# ============================================================
# CLOSE
# ============================================================


@patch("serial_manager.serial.Serial")
def test_close(mock_serial_class):
    esp = ESP32Serial()
    mock_serial = MagicMock()
    esp.serial = mock_serial
    esp.running = True

    esp.close()

    assert esp.serial is None
    assert esp.running is False
    mock_serial.close.assert_called_once()


@patch("serial_manager.serial.Serial")
def test_close_with_exception(mock_serial_class):
    esp = ESP32Serial()
    mock_serial = MagicMock()
    mock_serial.close.side_effect = serial.SerialException("already closed")
    esp.serial = mock_serial
    esp.running = True

    esp.close()

    assert esp.serial is None
    assert esp.running is False
