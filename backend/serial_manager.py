import logging
import serial
import threading
import time
from collections import deque

logger = logging.getLogger(__name__)


class MovingAverage:

    def __init__(self, window=3):
        self._window = window
        self._buffer = deque(maxlen=window)

    def update(self, value):
        self._buffer.append(value)
        return sum(self._buffer) / len(self._buffer)

    def reset(self):
        self._buffer.clear()


class ESP32Serial:

    RECONNECT_DELAY = 2.0
    MAX_RECONNECT_ATTEMPTS = 10

    def __init__(self, port="COM7", baudrate=115200, filter_window=8):
        self.port = port
        self.baudrate = baudrate

        self.serial = None
        self.running = False

        # Feedback dos potenciômetros
        self.position = [0, 0, 0]

        # Últimos comandos enviados
        self.command = [0, 0, 0]

        # Filtros de média móvel (um por motor)
        self._filters = [
            MovingAverage(filter_window),
            MovingAverage(filter_window),
            MovingAverage(filter_window),
        ]

        self.lock = threading.Lock()

    # =================================================
    # CONECTAR
    # =================================================

    def connect(self):

        self.serial = serial.Serial(
            self.port,
            self.baudrate,
            timeout=0.1
        )

        time.sleep(2)

        self.running = True

        thread = threading.Thread(
            target=self._read_loop,
            daemon=True
        )

        thread.start()

        logger.info("ESP32 conectado em %s", self.port)

    # =================================================
    # TENTAR RECONECTAR
    # =================================================

    def _try_reconnect(self):
        """Tenta reconectar ao ESP32 após perda de conexão."""
        logger.warning(
            "Tentando reconectar ao ESP32 em %s...",
            self.port,
        )

        attempts = 0
        while (
            self.running
            and attempts < self.MAX_RECONNECT_ATTEMPTS
        ):
            attempts += 1
            logger.info(
                "Tentativa %d/%d...",
                attempts,
                self.MAX_RECONNECT_ATTEMPTS,
            )

            try:
                if self.serial is not None:
                    try:
                        self.serial.close()
                    except Exception:
                        pass

                self.serial = serial.Serial(
                    self.port,
                    self.baudrate,
                    timeout=0.1,
                )

                time.sleep(2)

                logger.info(
                    "Reconectado ao ESP32 em %s",
                    self.port,
                )
                return True

            except serial.SerialException as exc:
                logger.warning(
                    "Falha ao reconectar: %s", exc
                )
                time.sleep(self.RECONNECT_DELAY)

        logger.error(
            "Não foi possível reconectar após %d tentativas.",
            self.MAX_RECONNECT_ATTEMPTS,
        )
        return False

    # =================================================
    # LEITURA SERIAL
    # =================================================

    def _read_loop(self):

        while self.running:

            try:

                if self.serial is not None and self.serial.in_waiting:

                    line = (
                        self.serial
                        .readline()
                        .decode("utf-8", errors="ignore")
                        .strip()
                    )

                    self._process_data(line)

                else:

                    time.sleep(0.001)

            except serial.SerialException as exc:
                logger.error("Erro na comunicação serial: %s", exc)
                self.running = False

                if self._try_reconnect():
                    self.running = True
                else:
                    break

            except Exception as exc:
                logger.error("Erro inesperado na leitura serial: %s", exc)
                self.running = False
                break

    # =================================================
    # PROCESSAR FEEDBACK
    #
    # ESP32 envia:
    #
    # FB,2048,1800,2500
    # =================================================

    def _process_data(self, line):

        if not line.startswith("FB,"):
            return

        data = line.split(",")

        if len(data) != 4:
            return

        try:

            raw = [
                int(data[1]),
                int(data[2]),
                int(data[3])
            ]

            filtered = [
                int(self._filters[i].update(raw[i]))
                for i in range(3)
            ]

            with self.lock:

                self.position = filtered

        except ValueError:
            pass

    # =================================================
    # ENVIAR COMANDO
    #
    # -255 ... +255
    # =================================================

    def send_command(self, m1, m2, m3):

        if self.serial is None:
            return

        commands = [
            max(-255, min(255, int(m1))),
            max(-255, min(255, int(m2))),
            max(-255, min(255, int(m3)))
        ]

        command = (
            f"CMD,"
            f"{commands[0]},"
            f"{commands[1]},"
            f"{commands[2]}\n"
        )

        try:

            self.serial.write(
                command.encode("utf-8")
            )

            with self.lock:

                self.command = commands

        except serial.SerialException as exc:
            logger.error("Erro ao enviar comando: %s", exc)

        except Exception as exc:
            logger.error("Erro inesperado ao enviar comando: %s", exc)

    # =================================================
    # OBTER FEEDBACK
    # =================================================

    def get_feedback(self):

        with self.lock:

            return {
                "position": self.position.copy(),
                "command": self.command.copy()
            }

    # =================================================
    # PARAR MOTORES
    # =================================================

    def stop(self):

        if self.serial is not None:

            try:

                self.serial.write(
                    b"CMD,0,0,0\n"
                )

                with self.lock:

                    self.command = [0, 0, 0]

            except serial.SerialException as exc:
                logger.error("Erro ao parar motores: %s", exc)

            except Exception as exc:
                logger.error("Erro inesperado ao parar motores: %s", exc)

    # =================================================
    # FECHAR
    # =================================================

    def close(self):

        self.running = False

        self.stop()

        time.sleep(0.1)

        if self.serial is not None:

            try:
                self.serial.close()
            except serial.SerialException as exc:
                logger.error("Erro ao fechar serial: %s", exc)

        self.serial = None

        logger.info("ESP32 desconectado.")
