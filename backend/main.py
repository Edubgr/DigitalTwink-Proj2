import logging
from contextlib import asynccontextmanager
from pathlib import Path
import asyncio
import json
import os
import time
import urllib.error
import urllib.request

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse

from serial_manager import ESP32Serial
from pid import PID

logger = logging.getLogger(__name__)

# ============================================================
# CAMINHOS
# ============================================================

BASE_DIR = Path(__file__).resolve().parent
FRONTEND_DIR = BASE_DIR.parent / "frontend"
URDF_MESH_DIR = BASE_DIR.parent / "problemas2_urdf_description" / "meshes"

PID_CONFIG_PATH = BASE_DIR / "pid_config.json"



# ============================================================
# ESP32
# ============================================================

esp32 = ESP32Serial(
    port="COM7",
    baudrate=115200
)


# ============================================================
# PID
#
# Existe UM conjunto de ganhos:
#
#     Kp
#     Ki
#     Kd
#
# Porém existem 3 objetos PID.
#
# Isso significa que:
#
# Motor 1 -> estado próprio
# Motor 2 -> estado próprio
# Motor 3 -> estado próprio
#
# Mas todos usam os mesmos ganhos.
# ============================================================

pid = [
    PID(
        kp=2.0,
        ki=0.0,
        kd=0.02
    ),

    PID(
        kp=2.0,
        ki=0.0,
        kd=0.02
    ),

    PID(
        kp=2.0,
        ki=0.0,
        kd=0.02
    )
]


# ============================================================
# ALVOS
# ============================================================

targets = [
    0.0,
    0.0,
    0.0
]


# ============================================================
# ESTADO DOS MOTORES
# ============================================================

motor_stopped = [
    True,
    True,
    True
]


# ============================================================
# CONFIGURAÇÃO DO CONTROLE
# ============================================================

DEADBAND = 1.0

RELEASE_ERROR = 1.5

PWM_MIN = 60

PWM_MAX = 255

control_enabled = False


# ============================================================
# TESTE
# ============================================================

test_running = False

test_data = []

test_start_time = None

test_profile_active = False

test_profile_motor = 0

websocket_clients = set()

_background_tasks: list[asyncio.Task] = []


# ============================================================
# CONVERSÃO ADC -> GRAUS
# ============================================================

def adc_to_degree(adc):

    """
    Mapeamento provisório:

    ADC 4095 -> 0°
    ADC 0    -> 180°

    A calibração real pode ser alterada depois.
    """

    adc = max(
        0,
        min(4095, adc)
    )

    return 180.0 - (
        adc / 4095.0
    ) * 180.0


# ============================================================
# RESET DOS PID
# ============================================================

def reset_pid():

    for controller in pid:

        controller.reset()


# ============================================================
# PERSISTÊNCIA DO PID
# ============================================================

def load_pid_config():
    """Carrega configurações PID salvas em disco."""

    if not PID_CONFIG_PATH.exists():
        return

    try:

        with open(PID_CONFIG_PATH, "r") as f:
            data = json.load(f)

        for i in range(3):
            key = f"motor_{i}"
            if key in data:
                pid[i].kp = float(data[key].get("kp", pid[i].kp))
                pid[i].ki = float(data[key].get("ki", pid[i].ki))
                pid[i].kd = float(data[key].get("kd", pid[i].kd))

        logger.info("PID config carregada de %s", PID_CONFIG_PATH)

    except Exception as exc:
        logger.error("Erro ao carregar PID config: %s", exc)


def save_pid_config():
    """Salva configurações PID atuais em disco."""

    data = {}
    for i in range(3):
        data[f"motor_{i}"] = {
            "kp": pid[i].kp,
            "ki": pid[i].ki,
            "kd": pid[i].kd,
        }

    try:

        with open(PID_CONFIG_PATH, "w") as f:
            json.dump(data, f, indent=2)

        logger.info("PID config salva em %s", PID_CONFIG_PATH)

    except Exception as exc:
        logger.error("Erro ao salvar PID config: %s", exc)


# ============================================================
# STOP DOS MOTORES
# ============================================================

def stop_motors():

    global control_enabled

    control_enabled = False

    esp32.stop()

    reset_pid()

    for i in range(3):

        motor_stopped[i] = True


# ============================================================
# LIFESPAN
# ============================================================

@asynccontextmanager
async def lifespan(app):

    # ========================================================
    # STARTUP
    # ========================================================

    try:
        esp32.connect()
        load_pid_config()
        logger.info("======================================")
        logger.info(" CONTROL ARM DIGITAL TWIN")
        logger.info("======================================")
        logger.info("ESP32 conectado")
        logger.info("Porta: COM7")
        logger.info("Baudrate: 115200")
        logger.info("PID:")
        logger.info("Kp = %s", pid[0].kp)
        logger.info("Ki = %s", pid[0].ki)
        logger.info("Kd = %s", pid[0].kd)
        logger.info("======================================")
    except Exception as e:
        logger.error("ERRO AO CONECTAR ESP32: %s", e)

    task_control = asyncio.create_task(control_loop())
    task_broadcast = asyncio.create_task(broadcast_feedback())
    _background_tasks.extend([task_control, task_broadcast])

    yield

    # ========================================================
    # SHUTDOWN
    # ========================================================

    for task in _background_tasks:
        task.cancel()

    await asyncio.gather(
        *_background_tasks,
        return_exceptions=True,
    )
    _background_tasks.clear()

    stop_motors()
    esp32.close()


# ============================================================
# FASTAPI
# ============================================================

app = FastAPI(lifespan=lifespan)


# ============================================================
# PÁGINA PRINCIPAL
# ============================================================

@app.get("/")
async def index():

    return FileResponse(
        FRONTEND_DIR / "index.html"
    )


@app.get("/urdf")
async def urdf_viewer():

    return FileResponse(
        FRONTEND_DIR / "urdf.html"
    )


@app.get("/robot")
async def robot_viewer():

    return FileResponse(
        FRONTEND_DIR / "urdf.html"
    )


@app.get("/assets/meshes/{mesh_name}")
async def mesh_asset(mesh_name: str):

    mesh_path = URDF_MESH_DIR / mesh_name

    if not mesh_path.exists():

        raise HTTPException(
            status_code=404,
            detail="Mesh not found"
        )

    return FileResponse(
        mesh_path,
        media_type="application/sla"
    )


# ============================================================
# MOVEIT
#
# O move_group roda no container (docker compose up) e expõe a
# ponte HTTP em 8081. O backend só repassa: o navegador não fala
# com o ROS diretamente.
#
#     interface  ->  /api/moveit/plan  ->  moveit_bridge  ->  move_group
#
# Tudo aqui trabalha em RADIANOS no frame do URDF. A conversão
# para os graus do ESP32 é outro assunto (ver adc_to_degree) e
# continua pendente de calibração.
# ============================================================

MOVEIT_URL = os.environ.get(
    "MOVEIT_URL",
    "http://localhost:8081"
)


def moveit_call(path, payload=None, timeout=30.0):

    url = MOVEIT_URL + path

    if payload is None:
        request = urllib.request.Request(url, method="GET")
    else:
        request = urllib.request.Request(
            url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST"
        )

    try:

        with urllib.request.urlopen(request, timeout=timeout) as response:

            return json.loads(
                response.read().decode("utf-8")
            )

    except urllib.error.URLError as exc:

        return {
            "ok": False,
            "error": f"MoveIt indisponível em {MOVEIT_URL} ({exc.reason}). "
                     f"Suba o container com: docker compose up -d"
        }

    except Exception as exc:

        return {
            "ok": False,
            "error": str(exc)
        }


@app.get("/api/moveit/health")
async def moveit_health():

    return await asyncio.to_thread(
        moveit_call,
        "/health",
        None,
        5.0
    )


@app.post("/api/moveit/plan")
async def moveit_plan(request: Request):

    body = await request.json()

    return await asyncio.to_thread(
        moveit_call,
        "/plan",
        body,
        30.0
    )


@app.post("/api/moveit/state")
async def moveit_state(request: Request):

    body = await request.json()

    return await asyncio.to_thread(
        moveit_call,
        "/state",
        body,
        5.0
    )


# ============================================================
# FEEDBACK PARA FRONTEND
# ============================================================

def build_feedback_payload():

    feedback = esp32.get_feedback()

    positions = [
        adc_to_degree(feedback["position"][0]),
        adc_to_degree(feedback["position"][1]),
        adc_to_degree(feedback["position"][2])
    ]

    errors = [
        targets[i] - positions[i]
        for i in range(3)
    ]

    return {
        "type": "feedback",

        # Sem ESP32 na porta, o ADC lê 0 e adc_to_degree devolve 180°
        # para as três juntas. Isso parecia leitura real na interface,
        # então o estado da serial vai junto.
        "robot": esp32.serial is not None,

        "position": positions,
        "target": targets.copy(),
        "error": errors,
        "pwm": feedback["command"],
        "pid": {
            f"motor_{i}": {
                "kp": pid[i].kp,
                "ki": pid[i].ki,
                "kd": pid[i].kd,
            }
            for i in range(3)
        }
    }


async def broadcast_feedback():

    while True:

        if websocket_clients:

            payload = json.dumps(build_feedback_payload())

            dead_clients = []

            for websocket in list(websocket_clients):

                try:

                    if websocket.application_state.name == "CONNECTED":

                        await websocket.send_text(payload)

                except Exception:

                    dead_clients.append(websocket)

            for websocket in dead_clients:

                websocket_clients.discard(websocket)

        await asyncio.sleep(0.05)


# ============================================================
# LOOP DE CONTROLE
# ============================================================

async def control_loop():

    global test_running
    global test_start_time

    while True:

        # ====================================================
        # LEITURA DO ESP32
        # ====================================================

        feedback = esp32.get_feedback()

        adc_positions = feedback["position"]


        # ====================================================
        # ADC -> GRAUS
        # ====================================================

        positions = [

            adc_to_degree(
                adc_positions[0]
            ),

            adc_to_degree(
                adc_positions[1]
            ),

            adc_to_degree(
                adc_positions[2]
            )

        ]


        # ====================================================
        # COMANDOS DOS MOTORES
        # ====================================================

        commands = [
            0,
            0,
            0
        ]


        # ====================================================
        # CONTROLE PID
        # ====================================================

        if control_enabled:

            for i in range(3):

                # --------------------------------------------
                # ERRO DO MOTOR
                # --------------------------------------------

                error = (
                    targets[i]
                    - positions[i]
                )


                # =================================================
                # MOTOR ESTÁ PARADO
                # =================================================

                if motor_stopped[i]:

                    # ---------------------------------------------
                    # Continua parado enquanto estiver próximo
                    # ---------------------------------------------

                    if abs(error) <= RELEASE_ERROR:

                        commands[i] = 0

                    # ---------------------------------------------
                    # Sai da condição de parado
                    # ---------------------------------------------

                    else:

                        motor_stopped[i] = False


                # =================================================
                # MOTOR ESTÁ RODANDO
                # =================================================

                if not motor_stopped[i]:

                    # ---------------------------------------------
                    # Chegou no alvo
                    # ---------------------------------------------

                    if abs(error) <= DEADBAND:

                        commands[i] = 0

                        motor_stopped[i] = True

                        # Reseta SOMENTE o PID daquele motor
                        pid[i].reset()


                    # ---------------------------------------------
                    # Continua controlando
                    # ---------------------------------------------

                    else:

                        # =========================================
                        # PID INDIVIDUAL
                        #
                        # Os ganhos são iguais para todos,
                        # mas o estado interno é independente.
                        # =========================================

                        output = pid[i].update(
                            targets[i],
                            positions[i]
                        )


                        # -----------------------------------------
                        # LIMITAÇÃO DO PWM
                        # -----------------------------------------

                        output = max(
                            -PWM_MAX,
                            min(
                                PWM_MAX,
                                output
                            )
                        )


                        # -----------------------------------------
                        # PWM MÍNIMO
                        #
                        # O motor não gira abaixo de ~60.
                        # -----------------------------------------

                        if output != 0:

                            if abs(output) < PWM_MIN:

                                if output > 0:

                                    output = PWM_MIN

                                else:

                                    output = -PWM_MIN


                        commands[i] = output


        # ====================================================
        # CONTROLE DESABILITADO
        # ====================================================

        else:

            commands = [
                0,
                0,
                0
            ]


        # ====================================================
        # ENVIA COMANDOS PARA ESP32
        # ====================================================

        esp32.send_command(
            commands[0],
            commands[1],
            commands[2]
        )


        # ====================================================
        # REGISTRO DO TESTE
        # ====================================================

        if test_running:

            if test_start_time is None:

                test_start_time = (
                    time.perf_counter()
                )


            elapsed = (
                time.perf_counter()
                - test_start_time
            )


            test_data.append({

                "time": elapsed,

                "position": positions.copy(),

                "target": targets.copy(),

                "error": [

                    targets[i]
                    - positions[i]

                    for i in range(3)

                ],

                "pwm": commands.copy()

            })


        # ====================================================
        # LOOP DE 10 ms
        # ====================================================

        await asyncio.sleep(0.01)



# ============================================================
# WEBSOCKET
# ============================================================

@app.websocket("/ws")
async def websocket_endpoint(
    websocket: WebSocket
):

    global control_enabled
    global test_running
    global test_data
    global test_start_time
    global test_profile_active
    global test_profile_motor


    # ========================================================
    # ACEITA CONEXÃO
    # ========================================================

    await websocket.accept()

    websocket_clients.add(websocket)

    print(
        "WebSocket conectado."
    )


    try:

        while True:

            # =================================================
            # RECEBE MENSAGEM
            # =================================================

            message = await websocket.receive_text()


            try:

                data = json.loads(
                    message
                )

            except json.JSONDecodeError:

                continue


            message_type = data.get(
                "type"
            )


            # =================================================
            # ALTERAR TARGET
            # =================================================

            if message_type == "set_target":

                motor = int(
                    data.get(
                        "motor",
                        0
                    )
                )

                target = float(
                    data.get(
                        "target",
                        0
                    )
                )


                if test_profile_active and motor == test_profile_motor:

                    continue


                if 0 <= motor < 3:

                    targets[motor] = max(
                        0.0,
                        min(
                            180.0,
                            target
                        )
                    )


            elif message_type == "set_targets":

                new_targets = data.get("targets", [])

                if isinstance(new_targets, list) and len(new_targets) == 3:

                    for i in range(3):

                        if test_profile_active and i == test_profile_motor:
                            continue

                        targets[i] = max(
                            0.0,
                            min(
                                180.0,
                                float(new_targets[i])
                            )
                        )


            elif message_type == "start_test_profile":

                motor = int(
                    data.get(
                        "motor",
                        0
                    )
                )

                if 0 <= motor < 3:

                    test_profile_active = True

                    test_profile_motor = motor

                print(
                    f"Perfil de teste ativo no motor {motor}."
                )


            elif message_type == "apply_test_target":

                motor = int(
                    data.get(
                        "motor",
                        0
                    )
                )

                target = float(
                    data.get(
                        "target",
                        0
                    )
                )

                if test_profile_active and motor == test_profile_motor:

                    if 0 <= motor < 3:

                        targets[motor] = max(
                            0.0,
                            min(
                                180.0,
                                target
                            )
                        )


            elif message_type == "stop_test_profile":

                test_profile_active = False

                print(
                    "Perfil de teste parado."
                )


            # =================================================
            # ALTERAR PID
            #
            # Aceita "motor": "all", "0", "1", "2"
            # =================================================

            elif message_type == "set_pid":

                kp = float(
                    data.get(
                        "kp",
                        pid[0].kp
                    )
                )

                ki = float(
                    data.get(
                        "ki",
                        pid[0].ki
                    )
                )

                kd = float(
                    data.get(
                        "kd",
                        pid[0].kd
                    )
                )

                motor_target = data.get("motor", "all")


                if motor_target == "all":

                    for controller in pid:
                        controller.kp = kp
                        controller.ki = ki
                        controller.kd = kd
                        controller.reset()

                    logger.info(
                        "PID GLOBAL: Kp=%.3f Ki=%.3f Kd=%.3f",
                        kp, ki, kd,
                    )

                elif motor_target in ("0", "1", "2"):

                    idx = int(motor_target)
                    pid[idx].kp = kp
                    pid[idx].ki = ki
                    pid[idx].kd = kd
                    pid[idx].reset()

                    logger.info(
                        "PID MOTOR %d: Kp=%.3f Ki=%.3f Kd=%.3f",
                        idx, kp, ki, kd,
                    )

                save_pid_config()

                all_pids = {
                    f"motor_{i}": {
                        "kp": pid[i].kp,
                        "ki": pid[i].ki,
                        "kd": pid[i].kd,
                    }
                    for i in range(3)
                }

                await websocket.send_text(
                    json.dumps({
                        "type": "pid_ack",
                        "motor": motor_target,
                        "kp": kp,
                        "ki": ki,
                        "kd": kd,
                        "all": all_pids,
                    })
                )


            # =================================================
            # OBTER PID
            # =================================================

            elif message_type == "get_pid":

                all_pids = {
                    f"motor_{i}": {
                        "kp": pid[i].kp,
                        "ki": pid[i].ki,
                        "kd": pid[i].kd,
                    }
                    for i in range(3)
                }

                await websocket.send_text(
                    json.dumps({
                        "type": "pid_state",
                        "all": all_pids,
                    })
                )


            # =================================================
            # HABILITAR PID
            # =================================================

            elif message_type == "enable_pid":

                control_enabled = True

                # ---------------------------------------------
                # Cada motor começa com seu próprio estado
                # zerado.
                # ---------------------------------------------

                reset_pid()


                for i in range(3):

                    motor_stopped[i] = False


                print(
                    "PID habilitado."
                )


            # =================================================
            # STOP
            # =================================================

            elif message_type == "stop":

                stop_motors()

                print(
                    "STOP."
                )


            # =================================================
            # INICIAR TESTE
            # =================================================

            elif message_type == "start_test":

                test_data = []

                test_start_time = (
                    time.perf_counter()
                )

                test_running = True

                await websocket.send_text(
                    json.dumps({
                        "type": "test_status",
                        "status": "recording",
                        "running": True
                    })
                )

                print(
                    "Teste iniciado."
                )


            # =================================================
            # FINALIZAR TESTE
            # =================================================

            elif message_type == "stop_test":

                test_running = False

                await websocket.send_text(
                    json.dumps({
                        "type": "test_status",
                        "status": "ready",
                        "running": False
                    })
                )

                print(
                    "Teste finalizado."
                )


            # =================================================
            # SOLICITAR DADOS DO TESTE
            # =================================================

            elif message_type == "get_test_data":

                await websocket.send_text(

                    json.dumps({

                        "type":
                            "test_data",

                        "data":
                            test_data

                    })

                )


            # =================================================
            # ENVIA FEEDBACK PARA O FRONTEND
            #
            # Mesmo payload do broadcast periódico.
            # =================================================

            await websocket.send_text(
                json.dumps(build_feedback_payload())
            )


    # ========================================================
    # WEBSOCKET DESCONECTADO
    # ========================================================

    except WebSocketDisconnect:

        websocket_clients.discard(websocket)

        print(
            "WebSocket desconectado."
        )

        stop_motors()


    # ========================================================
    # OUTRO ERRO
    # ========================================================

    except Exception as e:

        websocket_clients.discard(websocket)

        print(
            "Erro no WebSocket:",
            e
        )

        stop_motors()