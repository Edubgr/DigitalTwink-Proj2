"""
Ponte HTTP <-> MoveIt.

Este nó é a única coisa que a interface web precisa conhecer do ROS.
Ele expõe um HTTP simples e, por baixo, conversa com o move_group
pelos serviços padrão do MoveIt:

    POST /plan    ->  /compute_ik  +  /plan_kinematic_path
    POST /state   ->  atualiza /joint_states (o gêmeo virtual)
    GET  /health  ->  diagnóstico

O estado do gêmeo é empurrado pela interface (POST /state) e
republicado em /joint_states, então o RViz mostra exatamente o mesmo
braço que o three.js. Quando o robô real entrar no caminho, é esta
fonte de /joint_states que muda — nada acima disso precisa mudar.
"""

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from moveit_msgs.msg import Constraints, JointConstraint, PositionIKRequest
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from sensor_msgs.msg import JointState

JOINT_NAMES = ["revbase", "revlink1", "relink2"]

# Limites do URDF: as tres juntas cobrem o circulo inteiro. Quem
# restringe de fato e a janela operacional definida na interface, que
# chega em cada pedido no campo "limits".
JOINT_LIMITS = {
    "revbase": (-3.141593, 3.141593),
    "revlink1": (-3.141593, 3.141593),
    "relink2": (-3.141593, 3.141593),
}

SERVICE_WAIT = 30.0
CALL_TIMEOUT = 10.0


class MoveItBridge(Node):

    def __init__(self):
        super().__init__("moveit_bridge")

        self.declare_parameter("http_port", 8081)
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("tip_link", "tool0")
        self.declare_parameter("base_frame", "base_link")

        self.http_port = self.get_parameter("http_port").value
        self.group = self.get_parameter("planning_group").value
        self.tip_link = self.get_parameter("tip_link").value
        self.base_frame = self.get_parameter("base_frame").value

        callbacks = ReentrantCallbackGroup()

        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=callbacks
        )
        self.plan_client = self.create_client(
            GetMotionPlan, "/plan_kinematic_path", callback_group=callbacks
        )

        # Estado do gêmeo virtual. A interface empurra isso.
        self.q = [0.0, 0.0, 0.0]
        self.q_lock = threading.Lock()

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / 30.0, self.publish_joint_states, callback_group=callbacks)

        self.get_logger().info(
            f"moveit_bridge pronto. grupo={self.group} tip={self.tip_link} http={self.http_port}"
        )

    # ------------------------------------------------------------------
    # /joint_states
    # ------------------------------------------------------------------

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        with self.q_lock:
            msg.position = list(self.q)
        self.joint_pub.publish(msg)

    def set_state(self, q):
        with self.q_lock:
            self.q = [float(v) for v in q]

    def get_state(self):
        with self.q_lock:
            return list(self.q)

    # ------------------------------------------------------------------
    # Chamada de serviço bloqueante (o executor gira em outra thread)
    # ------------------------------------------------------------------

    def call(self, client, request, timeout=CALL_TIMEOUT):
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=SERVICE_WAIT):
                raise RuntimeError(f"serviço {client.srv_name} indisponível")

        future = client.call_async(request)

        deadline = time.time() + timeout
        while time.time() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.005)

        future.cancel()
        raise RuntimeError(f"timeout em {client.srv_name}")

    # ------------------------------------------------------------------
    # IK: posição do TCP -> ângulos de junta
    # ------------------------------------------------------------------

    def compute_ik(self, xyz, seed, limits=None, hint=None):
        """
        O KDL resolve dentro dos limites do URDF, não dos operacionais
        definidos na interface. Como um braço de 3 GDL costuma ter mais
        de uma solução para o mesmo ponto, uma solução fora da faixa
        não significa que o ponto seja inalcançável: vale tentar de
        novo a partir de outras sementes dentro da faixa.
        """
        ranges = limits or [JOINT_LIMITS[name] for name in JOINT_NAMES]

        def inside(index, value):
            low, high = float(ranges[index][0]), float(ranges[index][1])
            return max(low, min(high, float(value)))

        def fraction_of(index, fraction):
            low, high = float(ranges[index][0]), float(ranges[index][1])
            return low + (high - low) * fraction

        attempts = []

        # Semente vinda da interface: ela conhece a configuracao exata
        # que atinge o alvo, entao o KDL so precisa refinar. Sem isso o
        # solver cai em outro ramo e devolve solucao fora da janela.
        if hint:
            attempts.append([float(v) for v in hint])

        attempts.append(list(seed))

        # O KDL e um solver LOCAL: partindo de revbase=0 ele nao acha
        # solucoes do lado oposto do robo, e 26 de 87 pontos de teste
        # falhavam so por isso. Como revbase e justamente a junta que
        # controla o azimute, apontar a base para o alvo e a melhor
        # semente disponivel.
        azimuth = math.atan2(float(xyz[1]), float(xyz[0]))
        attempts.append([inside(0, azimuth), seed[1], seed[2]])

        for fraction in (0.35, 0.65, 0.15, 0.85):
            attempts.append(
                [inside(0, azimuth), fraction_of(1, fraction), fraction_of(2, fraction)]
            )

        # Os CANTOS do espaco de juntas. As bordas do volume so sao
        # alcancaveis com as juntas nos extremos -- o fundo, por
        # exemplo, exige revlink1 no maximo e relink2 no minimo -- e
        # sementes em fracoes intermediarias nunca chegam la.
        for f1 in (0.0, 1.0):
            for f2 in (0.0, 1.0):
                attempts.append([inside(0, azimuth), fraction_of(1, f1), fraction_of(2, f2)])

        # Ultimo recurso: varre tambem o azimute, para o caso de o alvo
        # so ser alcancavel com a base em outra direcao.
        for fraction in (0.25, 0.5, 0.75):
            attempts.append([fraction_of(0, fraction), fraction_of(1, 0.5), fraction_of(2, 0.5)])

        last_error = None

        for attempt in attempts:
            try:
                solution = self._compute_ik_once(xyz, attempt)
            except RuntimeError as exc:
                last_error = exc
                continue

            if not limits or self._within(solution, limits):
                return solution

            last_error = RuntimeError("IK resolveu fora dos limites definidos")

        raise last_error or RuntimeError("IK falhou")

    @staticmethod
    def _within(q, limits, tolerance=1e-4):
        return all(
            float(lo) - tolerance <= value <= float(hi) + tolerance
            for value, (lo, hi) in zip(q, limits)
        )

    def _compute_ik_once(self, xyz, seed):
        request = GetPositionIK.Request()

        ik = PositionIKRequest()
        ik.group_name = self.group
        ik.ik_link_name = self.tip_link
        ik.avoid_collisions = True

        ik.robot_state.joint_state.name = list(JOINT_NAMES)
        ik.robot_state.joint_state.position = list(seed)
        ik.robot_state.is_diff = False

        ik.pose_stamped.header.frame_id = self.base_frame
        ik.pose_stamped.pose.position.x = float(xyz[0])
        ik.pose_stamped.pose.position.y = float(xyz[1])
        ik.pose_stamped.pose.position.z = float(xyz[2])
        # Com position_only_ik a orientação é ignorada, mas o
        # quaternion precisa ser válido.
        ik.pose_stamped.pose.orientation.w = 1.0

        ik.timeout = Duration(sec=0, nanosec=200_000_000)

        request.ik_request = ik

        response = self.call(self.ik_client, request)

        if response is None or response.error_code.val != 1:
            if response is None:
                raise RuntimeError("IK sem resposta do move_group")

            code = response.error_code.val

            # -31 = NO_IK_SOLUTION. Em um braço de 3 GDL isso quase
            # sempre significa ponto fora do alcance, não um bug.
            if code == -31:
                raise RuntimeError("ponto fora do alcance do braço")

            raise RuntimeError(f"IK falhou (error_code={code})")

        solution = response.solution.joint_state
        index = {name: i for i, name in enumerate(solution.name)}

        missing = [n for n in JOINT_NAMES if n not in index]
        if missing:
            raise RuntimeError(f"IK não retornou as juntas {missing}")

        return [float(solution.position[index[n]]) for n in JOINT_NAMES]

    # ------------------------------------------------------------------
    # Planejamento: start + goal em juntas -> trajetória
    # ------------------------------------------------------------------

    def plan(self, start, goal, velocity_scale=0.3, acceleration_scale=0.3, limits=None):
        request = GetMotionPlan.Request()
        mpr = request.motion_plan_request

        mpr.group_name = self.group
        mpr.num_planning_attempts = 10
        mpr.allowed_planning_time = 2.0
        mpr.max_velocity_scaling_factor = float(velocity_scale)
        mpr.max_acceleration_scaling_factor = float(acceleration_scale)

        mpr.start_state.joint_state.name = list(JOINT_NAMES)
        mpr.start_state.joint_state.position = list(start)
        mpr.start_state.is_diff = False

        constraints = Constraints()
        constraints.name = "goal"
        for name, value in zip(JOINT_NAMES, goal):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        mpr.goal_constraints = [constraints]

        # Limites operacionais definidos na interface. Sem isto o
        # planejador usa só os limites do URDF e a trajetória pode sair
        # da faixa escolhida — prender apenas o alvo não basta, porque
        # o caminho até ele também precisa respeitá-la.
        if limits:
            path = Constraints()
            path.name = "limites"

            for name, pair in zip(JOINT_NAMES, limits):
                low, high = float(pair[0]), float(pair[1])

                jc = JointConstraint()
                jc.joint_name = name
                jc.position = (low + high) / 2.0
                jc.tolerance_above = (high - low) / 2.0
                jc.tolerance_below = (high - low) / 2.0
                jc.weight = 1.0
                path.joint_constraints.append(jc)

            mpr.path_constraints = path

        response = self.call(self.plan_client, request)

        if response is None:
            raise RuntimeError("planejamento sem resposta")

        result = response.motion_plan_response

        if result.error_code.val != 1:
            raise RuntimeError(f"planejamento falhou (error_code={result.error_code.val})")

        trajectory = result.trajectory.joint_trajectory
        index = {name: i for i, name in enumerate(trajectory.joint_names)}

        points = []
        for point in trajectory.points:
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            points.append(
                {
                    "t": t,
                    "q": [float(point.positions[index[n]]) for n in JOINT_NAMES],
                }
            )

        return {
            "joint_names": list(JOINT_NAMES),
            "points": points,
            "planning_time": float(result.planning_time),
        }


# ----------------------------------------------------------------------
# HTTP
# ----------------------------------------------------------------------


def clamp_to_limits(q):
    out = []
    for name, value in zip(JOINT_NAMES, q):
        low, high = JOINT_LIMITS[name]
        out.append(max(low, min(high, float(value))))
    return out


class Handler(BaseHTTPRequestHandler):

    bridge = None

    def log_message(self, fmt, *args):
        pass

    def _send(self, code, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        length = int(self.headers.get("Content-Length", 0))
        if not length:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_OPTIONS(self):
        self._send(204, {})

    def do_GET(self):
        if self.path.startswith("/health"):
            self._send(
                200,
                {
                    "ok": True,
                    "ik_service": self.bridge.ik_client.service_is_ready(),
                    "plan_service": self.bridge.plan_client.service_is_ready(),
                    "joint_names": JOINT_NAMES,
                    "limits": JOINT_LIMITS,
                    "q": self.bridge.get_state(),
                },
            )
            return

        if self.path.startswith("/state"):
            self._send(200, {"ok": True, "q": self.bridge.get_state()})
            return

        self._send(404, {"ok": False, "error": "rota desconhecida"})

    def do_POST(self):
        try:
            body = self._body()
        except Exception as exc:
            self._send(400, {"ok": False, "error": f"JSON inválido: {exc}"})
            return

        try:
            if self.path.startswith("/state"):
                q = body.get("q")
                if not isinstance(q, list) or len(q) != 3:
                    raise ValueError("espera-se q com 3 valores em radianos")
                self.bridge.set_state(clamp_to_limits(q))
                self._send(200, {"ok": True, "q": self.bridge.get_state()})
                return

            if self.path.startswith("/plan"):
                self._send(200, self._plan(body))
                return

            self._send(404, {"ok": False, "error": "rota desconhecida"})

        except Exception as exc:
            self._send(200, {"ok": False, "error": str(exc)})

    def _plan(self, body):
        start = body.get("start")
        if not isinstance(start, list) or len(start) != 3:
            start = self.bridge.get_state()
        start = clamp_to_limits(start)

        limits = body.get("limits")
        if limits is not None:
            if not isinstance(limits, list) or len(limits) != 3:
                raise ValueError("limits espera 3 pares [min, max] em radianos")

        goal = body.get("goal") or {}
        goal_type = goal.get("type", "joint")

        if goal_type == "joint":
            q_goal = goal.get("q")
            if not isinstance(q_goal, list) or len(q_goal) != 3:
                raise ValueError("goal joint espera q com 3 valores")
            q_goal = clamp_to_limits(q_goal)
            ik_used = False

        elif goal_type == "pose":
            xyz = goal.get("xyz")
            if not isinstance(xyz, list) or len(xyz) != 3:
                raise ValueError("goal pose espera xyz com 3 valores")
            q_goal = self.bridge.compute_ik(xyz, start, limits, body.get("seed"))
            ik_used = True

        else:
            raise ValueError(f"goal.type desconhecido: {goal_type}")

        plan = self.bridge.plan(
            start,
            q_goal,
            velocity_scale=float(body.get("velocity_scale", 0.3)),
            acceleration_scale=float(body.get("acceleration_scale", 0.3)),
            limits=limits,
        )

        plan["ok"] = True
        plan["ik_used"] = ik_used
        plan["start"] = start
        plan["goal_q"] = q_goal
        return plan


def main():
    rclpy.init()

    bridge = MoveItBridge()
    Handler.bridge = bridge

    executor = MultiThreadedExecutor()
    executor.add_node(bridge)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    server = ThreadingHTTPServer(("0.0.0.0", bridge.http_port), Handler)
    bridge.get_logger().info(f"HTTP ouvindo em 0.0.0.0:{bridge.http_port}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
