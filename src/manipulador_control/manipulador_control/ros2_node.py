from pathlib import Path
import sys
import threading
import time

# Inclui o diretório do pacote para imports internos (ik_craig, robot_control, serial_driver, yolo_detector)
_PKG_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_PKG_DIR))

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

from config import GRBL_SETTINGS
import ik_craig as ik
from robot_control import RobotController
from serial_driver import SerialDriver
from yolo_detector import YOLODetector

CAMERA_TICK_S = 0.03      # 30 ms (~33 FPS), igual ao CAMERA_TICK_MS = 30 do gui.py
ANIM_FPS = 30.0           # 30 quadros/s para interpolação suave no RViz2
ANIM_TICK_S = 1.0 / ANIM_FPS
PREVIEW_VEL_MULT = 3.0    # Prévia (sombra) 3x mais rápida que o robô real
POSE_BUSCA = (-110.0, 0.0, -20.0, 0.0, -105.0, 0.0)
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'jaw1', 'jaw2']
_T_CRAIG_TO_RVIZ = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
_MAX_RATES_DEG_S = np.array([GRBL_SETTINGS[110 + i] for i in range(6)], dtype=float) / 60.0
_ACCEL_DEG_S2 = max(1.0, float(min(GRBL_SETTINGS[120 + i] for i in range(6))))


def craig_deg_to_urdf_rad(theta1, theta2, theta3, A, B, C, jaw_m=0.0):
    """Converte ângulos de Craig (graus) para o sentido dos eixos de arctos_urdf.xacro (radianos)."""
    return [
        float(np.deg2rad(theta1)),
        float(np.deg2rad(theta2)),
        float(np.deg2rad(-theta3)),
        float(np.deg2rad(-A)),
        float(np.deg2rad(B)),
        float(np.deg2rad(-C)),
        float(jaw_m),
        float(jaw_m),
    ]


def craig_mm_to_rviz_m(x_mm, y_mm, z_mm):
    """Converte coordenadas de Craig (mm, frente +X) para base_link do URDF no RViz2 (m, frente -Y)."""
    return float(y_mm) / 1000.0, -float(x_mm) / 1000.0, float(z_mm) / 1000.0


def rviz_m_to_craig_mm(rx_m, ry_m, rz_m):
    """Converte coordenadas de base_link no RViz2 (m, frente -Y) para Craig (mm, frente +X)."""
    return -float(ry_m) * 1000.0, float(rx_m) * 1000.0, float(rz_m) * 1000.0


def quat_to_rot_matrix(qx, qy, qz, qw):
    """Converte um quatérnio (x, y, z, w) em matriz de rotação 3x3."""
    n = float(np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw))
    if n < 1e-9:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw),       2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw),       1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw),       2.0 * (qy * qz + qx * qw),       1.0 - 2.0 * (qx * qx + qy * qy)],
    ], dtype=float)


def _duracao_segmento(ini, fim, vel_deg_s):
    """Estima o tempo do segmento respeitando F (norma 6D do GRBL) e $110..$115."""
    delta = fim - ini
    dist_6d = float(np.linalg.norm(delta))
    dt = max(dist_6d / vel_deg_s, float(np.max(np.abs(delta) / _MAX_RATES_DEG_S)))
    return dt, dist_6d


def _indice_por_tempo(n_quadros, t0, t_agora, fps=ANIM_FPS):
    return min(n_quadros - 1, max(0, int((t_agora - t0) * fps)))


def interpolar_angulos_30fps(
    seq_ou_eventos,
    feedrate_deg_min=800.0,
    fps=ANIM_FPS,
    angulo_inicial=None,
    jaw_inicial=0.0,
):
    """Reamostra movimentos e pausas de garra a 30 FPS pelo tempo real estimado do GRBL."""
    if not seq_ou_eventos:
        return [], []

    if isinstance(seq_ou_eventos[0], dict):
        eventos = seq_ou_eventos
    else:
        eventos = [{"tipo": "mov", "angulos": seq_ou_eventos, "feedrate": feedrate_deg_min}]

    cur_jaw = float(jaw_inicial)
    pts, jaws, t_cum = [], [], []

    if angulo_inicial is not None and np.all(np.isfinite(angulo_inicial)):
        pts.append(np.asarray(angulo_inicial, dtype=float))
        jaws.append(cur_jaw)
        t_cum.append(0.0)

    for ev in eventos:
        if ev["tipo"] == "mov":
            vel_deg_s = max(float(ev.get("feedrate", feedrate_deg_min)), 10.0) / 60.0
            t0_bloco = t_cum[-1] if t_cum else 0.0
            idx_ini = len(t_cum)
            dist_bloco = 0.0
            for a in ev["angulos"]:
                if not np.all(np.isfinite(a)):
                    continue
                fim = np.asarray(a, dtype=float)
                if not pts:
                    pts.append(fim)
                    jaws.append(cur_jaw)
                    t_cum.append(0.0)
                    t0_bloco = 0.0
                    idx_ini = 1
                    continue
                dt, dist_6d = _duracao_segmento(pts[-1], fim, vel_deg_s)
                if not np.isfinite(dt) or dt <= 1e-9:
                    continue
                dist_bloco += dist_6d
                pts.append(fim)
                jaws.append(cur_jaw)
                t_cum.append(t_cum[-1] + dt)
            dt_bloco = (t_cum[-1] - t0_bloco) if len(t_cum) > idx_ini else 0.0
            if dist_bloco > 1e-9 and dt_bloco > 1e-9:
                v_bloco = dist_bloco / dt_bloco
                if dist_bloco >= (v_bloco * v_bloco) / _ACCEL_DEG_S2:
                    t_real = dt_bloco + v_bloco / _ACCEL_DEG_S2
                else:
                    t_real = 2.0 * float(np.sqrt(dist_bloco / _ACCEL_DEG_S2))
                escala = t_real / dt_bloco
                for k in range(idx_ini, len(t_cum)):
                    t_cum[k] = t0_bloco + (t_cum[k] - t0_bloco) * escala
        elif ev["tipo"] == "garra" and pts:
            novo_jaw = float(ev["jaw_m"])
            pausa = max(0.0, float(ev.get("pausa", 0.0)))
            if pausa > 1e-9:
                t_atuacao = min(0.2, pausa)  # acompanha T0.2 do M97
                pts.append(pts[-1])
                jaws.append(novo_jaw)
                t_cum.append(t_cum[-1] + t_atuacao)
                if pausa > t_atuacao + 1e-9:
                    pts.append(pts[-1])
                    jaws.append(novo_jaw)
                    t_cum.append(t_cum[-1] + (pausa - t_atuacao))
            cur_jaw = novo_jaw

    if not pts:
        return [], []
    if len(pts) == 1:
        return [tuple(float(v) for v in pts[0])], [float(jaws[0])]

    t_total = t_cum[-1]
    t_samples = np.arange(0.0, t_total, 1.0 / fps)
    if t_samples.size == 0 or (t_total - t_samples[-1]) > 1e-6:
        t_samples = np.append(t_samples, t_total)

    pts_arr = np.vstack(pts)
    cols = [np.interp(t_samples, t_cum, pts_arr[:, c]) for c in range(6)]
    seq_ang = [tuple(float(cols[c][k]) for c in range(6)) for k in range(len(t_samples))]
    seq_jaw = [float(v) for v in np.interp(t_samples, t_cum, np.asarray(jaws, dtype=float))]
    return seq_ang, seq_jaw


class ManipuladorROS2Node(Node):
    def __init__(self):
        super().__init__('manipulador_node')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.camera_index = int(self.get_parameter('camera_index').value)
        serial_port = str(self.get_parameter('serial_port').value)
        self._cb_group = ReentrantCallbackGroup()
        self._cam_group = MutuallyExclusiveCallbackGroup()
        self.em_movimento = threading.Event()
        self._lock = threading.Lock()

        # Estado atual e de prévia (theta1, theta2, theta3, A, B, C) em graus de Craig
        self._current_angles = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._current_jaw_m = 0.0
        self._real_seq = []
        self._real_jaw_seq = []
        self._real_idx = 0
        self._real_t0 = 0.0
        self._preview_angles = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._preview_jaw_m = 0.0
        self._preview_seq = []
        self._preview_jaw_seq = []
        self._preview_idx = 0
        self._preview_t0 = 0.0
        self._cached_points_mm = []

        # 1. Publishers para o RViz2 e Painel Lateral (criados logo no início)
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_preview_joints = self.create_publisher(JointState, '/preview/joint_states', 10)
        self.pub_path = self.create_publisher(RosPath, '/planned_path', 10)
        self.pub_image = self.create_publisher(Image, '/camera/image_raw', 10)
        self.pub_status = self.create_publisher(String, '/manipulador_status', 10)

        # 2. Gizmo 6-DOF Interativo criado imediatamente para o RViz2 encontrar o serviço ao abrir
        self._gizmo_R_ref = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], dtype=float)
        self.im_server = InteractiveMarkerServer(self, 'gizmo_efetuador')
        self.menu_handler = MenuHandler()
        self._configurar_gizmo_6dof(np.array([403.3643, 0.0, 570.3432]))

        # 3. Hardware / Simulação GRBL + RobotController (mesma sequência de boot do main.py)
        self.serial = SerialDriver(port=serial_port)
        self.controller = RobotController(
            self.serial,
            on_angle_update=self._on_controller_angles,
            on_gripper_update=self._on_controller_gripper,
        )
        self.serial.send_settings()
        self.serial.send("G90")
        self.serial.send("F800")
        self.controller.recuperar_do_log()
        self.serial.reset_log()
        self.serial.send("M97 B60 T0.2")

        # 4. Visão Computacional (YOLO + Câmera V4L2 idêntica ao gui.py)
        self.detector = YOLODetector(weights_path=_PKG_DIR / "yolo" / "pesos" / "last.pt")
        self.alvos = []
        self.camera_ao_vivo = True
        self.ultimo_frame_anotado = None
        self._placeholder_frame = self._criar_frame_placeholder()
        self.cap = self._abrir_camera()

        # 5. Subscribers para entrada numérica de Juntas, Pose Cartesiana e Velocidade (Feedrate)
        self.create_subscription(
            Float64MultiArray, '/planejar_juntas', self._on_planejar_juntas, 10, callback_group=self._cb_group
        )
        self.create_subscription(
            Float64MultiArray, '/planejar_pose', self._on_planejar_pose, 10, callback_group=self._cb_group
        )
        self.create_subscription(
            Float64MultiArray, '/configurar_feedrate', self._on_configurar_feedrate, 10, callback_group=self._cb_group
        )

        # 6. Services para todos os botões da interface
        self.create_service(Trigger, '/planejar_home', self.srv_planejar_home, callback_group=self._cb_group)
        self.create_service(Trigger, '/planejar_rotina_lapis', self.srv_planejar_lapis, callback_group=self._cb_group)
        self.create_service(Trigger, '/planejar_manipular', self.srv_planejar_manipular, callback_group=self._cb_group)
        self.create_service(Trigger, '/executar', self.srv_executar, callback_group=self._cb_group)
        self.create_service(Trigger, '/reconhecer', self.srv_reconhecer, callback_group=self._cb_group)
        self.create_service(Trigger, '/abrir_garra', self.srv_abrir_garra, callback_group=self._cb_group)
        self.create_service(Trigger, '/fechar_garra', self.srv_fechar_garra, callback_group=self._cb_group)
        self.create_service(Trigger, '/calibrar', self.srv_calibrar, callback_group=self._cb_group)

        # 7. Timers de atualização do RViz2 (30 FPS) e Câmera
        self.create_timer(ANIM_TICK_S, self._tick_joint_states, callback_group=self._cb_group)
        self.create_timer(0.5, self._tick_republish_path, callback_group=self._cb_group)
        self.create_timer(CAMERA_TICK_S, self._tick_camera, callback_group=self._cam_group)

        self._emit_status("Pronto! Arraste o Gizmo 3D na ponta da garra ou use o painel lateral.")

    def _emit_status(self, texto: str):
        self.get_logger().info(texto)
        msg = String()
        msg.data = texto
        self.pub_status.publish(msg)

    # --- GIZMO 6-DOF INTERATIVO (POSIÇÃO + ORIENTAÇÃO DA GARRA NO RVIZ2) ---

    def _configurar_gizmo_6dof(self, p0_craig_mm):
        self.menu_handler.insert("Planejar Curva Bezier ate o Gizmo", callback=self._on_gizmo_menu_planejar)
        self.menu_handler.insert("Executar Movimento Planejado", callback=self._on_gizmo_menu_executar)
        self.menu_handler.insert("Resetar Gizmo na Posicao Atual da Garra", callback=self._on_gizmo_menu_reset)

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "gizmo_garra"
        int_marker.description = "Gizmo 6-DOF da Garra"
        int_marker.scale = 0.22

        rx, ry, rz = craig_mm_to_rviz_m(*p0_craig_mm)
        int_marker.pose.position.x = rx
        int_marker.pose.position.y = ry
        int_marker.pose.position.z = rz
        int_marker.pose.orientation.w = 1.0

        # Esfera central com menu de contexto (clique direito)
        ctrl_center = InteractiveMarkerControl()
        ctrl_center.always_visible = True
        ctrl_center.interaction_mode = InteractiveMarkerControl.MENU
        ctrl_center.name = "menu_control"
        esfera = Marker()
        esfera.type = Marker.SPHERE
        esfera.scale.x = 0.045
        esfera.scale.y = 0.045
        esfera.scale.z = 0.045
        esfera.color.r = 1.0
        esfera.color.g = 0.6
        esfera.color.b = 0.0
        esfera.color.a = 0.9
        ctrl_center.markers.append(esfera)
        int_marker.controls.append(ctrl_center)

        # Controles 6-DOF (Setas de Translação + Anéis de Rotação em X, Y, Z)
        eixos = [
            ("x", 1.0, 1.0, 0.0, 0.0),
            ("z", 1.0, 0.0, 1.0, 0.0),
            ("y", 1.0, 0.0, 0.0, 1.0),
        ]
        for nome, qw, qx, qy, qz in eixos:
            n = float(np.sqrt(qw * qw + qx * qx + qy * qy + qz * qz))
            for modo, prefixo in [
                (InteractiveMarkerControl.ROTATE_AXIS, "rotate_"),
                (InteractiveMarkerControl.MOVE_AXIS, "move_"),
            ]:
                ctrl = InteractiveMarkerControl()
                ctrl.always_visible = True
                ctrl.orientation.w = qw / n
                ctrl.orientation.x = qx / n
                ctrl.orientation.y = qy / n
                ctrl.orientation.z = qz / n
                ctrl.name = prefixo + nome
                ctrl.interaction_mode = modo
                int_marker.controls.append(ctrl)

        self.im_server.insert(int_marker, feedback_callback=self._on_gizmo_feedback)
        self.menu_handler.apply(self.im_server, int_marker.name)
        self.im_server.applyChanges()

    def _sincronizar_gizmo_com_pose(self, p_craig_mm):
        pose = PoseStamped().pose
        rx, ry, rz = craig_mm_to_rviz_m(*p_craig_mm)
        pose.position.x = rx
        pose.position.y = ry
        pose.position.z = rz
        pose.orientation.w = 1.0
        self.im_server.setPose("gizmo_garra", pose)
        self.im_server.applyChanges()

    def _planejar_a_partir_do_gizmo(self, pose):
        if self.em_movimento.is_set():
            self._emit_status("Robô em movimento — aguarde terminar.")
            return
        x_mm, y_mm, z_mm = rviz_m_to_craig_mm(pose.position.x, pose.position.y, pose.position.z)
        P_base = np.array([x_mm, y_mm, z_mm], dtype=float)

        dR_rviz = quat_to_rot_matrix(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        dR_craig = _T_CRAIG_TO_RVIZ.T @ dR_rviz @ _T_CRAIG_TO_RVIZ

        if self.controller.modo_juntas:
            self.controller.modo_juntas = False

        ok = False
        for R_cand in (dR_craig @ self._gizmo_R_ref, dR_craig @ self.controller.Ri):
            if self.controller.ponto_alcancavel(P_base, R_cand):
                ok, _ = self.controller.planejar_pose(P_base, usar_referencial_mesa=False, R_alvo=R_cand)
                if ok:
                    break

        P_mesa = P_base + self.controller.base_offset
        if ok:
            self._atualizar_visualizacao_plano(atualizar_gizmo=False)
            self._emit_status(
                f"Gizmo planejado -> Mesa X={P_mesa[0]:.1f} Y={P_mesa[1]:.1f} Z={P_mesa[2]:.1f} mm"
            )
        else:
            self._emit_status(
                f"Gizmo fora de alcance: Base=({x_mm:.0f}, {y_mm:.0f}, {z_mm:.0f}) mm | "
                f"Mesa=({P_mesa[0]:.0f}, {P_mesa[1]:.0f}, {P_mesa[2]:.0f}) mm"
            )

    def _on_gizmo_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self._planejar_a_partir_do_gizmo(feedback.pose)

    def _on_gizmo_menu_planejar(self, feedback: InteractiveMarkerFeedback):
        self._planejar_a_partir_do_gizmo(feedback.pose)

    def _on_gizmo_menu_executar(self, feedback: InteractiveMarkerFeedback):
        req, res = Trigger.Request(), Trigger.Response()
        self.srv_executar(req, res)

    def _on_gizmo_menu_reset(self, feedback: InteractiveMarkerFeedback):
        self._gizmo_R_ref = self.controller.R_baixo.copy()
        self._sincronizar_gizmo_com_pose(self.controller.P0)
        self._emit_status("Gizmo reposicionado na ponta atual da garra.")

    # --- CÂMERA E IMAGEM ---

    def _abrir_camera(self):
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            self.get_logger().warning(f"Câmera no índice {self.camera_index} indisponível. Exibindo placeholder no RViz2.")
            return None

        # ponytail: a calibração vale só para 640x480 e foco fixo; mudou qualquer um, recalibre
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_FOCUS, 5)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        cap.set(cv2.CAP_PROP_EXPOSURE, 150)
        cap.set(cv2.CAP_PROP_GAIN, 0)
        return cap

    def _criar_frame_placeholder(self):
        img = np.full((480, 640, 3), 51, dtype=np.uint8)
        cv2.putText(
            img, "Camera indisponivel", (165, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (170, 170, 170), 2
        )
        return img

    def _publicar_frame_bgr(self, frame):
        h, w, ch = frame.shape
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.height = h
        msg.width = w
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = w * ch
        msg.data = frame.tobytes()
        self.pub_image.publish(msg)

    def _tick_camera(self):
        if not self.camera_ao_vivo and self.ultimo_frame_anotado is not None:
            self._publicar_frame_bgr(self.ultimo_frame_anotado)
            return

        if self.cap and self.cap.isOpened():
            ok, frame = self.cap.read()
            if ok:
                self._publicar_frame_bgr(frame)
                return

        self._publicar_frame_bgr(self._placeholder_frame)

    # --- PUBLICAÇÃO DE JUNTAS (INTERPOLADAS A 30 FPS) E TRAJETÓRIA BÉZIER NO RVIZ2 ---

    def _on_controller_angles(self, theta1, theta2, theta3, A, B, C, dur=0.02, live=False):
        with self._lock:
            if live:
                self._real_seq = []
                self._real_jaw_seq = []
                self._current_angles = (theta1, theta2, theta3, A, B, C)
                if not self.controller.plano_pendente:
                    self._preview_angles = self._current_angles
                return
            if not self._real_seq and not self.em_movimento.is_set():
                self._current_angles = (theta1, theta2, theta3, A, B, C)
                if not self.controller.plano_pendente:
                    self._preview_angles = self._current_angles

    def _on_controller_gripper(self, jaw_m):
        with self._lock:
            if not self._real_seq:
                self._current_jaw_m = float(jaw_m)
                if not self.controller.plano_pendente:
                    self._preview_jaw_m = self._current_jaw_m

    def _tick_joint_states(self):
        now = self.get_clock().now().to_msg()
        t_agora = time.monotonic()
        with self._lock:
            if self._real_seq:
                self._real_idx = _indice_por_tempo(len(self._real_seq), self._real_t0, t_agora)
                cur = self._real_seq[self._real_idx]
                jaw_real = self._real_jaw_seq[self._real_idx]
                self._current_angles = cur
                self._current_jaw_m = jaw_real
            else:
                cur = self._current_angles
                jaw_real = self._current_jaw_m

            if self._preview_seq:
                self._preview_idx = _indice_por_tempo(
                    len(self._preview_seq), self._preview_t0, t_agora, fps=ANIM_FPS * PREVIEW_VEL_MULT
                )
                prv = self._preview_seq[self._preview_idx]
                jaw_prev = self._preview_jaw_seq[self._preview_idx]
            else:
                prv = self._preview_angles
                jaw_prev = self._preview_jaw_m

        msg_real = JointState()
        msg_real.header.stamp = now
        msg_real.name = JOINT_NAMES
        msg_real.position = craig_deg_to_urdf_rad(*cur, jaw_m=jaw_real)
        self.pub_joints.publish(msg_real)

        msg_prev = JointState()
        msg_prev.header.stamp = now
        msg_prev.name = JOINT_NAMES
        msg_prev.position = craig_deg_to_urdf_rad(*prv, jaw_m=jaw_prev)
        self.pub_preview_joints.publish(msg_prev)

    def _atualizar_visualizacao_plano(self, atualizar_gizmo=True):
        pontos_mm, eventos, angulo_final = self.controller.obter_resumo_plano()
        with self._lock:
            ang_ini = self._current_angles
            jaw_ini = self._current_jaw_m
        seq_30fps, jaws_30fps = interpolar_angulos_30fps(
            eventos,
            feedrate_deg_min=self.controller.feedrate,
            fps=ANIM_FPS,
            angulo_inicial=ang_ini,
            jaw_inicial=jaw_ini,
        )
        with self._lock:
            self._cached_points_mm = list(pontos_mm)
            self._preview_seq = seq_30fps
            self._preview_jaw_seq = jaws_30fps
            self._preview_idx = 0
            self._preview_t0 = time.monotonic()
            self._preview_angles = angulo_final if angulo_final is not None else self._current_angles
            self._preview_jaw_m = jaws_30fps[-1] if jaws_30fps else self._current_jaw_m
        self._publicar_path(self._cached_points_mm)
        if atualizar_gizmo and pontos_mm:
            self._sincronizar_gizmo_com_pose(pontos_mm[-1])

    def _publicar_path(self, pontos_mm):
        path_msg = RosPath()
        now = self.get_clock().now().to_msg()
        path_msg.header.stamp = now
        path_msg.header.frame_id = "base_link"

        for pt in pontos_mm:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = "base_link"
            rx, ry, rz = craig_mm_to_rviz_m(pt[0], pt[1], pt[2])
            pose.pose.position.x = rx
            pose.pose.position.y = ry
            pose.pose.position.z = rz
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.pub_path.publish(path_msg)

    def _tick_republish_path(self):
        with self._lock:
            pts = list(self._cached_points_mm)
        self._publicar_path(pts)

    # --- SUBSCRIBERS DE PLANEJAMENTO NUMÉRICO E VELOCIDADE ---

    def _on_configurar_feedrate(self, msg: Float64MultiArray):
        if msg.data and msg.data[0] > 1.0:
            self.controller.feedrate = int(round(msg.data[0]))
            if self.controller.plano_pendente and not self.em_movimento.is_set():
                self._atualizar_visualizacao_plano(atualizar_gizmo=False)
            self._emit_status(f"Velocidade GRBL ajustada para F{self.controller.feedrate} (graus/min).")

    def _on_planejar_juntas(self, msg: Float64MultiArray):
        if self.em_movimento.is_set():
            self._emit_status("Robô em movimento — aguarde terminar antes de planejar.")
            return
        if len(msg.data) < 6:
            self._emit_status("Erro: envie 6 ângulos em graus [J1..J6].")
            return
        if len(msg.data) >= 7 and msg.data[6] > 1.0:
            self.controller.feedrate = int(round(msg.data[6]))
        ok, texto = self.controller.planejar_juntas(*msg.data[:6])
        self._atualizar_visualizacao_plano()
        self._emit_status(texto)

    def _on_planejar_pose(self, msg: Float64MultiArray):
        if self.em_movimento.is_set():
            self._emit_status("Robô em movimento — aguarde terminar antes de planejar.")
            return
        if len(msg.data) < 3:
            self._emit_status("Erro: envie ao menos [X, Y, Z] em mm.")
            return
        usar_mesa = True if len(msg.data) < 4 else bool(msg.data[3])
        if len(msg.data) >= 5 and msg.data[4] > 1.0:
            self.controller.feedrate = int(round(msg.data[4]))
        ok, texto = self.controller.planejar_pose(msg.data[:3], usar_referencial_mesa=usar_mesa)
        if ok:
            self._atualizar_visualizacao_plano()
        self._emit_status(texto)

    # --- SERVICES (BOTÕES DO PAINEL LATERAL NO RVIZ2) ---

    def srv_planejar_home(self, request, response):
        if self.em_movimento.is_set():
            response.success = False
            response.message = "Robô em movimento."
            return response
        ok, msg = self.controller.planejar_home()
        self._atualizar_visualizacao_plano()
        self._emit_status(msg)
        response.success = ok
        response.message = msg
        return response

    def srv_planejar_lapis(self, request, response):
        if self.em_movimento.is_set():
            response.success = False
            response.message = "Robô em movimento."
            return response
        ok, msg = self.controller.planejar_rotina_lapis()
        if ok:
            self._atualizar_visualizacao_plano()
        self._emit_status(msg)
        response.success = ok
        response.message = msg
        return response

    def srv_planejar_manipular(self, request, response):
        if self.em_movimento.is_set():
            response.success = False
            response.message = "Robô em movimento."
            return response
        ok, msg = self.controller.planejar_manipular(list(self.alvos), pose_busca=POSE_BUSCA)
        if ok:
            self._atualizar_visualizacao_plano()
        self._emit_status(msg)
        response.success = ok
        response.message = msg
        return response

    def srv_executar(self, request, response):
        if self.em_movimento.is_set():
            response.success = False
            response.message = "Execução já em andamento."
            return response
        if not self.controller.plano_pendente:
            msg = "Nenhum plano pendente. Planeje um movimento antes de executar."
            self._emit_status(msg)
            response.success = False
            response.message = msg
            return response

        self.camera_ao_vivo = True
        self.em_movimento.set()

        # Prepara a sequência interpolada a 30 FPS para o robô real no RViz2
        with self._lock:
            self._real_seq = list(self._preview_seq)
            self._real_jaw_seq = list(self._preview_jaw_seq)
            self._real_idx = 0
            self._real_t0 = time.monotonic()

        def _run():
            try:
                self._emit_status(f"Executando movimento no GRBL (F{int(self.controller.feedrate)})...")
                self.controller.executar_plano()
                # Aguarda a interpolação visual de 30 FPS alcançar o último quadro
                while True:
                    with self._lock:
                        if not self._real_seq or self._real_idx >= len(self._real_seq) - 1:
                            if self._real_seq:
                                self._current_angles = self._real_seq[-1]
                                self._current_jaw_m = self._real_jaw_seq[-1]
                            self._real_seq = []
                            self._real_jaw_seq = []
                            self._cached_points_mm = []
                            self._preview_seq = []
                            self._preview_jaw_seq = []
                            self._preview_angles = self._current_angles
                            self._preview_jaw_m = self._current_jaw_m
                            break
                    time.sleep(ANIM_TICK_S)
                self._publicar_path([])
                self._sincronizar_gizmo_com_pose(self.controller.P0)
                self._emit_status("Movimento concluído! Aguardando comando...")
            finally:
                self.em_movimento.clear()

        threading.Thread(target=_run, daemon=True).start()
        response.success = True
        response.message = "Execução iniciada no manipulador."
        return response

    def srv_reconhecer(self, request, response):
        if self.em_movimento.is_set():
            response.success = False
            response.message = "Robô ocupado em movimento."
            return response
        if not self.cap or not self.cap.isOpened():
            msg = "Câmera indisponível."
            self._emit_status(msg)
            response.success = False
            response.message = msg
            return response

        self.em_movimento.set()

        def _posicionar_e_detectar():
            try:
                self._emit_status("Posicionando em POSE_BUSCA para detecção YOLO...")
                with self._lock:
                    ang_ini = self._current_angles
                    jaw_ini = self._current_jaw_m
                seq_busca, jaws_busca = interpolar_angulos_30fps(
                    [ang_ini, POSE_BUSCA],
                    feedrate_deg_min=self.controller.feedrate,
                    fps=ANIM_FPS,
                    jaw_inicial=jaw_ini,
                )
                with self._lock:
                    self._real_seq = seq_busca
                    self._real_jaw_seq = jaws_busca
                    self._real_idx = 0
                    self._real_t0 = time.monotonic()
                    self._preview_angles = POSE_BUSCA
                    self._preview_jaw_m = jaw_ini
                self.controller.enviar_juntas(*POSE_BUSCA)
                self.serial.wait_idle()
                while True:
                    with self._lock:
                        if not self._real_seq:
                            break
                        if self._real_idx >= len(self._real_seq) - 1:
                            self._current_angles = POSE_BUSCA
                            self._real_seq = []
                            self._real_jaw_seq = []
                            break
                    time.sleep(ANIM_TICK_S)

                frame = None
                for _ in range(5):
                    ok, f = self.cap.read()
                    if ok:
                        frame = f

                if frame is None:
                    self._emit_status("Erro: não foi possível capturar frame da câmera.")
                    return

                annotated, detections = self.detector.detect(frame)
                self.ultimo_frame_anotado = annotated
                self.camera_ao_vivo = False

                self.alvos = []
                if not detections:
                    self._emit_status("Nenhum objeto detectado pelo YOLO.")
                    return

                for d in detections:
                    if "phys_center" not in d:
                        continue
                    p = np.array([*d["phys_center"], 0.0])
                    ok = self.controller.ponto_alcancavel(p - self.controller.base_offset)
                    if ok:
                        self.alvos.append((d["confidence"], p, d.get("angle", 0.0)))
                    msg = (
                        f"Detectado: {d['class']} ({d['confidence']:.2f}) -> "
                        f"X={p[0]:.1f} Y={p[1]:.1f} Z={p[2]:.1f}"
                        + ("" if ok else " [FORA DE ALCANCE]")
                    )
                    self._emit_status(msg)

                self.alvos.sort(key=lambda a: a[0], reverse=True)
                self._emit_status(
                    f"{len(self.alvos)} objeto(s) alcançável(is). Clique em 'Planejar Manipular'."
                )
            finally:
                self.em_movimento.clear()

        threading.Thread(target=_posicionar_e_detectar, daemon=True).start()
        response.success = True
        response.message = "Posicionando em POSE_BUSCA e iniciando detecção YOLO..."
        return response

    def srv_abrir_garra(self, request, response):
        cmd = "M97 B50 T0.2"
        self.serial.send(cmd)
        self._on_controller_gripper(RobotController.jaw_m_de_cmd(cmd))
        self._emit_status(f"Garra aberta ({cmd}).")
        response.success = True
        response.message = f"Garra aberta ({cmd})."
        return response

    def srv_fechar_garra(self, request, response):
        cmd = "M97 B0 T0.2"
        self.serial.send(cmd)
        self._on_controller_gripper(RobotController.jaw_m_de_cmd(cmd))
        self._emit_status(f"Garra fechada ({cmd}).")
        response.success = True
        response.message = f"Garra fechada ({cmd})."
        return response

    def srv_calibrar(self, request, response):
        if self.em_movimento.is_set():
            response.success = False
            response.message = "Robô em movimento."
            return response
        if not self.cap or not self.cap.isOpened():
            msg = "Câmera indisponível para calibração."
            self._emit_status(msg)
            response.success = False
            response.message = msg
            return response

        self.em_movimento.set()

        def _calibrar():
            try:
                self._emit_status("Iniciando rotina de captura de calibração...")
                self.controller.rotina_captura_calibracao(self.cap)
                self._emit_status("Calibração concluída!")
            finally:
                self.em_movimento.clear()

        threading.Thread(target=_calibrar, daemon=True).start()
        response.success = True
        response.message = "Rotina de calibração iniciada."
        return response

    def destroy_node(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManipuladorROS2Node()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
