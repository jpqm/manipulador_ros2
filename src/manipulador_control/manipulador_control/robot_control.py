import json
import os
from pathlib import Path
import re
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2
import numpy as np
import bezier as bz
import ik_craig as ik
from config import CAPTURAS_DIR, GCODE_LOG


def calc_semi_circ(c, u, v, r=500):
    t = np.linspace(0, 1, 20)[:, np.newaxis]
    u = np.asarray(u, dtype=float) / np.linalg.norm(u)
    v = np.asarray(v, dtype=float)
    v = v - np.dot(v, u) * u
    v = v / np.linalg.norm(v)
    pontos = c + r * np.cos(np.pi * t) * u + r * np.sin(np.pi * t) * v
    return pontos[:, 0], pontos[:, 1], pontos[:, 2]


def calc_semi_esfera(c, r=500, elev_range=(65, 85), azim_range=(0, 110), n_elev=5, n_azim=6):
    """Gera pontos em trajetória serpenteada sobre a semi-esfera sem tocar a mesa."""
    elevs = np.linspace(elev_range[0], elev_range[1], n_elev)
    pontos = []
    for idx, el_deg in enumerate(elevs):
        azims = np.linspace(azim_range[0], azim_range[1], n_azim)
        if idx % 2 == 1:
            azims = azims[::-1]
        el = np.deg2rad(el_deg)
        for az_deg in azims:
            az = np.deg2rad(az_deg)
            pontos.append(c + r * np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)]))
    return np.array(pontos)


class RobotController:
    def __init__(self, serial_driver, on_angle_update=None, on_gripper_update=None):
        """Guarda as referências de comunicação e o estado inicial de posição e orientação do robô."""
        self.serial = serial_driver
        self.on_angle_update = on_angle_update
        self.on_gripper_update = on_gripper_update

        # Estado inicial do manipulador
        self.P0 = np.array([403.3643, 0, 570.3432])
        self.Ri = np.array([[0, 0, 1],
                            [0, -1, 0],
                            [1, 0, 0]])
        self.angulos_atuais = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.feedrate = 800
        self.base_offset = np.array([-137, 645, 26])
        self.modo_juntas = False
        self.R_baixo = np.array([[0, -1, 0],
                                 [-1, 0, 0],
                                 [0, 0, -1]])

        # Estado de planejamento (Planejar -> Visualizar -> Executar)
        self.plano_pendente = []
        self.P0_planejado = self.P0.copy()
        self.Ri_planejado = self.Ri.copy()
        self.angulos_planejados = self.angulos_atuais
        self.modo_juntas_planejado = self.modo_juntas

    def limpar_plano(self):
        """Reinicia a fila de planejamento a partir do estado real atual do robô."""
        self.plano_pendente = []
        self.P0_planejado = self.P0.copy()
        self.Ri_planejado = self.Ri.copy()
        self.angulos_planejados = self.angulos_atuais
        self.modo_juntas_planejado = self.modo_juntas

    def _emit_angles(self, theta1, theta2, theta3, A, B, C, dur=0.02):
        self.angulos_atuais = (float(theta1), float(theta2), float(theta3), float(A), float(B), float(C))
        if self.on_angle_update:
            self.on_angle_update(*self.angulos_atuais, float(dur))

    @staticmethod
    def jaw_m_de_cmd(cmd):
        """Converte um comando M97 de garra para a abertura das garras no URDF (m)."""
        return 0.0 if "B0" not in cmd else 0.015

    def _emit_gripper(self, cmd):
        if self.on_gripper_update:
            self.on_gripper_update(self.jaw_m_de_cmd(cmd))

    def interpolar_abc(self, R_ini, P_ini, R_fim, P_fim, n=21, A=None, B=None, C=None):
        """Interpola linearmente os ângulos do pulso (A, B, C) entre a pose inicial e a final."""
        if A is None and B is None and C is None:
            A0, B0, C0 = ik.calculo_angulos_abc(R_ini, P_ini)
        else:
            A0, B0, C0 = A, B, C
        Af, Bf, Cf = ik.calculo_angulos_abc(R_fim, P_fim)
        return (np.round(np.linspace(A0, Af, n), 2),
                np.round(np.linspace(B0, Bf, n), 2),
                np.round(np.linspace(C0, Cf, n), 2))

    def _trajetoria_valida(self, x, y, z, A, B, C):
        """Verifica se todos os pontos da curva possuem solução válida (sem NaN) na cinemática inversa."""
        if np.any(np.isnan(A)) or np.any(np.isnan(B)) or np.any(np.isnan(C)):
            return False
        with np.errstate(invalid="ignore"):
            for i in range(len(x)):
                t1, t2, t3 = ik.calculo_angulos(x[i], y[i], z[i])
                if np.any(np.isnan([t1, t2, t3])):
                    return False
        return True

    def enfileirar_movimento(self, x, y, z, A, B, C, feedrate=None):
        """Calcula os ângulos e pontos 3D da trajetória (bezier.py) e adiciona à fila sem mover o GRBL."""
        f = int(round(feedrate if feedrate is not None else self.feedrate))
        angulos = []
        pontos_efetuador = []
        with np.errstate(invalid="ignore"):
            for i in range(len(x)):
                theta1, theta2, theta3 = ik.calculo_angulos(x[i], y[i], z[i])
                a_val, b_val, c_val = float(A[i]), float(B[i]), float(C[i])
                vals = (float(theta1), float(theta2), float(theta3), a_val, b_val, c_val)
                if np.any(np.isnan(vals)):
                    continue
                angulos.append(vals)
                _, p_tip = ik.cinematica_direta(*vals)
                pontos_efetuador.append(p_tip)

        if not angulos:
            return False

        tempo_s = self.calcular_tempo_trajetoria(x, y, z, f)
        self.angulos_planejados = angulos[-1]
        self.plano_pendente.append({
            "tipo": "trajetoria",
            "angulos": angulos,
            "tempos": tempo_s,
            "pontos_mm": pontos_efetuador,
            "feedrate": f,
        })
        return True

    def enfileirar_juntas(self, j1, j2, j3, j4, j5, j6, feedrate=None):
        """Enfileira um comando direto de juntas."""
        f = int(round(feedrate if feedrate is not None else self.feedrate))
        alvo = (float(j1), float(j2), float(j3), float(j4), float(j5), float(j6))
        ini = np.asarray(self.angulos_planejados, dtype=float)
        fim = np.asarray(alvo, dtype=float)
        angulos_interp = [
            tuple(float(v) for v in (ini + (fim - ini) * (k / 20.0)))
            for k in range(21)
        ]
        _, p_tip = ik.cinematica_direta(*alvo)

        self.angulos_planejados = alvo
        self.plano_pendente.append({
            "tipo": "juntas",
            "juntas": alvo,
            "angulos": angulos_interp,
            "pontos_mm": [p_tip],
            "feedrate": f,
        })

    def enfileirar_garra(self, cmd, pausa=1.0):
        """Enfileira um comando M97 de abertura/fechamento da garra."""
        self.plano_pendente.append({
            "tipo": "garra",
            "cmd": cmd,
            "pausa": pausa,
        })

    def obter_resumo_plano(self):
        """Retorna pontos 3D (mm, frame Craig), eventos (movimento/garra) e o ângulo final planejado."""
        pontos_mm = []
        eventos = []
        angulo_final = None
        for item in self.plano_pendente:
            if item["tipo"] in ("trajetoria", "juntas"):
                pontos_mm.extend(item["pontos_mm"])
                eventos.append({
                    "tipo": "mov",
                    "angulos": item["angulos"],
                    "feedrate": item["feedrate"],
                })
                if item["angulos"]:
                    angulo_final = item["angulos"][-1]
            elif item["tipo"] == "garra":
                eventos.append({
                    "tipo": "garra",
                    "jaw_m": self.jaw_m_de_cmd(item["cmd"]),
                    "pausa": item["pausa"],
                })
        return pontos_mm, eventos, angulo_final

    def executar_plano(self):
        """Executa no GRBL (ou MockSerialGRBL) os passos planejados usando apenas o feedrate do G1."""
        if not self.plano_pendente:
            return False

        for item in self.plano_pendente:
            tipo = item["tipo"]
            if tipo == "trajetoria":
                angulos = item["angulos"]
                tempos = item["tempos"]
                feedrate = int(round(item["feedrate"]))
                for i, (theta1, theta2, theta3, a_val, b_val, c_val) in enumerate(angulos):
                    dur = float(tempos[i - 1]) if i > 0 else 0.02
                    self._emit_angles(theta1, theta2, theta3, a_val, b_val, c_val, dur)
                    self.serial.send(
                        f"G1 X{theta1} Y{theta2} Z{theta3} A{a_val} B{-c_val} C{b_val} F{feedrate}"
                    )
                if not self.serial.wait_idle():
                    print("[TRAJETÓRIA] Status '?' indisponível — usando tempo estimado.")
                    pausa = float(np.sum(tempos))
                    time.sleep(pausa + 1.0)

            elif tipo == "juntas":
                j1, j2, j3, j4, j5, j6 = item["juntas"]
                feedrate = int(round(item["feedrate"]))
                self.serial.send(f"G1 X{j1} Y{j2} Z{j3} A{j4} B{j6} C{j5} F{feedrate}")
                self._emit_angles(j1, j2, j3, j4, j5, j6, 0.5)
                self.serial.wait_idle()

            elif tipo == "garra":
                self.serial.send(item["cmd"])
                self._emit_gripper(item["cmd"])
                time.sleep(item["pausa"])

        self.P0 = self.P0_planejado.copy()
        self.Ri = self.Ri_planejado.copy()
        self.modo_juntas = self.modo_juntas_planejado
        self.plano_pendente = []
        return True

    def executar_movimento(self, x, y, z, A, B, C, feedrate=800):
        """Calcula os ângulos das juntas para cada ponto, envia ao GRBL (G1) e atualiza o ROS."""
        angulos = []
        for i in range(len(x)):
            theta1, theta2, theta3 = ik.calculo_angulos(x[i], y[i], z[i])
            angulos.append([theta1, theta2, theta3, A[i], -C[i], B[i]])

        tempo_s = self.calcular_tempo_trajetoria(x, y, z, feedrate)
        for i in range(len(angulos)):
            theta1, theta2, theta3, A_grbl, B_grbl, C_grbl = angulos[i]
            dur = tempo_s[i - 1] if i > 0 else 0.02
            self._emit_angles(theta1, theta2, theta3, A[i], B[i], C[i], dur)
            self.serial.send(f"G1 X{theta1} Y{theta2} Z{theta3} A{A_grbl} B{B_grbl} C{C_grbl} F{feedrate}")

    def ponto_alcancavel(self, ponto, R=None):
        """Retorna True se todos os 6 ângulos da cinemática inversa forem calculáveis."""
        if R is None:
            R = self.R_baixo
        try:
            with np.errstate(invalid="ignore"):
                # ponytail: valida apenas solução matemática (sem NaN); upgrade: checar qlim se colidir
                Pw = np.asarray(ponto, dtype=float) - ik.de * R[:, -1]
                t1, t2, t3 = ik.calculo_angulos(*Pw)
                t4, t5, t6 = ik.calculo_angulos_abc(R, ponto)
            return not np.any(np.isnan([t1, t2, t3, t4, t5, t6]))
        except Exception:
            return False

    def enviar_juntas(self, j1, j2, j3, j4, j5, j6):
        """Envia um G1 direto com os 6 ângulos das juntas (valores GRBL) e atualiza o ROS."""
        self.modo_juntas = True
        self.serial.send(f"G1 X{j1} Y{j2} Z{j3} A{j4} B{j6} C{j5} F{int(self.feedrate)}")
        self._emit_angles(j1, j2, j3, j4, j5, j6, 0.5)

    def calcular_tempo_trajetoria(self, x, y, z, feedrate=800):
        """Estima o tempo (s) da trajetória pela distância percorrida em cada segmento dividida pelo feedrate."""
        tempo_s = np.zeros(len(x) - 1)
        for i in range(1, len(x)):
            dx = x[i] - x[i - 1]
            dy = y[i] - y[i - 1]
            dz = z[i] - z[i - 1]
            tempo_s[i - 1] = (np.sqrt(np.pow(dx, 2) + np.pow(dy, 2) + np.pow(dz, 2)) / feedrate) * 60
        return tempo_s

    def _mover_e_aguardar(self, x, y, z, A, B, C, feedrate=800):
        """Executa o movimento e aguarda o GRBL ficar Idle; se o status falhar, usa o tempo estimado."""
        self.executar_movimento(x, y, z, A, B, C, feedrate=feedrate)
        if self.serial.wait_idle():
            return
        print("[TRAJETÓRIA] Status '?' indisponível — usando tempo estimado.")
        pausa = float(np.sum(self.calcular_tempo_trajetoria(x, y, z, feedrate=feedrate)))
        time.sleep(pausa + 1)

    # --- MÉTODOS DE PLANEJAMENTO (SEPARADOS DA EXECUÇÃO) ---

    def _enfileirar_home_interno(self):
        """Adiciona o retorno ao Home ao plano atual, partindo de P0_planejado/Ri_planejado."""
        P3 = np.array([403.3643, 0, 570.3432])
        Rf = np.array([[0, 0, 1],
                       [0, -1, 0],
                       [1, 0, 0]])

        if self.modo_juntas_planejado:
            self.enfileirar_juntas(0, 0, 0, 0, 0, 0)
            self.modo_juntas_planejado = False
            self.P0_planejado = P3.copy()
            self.Ri_planejado = Rf.copy()
            return

        x1, y1, z1 = bz.calculo_pontos(self.P0_planejado, P3, self.Ri_planejado, Rf, recuar_final=False)
        A1, B1, C1 = self.interpolar_abc(self.Ri_planejado, self.P0_planejado, Rf, P3, 21)
        self.enfileirar_movimento(x1, y1, z1, A1, B1, C1)
        self.Ri_planejado = Rf.copy()
        self.P0_planejado = P3.copy()

    def planejar_home(self):
        """Planeja o retorno ao Home via curva de Bézier (ou zero direto se em modo juntas)."""
        self.limpar_plano()
        self._enfileirar_home_interno()
        return True, "Trajetória para Home planejada. Confira no RViz2 e acione Executar."

    def planejar_juntas(self, j1, j2, j3, j4, j5, j6):
        """Planeja o movimento direto para os ângulos de juntas informados."""
        self.limpar_plano()
        self.enfileirar_juntas(j1, j2, j3, j4, j5, j6)
        self.modo_juntas_planejado = True
        R_fim, P_fim = ik.cinematica_direta(j1, j2, j3, j4, j5, j6)
        self.P0_planejado = P_fim
        self.Ri_planejado = R_fim
        return True, f"Juntas ({j1}, {j2}, {j3}, {j4}, {j5}, {j6}) planejadas (F{int(self.feedrate)})."

    def planejar_pose(self, p_alvo, usar_referencial_mesa=True, R_alvo=None):
        """Planeja uma curva de Bézier (bezier.py) da pose atual até p_alvo (mm)."""
        if self.modo_juntas:
            return False, "Modo juntas ativo — planeje e execute o Home antes de planejar trajetória cartesiana."

        self.limpar_plano()
        R_fim = self.R_baixo if R_alvo is None else R_alvo
        P3 = (np.asarray(p_alvo, dtype=float) - self.base_offset) if usar_referencial_mesa else np.asarray(p_alvo, dtype=float)

        if not self.ponto_alcancavel(P3, R_fim):
            return False, f"Ponto {p_alvo} fora de alcance pela cinemática inversa (ik_craig)."

        x, y, z = bz.calculo_pontos(self.P0_planejado, P3, self.Ri_planejado, R_fim)
        A, B, C = self.interpolar_abc(self.Ri_planejado, self.P0_planejado, R_fim, P3, 21)
        if not self._trajetoria_valida(x, y, z, A, B, C):
            x, y, z = bz.calculo_pontos(
                self.P0_planejado, P3, self.Ri_planejado, R_fim,
                recuar_inicio=False, recuar_final=False,
            )
            if not self._trajetoria_valida(x, y, z, A, B, C):
                return False, f"Curva até {np.round(p_alvo, 1).tolist()} passa fora do alcance do robô."

        self.enfileirar_movimento(x, y, z, A, B, C)
        self.P0_planejado = P3.copy()
        self.Ri_planejado = R_fim.copy()
        return True, f"Trajetória Bézier planejada para {np.round(p_alvo, 1).tolist()} (F{int(self.feedrate)})."

    def planejar_rotina_lapis(self):
        """Planeja todos os 5 estados da rotina do lápis usando bezier.py sem mover o robô."""
        if self.modo_juntas:
            return False, "Modo juntas ativo — use o Home para retornar antes de planejar a rotina."

        self.limpar_plano()
        b = self.base_offset
        P_lapis = np.array([-305, 210, 0]) - b
        P_apr_lapis = P_lapis + np.array([0, 0, 100])

        if not self.ponto_alcancavel(P_lapis):
            return False, "Posição do lápis fora de alcance."

        P_suporte = np.array([10, 120, 250]) - b
        P_apr_suporte = P_suporte + np.array([0, 0, 100])

        self.R_baixo = np.array([[0, -1, 0],
                                 [-1, 0, 0],
                                 [0, 0, -1]])

        alpha = np.rad2deg(np.arctan2(abs(P_suporte[1]), abs(P_suporte[0])))
        if b[0] > P_suporte[0]:
            gama = -1 * (180 - alpha)
        else:
            gama = -alpha
        gama_rad = np.deg2rad(gama)

        R_vertical = np.array([[-np.sin(gama_rad), 0, np.cos(gama_rad)],
                               [np.cos(gama_rad), 0, np.sin(gama_rad)],
                               [0, 1, 0]], dtype=float)

        # Estado 1: Home -> P_apr_lapis (Bézier)
        x, y, z = bz.calculo_pontos(self.P0_planejado, P_apr_lapis, self.Ri_planejado, self.R_baixo)
        A, B, C = self.interpolar_abc(self.Ri_planejado, self.P0_planejado, self.R_baixo, P_lapis, 21)
        self.enfileirar_movimento(x, y, z, A, B, C)
        self.Ri_planejado = self.R_baixo.copy()
        self.P0_planejado = P_apr_lapis.copy()

        # Estado 2: Descer, pegar lápis e recuar (Linear)
        x, y, z = bz.calculo_linear(P_apr_lapis, P_lapis, self.R_baixo)
        A, B, C = np.full(21, A[-1]), np.full(21, B[-1]), np.full(21, C[-1])
        self.enfileirar_movimento(x, y, z, A, B, C)
        self.enfileirar_garra("M97 B0 T0.2", pausa=1.0)

        x, y, z = bz.calculo_linear(P_lapis, P_apr_lapis, self.R_baixo)
        self.enfileirar_movimento(x, y, z, A, B, C)
        self.P0_planejado = P_apr_lapis.copy()

        # Estado 3: P_apr_lapis -> P_apr_suporte (Bézier)
        x, y, z = bz.calculo_pontos(self.P0_planejado, P_apr_suporte, self.Ri_planejado, R_vertical)
        A, B, C = self.interpolar_abc(self.Ri_planejado, self.P0_planejado, R_vertical, P_suporte, 21)
        self.enfileirar_movimento(x, y, z, A, B, C)
        self.Ri_planejado = R_vertical.copy()
        self.P0_planejado = P_apr_suporte.copy()

        # Estado 4: Descer no suporte, soltar e recuar (Linear)
        x, y, z = bz.calculo_linear(P_apr_suporte, P_suporte, R_vertical)
        A, B, C = np.full(21, A[-1]), np.full(21, B[-1]), np.full(21, C[-1])
        self.enfileirar_movimento(x, y, z, A, B, C)
        self.enfileirar_garra("M97 B60 T0.2", pausa=1.0)

        x, y, z = bz.calculo_linear(P_suporte, P_apr_suporte, R_vertical)
        self.enfileirar_movimento(x, y, z, A, B, C)
        self.P0_planejado = P_apr_suporte.copy()

        # Estado 5: Voltar para Home
        self._enfileirar_home_interno()
        return True, "Rotina Lápis planejada (5 estados). Confira no RViz2 e acione Executar."

    def _enfileirar_manipular_alvo(self, p0, p3, Ri, Ac, Bc, Cc, angulo=0.0):
        """Enfileira os movimentos de manipulação de um único alvo detectado pelo YOLO."""
        self.modo_juntas_planejado = False
        p3 = p3 - self.base_offset
        if not self.ponto_alcancavel(p3, self.R_baixo):
            print("Objeto não alcançável")
            return False

        rad = np.radians(angulo)
        Rz = np.array([
            [np.cos(rad), -np.sin(rad), 0],
            [np.sin(rad),  np.cos(rad), 0],
            [0,            0,           1]
        ])
        self.R_pega = Rz @ self.R_baixo
        p3_apr = p3 + np.array([0, 0, 100])

        # 1. Aproximar, descer, pegar e subir
        self.enfileirar_garra("M97 B60 T0.2", pausa=0.2)
        x, y, z = bz.calculo_pontos(p0, p3_apr, Ri, self.R_pega, recuar_inicio=(Ac is None))
        A, B, C = self.interpolar_abc(Ri, p0, self.R_pega, p3, 21, Ac, Bc, Cc)
        self.enfileirar_movimento(x, y, z, A, B, C)

        x, y, z = bz.calculo_linear(p3_apr, p3, self.R_pega)
        A, B, C = np.full(21, A[-1]), np.full(21, B[-1]), np.full(21, C[-1])
        self.enfileirar_movimento(x, y, z, A, B, C)
        self.enfileirar_garra("M97 B0 T0.2", pausa=1.0)

        x, y, z = bz.calculo_linear(p3, p3_apr, self.R_pega)
        self.enfileirar_movimento(x, y, z, A, B, C)

        self.P0_planejado = p3_apr.copy()
        self.Ri_planejado = self.R_baixo.copy()
        return True

    def planejar_manipular(self, alvos, pose_busca=(-110, 0, -20, 0, -105, 0)):
        """Planeja a manipulação de todos os alvos detectados pelo YOLO e o retorno ao Home."""
        if not alvos:
            return False, "Nenhum objeto alcançável: rode Reconhecer (YOLO) primeiro."

        self.limpar_plano()
        Rc, Pc = ik.cinematica_direta(*pose_busca)

        for i, (_, Po, angulo_alvo) in enumerate(alvos):
            p0, Ri = (Pc, Rc) if i == 0 else (self.P0_planejado, self.Ri_planejado)
            abc = (pose_busca[3], pose_busca[4], pose_busca[5]) if i == 0 else (None, None, None)
            self._enfileirar_manipular_alvo(p0, Po.copy(), Ri, *abc, angulo=angulo_alvo)

        self._enfileirar_home_interno()
        return True, f"Manipulação planejada para {len(alvos)} objeto(s). Confira no RViz2 e acione Executar."

    # --- MÉTODOS DIRETOS (COMPATIBILIDADE COM O CÓDIGO ORIGINAL) ---

    def home(self):
        """Retorna o robô à posição inicial (Home) com trajetória Bézier e atualiza o estado."""
        self.planejar_home()
        self.executar_plano()

    def recuperar_do_log(self):
        """Se o último G1 do log não for tudo zero, envia o inverso para
        desfazer o deslocamento e reancora o zero (G92)."""
        try:
            with open(GCODE_LOG) as f:
                linhas = f.read().splitlines()
        except (FileNotFoundError, OSError):
            return

        padrao = re.compile(r'G1\s+X(-?[\d.]+)\s+Y(-?[\d.]+)\s+Z(-?[\d.]+)\s+A(-?[\d.]+)\s+B(-?[\d.]+)\s+C(-?[\d.]+)')
        ultimo = None
        for linha in reversed(linhas):
            m = padrao.search(linha)
            if m:
                ultimo = m
                break
        if not ultimo:
            return

        vals = [float(v) for v in ultimo.groups()]
        if all(v == 0 for v in vals):
            return

        neg = [0.0 if v == 0 else round(-v, 2) for v in vals]
        self.serial.send(f"G1 X{neg[0]:g} Y{neg[1]:g} Z{neg[2]:g} A{neg[3]:g} B{neg[4]:g} C{neg[5]:g} F800")
        self.serial.send("G92 X0 Y0 Z0 A0 B0 C0")
        self.serial.send("G1 X0 Y0 Z0 A0 B0 C0 F800")

    def rotina_lapis_suporte(self):
        """Máquina de estados que pega o lápis da mesa e o encaixa no suporte."""
        ok, _ = self.planejar_rotina_lapis()
        if ok:
            self.executar_plano()

    def rotina_captura_calibracao(self, cap):
        b = self.base_offset
        c = np.array([-300, 210, 0]) - b
        pontos = calc_semi_esfera(c, r=500)
        poses_salvas = {}

        for i, P_atual in enumerate(pontos):
            try:
                x, y, z = ik.calculo_angulos(P_atual[0], P_atual[1], P_atual[2])
                if not (-100 <= z <= 50):
                    continue
            except Exception:
                continue

            Z_e = c - P_atual
            Z_hat = Z_e / np.linalg.norm(Z_e)

            up = np.array([0, 0, 1])
            Y_e = up - np.dot(up, Z_hat) * Z_hat
            if np.linalg.norm(Y_e) < 1e-4:
                up = np.array([0, 1, 0])
                Y_e = up - np.dot(up, Z_hat) * Z_hat
            Y_hat = Y_e / np.linalg.norm(Y_e)

            X_hat = np.cross(Y_hat, Z_hat)
            X_hat = X_hat / np.linalg.norm(X_hat)

            R_matrix = np.column_stack((X_hat, Y_hat, Z_hat))
            try:
                A, B, C = ik.calculo_angulos_abc(R_matrix, P_atual, compensar_de=False)
            except Exception:
                continue
            C = 0
            self.enviar_juntas(x, y, z, A, B, C)
            if i == 0:
                time.sleep(15)
            else:
                delta_graus = np.max(np.abs(np.array([x, y, z]) - np.array([x_ant, y_ant, z_ant])))
                tempo_espera = max(4.0, (delta_graus / 800.0) * 60 * 1.2)
                time.sleep(tempo_espera)
            x_ant, y_ant, z_ant = x, y, z

            print("Realizando a captura de imagem da calibração...")
            ret, frame = cap.read()
            if ret:
                CAPTURAS_DIR.mkdir(parents=True, exist_ok=True)
                P_mesa = P_atual + self.base_offset
                nome_arquivo = f"pos(x={P_mesa[0]:.2f}__y={P_mesa[1]:.2f}__z={P_mesa[2]:.2f}).jpg"
                caminho_arquivo = str(CAPTURAS_DIR / nome_arquivo)
                sucesso = cv2.imwrite(caminho_arquivo, frame)
                if sucesso:
                    print(f"Imagem salva com sucesso como {caminho_arquivo}")
                    R_fk, P_fk = ik.cinematica_direta(x, y, z, A, B, C)
                    T = np.eye(4)
                    T[:3, :3] = R_fk
                    T[:3, 3] = P_fk
                    poses_salvas[nome_arquivo] = {"T_flange_base": T.tolist()}
                else:
                    print(f"Erro: OpenCV falhou ao salvar {caminho_arquivo}")
            else:
                print("Erro: Falha ao ler o frame da câmera já aberta.")
            time.sleep(2)

        CAPTURAS_DIR.mkdir(parents=True, exist_ok=True)
        with open(CAPTURAS_DIR / "poses_calibracao.json", "w", encoding="utf-8") as f:
            json.dump(poses_salvas, f, indent=4)
        self.enviar_juntas(0, 0, 0, 0, 0, 0)
        self.modo_juntas = False

    def rotina_manipular(self, p0, p3, Ri, Ac, Bc, Cc, angulo=0.0):
        self.limpar_plano()
        if self._enfileirar_manipular_alvo(p0, p3, Ri, Ac, Bc, Cc, angulo=angulo):
            self.executar_plano()
