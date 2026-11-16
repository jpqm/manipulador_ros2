import serial
import time
import os
from config import GRBL_SETTINGS, GCODE_LOG

class SerialDriver:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, on_status=None):
        """Cria a conexão serial real ou simulada e abre o arquivo de log de G-Code."""
        self.on_status = on_status
        self._wco = [0.0] * 6
        self._rx_buf = b""
        self._ultimo_poll = 0.0
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)  # espera GRBL iniciar
        except (serial.SerialException, OSError):
            from mock_serial import MockSerialGRBL
            print(f"[SerialDriver] Dispositivo na porta {port} não encontrado. Usando MockSerialGRBL.")
            self.ser = MockSerialGRBL(port=port, baudrate=baudrate)

        while self.ser.in_waiting:
            line = self.ser.readline().decode().strip()
            if line:
                print("BOOT:", line)
        self.ser.timeout = 0.05

        os.makedirs(os.path.dirname(os.path.abspath(GCODE_LOG)), exist_ok=True)
        self._log = open(GCODE_LOG, "a")

    def _ler_linha(self):
        """Lê uma linha completa da serial sem quebrar pacotes em caso de timeout curto."""
        chunk = self.ser.readline()
        if not chunk:
            return ""
        if not chunk.endswith(b"\n"):
            self._rx_buf += chunk
            return ""
        linha = (self._rx_buf + chunk).decode(errors="ignore").strip()
        self._rx_buf = b""
        return linha

    def _processar_status(self, linha):
        """Extrai WPos (ou MPos - WCO) de relatórios <Run|...> / <Idle|...> do GRBL."""
        if not (linha.startswith("<") and linha.endswith(">")):
            return
        wpos = None
        mpos = None
        for campo in linha[1:-1].split("|"):
            try:
                if campo.startswith("WCO:"):
                    vals = [float(v) for v in campo[4:].split(",")]
                    if len(vals) >= 6:
                        self._wco = vals[:6]
                elif campo.startswith("WPos:"):
                    vals = [float(v) for v in campo[5:].split(",")]
                    if len(vals) >= 6:
                        wpos = vals[:6]
                elif campo.startswith("MPos:"):
                    vals = [float(v) for v in campo[5:].split(",")]
                    if len(vals) >= 6:
                        mpos = vals[:6]
            except ValueError:
                continue
        pos = wpos if wpos is not None else (
            [mpos[i] - self._wco[i] for i in range(6)] if mpos is not None else None
        )
        if pos is not None and self.on_status:
            self.on_status(*pos)

    def _poll_status(self, intervalo=0.04):
        agora = time.monotonic()
        if agora - self._ultimo_poll >= intervalo and not getattr(self.ser, "in_waiting", 0):
            self.ser.write(b"?")
            self._ultimo_poll = agora

    def send(self, cmd):
        """Envia um comando G-Code, grava no log e aguarda a resposta 'ok' do GRBL."""
        print(f">> {cmd}")
        self._log.write(cmd + "\n")
        self._log.flush()
        consultar = cmd.startswith(("G1", "G0"))
        self.ser.write((cmd + '\n').encode())
        while True:
            response = self._ler_linha()
            if response.startswith("<"):
                self._processar_status(response)
                continue
            if response:
                print("<<", response)
            if response == 'ok':
                break
            if consultar and self.on_status:
                self._poll_status()

    def wait_idle(self, intervalo=0.04, timeout_status=1.0):
        """Consulta o status ('?') até o GRBL reportar Idle, emitindo a posição em tempo real."""
        t_ultimo_status = time.monotonic()
        while True:
            if not getattr(self.ser, "in_waiting", 0):
                time.sleep(intervalo)
                if not getattr(self.ser, "in_waiting", 0):
                    self._poll_status(intervalo=intervalo)
            linha = self._ler_linha()
            if not linha:
                if time.monotonic() - t_ultimo_status > timeout_status:
                    print("[SerialDriver] Status '?' indisponível (timeout).")
                    return False
                continue
            if linha.startswith("<"):
                t_ultimo_status = time.monotonic()
                self._processar_status(linha)
                if linha.startswith("<Idle"):
                    return True
                if not linha.startswith(("<Run", "<Jog", "<Hold", "<Home")):
                    print(f"[SerialDriver] Status inesperado: {linha!r}")
                    return False

    def send_settings(self):
        """Envia os parâmetros de configuração ($) do GRBL para a placa."""
        for key, value in GRBL_SETTINGS.items():
            self.send(f"${key} = {value}")

    def reset_log(self):
        """Apaga o log, mantendo o arquivo vazio para a sessão atual."""
        self._log.seek(0)
        self._log.truncate()

    def close(self):
        """Fecha a conexão serial e o arquivo de log."""
        if self.ser:
            self.ser.close()
        self._log.close()