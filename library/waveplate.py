"""
Biblioteca para controlar as lâminas motorizadas pela serial.

O Arduino (firmware em firmware/src/main.cpp) gera os passos e a aceleração; esta
biblioteca só manda comandos e espera as respostas. Ângulos em graus da
lâmina, velocidades em graus/s e aceleração em graus/s².

Requer:  pip install pyserial

Uso básico (eixo X):

    from waveplate import WavePlate

    with WavePlate("COM4") as plate:
        plate.home()                      # zero mecânico no fim de curso
        plate.move_to(45)                 # ângulo absoluto, espera chegar
        plate.move_by(-10)                # relativo à posição atual
        print(plate.get_position())

Vários eixos na mesma porta (um objeto WavePlate por eixo):

    with WavePlate("COM4", "X") as x, WavePlate("COM4", "Y") as y:
        x.move_to(90, wait=False)         # os dois andam juntos
        y.move_to(45, wait=False)
        x.wait_move()
        y.wait_move()

    plates = open_plates("COM4")          # todos os eixos do firmware: {"X": ..., "Y": ...}

    Os eixos disponíveis vêm do firmware (EIXOS_CFG em firmware/src/main.cpp).

Zero de software e recalibração:

    plate.home()                          # zero mecânico (uma vez por conexão)
    plate.move_by(50)
    plate.set_zero()                      # aqui passa a ser 0°
    plate.move_by(20)                     # 20° no novo referencial
    plate.calibrate()                     # refaz o home e volta aos 20°, com o
                                          # zero de software no mesmo lugar
    print(plate.zero_offset)              # 50.0: zero de software a partir do mecânico

Chegar sempre pelo mesmo sentido (folga e histerese sempre do mesmo lado):

    plate = WavePlate("COM4", approach=+1, overshoot=10)   # ou plate.set_approach(+1, 10)
    plate.move_to(30)                     # de 0 a 30: já chega subindo, vai direto
    plate.move_to(20)                     # de 30 a 20: passa por 10° e sobe até 20°
    plate.set_approach(0)                 # desliga

Calibração (vale só para o eixo do objeto):

    plate.configure(min_speed=20, max_speed=90, acceleration=200)
    print(plate.get_settings())
    plate.restore_defaults()              # volta aos valores do firmware

Movimento sem bloquear:

    plate.move_to(180, wait=False)
    while plate.is_moving():
        print(plate.get_position())
        time.sleep(0.1)
    final = plate.wait_move()             # ou chame só wait_move() direto

Para usar no lugar da Thorlabs Elliptec, veja arduino_elliptec.py.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, Iterable, Optional

import serial

BAUD = 115200
DEFAULT_OVERSHOOT = 10.0     # graus de chegada no mesmo sentido (set_approach)

__all__ = ["WavePlate", "Settings", "WavePlateError", "open_plates", "BAUD", "DEFAULT_OVERSHOOT"]


class WavePlateError(Exception):
    """Erro informado pelo Arduino (resposta "ERR ...")."""


@dataclass(frozen=True)
class Settings:
    steps_per_degree: float
    min_speed: float         # graus/s — nunca anda mais devagar que isso
    max_speed: float         # graus/s
    acceleration: float      # graus/s²
    disable_delay_ms: int    # 0 = driver sempre habilitado
    min_angle: float         # limites aceitos pelo firmware
    max_angle: float
    speed_limit: float       # graus/s — teto do Arduino para max_speed

    def __str__(self) -> str:
        driver = ("sempre habilitado" if self.disable_delay_ms == 0
                  else f"desliga {self.disable_delay_ms} ms após parar")
        return (f"velocidade {self.min_speed:.2f} a {self.max_speed:.2f} °/s "
                f"(máx. possível {self.speed_limit:.2f} °/s), "
                f"aceleração {self.acceleration:.2f} °/s², "
                f"limites {self.min_angle:g}° a {self.max_angle:g}°, "
                f"driver {driver}, {self.steps_per_degree:.4f} passos/°")


# ── porta compartilhada entre eixos ───────────────────────────────────────────
class _SharedPort:
    """
    Uma porta serial usada por vários eixos. O firmware põe a letra do eixo
    na frente de cada resposta ("X DONE 45.00"); aqui elas são separadas numa
    fila por eixo, já sem a letra.
    """

    def __init__(self, port: str, baud: int, startup_timeout: float):
        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.users = 0
        self._lock = threading.Lock()
        self._buffer = bytearray()
        self._queues: dict[str, deque[str]] = {}
        self._ready: Optional[str] = None
        try:
            self.axes = self._wait_ready(startup_timeout)
        except Exception:
            self.ser.close()
            raise
        self._queues = {axis: deque() for axis in self.axes}
        # Onde está o 0 do firmware, em passos a partir do fim de curso (zero
        # mecânico), por eixo. None até o primeiro home() desta conexão: ao
        # ligar, o 0 é onde a lâmina estava. Fica aqui porque é estado do
        # Arduino, comum a todos os objetos do mesmo eixo.
        self.zero_steps: dict[str, Optional[int]] = {axis: None for axis in self.axes}

    def _wait_ready(self, timeout: float) -> tuple[str, ...]:
        if not self._await_ready_line(timeout):
            # placa que não reinicia ao abrir a porta: pergunta direto
            self.ser.write(b"?\n")
            if not self._await_ready_line(1.0):
                raise TimeoutError("sem resposta READY do Arduino")
        axes = tuple(self._ready.split()[1:])
        if not axes:
            raise WavePlateError("firmware sem suporte a eixos — grave o firmware atualizado")
        return axes

    def _await_ready_line(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while self._ready is None and time.monotonic() < deadline:
            self._receive(block=True)
        return self._ready is not None

    def _receive(self, block: bool):
        """Lê o que chegou e distribui as linhas completas entre os eixos."""
        n = self.ser.in_waiting
        if n == 0 and not block:
            return
        data = self.ser.read(max(1, n))      # bloqueia no máximo ser.timeout
        if not data:
            return
        self._buffer += data
        *lines, self._buffer = self._buffer.split(b"\n")
        for raw in lines:
            line = raw.decode("ascii", errors="replace").strip()
            if line.startswith("READY"):
                self._ready = line
                continue
            axis, _, rest = line.partition(" ")
            queue = self._queues.get(axis)
            if queue is not None:
                queue.append(rest)
            # linhas sem eixo conhecido ("? ERR ...", lixo do reset) são ignoradas

    def send(self, axis: str, command: str):
        with self._lock:
            self.ser.write(f"{axis} {command}\n".encode("ascii"))

    def read_line(self, axis: str, timeout: float) -> Optional[str]:
        deadline = time.monotonic() + timeout
        with self._lock:
            queue = self._queues[axis]
            while True:
                if queue:
                    return queue.popleft()
                if time.monotonic() >= deadline:
                    return None
                self._receive(block=True)

    def clear(self, axis: str):
        """Descarta as respostas pendentes só deste eixo."""
        with self._lock:
            self._receive(block=False)
            self._queues[axis].clear()

    def pending(self, axis: str) -> int:
        with self._lock:
            self._receive(block=False)
            return len(self._queues[axis])


_ports: dict[str, _SharedPort] = {}
_ports_lock = threading.Lock()


def _acquire_port(port: str, baud: int, startup_timeout: float) -> _SharedPort:
    """Abre a porta na primeira vez; os próximos eixos reaproveitam a conexão."""
    with _ports_lock:
        p = _ports.get(port)
        if p is None or not p.ser.is_open:
            p = _SharedPort(port, baud, startup_timeout)
            _ports[port] = p
        p.users += 1
        return p


def _release_port(port: str):
    """Fecha a porta quando o último eixo que a usa for fechado."""
    with _ports_lock:
        p = _ports.get(port)
        if p is None:
            return
        p.users -= 1
        if p.users <= 0:
            del _ports[port]
            p.ser.close()


# ── lâmina (um eixo) ──────────────────────────────────────────────────────────
class WavePlate:
    """
    Conexão com um eixo. O primeiro objeto aberto numa porta reinicia o
    Arduino; os outros eixos da mesma porta compartilham a conexão.
    """

    def __init__(self, port: str, axis: str = "X", *, baud: int = BAUD,
                 move_timeout: float = 120.0,
                 disable_when_idle: Optional[bool] = None,
                 startup_timeout: float = 5.0,
                 approach: int = 0, overshoot: float = DEFAULT_OVERSHOOT):
        """
        port               ex.: "COM4" ou "/dev/ttyUSB0"
        axis               letra do eixo no firmware ("X", "Y", ...)
        move_timeout       s máximos para um movimento ou referência
        disable_when_idle  True/False aplica o modo do driver ao conectar;
                           None mantém o padrão do firmware
        approach, overshoot  chegar sempre pelo mesmo sentido (veja set_approach);
                           approach=0 (padrão) desliga
        """
        self.axis = axis.upper()
        self.port = port
        self.move_timeout = move_timeout
        self._moving = False
        self._homing = False
        self._done: Optional[str] = None     # DONE que chegou fora de hora
        self._target: Optional[float] = None # último ângulo pedido, sem arredondar
        self._closed = False
        self._legs: deque[float] = deque()   # alvos que faltam do movimento atual
        self._approached = False             # já está chegando pelo sentido certo?
        self.set_approach(approach, overshoot)

        self._serial = _acquire_port(port, baud, startup_timeout)
        try:
            if self.axis not in self._serial.axes:
                raise ValueError(f"eixo {self.axis!r} não existe no firmware "
                                 f"(disponíveis: {', '.join(self._serial.axes)})")
            self._settings = self._read_settings()
            self._defaults = self._settings
            if disable_when_idle is not None:
                self.disable_when_idle(disable_when_idle)
        except Exception:
            _release_port(port)
            raise

    def __repr__(self) -> str:
        return f"WavePlate({self.port!r}, {self.axis!r})"

    # ── comunicação ────────────────────────────────────────────────────────
    def _send(self, command: str):
        self._serial.send(self.axis, command)

    def _wait_for(self, prefixes, timeout: float, ignore_errors: bool = False) -> str:
        """Lê linhas deste eixo até achar uma que comece com um dos prefixos."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self._serial.read_line(self.axis, 0.05)
            if line is None:
                continue
            if line.startswith("DONE"):
                if self._homing:             # referência terminou bem (se falhar, vem ERR antes)
                    self._serial.zero_steps[self.axis] = 0
                self._moving = self._homing = False
                if "DONE" not in prefixes:
                    self._done = line        # fim de um movimento sem bloqueio
                    continue
            if line.startswith("ERR"):
                if self._homing:             # a referência falhou: o eixo parou
                    self._moving = self._homing = False
                if ignore_errors:
                    continue
                raise WavePlateError(f"[eixo {self.axis}] {line[4:]}")
            if line.startswith(prefixes):
                return line
        raise TimeoutError(f"[eixo {self.axis}] sem resposta {prefixes} do Arduino")

    def _command(self, command: str, reply: str = "OK", timeout: float = 1.0) -> str:
        if not self._moving:
            self._serial.clear(self.axis)    # descarta sobras de comandos antigos
        self._send(command)
        return self._wait_for((reply,), timeout)

    @staticmethod
    def _number(line: str) -> float:
        return float(line.split()[1])

    def _require_idle(self):
        if self._moving or self._legs:
            raise WavePlateError(f"[eixo {self.axis}] em movimento (use wait_move() ou stop())")

    # ── movimento ──────────────────────────────────────────────────────────
    def move_to(self, angle: float, wait: bool = True) -> float:
        """
        Vai até o ângulo absoluto (graus). Com wait=True bloqueia até parar e
        retorna a posição final; Ctrl+C para a lâmina com desaceleração.
        Com wait=False retorna na hora o ângulo-alvo real (arredondado ao
        passo) e o movimento segue; use wait_move() ou is_moving().
        Com set_approach() ligado, pode passar antes por um ponto de
        pré-aproximação (o movimento só termina no alvo).
        """
        self._require_idle()
        s = self._settings
        if not s.min_angle <= angle <= s.max_angle:
            raise ValueError(f"[eixo {self.axis}] ângulo {angle:g}° fora dos limites "
                             f"({s.min_angle:g}° a {s.max_angle:g}°)")
        legs = self._plan_legs(angle)
        self._done = None
        target = self._number(self._command(f"G {legs[0]:.4f}"))
        self._moving = True
        self._target = angle
        self._legs = deque(legs[1:])
        if self._legs:
            target = round(round(angle * s.steps_per_degree) / s.steps_per_degree, 2)
        if not wait:
            return target
        return self.wait_move()

    def _plan_legs(self, angle: float) -> list[float]:
        """
        [alvo] ou [pré-aproximação, alvo]. Com approach ligado, o movimento
        termina andando no sentido escolhido, num trecho de pelo menos
        overshoot graus; movimentos curtos no mesmo sentido, logo depois de uma
        chegada correta, vão direto (não há inversão no meio).
        """
        d, length = self._approach, self._overshoot
        if d == 0:
            return [angle]
        here = self._target if self._target is not None else self.get_position()
        delta = angle - here
        if abs(delta) < 1e-9:
            return [angle]
        if d * delta > 0 and (d * delta >= length or self._approached):
            self._approached = True
            return [angle]
        pre = angle - d * length
        s = self._settings
        if not s.min_angle <= pre <= s.max_angle:   # sem espaço perto do limite: vai direto
            self._approached = False
            return [angle]
        self._approached = True
        return [pre, angle]

    def _next_leg(self):
        """Chegou na pré-aproximação: manda o trecho final."""
        self._done = None
        self._command(f"G {self._legs.popleft():.4f}")
        self._moving = True

    def set_approach(self, direction: int = +1, overshoot: float = DEFAULT_OVERSHOOT):
        """
        Faz todo move_to()/move_by() chegar ao alvo girando no mesmo sentido
        (+1 ou -1), depois de pelo menos `overshoot` graus nesse sentido. Se o
        movimento vier do outro lado (ou for curto demais logo depois de uma
        inversão), a lâmina passa antes por alvo − direction·overshoot. Assim a
        folga e a histerese das engrenagens ficam sempre do mesmo lado.
        direction=0 desliga (cada movimento vai direto).
        """
        if direction not in (-1, 0, 1):
            raise ValueError("direction precisa ser +1, -1 ou 0")
        if direction and not overshoot > 0:
            raise ValueError("overshoot precisa ser maior que 0")
        self._require_idle()
        self._approach, self._overshoot = int(direction), float(overshoot)
        self._approached = False

    def get_approach(self) -> tuple[int, float]:
        """(sentido, overshoot) da chegada; sentido 0 = desligado."""
        return self._approach, self._overshoot

    def move_by(self, delta: float, wait: bool = True) -> float:
        """
        Anda delta graus a partir da posição atual (positivo ou negativo).
        Parte do último ângulo pedido, para vários passos relativos seguidos
        não acumularem o arredondamento da posição informada pelo Arduino.
        """
        self._require_idle()
        base = self.get_position() if self._target is None else self._target
        return self.move_to(base + delta, wait)

    def home(self, wait: bool = True) -> float:
        """
        Procura o fim de curso e define o zero mecânico. Com wait=True bloqueia
        até terminar (Ctrl+C cancela); com wait=False segue como um movimento
        comum (wait_move(), is_moving()). Os outros eixos podem continuar andando.
        """
        self._require_idle()
        self._target = None
        self._done = None
        self._approached = False
        self._command("R")
        self._moving = self._homing = True
        if not wait:
            return self.get_position()
        return self.wait_move()

    def wait_move(self, timeout: Optional[float] = None) -> float:
        """Espera o movimento (ou referência) em curso terminar e retorna a posição final."""
        try:
            while True:
                if self._done is None and self._moving:
                    self._done = self._wait_for(("DONE",), timeout or self.move_timeout)
                if self._legs and (self._done is not None or not self._moving):
                    self._next_leg()         # chegou na pré-aproximação
                    continue
                break
        except KeyboardInterrupt:
            self.stop()
            raise
        if self._done is None:
            return self.get_position()
        final, self._done = self._number(self._done), None
        return final

    def is_moving(self) -> bool:
        """True enquanto um movimento iniciado com wait=False não terminou."""
        if self._moving and self._serial.pending(self.axis):
            try:
                self._wait_for(("\0",), 0.05)  # só processa o que já chegou
            except TimeoutError:
                pass
        if not self._moving and self._legs:
            self._next_leg()                 # chegou na pré-aproximação
        return self._moving

    def stop(self) -> float:
        """Para com desaceleração (ou cancela a referência) e retorna onde parou."""
        self._legs.clear()
        self._approached = False
        self._send("S")
        self._done = None
        final = self._number(self._wait_for(("DONE",), 10.0, ignore_errors=True))
        self._moving = self._homing = False
        self._target = None                  # parou no meio do caminho
        return final

    # ── estado ─────────────────────────────────────────────────────────────
    def get_position(self) -> float:
        """Ângulo atual em graus (funciona também durante o movimento)."""
        return self._number(self._command("P", "POS"))

    def set_zero(self):
        """
        Define a posição atual como 0° (sem procurar o fim de curso). Se já
        houve home() nesta conexão, guarda onde fica esse zero em relação ao
        zero mecânico, para calibrate() poder restaurá-lo.
        """
        self._require_idle()
        zero = self._serial.zero_steps[self.axis]
        if zero is not None:
            zero += round(self.get_position() * self._settings.steps_per_degree)
        self._command("Z", "POS")
        self._serial.zero_steps[self.axis] = zero
        self._target = 0.0

    @property
    def zero_offset(self) -> Optional[float]:
        """
        Onde está o 0° atual, em graus a partir do zero mecânico (fim de
        curso). 0 logo depois de home(); None se ainda não houve home() nesta
        conexão (aí o 0° é onde a lâmina estava ao ligar).
        """
        zero = self._serial.zero_steps[self.axis]
        return None if zero is None else zero / self._settings.steps_per_degree

    def calibrate(self) -> float:
        """
        Recalibra sem mudar o referencial: refaz o home() (zero mecânico), volta
        ao zero de software definido por set_zero() e então ao último ângulo
        pedido. Retorna a posição final. Use para corrigir passos perdidos ou
        histerese acumulada. Precisa de um home() antes do set_zero(); sem
        set_zero(), equivale a home() + move_to(último ângulo).
        """
        target, zero = self._calibration_plan()
        self.home()
        if zero:
            self.move_to(zero / self._settings.steps_per_degree)
        self._apply_zero(zero)
        return self.move_to(target)

    def _calibration_plan(self) -> tuple[float, int]:
        """
        (ângulo a restaurar, passos do zero de software a partir do mecânico).
        O zero vai para a volta mais próxima do fim de curso: o home() só acha o
        fim de curso, não quantas voltas a lâmina deu, e assim o caminho é curto.
        """
        self._require_idle()
        zero = self._serial.zero_steps[self.axis]
        if zero is None:
            raise WavePlateError(f"[eixo {self.axis}] zero mecânico desconhecido: faça home() "
                                 f"depois de conectar e só então set_zero()")
        target = self._target if self._target is not None else self.get_position()
        turn = round(360.0 * self._settings.steps_per_degree)
        zero = (zero + turn // 2) % turn - turn // 2
        return target, zero

    def _apply_zero(self, zero: int):
        """Parado sobre o zero de software logo após o home(): ele vira o 0 do firmware."""
        if zero:
            self._command("Z", "POS")
            self._target = 0.0
        self._serial.zero_steps[self.axis] = zero

    def limit_switch_triggered(self) -> bool:
        """True se o fim de curso está acionado."""
        return self._command("F", "FIM").split()[1] == "1"

    # ── driver ─────────────────────────────────────────────────────────────
    def enable(self):
        """Liga o driver: o motor segura a posição com força."""
        self._command("H")

    def disable(self):
        """
        Desliga o driver: motor solto, sem torque e sem aquecer. No CNC Shield
        o ENABLE é um só para todos os eixos, então o driver só desliga de fato
        quando todos os eixos estiverem desabilitados.
        """
        self._require_idle()
        self._command("D")

    def disable_when_idle(self, active: bool, delay_ms: int = 200):
        """
        True: o driver desliga delay_ms depois de cada movimento e religa
        sozinho no próximo. False: fica sempre habilitado.
        """
        self._command(f"T {int(delay_ms) if active else 0}")
        self._settings = self._read_settings()

    # ── calibração ─────────────────────────────────────────────────────────
    def get_settings(self) -> Settings:
        """Valores atuais lidos do Arduino."""
        self._settings = self._read_settings()
        return self._settings

    def configure(self, min_speed: Optional[float] = None,
                  max_speed: Optional[float] = None,
                  acceleration: Optional[float] = None) -> Settings:
        """
        Muda velocidades (graus/s) e aceleração (graus/s²). Só os valores
        passados mudam. Retorna a configuração aplicada.

        min_speed: velocidade de arranque e de chegada; também é a velocidade
                   da busca do zero. Abaixo dela o motor tende a vibrar.
        max_speed: velocidade de cruzeiro (até speed_limit).
        """
        self._require_idle()
        s = self._settings
        new_min = s.min_speed if min_speed is None else min_speed
        new_max = s.max_speed if max_speed is None else max_speed
        if new_min <= 0:
            raise ValueError("min_speed precisa ser maior que 0")
        if new_min > new_max:
            raise ValueError("min_speed não pode ser maior que max_speed")
        if new_max > s.speed_limit:
            raise ValueError(f"max_speed acima do limite do Arduino ({s.speed_limit:.2f} °/s)")
        if acceleration is not None and acceleration <= 0:
            raise ValueError("acceleration precisa ser maior que 0")

        spd = s.steps_per_degree
        # O firmware exige min <= max a cada comando: a ordem importa.
        if new_min <= s.max_speed:
            self._command(f"M {new_min * spd:.2f}")
            self._command(f"V {new_max * spd:.2f}")
        else:
            self._command(f"V {new_max * spd:.2f}")
            self._command(f"M {new_min * spd:.2f}")
        if acceleration is not None:
            self._command(f"A {acceleration * spd:.2f}")
        return self.get_settings()

    def restore_defaults(self) -> Settings:
        """Volta velocidades e aceleração aos valores que o firmware tinha ao conectar."""
        d = self._defaults
        return self.configure(d.min_speed, d.max_speed, d.acceleration)

    def _read_settings(self) -> Settings:
        try:
            line = self._command("I", "INFO")
        except WavePlateError as e:
            raise WavePlateError(f"{e} — grave o firmware atualizado (comando I)") from None
        spd, vmin, vmax, accel, delay, amin, amax, vlim = (float(x) for x in line.split()[1:9])
        return Settings(
            steps_per_degree=spd,
            min_speed=vmin / spd,
            max_speed=vmax / spd,
            acceleration=accel / spd,
            disable_delay_ms=int(delay),
            min_angle=amin,
            max_angle=amax,
            speed_limit=vlim / spd,
        )

    # ── conexão ────────────────────────────────────────────────────────────
    def close(self, disable: bool = True):
        """Para este eixo, desabilita o driver dele (se pedido) e solta a porta."""
        if self._closed:
            return
        self._closed = True
        if self._serial.ser.is_open:
            try:
                if self._moving:
                    self.stop()
                if disable:
                    self._command("D")
            except (WavePlateError, TimeoutError, serial.SerialException):
                pass
        _release_port(self.port)

    @property
    def closed(self) -> bool:
        return self._closed

    def __enter__(self) -> "WavePlate":
        return self

    def __exit__(self, *exc):
        self.close()


def open_plates(port: str, axes: Optional[Iterable[str]] = None, **kwargs) -> Dict[str, WavePlate]:
    """
    Abre vários eixos da mesma porta de uma vez e retorna {letra: WavePlate}.
    axes=None abre todos os eixos que o firmware tiver. kwargs vão para cada
    WavePlate (move_timeout, disable_when_idle, ...).
    """
    baud = kwargs.get("baud", BAUD)
    startup_timeout = kwargs.get("startup_timeout", 5.0)
    shared = _acquire_port(port, baud, startup_timeout)   # segura a porta aberta
    plates: Dict[str, WavePlate] = {}
    try:
        for axis in (shared.axes if axes is None else axes):
            plates[axis.upper()] = WavePlate(port, axis, **kwargs)
    except Exception:
        for p in plates.values():
            p.close()
        raise
    finally:
        _release_port(port)
    return plates
