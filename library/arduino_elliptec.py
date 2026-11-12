"""
Interface compatível com a Thorlabs Elliptec para as lâminas no Arduino.

Imita os métodos do pylablib ElliptecMotor (e do EASYMOTOR.EasyMotor do
bell-measurements) usados nos códigos do laboratório, para trocar os motores
Thorlabs pelas lâminas do Arduino mudando só o começo do código:

    # from EASYMOTOR import EasyMotor                 # Thorlabs
    from arduino_elliptec import EasyMotor            # Arduino

    stage = EasyMotor("COM4", axes={0: "X", 2: "Y"})  # addr Elliptec → eixo
    stage.home(addr=[0, 2])                           # igual à Thorlabs
    stage.move_to(45, addr=2)
    stage.close()

Endereços: na Elliptec cada montagem tem um endereço (0, 1, 2, ... ou "0"
a "F") no mesmo cabo; aqui cada endereço vira um eixo do CNC Shield na mesma
porta. Sem `axes`, vale o mapa padrão DEFAULT_AXES (0→X, 1→Y, 2→Z, 3→A) com
os eixos que o firmware tiver.

Diferenças em relação à Thorlabs:
  * home() leva mais tempo (procura o fim de curso devagar) e faz todos os
    endereços pedidos ao mesmo tempo;
  * home_dir (home) e max_tries (move_to) são aceitos só por compatibilidade:
    o sentido da busca vem do firmware, e a serial não precisa de novas
    tentativas;
  * o zero é onde o fim de curso aciona, não o zero de fábrica da Thorlabs:
    os ângulos de calibração (H_alice, H_bob, ...) precisam ser medidos de novo.

Calibração de velocidade de um endereço: stage.plate(2).configure(...)
(veja waveplate.WavePlate).
"""

from __future__ import annotations

import time
from typing import Callable, Dict, List, Mapping, Optional, Sequence, Union

from waveplate import BAUD, DEFAULT_OVERSHOOT, WavePlate, WavePlateError, open_plates

__all__ = ["ArduinoElliptec", "EasyMotor", "ElliptecMotor", "DEFAULT_AXES", "WavePlateError"]

DEFAULT_AXES: Dict[int, str] = {0: "X", 1: "Y"}

Addr = Union[int, str]


def _normalize_addr(addr: Addr) -> int:
    """Aceita 0..15 ou "0".."F", como a Elliptec."""
    if isinstance(addr, str):
        try:
            value = int(addr, 16)
        except ValueError:
            raise ValueError(f"endereço inválido: {addr!r}") from None
    else:
        value = int(addr)
    if not 0 <= value <= 15:
        raise ValueError(f"endereço fora de 0..15: {addr!r}")
    return value


class ArduinoElliptec:
    """Vários eixos de um Arduino vistos como montagens Elliptec num mesmo cabo."""

    def __init__(self, conn: str, axes: Optional[Mapping[Addr, str]] = None, *,
                 default_addr: Optional[Addr] = None, baud: int = BAUD,
                 move_timeout: float = 120.0,
                 disable_when_idle: Optional[bool] = True,
                 startup_timeout: float = 5.0,
                 approach: int = 0, overshoot: float = DEFAULT_OVERSHOOT):
        """
        conn               porta serial, ex.: "COM4"
        axes               {endereço: eixo}, ex.: {0: "X", 2: "Y"}; None usa
                           DEFAULT_AXES com os eixos que o firmware tiver
        default_addr       endereço usado quando addr=None (padrão: o menor)
        move_timeout       s máximos para um movimento ou referência
        disable_when_idle  True/False aplica o modo do driver a todos os eixos;
                           None mantém o padrão do firmware (sempre habilitado)
        approach, overshoot  chegar sempre pelo mesmo sentido, em todos os
                           endereços (veja set_approach); 0 desliga
        """
        self.conn = conn
        options = dict(baud=baud, move_timeout=move_timeout,
                       disable_when_idle=disable_when_idle,
                       startup_timeout=startup_timeout,
                       approach=approach, overshoot=overshoot)

        if axes is None:
            plates = open_plates(conn, **options)
            addr_of = {axis: addr for addr, axis in DEFAULT_AXES.items()}
            self._plates: Dict[int, WavePlate] = {}
            for axis, plate in plates.items():
                if axis in addr_of:
                    self._plates[addr_of[axis]] = plate
                else:
                    plate.close()            # eixo com letra fora do mapa padrão
        else:
            mapping = {_normalize_addr(a): ax.upper() for a, ax in axes.items()}
            if len(set(mapping.values())) != len(mapping):
                raise ValueError("dois endereços apontando para o mesmo eixo")
            plates = open_plates(conn, list(mapping.values()), **options)
            self._plates = {addr: plates[axis] for addr, axis in mapping.items()}

        if not self._plates:
            raise WavePlateError("nenhum eixo disponível no firmware")
        self._default_addr = min(self._plates)
        if default_addr is not None:
            self.set_default_addr(default_addr)

    def __repr__(self) -> str:
        axes = ", ".join(f"{a}: {p.axis!r}" for a, p in sorted(self._plates.items()))
        return f"ArduinoElliptec({self.conn!r}, axes={{{axes}}})"

    # ── endereços ──────────────────────────────────────────────────────────
    def get_connected_addrs(self) -> List[int]:
        """Endereços disponíveis, em ordem."""
        return sorted(self._plates)

    get_all_addrs = get_connected_addrs

    def set_default_addr(self, addr: Addr):
        """Endereço usado quando addr=None."""
        addr = _normalize_addr(addr)
        self._check(addr)
        self._default_addr = addr

    def get_default_addr(self) -> int:
        return self._default_addr

    def plate(self, addr: Optional[Addr] = None) -> WavePlate:
        """O WavePlate de um endereço, para calibrar ou usar funções extras."""
        return self._plates[self._resolve(addr)[0][0]]

    def _check(self, addr: int):
        if addr not in self._plates:
            available = ", ".join(str(a) for a in sorted(self._plates))
            raise ValueError(f"endereço {addr} não configurado (disponíveis: {available}); "
                             f"passe axes={{...}} ao criar o objeto")

    def _resolve(self, addr) -> tuple[List[int], bool]:
        """Retorna (endereços, era_lista). Aceita None, um endereço, lista ou "all"."""
        if addr is None:
            return [self._default_addr], False
        if isinstance(addr, str) and addr.lower() == "all":
            return sorted(self._plates), True
        if isinstance(addr, (list, tuple, set)):
            addrs = [_normalize_addr(a) for a in addr]
            many = True
        else:
            addrs, many = [_normalize_addr(addr)], False
        for a in addrs:
            self._check(a)
        return addrs, many

    def _run(self, addr, start: Callable[[WavePlate], object],
             timeout: Optional[float]) -> Union[float, List[float]]:
        """
        Inicia o movimento em todos os endereços (andam juntos) e espera todos.
        Ctrl+C para todos com desaceleração.
        """
        addrs, many = self._resolve(addr)
        plates = [self._plates[a] for a in addrs]
        started: List[WavePlate] = []
        limit = timeout if timeout is not None else max(p.move_timeout for p in plates)
        try:
            for p in plates:
                start(p)
                started.append(p)
            # acompanha todos juntos: is_moving() manda o trecho final de quem
            # chegou na pré-aproximação, sem esperar os outros terminarem
            deadline = time.monotonic() + limit
            while any([p.is_moving() for p in plates]):
                if time.monotonic() > deadline:
                    raise TimeoutError(f"movimento não terminou em {limit:g} s")
                time.sleep(0.005)
            finals = [p.wait_move(timeout) for p in plates]
        except BaseException:
            for p in started:                # não deixa os outros eixos andando sozinhos
                try:
                    if p.is_moving():
                        p.stop()
                except (WavePlateError, TimeoutError):
                    pass
            raise
        return finals if many else finals[0]

    # ── comandos estilo Elliptec ───────────────────────────────────────────
    def home(self, home_dir: str = "cw", addr=None, timeout: Optional[float] = None):
        """
        Zero mecânico no fim de curso. addr pode ser um endereço, uma lista ou
        "all"; os endereços pedidos fazem a referência ao mesmo tempo.
        home_dir é aceito por compatibilidade (o sentido vem do firmware).
        """
        return self._run(addr, lambda p: p.home(wait=False), timeout)

    def move_to(self, position: float, addr=None, timeout: Optional[float] = None,
                max_tries: Optional[int] = None):
        """
        Vai até o ângulo absoluto (graus) e espera chegar. Retorna a posição
        final (lista, se addr for lista). max_tries é aceito por
        compatibilidade com EASYMOTOR.EasyMotor.
        """
        return self._run(addr, lambda p: p.move_to(position, wait=False), timeout)

    def move_by(self, distance: float, addr=None, timeout: Optional[float] = None):
        """Anda `distance` graus a partir da posição atual e espera chegar."""
        return self._run(addr, lambda p: p.move_by(distance, wait=False), timeout)

    def set_approach(self, direction: int = +1, overshoot: float = DEFAULT_OVERSHOOT, addr="all"):
        """
        Chegar sempre girando no mesmo sentido (veja WavePlate.set_approach).
        Sem addr, vale para todos os endereços; direction=0 desliga.
        """
        addrs, _ = self._resolve(addr)
        for a in addrs:
            self._plates[a].set_approach(direction, overshoot)

    def set_zero(self, addr=None):
        """Define a posição atual como 0° (veja WavePlate.set_zero)."""
        addrs, _ = self._resolve(addr)
        for a in addrs:
            self._plates[a].set_zero()

    def calibrate(self, addr=None, timeout: Optional[float] = None):
        """
        Refaz o home() e volta ao último ângulo pedido, mantendo o zero de
        software de set_zero() (veja WavePlate.calibrate). Os endereços
        pedidos fazem tudo ao mesmo tempo. Retorna a posição final (lista, se
        addr for lista).
        """
        addrs, many = self._resolve(addr)
        plates = [self._plates[a] for a in addrs]
        plans = {id(p): p._calibration_plan() for p in plates}   # confere todos antes de mexer
        self._run(addrs, lambda p: p.home(wait=False), timeout)
        zero_angle = {id(p): plans[id(p)][1] / p.get_settings().steps_per_degree for p in plates}
        self._run(addrs, lambda p: p.move_to(zero_angle[id(p)], wait=False), timeout)
        for p in plates:
            p._apply_zero(plans[id(p)][1])
        finals = self._run(addrs, lambda p: p.move_to(plans[id(p)][0], wait=False), timeout)
        return finals if many else finals[0]

    def get_position(self, addr=None):
        """Ângulo atual em graus (lista, se addr for lista)."""
        addrs, many = self._resolve(addr)
        positions = [self._plates[a].get_position() for a in addrs]
        return positions if many else positions[0]

    def stop(self, addr=None):
        """Para com desaceleração. addr=None para o endereço padrão; "all" para todos."""
        addrs, many = self._resolve(addr)
        finals = [self._plates[a].stop() for a in addrs]
        return finals if many else finals[0]

    def is_moving(self, addr=None):
        addrs, many = self._resolve(addr)
        moving = [self._plates[a].is_moving() for a in addrs]
        return moving if many else moving[0]

    def wait_move(self, addr=None, timeout: Optional[float] = None):
        addrs, many = self._resolve(addr)
        finals = [self._plates[a].wait_move(timeout) for a in addrs]
        return finals if many else finals[0]

    # ── conexão ────────────────────────────────────────────────────────────
    def is_opened(self) -> bool:
        return any(not p.closed for p in self._plates.values())

    def close(self, disable: bool = True):
        """Para tudo, desliga os drivers (se pedido) e libera a porta."""
        for p in self._plates.values():
            p.close(disable)

    def __enter__(self) -> "ArduinoElliptec":
        return self

    def __exit__(self, *exc):
        self.close()


# Nomes usados nos códigos da Thorlabs, para trocar só a linha do import.
EasyMotor = ArduinoElliptec
ElliptecMotor = ArduinoElliptec
