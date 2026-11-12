"""
Bateria de testes de hardware + software das lâminas motorizadas.

Roda contra o Arduino de verdade (firmware em firmware/src/main.cpp) usando waveplate.py e
arduino_elliptec.py. Não precisa de óptica: a referência externa é o próprio
fim de curso. A "borda" (o ângulo comandado em que ele passa a ler acionado,
chegando sempre pelo mesmo lado) é medida com resolução de um micropasso e
serve para conferir passos perdidos, escala (passos por grau) e a
repetibilidade do zero. Para medir folga das engrenagens com luz, veja
hysteresis_test.py.

Uso:
    python tests/hardware_test.py                         # COM4, lâminas X e Y
    python tests/hardware_test.py --port COM5 --plates X Y Z
    python tests/hardware_test.py --plates COM4:X COM5:X  # lâminas em Arduinos diferentes
    python tests/hardware_test.py --quick                 # pula os testes demorados
    python tests/hardware_test.py --only simultaneous speed-limit
    python tests/hardware_test.py --no-interactive        # sem perguntas (só automático)
    python tests/hardware_test.py --list                  # lista os testes

Grupos de testes (veja --list):
  básico       conexão, configurações, validação de comandos, robustez da
               serial, leitura do fim de curso, latência
  uma lâmina   referência, borda do fim de curso, escala por volta,
               posicionamento, tempos do perfil de velocidade, movimento sem
               bloquear, parada, cancelamento da referência, Ctrl+C, driver,
               configuração, zero de software + calibrate()
  duas ou mais roteamento das respostas, independência, fios dos fins de curso,
               referência simultânea, simultâneo × uma depois da outra,
               limite de velocidade somada, threads, API da Elliptec
  resistência  curso até os limites, movimentos aleatórios, reconexão

Resultado: uma tabela no fim e um JSON em test_results/.

ATENÇÃO: as lâminas dão várias voltas (até 10 em cada sentido no teste
travel-limits). Confira que nada no caminho (cabos, suportes) impede a rotação.
"""

from __future__ import annotations

import _thread
import argparse
import json
import random
import statistics
import sys
import threading
import time
import traceback
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import serial

from arduino_elliptec import ArduinoElliptec
from waveplate import Settings, WavePlate, WavePlateError

# ── o que o firmware deveria ter (firmware/src/main.cpp) ──────────────────────
DEFAULT_PORT = "COM4"
DEFAULT_PLATES = ("X", "Y")
FULL_STEPS_PER_REV = 200     # PASSOS_POR_VOLTA_MOTOR
MICROSTEP = 8                # EIXOS_CFG: microstep
REDUCTION = 5.0              # EIXOS_CFG: reducao
HOME_DIRECTION = +1          # EIXOS_CFG: sentido_ref
SWITCH_ANGLE = 0.0           # ANGULO_FIM_CURSO

# ── tolerâncias ───────────────────────────────────────────────────────────────
PRINT_TOL = 0.006            # o firmware responde com 2 casas decimais
EDGE_TOL_MICROSTEPS = 2      # borda do fim de curso logo depois da referência
LOST_STEPS_TOL_MICROSTEPS = 3
TIMING_TOL = 0.15            # tempo de movimento: ±15 % do teórico...
TIMING_TOL_S = 0.05          # ...+ 50 ms
SLOWDOWN_TOL = 1.10          # juntas podem levar até 10 % a mais que a mais lenta sozinha

RESULTS_DIR = Path(__file__).resolve().parent.parent / "test_results"   # na raiz do projeto


# ══════════════════════════════════════════════════════════════════════════════
# Infraestrutura
# ══════════════════════════════════════════════════════════════════════════════
class SkipTest(Exception):
    """O teste não se aplica (ex.: precisa de duas lâminas)."""


STATUS_RANK = {"PASS": 0, "SKIP": 0, "WARN": 1, "FAIL": 2, "ERROR": 3}


class Result:
    """Coleta as verificações de um teste e imprime conforme acontecem."""

    def __init__(self, name: str):
        self.name = name
        self.status = "PASS"
        self.lines: List[str] = []
        self.metrics: Dict[str, object] = {}
        self.duration = 0.0
        self.headline = ""          # a mensagem mais grave, para o resumo

    def _log(self, mark: str, text: str):
        line = f"{mark:<6}{text}"
        print("    " + line, flush=True)
        self.lines.append(line)

    def _escalate(self, status: str, text: str):
        if STATUS_RANK[status] > STATUS_RANK[self.status]:
            self.status = status
            self.headline = text

    def check(self, ok, text: str, detail: str = "") -> bool:
        if detail and not ok:
            text = f"{text} [{detail}]"
        self._log("ok" if ok else "FALHA", text)
        if not ok:
            self._escalate("FAIL", text)
        return bool(ok)

    def warn(self, text: str):
        self._log("!", text)
        self._escalate("WARN", text)

    def info(self, text: str):
        self._log("·", text)

    def error(self, text: str):
        self._log("ERRO", text)
        self._escalate("ERROR", text)

    def metric(self, key: str, value):
        self.metrics[key] = value


@dataclass
class TestSpec:
    name: str
    group: str
    description: str
    func: Callable
    min_plates: int = 1
    slow: bool = False
    interactive: bool = False


TESTS: List[TestSpec] = []


def register(name: str, group: str, description: str, *, min_plates: int = 1,
             slow: bool = False, interactive: bool = False):
    def decorator(func):
        TESTS.append(TestSpec(name, group, description, func, min_plates, slow, interactive))
        return func
    return decorator


# ── utilidades ────────────────────────────────────────────────────────────────
def parse_plate(spec: str, default_port: str) -> Tuple[str, str]:
    """"X" → (porta padrão, "X");  "COM5:Y" → ("COM5", "Y")."""
    if ":" in spec:
        port, axis = spec.rsplit(":", 1)
        return port, axis.upper()
    return default_port, spec.upper()


def snap(angle: float, spd: float) -> float:
    """Ângulo arredondado ao micropasso, como o firmware faz (lround)."""
    return round(angle * spd) / spd


def raw(plate: WavePlate, command: str, reply: str = "OK", timeout: float = 1.0) -> str:
    """Manda um comando do protocolo direto, sem as checagens da biblioteca."""
    return plate._command(command, reply, timeout)


def expect_error(func: Callable, exc_type=Exception) -> str:
    """"" se func() levantou exc_type; senão, o que aconteceu."""
    try:
        func()
    except exc_type:
        return ""
    except Exception as e:  # noqa: BLE001 — queremos relatar qualquer outra coisa
        return f"{type(e).__name__}: {e}"
    return "aceitou (deveria recusar)"


def timed(func: Callable) -> Tuple[float, object]:
    t0 = time.perf_counter()
    value = func()
    return time.perf_counter() - t0, value


def move_time(s: Settings, distance: float) -> float:
    """
    Tempo teórico do perfil do firmware: arranca em min_speed, acelera até
    max_speed, desacelera até min_speed (trapézio, ou triângulo se for curto).
    """
    d = abs(distance)
    if d == 0:
        return 0.0
    vmin, vmax, a = s.min_speed, s.max_speed, s.acceleration
    ramp = (vmax ** 2 - vmin ** 2) / (2 * a)
    if d >= 2 * ramp:
        return 2 * (vmax - vmin) / a + (d - 2 * ramp) / vmax
    peak = (vmin ** 2 + a * d) ** 0.5
    return 2 * (peak - vmin) / a


def stop_quietly(plates: Sequence[WavePlate]):
    for p in plates:
        try:
            if p.is_moving():
                p.stop()
        except Exception:  # noqa: BLE001
            pass


def run_together(plates: Sequence[WavePlate], start: Callable[[WavePlate], object]) -> List[float]:
    """Chama start(p) (movimento sem bloquear) em todas e espera todas pararem."""
    started: List[WavePlate] = []
    try:
        for p in plates:
            start(p)
            started.append(p)
        return [p.wait_move() for p in started]
    except BaseException:
        stop_quietly(started)
        raise


def move_all(plates: Sequence[WavePlate], targets) -> List[float]:
    """Move várias lâminas ao mesmo tempo (um alvo para todas, ou uma lista)."""
    if not isinstance(targets, (list, tuple)):
        targets = [targets] * len(plates)
    by_plate = {id(p): t for p, t in zip(plates, targets)}
    return run_together(plates, lambda p: p.move_to(by_plate[id(p)], wait=False))


def command_overhead(p: WavePlate) -> float:
    """Tempo de um G sem movimento (ida e volta da serial). Chamar já parado no alvo."""
    here = p.get_position()
    samples = [timed(lambda: p.move_to(here))[0] for _ in range(5)]
    return statistics.median(samples)


def probe_edge(p: WavePlate, spd: float, expected: float, direction: int,
               window: float = 3.0, coarse: float = 0.25, backoff: float = 1.5) -> Optional[float]:
    """
    Ângulo comandado em que o fim de curso passa a ler acionado quando a lâmina
    chega perto de `expected` girando no sentido `direction` (+1/-1).

    Primeiro anda em passos de `coarse` graus; depois volta `backoff` graus
    (mais que a folga das engrenagens) e se aproxima de novo um micropasso por
    vez. Sempre chega pelo mesmo lado, então o resultado é repetível. Retorna
    None se não achar a borda em ±window.
    """
    d = direction
    start = expected - d * window
    p.move_to(start - d * backoff)
    p.move_to(start)
    if p.limit_switch_triggered():
        return None
    angle, end = start, expected + d * window
    found = None
    while d * (end - angle) > 1e-9:
        angle += d * coarse
        p.move_to(angle)
        if p.limit_switch_triggered():
            found = angle
            break
    if found is None:
        return None

    first = snap(found - d * (coarse + backoff), spd)
    p.move_to(first - d * backoff)
    p.move_to(first)
    if p.limit_switch_triggered():
        return None                      # não libera onde liberava: leitura instável
    k0 = round(first * spd)
    for i in range(1, int((2 * coarse + backoff) * spd) + 3):
        angle = (k0 + d * i) / spd
        p.move_to(angle)
        if p.limit_switch_triggered():
            return angle
    return None


def probe_release(p: WavePlate, spd: float, edge: float, direction: int,
                  depth: float = 1.0, span: float = 6.0) -> Optional[float]:
    """
    Entra `depth` graus no fim de curso (chegando no sentido `direction`) e
    volta um micropasso por vez até ele liberar. Retorna o ângulo de liberação.
    A distância até a borda é folga das engrenagens + histerese do sensor.
    """
    d = direction
    inside = snap(edge + d * depth, spd)
    p.move_to(edge - d * 2.0)
    p.move_to(inside)
    if not p.limit_switch_triggered():
        return None
    k0 = round(inside * spd)
    for i in range(1, int(span * spd) + 1):
        angle = (k0 - d * i) / spd
        p.move_to(angle)
        if not p.limit_switch_triggered():
            return angle
    return None


def scan_next_edge(p: WavePlate, start: float, direction: int, max_travel: float,
                   step: float = 2.0) -> Optional[float]:
    """Varre no sentido `direction` até o fim de curso liberar e acionar de novo."""
    d = direction
    p.move_to(start)
    angle = start
    was_on = p.limit_switch_triggered()
    while d * (angle - start) < max_travel:
        angle += d * step
        p.move_to(angle)
        now_on = p.limit_switch_triggered()
        if now_on and not was_on:
            return angle
        was_on = now_on
    return None


def find_edge_in_turn(p: WavePlate, spd: float, expected: float, direction: int,
                      window: float = 3.0, step: float = 2.0) -> Tuple[Optional[float], bool]:
    """
    Como probe_edge, mas se a borda não estiver a ±window de `expected`, varre
    uma volta inteira (centrada em `expected`) antes de desistir e refina onde
    achou. Retorna (ângulo da borda ou None, se precisou da volta inteira).
    """
    edge = probe_edge(p, spd, expected, direction, window)
    if edge is not None:
        return edge, False
    d = direction
    coarse = scan_next_edge(p, expected - d * 180.0, d, max_travel=360.0 + step, step=step)
    if coarse is None:
        return None, True
    fine = probe_edge(p, spd, coarse, d, window=step + 1.0)
    return (coarse if fine is None else fine), True


def reference_edges(b: "Bench", names: Optional[Sequence[str]] = None) -> Dict[str, Optional[float]]:
    """
    Borda do fim de curso de cada lâmina, para comparar depois. Se ela não
    estiver perto do zero (um teste anterior perdeu passos), refaz a referência.
    """
    out: Dict[str, Optional[float]] = {}
    for name in names or list(b.plates):
        e = b.edge(name)
        if e is None:
            print(f"    (borda de {name} longe do zero: refazendo a referência)", flush=True)
            b.plates[name].home()
            e = b.edge(name)
        out[name] = e
    return out


def ask(question: str) -> str:
    try:
        return input(f"    ? {question} ").strip().lower()
    except EOFError:
        raise SkipTest("sem entrada do teclado") from None


def is_yes(answer: str) -> bool:
    return answer in ("s", "sim", "y", "yes")


def fmt_states(states: Dict[str, bool]) -> str:
    return ", ".join(f"{n}={'1' if on else '0'}" for n, on in states.items())


# ── bancada ───────────────────────────────────────────────────────────────────
class Bench:
    """As lâminas abertas e o que já se sabe delas (referência, configuração)."""

    def __init__(self, args, seed: int):
        self.args = args
        self.rng = random.Random(seed)
        self.home_dir = args.home_direction
        self.plates: Dict[str, WavePlate] = {}
        self.defaults: Dict[str, Settings] = {}
        self.homed: set = set()
        self.connect_time = 0.0
        self.home_times: Dict[str, float] = {}

    def open(self):
        specs = [parse_plate(s, self.args.port) for s in self.args.plates]
        same_port = len({port for port, _ in specs}) == 1
        t0 = time.perf_counter()
        try:
            for port, axis in specs:
                name = axis if same_port else f"{port}:{axis}"
                if name in self.plates:
                    raise ValueError(f"lâmina repetida: {name}")
                plate = WavePlate(port, axis, disable_when_idle=False,
                                  move_timeout=self.args.move_timeout)
                self.plates[name] = plate
                self.defaults[name] = plate.get_settings()
        except BaseException:
            self.close()
            raise
        self.connect_time = time.perf_counter() - t0

    def items(self) -> List[Tuple[str, WavePlate]]:
        return list(self.plates.items())

    def all_plates(self) -> List[WavePlate]:
        return list(self.plates.values())

    def spd(self, name: str) -> float:
        return self.defaults[name].steps_per_degree

    def micro(self, name: str) -> float:
        return 1.0 / self.spd(name)

    def tol(self, name: str) -> float:
        """Meio micropasso + arredondamento da resposta."""
        return 0.5 * self.micro(name) + PRINT_TOL

    def ensure_homed(self, names: Optional[Sequence[str]] = None):
        todo = [n for n in (names or list(self.plates)) if n not in self.homed]
        if not todo:
            return
        print(f"    (referenciando {', '.join(todo)}...)", flush=True)
        run_together([self.plates[n] for n in todo], lambda p: p.home(wait=False))
        self.homed.update(todo)

    def all_to(self, angle: float = 0.0) -> List[float]:
        return move_all(self.all_plates(), angle)

    def edge(self, name: str, expected: float = SWITCH_ANGLE, window: float = 3.0) -> Optional[float]:
        return probe_edge(self.plates[name], self.spd(name), expected, self.home_dir, window)

    def stop_all(self):
        stop_quietly(self.all_plates())

    def restore_all(self):
        for p in self.all_plates():
            try:
                p.restore_defaults()
            except Exception:  # noqa: BLE001
                pass

    def close(self):
        for p in self.all_plates():
            try:
                p.close()
            except Exception:  # noqa: BLE001
                pass


# ══════════════════════════════════════════════════════════════════════════════
# Básico — sem movimento grande
# ══════════════════════════════════════════════════════════════════════════════
@register("connect", "básico", "abre a porta, lê os eixos do firmware e recusa eixo inexistente")
def test_connect(b: Bench, r: Result):
    r.check(True, f"{len(b.plates)} lâmina(s) aberta(s) em {b.connect_time:.1f} s "
                  f"(abrir a porta reinicia o Arduino)")
    r.metric("connect_s", round(b.connect_time, 3))
    ports: Dict[str, WavePlate] = {}
    for name, p in b.items():
        ports.setdefault(p.port, p)
    for port, p in ports.items():
        axes = p._serial.axes
        r.info(f"{port}: firmware com os eixos {' '.join(axes)}")
        missing = next((c for c in "WVUCBXYZA" if c not in axes), None)
        if missing:
            problem = expect_error(lambda: WavePlate(port, missing), ValueError)
            r.check(not problem, f"{port}: WavePlate(..., {missing!r}) recusa eixo que não existe", problem)


@register("settings", "básico", "passos/grau batem com microstep × redução; velocidades e limites coerentes")
def test_settings(b: Bench, r: Result):
    expected = FULL_STEPS_PER_REV * b.args.microstep * b.args.reduction / 360.0
    seen = {}
    for name, p in b.items():
        s = p.get_settings()
        r.info(f"{name}: {s}")
        r.check(abs(s.steps_per_degree - expected) < 1e-3,
                f"{name}: {s.steps_per_degree:.4f} passos/° (esperado {expected:.4f} = "
                f"{FULL_STEPS_PER_REV}×{b.args.microstep:g}×{b.args.reduction:g}/360)")
        r.check(0 < s.min_speed <= s.max_speed <= s.speed_limit + 1e-6,
                f"{name}: 0 < mínima ({s.min_speed:.1f}) ≤ máxima ({s.max_speed:.1f}) "
                f"≤ limite ({s.speed_limit:.1f} °/s)")
        r.check(s.acceleration > 0, f"{name}: aceleração {s.acceleration:.1f} °/s²")
        r.check(s.min_angle < SWITCH_ANGLE < s.max_angle,
                f"{name}: limites {s.min_angle:g}° a {s.max_angle:g}° contêm o zero")
        r.check(s.disable_delay_ms == 0, f"{name}: driver sempre habilitado durante os testes")
        seen[name] = (s.steps_per_degree, s.min_speed, s.max_speed, s.acceleration)
        r.metric(f"{name}.settings", asdict(s))
    if len(set(seen.values())) > 1:
        r.warn("as lâminas têm configurações diferentes entre si: " +
               "; ".join(f"{n}: {v}" for n, v in seen.items()))


@register("errors", "básico", "biblioteca e firmware recusam valores inválidos sem mudar nada")
def test_errors(b: Bench, r: Result):
    for name, p in b.items():
        s = p.get_settings()
        before = p.get_position()
        spd = s.steps_per_degree

        lib_cases = [
            (f"move_to({s.max_angle + 1:g})", lambda: p.move_to(s.max_angle + 1)),
            (f"move_to({s.min_angle - 1:g})", lambda: p.move_to(s.min_angle - 1)),
            ("configure(min_speed=0)", lambda: p.configure(min_speed=0)),
            ("configure(min_speed > max_speed)", lambda: p.configure(min_speed=s.max_speed + 1)),
            ("configure(max_speed > limite)", lambda: p.configure(max_speed=s.speed_limit * 1.05)),
            ("configure(acceleration=0)", lambda: p.configure(acceleration=0)),
        ]
        bad = [f"{label}: {msg}" for label, func in lib_cases
               if (msg := expect_error(func, ValueError))]
        r.check(not bad, f"{name}: biblioteca recusa {len(lib_cases)} valores inválidos "
                         f"com ValueError", "; ".join(bad))

        fw_cases = [
            "G abc", "G 90x", "G", f"G {s.max_angle + 1:.2f}", f"G {s.min_angle - 1:.2f}",
            "M 0", f"M {s.max_speed * spd * 1.5:.2f}",                    # M > V
            f"V {s.speed_limit * spd + 1:.2f}", f"V {s.min_speed * spd * 0.5:.2f}",  # V < M
            "A 0", "A -10", "T -1", "T 700000", "Q", "",
        ]
        bad = [f"'{cmd}': {msg}" for cmd in fw_cases
               if (msg := expect_error(lambda: raw(p, cmd), WavePlateError))]
        r.check(not bad, f"{name}: firmware responde ERR a {len(fw_cases)} comandos inválidos",
                "; ".join(bad))

        after = p.get_settings()
        r.check(after == s, f"{name}: configuração não mudou", f"antes {s}, depois {after}")
        if after != s:
            p.restore_defaults()
        pos = p.get_position()
        r.check(abs(pos - before) <= PRINT_TOL, f"{name}: posição não mudou ({pos:.2f}°)")


@register("serial", "básico", "minúsculas, CR+LF, linha longa, eixo desconhecido e rajada de comandos")
def test_serial(b: Bench, r: Result):
    for name, p in b.items():
        shared, ax = p._serial, p.axis
        unknown = next(c for c in "WVUCBQ" if c not in shared.axes)

        def write_raw(data: bytes):
            with shared._lock:
                shared.ser.write(data)

        before = p.get_position()
        shared.clear(ax)
        write_raw(f"{ax.lower()} p\n".encode())
        line = shared.read_line(ax, 1.0)
        r.check(line is not None and line.startswith("POS"),
                f"{name}: aceita minúsculas ('{ax.lower()} p')", repr(line))

        shared.clear(ax)
        write_raw(f"  {ax}   P\r\n".encode())
        line = shared.read_line(ax, 1.0)
        r.check(line is not None and line.startswith("POS"),
                f"{name}: aceita espaços extras e CR+LF", repr(line))

        write_raw(f"{ax} G {'1' * 40}\n".encode())
        write_raw(f"{unknown} P\n".encode())
        time.sleep(0.05)
        pos = p.get_position()
        r.check(abs(pos - before) <= PRINT_TOL,
                f"{name}: linha longa demais e eixo desconhecido ('{unknown} P') são "
                f"descartados e a serial continua respondendo")

        n = 12
        shared.clear(ax)
        write_raw(f"{ax} P\n".encode() * n)
        got = 0
        while got < n:
            line = shared.read_line(ax, 0.5)
            if line is None:
                break
            got += line.startswith("POS")
        if got == n:
            r.check(True, f"{name}: {n} comandos seguidos sem esperar resposta: {got}/{n} respostas")
        else:
            r.warn(f"{name}: rajada de {n} comandos: só {got}/{n} respostas (buffer de 64 bytes "
                   f"do Arduino). A biblioteca espera cada resposta, então não afeta o uso normal")


@register("switch-read", "básico", "leitura do fim de curso parada é estável (30 leituras)")
def test_switch_read(b: Bench, r: Result):
    for name, p in b.items():
        on = sum(p.limit_switch_triggered() for _ in range(30))
        if on in (0, 30):
            r.check(True, f"{name}: fim de curso {'ACIONADO' if on else 'livre'} em 30/30 leituras")
        else:
            r.warn(f"{name}: leitura instável ({on}/30 acionado): ruído, mau contato ou "
                   f"lâmina parada bem na borda do sensor")


@register("latency", "básico", "tempo de resposta da serial (100 consultas de posição)")
def test_latency(b: Bench, r: Result):
    for name, p in b.items():
        times = sorted(timed(p.get_position)[0] for _ in range(100))
        mean, p95, worst = statistics.mean(times), times[94], times[-1]
        r.metric(f"{name}.mean_ms", round(mean * 1000, 2))
        r.metric(f"{name}.max_ms", round(worst * 1000, 2))
        text = (f"{name}: P → POS em {mean * 1000:.1f} ms em média "
                f"(95 %: {p95 * 1000:.1f} ms, pior: {worst * 1000:.1f} ms)")
        if worst > 0.1:
            r.warn(text + " — algum atraso acima de 100 ms")
        else:
            r.check(True, text)


@register("identify", "básico", "você confirma qual montagem gira e o sentido do ângulo positivo",
          interactive=True)
def test_identify(b: Bench, r: Result):
    for name, p in b.items():
        ask(f"Vou girar a lâmina {name} +90° e voltar. Olhe as montagens e tecle Enter.")
        start = p.get_position()
        p.move_to(start + 90)
        r.check(is_yes(ask(f"Girou só a lâmina {name}? [s/n]")),
                f"{name}: girou a montagem certa (e só ela)")
        answer = ask("Olhando a lâmina de frente (feixe vindo na sua direção), girou no "
                     "sentido anti-horário? [s/n/Enter = não sei]")
        sense = "anti-horário" if is_yes(answer) else ("horário" if answer in ("n", "nao", "não") else "?")
        r.info(f"{name}: ângulo positivo = {sense} (visto contra o feixe)")
        r.metric(f"{name}.positive_sense", sense)
        p.move_to(start)


# ══════════════════════════════════════════════════════════════════════════════
# Uma lâmina por vez
# ══════════════════════════════════════════════════════════════════════════════
@register("home", "uma lâmina", "referência de cada lâmina, uma depois da outra; sensor acionado só no zero")
def test_home(b: Bench, r: Result):
    d = b.home_dir
    for name, p in b.items():
        tol = b.tol(name)
        dt, final = timed(p.home)
        b.homed.add(name)
        b.home_times[name] = dt
        r.metric(f"{name}.home_s", round(dt, 2))
        r.check(abs(final - SWITCH_ANGLE) <= tol, f"{name}: referência terminou em {final:.2f}° ({dt:.1f} s)")
        if p.limit_switch_triggered():
            r.check(True, f"{name}: parada no zero, o fim de curso lê acionado")
        else:
            r.warn(f"{name}: parada no zero, o fim de curso lê LIVRE (parou bem na borda?)")
        p.move_to(SWITCH_ANGLE - d * 10)
        r.check(not p.limit_switch_triggered(), f"{name}: 10° antes do zero o fim de curso libera")
        dt, final = timed(p.home)
        r.check(abs(final - SWITCH_ANGLE) <= tol, f"{name}: referência partindo de 10° antes do zero: {dt:.1f} s")


@register("edge", "uma lâmina", "borda do fim de curso cai no zero após cada referência (repetibilidade do zero)")
def test_edge(b: Bench, r: Result):
    reps = 3 if b.args.quick else 6
    b.ensure_homed()
    for name, p in b.items():
        spd, micro = b.spd(name), b.micro(name)
        edges = []
        for i in range(reps):
            if i > 0:
                if i % 2:
                    p.move_to(SWITCH_ANGLE)      # parte EM CIMA do sensor (fase "sair do sensor")
                else:
                    p.move_to(round(b.rng.uniform(-300, 300), 2))
                p.home()
            e = b.edge(name)
            if e is None:
                r.check(False, f"{name}: borda do fim de curso não encontrada perto de 0°")
                break
            edges.append(e)
        if not edges:
            continue
        worst = max(abs(e - SWITCH_ANGLE) for e in edges)
        spread = max(edges) - min(edges)
        r.info(f"{name}: borda após {len(edges)} referências: " +
               ", ".join(f"{e:+.3f}" for e in edges) + "°")
        r.metric(f"{name}.edges", edges)
        limit = EDGE_TOL_MICROSTEPS * micro + PRINT_TOL
        text = (f"{name}: borda a até {worst / micro:.1f} micropasso(s) do zero, "
                f"espalhamento {spread:.3f}° ({spread / micro:.1f} µp)")
        if worst <= limit:
            r.check(True, text)
        elif worst <= 2 * limit:
            r.warn(text)
        else:
            r.check(False, text)

        release = probe_release(p, spd, edges[-1], b.home_dir)
        if release is None:
            r.warn(f"{name}: não consegui medir onde o fim de curso libera voltando")
        else:
            gap = abs(edges[-1] - release)
            r.metric(f"{name}.backlash_plus_switch_hysteresis", round(gap, 4))
            r.info(f"{name}: voltando, libera em {release:+.3f}° → folga das engrenagens + "
                   f"histerese do sensor ≈ {gap:.3f}° (limite superior da folga)")
        p.move_to(SWITCH_ANGLE)


@register("revolution", "uma lâmina", "uma volta comandada = uma volta da lâmina (escala de passos/grau)")
def test_revolution(b: Bench, r: Result):
    turns = [1, -1] if b.args.quick else [1, -1, 3, -3]
    b.ensure_homed()
    for name, p in b.items():
        micro = b.micro(name)
        e0 = reference_edges(b, [name])[name]
        if e0 is None:
            r.check(False, f"{name}: borda do zero não encontrada")
            continue
        errors = []
        for k in turns:
            expected = SWITCH_ANGLE + 360.0 * k
            e = b.edge(name, expected, window=4.0)
            if e is None:
                found = scan_next_edge(p, e0, b.home_dir, max_travel=3 * 360.0)
                hint = ""
                if found is not None:
                    per_turn = abs(found - e0)
                    ratio = per_turn / 360.0
                    simple = any(abs(ratio - x) < 0.03 for x in (0.25, 0.5, 2.0, 4.0, 8.0))
                    cause = ("razão simples: confira microstep (jumpers MS1-3 embaixo do driver) e a "
                             "redução em EIXOS_CFG" if simple else
                             "não é razão simples: a lâmina perde passos (teste com max_speed menor) "
                             "ou a redução em EIXOS_CFG está errada")
                    hint = (f"; a próxima borda está em ≈ {found:.0f}° comandados = {ratio:.2f} × 360° "
                            f"→ {cause}")
                r.check(False, f"{name}: borda não está perto de {expected:+.0f}°{hint}")
                break
            per_turn = (e - e0) / k
            err = e - e0 - 360.0 * k
            errors.append((k, err))
            r.check(abs(err) <= LOST_STEPS_TOL_MICROSTEPS * micro + PRINT_TOL,
                    f"{name}: {k:+d} volta(s): borda em {e:+.3f}° → {per_turn:.3f}°/volta "
                    f"(erro {err:+.3f}° = {err / micro:+.1f} µp)")
        r.metric(f"{name}.turn_errors", errors)
        if len(errors) >= 3 and all(abs(e) > 2 * micro for _, e in errors):
            k, err = max(errors, key=lambda t: abs(t[0]))
            per_turn = 360.0 + err / k
            r.info(f"{name}: se o erro cresce com o número de voltas, a redução real é "
                   f"≈ {b.args.reduction * per_turn / 360.0:.4f}")
        p.move_to(SWITCH_ANGLE)


@register("positioning", "uma lâmina", "alvos absolutos arredondados ao micropasso; move_by não acumula erro")
def test_positioning(b: Bench, r: Result):
    b.ensure_homed()
    targets = [10, 45, 90.5, -30, 0.02, 0.03, -0.045, 123.456, 359.99, -179.3, 0]
    for name, p in b.items():
        spd, tol = b.spd(name), b.tol(name)
        bad = []
        for t in targets:
            expected = snap(t, spd)
            ok_angle = p.move_to(t, wait=False)
            final = p.wait_move()
            pos = p.get_position()
            if max(abs(ok_angle - expected), abs(final - expected), abs(pos - expected)) > PRINT_TOL:
                bad.append(f"{t}: OK {ok_angle} DONE {final} POS {pos} (esperado {expected:.3f})")
        r.check(not bad, f"{name}: {len(targets)} alvos: OK, DONE e POS = alvo arredondado ao "
                         f"micropasso ({1 / spd:.4f}°)", "; ".join(bad))

        p.move_to(0)
        for _ in range(50):
            p.move_by(0.1)
        pos = p.get_position()
        r.check(abs(pos - 5.0) <= tol,
                f"{name}: 50 × move_by(0.1) → {pos:.3f}° (esperado 5.000°; cada passo de 0.1° "
                f"seria arredondado a {snap(0.1, spd):.3f}° se acumulasse)")
        p.move_by(-5.0)
        pos = p.get_position()
        r.check(abs(pos) <= tol, f"{name}: move_by(-5) volta a {pos:.3f}°")


@register("timing", "uma lâmina", "tempo dos movimentos bate com o perfil de velocidade (teórico)")
def test_timing(b: Bench, r: Result):
    distances = [1, 10, 90, 360] if b.args.quick else [1, 10, 90, 360, 1080]
    b.ensure_homed()
    for name, p in b.items():
        s = p.get_settings()
        p.move_to(0)
        overhead = command_overhead(p)
        rows, bad = [], []
        for dist in distances:
            for target in (float(dist), 0.0):
                dt, _ = timed(lambda: p.move_to(target))
                motion = dt - overhead
                theory = move_time(s, dist)
                label = f"{dist:g}°{'+' if target else '-'} {motion * 1000:.0f}/{theory * 1000:.0f}"
                rows.append(label)
                if abs(motion - theory) > TIMING_TOL * theory + TIMING_TOL_S:
                    bad.append(label)
        r.info(f"{name}: medido/teórico em ms (descontados {overhead * 1000:.0f} ms de serial): "
               + ", ".join(rows))
        if bad:
            r.warn(f"{name}: fora de ±{TIMING_TOL:.0%}: {', '.join(bad)}")
        else:
            r.check(True, f"{name}: todos dentro de ±{TIMING_TOL:.0%} + {TIMING_TOL_S * 1000:.0f} ms")


@register("nonblocking", "uma lâmina", "move_to(wait=False) + is_moving() + posição durante o movimento")
def test_nonblocking(b: Bench, r: Result):
    b.ensure_homed()
    for name, p in b.items():
        tol = b.tol(name)
        p.move_to(0)
        t_plain, _ = timed(lambda: p.move_to(360))
        p.move_to(0)

        t0 = time.perf_counter()
        ret = p.move_to(360, wait=False)
        r.check(abs(ret - 360) <= PRINT_TOL, f"{name}: move_to(wait=False) devolve o alvo na hora ({ret:.2f}°)")
        positions = []
        while p.is_moving() and time.perf_counter() - t0 < 30:
            positions.append(p.get_position())
        t_poll = time.perf_counter() - t0
        final = p.wait_move()
        r.check(abs(final - 360) <= tol, f"{name}: wait_move() depois do fim devolve {final:.2f}°")
        increasing = all(y >= x - 1e-9 for x, y in zip(positions, positions[1:]))
        r.check(len(positions) >= 3 and increasing,
                f"{name}: {len(positions)} leituras de posição durante o movimento, sempre crescentes")
        r.check(not p.is_moving(), f"{name}: is_moving() é False no fim")
        ratio = t_poll / t_plain
        r.metric(f"{name}.poll_slowdown", round(ratio, 3))
        text = (f"{name}: consultando a posição sem parar, o movimento levou {t_poll:.2f} s "
                f"(sem consultar: {t_plain:.2f} s)")
        if ratio > 1.05 + 0.03 / t_plain:
            r.warn(text + " — as consultas atrasam os passos")
        else:
            r.check(True, text)
        p.move_to(0)
        r.check(abs(p.wait_move()) <= tol, f"{name}: wait_move() parado devolve a posição atual")


@register("busy", "uma lâmina", "em movimento: recusa G/Z/M/R/D e configure, aceita P/F/I/H/T")
def test_busy(b: Bench, r: Result):
    b.ensure_homed()
    for name, p in b.items():
        tol = b.tol(name)
        p.move_to(0)
        p.move_to(720, wait=False)
        try:
            lib_cases = [("move_to", lambda: p.move_to(10)), ("move_by", lambda: p.move_by(5)),
                         ("home", p.home), ("set_zero", p.set_zero),
                         ("configure", lambda: p.configure(max_speed=10)), ("disable", p.disable)]
            bad = [f"{label}: {msg}" for label, func in lib_cases
                   if (msg := expect_error(func, WavePlateError))]
            r.check(not bad, f"{name}: biblioteca recusa {', '.join(l for l, _ in lib_cases)} em movimento",
                    "; ".join(bad))

            fw_cases = [("G 10", "OK"), ("Z", "POS"), ("M 100", "OK"), ("V 1000", "OK"),
                        ("A 1000", "OK"), ("R", "OK"), ("D", "OK")]
            bad = [f"'{cmd}': {msg}" for cmd, reply in fw_cases
                   if (msg := expect_error(lambda: raw(p, cmd, reply), WavePlateError))]
            r.check(not bad, f"{name}: firmware responde 'ERR em movimento' a "
                             f"{', '.join(c for c, _ in fw_cases)}", "; ".join(bad))

            allowed = [("get_position (P)", p.get_position),
                       ("limit_switch_triggered (F)", p.limit_switch_triggered),
                       ("get_settings (I)", p.get_settings),
                       ("enable (H)", p.enable),
                       ("disable_when_idle(False) (T)", lambda: p.disable_when_idle(False))]
            bad = []
            for label, func in allowed:
                try:
                    func()
                except Exception as e:  # noqa: BLE001
                    bad.append(f"{label}: {type(e).__name__}: {e}")
            r.check(not bad, f"{name}: aceita P, F, I, H e T em movimento", "; ".join(bad))
            r.check(p.is_moving(), f"{name}: continua andando depois de tudo isso")
        finally:
            final = p.wait_move()
        r.check(abs(final - 720) <= tol, f"{name}: chegou no alvo original ({final:.2f}°)")
        p.move_to(0)


@register("stop", "uma lâmina", "stop() no meio do caminho freia com a aceleração configurada")
def test_stop(b: Bench, r: Result):
    b.ensure_homed()
    for name, p in b.items():
        tol = b.tol(name)
        s = p.get_settings()
        p.move_to(0)
        t0 = time.perf_counter()
        p.move_to(720, wait=False)
        time.sleep(0.6)
        at_stop = p.get_position()
        elapsed = time.perf_counter() - t0
        final = p.stop()
        speed = min(s.max_speed, s.min_speed + s.acceleration * elapsed)
        brake = (speed ** 2 - s.min_speed ** 2) / (2 * s.acceleration)
        travelled = final - at_stop
        r.check(0 < final < 720, f"{name}: parou antes do alvo, em {final:.2f}°")
        time.sleep(0.3)
        still = p.get_position()
        r.check(abs(still - final) <= tol and not p.is_moving(),
                f"{name}: ficou parada depois do DONE ({still:.2f}°)")
        text = (f"{name}: andou {travelled:.1f}° depois do S, a {speed:.0f} °/s "
                f"(frenagem teórica ≈ {brake:.1f}°)")
        margin = speed * 0.03 + 2.0          # ~30 ms de serial até o S chegar
        if 0.5 * brake - margin <= travelled <= 1.3 * brake + margin:
            r.check(True, text)
        else:
            r.warn(text + " — fora do esperado para desaceleração")
        r.metric(f"{name}.brake_deg", round(travelled, 2))

        pos = p.get_position()
        idle = p.stop()
        r.check(abs(idle - pos) <= tol, f"{name}: stop() parada devolve a posição sem mexer")
        r.check(abs(p.move_to(0)) <= tol, f"{name}: aceita movimento depois do stop()")


@register("home-cancel", "uma lâmina", "stop() durante a referência cancela a busca e a lâmina continua usável")
def test_home_cancel(b: Bench, r: Result):
    b.ensure_homed()
    for name, p in b.items():
        tol = b.tol(name)
        p.move_to(SWITCH_ANGLE - b.home_dir * 90)
        p.home(wait=False)
        time.sleep(0.5)
        final = p.stop()
        b.homed.discard(name)
        r.check(not p.is_moving(), f"{name}: stop() cancelou a referência (parou em {final:.2f}°)")
        time.sleep(0.3)
        r.check(abs(p.get_position() - final) <= tol, f"{name}: ficou parada")
        moved = p.move_by(5)
        r.check(abs(moved - (final + 5)) <= tol, f"{name}: move_by(5) depois do cancelamento → {moved:.2f}°")
    b.ensure_homed()


@register("ctrl-c", "uma lâmina", "Ctrl+C durante move_to(): para com desaceleração e a conexão segue usável")
def test_ctrl_c(b: Bench, r: Result):
    if threading.current_thread() is not threading.main_thread():
        raise SkipTest("só funciona na thread principal")
    b.ensure_homed()
    for name, p in b.items():
        tol = b.tol(name)
        p.move_to(0)
        timer = threading.Timer(0.6, _thread.interrupt_main)
        interrupted = False
        try:
            timer.start()
            p.move_to(720)
        except KeyboardInterrupt:
            interrupted = True
        finally:
            timer.cancel()
        r.check(interrupted, f"{name}: o KeyboardInterrupt chegou durante o move_to()")
        p1 = p.get_position()
        time.sleep(0.3)
        p2 = p.get_position()
        r.check(not p.is_moving() and abs(p1 - p2) <= tol and 0 < p1 < 720,
                f"{name}: parou em {p1:.2f}° e ficou parada")
        r.check(abs(p.move_to(0)) <= tol, f"{name}: aceita o próximo movimento")


@register("driver", "uma lâmina", "enable/disable e disable_when_idle; G religa o driver sozinho")
def test_driver(b: Bench, r: Result):
    b.ensure_homed()
    for name, p in b.items():
        tol = b.tol(name)
        try:
            p.disable_when_idle(True, 300)
            delay = p.get_settings().disable_delay_ms
            r.check(delay == 300, f"{name}: disable_when_idle(True, 300) → firmware desliga após {delay} ms")
            p.move_to(10)
            time.sleep(0.5)                  # o driver já desligou
            r.check(abs(p.move_to(0)) <= tol, f"{name}: anda de novo depois do desligamento automático")
        finally:
            p.disable_when_idle(False)
        r.check(p.get_settings().disable_delay_ms == 0, f"{name}: disable_when_idle(False) → sempre habilitado")
        p.disable()
        p.enable()
        p.disable()
        r.check(abs(p.move_to(5) - 5) <= tol, f"{name}: G depois de disable() religa o driver e anda")
        p.move_to(0)


@register("configure", "uma lâmina", "configure() muda o perfil de verdade; ordem V/M; restore_defaults()")
def test_configure(b: Bench, r: Result):
    b.ensure_homed()
    for name, p in b.items():
        d0 = b.defaults[name]
        try:
            p.restore_defaults()
            p.move_to(0)
            overhead = command_overhead(p)
            for label, kwargs in [("max_speed/2", dict(max_speed=d0.max_speed / 2)),
                                  ("acceleration/2", dict(acceleration=d0.acceleration / 2)),
                                  ("min_speed/2", dict(min_speed=d0.min_speed / 2))]:
                p.restore_defaults()
                s = p.configure(**kwargs)
                applied = all(abs(getattr(s, k) - v) <= 0.01 * v + 0.01 for k, v in kwargs.items())
                r.check(applied, f"{name}: configure({label}) aplicado", str(s))
                dt, _ = timed(lambda: p.move_to(360))
                motion, theory = dt - overhead, move_time(s, 360)
                text = (f"{name}: 360° com {label}: {motion:.2f} s (teórico {theory:.2f} s, "
                        f"padrão {move_time(d0, 360):.2f} s)")
                if abs(motion - theory) <= TIMING_TOL * theory + TIMING_TOL_S:
                    r.check(True, text)
                else:
                    r.warn(text)
                p.move_to(0)

            p.restore_defaults()
            if d0.speed_limit > d0.max_speed * 1.01:
                new_min = (d0.max_speed + d0.speed_limit) / 2
                s = p.configure(min_speed=new_min, max_speed=d0.speed_limit)
                r.check(abs(s.min_speed - new_min) <= 0.05 and abs(s.max_speed - d0.speed_limit) <= 0.05,
                        f"{name}: min_speed acima da máxima antiga (manda V antes de M)", str(s))
        finally:
            s = p.restore_defaults()
        same = all(abs(getattr(s, k) - getattr(d0, k)) <= 0.01
                   for k in ("min_speed", "max_speed", "acceleration"))
        r.check(same, f"{name}: restore_defaults() volta aos valores do firmware")


@register("calibrate", "uma lâmina", "set_zero() + calibrate(): refaz o home e volta ao último ângulo "
                                     "sem mudar o zero de software")
def test_calibrate(b: Bench, r: Result):
    b.ensure_homed()
    for name, p in b.items():
        spd, micro = b.spd(name), b.micro(name)
        edge_tol = EDGE_TOL_MICROSTEPS * micro + PRINT_TOL
        try:
            # (onde fica o zero de software, ângulo pedido depois)
            for offset, target in ((50.0, 20.0), (500.0, -35.5)):
                p.home()
                p.move_by(offset)
                p.set_zero()
                p.move_by(target)
                r.check(abs(p.zero_offset - snap(offset, spd)) <= PRINT_TOL,
                        f"{name}: set_zero() em {offset:g}° → zero_offset = {p.zero_offset:.3f}°")
                # simula passos perdidos: o firmware passa a contar errado ({target:g}° de erro)
                raw(p, "Z", "POS")
                final = p.calibrate()
                r.check(abs(final - snap(target, spd)) <= PRINT_TOL,
                        f"{name}: com o firmware contando {target:+g}° errado, calibrate() volta a "
                        f"{final:.3f}° (pedido {target:g}°)")
                wrapped = (offset + 180.0) % 360.0 - 180.0
                r.check(abs(p.zero_offset - snap(wrapped, spd)) <= PRINT_TOL,
                        f"{name}: zero_offset continua {p.zero_offset:.3f}° (= {offset:g}° módulo 360°)")
                # procura a borda numa volta inteira: se ela estiver longe, mostra onde está
                e, full_turn = find_edge_in_turn(p, spd, -wrapped, b.home_dir)
                if e is None:
                    where = "não aciona em nenhum ponto de uma volta inteira"
                else:
                    where = f"aciona em {e:.3f}° do referencial"
                    if abs(e + wrapped) > edge_tol:
                        where += f" ({e + wrapped:+.3f}° = {(e + wrapped) * spd:+.0f} µp do esperado)"
                r.check(e is not None and abs(e + wrapped) <= edge_tol,
                        f"{name}: o fim de curso {where} (esperado {-wrapped:.3f}°"
                        f"{', achado varrendo 1 volta' if full_turn else ''}): "
                        f"o zero de software ficou no mesmo lugar físico")

            p.home()
            saved = p._serial.zero_steps[p.axis]
            p._serial.zero_steps[p.axis] = None          # como logo depois de conectar
            problem = expect_error(p.calibrate, WavePlateError)
            p._serial.zero_steps[p.axis] = saved
            r.check(not problem, f"{name}: sem home() desde a conexão, calibrate() recusa com WavePlateError",
                    problem)
        finally:
            p.home()                                     # deixa o 0 do firmware no fim de curso
        p.move_to(SWITCH_ANGLE)


def triggered_arcs(samples: List[Tuple[float, bool]]) -> List[Tuple[float, float]]:
    """Trechos (início, fim) em que o fim de curso leu acionado, na ordem da varredura."""
    arcs, start, last = [], None, None
    for angle, on in samples:
        if on and start is None:
            start = angle
        elif not on and start is not None:
            arcs.append((start, last))
            start = None
        last = angle
    if start is not None:
        arcs.append((start, last))
    return arcs


@register("switch-turn", "uma lâmina", "dá uma volta completa em passos de 1° e verifica se o fim de "
                                       "curso libera e volta a acionar")
def test_switch_turn(b: Bench, r: Result):
    b.ensure_homed()
    step = 1.0
    for name, p in b.items():
        d = -b.home_dir                          # sai do fim de curso e dá a volta até ele de novo
        try:
            p.home()
            samples = []
            for k in range(int(round(360.0 / step)) + 1):
                angle = SWITCH_ANGLE + d * k * step
                p.move_to(angle)
                samples.append((angle, p.limit_switch_triggered()))
        finally:
            p.home()
        on = [a for a, s in samples if s]
        off = [a for a, s in samples if not s]
        arcs = triggered_arcs(samples)
        arc_text = ", ".join(f"{a:+.0f}°…{e:+.0f}°" for a, e in arcs) or "nenhum"
        r.info(f"{name}: {len(samples)} leituras em 360°: acionado em {len(on)}, liberado em {len(off)}; "
               f"trechos acionados: {arc_text}")
        r.check(bool(off), f"{name}: o fim de curso libera em algum ponto da volta",
                "acionado a volta inteira: sensor desconectado (NF lê acionado sem fio), "
                "fio rompido ou bandeira não sai do sensor")
        r.check(bool(on), f"{name}: o fim de curso aciona em algum ponto da volta",
                "nunca aciona: bandeira não passa pelo sensor ou sensor ligado em outro pino")
        if on and off:
            # o primeiro e o último trecho são a mesma bandeira (início e fim da volta)
            flags = len(arcs) - (1 if len(arcs) > 1 and samples[0][1] and samples[-1][1] else 0)
            width = len(on) * step - (step if samples[0][1] and samples[-1][1] else 0)
            r.check(flags == 1, f"{name}: uma única bandeira por volta, com ~{width:.0f}° de largura",
                    f"{flags} trechos acionados separados: leitura instável ou mais de uma bandeira")
# ══════════════════════════════════════════════════════════════════════════════
@register("routing", "duas ou mais", "cada resposta volta para o eixo certo, inclusive com outra andando",
          min_plates=2)
def test_routing(b: Bench, r: Result):
    b.ensure_homed()
    items = b.items()
    targets = {name: snap((i + 1) * 11.25 * (-1) ** i, b.spd(name)) for i, (name, _) in enumerate(items)}
    move_all([p for _, p in items], [targets[n] for n, _ in items])
    wrong = []
    for _ in range(40):
        for name, p in items:
            pos = p.get_position()
            if abs(pos - targets[name]) > PRINT_TOL:
                wrong.append(f"{name} leu {pos}")
    r.check(not wrong, f"{len(items)} lâminas × 40 consultas intercaladas: cada uma lê a própria "
                       f"posição ({', '.join(f'{n}={t:.2f}°' for n, t in targets.items())})",
            "; ".join(wrong[:5]))

    (a, pa), rest = items[0], items[1:]
    pa.move_to(targets[a] + 360, wait=False)
    wrong, seen = [], []
    while pa.is_moving():
        seen.append(pa.get_position())
        for name, p in rest:
            pos = p.get_position()
            if abs(pos - targets[name]) > PRINT_TOL:
                wrong.append(f"{name} leu {pos}")
    pa.wait_move()
    r.check(not wrong and len(seen) >= 3,
            f"enquanto {a} anda, as outras respondem a própria posição ({len(seen)} rodadas)",
            "; ".join(wrong[:5]))
    b.all_to(0)


@register("independence", "duas ou mais", "parar uma não para as outras; referenciar uma enquanto as outras andam",
          min_plates=2)
def test_independence(b: Bench, r: Result):
    b.ensure_homed()
    items = b.items()
    (a, pa), rest = items[0], items[1:]
    tol = max(b.tol(n) for n, _ in items)
    b.all_to(0)

    pa.move_to(720, wait=False)
    for _, p in rest:
        p.move_to(-720, wait=False)
    time.sleep(0.6)
    stopped = pa.stop()
    finals = [p.wait_move() for _, p in rest]
    r.check(stopped < 700, f"{a} parou com stop() em {stopped:.1f}°")
    r.check(all(abs(f + 720) <= tol for f in finals),
            f"as outras continuaram até -720° ({', '.join(f'{f:.2f}' for f in finals)})")
    b.all_to(0)

    pa.move_to(SWITCH_ANGLE - b.home_dir * 60)
    for _, p in rest:
        p.move_to(360, wait=False)
    pa.home(wait=False)
    home_final = pa.wait_move()
    finals = [p.wait_move() for _, p in rest]
    r.check(abs(home_final - SWITCH_ANGLE) <= tol, f"{a} fez a referência enquanto as outras andavam")
    r.check(all(abs(f - 360) <= tol for f in finals),
            f"e as outras chegaram em 360° ({', '.join(f'{f:.2f}' for f in finals)})")

    before = {n: p.get_position() for n, p in rest}
    pa.move_to(180)
    after = {n: p.get_position() for n, p in rest}
    r.check(before == after, f"mover {a} não muda a posição lida das outras")
    b.all_to(0)


@register("switch-map", "duas ou mais", "cada fim de curso está ligado no eixo certo (nenhum trocado ou em curto)",
          min_plates=2)
def test_switch_map(b: Bench, r: Result):
    b.ensure_homed()
    items = b.items()
    d = b.home_dir
    park = SWITCH_ANGLE - d * 30
    move_all([p for _, p in items], park)
    states = {n: p.limit_switch_triggered() for n, p in items}
    r.check(not any(states.values()), f"todas a 30° do zero: nenhum fim de curso acionado",
            fmt_states(states))
    for name, p in items:
        p.move_to(SWITCH_ANGLE - d * 2)
        p.move_to(SWITCH_ANGLE + d * 0.3)       # chega como na referência e entra no sensor
        states = {n: q.limit_switch_triggered() for n, q in items}
        expected = {n: n == name for n in states}
        r.check(states == expected, f"{name} sobre o próprio fim de curso: só {name} lê acionado",
                fmt_states(states))
        p.move_to(park)
        states = {n: q.limit_switch_triggered() for n, q in items}
        r.check(not any(states.values()), f"{name} fora dele de novo: nenhum acionado", fmt_states(states))
    b.all_to(0)


@register("switch-press", "duas ou mais", "você aperta cada fim de curso com a mão; só o eixo dele acusa",
          interactive=True)
def test_switch_press(b: Bench, r: Result):
    b.ensure_homed()
    items = b.items()
    move_all([p for _, p in items], SWITCH_ANGLE - b.home_dir * 30)
    for name, p in items:
        if ask(f"Aperte e SEGURE o fim de curso da lâmina {name} e tecle Enter ('p' pula):") == "p":
            r.info(f"{name}: pulado")
            continue
        states = {n: q.limit_switch_triggered() for n, q in items}
        r.check(states == {n: n == name for n in states},
                f"apertando o de {name}: só {name} acionado", fmt_states(states))
        ask("Solte e tecle Enter.")
        states = {n: q.limit_switch_triggered() for n, q in items}
        r.check(not any(states.values()), "solto: nenhum acionado", fmt_states(states))
    b.all_to(0)


@register("home-both", "duas ou mais", "referência de todas ao mesmo tempo (como stage.home(addr=[0, 2]))",
          min_plates=2)
def test_home_both(b: Bench, r: Result):
    items = b.items()
    plates = [p for _, p in items]
    move_all(plates, [round(b.rng.uniform(-300, -20), 2) for _ in plates])
    dt, finals = timed(lambda: run_together(plates, lambda p: p.home(wait=False)))
    b.homed.update(n for n, _ in items)
    r.check(all(abs(f - SWITCH_ANGLE) <= b.tol(n) for (n, _), f in zip(items, finals)),
            f"todas referenciadas juntas em {dt:.1f} s ({', '.join(f'{f:.2f}°' for f in finals)})")
    if len(b.home_times) == len(items):
        r.info(f"uma depois da outra (teste home): {sum(b.home_times.values()):.1f} s no total")
    for name, _ in items:
        e = b.edge(name)
        micro = b.micro(name)
        r.check(e is not None and abs(e - SWITCH_ANGLE) <= EDGE_TOL_MICROSTEPS * micro + PRINT_TOL,
                f"{name}: borda em {e if e is None else f'{e:+.3f}°'} depois da referência simultânea")
    b.all_to(0)


@register("simultaneous", "duas ou mais", "todas juntas × uma depois da outra: tempos e posições finais",
          min_plates=2)
def test_simultaneous(b: Bench, r: Result):
    b.ensure_homed()
    items = b.items()
    plates = [p for _, p in items]
    tol = max(b.tol(n) for n, _ in items)
    for dist in ([90, 360] if b.args.quick else [90, 360, 1080]):
        b.all_to(0)
        singles = {}
        for name, p in items:
            dt, final = timed(lambda: p.move_to(dist))
            singles[name] = dt
            r.check(abs(final - dist) <= tol, f"{dist}°: {name} sozinha chegou em {final:.2f}° ({dt:.2f} s)")
        b.all_to(0)
        dt, finals = timed(lambda: move_all(plates, dist))
        r.check(all(abs(f - dist) <= tol for f in finals),
                f"{dist}°: juntas chegaram em {', '.join(f'{f:.2f}' for f in finals)}")
        slowest = max(singles.values())
        ratio = dt / slowest
        r.metric(f"ratio_{dist}", round(ratio, 3))
        text = (f"{dist}°: uma depois da outra {sum(singles.values()):.2f} s; juntas {dt:.2f} s "
                f"= {ratio:.2f} × a mais lenta sozinha")
        if ratio <= SLOWDOWN_TOL:
            r.check(True, text)
        else:
            r.warn(text + " — juntas ficam mais lentas (limite de passos/s somados do Uno; "
                          "veja o teste speed-limit)")
    total = sum(p.get_settings().max_speed * b.spd(n) for n, p in items)
    r.info(f"velocidade máxima pedida somada: {total:.0f} passos/s")
    b.all_to(0)


@register("speed-limit", "duas ou mais", "acha a velocidade máxima por lâmina para andarem juntas sem "
                                         "atrasar nem perder passo", min_plates=2, slow=True)
def test_speed_limit(b: Bench, r: Result):
    b.ensure_homed()
    items = b.items()
    plates = [p for _, p in items]
    b.all_to(0)
    ref = reference_edges(b)
    if any(e is None for e in ref.values()):
        r.check(False, "borda do fim de curso não encontrada", str(ref))
        return
    best = None
    try:
        for frac in (0.25, 0.5, 0.75, 1.0):
            for name, p in items:
                d0 = b.defaults[name]
                vmax = frac * d0.speed_limit
                p.configure(min_speed=min(d0.min_speed, vmax), max_speed=vmax)
            speed = plates[0].get_settings().max_speed
            steps = sum(p.get_settings().max_speed * b.spd(n) for n, p in items)
            b.all_to(0)
            singles = []
            for p in plates:
                singles.append(timed(lambda: p.move_to(360))[0])
                p.move_to(0)
            t_all, _ = timed(lambda: move_all(plates, 360))
            for _ in range(3):
                b.all_to(0)
                move_all(plates, 360)
            b.all_to(0)
            drifts = {}
            for name, _ in items:
                e = b.edge(name)
                drifts[name] = None if e is None else (e - ref[name]) / b.micro(name)
                # o próximo nível compara com a borda de agora (ou com uma referência nova)
                ref[name] = e if e is not None else reference_edges(b, [name])[name]
            if any(e is None for e in ref.values()):
                r.check(False, "borda do fim de curso não encontrada nem depois de refazer a referência")
                return
            ratio = t_all / max(singles)
            lost = any(v is None or abs(v) > LOST_STEPS_TOL_MICROSTEPS for v in drifts.values())
            drift_txt = ", ".join(f"{n} {'?' if v is None else f'{v:+.0f}'} µp" for n, v in drifts.items())
            text = (f"{speed:.0f} °/s por lâmina ({frac:.0%} do limite, {steps:.0f} passos/s somados): "
                    f"juntas {ratio:.2f} × mais lentas; borda {drift_txt}")
            r.metric(f"level_{frac}", {"deg_s": round(speed, 1), "ratio": round(ratio, 3), "drift_us": drifts})
            if lost:
                r.check(False, text + " → PERDEU PASSOS")
            elif ratio > SLOWDOWN_TOL:
                r.warn(text + " → juntas ficam mais lentas que o pedido")
            else:
                r.check(True, text)
                best = speed
    finally:
        b.restore_all()
    if best is None:
        r.info("nenhuma das velocidades testadas andou junto sem atrasar")
    else:
        r.info(f"juntas sem atraso nem perda até ≈ {best:.0f} °/s por lâmina "
               f"(padrão do firmware: {b.defaults[items[0][0]].max_speed:.0f} °/s)")


@register("threads", "duas ou mais", "uma thread por lâmina na mesma porta, sem misturar respostas",
          min_plates=2)
def test_threads(b: Bench, r: Result):
    b.ensure_homed()
    items = b.items()
    moves = 4 if b.args.quick else 8
    errors: List[str] = []
    done: Dict[str, int] = {n: 0 for n, _ in items}
    seed = b.rng.randrange(1 << 30)

    def worker(name: str, p: WavePlate, rng: random.Random):
        try:
            for _ in range(moves):
                target = round(rng.uniform(-180, 180), 2)
                final = p.move_to(target)
                if abs(final - snap(target, b.spd(name))) > PRINT_TOL:
                    errors.append(f"{name}: alvo {target} → {final}")
                pos = p.get_position()
                if abs(pos - final) > PRINT_TOL:
                    errors.append(f"{name}: DONE {final} mas POS {pos}")
                done[name] += 1
        except Exception as e:  # noqa: BLE001
            errors.append(f"{name}: {type(e).__name__}: {e}")

    threads = [threading.Thread(target=worker, args=(n, p, random.Random(seed + i)), daemon=True)
               for i, (n, p) in enumerate(items)]
    t0 = time.perf_counter()
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=moves * 30)
    dt = time.perf_counter() - t0
    r.check(not errors and all(v == moves for v in done.values()),
            f"{len(threads)} threads × {moves} movimentos em {dt:.1f} s, sem erro nem resposta trocada",
            "; ".join(errors[:4]))
    b.stop_all()
    b.all_to(0)


@register("elliptec", "duas ou mais", "ArduinoElliptec/EasyMotor como no código do laboratório (Bob=0, Alice=2)")
def test_elliptec(b: Bench, r: Result):
    items = b.items()
    ports = {p.port for _, p in items}
    if len(ports) > 1:
        raise SkipTest("lâminas em portas diferentes; a Elliptec usa um cabo só")
    port = ports.pop()
    addrs = [0, 2, 1, 3][:len(items)]
    mapping = {addr: p.axis for addr, (_, p) in zip(addrs, items)}
    tol = max(b.tol(n) for n, _ in items)
    spd = b.spd(items[0][0])
    r.info(f"EasyMotor({port!r}, axes={mapping})")
    stage = ArduinoElliptec(port, axes=mapping)
    try:
        r.check(stage.get_connected_addrs() == sorted(mapping), f"get_connected_addrs() = {stage.get_connected_addrs()}")
        dt, homes = timed(lambda: stage.home(addr=list(mapping)))
        r.check(isinstance(homes, list) and all(abs(h) <= tol for h in homes),
                f"home(addr={list(mapping)}) → {homes} em {dt:.1f} s")
        for addr in mapping:
            target = 30 + addr
            v = stage.move_to(target, addr=addr)
            pos_int = stage.get_position(addr=addr)
            pos_str = stage.get_position(addr=f"{addr:X}")
            r.check(max(abs(v - target), abs(pos_int - target), abs(pos_str - target)) <= tol,
                    f"move_to({target}, addr={addr}) e get_position(addr={addr} / '{addr:X}')")
        v = stage.move_to(45, addr=list(mapping))
        r.check(isinstance(v, list) and all(abs(x - 45) <= tol for x in v),
                f"move_to(45, addr={list(mapping)}) → {v}")
        v = stage.move_by(-10, addr="all")
        r.check(isinstance(v, list) and all(abs(x - 35) <= tol for x in v), f"move_by(-10, addr='all') → {v}")
        first, last = min(mapping), max(mapping)
        v = stage.move_to(20, addr=first, max_tries=5)
        r.check(abs(v - 20) <= tol, "move_to(..., max_tries=5) aceito por compatibilidade")
        stage.set_default_addr(last)
        stage.move_to(12)
        r.check(abs(stage.get_position(addr=last) - 12) <= tol, f"set_default_addr({last}) + move_to(12)")
        bad = [f"addr={a!r}: {msg}" for a in (5 if 5 not in mapping else 6, 16, "Z")
               if (msg := expect_error(lambda: stage.move_to(10, addr=a), ValueError))]
        r.check(not bad, "endereços inválidos dão ValueError", "; ".join(bad))

        if 0 in mapping and 2 in mapping:
            h_alice, h_bob = 99.0, 55.0                     # controle_motores.py
            alice = {"H": h_alice, "V": h_alice + 45, "D": h_alice + 22.5, "A": h_alice + 67.5}
            bob = {"H": h_bob, "V": h_bob + 45, "D": h_bob + 22.5, "A": h_bob + 67.5}
            bad, times = [], []
            for pair in ("HH", "HV", "VH", "VV", "DD", "DA", "AD", "AA"):
                dt, _ = timed(lambda: (stage.move_to(alice[pair[0]], addr=2),
                                       stage.move_to(bob[pair[1]], addr=0)))
                times.append(dt)
                got = (stage.get_position(addr=2), stage.get_position(addr=0))
                want = (snap(alice[pair[0]], spd), snap(bob[pair[1]], spd))
                if max(abs(g - w) for g, w in zip(got, want)) > PRINT_TOL:
                    bad.append(f"{pair}: {got} ≠ {want}")
            r.check(not bad, f"sequência de bases do controle_motores.py (8 pares, Alice=2, Bob=0): "
                             f"{statistics.mean(times):.2f} s por par em média", "; ".join(bad))
        all_addrs = list(mapping)
        stage.home(addr=all_addrs)
        stage.move_to(30, addr=all_addrs)
        stage.set_zero(addr="all")
        stage.move_by(15, addr="all")
        v = stage.calibrate(addr=all_addrs)
        r.check(isinstance(v, list) and all(abs(x - 15) <= tol for x in v),
                f"set_zero(addr='all') em 30° + calibrate(addr={all_addrs}) → {v}")
        stage.home(addr=all_addrs)                       # 0 do firmware de volta ao fim de curso
        r.check(stage.is_opened(), "is_opened() = True")
    finally:
        stage.close()
    r.check(not stage.is_opened(), "close() fecha os endereços")
    for _, p in items:                                   # o stage ligou o desligamento automático
        p.disable_when_idle(False)
    positions = [p.get_position() for _, p in items]
    r.check(True, f"as outras conexões na mesma porta continuam funcionando depois do close() "
                  f"({', '.join(f'{x:.2f}°' for x in positions)})")
    b.all_to(0)


# ══════════════════════════════════════════════════════════════════════════════
# Resistência
# ══════════════════════════════════════════════════════════════════════════════
@register("travel-limits", "resistência", "todas juntas até +limite, -limite e de volta; sem perder passo",
          slow=True)
def test_travel_limits(b: Bench, r: Result):
    b.ensure_homed()
    items = b.items()
    plates = [p for _, p in items]
    b.all_to(0)
    ref = reference_edges(b)
    for kind in ("max_angle", "min_angle"):
        targets = [getattr(b.defaults[n], kind) for n, _ in items]
        dt, finals = timed(lambda: move_all(plates, targets))
        r.check(all(abs(f - t) <= PRINT_TOL for f, t in zip(finals, targets)),
                f"todas em {targets[0]:+.0f}° em {dt:.0f} s ({', '.join(f'{f:.2f}' for f in finals)})")
    b.all_to(0)
    check_edges_after(b, r, ref, "depois de ~20 voltas")


def check_edges_after(b: Bench, r: Result, ref: Dict[str, Optional[float]], when: str):
    """Compara a borda do fim de curso de cada lâmina com a medida antes (passos perdidos)."""
    for name, _ in b.items():
        if ref.get(name) is None:
            r.check(False, f"{name}: borda do fim de curso não encontrada antes do teste")
            continue
        e = b.edge(name)
        if e is None:
            r.check(False, f"{name}: {when}, a borda não está mais a ±3° do zero: perdeu passos "
                           f"(mais de 3°) ou o fim de curso parou de funcionar")
            continue
        drift = (e - ref[name]) / b.micro(name)
        r.check(abs(drift) <= LOST_STEPS_TOL_MICROSTEPS,
                f"{name}: {when}, a borda mudou {drift:+.0f} micropasso(s)")


@register("endurance", "resistência", "muitos movimentos aleatórios (juntas e em sequência, velocidades variadas)",
          slow=True)
def test_endurance(b: Bench, r: Result):
    b.ensure_homed()
    items = b.items()
    plates = [p for _, p in items]
    b.all_to(0)
    ref = reference_edges(b)
    errors: List[str] = []
    moves = 0
    t0 = time.perf_counter()
    try:
        for cycle in range(b.args.cycles):
            for name, p in items:
                d0 = b.defaults[name]
                vmax = d0.max_speed * b.rng.uniform(0.3, 1.0)
                p.configure(min_speed=min(d0.min_speed, vmax), max_speed=vmax)
            targets = [round(b.rng.uniform(-720, 720), 3) for _ in plates]
            together = len(plates) > 1 and b.rng.random() < 0.5
            try:
                finals = move_all(plates, targets) if together else [p.move_to(t) for p, t in zip(plates, targets)]
                for (name, _), f, t in zip(items, finals, targets):
                    if abs(f - snap(t, b.spd(name))) > PRINT_TOL:
                        errors.append(f"ciclo {cycle}: {name} alvo {t} → {f}")
                moves += len(plates)
            except (WavePlateError, TimeoutError) as e:
                errors.append(f"ciclo {cycle}: {type(e).__name__}: {e}")
                b.stop_all()
            print(f"\r    ... ciclo {cycle + 1}/{b.args.cycles}", end="", flush=True)
        print()
    finally:
        b.restore_all()
    dt = time.perf_counter() - t0
    r.check(not errors, f"{moves} movimentos em {b.args.cycles} ciclos ({dt:.0f} s) sem erro",
            "; ".join(errors[:4]))
    b.all_to(0)
    check_edges_after(b, r, ref, "depois dos ciclos")


@register("torque", "resistência", "você sente o driver segurando / soltando; ENABLE comum do CNC Shield",
          interactive=True)
def test_torque(b: Bench, r: Result):
    items = b.items()
    for _, p in items:
        p.enable()
    first, p0 = items[0]
    r.check(is_yes(ask(f"Driver ligado. Tente girar a lâmina {first} com a mão, SEM forçar. "
                       f"Está travada? [s/n]")), "driver ligado segura a lâmina")
    same_port = [p for _, p in items if p.port == p0.port]
    if len(same_port) > 1:
        p0.disable()
        r.check(is_yes(ask(f"Só {first} pediu para desligar. Ela continua travada? [s/n]")),
                "ENABLE comum: o driver só desliga quando todos os eixos da placa desligam")
    for p in same_port:
        p.disable()
    r.check(is_yes(ask(f"Todas desligadas. Agora {first} gira livre? [s/n]")), "driver desligado solta a lâmina")
    for _, p in items:
        p.enable()
    b.homed.clear()                                  # mexeu com a mão
    b.ensure_homed()


@register("reconnect", "resistência", "segundo objeto na mesma porta não reinicia; fechar e reabrir reinicia")
def test_reconnect(b: Bench, r: Result):
    b.ensure_homed()
    name, p = b.items()[0]
    tol = b.tol(name)
    p.move_to(37.0)
    extra = WavePlate(p.port, p.axis)
    pos = extra.get_position()
    extra.close(disable=False)
    r.check(abs(pos - 37.0) <= tol, f"outro WavePlate na mesma porta lê {pos:.2f}° (conexão compartilhada, "
                                    f"sem reiniciar o Arduino)")
    r.check(abs(p.get_position() - 37.0) <= tol, "o objeto original continua funcionando depois do close() do outro")

    by_port: Dict[str, List[str]] = {}
    for n, q in b.items():
        by_port.setdefault(q.port, []).append(n)
    for port, names in by_port.items():
        axes = {n: b.plates[n].axis for n in names}
        for n in names:
            b.plates[n].close()
        t0 = time.perf_counter()
        for n in names:
            b.plates[n] = WavePlate(port, axes[n], disable_when_idle=False,
                                    move_timeout=b.args.move_timeout)
        dt = time.perf_counter() - t0
        positions = {n: b.plates[n].get_position() for n in names}
        b.homed.difference_update(names)
        r.check(True, f"{port}: fechou e reabriu em {dt:.1f} s")
        if all(abs(v) <= PRINT_TOL for v in positions.values()):
            r.info(f"{port}: a posição voltou a 0.00° → reabrir a porta REINICIA o Arduino e a "
                   f"referência se perde; depois de reconectar, faça home() de novo")
        else:
            r.info(f"{port}: reabrir NÃO reiniciou o Arduino (posições {positions})")
    b.ensure_homed()
    b.all_to(0)


# ══════════════════════════════════════════════════════════════════════════════
# Execução
# ══════════════════════════════════════════════════════════════════════════════
def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="Testes de hardware e software das lâminas motorizadas.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Use --list para ver os testes.")
    ap.add_argument("--port", default=DEFAULT_PORT, help=f"porta padrão (padrão {DEFAULT_PORT})")
    ap.add_argument("--plates", nargs="+", default=list(DEFAULT_PLATES), metavar="EIXO|PORTA:EIXO",
                    help=f"lâminas a testar (padrão: {' '.join(DEFAULT_PLATES)})")
    ap.add_argument("--only", nargs="+", metavar="TESTE", help="roda só estes testes")
    ap.add_argument("--skip", nargs="+", default=[], metavar="TESTE", help="pula estes testes")
    ap.add_argument("--quick", action="store_true", help="pula os demorados e repete menos")
    ap.add_argument("--no-interactive", dest="interactive", action="store_false",
                    help="pula os testes que fazem perguntas")
    ap.add_argument("--yes", "-y", action="store_true", help="começa sem pedir confirmação")
    ap.add_argument("--list", action="store_true", help="lista os testes e sai")
    ap.add_argument("--seed", type=int, help="semente dos movimentos aleatórios")
    ap.add_argument("--cycles", type=int, default=30, help="ciclos do teste endurance (padrão 30)")
    ap.add_argument("--microstep", type=float, default=MICROSTEP, help=f"microstep no firmware (padrão {MICROSTEP})")
    ap.add_argument("--reduction", type=float, default=REDUCTION, help=f"redução no firmware (padrão {REDUCTION:g})")
    ap.add_argument("--home-direction", type=int, choices=(-1, 1), default=HOME_DIRECTION,
                    help="sentido da busca do zero no firmware (sentido_ref)")
    ap.add_argument("--move-timeout", type=float, default=120.0, help="s máximos por movimento")
    args = ap.parse_args(argv)
    if not sys.stdin.isatty():
        args.interactive = False
    return args


def print_list():
    group = None
    for t in TESTS:
        if t.group != group:
            group = t.group
            print(f"\n{group}:")
        flags = " ".join(f for f, on in (("[demorado]", t.slow), ("[pergunta]", t.interactive),
                                         (f"[≥{t.min_plates} lâminas]", t.min_plates > 1)) if on)
        print(f"  {t.name:<14} {t.description} {flags}")


def select_tests(args) -> List[TestSpec]:
    names = {t.name for t in TESTS}
    unknown = [n for n in (args.only or []) + args.skip if n not in names]
    if unknown:
        raise SystemExit(f"teste(s) desconhecido(s): {', '.join(unknown)} (veja --list)")
    chosen = []
    for t in TESTS:
        if args.only and t.name not in args.only:
            continue
        if t.name in args.skip:
            continue
        if t.slow and args.quick and not args.only:
            continue
        if t.interactive and not args.interactive:
            continue
        chosen.append(t)
    return chosen


def run_one(bench: Bench, spec: TestSpec, index: int, total: int, results: List[Result]):
    r = Result(spec.name)
    results.append(r)
    print(f"\n[{index}/{total}] {spec.name} — {spec.description}", flush=True)
    t0 = time.perf_counter()
    try:
        if len(bench.plates) < spec.min_plates:
            raise SkipTest(f"precisa de pelo menos {spec.min_plates} lâminas")
        spec.func(bench, r)
    except SkipTest as e:
        r.status = "SKIP"
        r.headline = str(e)
        r._log("-", f"pulado: {e}")
    except KeyboardInterrupt:
        r.error("interrompido pelo usuário")
        raise
    except Exception as e:  # noqa: BLE001
        r.error(f"{type(e).__name__}: {e}")
        r.lines.extend(traceback.format_exc().splitlines())
        bench.stop_all()
        bench.restore_all()
    finally:
        r.duration = time.perf_counter() - t0
        print(f"    => {r.status} ({r.duration:.1f} s)", flush=True)


def save_report(args, bench: Bench, results: List[Result], seed: int) -> Path:
    RESULTS_DIR.mkdir(exist_ok=True)
    path = RESULTS_DIR / f"hardware_{datetime.now():%Y%m%d_%H%M%S}.json"
    data = {
        "date": datetime.now().isoformat(timespec="seconds"),
        "plates": {n: f"{p.port}:{p.axis}" for n, p in bench.plates.items()},
        "seed": seed,
        "args": vars(args),
        "settings": {n: asdict(s) for n, s in bench.defaults.items()},
        "tests": [{"name": r.name, "status": r.status, "duration_s": round(r.duration, 2),
                   "metrics": r.metrics, "log": r.lines} for r in results],
    }
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
    return path


def print_summary(results: List[Result], path: Optional[Path]):
    print("\n" + "=" * 78)
    print("RESUMO")
    for r in results:
        extra = ""
        if r.status != "PASS" and r.headline:
            extra = "  " + (r.headline if len(r.headline) <= 90 else r.headline[:87] + "...")
        print(f"  {r.status:<5} {r.name:<14} {r.duration:6.1f} s{extra}")
    counts = {s: sum(r.status == s for r in results) for s in ("PASS", "WARN", "FAIL", "ERROR", "SKIP")}
    print("  " + ", ".join(f"{v} {k}" for k, v in counts.items() if v))
    if path:
        print(f"Relatório completo: {path}")


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass
    args = parse_args(argv)
    if args.list:
        print_list()
        return 0
    selected = select_tests(args)
    seed = args.seed if args.seed is not None else random.randrange(100000)

    print(f"Lâminas: {', '.join(args.plates)} (porta padrão {args.port}) — {len(selected)} testes, "
          f"semente {seed}")
    print("As lâminas vão girar várias voltas: confira que nada impede a rotação.")
    if args.interactive and not args.yes:
        try:
            input("Enter para começar (Ctrl+C cancela) ")
        except (EOFError, KeyboardInterrupt):
            print()
            return 1

    bench = Bench(args, seed)
    try:
        bench.open()
    except (serial.SerialException, TimeoutError, WavePlateError, ValueError) as e:
        print(f"ERRO ao conectar: {e}")
        return 2

    results: List[Result] = []
    path = None
    try:
        for i, spec in enumerate(selected, 1):
            run_one(bench, spec, i, len(selected), results)
    except KeyboardInterrupt:
        print("\nInterrompido pelo usuário — parando as lâminas.")
        bench.stop_all()
    finally:
        bench.restore_all()
        bench.close()
        try:
            path = save_report(args, bench, results, seed)
        except OSError as e:
            print(f"(não consegui salvar o relatório: {e})")
        print_summary(results, path)
    return 1 if any(r.status in ("FAIL", "ERROR") for r in results) else 0


if __name__ == "__main__":
    sys.exit(main())
