"""
Testes ópticos de histerese das lâminas motorizadas: folga das engrenagens
(backlash), repetibilidade, passos perdidos, efeito de desligar o driver e
estabilidade, medidos com polarizadores e um detector de potência.

MONTAGEM
    laser → P1 (polarizador fixo) → lâmina motorizada → P2 (analisador fixo) → detector

    --element hwp        λ/2 entre polarizadores: sinal ∝ cos²(2θ)        período 90°
    --element qwp        λ/4 entre polarizadores: sinal ∝ 1 − ½·sin²(2θ)  período 90°
                         (contraste máximo 1/3: funciona, mas é 3× menos sensível)
    --element polarizer  polarizador na montagem motorizada, sem P2:
                         sinal ∝ cos²(θ) (lei de Malus)                    período 180°

    P1 e P2 paralelos ou cruzados, tanto faz: o ajuste acha a fase sozinho.
    Deixe o laser aquecer, centre o feixe na lâmina e evite luz ambiente.

DETECTOR
    --detector powermeter  Thorlabs PM pelo EASYPM.py de bell-measurements-main (padrão)
    --detector timetagger  taxa de contagem de um canal da Swabian TimeTagger
    --detector manual      você digita a leitura (para testar sem instrumento)

COMO A HISTERESE É MEDIDA
    O sinal S(θ) = a0 + B·cos(n(θ − φ)) é ajustado por mínimos quadrados.
    No ponto de maior inclinação, uma rotação pequena vira variação de sinal
    (δθ = δS / S'), o que mostra deslocamentos de centésimos de grau. A folga
    aparece como a diferença entre chegar a um ângulo subindo e chegar descendo.

    Nos testes longos (repeat, drift, speed) a posição é lida de forma
    diferencial, em dois pontos de inclinação oposta (θ₊ e θ₊ + 180°/n):
        δ = (S₊ − S₋)/(S₊ + S₋) / (V·n·π/180)       V = B/a0 (contraste)
    o que cancela a variação de potência do laser.

TESTES (--list)
    sweep      curva completa subindo e descendo: fase, contraste, folga, ângulo dos eixos
    approach   chegar ao mesmo ângulo por baixo e por cima, com várias distâncias
    loop       laço de histerese: vai e volta em passos pequenos no ponto de trabalho
    idle       quanto a lâmina se mexe quando o driver desliga (disable_when_idle)
    stability  ruído e deriva parada: driver ligado × desligado, e ruído do laser
    repeat     repetibilidade de posicionamento, sem e com referência (home) no meio
    drift      passos perdidos: muitos movimentos aleatórios e volta ao ponto de trabalho
    speed      folga e passos perdidos com perfis de velocidade diferentes
    crosstalk  a outra lâmina andando perturba esta? (precisa de --other)

Uso:
    python hysteresis_test.py --plate X --element hwp
    python hysteresis_test.py --plate COM4:Y --element polarizer --only sweep loop
    python hysteresis_test.py --plate X --other Y --only sweep crosstalk
    python hysteresis_test.py --quick                     # ~metade do tempo

Resultados (CSV de cada teste, resumo JSON e gráficos PNG se houver matplotlib)
em test_results/hysteresis_<eixo>_<data>/.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import statistics
import sys
import time
import traceback
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np
import serial

from waveplate import WavePlate, WavePlateError

DEFAULT_PORT = "COM4"
DEFAULT_PLATE = "X"
MICROSTEP = 8                 # EIXOS_CFG: microstep (micropassos por passo inteiro)
LAB_DIR = Path(__file__).resolve().parent / "bell-measurements-main"
RESULTS_DIR = Path(__file__).resolve().parent / "test_results"

HARMONIC = {"hwp": 4, "qwp": 4, "polarizer": 2}
MAX_VISIBILITY = {"hwp": 1.0, "qwp": 1 / 3, "polarizer": 1.0}

REPEAT_WARN_MICROSTEPS = 2    # repetibilidade (σ) acima disso vira aviso
RESIDUAL_WARN = 0.05          # resíduo do ajuste acima de 5 % da amplitude vira aviso

# cores dos gráficos (paleta de referência da skill de visualização)
UP_COLOR, DOWN_COLOR = "#2a78d6", "#eb6834"
INK, MUTED, GRID, AXIS, SURFACE = "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"


# ══════════════════════════════════════════════════════════════════════════════
# Detectores
# ══════════════════════════════════════════════════════════════════════════════
class Detector:
    name = "?"
    unit = ""

    def read(self) -> float:
        raise NotImplementedError

    def close(self):
        pass


class PowerMeterDetector(Detector):
    """Thorlabs PM100 pelo EASYPM.PowerMeter do código do laboratório."""
    name, unit = "power meter", "W"

    def __init__(self, args):
        sys.path.insert(0, str(Path(args.lab_dir).resolve()))
        from EASYPM import PowerMeter            # precisa do TLPMX.py e da DLL da Thorlabs
        self.pm = PowerMeter(args.wavelength)

    def read(self) -> float:
        return float(self.pm.measure())

    def close(self):
        self.pm.close()


class TimeTaggerDetector(Detector):
    """Taxa de contagem média de um canal durante --gate segundos."""
    name, unit = "TimeTagger", "contagens/s"

    def __init__(self, args):
        import TimeTagger
        self.tt = TimeTagger
        self.tagger = TimeTagger.createTimeTagger()
        self.rate = TimeTagger.Countrate(self.tagger, [args.channel])
        self.gate_ps = int(args.gate * 1e12)

    def read(self) -> float:
        self.rate.startFor(self.gate_ps)
        self.rate.waitUntilFinished()
        return float(self.rate.getData()[0])

    def close(self):
        self.tt.freeTimeTagger(self.tagger)


class ManualDetector(Detector):
    name, unit = "manual", ""

    def __init__(self, args):
        args.samples = 1

    def read(self) -> float:
        while True:
            text = input("\n    leitura do detector → ").strip().replace(",", ".")
            try:
                return float(text)
            except ValueError:
                print("    ! digite um número")


DETECTORS: Dict[str, Callable] = {
    "powermeter": PowerMeterDetector,
    "timetagger": TimeTaggerDetector,
    "manual": ManualDetector,
}


# ══════════════════════════════════════════════════════════════════════════════
# Modelo e ajuste
# ══════════════════════════════════════════════════════════════════════════════
@dataclass
class Fit:
    """S(θ) = offset + amp·cos(n·(θ − phase)), θ em graus, amp > 0."""
    n: int
    offset: float
    amp: float
    phase: float
    rms: float
    points: int = 0

    @property
    def period(self) -> float:
        return 360.0 / self.n

    @property
    def visibility(self) -> float:
        return self.amp / self.offset if self.offset else float("nan")

    @property
    def max_slope(self) -> float:
        """|dS/dθ| máximo, por grau."""
        return self.amp * self.n * math.pi / 180.0

    def value(self, theta):
        return self.offset + self.amp * np.cos(np.radians(self.n * (np.asarray(theta, float) - self.phase)))

    def slope(self, theta):
        return -self.max_slope * np.sin(np.radians(self.n * (np.asarray(theta, float) - self.phase)))

    def nearest(self, near: float, u: float) -> float:
        """Ângulo mais perto de `near` em que n·(θ − phase) ≡ u (mod 360)."""
        k = round((self.n * (near - self.phase) - u) / 360.0)
        return self.phase + (u + 360.0 * k) / self.n

    def invert(self, signal: float, near: float) -> float:
        """Ângulo com esse sinal no ramo da curva mais perto de `near`."""
        c = max(-1.0, min(1.0, (signal - self.offset) / self.amp))
        u = math.degrees(math.acos(c))
        base = self.n * (near - self.phase)
        best = None
        for cand_u in (u, -u):
            cand = cand_u + 360.0 * round((base - cand_u) / 360.0)
            if best is None or abs(cand - base) < abs(best - base):
                best = cand
        return self.phase + best / self.n


def fit_curve(theta, signal, n: int) -> Fit:
    x = np.radians(n * np.asarray(theta, float))
    y = np.asarray(signal, float)
    m = np.column_stack([np.ones_like(x), np.cos(x), np.sin(x)])
    coef, *_ = np.linalg.lstsq(m, y, rcond=None)
    a0, a, b = coef
    amp = math.hypot(a, b)
    phase = (math.degrees(math.atan2(b, a)) / n) % (360.0 / n)
    rms = float(np.sqrt(np.mean((y - m @ coef) ** 2)))
    return Fit(n, float(a0), float(amp), float(phase), rms, len(y))


def wrap(x: float, period: float) -> float:
    """Leva x para (−período/2, período/2]."""
    return (x + period / 2) % period - period / 2


def mid_fit(up: Fit, down: Fit) -> Fit:
    """Curva "no meio" das duas direções: o referencial em que a folga é ±b/2."""
    b = wrap(up.phase - down.phase, up.period)
    return Fit(up.n, (up.offset + down.offset) / 2, (up.amp + down.amp) / 2,
               (down.phase + b / 2) % up.period, (up.rms + down.rms) / 2, up.points + down.points)


def stdev(values: Sequence[float]) -> float:
    return statistics.stdev(values) if len(values) > 1 else 0.0


# ══════════════════════════════════════════════════════════════════════════════
# Infraestrutura
# ══════════════════════════════════════════════════════════════════════════════
class SkipTest(Exception):
    pass


STATUS_RANK = {"PASS": 0, "SKIP": 0, "WARN": 1, "FAIL": 2, "ERROR": 3}


class Result:
    def __init__(self, name: str):
        self.name = name
        self.status = "PASS"
        self.lines: List[str] = []
        self.metrics: Dict[str, object] = {}
        self.duration = 0.0
        self.headline = ""

    def _log(self, mark: str, text: str):
        line = f"{mark:<6}{text}"
        print("    " + line, flush=True)
        self.lines.append(line)

    def _escalate(self, status: str, text: str):
        if STATUS_RANK[status] > STATUS_RANK[self.status]:
            self.status, self.headline = status, text

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
        self.metrics[key] = to_json(value)


def to_json(value):
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, (list, tuple)):
        return [to_json(v) for v in value]
    if isinstance(value, dict):
        return {k: to_json(v) for k, v in value.items()}
    return value


@dataclass
class TestSpec:
    name: str
    description: str
    func: Callable


TESTS: List[TestSpec] = []


def register(name: str, description: str):
    def decorator(func):
        TESTS.append(TestSpec(name, description, func))
        return func
    return decorator


def parse_plate(spec: str, default_port: str) -> Tuple[str, str]:
    if ":" in spec:
        port, axis = spec.rsplit(":", 1)
        return port, axis.upper()
    return default_port, spec.upper()


_progress_time = 0.0


def progress(text: str):
    """Linha de progresso que se sobrescreve (só no terminal, no máximo 5× por segundo)."""
    global _progress_time
    if not sys.stdout.isatty() or time.monotonic() - _progress_time < 0.2:
        return
    _progress_time = time.monotonic()
    print(f"\r    ... {text}      ", end="", flush=True)


def progress_end():
    if sys.stdout.isatty():
        print("\r" + " " * 70 + "\r", end="", flush=True)


# ── gráficos (opcionais: só se o matplotlib estiver instalado) ─────────────────
def pyplot():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt
    except ImportError:
        return None


def styled_axes(ax, title: str, xlabel: str, ylabel: str):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=INK, labelsize=9)
    ax.set_title(title, color="#0b0b0b", fontsize=11, loc="left")
    ax.set_xlabel(xlabel, color=INK)
    ax.set_ylabel(ylabel, color=INK)


def save_figure(plt, fig, path: Path):
    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor=SURFACE)
    plt.close(fig)


# ── bancada ───────────────────────────────────────────────────────────────────
class Bench:
    def __init__(self, args, plate: WavePlate, detector: Detector, other: Optional[WavePlate],
                 out_dir: Path):
        self.args = args
        self.plate = plate
        self.det = detector
        self.other = other
        self.out_dir = out_dir
        self.defaults = plate.get_settings()
        self.spd = self.defaults.steps_per_degree
        self.micro = 1.0 / self.spd
        self.full_step = args.microstep * self.micro
        self.n = args.harmonic or HARMONIC[args.element]
        self.period = 360.0 / self.n
        self.range = args.range or 2 * self.period
        self.step = args.step or self.period / 45
        self.rng = random.Random(args.seed)
        self.calib: Optional[Fit] = None
        self.backlash: Optional[float] = None
        self.noise_deg: Optional[float] = None
        self.work = self.work_neg = self.peak = 0.0
        self._mates: List[WavePlate] = []

    # ── medidas ────────────────────────────────────────────────────────────
    def measure(self, settle: Optional[float] = None) -> Tuple[float, float]:
        """Espera assentar e devolve (média, desvio) de --samples leituras."""
        time.sleep(self.args.settle if settle is None else settle)
        values = [self.det.read() for _ in range(self.args.samples)]
        return statistics.fmean(values), stdev(values)

    def goto(self, angle: float, approach: int = 0) -> float:
        """approach=+1 chega subindo (vindo de baixo), -1 descendo, 0 direto."""
        if approach:
            self.plate.move_to(angle - approach * self.args.pre)
        return self.plate.move_to(angle)

    def read_angle(self, near: Optional[float] = None, settle: Optional[float] = None) -> Tuple[float, float]:
        """(ângulo físico estimado pelo sinal, sinal), no ramo perto de `near`."""
        s, _ = self.measure(settle)
        return self.calib.invert(s, self.work if near is None else near), s

    def read_offset(self, approach: int = +1) -> float:
        """
        Deslocamento da lâmina (graus) em relação ao referencial da calibração,
        lido nos dois pontos de inclinação oposta: imune à potência do laser.
        """
        if approach >= 0:
            self.goto(self.work, +1)
            s_plus, _ = self.measure()
            self.plate.move_to(self.work_neg)
            s_minus, _ = self.measure()
        else:
            self.goto(self.work_neg, -1)
            s_minus, _ = self.measure()
            self.plate.move_to(self.work)
            s_plus, _ = self.measure()
        ratio = (s_plus - s_minus) / (s_plus + s_minus)
        return math.degrees(ratio / (self.calib.visibility * self.n))

    def grid(self, start: float, span: float, step: float) -> np.ndarray:
        """Grade de ângulos arredondada ao micropasso."""
        k_step = max(1, round(step * self.spd))
        k0 = round(start * self.spd)
        k1 = round((start + span) * self.spd)
        return np.arange(k0, k1 + 1, k_step) / self.spd

    def sweep_once(self, grid: np.ndarray, label: str = "") -> Tuple[list, list]:
        """Sobe e desce pela grade; devolve [(sinal, desvio)] de cada direção, na ordem da grade."""
        up, down = [], []
        self.goto(grid[0], +1)
        for i, angle in enumerate(grid):
            self.plate.move_to(angle)
            up.append(self.measure())
            progress(f"{label}subindo {i + 1}/{len(grid)}")
        self.goto(grid[-1], -1)
        for i, angle in enumerate(grid[::-1]):
            self.plate.move_to(angle)
            down.append(self.measure())
            progress(f"{label}descendo {i + 1}/{len(grid)}")
        progress_end()
        down.reverse()
        return up, down

    # ── calibração ─────────────────────────────────────────────────────────
    def set_calibration(self, fit: Fit):
        self.calib = fit
        center = self.args.work if self.args.work is not None else self.args.start + self.range / 2
        self.work = fit.nearest(center, -90.0)            # inclinação positiva
        self.work_neg = self.work + self.period / 2       # inclinação negativa
        self.peak = fit.nearest(self.work, 0.0)           # máximo (insensível ao ângulo)

    def need_calib(self) -> Fit:
        if self.calib is None:
            print("    (sem calibração: varredura rápida de um período, subindo e descendo)")
            grid = self.grid(self.args.start + self.range / 2 - self.period / 2, self.period, self.period / 36)
            up, down = self.sweep_once(grid, "calibração: ")
            f_up = fit_curve(grid, [s for s, _ in up], self.n)
            f_down = fit_curve(grid, [s for s, _ in down], self.n)
            self.backlash = wrap(f_up.phase - f_down.phase, self.period)
            self.set_calibration(mid_fit(f_up, f_down))
            print(f"    (contraste {self.calib.visibility:.3f}, folga ≈ {self.backlash:.3f}°, "
                  f"ponto de trabalho {self.work:.2f}°)")
        if self.noise_deg is None:
            self.measure_noise()
        return self.calib

    def measure_noise(self, count: int = 10) -> float:
        """Ruído do sinal parado no ponto de trabalho, em graus equivalentes."""
        self.goto(self.work, +1)
        values = [self.measure(None if i == 0 else 0.02)[0] for i in range(count)]
        self.noise_deg = stdev(values) / self.calib.max_slope
        return self.noise_deg

    def threshold(self) -> float:
        """Menor deslocamento que vale a pena chamar de real."""
        return max(3 * (self.noise_deg or 0.0), 1.5 * self.micro)

    # ── driver ─────────────────────────────────────────────────────────────
    def driver_off(self):
        """
        Desliga o driver de verdade. No CNC Shield o ENABLE é comum: só desliga
        quando todos os eixos da placa pedem, então desabilita todos.
        """
        if not self._mates:
            axes = [a for a in self.plate._serial.axes if a != self.plate.axis]
            self._mates = [WavePlate(self.plate.port, a) for a in axes]
        for m in self._mates:
            m.disable()
        self.plate.disable()

    def driver_on(self):
        self.plate.enable()

    # ── arquivos ───────────────────────────────────────────────────────────
    def save_csv(self, name: str, header: Sequence[str], rows) -> Path:
        path = self.out_dir / f"{name}.csv"
        with path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(header)
            for row in rows:
                w.writerow([f"{v:.6g}" if isinstance(v, float) else v for v in row])
        return path

    def close(self):
        for m in self._mates:
            try:
                m.close(disable=False)
            except Exception:  # noqa: BLE001
                pass


# ══════════════════════════════════════════════════════════════════════════════
# Testes
# ══════════════════════════════════════════════════════════════════════════════
@register("sweep", "curva completa subindo e descendo: fase, contraste, folga, ângulo dos eixos")
def test_sweep(b: Bench, r: Result):
    grid = b.grid(b.args.start, b.range, b.step)
    cycles = b.args.cycles
    rows, per_cycle = [], []
    all_up, all_down = [], []
    for c in range(cycles):
        up, down = b.sweep_once(grid, f"ciclo {c + 1}/{cycles}: ")
        su, sd = [s for s, _ in up], [s for s, _ in down]
        rows += [(c, "up", float(a), s, e) for a, (s, e) in zip(grid, up)]
        rows += [(c, "down", float(a), s, e) for a, (s, e) in zip(grid, down)]
        per_cycle.append((fit_curve(grid, su, b.n), fit_curve(grid, sd, b.n)))
        all_up.append(su)
        all_down.append(sd)
    b.save_csv("sweep", ["cycle", "direction", "angle_deg", "signal", "signal_std"], rows)

    theta = np.tile(grid, cycles)
    f_up = fit_curve(theta, np.concatenate(all_up), b.n)
    f_down = fit_curve(theta, np.concatenate(all_down), b.n)
    cal = mid_fit(f_up, f_down)
    b.backlash = wrap(f_up.phase - f_down.phase, b.period)
    b.set_calibration(cal)
    b.measure_noise()

    backlashes = [wrap(u.phase - d.phase, b.period) for u, d in per_cycle]
    for c, ((u, d), bl) in enumerate(zip(per_cycle, backlashes)):
        r.info(f"ciclo {c + 1}: fase subindo {u.phase:.3f}°, descendo {d.phase:.3f}° → "
               f"folga {bl:+.4f}°")

    vis_max = MAX_VISIBILITY.get(b.args.element, 1.0)
    text = (f"contraste V = {cal.visibility:.3f} (máximo possível para {b.args.element}: {vis_max:.2f}); "
            f"razão de extinção mín/máx = {(cal.offset - cal.amp) / (cal.offset + cal.amp):.4f}")
    if cal.visibility < 0.5 * vis_max:
        r.warn(text + " — baixo: alinhe P1/P2, confira o elemento (--element) e a luz ambiente")
    else:
        r.check(True, text)
    residual = cal.rms / cal.amp
    text = f"resíduo do ajuste cos({b.n}θ): {residual:.1%} da amplitude"
    if residual > RESIDUAL_WARN:
        r.warn(text + " — a curva foge do modelo (feixe na borda, lâmina inclinada, elemento errado?)")
    else:
        r.check(True, text)

    r.info(f"ruído do sinal no ponto de trabalho ({b.work:.2f}°): {b.noise_deg:.4f}° equivalentes "
           f"(micropasso = {b.micro:.4f}°)")
    r.check(True, f"FOLGA (fase subindo − descendo): {b.backlash:+.4f}° = {b.backlash / b.micro:+.1f} "
                  f"micropassos (entre ciclos: σ {stdev(backlashes):.4f}°)")

    phases_up = [u.phase for u, _ in per_cycle]
    drift = wrap(phases_up[-1] - phases_up[0], b.period)
    if cycles > 1:
        text = f"a fase andou {drift:+.4f}° do primeiro ao último ciclo"
        if abs(drift) > b.threshold():
            r.warn(text + " — passos perdidos, deriva térmica ou do laser (veja o teste drift)")
        else:
            r.check(True, text)

    # folga ponto a ponto (só onde a curva é inclinada o bastante)
    local = []
    for c in range(cycles):
        for a, su, sd in zip(grid, all_up[c], all_down[c]):
            if abs(cal.slope(a)) >= 0.5 * cal.max_slope:
                local.append((float(a), cal.invert(sd, a) - cal.invert(su, a)))
    if local:
        values = [v for _, v in local]
        text = (f"folga ponto a ponto ao longo da volta: média {statistics.fmean(values):.4f}°, "
                f"σ {stdev(values):.4f}°, de {min(values):.4f}° a {max(values):.4f}°")
        if stdev(values) > max(3 * b.noise_deg * math.sqrt(2), b.micro):
            r.warn(text + " — varia com o ângulo (engrenagem excêntrica ou com defeito?)")
        else:
            r.info(text)

    p = b.period
    max_at = cal.phase % p
    min_at = (cal.phase + p / 2) % p
    r.info(f"máximo do sinal em {max_at:.2f}° e mínimo em {min_at:.2f}° (+ múltiplos de {p:g}°)")
    if b.args.element in ("hwp", "qwp"):
        r.info("eixo (rápido ou lento) da lâmina alinhado com P1: no MÁXIMO se P1 ∥ P2, "
               "no MÍNIMO se P1 ⊥ P2 — use isso para os ângulos H/V/D/A do laboratório")
    else:
        r.info("transmissão máxima = polarizador da montagem paralelo a P1")
    r.metric("phase_up", f_up.phase)
    r.metric("phase_down", f_down.phase)
    r.metric("backlash_deg", b.backlash)
    r.metric("backlash_per_cycle", backlashes)
    r.metric("visibility", cal.visibility)
    r.metric("residual", residual)
    r.metric("noise_deg", b.noise_deg)
    r.metric("max_at_deg", max_at)
    r.metric("min_at_deg", min_at)
    r.metric("work_deg", b.work)

    plt = pyplot()
    if plt:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), facecolor=SURFACE,
                                       gridspec_kw={"height_ratios": [2, 1]}, sharex=True)
        dense = np.linspace(grid[0], grid[-1], 1000)
        for c in range(cycles):
            ax1.plot(grid, all_up[c], "o", ms=3.5, color=UP_COLOR, label="subindo" if c == 0 else None)
            ax1.plot(grid, all_down[c], "o", ms=3.5, color=DOWN_COLOR, label="descendo" if c == 0 else None)
        ax1.plot(dense, f_up.value(dense), color=UP_COLOR, lw=1.2)
        ax1.plot(dense, f_down.value(dense), color=DOWN_COLOR, lw=1.2)
        styled_axes(ax1, f"Varredura {b.plate.axis} ({b.args.element}): folga {b.backlash:+.3f}°",
                    "", f"sinal ({b.det.unit})")
        ax1.legend(frameon=False, labelcolor=INK)
        if local:
            ax2.plot([a for a, _ in local], [v for _, v in local], "o", ms=3.5, color=INK)
            ax2.axhline(b.backlash, color=MUTED, lw=1, ls="--")
        styled_axes(ax2, "Folga ponto a ponto (onde a curva é inclinada)", "ângulo comandado (°)", "folga (°)")
        save_figure(plt, fig, b.out_dir / "sweep.png")


@register("approach", "mesmo ângulo chegando por baixo e por cima, com várias distâncias de aproximação")
def test_approach(b: Bench, r: Result):
    b.need_calib()
    target = b.work
    distances = b.args.distances
    reps = b.args.reps
    rows = []
    b.goto(target, +1)
    for d in distances:
        for rep in range(reps):
            for side in (+1, -1):
                b.plate.move_to(target - side * d)
                b.plate.move_to(target)
                angle, s = b.read_angle()
                rows.append((d, rep, side, s, angle))
            progress(f"d = {d:g}°: {rep + 1}/{reps}")
    progress_end()
    b.save_csv("approach", ["distance_deg", "rep", "side", "signal", "angle_deg"], rows)

    summary = []
    for d in distances:
        below = [a for dd, _, side, _, a in rows if dd == d and side > 0]
        above = [a for dd, _, side, _, a in rows if dd == d and side < 0]
        diff = statistics.fmean(above) - statistics.fmean(below)
        summary.append((d, diff, stdev(below), stdev(above)))
        r.info(f"d = {d:5g}°: por baixo {statistics.fmean(below) - target:+.4f}° (σ {stdev(below):.4f}), "
               f"por cima {statistics.fmean(above) - target:+.4f}° (σ {stdev(above):.4f}) "
               f"→ diferença {diff:+.4f}°")
    d_max, sat, s_below, s_above = summary[-1]
    knee = next((d for d, diff, _, _ in summary if diff >= 0.9 * sat), d_max)
    r.check(True, f"FOLGA (aproximação de {d_max:g}°): {sat:+.4f}° = {sat / b.micro:+.1f} micropassos; "
                  f"vencida depois de ≈ {knee:g}° de movimento")
    rep_sigma = max(s_below, s_above)
    text = f"repetibilidade chegando sempre pelo mesmo lado: σ ≤ {rep_sigma:.4f}° ({rep_sigma / b.micro:.2f} µp)"
    if rep_sigma > REPEAT_WARN_MICROSTEPS * b.micro:
        r.warn(text)
    else:
        r.check(True, text)
    if sat > b.threshold():
        r.info(f"para medidas: chegue aos ângulos sempre pelo mesmo lado, de ≥ {max(2 * knee, 1):g}° "
               f"de distância (ex.: move_to(alvo − {max(2 * knee, 1):g}) e depois move_to(alvo))")
    r.metric("summary", [{"distance": d, "difference": diff, "sigma_below": sb, "sigma_above": sa}
                         for d, diff, sb, sa in summary])
    r.metric("backlash_deg", sat)

    plt = pyplot()
    if plt:
        fig, ax = plt.subplots(figsize=(7, 4.2), facecolor=SURFACE)
        ds = [s[0] for s in summary]
        ax.errorbar(ds, [s[1] for s in summary], yerr=[math.hypot(s[2], s[3]) for s in summary],
                    fmt="o-", color=UP_COLOR, lw=2, ms=6, capsize=3)
        ax.set_xscale("log")
        styled_axes(ax, f"Chegar por cima − chegar por baixo ({b.plate.axis}, em {target:.2f}°)",
                    "distância de aproximação (°)", "diferença (°)")
        save_figure(plt, fig, b.out_dir / "approach.png")


@register("loop", "laço de histerese: vai e volta em passos pequenos em torno do ponto de trabalho")
def test_loop(b: Bench, r: Result):
    b.need_calib()
    center = b.work
    amplitude = b.args.amplitude
    up_grid = b.grid(center - amplitude, 2 * amplitude, b.args.loop_step)
    down_grid = up_grid[::-1]
    step = up_grid[1] - up_grid[0]
    rows = []
    b.goto(up_grid[0], +1)
    for c in range(b.args.loop_cycles):
        for direction, grid in (("up", up_grid), ("down", down_grid)):
            for i, angle in enumerate(grid):
                b.plate.move_to(angle)
                measured, s = b.read_angle(center)
                rows.append((c, direction, float(angle), s, measured))
                progress(f"ciclo {c + 1}/{b.args.loop_cycles} {direction} {i + 1}/{len(grid)}")
    progress_end()
    b.save_csv("loop", ["cycle", "direction", "commanded_deg", "signal", "measured_deg"], rows)

    def branch(direction):
        out: Dict[float, List[float]] = {}
        for _, dr, a, _, m in rows:
            if dr == direction:
                out.setdefault(round(a, 6), []).append(m)
        return {a: statistics.fmean(v) for a, v in out.items()}

    up, down = branch("up"), branch("down")
    central = [a for a in up if abs(a - center) <= amplitude / 2 and a in down]
    gap = statistics.fmean(down[a] - up[a] for a in central)
    r.check(True, f"FOLGA no laço (descendo − subindo, na metade central): {gap:+.4f}° = "
                  f"{gap / b.micro:+.1f} micropassos (passo do laço {step:.4f}°)")
    if 2 * amplitude < 1.5 * abs(gap):
        r.warn(f"a amplitude ±{amplitude:g}° é pequena para essa folga: use --amplitude {2 * abs(gap):.1f}")

    xs = np.array([a for _, dr, a, _, _ in rows if dr == "up" and abs(a - center) <= amplitude / 2])
    ys = np.array([m for _, dr, a, _, m in rows if dr == "up" and abs(a - center) <= amplitude / 2])
    if len(xs) >= 3:
        slope, intercept = np.polyfit(xs, ys, 1)
        resid = float(np.std(ys - (slope * xs + intercept)))
        text = (f"subindo, ângulo medido / comandado = {slope:.3f} (ideal 1.000); desvio da reta "
                f"{resid:.4f}° (irregularidade do micropasso + ruído)")
        if abs(slope - 1) > 0.1:
            r.warn(text + " — escala local diferente de 1")
        else:
            r.check(True, text)
        r.metric("scale", float(slope))
        r.metric("linearity_rms_deg", resid)
    r.metric("backlash_deg", gap)

    plt = pyplot()
    if plt:
        fig, ax = plt.subplots(figsize=(6.5, 6), facecolor=SURFACE)
        span = [-amplitude, amplitude]
        ax.plot(span, span, ls="--", color=MUTED, lw=1)
        for direction, color, label in (("up", UP_COLOR, "subindo"), ("down", DOWN_COLOR, "descendo")):
            for c in range(b.args.loop_cycles):
                pts = [(a - center, m - center) for cc, dr, a, _, m in rows if cc == c and dr == direction]
                ax.plot([p[0] for p in pts], [p[1] for p in pts], "-o", ms=3, lw=1.5, color=color,
                        label=label if c == 0 else None)
        styled_axes(ax, f"Laço de histerese ({b.plate.axis}): folga {gap:+.3f}°",
                    f"comandado − {center:.2f}° (°)", f"medido − {center:.2f}° (°)")
        ax.legend(frameon=False, labelcolor=INK)
        save_figure(plt, fig, b.out_dir / "loop.png")


@register("idle", "quanto a lâmina se mexe quando o driver desliga (e quando religa)")
def test_idle(b: Bench, r: Result):
    b.need_calib()
    phases = max(1, int(b.args.microstep))
    base = round(b.work * b.spd) / b.spd
    rows = []
    try:
        for j in range(phases):
            angle = base + j * b.micro
            b.goto(angle, +1)
            on, s_on = b.read_angle()
            b.driver_off()
            off, s_off = b.read_angle(settle=b.args.idle_wait)
            b.driver_on()
            back, s_back = b.read_angle()
            rows.append((j, angle, s_on, on, s_off, off, s_back, back))
            progress(f"fase do micropasso {j + 1}/{phases}")
        progress_end()

        b.driver_off()                           # todos os outros eixos querem desligado
        b.plate.disable_when_idle(True, 200)
        b.goto(base, +1)
        auto, _ = b.read_angle(settle=max(0.5, b.args.idle_wait))
    finally:
        b.plate.disable_when_idle(False)
        b.driver_on()
    b.goto(base, +1)
    hold, _ = b.read_angle()
    b.save_csv("idle", ["microstep_phase", "commanded_deg", "signal_on", "angle_on", "signal_off",
                        "angle_off", "signal_back", "angle_back"], rows)

    shifts = [row[5] - row[3] for row in rows]          # desligado − ligado
    returns = [row[7] - row[3] for row in rows]         # religado − ligado
    for row, sh, rt in zip(rows, shifts, returns):
        r.info(f"micropasso {row[0]} do passo inteiro: desligando {sh:+.4f}°, religando {rt:+.4f}°")
    worst = max(abs(x) for x in shifts)
    limit = b.threshold()
    text = (f"desligar o driver mexe a lâmina até {worst:.4f}° ({worst / b.micro:.1f} µp; "
            f"meio passo inteiro = {b.full_step / 2:.3f}°)")
    if worst > limit:
        r.warn(text + " → com disable_when_idle=True (padrão do EasyMotor em arduino_elliptec) a lâmina "
                      "não fica no ângulo pedido; para medir, use disable_when_idle=False")
    else:
        r.check(True, text)
    worst_back = max(abs(x) for x in returns)
    text = f"ao religar, volta a até {worst_back:.4f}° da posição original"
    if worst_back > limit:
        r.warn(text + " — não volta: a folga deixou a lâmina num lado diferente")
    else:
        r.check(True, text)
    r.info(f"com disable_when_idle(True, 200): {auto - hold:+.4f}° em relação ao driver sempre ligado")
    r.metric("max_shift_deg", worst)
    r.metric("max_return_deg", worst_back)
    r.metric("auto_disable_shift_deg", auto - hold)

    plt = pyplot()
    if plt:
        fig, ax = plt.subplots(figsize=(7, 4.2), facecolor=SURFACE)
        js = [row[0] for row in rows]
        ax.plot(js, shifts, "-o", color=UP_COLOR, lw=2, ms=6, label="driver desligado")
        ax.plot(js, returns, "-o", color=DOWN_COLOR, lw=2, ms=6, label="religado")
        ax.axhline(0, color=AXIS, lw=1)
        styled_axes(ax, f"Deslocamento ao desligar o driver ({b.plate.axis})",
                    "micropasso dentro do passo inteiro", "deslocamento (°)")
        ax.legend(frameon=False, labelcolor=INK)
        save_figure(plt, fig, b.out_dir / "idle.png")


@register("stability", "ruído e deriva parada: driver ligado × desligado, e ruído do próprio laser")
def test_stability(b: Bench, r: Result):
    b.need_calib()
    seconds = b.args.hold

    def record(label: str, duration: float, invert: bool, interval: float = 0.1) -> list:
        out = []
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < duration:
            time.sleep(max(0.0, t0 + len(out) * interval - time.perf_counter()))
            s, _ = b.measure(0.0)
            angle = b.calib.invert(s, b.work) if invert else float("nan")
            out.append((label, time.perf_counter() - t0, s, angle))
            progress(f"{label}: {time.perf_counter() - t0:.0f}/{duration:.0f} s")
        progress_end()
        return out

    b.goto(b.work, +1)
    time.sleep(b.args.settle)
    on = record("ligado", seconds, True)
    try:
        b.driver_off()
        time.sleep(b.args.idle_wait)
        off = record("desligado", seconds, True)
    finally:
        b.driver_on()
    b.goto(b.peak, +1)
    time.sleep(b.args.settle)
    peak = record("pico", seconds / 2, False)
    b.save_csv("stability", ["state", "time_s", "signal", "angle_deg"], on + off + peak)

    def stats(rows):
        t = np.array([x[1] for x in rows])
        a = np.array([x[3] for x in rows])
        rate = float(np.polyfit(t, a, 1)[0]) * 60 if len(rows) > 2 else 0.0
        return float(np.std(a)), rate

    s_on, rate_on = stats(on)
    s_off, rate_off = stats(off)
    sig = [x[2] for x in peak]
    rel = stdev(sig) / statistics.fmean(sig)
    laser_deg = rel * b.calib.offset / b.calib.max_slope
    r.info(f"driver ligado: σ {s_on:.4f}°, deriva {rate_on:+.4f}°/min ({len(on)} leituras)")
    r.info(f"driver desligado: σ {s_off:.4f}°, deriva {rate_off:+.4f}°/min")
    r.info(f"no pico (insensível ao ângulo) a potência flutua {rel:.3%} → equivale a {laser_deg:.4f}° "
           f"no ponto de trabalho: isso é o piso de ruído do laser/detector")
    if s_off > 2 * s_on + b.micro / 2:
        r.warn("com o driver desligado a lâmina fica mais instável (vibração, folga solta)")
    else:
        r.check(True, "driver desligado não piora a estabilidade parada")
    if abs(rate_on) * seconds / 60 > b.threshold():
        r.warn(f"deriva com o driver ligado ({rate_on:+.4f}°/min): aquecimento do motor ou do laser?")
    r.metric("sigma_on_deg", s_on)
    r.metric("sigma_off_deg", s_off)
    r.metric("drift_on_deg_min", rate_on)
    r.metric("drift_off_deg_min", rate_off)
    r.metric("laser_rel_noise", rel)

    plt = pyplot()
    if plt:
        fig, ax = plt.subplots(figsize=(8, 4.2), facecolor=SURFACE)
        for rows, color, label in ((on, UP_COLOR, "driver ligado"), (off, DOWN_COLOR, "driver desligado")):
            a = np.array([x[3] for x in rows])
            ax.plot([x[1] for x in rows], a - a.mean(), lw=2, color=color, label=label)
        styled_axes(ax, f"Lâmina parada no ponto de trabalho ({b.plate.axis})", "tempo (s)",
                    "ângulo − média (°)")
        ax.legend(frameon=False, labelcolor=INK)
        save_figure(plt, fig, b.out_dir / "stability.png")


@register("repeat", "repetibilidade: ir a um ângulo aleatório e voltar, sem e com referência (home) no meio")
def test_repeat(b: Bench, r: Result):
    b.need_calib()
    n = b.args.repeats
    rows = []
    results = {}
    for mode, label in (("move", "só posicionamento"), ("home", "com home() no meio")):
        values = []
        for i in range(n):
            b.plate.move_to(round(b.rng.uniform(b.work - 180, b.work + 180), 2))
            if mode == "home":
                b.plate.home()
            delta = b.read_offset(+1)
            values.append(delta)
            rows.append((mode, i, delta))
            progress(f"{label}: {i + 1}/{n}")
        progress_end()
        sigma = stdev(values)
        results[mode] = values
        text = (f"{label}: σ {sigma:.4f}° ({sigma / b.micro:.2f} µp), "
                f"faixa {max(values) - min(values):.4f}° em {n} repetições")
        if sigma > REPEAT_WARN_MICROSTEPS * b.micro:
            r.warn(text)
        else:
            r.check(True, text)
        r.metric(f"sigma_{mode}_deg", sigma)
    shift = statistics.fmean(results["home"]) - statistics.fmean(results["move"])
    r.info(f"diferença média com home − sem home: {shift:+.4f}° (zero do fim de curso × referencial atual)")
    r.metric("home_shift_deg", shift)
    b.save_csv("repeat", ["mode", "rep", "offset_deg"], rows)

    plt = pyplot()
    if plt:
        fig, ax = plt.subplots(figsize=(7, 4.2), facecolor=SURFACE)
        for mode, color, label in (("move", UP_COLOR, "só posicionamento"), ("home", DOWN_COLOR, "com home()")):
            ax.plot(range(1, n + 1), results[mode], "-o", color=color, lw=2, ms=6, label=label)
        styled_axes(ax, f"Repetibilidade ({b.plate.axis})", "repetição", "posição lida (°)")
        ax.legend(frameon=False, labelcolor=INK)
        save_figure(plt, fig, b.out_dir / "repeat.png")


@register("drift", "passos perdidos: blocos de movimentos aleatórios e volta ao ponto de trabalho")
def test_drift(b: Bench, r: Result):
    b.need_calib()
    blocks, moves = b.args.blocks, b.args.moves
    series = [(0, b.read_offset(+1))]
    for k in range(1, blocks + 1):
        for m in range(moves):
            b.plate.move_to(round(b.rng.uniform(b.work - 360, b.work + 360), 2))
            progress(f"bloco {k}/{blocks}: movimento {m + 1}/{moves}")
        series.append((k * moves, b.read_offset(+1)))
    progress_end()
    b.save_csv("drift", ["moves_done", "offset_deg"], series)

    base = series[0][1]
    limit = b.threshold()
    for done, value in series[1:]:
        r.info(f"depois de {done:3d} movimentos: {value - base:+.4f}° ({(value - base) / b.micro:+.1f} µp)")
    jumps = [(series[i][0], series[i][1] - series[i - 1][1]) for i in range(1, len(series))
             if abs(series[i][1] - series[i - 1][1]) > limit]
    total = series[-1][1] - base
    r.check(not jumps and abs(total) <= limit,
            f"depois de {blocks * moves} movimentos a lâmina voltou a {total:+.4f}° do início "
            f"(limite {limit:.4f}°)",
            "; ".join(f"salto de {j:+.4f}° ({j / b.full_step:+.1f} passos inteiros) até {d} movimentos"
                      for d, j in jumps))
    if any(abs(v - base) > b.period / 8 for _, v in series):
        r.info(f"deslocamentos acima de ~{b.period / 8:g}° saem da faixa linear da leitura: o valor "
               f"exato não vale, mas a lâmina com certeza perdeu passos")
    r.metric("total_deg", total)
    r.metric("series", series)

    plt = pyplot()
    if plt:
        fig, ax = plt.subplots(figsize=(7, 4.2), facecolor=SURFACE)
        ax.plot([d for d, _ in series], [v - base for _, v in series], "-o", color=UP_COLOR, lw=2, ms=6)
        ax.axhspan(-limit, limit, color=GRID, alpha=0.6, lw=0)
        styled_axes(ax, f"Posição no ponto de trabalho ao longo dos movimentos ({b.plate.axis})",
                    "movimentos aleatórios feitos", "deslocamento (°)")
        save_figure(plt, fig, b.out_dir / "drift.png")


@register("speed", "folga, repetibilidade e passos perdidos com perfis de velocidade diferentes")
def test_speed(b: Bench, r: Result):
    b.need_calib()
    d0 = b.defaults
    profiles = [
        ("lento", dict(min_speed=min(d0.min_speed, d0.max_speed / 4), max_speed=d0.max_speed / 4,
                       acceleration=d0.acceleration / 2)),
        ("padrão", dict(min_speed=d0.min_speed, max_speed=d0.max_speed, acceleration=d0.acceleration)),
        ("rápido", dict(min_speed=d0.min_speed, max_speed=d0.speed_limit, acceleration=d0.acceleration * 2)),
    ]
    distance = max(2.0, 3 * abs(b.backlash or 0.0))
    reps = max(2, b.args.reps)
    rows = []
    try:
        for label, kwargs in profiles:
            s = b.plate.configure(**kwargs)
            below, above = [], []
            b.goto(b.work, +1)
            for _ in range(reps):
                for side, out in ((+1, below), (-1, above)):
                    b.plate.move_to(b.work - side * distance)
                    b.plate.move_to(b.work)
                    out.append(b.read_angle()[0])
            before = b.read_offset(+1)
            for _ in range(10):
                b.plate.move_to(round(b.rng.uniform(b.work - 360, b.work + 360), 2))
            change = b.read_offset(+1) - before
            gap = statistics.fmean(above) - statistics.fmean(below)
            sigma = max(stdev(below), stdev(above))
            rows.append((label, s.max_speed, s.acceleration, gap, sigma, change))
            text = (f"{label} ({s.max_speed:.0f} °/s, {s.acceleration:.0f} °/s²): folga {gap:+.4f}°, "
                    f"σ {sigma:.4f}°, depois de 10 movimentos {change:+.4f}°")
            if abs(change) > b.threshold():
                r.check(False, text + " → perdeu passos nessa velocidade")
            else:
                r.check(True, text)
    finally:
        b.plate.restore_defaults()
    b.save_csv("speed", ["profile", "max_speed", "acceleration", "backlash_deg", "sigma_deg",
                         "change_after_moves_deg"], rows)
    r.metric("profiles", [dict(zip(("profile", "max_speed", "acceleration", "backlash", "sigma", "change"), row))
                          for row in rows])


@register("crosstalk", "a outra lâmina dando voltas mexe ou faz vibrar esta? (precisa de --other)")
def test_crosstalk(b: Bench, r: Result):
    if b.other is None:
        raise SkipTest("passe --other EIXO (a outra lâmina, fora do feixe ou no mesmo feixe)")
    b.need_calib()
    b.goto(b.work, +1)
    before = [b.read_angle(settle=0.05 if i else None)[0] for i in range(8)]
    start = b.other.get_position()
    b.other.move_to(start + 720, wait=False)
    during = []
    while b.other.is_moving():
        during.append(b.read_angle(settle=0.05)[0])
    b.other.wait_move()
    b.other.move_to(start)
    after = [b.read_angle(settle=0.05 if i else None)[0] for i in range(8)]
    b.save_csv("crosstalk", ["phase", "angle_deg"],
               [("before", a) for a in before] + [("during", a) for a in during] + [("after", a) for a in after])

    change = statistics.fmean(after) - statistics.fmean(before)
    s_before, s_during = stdev(before), stdev(during)
    r.check(abs(change) <= b.threshold(),
            f"{b.other.axis} dar 2 voltas e voltar mudou {b.plate.axis} em {change:+.4f}° "
            f"(limite {b.threshold():.4f}°)")
    if during:
        text = f"durante o movimento: σ {s_during:.4f}° (parada: {s_before:.4f}°, {len(during)} leituras)"
        if s_during > 2 * s_before + b.micro / 2:
            r.warn(text + " — vibra quando a outra anda")
        else:
            r.check(True, text)
    r.metric("change_deg", change)
    r.metric("sigma_before_deg", s_before)
    r.metric("sigma_during_deg", s_during)


# ══════════════════════════════════════════════════════════════════════════════
# Execução
# ══════════════════════════════════════════════════════════════════════════════
def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="Testes ópticos de histerese das lâminas motorizadas (polarizadores + detector).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Use --list para ver os testes e leia o cabeçalho do arquivo para a montagem.")
    g = ap.add_argument_group("lâminas")
    g.add_argument("--port", default=DEFAULT_PORT, help=f"porta padrão (padrão {DEFAULT_PORT})")
    g.add_argument("--plate", default=DEFAULT_PLATE, metavar="EIXO|PORTA:EIXO",
                   help="lâmina no feixe (padrão X)")
    g.add_argument("--other", metavar="EIXO|PORTA:EIXO", help="outra lâmina, para o teste crosstalk")
    g.add_argument("--no-home", action="store_true", help="não faz home() no começo")
    g.add_argument("--microstep", type=float, default=MICROSTEP, help=f"microstep do firmware (padrão {MICROSTEP})")

    g = ap.add_argument_group("óptica e detector")
    g.add_argument("--element", choices=sorted(HARMONIC), default="hwp", help="o que está na montagem (padrão hwp)")
    g.add_argument("--harmonic", type=int, help="força o n de cos(nθ) (normalmente vem de --element)")
    g.add_argument("--detector", choices=sorted(DETECTORS), default="powermeter")
    g.add_argument("--wavelength", type=float, default=780.0, help="nm, para o power meter (padrão 780)")
    g.add_argument("--lab-dir", default=str(LAB_DIR), help="pasta com EASYPM.py e TLPMX.py")
    g.add_argument("--channel", type=int, default=1, help="canal da TimeTagger (padrão 1)")
    g.add_argument("--gate", type=float, default=0.2, help="s de contagem por leitura na TimeTagger")
    g.add_argument("--samples", type=int, default=5, help="leituras promediadas por ponto (padrão 5)")
    g.add_argument("--settle", type=float, default=0.3, help="s de espera depois de parar (padrão 0.3)")

    g = ap.add_argument_group("parâmetros dos testes")
    g.add_argument("--start", type=float, default=0.0, help="início da varredura (°)")
    g.add_argument("--range", type=float, help="extensão da varredura (padrão: 2 períodos)")
    g.add_argument("--step", type=float, help="passo da varredura (padrão: período/45)")
    g.add_argument("--cycles", type=int, help="ciclos sobe/desce da varredura (padrão 2)")
    g.add_argument("--work", type=float, help="ângulo perto do qual escolher o ponto de trabalho")
    g.add_argument("--pre", type=float, default=3.0, help="° de pré-aproximação para vencer a folga (padrão 3)")
    g.add_argument("--distances", type=float, nargs="+", help="distâncias do teste approach (°)")
    g.add_argument("--reps", type=int, help="repetições por distância no approach (padrão 4)")
    g.add_argument("--amplitude", type=float, default=2.0, help="± graus do laço (padrão 2)")
    g.add_argument("--loop-step", type=float, default=0.09, help="passo do laço em ° (padrão 0.09 = 2 µp)")
    g.add_argument("--loop-cycles", type=int, help="voltas do laço (padrão 2)")
    g.add_argument("--repeats", type=int, help="repetições do teste repeat (padrão 8)")
    g.add_argument("--blocks", type=int, help="blocos do teste drift (padrão 4)")
    g.add_argument("--moves", type=int, help="movimentos por bloco no drift (padrão 15)")
    g.add_argument("--hold", type=float, help="s parada em cada estado no stability (padrão 30)")
    g.add_argument("--idle-wait", type=float, default=0.5, help="s depois de desligar o driver (padrão 0.5)")
    g.add_argument("--seed", type=int, help="semente dos movimentos aleatórios")

    g = ap.add_argument_group("seleção")
    g.add_argument("--only", nargs="+", metavar="TESTE")
    g.add_argument("--skip", nargs="+", default=[], metavar="TESTE")
    g.add_argument("--quick", action="store_true", help="menos repetições (~metade do tempo)")
    g.add_argument("--list", action="store_true")
    args = ap.parse_args(argv)

    q = args.quick
    defaults = dict(cycles=(1, 2), reps=(2, 4), loop_cycles=(1, 2), repeats=(4, 8), blocks=(2, 4),
                    moves=(10, 15), hold=(10.0, 30.0))
    for key, (quick_value, normal_value) in defaults.items():
        if getattr(args, key) is None:
            setattr(args, key, quick_value if q else normal_value)
    if args.distances is None:
        args.distances = [0.1, 0.5, 2, 5] if q else [0.05, 0.1, 0.25, 0.5, 1, 2, 5]
    args.distances = sorted(args.distances)
    if args.seed is None:
        args.seed = random.randrange(100000)
    return args


def select_tests(args) -> List[TestSpec]:
    names = {t.name for t in TESTS}
    unknown = [n for n in (args.only or []) + args.skip if n not in names]
    if unknown:
        raise SystemExit(f"teste(s) desconhecido(s): {', '.join(unknown)} (veja --list)")
    return [t for t in TESTS if (not args.only or t.name in args.only) and t.name not in args.skip]


def run_one(bench: Bench, spec: TestSpec, index: int, total: int, results: List[Result]):
    r = Result(spec.name)
    results.append(r)
    print(f"\n[{index}/{total}] {spec.name} — {spec.description}", flush=True)
    t0 = time.perf_counter()
    try:
        spec.func(bench, r)
    except SkipTest as e:
        r.status, r.headline = "SKIP", str(e)
        r._log("-", f"pulado: {e}")
    except KeyboardInterrupt:
        progress_end()
        r.error("interrompido pelo usuário")
        raise
    except Exception as e:  # noqa: BLE001
        progress_end()
        r.error(f"{type(e).__name__}: {e}")
        r.lines.extend(traceback.format_exc().splitlines())
        try:
            if bench.plate.is_moving():
                bench.plate.stop()
            bench.plate.restore_defaults()
        except Exception:  # noqa: BLE001
            pass
    finally:
        r.duration = time.perf_counter() - t0
        print(f"    => {r.status} ({r.duration:.0f} s)", flush=True)


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass
    args = parse_args(argv)
    if args.list:
        for t in TESTS:
            print(f"  {t.name:<10} {t.description}")
        return 0
    selected = select_tests(args)

    port, axis = parse_plate(args.plate, args.port)
    out_dir = RESULTS_DIR / f"hysteresis_{axis}_{datetime.now():%Y%m%d_%H%M%S}"
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"Lâmina {port}:{axis} com {args.element}, detector {args.detector} — "
          f"{len(selected)} testes, semente {args.seed}")
    print(f"Resultados em {out_dir}")

    try:
        detector = DETECTORS[args.detector](args)
    except Exception as e:  # noqa: BLE001
        print(f"ERRO ao abrir o detector '{args.detector}': {type(e).__name__}: {e}")
        return 2
    plate = other = None
    try:
        plate = WavePlate(port, axis, disable_when_idle=False)
        if args.other:
            o_port, o_axis = parse_plate(args.other, args.port)
            other = WavePlate(o_port, o_axis, disable_when_idle=False)
    except (serial.SerialException, TimeoutError, WavePlateError, ValueError) as e:
        print(f"ERRO ao conectar: {e}")
        for p in (plate, other):
            if p is not None:
                p.close()
        detector.close()
        return 2

    bench = Bench(args, plate, detector, other, out_dir)
    results: List[Result] = []
    try:
        value = detector.read()
        print(f"Leitura de teste do detector: {value:.4g} {detector.unit}")
        if not args.no_home:
            print("Referenciando (home)...", flush=True)
            plate.home()
            if other is not None:
                other.home()
        for i, spec in enumerate(selected, 1):
            run_one(bench, spec, i, len(selected), results)
    except KeyboardInterrupt:
        print("\nInterrompido pelo usuário.")
        try:
            if plate.is_moving():
                plate.stop()
        except Exception:  # noqa: BLE001
            pass
    finally:
        bench.close()
        for p in (plate, other):
            if p is not None:
                try:
                    p.close()
                except Exception:  # noqa: BLE001
                    pass
        detector.close()

        summary = {
            "date": datetime.now().isoformat(timespec="seconds"),
            "plate": f"{port}:{axis}",
            "args": vars(args),
            "calibration": asdict(bench.calib) if bench.calib else None,
            "work_deg": bench.work,
            "backlash_deg": bench.backlash,
            "noise_deg": bench.noise_deg,
            "microstep_deg": bench.micro,
            "tests": [{"name": r.name, "status": r.status, "duration_s": round(r.duration, 1),
                       "metrics": r.metrics, "log": r.lines} for r in results],
        }
        path = out_dir / "summary.json"
        path.write_text(json.dumps(to_json(summary), indent=2, ensure_ascii=False, default=str),
                        encoding="utf-8")

        print("\n" + "=" * 78)
        print("RESUMO")
        if bench.calib:
            print(f"  folga {bench.backlash:+.4f}° · contraste {bench.calib.visibility:.3f} · "
                  f"ruído {bench.noise_deg or 0:.4f}° · micropasso {bench.micro:.4f}°")
        for r in results:
            extra = ""
            if r.status != "PASS" and r.headline:
                extra = "  " + (r.headline if len(r.headline) <= 90 else r.headline[:87] + "...")
            print(f"  {r.status:<5} {r.name:<10} {r.duration:6.0f} s{extra}")
        print(f"Arquivos: {out_dir}")
        if pyplot() is None:
            print("(instale o matplotlib para gerar os gráficos PNG: pip install matplotlib)")
    return 1 if any(r.status in ("FAIL", "ERROR") for r in results) else 0


if __name__ == "__main__":
    sys.exit(main())
