"""
Testes das funções novas da biblioteca: chegada pelo mesmo sentido
(set_approach), zero de software (set_zero, zero_offset) e recalibração
(calibrate), na WavePlate e no EasyMotor.

Usa a mesma infraestrutura e as mesmas opções de hardware_test.py; o padrão
aqui é só a lâmina X (passe --plates X Y para incluir a Y).

Testes (veja --list):
  aproximação       approach-config   set_approach/get_approach: valores, construtor, em movimento
                    approach-legs     quais movimentos passam pela pré-aproximação (comandos G)
                    approach-wait     wait=False, is_moving(), stop() no meio da pré-aproximação
                    approach-history  borda do fim de curso depois de movimentos longos em + e em −,
                                      sem e com aproximação: mostra qual overshoot resolve [demorado]
  zero e calibrate  zero              set_zero() acumula, zero_offset, None antes do primeiro home()
                    calibrate         refaz o home e volta ao ângulo pedido sem mudar o zero de software
                    calibrate-extra   sem set_zero, depois de stop(), e com a aproximação ligada
  EasyMotor         easymotor         approach/overshoot no construtor, set_approach, set_zero e
                                      calibrate com vários endereços

Uso:
    python features_test.py                          # X na COM4
    python features_test.py --quick                  # approach-history mais curto
    python features_test.py --only approach-history --overshoots 3 10 30 60
    python features_test.py --plates X Y             # inclui a Y no EasyMotor

Opções extras deste arquivo:
    --overshoots GRAUS ...   overshoots testados no approach-history (padrão 3 10 30)
    --history-reps N         repetições de cada histórico no approach-history (padrão 2)
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from typing import Dict, List

import hardware_test as base
from arduino_elliptec import ArduinoElliptec
from hardware_test import (EDGE_TOL_MICROSTEPS, PRINT_TOL, SWITCH_ANGLE, Bench, Result, expect_error,
                           raw, snap, stop_quietly)
from waveplate import DEFAULT_OVERSHOOT, WavePlate, WavePlateError

base.TESTS.clear()                      # só os testes deste arquivo
base.DEFAULT_PLATES = ("X",)
register = base.register


def approach_off(p: WavePlate):
    """Desliga a aproximação, parando a lâmina antes se preciso."""
    stop_quietly([p])
    try:
        p.wait_move()
    except (WavePlateError, TimeoutError):
        p.stop()
    p.set_approach(0)


class CommandLog:
    """Grava os comandos que um WavePlate manda (para ver os G de cada movimento)."""

    def __init__(self, plate: WavePlate):
        self.plate = plate
        self.sent: List[str] = []
        original = plate._send

        def recording_send(command: str):
            self.sent.append(command)
            original(command)

        plate._send = recording_send

    def moves(self) -> List[float]:
        out = [float(c.split()[1]) for c in self.sent if c.startswith("G")]
        self.sent.clear()
        return out

    def close(self):
        del self.plate._send                    # volta ao método da classe


# ══════════════════════════════════════════════════════════════════════════════
# Aproximação pelo mesmo sentido
# ══════════════════════════════════════════════════════════════════════════════
@register("approach-config", "aproximação", "set_approach/get_approach: valores, construtor, em movimento")
def test_approach_config(b: Bench, r: Result):
    b.ensure_homed()
    for name, p in b.items():
        try:
            r.check(p.get_approach() == (0, DEFAULT_OVERSHOOT),
                    f"{name}: desligada por padrão: get_approach() = {p.get_approach()}")
            p.set_approach(-1, 4.5)
            r.check(p.get_approach() == (-1, 4.5), f"{name}: set_approach(-1, 4.5) → {p.get_approach()}")
            bad = [f"{args}: {msg}" for args in ((2, 10), (1, 0), (1, -5))
                   if (msg := expect_error(lambda: p.set_approach(*args), ValueError))]
            r.check(not bad, f"{name}: set_approach recusa sentido 2 e overshoot ≤ 0 com ValueError",
                    "; ".join(bad))
            p.set_approach(0, 0)
            r.check(p.get_approach()[0] == 0, f"{name}: set_approach(0, ...) desliga (overshoot não importa)")

            other = WavePlate(p.port, p.axis, approach=+1, overshoot=7)
            got = other.get_approach()
            other.close(disable=False)
            r.check(got == (1, 7.0), f"{name}: WavePlate(..., approach=+1, overshoot=7) → {got}")
            problem = expect_error(lambda: WavePlate(p.port, p.axis, approach=3), ValueError)
            r.check(not problem, f"{name}: construtor com approach=3 dá ValueError (e não prende a porta)",
                    problem)

            p.move_to(0)
            p.move_to(360, wait=False)
            problem = expect_error(lambda: p.set_approach(+1, 10), WavePlateError)
            p.wait_move()
            r.check(not problem, f"{name}: set_approach() em movimento é recusado", problem)
        finally:
            approach_off(p)
        p.move_to(0)


@register("approach-legs", "aproximação", "quais movimentos passam pela pré-aproximação (comandos G enviados)")
def test_approach_legs(b: Bench, r: Result):
    b.ensure_homed()
    L = 10.0
    for name, p in b.items():
        spd = b.spd(name)
        log = CommandLog(p)
        try:
            def run(direction: int, steps):
                p.home()                               # começa sem saber de onde veio
                p.set_approach(direction, L)
                log.moves()
                for label, action, target, expected in steps:
                    final = action()
                    sent = log.moves()
                    ok = (len(sent) == len(expected) and all(abs(s - e) < 1e-3 for s, e in zip(sent, expected))
                          and abs(final - snap(target, spd)) <= PRINT_TOL)
                    r.check(ok, f"{name}: sentido {direction:+d}, {label}: G {sent} → {final:.2f}°",
                            f"esperado G {expected} → {snap(target, spd):.2f}°")

            run(+1, [
                ("0 → 30 (subindo, ≥ overshoot)", lambda: p.move_to(30), 30, [30]),
                ("30 → 32 (subindo curto, já chegou certo)", lambda: p.move_to(32), 32, [32]),
                ("32 → 20 (descendo)", lambda: p.move_to(20), 20, [10, 20]),
                ("20 → 25 (subindo curto depois da pré-aproximação)", lambda: p.move_to(25), 25, [25]),
                ("move_by(-0.5)", lambda: p.move_by(-0.5), 24.5, [14.5, 24.5]),
                ("mesmo alvo", lambda: p.move_to(24.5), 24.5, [24.5]),
            ])
            run(-1, [
                ("0 → -30 (descendo, ≥ overshoot)", lambda: p.move_to(-30), -30, [-30]),
                ("-30 → -20 (subindo)", lambda: p.move_to(-20), -20, [-10, -20]),
                ("-20 → -25 (descendo curto)", lambda: p.move_to(-25), -25, [-25]),
            ])
            run(+1, [
                ("logo depois do home(), 0 → 5 (curto)", lambda: p.move_to(5), 5, [-5, 5]),
            ])

            # perto do limite não há espaço para a pré-aproximação: vai direto (só o planejamento)
            s = p.get_settings()
            saved = p._target
            p._target = s.min_angle + 50
            legs = p._plan_legs(s.min_angle + 3)
            p._target = saved
            r.check(legs == [s.min_angle + 3],
                    f"{name}: alvo a 3° do limite mínimo vai direto (pré-aproximação cairia fora): {legs}")
        finally:
            log.close()
            approach_off(p)
        p.home()


@register("approach-wait", "aproximação", "wait=False, is_moving(), stop() no meio da pré-aproximação")
def test_approach_wait(b: Bench, r: Result):
    b.ensure_homed()
    L = 10.0
    for name, p in b.items():
        tol = b.tol(name)
        try:
            p.set_approach(+1, L)
            p.move_to(100)
            ret = p.move_to(50, wait=False)
            r.check(abs(ret - 50) <= PRINT_TOL,
                    f"{name}: move_to(50, wait=False) devolve o alvo final na hora ({ret:.2f}°), "
                    f"não a pré-aproximação")
            problem = expect_error(lambda: p.move_to(60), WavePlateError)
            r.check(not problem, f"{name}: outro move_to() durante a pré-aproximação é recusado", problem)
            positions = []
            t0 = time.perf_counter()
            while p.is_moving() and time.perf_counter() - t0 < 30:
                positions.append(p.get_position())
                time.sleep(0.02)
            final = p.wait_move()
            lowest = min(positions) if positions else float("nan")
            # as leituras são espaçadas: basta ver que passou bem abaixo do alvo
            r.check(lowest <= 50 - L / 2,
                    f"{name}: is_moving() fica True nos dois trechos (leu até {lowest:.2f}°, a caminho "
                    f"da pré-aproximação em {50 - L:g}°)")
            r.check(abs(final - 50) <= tol and abs(p.get_position() - 50) <= tol,
                    f"{name}: termina em {final:.2f}° e get_position() confirma")

            p.move_to(150)
            p.move_to(20, wait=False)                 # pré-aproximação em 10°
            time.sleep(0.2)
            stopped = p.stop()
            time.sleep(0.5)
            moving = p.is_moving()
            later = p.get_position()
            r.check(10 + 1 < stopped < 150 and not moving and abs(later - stopped) <= PRINT_TOL,
                    f"{name}: stop() no primeiro trecho para em {stopped:.2f}° e o trecho final não é "
                    f"mandado depois (0,5 s depois: {later:.2f}°, is_moving={moving})")
            final = p.move_to(20)
            r.check(abs(final - snap(20, b.spd(name))) <= PRINT_TOL,
                    f"{name}: depois do stop(), move_to(20) funciona ({final:.2f}°)")
        finally:
            approach_off(p)
        p.move_to(0)


@register("approach-history", "aproximação", "borda do fim de curso depois de movimentos longos em + e em −, "
                                             "sem e com aproximação (qual overshoot resolve)", slow=True)
def test_approach_history(b: Bench, r: Result):
    reps = b.args.history_reps or (1 if b.args.quick else 2)
    overshoots = b.args.overshoots or ([DEFAULT_OVERSHOOT] if b.args.quick else [3.0, 10.0, 30.0])
    far = 400.0
    b.ensure_homed()
    for name, p in b.items():
        micro = b.micro(name)
        limit = EDGE_TOL_MICROSTEPS * micro + PRINT_TOL
        results: Dict[str, Dict[int, List[float]]] = {}
        try:
            for overshoot in [0.0] + overshoots:
                label = "sem aproximação" if overshoot == 0 else f"overshoot {overshoot:g}°"
                p.set_approach(b.home_dir if overshoot else 0, overshoot or DEFAULT_OVERSHOOT)
                edges: Dict[int, List[float]] = {+1: [], -1: []}
                for _ in range(reps):
                    for history in (+1, -1):
                        # o último movimento longo antes de medir a borda é no sentido `history`
                        p.move_to(-history * far)
                        e = b.edge(name)
                        if e is None:
                            r.check(False, f"{name}: {label}: borda não encontrada perto de 0° "
                                           f"(histórico {history:+d})")
                            return
                        edges[history].append(e)
                results[label] = edges
                every = edges[+1] + edges[-1]
                spread = (max(every) - min(every)) / micro
                shift = (statistics.mean(edges[+1]) - statistics.mean(edges[-1])) / micro
                r.info(f"{name}: {label}: borda depois de vir de baixo − depois de vir de cima = "
                       f"{shift:+.1f} µp; espalhamento {spread:.1f} µp ({len(every)} medidas)")
                r.metric(f"{name}.{label}", {"after_up": edges[+1], "after_down": edges[-1],
                                             "shift_microsteps": round(shift, 2),
                                             "spread_microsteps": round(spread, 2)})
        finally:
            approach_off(p)
            p.move_to(SWITCH_ANGLE)

        def spread_of(label):
            every = results[label][+1] + results[label][-1]
            return max(every) - min(every)

        off = spread_of("sem aproximação")
        good = [o for o in overshoots if spread_of(f"overshoot {o:g}°") <= limit]
        if off <= limit:
            r.check(True, f"{name}: sem aproximação a borda já varia só {off / micro:.1f} µp com o histórico: "
                          f"nada a corrigir aqui")
        elif good:
            r.check(True, f"{name}: sem aproximação varia {off / micro:.1f} µp; com overshoot ≥ {min(good):g}° "
                          f"fica dentro de {EDGE_TOL_MICROSTEPS} µp → use set_approach({b.home_dir:+d}, "
                          f"{min(good):g}) ou maior")
        else:
            best = min(overshoots, key=lambda o: spread_of(f"overshoot {o:g}°"))
            best_spread = spread_of(f"overshoot {best:g}°")
            if best_spread < off * 0.7:
                r.warn(f"{name}: a aproximação ajuda ({off / micro:.1f} → {best_spread / micro:.1f} µp com "
                       f"{best:g}°), mas não chega a {EDGE_TOL_MICROSTEPS} µp: teste overshoots maiores "
                       f"(--overshoots 30 60 90)")
            else:
                r.check(False, f"{name}: a aproximação não muda a variação ({off / micro:.1f} µp sem, "
                               f"{best_spread / micro:.1f} µp no melhor caso): o efeito não é folga da "
                               f"transmissão; suspeite do fim de curso ou de passos perdidos")


# ══════════════════════════════════════════════════════════════════════════════
# Zero de software e calibrate
# ══════════════════════════════════════════════════════════════════════════════
@register("zero", "zero e calibrate", "set_zero() acumula, zero_offset, None antes do primeiro home()")
def test_zero(b: Bench, r: Result):
    b.ensure_homed()
    for name, p in b.items():
        spd, micro = b.spd(name), b.micro(name)
        try:
            p.home()
            r.check(p.zero_offset == 0, f"{name}: depois do home(), zero_offset = {p.zero_offset}")
            p.move_by(50)
            p.set_zero()
            pos = p.get_position()
            r.check(abs(p.zero_offset - snap(50, spd)) <= PRINT_TOL and abs(pos) <= PRINT_TOL,
                    f"{name}: set_zero() em 50° → zero_offset {p.zero_offset:.3f}°, posição {pos:.2f}°")
            p.move_by(30)
            p.set_zero()
            r.check(abs(p.zero_offset - snap(80, spd)) <= PRINT_TOL,
                    f"{name}: mais 30° e set_zero() de novo → zero_offset {p.zero_offset:.3f}° (acumula)")
            e = b.edge(name, expected=-p.zero_offset)
            r.check(e is not None and abs(e + p.zero_offset) <= EDGE_TOL_MICROSTEPS * micro + PRINT_TOL,
                    f"{name}: o fim de curso aciona em {'?' if e is None else f'{e:.3f}'}° "
                    f"(esperado {-p.zero_offset:.3f}°)")
            p.home()
            r.check(p.zero_offset == 0 and abs(p.get_position()) <= PRINT_TOL,
                    f"{name}: home() volta o zero de software ao fim de curso (zero_offset 0)")

            saved = p._serial.zero_steps[p.axis]
            p._serial.zero_steps[p.axis] = None         # como logo depois de conectar
            try:
                r.check(p.zero_offset is None, f"{name}: antes do primeiro home(), zero_offset = None")
                p.move_by(10)
                p.set_zero()
                r.check(p.zero_offset is None, f"{name}: set_zero() sem home() continua None (zero mecânico "
                                               f"desconhecido)")
            finally:
                p._serial.zero_steps[p.axis] = saved
        finally:
            p.home()


register("calibrate", "zero e calibrate",
         "refaz o home e volta ao ângulo pedido sem mudar o zero de software")(base.test_calibrate)


@register("calibrate-extra", "zero e calibrate", "calibrate sem set_zero, depois de stop(), e com aproximação")
def test_calibrate_extra(b: Bench, r: Result):
    b.ensure_homed()
    for name, p in b.items():
        spd, micro = b.spd(name), b.micro(name)
        try:
            p.home()
            p.move_to(33)
            final = p.calibrate()
            r.check(abs(final - snap(33, spd)) <= PRINT_TOL and p.zero_offset == 0,
                    f"{name}: sem set_zero(): calibrate() = home() + volta a {final:.2f}° (zero_offset 0)")

            p.move_to(0)
            p.move_to(300, wait=False)
            time.sleep(0.4)
            stopped = p.stop()
            final = p.calibrate()
            r.check(abs(final - stopped) <= 0.5 * micro + PRINT_TOL,
                    f"{name}: depois de stop() em {stopped:.2f}°, calibrate() volta a {final:.2f}° "
                    f"(sem ângulo pedido, usa a posição)")

            p.home()
            p.set_approach(b.home_dir, 10)
            log = CommandLog(p)
            try:
                p.move_to(50)
                p.set_zero()
                p.move_to(-20)
                raw(p, "Z", "POS")                  # firmware passa a contar errado
                log.moves()
                final = p.calibrate()
                sent = log.moves()
            finally:
                log.close()
            r.check(abs(final - snap(-20, spd)) <= PRINT_TOL,
                    f"{name}: com aproximação {b.home_dir:+d}/10°, calibrate() volta a {final:.2f}° "
                    f"(G enviados: {sent})")
            last_two = sent[-2:]
            r.check(len(sent) >= 2 and b.home_dir * (last_two[1] - last_two[0]) >= 10 - PRINT_TOL,
                    f"{name}: o trecho final do calibrate() chega no sentido {b.home_dir:+d} com ≥ 10°")
            e = b.edge(name, expected=-snap(50, spd))
            r.check(e is not None and abs(e + snap(50, spd)) <= EDGE_TOL_MICROSTEPS * micro + PRINT_TOL,
                    f"{name}: zero de software continua a 50° do fim de curso "
                    f"(borda em {'?' if e is None else f'{e:.3f}'}°)")
        finally:
            approach_off(p)
            p.home()


# ══════════════════════════════════════════════════════════════════════════════
# EasyMotor
# ══════════════════════════════════════════════════════════════════════════════
@register("easymotor", "EasyMotor", "approach/overshoot no construtor, set_approach, set_zero e calibrate "
                                    "com vários endereços")
def test_easymotor(b: Bench, r: Result):
    items = b.items()
    if len({p.port for _, p in items}) > 1:
        raise base.SkipTest("lâminas em portas diferentes; o EasyMotor usa uma porta só")
    b.ensure_homed()
    port = items[0][1].port
    mapping = {addr: p.axis for addr, (_, p) in zip([0, 2, 1, 3], items)}
    addrs = list(mapping)
    tol = max(b.tol(n) for n, _ in items)
    stage = ArduinoElliptec(port, axes=mapping, approach=+1, overshoot=10, disable_when_idle=False)
    try:
        got = [stage.plate(a).get_approach() for a in addrs]
        r.check(all(g == (1, 10.0) for g in got), f"EasyMotor(..., approach=+1, overshoot=10) → {got}")
        stage.home(addr=addrs)
        stage.move_to(40, addr=addrs[0])
        log = CommandLog(stage.plate(addrs[0]))
        try:
            t0 = time.perf_counter()
            v = stage.move_to(20, addr=addrs)
            dt = time.perf_counter() - t0
            sent = log.moves()
        finally:
            log.close()
        r.check(all(abs(x - 20) <= tol for x in v) and sent == [10.0, 20.0],
                f"move_to(20, addr={addrs}) → {v} em {dt:.1f} s; endereço {addrs[0]} (vindo de 40°) "
                f"passou pela pré-aproximação: G {sent}")
        stage.set_approach(-1, 5)
        got = [stage.plate(a).get_approach() for a in addrs]
        r.check(all(g == (-1, 5.0) for g in got), f"set_approach(-1, 5) vale para todos: {got}")
        stage.set_approach(0)

        stage.move_to(30, addr=addrs)
        stage.set_zero(addr="all")
        stage.move_by(15, addr="all")
        v = stage.calibrate(addr=addrs)
        r.check(isinstance(v, list) and all(abs(x - 15) <= tol for x in v),
                f"set_zero(addr='all') em 30° + calibrate(addr={addrs}) → {v}")
        offsets = [stage.plate(a).zero_offset for a in addrs]
        r.check(all(o is not None and abs(o - 30) <= tol for o in offsets),
                f"zero_offset de cada endereço continua ≈ 30°: {offsets}")
        v = stage.calibrate(addr=addrs[0])
        r.check(isinstance(v, float) and abs(v - 15) <= tol, f"calibrate(addr={addrs[0]}) → {v}")
    finally:
        stage.close(disable=False)
    for _, p in items:
        p.home()                                         # 0 do firmware de volta ao fim de curso
    b.all_to(0)


# ══════════════════════════════════════════════════════════════════════════════
# Execução
# ══════════════════════════════════════════════════════════════════════════════
_base_parse_args = base.parse_args


def parse_args(argv=None):
    extra = argparse.ArgumentParser(add_help=False)
    extra.add_argument("--overshoots", type=float, nargs="+")
    extra.add_argument("--history-reps", type=int)
    known, rest = extra.parse_known_args(sys.argv[1:] if argv is None else argv)
    args = _base_parse_args(rest)
    args.overshoots = known.overshoots
    args.history_reps = known.history_reps
    return args


base.parse_args = parse_args

if __name__ == "__main__":
    sys.exit(base.main())
