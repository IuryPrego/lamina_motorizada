"""
Teste de bancada para motores de passo SOLTOS (sem fim de curso e sem lâmina).

Serve para descobrir, passo a passo, por que um motor não funciona:

    1. porta serial     o Arduino aparece, a porta está livre, o firmware responde?
    2. eixos            o firmware tem os eixos pedidos? configuração de cada um
    3. torque           com o driver ligado, o eixo do motor fica duro?
    4. uma volta        devagar: gira exatamente 1 volta? só vibra? nada?
    5. marca            várias voltas para frente e para trás: volta à mesma posição?
    6. velocidade       sobe a velocidade até o motor começar a perder passos
    7. juntos           todos os motores ao mesmo tempo

As etapas 3 a 6 são feitas em um motor por vez. Você olha o motor e responde
às perguntas; no fim sai um resumo com a causa provável de cada problema.
Nenhum comando usa o fim de curso: não há home(), só movimentos relativos a
onde o motor estava ao ligar.

MONTAGEM
    * Fonte de 12–24 V no borne azul do CNC Shield (o USB NÃO alimenta os
      motores: sem a fonte, o Arduino responde normalmente e nada gira).
    * Drivers (A4988/DRV8825) encaixados na posição certa: o pino EN do driver
      no canto marcado EN do shield. Ao contrário, o driver queima.
    * Nunca ligue/desligue o motor com a fonte ligada.
    * Cole um pedaço de fita no eixo de cada motor, como uma bandeirinha: é
      por ela que você vê quanto o motor girou.

Uso:
    python tests/motor_test.py                       # COM4, motores X Y Z, com perguntas
    python tests/motor_test.py --port COM5
    python tests/motor_test.py --axes X Y            # só alguns eixos
    python tests/motor_test.py --no-ask              # só gira, sem perguntas (olhe os motores)
    python tests/motor_test.py --list-ports          # mostra as portas seriais

Se o motor girar 8 voltas quando pedido 1 (ou 1/2, 2...), os jumpers de
microstep embaixo do driver não batem com o firmware: veja --microstep e a
tabela em firmware/src/main.cpp.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import serial
import serial.tools.list_ports

from waveplate import BAUD, WavePlate, WavePlateError

DEFAULT_PORT = "COM4"
DEFAULT_AXES = ("X", "Y", "Z")
FULL_STEPS_PER_REV = 200      # motor de 1,8° por passo
MICROSTEP = 8                 # EIXOS_CFG em firmware/src/main.cpp (jumpers MS1 e MS2 ligados)

SLOW_RPS = 0.5                # voltas do motor por segundo nos testes devagar
SLOW_ACCEL = 2.0              # voltas/s²
LADDER_RPS = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)
LADDER_ACCEL = 4.0
MARK_TURNS = 5                # voltas no teste da marca

HINT_NO_POWER = ("sem torque: confira a fonte de 12–24 V no borne do shield (o USB não alimenta os "
                 "motores), se o driver está encaixado com o EN no canto EN, se o motor está no "
                 "conector deste eixo e a corrente (Vref) do driver, que pode estar quase zero")
HINT_VIBRATES = ("vibra sem girar: as duas bobinas estão misturadas ou um fio está solto. Com o "
                 "multímetro, ache os pares de fios com continuidade (cada par é uma bobina, poucos "
                 "ohms) e ligue um par em 1A/1B e o outro em 2A/2B. Corrente (Vref) muito baixa "
                 "também faz isso")
HINT_NOTHING = ("não se mexe: se o eixo também não fica duro, é alimentação/driver (fonte, driver "
                "mal encaixado ou queimado). Troque o driver com o de um eixo que funciona: se o "
                "problema for junto com o driver, é o driver; se ficar no eixo, é o soquete/conector "
                "ou o motor")
HINT_LOSES_STEPS = ("perde passos: aumente um pouco a corrente (Vref do driver) ou use velocidades "
                    "menores; motor solto às vezes ressoa numa faixa de velocidade, então teste "
                    "velocidades vizinhas")


@dataclass
class AxisReport:
    axis: str
    ok: List[str] = field(default_factory=list)
    problems: List[str] = field(default_factory=list)
    max_rps: Optional[float] = None


# ══════════════════════════════════════════════════════════════════════════════
# Perguntas
# ══════════════════════════════════════════════════════════════════════════════
class Asker:
    def __init__(self, enabled: bool):
        self.enabled = enabled

    def text(self, question: str) -> str:
        if not self.enabled:
            print(f"    ({question.split('[')[0].strip()} — sem perguntas: olhe o motor)")
            return ""
        try:
            return input(f"    ? {question} ").strip().lower()
        except EOFError:
            self.enabled = False
            return ""

    def yes(self, question: str) -> Optional[bool]:
        """True / False, ou None se não houve resposta."""
        answer = self.text(question + " [s/n]")
        if answer in ("s", "sim", "y", "yes"):
            return True
        if answer in ("n", "nao", "não", "no"):
            return False
        return None

    def pause(self, message: str):
        if self.enabled:
            self.text(message + " (Enter)")


# ══════════════════════════════════════════════════════════════════════════════
# 1. Porta serial
# ══════════════════════════════════════════════════════════════════════════════
def print_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("  nenhuma porta serial encontrada (Arduino desconectado ou cabo só de carga?)")
    for p in ports:
        print(f"  {p.device:<8} {p.description}")


def check_port(port: str) -> Optional[tuple]:
    """Abre a porta direto e espera o READY. Retorna os eixos do firmware ou None."""
    print(f"\n[1] Porta {port}")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
    except serial.SerialException as e:
        text = str(e)
        if "denied" in text.lower() or "negado" in text.lower() or "PermissionError" in text:
            print("    ERRO  porta ocupada: feche o Serial Monitor (PlatformIO/Arduino IDE), outro "
                  "Python ou notebook Jupyter que esteja com ela aberta")
        else:
            print(f"    ERRO  não abriu ({e}). Portas disponíveis:")
            print_ports()
        return None

    received = b""
    ready = None
    try:
        for attempt in range(2):
            deadline = time.monotonic() + (4.0 if attempt == 0 else 1.5)
            while time.monotonic() < deadline and ready is None:
                received += ser.read(256)
                for line in received.split(b"\n"):
                    if line.strip().startswith(b"READY"):
                        ready = line.strip().decode("ascii", "replace")
            if ready:
                break
            ser.write(b"?\n")          # placa que não reinicia ao abrir a porta
    finally:
        ser.close()
    time.sleep(0.3)

    if ready:
        axes = tuple(ready.split()[1:])
        print(f"    ok    o firmware respondeu: {ready}")
        return axes
    if received:
        sample = received[:60]
        print(f"    ERRO  o Arduino mandou {len(received)} bytes, mas não o READY deste firmware "
              f"({sample!r}). Provavelmente é outro programa (ex.: Firmata). Grave este firmware: "
              f"'pio run -t upload' na pasta do projeto")
    else:
        print("    ERRO  o Arduino não respondeu nada. Confira: é a porta certa (--list-ports)? o "
              "firmware deste projeto foi gravado ('pio run -t upload')? o cabo USB passa dados?")
    return None


# ══════════════════════════════════════════════════════════════════════════════
# 3 a 6. Um motor por vez
# ══════════════════════════════════════════════════════════════════════════════
class Motor:
    """Um eixo visto como motor solto: ângulos em voltas do motor."""

    def __init__(self, plate: WavePlate, microstep: float):
        self.plate = plate
        self.axis = plate.axis
        s = plate.get_settings()
        self.deg_per_turn = FULL_STEPS_PER_REV * microstep / s.steps_per_degree
        self.max_rps = s.speed_limit / self.deg_per_turn

    def speed(self, rps: float, accel: float):
        """Velocidade em voltas do motor por segundo."""
        rps = min(rps, self.max_rps)
        self.plate.configure(min_speed=min(0.25, rps) * self.deg_per_turn,
                             max_speed=rps * self.deg_per_turn,
                             acceleration=accel * self.deg_per_turn)

    def turn_to(self, turns: float, wait: bool = True) -> float:
        return self.plate.move_to(turns * self.deg_per_turn, wait=wait)

    def position_turns(self) -> float:
        return self.plate.get_position() / self.deg_per_turn


def test_motor(m: Motor, ask: Asker, rep: AxisReport, microstep: float, everyone: List[WavePlate]):
    print(f"\n=== Motor {m.axis} ===")
    s = m.plate.get_settings()
    print(f"    ·     {s.steps_per_degree:.4f} passos/° no firmware → 1 volta do motor = "
          f"{m.deg_per_turn:.2f}° comandados (microstep {microstep:g}); até {m.max_rps:.2f} voltas/s")
    m.plate.set_zero()                         # referencial: onde o motor está agora

    # 3. torque
    print("\n[3] Torque com o driver ligado")
    for p in everyone:                         # ENABLE comum: só desliga com todos desligados
        p.disable()
    ask.pause(f"Drivers DESLIGADOS. Gire o eixo do motor {m.axis} com os dedos para sentir como é solto.")
    m.plate.enable()
    held = ask.yes(f"Driver LIGADO. Agora o eixo do motor {m.axis} ficou duro (difícil de girar)?")
    if held is True:
        rep.ok.append("segura com o driver ligado")
    elif held is False:
        rep.problems.append(HINT_NO_POWER)
        print(f"    !     {HINT_NO_POWER}")

    # 4. uma volta devagar
    print(f"\n[4] Uma volta devagar ({SLOW_RPS:g} volta/s)")
    m.speed(SLOW_RPS, SLOW_ACCEL)
    ask.pause("Olhe a fita no eixo.")
    m.turn_to(1)
    answer = ask.text("O que aconteceu? [s = 1 volta certinha / m = girou, mas não 1 volta / "
                      "v = vibrou ou zumbiu sem girar / n = nada]")
    m.turn_to(0)
    if answer == "s":
        rep.ok.append("1 volta certa devagar")
        sense = ask.yes("Olhando de frente para a ponta do eixo, ele girou primeiro no sentido "
                        "anti-horário?")
        if sense is not None:
            rep.ok.append(f"ângulo positivo = {'anti-horário' if sense else 'horário'} (de frente para o eixo)")
    elif answer == "m":
        got = ask.text("Quantas voltas ele deu? (ex.: 8, 2, 0.5)")
        try:
            turns = float(got.replace(",", "."))
        except ValueError:
            turns = 0.0
        if turns > 0:
            real = microstep / turns
            text = (f"girou {turns:g} voltas quando pedido 1 → o driver está em microstep ≈ {real:g}, "
                    f"o firmware espera {microstep:g}. Acerte os jumpers MS1-3 embaixo do driver "
                    f"(tabela em firmware/src/main.cpp) ou o 'microstep' do eixo em EIXOS_CFG")
        else:
            text = "não girou a volta pedida: confira os jumpers de microstep e se perde passos"
        rep.problems.append(text)
        print(f"    !     {text}")
    elif answer == "v":
        rep.problems.append(HINT_VIBRATES)
        print(f"    !     {HINT_VIBRATES}")
        return
    elif answer == "n":
        rep.problems.append(HINT_NOTHING)
        print(f"    !     {HINT_NOTHING}")
        return

    # 5. volta à marca
    print(f"\n[5] {MARK_TURNS} voltas para frente e para trás, devagar")
    ask.pause("Guarde onde a fita está apontando.")
    m.turn_to(MARK_TURNS)
    m.turn_to(-MARK_TURNS)
    m.turn_to(0)
    back = ask.yes(f"Depois de +{MARK_TURNS}, −{MARK_TURNS} e de volta, a fita está no mesmo lugar?")
    if back is True:
        rep.ok.append("volta à marca devagar (não perde passos)")
    elif back is False:
        rep.problems.append("não volta à marca mesmo devagar: " + HINT_LOSES_STEPS)
        print(f"    !     {HINT_LOSES_STEPS}")

    # 6. velocidade
    print("\n[6] Subindo a velocidade (3 voltas para frente e para trás em cada uma)")
    speeds = [v for v in LADDER_RPS if v <= m.max_rps + 1e-9]
    for rps in speeds:
        m.speed(rps, LADDER_ACCEL)
        print(f"    ·     {rps:g} volta/s ({rps * 60:.0f} rpm)")
        m.turn_to(3)
        m.turn_to(0)
        smooth = ask.yes(f"A {rps:g} volta/s girou suave e a fita voltou ao mesmo lugar?")
        if smooth is False:
            text = (f"perde passos ou trava a partir de {rps:g} volta/s (ok até "
                    f"{rep.max_rps or 0:g}): " + HINT_LOSES_STEPS)
            rep.problems.append(text)
            print(f"    !     {text}")
            break
        if smooth is True:
            rep.max_rps = rps
    if rep.max_rps:
        rep.ok.append(f"gira bem até {rep.max_rps:g} volta/s")
    m.plate.restore_defaults()
    m.turn_to(0)


# ══════════════════════════════════════════════════════════════════════════════
# 7. Todos juntos
# ══════════════════════════════════════════════════════════════════════════════
def test_together(motors: List[Motor], ask: Asker, reports: Dict[str, AxisReport]):
    names = " ".join(m.axis for m in motors)
    # o Uno gera no máximo ~5000 passos/s SOMANDO todos os eixos
    rps = min([1.0] + [r.max_rps for r in reports.values() if r.max_rps])
    rps = min(rps, min(m.max_rps for m in motors) / len(motors))
    print(f"\n[7] Motores {names} juntos: 2 voltas para frente e para trás a {rps:.2g} volta/s")
    for m in motors:
        m.speed(rps, SLOW_ACCEL)
    ask.pause("Guarde onde as fitas estão apontando.")
    for target in (2, -2, 0):
        for m in motors:
            m.turn_to(target, wait=False)
        for m in motors:
            m.plate.wait_move()
    together = ask.yes("Todos giraram ao mesmo tempo e as fitas voltaram ao mesmo lugar?")
    for m in motors:
        m.plate.restore_defaults()
        if together is True:
            reports[m.axis].ok.append("anda junto com os outros")
        elif together is False:
            reports[m.axis].problems.append(
                "juntos falha, sozinho não: a fonte pode não aguentar a corrente de todos (confira "
                "a corrente da fonte) ou a soma de velocidades passou do limite do Uno")


# ══════════════════════════════════════════════════════════════════════════════
# Execução
# ══════════════════════════════════════════════════════════════════════════════
def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="Teste de motores de passo soltos (sem fim de curso).",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=DEFAULT_PORT, help=f"porta serial (padrão {DEFAULT_PORT})")
    ap.add_argument("--axes", nargs="+", default=list(DEFAULT_AXES), metavar="EIXO",
                    help="eixos a testar (padrão: X Y Z)")
    ap.add_argument("--microstep", type=float, default=MICROSTEP,
                    help=f"microstep configurado no firmware (padrão {MICROSTEP})")
    ap.add_argument("--no-ask", dest="ask", action="store_false", help="só gira, sem perguntas")
    ap.add_argument("--list-ports", action="store_true", help="mostra as portas seriais e sai")
    args = ap.parse_args(argv)
    args.axes = [a.upper() for a in args.axes]
    if not sys.stdin.isatty():
        args.ask = False
    return args


def print_summary(reports: Dict[str, AxisReport]):
    print("\n" + "=" * 78)
    print("RESUMO")
    for rep in reports.values():
        status = "OK" if rep.ok and not rep.problems else ("PROBLEMA" if rep.problems else "?")
        print(f"\n  Motor {rep.axis}: {status}")
        for line in rep.ok:
            print(f"    ok  {line}")
        for line in rep.problems:
            print(f"    !   {line}")
        if not rep.ok and not rep.problems:
            print("    (sem respostas: rode com perguntas para ter o diagnóstico)")


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass
    args = parse_args(argv)
    if args.list_ports:
        print_ports()
        return 0

    print(f"Teste de motores soltos: porta {args.port}, eixos {' '.join(args.axes)}")
    print("Os motores vão girar algumas voltas. Fonte de 12–24 V ligada no shield?")

    firmware_axes = check_port(args.port)
    if firmware_axes is None:
        return 2

    print("\n[2] Eixos")
    missing = [a for a in args.axes if a not in firmware_axes]
    if missing:
        print(f"    ERRO  o firmware não tem {' '.join(missing)} (tem {' '.join(firmware_axes)}): "
              f"confira EIXOS_CFG em firmware/src/main.cpp e grave de novo")
        return 2
    print(f"    ok    {' '.join(args.axes)} existem no firmware")

    plates: List[WavePlate] = []
    reports = {a: AxisReport(a) for a in args.axes}
    ask = Asker(args.ask)
    try:
        for axis in args.axes:
            plates.append(WavePlate(args.port, axis, disable_when_idle=False))
        motors = [Motor(p, args.microstep) for p in plates]
        for m in motors:
            test_motor(m, ask, reports[m.axis], args.microstep, plates)
        working = [m for m in motors if not any("vibra" in p or "não se mexe" in p
                                                for p in reports[m.axis].problems)]
        if len(working) > 1:
            test_together(working, ask, reports)
    except KeyboardInterrupt:
        print("\nInterrompido: parando os motores.")
        for p in plates:
            try:
                if p.is_moving():
                    p.stop()
            except (WavePlateError, TimeoutError):
                pass
    except (WavePlateError, TimeoutError, serial.SerialException) as e:
        print(f"\nERRO de comunicação: {type(e).__name__}: {e}")
        return 2
    finally:
        for p in plates:                         # desliga os drivers: motores soltos e frios
            try:
                p.close()
            except Exception:  # noqa: BLE001
                pass
        print_summary(reports)
    return 1 if any(r.problems for r in reports.values()) else 0


if __name__ == "__main__":
    sys.exit(main())
