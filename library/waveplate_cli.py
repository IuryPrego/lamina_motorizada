"""
Controle interativo de uma lâmina motorizada pela serial.
Usa a biblioteca waveplate.py (que pode ser importada nos seus próprios códigos).

Requer:  pip install pyserial
"""

import sys

import serial

from waveplate import WavePlate, WavePlateError

SERIAL_PORT = "COM4"
AXIS = "X"          # letra do eixo no firmware (X, Y, ...)

# Mude para True depois de testar o fim de curso com o comando 'home'
HOME_ON_START = False

# No modo "desabilitar quando parado", o driver desliga este tempo depois de
# cada movimento (dá tempo da lâmina assentar) e religa sozinho no próximo.
DISABLE_DELAY_MS = 200

HELP = """\
Digite um ângulo absoluto (ex: 90, -45, 180,5) e pressione Enter.
Comandos:
  rel <graus>    anda relativo à posição atual (ex: rel -10)
  home           busca o zero mecânico
  zero           define 0° aqui
  pos            mostra a posição
  limit          lê o fim de curso
  mode           alterna driver sempre habilitado / desabilitado quando parado
  settings       mostra velocidades, aceleração e limites
  vmin <°/s>     velocidade mínima (arranque, chegada e busca do zero)
  vmax <°/s>     velocidade máxima
  accel <°/s²>   aceleração
  defaults       volta velocidades e aceleração aos valores do firmware
  help, quit
Ctrl+C durante um movimento para a lâmina com desaceleração.
"""


def choose_mode() -> bool:
    """Pergunta o modo do driver. Retorna True para desabilitar quando parado."""
    print("Modo do driver:")
    print("  1 - sempre habilitado (segura a posição com força)")
    print("  2 - desabilitado quando parado (motor frio, sem ruído)")
    while True:
        try:
            choice = input("Escolha [1/2] → ").strip()
        except EOFError:
            return False
        if choice in ("1", ""):
            return False
        if choice == "2":
            return True
        print("  ! Digite 1 ou 2.")


def describe_mode(disable_idle: bool) -> str:
    return "desabilitado quando parado" if disable_idle else "sempre habilitado"


def parse_number(text: str) -> float:
    return float(text.replace(",", "."))


def run_move(plate: WavePlate, move, value: float):
    """Executa um movimento; Ctrl+C para a lâmina e mostra onde ficou."""
    try:
        final = move(value)
    except KeyboardInterrupt:
        print("\n  ! Parado.")
        final = plate.get_position()
    print(f"  → Posição atual: {final:.2f}°")


def main():
    print(f"Conectando ao Arduino em {SERIAL_PORT} (eixo {AXIS})...")
    try:
        plate = WavePlate(SERIAL_PORT, AXIS)
    except (serial.SerialException, TimeoutError, WavePlateError, ValueError) as e:
        print(f"ERRO ao conectar: {e}")
        sys.exit(1)

    print("Conectado!")
    print(f"  {plate.get_settings()}\n")

    disable_idle = choose_mode()
    plate.disable_when_idle(disable_idle, DISABLE_DELAY_MS)
    print(f"  → Driver: {describe_mode(disable_idle)}\n")
    print(HELP)

    if HOME_ON_START:
        print("Buscando o zero mecânico...")
        try:
            print(f"  → Referenciado em {plate.home():.2f}°\n")
        except (WavePlateError, TimeoutError, KeyboardInterrupt) as e:
            print(f"  ! Falha na referência: {e}\n")

    try:
        while True:
            try:
                line = input("Ângulo → ").strip().lower()
            except EOFError:
                break
            if not line:
                continue

            parts = line.split()
            cmd, arg = parts[0], (parts[1] if len(parts) > 1 else None)

            try:
                if cmd in ("quit", "exit", "q"):
                    break
                elif cmd == "help":
                    print(HELP)
                elif cmd == "zero":
                    plate.set_zero()
                    print("  → Posição atual definida como 0°.")
                elif cmd == "limit":
                    state = "ACIONADO" if plate.limit_switch_triggered() else "livre"
                    print(f"  → Fim de curso: {state}")
                elif cmd == "home":
                    print("  → Buscando o zero mecânico...")
                    try:
                        final = plate.home()
                        print(f"  → Referenciado. Posição atual: {final:.2f}°")
                    except KeyboardInterrupt:
                        print("\n  ! Busca cancelada.")
                elif cmd == "pos":
                    print(f"  → Posição atual: {plate.get_position():.2f}°")
                elif cmd == "mode":
                    disable_idle = not disable_idle
                    plate.disable_when_idle(disable_idle, DISABLE_DELAY_MS)
                    if disable_idle:
                        plate.disable()
                    else:
                        plate.enable()
                    print(f"  → Driver: {describe_mode(disable_idle)}")
                elif cmd == "settings":
                    print(f"  → {plate.get_settings()}")
                elif cmd in ("vmin", "vmax", "accel"):
                    if arg is None:
                        print(f"  ! Uso: {cmd} <valor>")
                        continue
                    key = {"vmin": "min_speed", "vmax": "max_speed", "accel": "acceleration"}[cmd]
                    print(f"  → {plate.configure(**{key: parse_number(arg)})}")
                elif cmd == "defaults":
                    print(f"  → {plate.restore_defaults()}")
                elif cmd == "rel":
                    if arg is None:
                        print("  ! Uso: rel <graus>")
                        continue
                    delta = parse_number(arg)
                    print(f"  → Andando {delta:+.2f}°...")
                    run_move(plate, plate.move_by, delta)
                else:
                    angle = parse_number(line)
                    print(f"  → Movendo para {angle:.2f}°...")
                    run_move(plate, plate.move_to, angle)

            except ValueError as e:
                msg = str(e)
                if msg.startswith("could not convert"):
                    msg = "valor inválido. Digite um número (ex: 90 ou -45.5)."
                print(f"  ! {msg}")
            except WavePlateError as e:
                print(f"  ! Arduino: {e}")
            except TimeoutError as e:
                print(f"  ! {e}")

    except KeyboardInterrupt:
        print("\nInterrompido pelo usuário.")

    finally:
        plate.close()
        print("Driver desabilitado. Conexão encerrada.")


if __name__ == "__main__":
    main()
