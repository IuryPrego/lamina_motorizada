"""
Testes ópticos de histerese só da lâmina X.

São os testes de hysteresis_test.py com a lâmina X no feixe: só o X é aberto
e nenhum comando vai para Y, Z ou A. O teste crosstalk (outra lâmina andando
perturba esta?) precisa de uma segunda lâmina e fica de fora; para ele use
hysteresis_test.py --other.

MONTAGEM
    laser → P1 (polarizador fixo) → lâmina X → P2 (analisador fixo) → detector

    --element hwp        λ/2 entre polarizadores (padrão)            período 90°
    --element qwp        λ/4 entre polarizadores (3× menos sensível) período 90°
    --element polarizer  polarizador na montagem do X, sem P2        período 180°

    P1 e P2 paralelos ou cruzados, tanto faz. Deixe o laser aquecer, centre o
    feixe na lâmina e evite luz ambiente.

DETECTOR
    --detector powermeter  Thorlabs PM pelo EASYPM.py de bell-measurements-main (padrão)
    --detector timetagger  taxa de contagem de um canal da Swabian TimeTagger
    --detector manual      você digita a leitura

TESTES (--list)
    sweep      curva completa subindo e descendo: fase, contraste, folga, ângulo dos eixos
    approach   chegar ao mesmo ângulo por baixo e por cima, com várias distâncias
    loop       laço de histerese: vai e volta em passos pequenos no ponto de trabalho
    idle       quanto a lâmina se mexe quando o driver desliga (disable_when_idle)
    stability  ruído e deriva parada: driver ligado × desligado, e ruído do laser
    repeat     repetibilidade de posicionamento, sem e com referência (home) no meio
    drift      passos perdidos: muitos movimentos aleatórios e volta ao ponto de trabalho
    speed      folga e passos perdidos com perfis de velocidade diferentes

    Como cada medida funciona: veja o cabeçalho de hysteresis_test.py.

Uso:
    python hysteresis_test_x.py                          # λ/2, power meter, COM4
    python hysteresis_test_x.py --element polarizer
    python hysteresis_test_x.py --detector timetagger --channel 2
    python hysteresis_test_x.py --quick                  # ~metade do tempo
    python hysteresis_test_x.py --only sweep loop

Aceita as mesmas opções de hysteresis_test.py, menos --plate e --other.
Resultados em test_results/hysteresis_X_<data>/.
"""

from __future__ import annotations

import sys
from pathlib import Path

import hysteresis_test as base

AXIS = "X"

base.TESTS[:] = [t for t in base.TESTS if t.name != "crosstalk"]

_base_parse_args = base.parse_args


def parse_args(argv=None):
    args = _base_parse_args(argv)
    if base.parse_plate(args.plate, args.port)[1] != AXIS or args.other:
        raise SystemExit(f"{Path(__file__).name} testa só a lâmina {AXIS}; "
                         f"para outras lâminas use hysteresis_test.py --plate ... --other ...")
    return args


def driver_off_single(self):
    """
    Desliga o driver mandando D só para o X. O ENABLE do CNC Shield é comum a
    todos os eixos, mas o firmware só o mantém ligado se algum eixo pediu (G, H
    ou R). Abrir a porta reinicia o Arduino e este arquivo nunca manda comando
    para Y, Z ou A, então nenhum deles pediu e o D do X basta.
    """
    self.plate.disable()


base.parse_args = parse_args
base.Bench.driver_off = driver_off_single

if __name__ == "__main__":
    sys.exit(base.main())
