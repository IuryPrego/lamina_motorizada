"""
Testes de hardware + software só da lâmina X.

São os testes de hardware_test.py rodando com o eixo X sozinho: só o X é
aberto e nenhum comando vai para Y, Z ou A. Serve para testar o X isolado
(por exemplo, com um motor só ligado no CNC Shield). Os testes que precisam de
duas lâminas (juntas × uma depois da outra, limite de velocidade somada,
fios trocados entre eixos, threads) ficam de fora; para eles use
hardware_test.py.

Testes (veja --list):
  básico       connect, settings, errors, serial, switch-read, latency,
               identify [pergunta]
  uma lâmina   home, edge, revolution, positioning, timing, nonblocking, busy,
               stop, home-cancel, ctrl-c, driver, configure, calibrate,
               switch-press [pergunta], elliptec
  resistência  travel-limits [demorado], endurance [demorado],
               torque [pergunta], reconnect

Uso:
    python tests/hardware_test_x.py                       # X na COM4
    python tests/hardware_test_x.py --port COM5
    python tests/hardware_test_x.py --quick               # pula os demorados
    python tests/hardware_test_x.py --no-interactive      # sem perguntas
    python tests/hardware_test_x.py --only edge revolution
    python tests/hardware_test_x.py --list

Aceita as mesmas opções de hardware_test.py, menos --plates. Resultado: uma
tabela no fim e um JSON em test_results/.

ATENÇÃO: a lâmina dá várias voltas (até 10 em cada sentido no teste
travel-limits). Confira que nada no caminho impede a rotação.
"""

from __future__ import annotations

import sys
from pathlib import Path

import hardware_test as base

AXIS = "X"

# só os testes que fazem sentido com uma lâmina
base.TESTS[:] = [t for t in base.TESTS if t.min_plates <= 1]
base.DEFAULT_PLATES = (AXIS,)

DESCRIPTIONS = {
    "home": f"referência do {AXIS}; sensor acionado só no zero",
    "switch-press": f"você aperta o fim de curso do {AXIS} com a mão; ele acusa, e solto não acusa",
    "elliptec": f"ArduinoElliptec/EasyMotor com o {AXIS} no endereço 0, como no código do laboratório",
    "travel-limits": f"{AXIS} até +limite, -limite e de volta; sem perder passo",
    "endurance": f"muitos movimentos aleatórios do {AXIS} com velocidades variadas",
    "torque": f"você sente o driver segurando / soltando o {AXIS}",
}
for spec in base.TESTS:
    spec.description = DESCRIPTIONS.get(spec.name, spec.description)
    if spec.group == "duas ou mais":         # os que sobraram (switch-press, elliptec) usam uma só
        spec.group = "uma lâmina"

_base_parse_args = base.parse_args


def parse_args(argv=None):
    args = _base_parse_args(argv)
    axes = [base.parse_plate(s, args.port)[1] for s in args.plates]
    if axes != [AXIS]:
        raise SystemExit(f"{Path(__file__).name} testa só a lâmina {AXIS}; "
                         f"para outras lâminas use hardware_test.py --plates ...")
    return args


base.parse_args = parse_args

if __name__ == "__main__":
    sys.exit(base.main())
