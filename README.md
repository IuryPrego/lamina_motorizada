# Lâmina motorizada

Lâminas de onda (λ/2, λ/4) e polarizadores girados por motores de passo, controlados por
um **Arduino Uno + CNC Shield v3**, com uma biblioteca Python que imita a interface das
montagens **Thorlabs Elliptec**. O objetivo é substituir as Elliptec nos códigos de
medida do laboratório (violação de Bell, visibilidade da fonte SPDC) trocando só o
`import`, o construtor e o `close()`.

```
PC (Python) ──USB/serial 115200── Arduino Uno + CNC Shield v3 ── drivers A4988/DRV8825 ── motores (X, Y, Z, A)
                                                     └── fins de curso (zero mecânico)
```

## Estrutura do repositório

| Caminho | O que é |
|---|---|
| [src/main.cpp](src/main.cpp) | Firmware: perfil de movimento com velocidade mínima, até 4 eixos independentes, referência no fim de curso, protocolo serial em texto |
| [platformio.ini](platformio.ini) | Ambientes de compilação `uno` e `nano` |
| [waveplate.py](waveplate.py) | Biblioteca principal: classe `WavePlate` (um objeto por eixo), `open_plates()` |
| [arduino_elliptec.py](arduino_elliptec.py) | Camada compatível com a Elliptec: `EasyMotor` / `ElliptecMotor` (`move_to(pos, addr=)`, `home(addr=[...])`, `close()`) |
| [waveplate_cli.py](waveplate_cli.py) | Controle interativo de uma lâmina pelo terminal |
| [test.py](test.py) | Exemplo mínimo de uso da `WavePlate` |
| [waveplate_tutorial.ipynb](waveplate_tutorial.ipynb) | Tutorial da biblioteca `waveplate` |
| [arduino_elliptec_tutorial.ipynb](arduino_elliptec_tutorial.ipynb) | Tutorial da troca Elliptec → Arduino |
| [motor_test.py](motor_test.py) | Diagnóstico de bancada de motores soltos (porta, torque, microstep, perda de passos) |
| [hardware_test.py](hardware_test.py) | Bateria completa de testes de hardware + software (usa o fim de curso como referência) |
| [hardware_test_x.py](hardware_test_x.py) | Mesma bateria, só com a lâmina X |
| [features_test.py](features_test.py) | Testes de `set_approach`, `set_zero` e `calibrate` |
| [hysteresis_test.py](hysteresis_test.py) | Testes ópticos de folga/histerese com polarizadores e detector |
| [hysteresis_test_x.py](hysteresis_test_x.py) | Mesmos testes ópticos, só com a lâmina X |
| [analyze_results.py](analyze_results.py) | Resume a histerese máxima medida em `test_results/` |
| [pyproject.toml](pyproject.toml) | Empacotamento (`pip install -e .`) |

Pastas `include/`, `lib/` e `test/` são do modelo padrão do PlatformIO (vazias por enquanto).

> **bell-measurements-main/** (códigos e notebooks do laboratório que usam as Elliptec)
> não faz parte deste repositório: fica só na máquina local, na raiz do projeto.
> `hysteresis_test.py` usa o `EASYPM.py` dessa pasta para ler o power meter Thorlabs.

## Firmware

Requer o [PlatformIO](https://platformio.org/).

```bash
pio run -t upload            # compila e grava no Uno
pio run -e nano -t upload    # se a placa for um Nano
pio device monitor           # teste manual: digite "X G 90", "X P", "?" ...
```

Os eixos ligados são definidos em `EIXOS_CFG` no início de [src/main.cpp](src/main.cpp)
(pinos, microstep, redução, sentido). Com o padrão (1/8 de passo, redução 5:1) são
22,22 micropassos por grau da lâmina (≈ 0,045°).

### Protocolo serial (resumo)

Cada linha começa com a letra do eixo; a resposta volta com a mesma letra.

| Comando | Ação | Resposta |
|---|---|---|
| `X G 45` | vai para 45° absolutos | `X OK 45.00`, depois `X DONE 45.00` |
| `X P` | posição | `X POS 12.30` |
| `X R` | referência no fim de curso | `X OK`, depois `X DONE ...` ou `X ERR ...` |
| `X S` | para com desaceleração | `X OK`, depois `X DONE ...` |
| `X Z` / `D` / `H` | zero aqui / desabilita / habilita driver | |
| `X M` / `V` / `A` / `T` | vel. mínima / máxima / aceleração / desligar após parar | `X OK` |
| `X F` / `X I` | lê o fim de curso / configuração | `X FIM 1`, `X INFO ...` |
| `?` | lista os eixos | `READY X Y` |

A lista completa está no cabeçalho de [src/main.cpp](src/main.cpp).

## Biblioteca Python

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -e .             # instala waveplate e arduino_elliptec (depende de pyserial)
```

```python
from waveplate import WavePlate

with WavePlate("COM4", "X") as plate:
    plate.home()             # zero mecânico no fim de curso
    plate.move_to(45)        # ângulo absoluto
    plate.move_by(-10)       # relativo
    print(plate.get_position())
```

### Trocando a Elliptec nos códigos do laboratório

```python
# from EASYMOTOR import EasyMotor                   # Thorlabs
from arduino_elliptec import EasyMotor              # Arduino

stage = EasyMotor("COM4", axes={0: "X", 2: "Y"})    # endereço Elliptec → eixo
stage.home(addr=[0, 2])
stage.move_to(45, addr=2)
stage.close()
```

Atenção: o zero é onde o fim de curso aciona, não o zero de fábrica da Thorlabs — os
ângulos de calibração (H_alice, H_bob, ...) precisam ser medidos de novo.

## Testes

Todos rodam contra o hardware real e gravam um JSON em `test_results/` (pasta fora do
git). Use `--list` em cada script para ver os testes disponíveis.

```bash
python motor_test.py --axes X Y           # motores soltos, diagnóstico guiado
python hardware_test.py --quick           # bateria de hardware + software
python features_test.py                   # aproximação, zero de software, calibrate
python hysteresis_test.py --element hwp   # óptico: laser → P1 → lâmina → P2 → detector
python analyze_results.py                 # resumo da histerese medida
```

As lâminas dão várias voltas durante os testes: confira que nenhum cabo impede a rotação.

## O que não vai para o git

Definido em [.gitignore](.gitignore): build do PlatformIO (`.pio/`), ambiente virtual
(`.venv/`), caches do Python, `*.egg-info`, resultados de teste (`test_results/`),
a pasta local `bell-measurements-main/` e vídeos (`*.avi`, `*.mp4`, `*.mov`).
