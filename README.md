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

```
lamina_motorizada/
├── firmware/                 código do Arduino (PlatformIO)
│   ├── src/main.cpp          firmware: movimento, referência, protocolo serial
│   └── include/ lib/ test/   pastas padrão do PlatformIO (vazias)
├── library/                  biblioteca Python (base, instalada pelo uv)
│   ├── waveplate.py
│   ├── arduino_elliptec.py
│   └── waveplate_cli.py
├── tests/                    testes contra o hardware real + análise
├── tutorials/                notebooks e exemplo mínimo
├── platformio.ini            configuração do PlatformIO (aponta para firmware/)
├── pyproject.toml, uv.lock   dependências Python (uv)
├── test_results/             resultados dos testes (local, fora do git)
└── bell-measurements-main/   códigos do laboratório (local, fora do git)
```

### firmware/

| Arquivo | O que é |
|---|---|
| [firmware/src/main.cpp](firmware/src/main.cpp) | Perfil de movimento com velocidade mínima, até 4 eixos independentes, referência no fim de curso, protocolo serial em texto |

### library/ — biblioteca

| Arquivo | O que é |
|---|---|
| [waveplate.py](library/waveplate.py) | Biblioteca principal: classe `WavePlate` (um objeto por eixo), `open_plates()` |
| [arduino_elliptec.py](library/arduino_elliptec.py) | Camada compatível com a Elliptec: `EasyMotor` / `ElliptecMotor` (`move_to(pos, addr=)`, `home(addr=[...])`, `close()`) |
| [waveplate_cli.py](library/waveplate_cli.py) | Controle interativo de uma lâmina pelo terminal (`uv run waveplate-cli`) |

Os módulos são instalados no nível superior: `import waveplate` e
`from arduino_elliptec import EasyMotor` funcionam de qualquer pasta, sem mexer no `sys.path`.

### tests/ — testes

| Arquivo | O que é |
|---|---|
| [motor_test.py](tests/motor_test.py) | Diagnóstico de bancada de motores soltos (porta, torque, microstep, perda de passos) |
| [hardware_test.py](tests/hardware_test.py) | Bateria completa de testes de hardware + software (usa o fim de curso como referência) |
| [hardware_test_x.py](tests/hardware_test_x.py) | Mesma bateria, só com a lâmina X |
| [features_test.py](tests/features_test.py) | Testes de `set_approach`, `set_zero` e `calibrate` |
| [hysteresis_test.py](tests/hysteresis_test.py) | Testes ópticos de folga/histerese com polarizadores e detector |
| [hysteresis_test_x.py](tests/hysteresis_test_x.py) | Mesmos testes ópticos, só com a lâmina X |
| [analyze_results.py](tests/analyze_results.py) | Resume a histerese máxima medida em `test_results/` |
| [EASYPM.py](tests/EASYPM.py) | Leitura do power meter Thorlabs (`PowerMeter`), usada pelos testes de histerese |
| [TLPMX.py](tests/TLPMX.py) | Wrapper Python da DLL do power meter, fornecido pela Thorlabs (usado pelo `EASYPM.py`) |

### tutorials/ — tutoriais

| Arquivo | O que é |
|---|---|
| [waveplate_tutorial.ipynb](tutorials/waveplate_tutorial.ipynb) | Tutorial da biblioteca `waveplate` |
| [arduino_elliptec_tutorial.ipynb](tutorials/arduino_elliptec_tutorial.ipynb) | Tutorial da troca Elliptec → Arduino |
| [basic_example.py](tutorials/basic_example.py) | Exemplo mínimo de uso da `WavePlate` |

> **bell-measurements-main/** (códigos e notebooks do laboratório que usam as Elliptec)
> não faz parte deste repositório: fica só na máquina local, na raiz do projeto.
> Os testes não dependem dela.

## Firmware

Requer o [PlatformIO](https://platformio.org/). O código fica em `firmware/`, mas os comandos
rodam na raiz do projeto (onde está o `platformio.ini`).

```bash
pio run -t upload            # compila e grava no Uno
pio run -e nano -t upload    # se a placa for um Nano
pio device monitor           # teste manual: digite "X G 90", "X P", "?" ...
```

Os eixos ligados são definidos em `EIXOS_CFG` no início de [firmware/src/main.cpp](firmware/src/main.cpp)
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

A lista completa está no cabeçalho de [firmware/src/main.cpp](firmware/src/main.cpp).

## Biblioteca Python

O ambiente é gerenciado pelo [uv](https://docs.astral.sh/uv/). Um único comando baixa
o Python certo (versão em [.python-version](.python-version)), cria o `.venv` e instala
tudo com as versões travadas em [uv.lock](uv.lock):

```bash
uv sync                          # projeto + todos os grupos de dependências
uv run python tests/hardware_test.py   # roda qualquer script dentro do ambiente
uv run waveplate-cli                   # controle interativo de uma lâmina
```

Instalar o uv no Windows (uma vez): `powershell -c "irm https://astral.sh/uv/install.ps1 | iex"`

| Grupo | Pacotes | Para quê |
|---|---|---|
| *(base)* | `pyserial` | bibliotecas `waveplate` e `arduino_elliptec` |
| `test` | `numpy`, `matplotlib` | testes de histerese e gráficos |
| `notebooks` | `ipykernel`, `numpy`, `matplotlib` | notebooks tutoriais no VS Code/Jupyter |
| `lab` | `pylablib-lightweight`, `pyft232`, `numba`, `scipy`, `tqdm` | códigos locais de `bell-measurements-main/` (Elliptec original) |

`uv sync` instala todos os grupos (`default-groups = "all"` no
[pyproject.toml](pyproject.toml)). Para instalar só o básico: `uv sync --no-default-groups`.
Para adicionar um pacote: `uv add <pacote>` (ou `uv add --group lab <pacote>`).

Não vêm do PyPI e precisam do instalador do fabricante: o SDK da **Swabian TimeTagger**
(`--detector timetagger`) e o driver do **power meter Thorlabs** (software Optical Power
Monitor), que instala a DLL `TLPMX_64.dll` carregada pelo `tests/TLPMX.py`.

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

Todos rodam contra o hardware real e gravam os resultados em `test_results/`, na raiz
do projeto (pasta fora do git). Use `--list` em cada script para ver os testes disponíveis.

```bash
uv run python tests/motor_test.py --axes X Y           # motores soltos, diagnóstico guiado
uv run python tests/hardware_test.py --quick           # bateria de hardware + software
uv run python tests/features_test.py                   # aproximação, zero de software, calibrate
uv run python tests/hysteresis_test.py --element hwp   # óptico: laser → P1 → lâmina → P2 → detector
uv run python tests/analyze_results.py                 # resumo da histerese medida
```

As lâminas dão várias voltas durante os testes: confira que nenhum cabo impede a rotação.

## O que não vai para o git

Definido em [.gitignore](.gitignore): build do PlatformIO (`.pio/`), ambiente virtual
(`.venv/`), caches do Python, `*.egg-info`, resultados de teste (`test_results/`),
a pasta local `bell-measurements-main/` e vídeos (`*.avi`, `*.mp4`, `*.mov`).
