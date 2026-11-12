"""
Analisa o último teste completo de cada bateria em test_results/ e mostra a
histerese máxima medida por cada teste em vigência (os que estão registrados
hoje em hardware_test.py, features_test.py e hysteresis_test.py).

"Histerese" aqui é tudo que mede quanto a lâmina NÃO volta ao mesmo lugar físico:
  - edge            distância da borda do fim de curso ao zero, espalhamento entre
                    referências e folga das engrenagens + histerese do sensor
  - revolution      erro da borda depois de ±N voltas
  - calibrate       borda depois do calibrate() × onde deveria estar
  - travel-limits,  quanto a borda mudou depois de muitos movimentos
    endurance
  - speed-limit     quanto a borda mudou em cada velocidade
  - approach-history (features) borda depois de vir de + × de −, com e sem aproximação
  - hysteresis_test folga, laço, deslocamento ao desligar o driver, deriva...
Os valores saem em micropassos (µp) e graus; 1 µp = 1/(passos por grau) = 0.045°.

Uso:
    python analyze_results.py              # último teste completo de cada bateria
    python analyze_results.py --detail     # todas as medidas, não só a máxima
    python analyze_results.py --file test_results/hardware_20261111_191926.json
    python analyze_results.py --no-later   # ignora testes rodados de novo depois (--only)
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Tuple

ROOT = Path(__file__).resolve().parent
RESULTS_DIR = ROOT / "test_results"
SUITE_SOURCES = {"hardware": "hardware_test.py", "features": "features_test.py",
                 "hysteresis": "hysteresis_test.py"}
FALLBACK_STEPS_PER_DEGREE = 200 * 8 * 5 / 360.0     # 22.2222


# ══════════════════════════════════════════════════════════════════════════════
# Leitura dos arquivos
# ══════════════════════════════════════════════════════════════════════════════
MULTI_PLATE: set = set()        # testes que precisam de duas ou mais lâminas


def registered_tests(source: Path) -> List[str]:
    """Nomes dos testes registrados no arquivo (decorator @register ou register(...))."""
    if not source.exists():
        return []
    text = source.read_text(encoding="utf-8")
    names = []
    for m in re.finditer(r'\bregister\(\s*"([^"]+)"(?:,\s*"([^"]*)")?', text):
        names.append(m.group(1))
        if m.group(2) == "duas ou mais":
            MULTI_PLATE.add(m.group(1))
    return list(dict.fromkeys(names))


@dataclass
class Run:
    path: Path
    suite: str
    date: datetime
    data: dict
    complete: bool

    @property
    def tests(self) -> Dict[str, dict]:
        return {t["name"]: t for t in self.data.get("tests", [])}

    def steps_per_degree(self) -> Dict[str, float]:
        """Passos por grau de cada lâmina ('' = valor geral da execução)."""
        out: Dict[str, float] = {}
        for plate, s in (self.data.get("settings") or {}).items():
            if isinstance(s, dict) and s.get("steps_per_degree"):
                out[plate] = float(s["steps_per_degree"])
        micro = self.data.get("microstep_deg")
        if micro:
            out[""] = 1.0 / float(micro)
        if "" not in out:
            out[""] = next(iter(out.values()), FALLBACK_STEPS_PER_DEGREE)
        return out

    def plates(self) -> str:
        if self.data.get("plates"):
            return ", ".join(self.data["plates"])
        return str(self.data.get("plate", "?"))


def load_runs(results_dir: Path, registry: Dict[str, List[str]]) -> List[Run]:
    features_only = set(registry["features"]) - set(registry["hardware"])
    runs = []
    candidates = [(p, None) for p in results_dir.glob("hardware_*.json")]
    candidates += [(p, "hysteresis") for p in results_dir.glob("hysteresis_*/summary.json")]
    for path, suite in candidates:
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            print(f"  (ignorando {path.name}: {exc})", file=sys.stderr)
            continue
        names = {t.get("name") for t in data.get("tests", [])}
        if not names:
            continue
        if suite is None:
            suite = "features" if names & features_only else "hardware"
        try:
            date = datetime.fromisoformat(data["date"])
        except (KeyError, ValueError):
            date = datetime.fromtimestamp(path.stat().st_mtime)
        args = data.get("args") or {}
        complete = not args.get("only")
        runs.append(Run(path, suite, date, data, complete))
    runs.sort(key=lambda r: r.date)
    return runs


# ══════════════════════════════════════════════════════════════════════════════
# Medidas de histerese
# ══════════════════════════════════════════════════════════════════════════════
@dataclass
class Finding:
    plate: str
    microsteps: float          # com sinal
    degrees: float
    what: str


LINE_PREFIX = re.compile(r"^\S+\s+")
PLATE_PREFIX = re.compile(r"^(.+?): ")


def log_lines(test: dict) -> Iterable[Tuple[str, str]]:
    """(lâmina, texto) de cada linha do log, sem o prefixo ok/FALHA/·."""
    for line in test.get("log", []):
        text = LINE_PREFIX.sub("", line, count=1)
        m = PLATE_PREFIX.match(text)
        yield (m.group(1), text[m.end():]) if m else ("", text)


def split_key(key: str) -> Tuple[str, str]:
    """'X.edges' → ('X', 'edges'); 'level_0.5' → ('', 'level_0.5')."""
    if "." in key and not key.startswith("level_"):
        plate, _, rest = key.partition(".")
        return plate, rest
    return "", key


class Context:
    def __init__(self, run: Run):
        self.spd = run.steps_per_degree()

    def spd_of(self, plate: str) -> float:
        return self.spd.get(plate, self.spd[""])

    def from_degrees(self, plate: str, deg: float, what: str) -> Finding:
        return Finding(plate, deg * self.spd_of(plate), deg, what)

    def from_microsteps(self, plate: str, us: float, what: str) -> Finding:
        return Finding(plate, us, us / self.spd_of(plate), what)


Extractor = Callable[[dict, Context], List[Finding]]


def extract_edge(test: dict, ctx: Context) -> List[Finding]:
    out = []
    for key, value in test.get("metrics", {}).items():
        plate, name = split_key(key)
        if name == "edges" and value:
            worst = max(value, key=abs)
            out.append(ctx.from_degrees(plate, worst, "borda após a referência, distância ao zero"))
            out.append(ctx.from_degrees(plate, max(value) - min(value),
                                        f"espalhamento da borda entre {len(value)} referências"))
        elif name == "backlash_plus_switch_hysteresis":
            out.append(ctx.from_degrees(plate, float(value),
                                        "folga das engrenagens + histerese do sensor (aciona × libera)"))
    return out


def extract_revolution(test: dict, ctx: Context) -> List[Finding]:
    out = []
    for key, value in test.get("metrics", {}).items():
        plate, name = split_key(key)
        if name == "turn_errors":
            for turns, err in value:
                out.append(ctx.from_degrees(plate, float(err), f"borda depois de {turns:+d} volta(s)"))
    return out


CALIBRATE_EDGE = re.compile(r"aciona em ([+-]?\d+\.\d+)° do referencial.*?esperado ([+-]?\d+\.\d+)°")


def extract_calibrate(test: dict, ctx: Context) -> List[Finding]:
    out = []
    for plate, text in log_lines(test):
        m = CALIBRATE_EDGE.search(text)
        if m:
            got, expected = float(m.group(1)), float(m.group(2))
            out.append(ctx.from_degrees(plate, got - expected,
                                        f"borda depois do calibrate() (esperada em {expected:g}°)"))
    return out


EDGE_SHIFT = re.compile(r"^(.*?),? a borda mudou ([+-]?\d+(?:\.\d+)?) micropasso")


def extract_edge_shift(test: dict, ctx: Context) -> List[Finding]:
    out = []
    for plate, text in log_lines(test):
        m = EDGE_SHIFT.search(text)
        if m:
            when = m.group(1).strip() or "depois do teste"
            out.append(ctx.from_microsteps(plate, float(m.group(2)), f"borda {when}"))
    return out


def extract_speed_limit(test: dict, ctx: Context) -> List[Finding]:
    out = []
    for key, value in test.get("metrics", {}).items():
        if key.startswith("level_") and isinstance(value, dict):
            for plate, us in (value.get("drift_us") or {}).items():
                if us is not None:
                    out.append(ctx.from_microsteps(plate, float(us),
                                                   f"borda depois de andar juntas a {value.get('deg_s', '?')} °/s"))
    return out


def extract_approach_history(test: dict, ctx: Context) -> List[Finding]:
    out = []
    for key, value in test.get("metrics", {}).items():
        plate, label = split_key(key)
        if isinstance(value, dict) and "shift_microsteps" in value:
            out.append(ctx.from_microsteps(plate, float(value["shift_microsteps"]),
                                           f"{label}: borda vindo de + × vindo de −"))
            out.append(ctx.from_microsteps(plate, float(value["spread_microsteps"]),
                                           f"{label}: espalhamento de todas as bordas"))
    return out


# hysteresis_test: métricas em graus cujo nome indica deslocamento/folga
HYST_KEY = re.compile(r"(backlash|shift|difference|change|return|total|gap)", re.I)


def walk_numbers(value, path: str = "") -> Iterable[Tuple[str, float]]:
    if isinstance(value, bool):
        return
    if isinstance(value, (int, float)):
        yield path, float(value)
    elif isinstance(value, dict):
        for k, v in value.items():
            yield from walk_numbers(v, f"{path}.{k}" if path else str(k))
    elif isinstance(value, list):
        for i, v in enumerate(value):
            yield from walk_numbers(v, f"{path}[{i}]")


HYST_LABELS = {
    "backlash_deg": "folga (mesmo ângulo subindo × descendo)",
    "backlash_per_cycle": "folga em um dos ciclos",
    "difference": "chegando por baixo × por cima",
    "max_shift_deg": "quanto a lâmina mexe ao desligar o driver",
    "max_return_deg": "quanto a lâmina mexe ao religar o driver",
    "auto_disable_shift_deg": "desligamento automático × driver sempre ligado",
    "home_shift_deg": "ponto de trabalho antes × depois de um home()",
    "total_deg": "deriva depois de todos os blocos de movimentos",
    "change_deg": "mudança enquanto a outra lâmina gira",
    "backlash": "folga com um perfil de velocidade",
    "change": "mudança com um perfil de velocidade",
}


def extract_hysteresis_suite(test: dict, ctx: Context) -> List[Finding]:
    out = []
    for path, number in walk_numbers(test.get("metrics", {})):
        leaf = re.sub(r"\[\d+\]", "", path.split(".")[-1])
        if HYST_KEY.search(leaf) and not path.endswith("_min") and "sigma" not in path:
            out.append(ctx.from_degrees("", number, f"{HYST_LABELS.get(leaf, leaf)} [{path}]"))
    return out


# genérico: qualquer "N µp" / "N micropasso(s)" no log (menos limites e tolerâncias)
GENERIC_US = re.compile(r"([+-]?\d+(?:\.\d+)?)\s*(?:µp|micropasso)")


def extract_generic(test: dict, ctx: Context) -> List[Finding]:
    out = []
    for plate, text in log_lines(test):
        if any(word in text for word in ("dentro de", "limite", "tolerância")):
            continue
        for m in GENERIC_US.finditer(text):
            out.append(ctx.from_microsteps(plate, float(m.group(1)), text[:70]))
    return out


EXTRACTORS: Dict[str, Extractor] = {
    "edge": extract_edge,
    "revolution": extract_revolution,
    "calibrate": extract_calibrate,
    "travel-limits": extract_edge_shift,
    "endurance": extract_edge_shift,
    "speed-limit": extract_speed_limit,
    "approach-history": extract_approach_history,
}


def findings_for(suite: str, test: dict, ctx: Context) -> List[Finding]:
    if suite == "hysteresis":
        found = extract_hysteresis_suite(test, ctx)
    else:
        found = EXTRACTORS.get(test["name"], extract_generic)(test, ctx)
        if not found and test["name"] not in EXTRACTORS:
            found = extract_edge_shift(test, ctx)
    return found


# ══════════════════════════════════════════════════════════════════════════════
# Relatório
# ══════════════════════════════════════════════════════════════════════════════
FLAG_WIDTH = re.compile(r"com ~(\d+)° de largura")


def notes_for(test: dict) -> List[str]:
    notes = []
    for plate, text in log_lines(test):
        m = FLAG_WIDTH.search(text)
        if m:
            notes.append(f"{plate}: bandeira do fim de curso com ~{m.group(1)}°")
    for line in test.get("log", []):
        if line.startswith("ERRO"):
            notes.append(LINE_PREFIX.sub("", line, count=1)[:90])
            break
    return notes


def fmt(f: Finding) -> str:
    return f"{f.microsteps:+6.1f} µp ({f.degrees:+.3f}°)"


def describe(test: dict, run: Run, detail: bool) -> Tuple[List[str], Optional[Finding]]:
    ctx = Context(run)
    found = findings_for(run.suite, test, ctx)
    lines = []
    worst = max(found, key=lambda f: abs(f.microsteps)) if found else None
    status = test.get("status", "?")
    if worst is None:
        lines.append(f"{status:<6} {'—':>22}   não mediu histerese")
    else:
        who = f"{worst.plate}: " if worst.plate else ""
        lines.append(f"{status:<6} {fmt(worst):>22}   {who}{worst.what}")
        if detail:
            for f in sorted(found, key=lambda f: -abs(f.microsteps))[1:]:
                who = f"{f.plate}: " if f.plate else ""
                lines.append(f"{'':<6} {fmt(f):>22}   {who}{f.what}")
    lines += [f"{'':<6} {'':>22}   ({n})" for n in notes_for(test)]
    return lines, worst


def report_suite(suite: str, names: List[str], runs: List[Run], chosen: Optional[Run],
                 detail: bool, later: bool) -> Optional[Tuple[Finding, str, Run]]:
    print("═" * 100)
    full = chosen
    if full is None:
        complete = [r for r in runs if r.suite == suite and r.complete]
        full = complete[-1] if complete else None
    if full is None:
        print(f"Bateria {suite} ({SUITE_SOURCES[suite]}): nenhum teste completo em {RESULTS_DIR.name}/")
        return None
    spd = full.steps_per_degree()[""]
    print(f"Bateria {suite} ({SUITE_SOURCES[suite]}) — último teste completo: {full.path.parent.name}/{full.path.name}")
    print(f"  {full.date:%d/%m/%Y %H:%M} · lâminas {full.plates()} · 1 µp = {1 / spd:.4f}°")
    print(f"\n  {'teste':<18} {'estado':<6} {'histerese máxima':>22}   o quê")
    tests = full.tests
    newer = [r for r in runs if r.suite == suite and r.date > full.date and r is not full] if later else []
    worst_all: Optional[Tuple[Finding, str, Run]] = None

    def keep(f: Optional[Finding], name: str, run: Run):
        nonlocal worst_all
        if f is not None and (worst_all is None or abs(f.microsteps) > abs(worst_all[0].microsteps)):
            worst_all = (f, name, run)

    for name in names:
        lines = []
        if name in tests:
            lines, w = describe(tests[name], full, detail)
            keep(w, name, full)
        elif name in MULTI_PLATE and len(full.data.get("plates") or {}) < 2:
            lines = [f"{'—':<6} {'':>22}   precisa de 2+ lâminas (o teste completo usou só {full.plates()})"]
        else:
            lines = [f"{'—':<6} {'':>22}   não rodou no teste completo"]
        for run in newer:
            if name in run.tests:
                more, w = describe(run.tests[name], run, detail)
                keep(w, name, run)
                lines.append(f"{'':<6} {'':>22}   ↳ rodou de novo em {run.path.name} ({run.date:%d/%m %H:%M}):")
                lines += [f"{'':<6} {'':>22}     {more[0]}"] + [f"{'':<6} {'':>22}     {m.strip()}"
                                                               for m in more[1:]]
        print(f"  {name:<18} {lines[0]}")
        for extra in lines[1:]:
            print(f"  {'':<18} {extra}")

    gone = [n for n in tests if n not in names]
    if gone:
        print(f"\n  (no arquivo, mas fora de vigência hoje: {', '.join(gone)})")
    if worst_all:
        f, name, run = worst_all
        who = f"{f.plate}: " if f.plate else ""
        print(f"\n  PIOR CASO: {fmt(f).strip()} em '{name}' ({run.path.name}) — {who}{f.what}")
    return worst_all


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", type=Path, default=RESULTS_DIR, help="pasta dos resultados")
    ap.add_argument("--file", type=Path, help="analisa este arquivo em vez do último teste completo")
    ap.add_argument("--detail", action="store_true", help="mostra todas as medidas, não só a máxima")
    ap.add_argument("--no-later", dest="later", action="store_false",
                    help="não mostra testes rodados de novo depois do teste completo")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    args = parse_args(argv)
    registry = {suite: registered_tests(ROOT / src) for suite, src in SUITE_SOURCES.items()}
    runs = load_runs(args.dir, registry)

    chosen = None
    if args.file:
        chosen = next((r for r in runs if r.path.resolve() == args.file.resolve()), None)
        if chosen is None:
            print(f"{args.file}: não é um resultado válido em {args.dir}")
            return 2

    worst = []
    for suite, names in registry.items():
        if chosen is not None and chosen.suite != suite:
            continue
        w = report_suite(suite, names, runs, chosen, args.detail, args.later)
        if w:
            worst.append((suite, w))
    print("═" * 100)
    for suite, (f, name, run) in worst:
        print(f"  {suite:<11} histerese máxima {fmt(f).strip()}  ({name}, {run.path.name})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
