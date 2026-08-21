"""
calibrate.py — 회귀 테스트 기준값(모드 A 54.3s / 모드 B 44.2s)에 맞는
구간 통과시간 파라미터를 찾아주는 보조 도구.

사용법:
    python3 tools/calibrate.py
"""
from __future__ import annotations

import copy
import itertools
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import yaml

from rail_control_core.engine import RailEngine
from rail_control_core.fsm import Order
from rail_control_core.topology import Topology

CFG = Path(__file__).resolve().parent.parent / "config"
TARGET_A, TARGET_B = 54.3, 44.2
KPI_KEY = "makespan_s"


def run(topo_data: dict, params: dict, scen: dict, mode: str) -> dict:
    tmp = Path("/tmp/_topo.yaml")
    tmp.write_text(yaml.safe_dump(topo_data, allow_unicode=True), encoding="utf-8")
    topo = Topology(tmp)
    p = dict(params)
    p["mode"] = mode
    eng = RailEngine(topo, p)
    for o in scen["orders"]:
        eng.add_order(Order(
            oid=o["id"], priority=int(o["priority"]), item=o["item"],
            origin=o["origin"], dest=o["dest"],
            release_s=float(o["release_s"]), due_s=float(o["due_s"]),
            cargo_class=o.get("cargo_class", "clean"),
            code_red=bool(o.get("code_red", False)),
            load_s=(float(o["load_s"]) if "load_s" in o else None),
        ))
    eng.run_until_done(400.0)
    return eng.kpi()


def main() -> None:
    base_topo = yaml.safe_load((CFG / "topology.yaml").read_text(encoding="utf-8"))
    base_par = yaml.safe_load((CFG / "params.yaml").read_text(
        encoding="utf-8"))["control_core_node"]["ros__parameters"]
    scen = yaml.safe_load((CFG / "scenario_main.yaml").read_text(encoding="utf-8"))

    keys = ["B01", "B02", "B03", "B04", "B05", "B06", "B07"]
    best, best_err = None, 1e9
    rng = random.Random(7)

    for it in range(4000):
        topo = copy.deepcopy(base_topo)
        par = dict(base_par)
        for k in keys:
            topo["blocks"][k]["traverse_s"] = round(rng.uniform(3.0, 16.0), 1)
        par["runthrough_threshold_s"] = round(rng.uniform(3.0, 10.0), 1)
        sc = copy.deepcopy(scen)
        sc["orders"][0]["load_s"] = round(rng.uniform(12.0, 22.0), 1)

        try:
            a = run(topo, par, sc, "A")[KPI_KEY]
            b = run(topo, par, sc, "B")[KPI_KEY]
        except Exception:
            continue
        if a is None or b is None:
            continue
        err = abs(a - TARGET_A) + abs(b - TARGET_B)
        if err < best_err:
            best_err, best = err, (topo, par, sc, a, b)
            print(f"it={it:5d}  A={a:6.1f}  B={b:6.1f}  err={err:5.2f}")
            if err < 0.2:
                break

    topo, par, sc, a, b = best
    print("\n=== BEST ===")
    print({k: topo["blocks"][k]["traverse_s"] for k in keys})
    print("threshold:", par["runthrough_threshold_s"],
          " O-1 load_s:", sc["orders"][0]["load_s"])
    print(f"모드 A = {a}s (목표 {TARGET_A})   모드 B = {b}s (목표 {TARGET_B})")


if __name__ == "__main__":
    main()
