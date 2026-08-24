"""
kpi_report.py — 모드 A/B 비교표를 뽑아 명세서 §6-2 회귀 기준값과 대조하고,
명세서 §3 블록 표가 topology.py 에서 재생성한 결과와 일치하는지, 그리고
config/params.yaml 의 engine 블록이 scenario.PARAMS 와 같은지 검사합니다.

기준 KPI 는 토폴로지 v3.1 §6-2 에서 'P0 혈액 도착 시각'으로 동결되었습니다.
이 도구는 그 표를 그대로 재생성하고, 현재 코드가 기준값에서 벗어났는지 표시합니다.
(엄격한 검증은 test/test_regression.py 가 합니다. 여기는 발표용 표 출력이 목적입니다.)

사용법:
    python3 tools/kpi_report.py
    python3 tools/kpi_report.py --md      # 마크다운 표로 출력 (발표자료에 붙여넣기용)
"""

from __future__ import annotations

import argparse
import ast
import contextlib
import io
import sys
import unicodedata
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SPEC = ROOT.parent.parent / "docs" / "topology_spec.md"
sys.path.insert(0, str(ROOT))

from rail_control_core.scenario import ORDER_DEFS, run  # noqa: E402

import gen_spec_tables  # noqa: E402  (같은 tools/ 디렉터리)


def _check_spec_section3() -> bool:
    """명세서 §3 이 topology.py 재생성 결과와 같은지 검사 (수기 편집 탐지).

    §3 은 tools/gen_spec_tables.py 가 생성합니다. 누군가 표를 손으로 고치면
    코드와 갈라지므로(v3.1~v3.3 에서 8칸 불일치 발생), 여기서 diff 0 을 강제합니다.
    반환 False = 불일치 (호출자가 exit 1).
    """
    if not SPEC.exists():
        print(f"§3 검사 건너뜀 — {SPEC} 없음")
        return True
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        gen_spec_tables.main()
    regen = buf.getvalue().rstrip()

    lines = SPEC.read_text(encoding="utf-8").split("\n")
    try:
        i3 = next(i for i, l in enumerate(lines) if l.startswith("## 3. 블록 정의"))
        i4 = next(i for i, l in enumerate(lines) if l.startswith("## 4. "))
    except StopIteration:
        print("⚠️ 명세서에서 §3~§4 경계를 찾지 못했습니다 — 절 제목을 확인하세요.")
        return False
    current = "\n".join(lines[i3:i4]).rstrip()

    if current == regen:
        print("§3 블록 표: ✅ topology.py 와 일치 (30/30, 수기 편집 없음)")
        return True

    import difflib
    print("⚠️ §3 블록 표가 topology.py 와 다릅니다 — 표를 손으로 고치지 마십시오.")
    print("   해결: python3 src/rail_control_core/tools/gen_spec_tables.py 로 재생성해 §3 을 교체")
    print("   (설명 문구를 바꾸려면 gen_spec_tables.py 의 NOTE 딕셔너리를 고칠 것)")
    for line in list(difflib.unified_diff(
            current.split("\n"), regen.split("\n"),
            fromfile="docs/topology_spec.md §3", tofile="재생성", lineterm=""))[:20]:
        print("   " + line)
    return False


def _check_params_yaml() -> bool:
    """config/params.yaml 의 engine 블록이 scenario.PARAMS 와 같은지 검사.

    노드는 기동 시 이 yaml 로 scenario.PARAMS 를 덮어씁니다. 그래서 한쪽만 고치면
    **테스트는 통과하는데 ROS 시연에서만 옛 값이 쓰이는** 상태가 됩니다 — 테스트가
    거짓 안심을 주는 부류라 여기서 구조적으로 막습니다 (§3 표 검사와 같은 취지).
    v3.5 에서 실제로 priority_due_sec[2] 가 85 로 남아 있었습니다.
    반환 False = 불일치 (호출자가 exit 1).
    """
    path = ROOT / "config" / "params.yaml"
    if not path.exists():
        print(f"params.yaml 검사 건너뜀 — {path} 없음")
        return True
    try:
        import yaml
    except ImportError:
        print("params.yaml 검사 건너뜀 — pyyaml 미설치 (pip3 install pyyaml)")
        return True
    from rail_control_core.scenario import PARAMS

    eng = (yaml.safe_load(path.read_text(encoding="utf-8")) or {}).get("engine") or {}
    unknown = sorted(set(eng) - set(PARAMS))
    diff = [(k, eng[k], PARAMS[k]) for k in eng if k in PARAMS and eng[k] != PARAMS[k]]
    if not diff and not unknown:
        print(f"params.yaml: ✅ scenario.PARAMS 와 일치 ({len(eng)}키, 런타임 오버라이드 안전)")
        return True

    print("⚠️ config/params.yaml 이 scenario.PARAMS 와 다릅니다 — 노드 기동 시 yaml 이 이깁니다.")
    print("   (테스트는 통과하지만 ROS 시연에서만 옛 값이 쓰이는 상태입니다)")
    for k, yv, pv in diff:
        print(f"   {k}: params.yaml={yv!r} vs scenario.PARAMS={pv!r}")
    for k in unknown:
        print(f"   {k}: params.yaml 에만 있는 키 — 엔진이 무시합니다 (오타 의심)")
    return False


def _load_baselines():
    """기준값은 test/test_regression.py 를 단일 출처로 삼습니다.
    (테스트와 이 도구로 기준값이 갈라지지 않게 하려는 것)

    임포트 대신 AST 로 리터럴만 뽑습니다. 임포트를 쓰면 __pycache__ 가
    소스 mtime 을 '초 단위 정수'로만 비교하기 때문에, 같은 1초 안에 파일을
    고쳤다 되돌리면 크기까지 같아져 낡은 .pyc 가 계속 로드됩니다.
    """
    path = ROOT / "test" / "test_regression.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    found = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id in ("BASE_A", "BASE_B", "TOL"):
                found[target.id] = ast.literal_eval(node.value)
    missing = {"BASE_A", "BASE_B", "TOL"} - set(found)
    if missing:
        sys.exit(f"{path} 에서 {sorted(missing)} 를 찾지 못했습니다 — 기준값 정의를 확인하세요.")
    return found["BASE_A"], found["BASE_B"], found["TOL"]


def _w(s: str) -> int:
    """한글·기호의 표시 폭(전각 2칸)을 센다 — 터미널 표 정렬용."""
    return sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in str(s))


def _pad(s, width: int, right: bool = False) -> str:
    gap = " " * max(0, width - _w(s))
    return gap + str(s) if right else str(s) + gap


PRIO = {oid: prio for oid, prio, *_ in ORDER_DEFS}
LABEL = {"O-1": "P3 오염기구", "O-2": "P2 항암제",
         "O-3": "P1 응급약품", "O-4": "P0 혈액(콘보이)"}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true", help="마크다운 표로 출력")
    args = ap.parse_args()

    base_a, base_b, tol = _load_baselines()
    a, b = run("A"), run("B")

    rows = []
    drift = []
    for oid in sorted(PRIO, key=lambda o: PRIO[o]):
        va = a["orders"][oid]["arrive"]
        vb = b["orders"][oid]["arrive"]
        diff = round(vb - va, 2)
        pct = f"{diff / va * 100:+.1f}%" if va else "—"
        ok = (abs(va - base_a[oid]) <= tol and abs(vb - base_b[oid]) <= tol)
        if not ok:
            drift.append((oid, va, base_a[oid], vb, base_b[oid]))
        rows.append((LABEL[oid], va, vb, diff, pct, "✅" if ok else "⚠️ 이탈"))

    ms_a, ms_b = a["makespan"], b["makespan"]

    if args.md:
        out = ["| 오더 | 모드 A (FCFS) | 모드 B (PEDD) | 차이 | 기준값 |",
               "|---|---|---|---|---|"]
        out += [f"| {l} | {va} s | {vb} s | {d:+} s ({p}) | {m} |"
                for l, va, vb, d, p, m in rows]
        out.append(f"| **전체 완료(makespan)** | {ms_a} s | {ms_b} s | "
                   f"{round(ms_b - ms_a, 2):+} s | — |")
    else:
        out = [_pad("오더", 20) + _pad("모드 A", 10, True) + _pad("모드 B", 10, True)
               + _pad("차이", 20, True) + "  기준값",
               "-" * 72]
        out += [_pad(l, 20) + _pad(va, 10, True) + _pad(vb, 10, True)
                + _pad(f"{d:+} ({p})", 20, True) + "  " + m
                for l, va, vb, d, p, m in rows]
        out.append(_pad("전체 완료(makespan)", 20) + _pad(ms_a, 10, True)
                   + _pad(ms_b, 10, True)
                   + _pad(f"{round(ms_b - ms_a, 2):+}", 20, True))

    print("\n".join(out))
    print()
    print(f"도착 순서 (모드 B): {' → '.join(sorted(b['orders'], key=lambda o: b['orders'][o]['arrive']))}")
    print(f"발생 장면: {b['scene_events']}")
    print()
    print("해석:")
    print("  · 선점(모드 B)은 P0 관련 지표만 개선합니다. 하위 오더를 뒤로 미루므로")
    print("    makespan·P2·P3 는 오히려 나빠지며, 그 값이 곧 '양보 비용'입니다.")
    print("  · 그래서 §6-2 의 헤드라인 지표는 P0 도착 시각입니다.")
    print()
    spec_ok = _check_spec_section3()
    params_ok = _check_params_yaml()

    if drift:
        print()
        print("⚠️ 기준값 이탈 — 명세서 §6-2 와 test/test_regression.py 를 함께 갱신하세요:")
        for oid, va, ea, vb, eb in drift:
            print(f"   {oid}: A {va} (기준 {ea}) / B {vb} (기준 {eb})")
    if drift or not spec_ok or not params_ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
