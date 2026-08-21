"""
kpi_report.py — 기준 KPI 후보별 모드 A/B 수치를 한 번에 뽑아 비교표로 출력합니다.

기준 KPI가 팀 미확정 상태이므로, 이 표를 팀 회의에 그대로 가져가서
"이 중 무엇을 기준으로 할까요"를 물으면 됩니다.

사용법:
    python3 tools/kpi_report.py
    python3 tools/kpi_report.py --md      # 마크다운 표로 출력
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rail_control_core.engine import build_engine  # noqa: E402


def collect(mode: str) -> dict:
    eng = build_engine(mode)
    eng.run_until_done(400.0)
    k = eng.kpi()
    p0 = next(o for o in eng.orders.values() if o.priority == 0)
    return {
        "p0_finish_s": round(p0.finish_s, 1),
        "p0_lead_time_s": round(p0.lead_time_s, 1),
        "makespan_s": k["makespan_s"],
        "avg_lead_time_s": k["avg_lead_time_s"],
        "overdue_count": len(k["overdue_orders"]),
        "total_wait_s": k["total_wait_s"],
    }


ROWS = [
    ("p0_finish_s",     "P0 인계 완료 시각",     "T+초", True),
    ("p0_lead_time_s",  "P0 리드타임",           "초",   True),
    ("makespan_s",      "전체 완료 시각",        "T+초", True),
    ("avg_lead_time_s", "평균 리드타임",         "초",   True),
    ("overdue_count",   "마감 초과 오더 수",     "건",   True),
    ("total_wait_s",    "총 대기시간(지연 비용)", "초",  False),
]

TARGET_A, TARGET_B = 54.3, 44.2   # 발표자료에 적힌 값 (정의 불명)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true", help="마크다운 표로 출력")
    args = ap.parse_args()

    a, b = collect("A"), collect("B")

    lines = []
    if args.md:
        lines.append("| 후보 지표 | 단위 | 모드 A | 모드 B | 차이 | 발표값 부합 |")
        lines.append("|---|---|---|---|---|---|")
    else:
        lines.append(f"{'후보 지표':<22}{'단위':<7}{'모드 A':>9}{'모드 B':>9}"
                     f"{'차이':>9}  발표값(54.3/44.2) 부합")
        lines.append("-" * 88)

    for key, label, unit, lower_better in ROWS:
        va, vb = a[key], b[key]
        diff = round(vb - va, 1)
        fit = (abs(va - TARGET_A) <= 0.5 and abs(vb - TARGET_B) <= 0.5)
        # 개선 방향 판정
        if lower_better:
            mark = "✅ 개선" if diff < -1 else ("— 거의 동일" if abs(diff) <= 1 else "❌ 악화")
        else:
            mark = "(비용)"
        fitmark = "★ 일치" if fit else "—"

        if args.md:
            lines.append(f"| {label} | {unit} | {va} | {vb} | {diff:+} ({mark}) | {fitmark} |")
        else:
            lines.append(f"{label:<22}{unit:<7}{va:>9}{vb:>9}{diff:>+9}  "
                         f"{mark:<12}{fitmark}")

    print("\n".join(lines))
    print()
    print("해석:")
    print("  · 선점(모드 B)은 P0 관련 지표만 개선합니다. 하위 오더를 뒤로 미루기 때문에")
    print("    makespan·평균 리드타임은 거의 그대로이거나 오히려 나빠질 수 있습니다.")
    print("  · 따라서 '선점의 효과'를 보여주는 기준 KPI는 P0 계열이어야 합니다.")
    print("  · 발표자료의 54.3 / 44.2 와 일치하는 후보에 ★ 표시가 붙습니다.")


if __name__ == "__main__":
    main()
