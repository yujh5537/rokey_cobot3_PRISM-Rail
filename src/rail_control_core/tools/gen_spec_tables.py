#!/usr/bin/env python3
"""tools/gen_spec_tables.py — 명세서 §3 블록 표를 topology.py 에서 자동 생성.

사용법 (레포 루트에서):
    python3 src/rail_control_core/tools/gen_spec_tables.py > /tmp/sec3.md
    # docs/topology_spec.md 의 §3 절 전체를 출력 내용으로 교체

원칙: 코드(topology.py)가 단일 진실 소스. 표를 손으로 고치지 말 것.
배경: v3.1~v3.3 동안 수치가 5회 재측정되며 표-코드 불일치가 3회 발생 —
      부분 수기 동기화의 구조적 한계로 판단, 생성 방식으로 전환 (2026-08-23).
비고(NOTE)만 이 파일에서 관리한다 (설명 문구는 코드에 없는 정보이므로).
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from rail_control_core import topology as T  # noqa: E402

NOTE = {
    "SB-UP": "상행 쉬프트 (x=-7.25, 0.5 m/s 고정), 차두간격 파이프라인",
    "SB-DN": "하행 쉬프트 (x=-6.75, 0.5 m/s 고정)",
    "BB-01": "본선 (곡선 실측 — 코너 존 감속 2.07~2.87m)",
    "BB-04b": "루프 A 대피 레인 (usable 3.0m→용량 3)",
    "BB-05": "분기 연결 — 전장 0.5 m/s",
    "BB-06c": "루프 B 대피 레인 (usable 3.0)",
    "BB-06d": "분기 연결 — 전장 0.5 m/s",
    "BB-07": "충전 존 통과부. 도크 10기는 측면 슬롯 정본 (5.8/4.9/4.0열, geometry.DOCK_SLOTS)",
    "BB-08": "출동 대기열 (L자, 경유점 (6.5, 5.0))",
    "BB-09": "복귀선 겸 출동 램프 (L자, 경유점 (3.2, -4.0), v3.1 용량 4 — "
             "역방향=혈액 입고 진입, 적재 4×0.9m=3.6 ≤ 5.3 OK)",
    "B2-01": "본선 (B1F 대칭 확정, B 회신 2026-08-23)",
    "B2-04b": "루프 A2 대피 레인 — **시연 장면 ③ 무대 (EVAC_LANE 치환 진입)**",
    "B2-05": "L자 (경유점 (3.2, -2.0), §10)",
    "B2-07b": "루프 B2 대피 레인",
    "B2-08": "수술실1 진입 (L자, visual y=5.0은 렌더 전용)",
    "B2-09": "수술실2 지선 (endpoint 실측 확정)",
}


def dirmark(bid, oneway):
    if not oneway:
        return "양(후진)" if bid.startswith("SP") or bid in ("B2-08", "B2-09") else "양"
    return "**단방향 ↑**" if bid == "SB-UP" else \
           ("**단방향 ↓**" if bid in ("SB-DN", "BB-08") else "**단방향**")


def table(ids):
    rows = ["| ID | 구간 | 방향 | 용량 | 길이(m) | 비고 |", "|---|---|---|---|---|---|"]
    for bid in ids:
        a, b, L, cap, ow = T.BLOCKS[bid]
        arrow = "→" if ow else "↔"
        note = NOTE.get(bid, "본선" if not bid.startswith("SP") else "지선")
        rows.append(f"| {bid} | {a} {arrow} {b} | {dirmark(bid, ow)} | {cap} | {L} | {note} |")
    return "\n".join(rows)


def main():
    b1 = [k for k in T.BLOCKS if k.startswith(("BB", "SP-INJ", "SP-PHM"))]
    f2 = [k for k in T.BLOCKS if k.startswith(("B2", "SP-CSR", "SP-ICU"))]
    total = 2 + len(b1) + len(f2)
    print(f"## 3. 블록 정의 (총 {total}개 — `topology.py` 자동 생성, 수기 편집 금지)\n")
    print("> ⚙️ 이 표는 `python3 src/rail_control_core/tools/gen_spec_tables.py` 로 생성됩니다.")
    print("> 길이·용량을 바꾸려면 **코드를 고치고 표를 재생성**하십시오. 손으로 표를 고치지 마십시오.\n")
    print("### 3-1. 층간 (2)\n")
    print(table(["SB-UP", "SB-DN"]))
    print(f"\n### 3-2. B1F ({len(b1)}개)\n")
    print(table(b1))
    print(f"\n### 3-3. 2F ({len(f2)}개)\n")
    print(table(f2))


if __name__ == "__main__":
    main()
