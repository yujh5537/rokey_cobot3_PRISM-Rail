"""토폴로지 v3.3 (FROZEN) — 씬 실측 좌표 2차 핸드오프 반영판 (2026-08-23, B ACTION_FOR_A §22 기준).
확정 (2026-08-23 B 회신 2건) — 좌표·길이 미결 0:
  - B1 N-L1=-1.5 / N-L2=2.5 (§14·§22는 stale), ST-OR2=(4.5,-0.8,13.0) 실측 일치
  - B2-01=4.53 : B1F 복제 구조(2F = B1F +9.0m Z) 대칭 확정
  - 디포 도크 : 씬에 D01~D10 개별 프림 없음 -> 아래 DOCK_SLOTS 가 정본 (B 가 이 좌표에 시각물 추후 배치)
  - BB-01/B2-01 곡선 웨이포인트 : 선택 사항으로 종결 (관제는 길이만 사용, 시각화는 근사 유지.
    정밀화가 필요해지면 USD BasisCurve 를 덤프해 BLOCK_PATHS 만 교체하면 됨)
BLOCK_PATHS: 시각화/xyz 변환용 폴리라인 (NORMALIZED = 실측 길이로 정규화되는 근사 경로).
"""

RAIL_Z = {"B1F": 4.0, "2F": 13.0}
SHAFT_X = {"SB-UP": -7.25, "SB-DN": -6.75}

# 블록: id -> (node_a, node_b, length_m, capacity, oneway)
BLOCKS = {
    "SB-UP":  ("N-W1", "N-W2", 9.0, 4, True),
    "SB-DN":  ("N-W2", "N-W1", 9.0, 4, True),
    # B1F
    "BB-01":  ("N-W1", "N-S1", 4.53, 1, False),
    "BB-02":  ("N-S1", "N-S2", 1.0, 1, False),
    "BB-03":  ("N-S2", "N-L1", 2.0, 1, False),
    "BB-04a": ("N-L1", "N-L2", 4.0, 1, False),
    "BB-04b": ("N-L1", "N-L2", 4.0, 3, False),   # 대피 레인 (usable 3.0 -> 용량 3)
    "BB-05":  ("N-L2", "N-E1", 0.7, 1, False),
    "BB-06a": ("N-E1", "N-L3", 1.8, 1, False),
    "BB-06b": ("N-L3", "N-L4", 4.0, 1, False),
    "BB-06c": ("N-L3", "N-L4", 4.0, 3, False),   # 대피 레인
    "BB-06d": ("N-L4", "N-D1", 0.4, 1, False),
    "BB-07":  ("N-D1", "N-D2", 0.8, 10, False),  # 디포 통과부 (도크 10기는 측면 베이 추상화)
    "BB-08":  ("N-D2", "N-B1", 12.3, 4, True),   # 출동 대기열 (L자: 동->남)
    "BB-09":  ("N-B1", "N-E1", 5.3, 4, False),   # 복귀선/출동램프 (L자: 서->북)
    "SP-INJ": ("N-S1", "ST-INJ", 6.0, 1, False),
    "SP-PHM": ("N-S2", "ST-PHM", 6.0, 1, False),
    # 2F
    "B2-01":  ("N-W2", "N-S3", 4.53, 1, False),  # B1F 대칭 확정 (B 회신 2026-08-23)
    "B2-02":  ("N-S3", "N-S4", 1.0, 1, False),
    "B2-03":  ("N-S4", "N-L5", 2.0, 1, False),
    "B2-04a": ("N-L5", "N-L6", 4.0, 1, False),
    "B2-04b": ("N-L5", "N-L6", 4.0, 3, False),   # 대피 레인
    "B2-05":  ("N-L6", "N-E2", 1.9, 1, False),  # L자(§10: 동->북, 코너 라운드)
    "B2-06":  ("N-E2", "N-L7", 0.8, 1, False),
    "B2-07a": ("N-L7", "N-L8", 4.0, 1, False),
    "B2-07b": ("N-L7", "N-L8", 4.0, 3, False),   # 대피 레인
    "B2-08":  ("N-L8", "ST-OR1", 1.8, 1, False),  # L자(북0.5+동1.3), visual y=5.0은 렌더 전용(§8)
    "B2-09":  ("N-E2", "ST-OR2", 1.3, 1, False),
    "SP-CSR": ("N-S3", "ST-CSR", 6.0, 1, False),
    "SP-ICU": ("N-S4", "ST-ICU", 6.0, 1, False),
}

# 노드/스테이션 XY (Z는 층 레일 기준: B1F 4.0 / 2F 13.0)
NODE_XY = {
    "N-W1": (-7.0, -5.0), "N-S1": (-4.5, -2.0), "N-S2": (-3.5, -2.0),
    "N-L1": (-1.5, -2.0), "N-L2": (2.5, -2.0), "N-E1": (3.2, -2.0),
    "N-L3": (3.2, -0.2), "N-L4": (3.2, 3.8), "N-D1": (3.2, 4.2),
    "N-D2": (3.2, 5.0), "N-B1": (6.5, -4.0),
    "ST-INJ": (-4.5, 4.0), "ST-PHM": (-3.5, 4.0),
    "N-W2": (-7.0, -5.0), "N-S3": (-4.5, -2.0), "N-S4": (-3.5, -2.0),
    "N-L5": (-1.5, -2.0), "N-L6": (2.5, -2.0), "N-E2": (3.2, -0.8),
    "N-L7": (3.2, 0.0), "N-L8": (3.2, 4.0),
    "ST-CSR": (-4.5, 4.0), "ST-ICU": (-3.5, 4.0),
    "ST-OR1": (4.5, 4.5), "ST-OR2": (4.5, -0.8),
}

FLOOR_2F = {"N-W2", "N-S3", "N-S4", "N-L5", "N-L6", "N-E2", "N-L7", "N-L8",
            "ST-CSR", "ST-ICU", "ST-OR1", "ST-OR2"}

# 블록 폴리라인 (a->b 순서 XY 웨이포인트). 미기재 블록은 노드 직선.
# NORMALIZED: 폴리라인 실길이와 실측 길이가 달라 비율 매핑하는 근사 경로(시각화용).
BLOCK_PATHS = {
    "BB-01":  [(-7.0, -5.0), (-7.0, -2.0), (-4.5, -2.0)],       # NORMALIZED (4.53m 곡선 근사)
    "B2-01":  [(-7.0, -5.0), (-7.0, -2.0), (-4.5, -2.0)],       # NORMALIZED
    "BB-08":  [(3.2, 5.0), (6.5, 5.0), (6.5, -4.0)],             # L자 12.3m 정합
    "BB-09":  [(6.5, -4.0), (3.2, -4.0), (3.2, -2.0)],           # L자 5.3m 정합
    "BB-04b": [(-1.5, -2.0), (-1.2, -1.4), (2.2, -1.4), (2.5, -2.0)],   # NORMALIZED 대피 레인
    "BB-06c": [(3.2, -0.2), (3.8, 0.1), (3.8, 3.5), (3.2, 3.8)],        # NORMALIZED
    "B2-04b": [(-1.5, -2.0), (-1.2, -1.4), (2.2, -1.4), (2.5, -2.0)],   # NORMALIZED (v3.3 노드)
    "B2-07b": [(3.2, 0.0), (3.8, 0.3), (3.8, 3.7), (3.2, 4.0)],         # NORMALIZED (v3.3 노드)
    "B2-05":  [(2.5, -2.0), (3.2, -2.0), (3.2, -0.8)],           # L자 1.9m 정합 (§10)
    "B2-08":  [(3.2, 4.0), (3.2, 4.5), (4.5, 4.5)],              # L자 1.8m 정합 (§8 logical)
}
PATH_NORMALIZED = {"BB-01", "B2-01", "BB-04b", "BB-06c", "B2-04b", "B2-07b"}

# 디포 도크 슬롯 — 관제 정의가 정본 (B 확인 2026-08-23: 씬에 도크 프림 없음, 이 좌표에 시각물 배치 예정)
# 시연 사용분은 뒤 2자리 (5.8, 3.7) / (5.8, 2.8) — C09·C10 (v3.6: C08 은 ST-OR1 선배치)
DOCK_SLOTS = [(4.0, 4.6), (4.0, 3.7), (4.0, 2.8), (4.9, 4.6), (4.9, 3.7),
              (4.9, 2.8), (5.8, 4.6), (5.8, 3.7), (5.8, 2.8), (5.8, 1.9)]

# 곡선·분기 감속 (A1: 곡선 및 분기 통과 0.5 m/s)
JUNCTION_BLOCKS = {"BB-05", "BB-06d"}      # 분기 연결 블록: 전장 감속

def _build_curve_zones(margin: float = 0.4):
    """폴리라인 내부 꺾임점 주변 ±margin[m] 구간(진입점 기준 arc)을 감속 존으로 산출.
    NORMALIZED 블록은 폴리라인 길이를 실측 길이로 비례 환산."""
    zones = {}
    for bid, pts in BLOCK_PATHS.items():
        if len(pts) < 3:
            continue
        L = BLOCKS[bid][2]
        seglens = [((x2-x1)**2 + (y2-y1)**2) ** 0.5 for (x1,y1),(x2,y2) in zip(pts, pts[1:])]
        total = sum(seglens)
        acc, zs = 0.0, []
        for sl in seglens[:-1]:
            acc += sl
            arc = acc / total * L
            zs.append((max(0.0, arc - margin), min(L, arc + margin)))
        zones[bid] = tuple(zs)
    return zones

CURVE_ZONES = _build_curve_zones()

# 회랑(방향 토큰 단위) — 변경 없음
CORRIDORS = {
    "C1": ["BB-01", "BB-02", "BB-03"],
    "C2": ["BB-05", "BB-06a"],
    "C3": ["BB-06d"],
    "C4": ["BB-09"],
    "C5": ["B2-01", "B2-02", "B2-03"],
    "C6": ["B2-05", "B2-06"],
}
BLOCK_TO_CORRIDOR = {b: c for c, bs in CORRIDORS.items() for b in bs}

ESCAPE_LANE = {
    "BB-04a": "BB-04b", "BB-06b": "BB-06c",
    "B2-04a": "B2-04b", "B2-07a": "B2-07b",
}

ROUTES = {
    "P0": [("BB-09", True), ("BB-05", False), ("BB-04a", False), ("BB-03", False),
           ("BB-02", False), ("BB-01", False), ("SB-UP", True),
           ("B2-01", True), ("B2-02", True), ("B2-03", True), ("B2-04a", True),
           ("B2-05", True), ("B2-09", True)],  # ◀ ST-OR2 단일 종점
    "P0_OR2": [("BB-09", True), ("BB-05", False), ("BB-04a", False), ("BB-03", False),
               ("BB-02", False), ("BB-01", False), ("SB-UP", True),
               ("B2-01", True), ("B2-02", True), ("B2-03", True), ("B2-04a", True),
               ("B2-05", True), ("B2-09", True)],
    "P1_ICU": [("SP-PHM", False), ("BB-02", False), ("BB-01", False), ("SB-UP", True),
               ("B2-01", True), ("B2-02", True), ("SP-ICU", True)],
    "P2_ICU": [("SP-INJ", False), ("BB-01", False), ("SB-UP", True),
               ("B2-01", True), ("B2-02", True), ("SP-ICU", True)],
    "X_ICU_PHM": [("SP-ICU", False), ("B2-02", False), ("B2-01", False), ("SB-DN", True),
                  ("BB-01", True), ("BB-02", True), ("SP-PHM", True)],
    # P3_CSR: v3.6부터 '사용 기구 회수'(O-5) 경로로 재배정 — 공급(P3_OR1)과 짝 흐름
    "P3_CSR": [("B2-08", False), ("B2-07a", False), ("B2-06", False), ("B2-05", False),
               ("B2-04a", False), ("B2-03", False), ("B2-02", False), ("SP-CSR", True)],
}


def entry_node(block_id: str, forward: bool) -> str:
    a, b, *_ = BLOCKS[block_id]
    return a if forward else b


def exit_node(block_id: str, forward: bool) -> str:
    a, b, *_ = BLOCKS[block_id]
    return b if forward else a


def _polyline_len(pts) -> float:
    return sum(((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
               for (x1, y1), (x2, y2) in zip(pts, pts[1:]))


def validate() -> list[str]:
    """경로 무결성 + 기하-길이 정합 자체 검증."""
    errors = []
    for name, route in ROUTES.items():
        for i, (bid, fwd) in enumerate(route):
            a, b, _, _, oneway = BLOCKS[bid]
            if oneway and not fwd:
                errors.append(f"{name}: {bid} 단방향 역주행")
            if i > 0:
                prev_bid, prev_fwd = route[i - 1]
                if exit_node(prev_bid, prev_fwd) != entry_node(bid, fwd):
                    errors.append(f"{name}: {route[i-1]} -> {route[i]} 불연속")
    for bid, (a, b, length, _, _) in BLOCKS.items():
        if bid.startswith("SB"):
            continue
        pts = BLOCK_PATHS.get(bid, [NODE_XY[a], NODE_XY[b]])
        plen = _polyline_len(pts)
        if bid not in PATH_NORMALIZED and abs(plen - length) > 0.05:
            errors.append(f"{bid}: 폴리라인 {plen:.2f}m != 실측 {length}m")
        if pts[0] != NODE_XY[a] or pts[-1] != NODE_XY[b]:
            errors.append(f"{bid}: 폴리라인 끝점이 노드 좌표와 불일치")
    return errors
