"""토폴로지 v3.1 (FROZEN) — docs/topology_spec.md §2~§4를 코드로 옮긴 데이터 모듈.
블록 길이는 [가정]값. B 씬 좌표 확정 시 이 파일의 숫자만 교체하고 회귀 기준값 재측정.
"""

# 블록: id -> (node_a, node_b, length_m, capacity, oneway)
# oneway=True 이면 a->b 방향으로만 주행 가능
BLOCKS = {
    # 층간
    "SB-UP":  ("N-W1", "N-W2", 4.0, 4, True),
    "SB-DN":  ("N-W2", "N-W1", 4.0, 4, True),
    # B1F
    "BB-01":  ("N-W1", "N-S1", 2.5, 1, False),
    "BB-02":  ("N-S1", "N-S2", 2.9, 1, False),
    "BB-03":  ("N-S2", "N-L1", 1.1, 1, False),
    "BB-04a": ("N-L1", "N-L2", 3.1, 1, False),
    "BB-04b": ("N-L1", "N-L2", 3.1, 3, False),   # 루프 A 대피 레인
    "BB-05":  ("N-L2", "N-E1", 0.7, 1, False),
    "BB-06a": ("N-E1", "N-L3", 1.1, 1, False),
    "BB-06b": ("N-L3", "N-L4", 3.4, 1, False),
    "BB-06c": ("N-L3", "N-L4", 3.4, 3, False),   # 루프 B 대피 레인
    "BB-06d": ("N-L4", "N-D1", 1.7, 1, False),
    "BB-07":  ("N-D1", "N-D2", 10.0, 10, False), # 디포 충전 존(추상화)
    "BB-08":  ("N-D2", "N-B1", 5.7, 4, True),    # 출동 대기열(단방향)
    "BB-09":  ("N-B1", "N-E1", 6.8, 4, False),   # 복귀선 겸 콘보이 출동 램프 (v3.1 확정: 용량 1->4, 팀 합의 2026-08-21)
    "SP-INJ": ("N-S1", "ST-INJ", 3.3, 1, False),
    "SP-PHM": ("N-S2", "ST-PHM", 3.3, 1, False),
    # 2F
    "B2-01":  ("N-W2", "N-S3", 2.5, 1, False),
    "B2-02":  ("N-S3", "N-S4", 2.9, 1, False),
    "B2-03":  ("N-S4", "N-L5", 1.1, 1, False),
    "B2-04a": ("N-L5", "N-L6", 3.1, 1, False),
    "B2-04b": ("N-L5", "N-L6", 3.1, 3, False),   # 루프 A2 대피 레인
    "B2-05":  ("N-L6", "N-E2", 0.7, 1, False),
    "B2-06":  ("N-E2", "N-L7", 1.1, 1, False),
    "B2-07a": ("N-L7", "N-L8", 3.4, 1, False),
    "B2-07b": ("N-L7", "N-L8", 3.4, 3, False),   # 루프 B2 대피 레인
    "B2-08":  ("N-L8", "ST-OR1", 3.6, 1, False),
    "B2-09":  ("N-E2", "ST-OR2", 3.7, 1, False),
    "SP-CSR": ("N-S3", "ST-CSR", 3.3, 1, False),
    "SP-ICU": ("N-S4", "ST-ICU", 3.3, 1, False),
}

# 회랑(방향 토큰 단위): 교행 가능 지점 사이의 양방향 구간 묶음 (R8)
CORRIDORS = {
    "C1": ["BB-01", "BB-02", "BB-03"],
    "C2": ["BB-05", "BB-06a"],
    "C3": ["BB-06d"],
    "C4": ["BB-09"],
    "C5": ["B2-01", "B2-02", "B2-03"],
    "C6": ["B2-05", "B2-06"],
}
BLOCK_TO_CORRIDOR = {b: c for c, bs in CORRIDORS.items() for b in bs}

# 대피 레인: 직선 레인 <-> 대피 레인 상호 매핑 (R3)
ESCAPE_LANE = {
    "BB-04a": "BB-04b", "BB-06b": "BB-06c",
    "B2-04a": "B2-04b", "B2-07a": "B2-07b",
}

# 경로 표기: (block_id, forward) — forward=True 는 node_a->node_b 방향
ROUTES = {
    # Code Crimson 콘보이: 대기열 -> 수술실 (OR1행 / OR2행)
    "P0_OR1": [("BB-09", True), ("BB-05", False), ("BB-04a", False), ("BB-03", False),
               ("BB-02", False), ("BB-01", False), ("SB-UP", True),
               ("B2-01", True), ("B2-02", True), ("B2-03", True), ("B2-04a", True),
               ("B2-05", True), ("B2-06", True), ("B2-07a", True), ("B2-08", True)],
    "P0_OR2": [("BB-09", True), ("BB-05", False), ("BB-04a", False), ("BB-03", False),
               ("BB-02", False), ("BB-01", False), ("SB-UP", True),
               ("B2-01", True), ("B2-02", True), ("B2-03", True), ("B2-04a", True),
               ("B2-05", True), ("B2-09", True)],
    # P1 응급약품: 약제부 -> ICU
    "P1_ICU": [("SP-PHM", False), ("BB-02", False), ("BB-01", False), ("SB-UP", True),
               ("B2-01", True), ("B2-02", True), ("SP-ICU", True)],
    # P2 항암제: 주사조제실 -> ICU
    "P2_ICU": [("SP-INJ", False), ("BB-01", False), ("SB-UP", True),
               ("B2-01", True), ("B2-02", True), ("SP-ICU", True)],
    # P3 오염 기구 회수: 수술실1 -> CSR
    "P3_CSR": [("B2-08", False), ("B2-07a", False), ("B2-06", False), ("B2-05", False),
               ("B2-04a", False), ("B2-03", False), ("B2-02", False), ("SP-CSR", True)],
}


def entry_node(block_id: str, forward: bool) -> str:
    a, b, *_ = BLOCKS[block_id]
    return a if forward else b


def exit_node(block_id: str, forward: bool) -> str:
    a, b, *_ = BLOCKS[block_id]
    return b if forward else a


def validate() -> list[str]:
    """경로 무결성 자체 검증: 인접성 + 단방향 역주행 여부 (명세 §4 검증의 자동화)."""
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
    return errors
