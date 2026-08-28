# -*- coding: utf-8 -*-
"""
목적: A의 topology_spec.md §4 "표준 경로(블록 체인)"를 그대로 따라
      4개 오더의 mock 데이터를 정확하게 생성한다.

출처:
  - §4 표준경로 (출발-경유-목적지 블록 체인 전체)
  - §5 시연 발령표 (release 시각: P2=1s, P1=2s, P0=8s, P3=2s — scenario.py
    v3.6 ORDER_DEFS와 동기화, 2026-08-24 재확인)
  - config/params.yaml (speed_default=1.5m/s, speed_shaft=1.0m/s,
    P3 저속규정=0.7m/s)
  - kpi_report.py 최신 실측값 (모드A 73.70 / 모드B 94.47)

🧩 가정: 블록별 정확한 실측 길이 중 Antigravity가 확인해준 값
(BB-01=4.53, BB-03=2.0, BB-05=0.7, BB-06a=1.8, BB-08=12.3, BB-09=5.3,
B2-03=2.0, B2-05=1.9, B2-06=0.8, B2-08=1.8)은 그대로 쓰고, 나머지
(BB-02, BB-04a, B2-01/02/04a, SB-UP/DN, SP-*)는 최신 노드 좌표
기준 직선거리로 추정했다. 실제 화면(v11)은 SVG 실제 경로 길이를
브라우저가 직접 계산해서 쓰므로, 여기 pos_m 값이 정확한 미터 단위가
아니어도 화면상 진행률(ratio)은 정상적으로 매끄럽게 표시된다.
"""
import json, math

# ------------------------------------------------------------------
# 최신 노드 좌표 (topology.py v3.4 기준, Antigravity 확인값)
# ------------------------------------------------------------------
NODES = {
    "N-W1":[-7.0,-5.0], "N-S1":[-4.5,-2.0], "N-S2":[-3.5,-2.0],
    "N-L1":[-1.5,-2.0], "N-L2":[2.5,-2.0], "N-E1":[3.2,-2.0],
    "N-L3":[3.2,-0.2], "N-L4":[3.2,3.8], "N-D1":[3.2,4.2], "N-D2":[3.2,5.0],
    "N-B1":[6.5,-4.0], "ST-INJ":[-4.5,4.0], "ST-PHM":[-3.5,4.0],
    "N-W2":[-7.0,-5.0], "N-S3":[-4.5,-2.0], "N-S4":[-3.5,-2.0],
    "N-L5":[-1.5,-2.0], "N-L6":[2.5,-2.0], "N-E2":[3.2,-0.8],
    "N-L7":[3.2,0.0], "N-L8":[3.2,4.0], "ST-OR1":[4.5,4.5], "ST-OR2":[4.5,-0.8],
    "ST-CSR":[-4.5,-3.5], "ST-ICU":[-3.5,-3.5],  # 🧩 가정: 지선 끝 근사
}

def dist(a, b):
    (x1,y1),(x2,y2) = NODES[a], NODES[b]
    return round(math.hypot(x2-x1, y2-y1), 3)

# 블록별 길이: 확인된 실측값 우선, 없으면 노드 좌표 직선거리로 추정
# 🔴 30개 블록 전체 길이 — 속도 불일치 버그의 근본 원인이었던 부분.
# 이전엔 일부만 확인된 값 + 나머지는 직선거리 추정이라, 실제 SVG 렌더링
# 경로 길이와 안 맞아서 구간마다 속도가 들쭉날쭉했음.
# 지금은: Antigravity가 topology.py에서 직접 확인해준 실측값(우선) +
# 나머지는 세은님이 붙여넣은 topology_spec.md §3 표 값(명세서 공식값) 전체 사용
CONFIRMED_LEN = {
    # --- topology.py 실측 확인값 (최우선) ---
    "BB-01":4.53, "BB-03":2.0, "BB-05":0.7, "BB-06a":1.8,
    "BB-08":12.3, "BB-09":5.3, "B2-03":2.0, "B2-05":1.9,
    "B2-06":0.8, "B2-08":1.8,
    # --- §3 명세서 공식표 (그 외 전 블록) ---
    "BB-02":2.9, "BB-04a":3.1, "BB-04b":3.1,
    "BB-06b":3.4, "BB-06c":3.4, "BB-06d":1.7, "BB-07":2.0,
    "SP-INJ":3.3, "SP-PHM":3.3,
    "B2-01":2.5, "B2-02":2.9, "B2-04a":3.1, "B2-04b":3.1,
    "B2-07a":3.4, "B2-07b":3.4, "B2-09":1.3,
    "SP-CSR":3.3, "SP-ICU":3.3,
    # SB-UP/SB-DN: topology.py 실측 확인값 9.0m (B1F_z=4.0 -> 2F_z=13.0, 층간 9m).
    # 예전엔 4.0으로 잘못 들어가 있었음 — 대시보드 T_BLOCKS(9.0)와 어긋나서
    # 샤프트 진행률이 44%(≈4.0/9.0)만 차오른 채로 다음 블록으로 넘어가 버려
    # 캡슐이 샤프트 중간에서 다음 블록으로 순간이동하는 것처럼 보이는 원인이었다.
    "SB-UP":9.0, "SB-DN":9.0,
}

# 표준경로(§4)의 각 구간을 (block_id, from_node, to_node) 로 정의
# SB-UP/SB-DN/SP-*는 실제 block_id가 아니라 명세서상 별도 표기라, 이 코드에서도
# 그대로 특수 취급 (SB-UP zone='shift', SP-*는 classifyBlock에서 'main' 취급됨)
ROUTES = {
    # P0: Code Crimson 콘보이 ×4 (C01~C04, v3.5 확정: ST-OR2 전원 단일 집결)
    # R9: CC 활성 중 블록 점유 상한을 convoy_bunch_cap(=4)까지 개방
    "O-4": {
        "priority": 0, "capsules": ["C01","C02","C03","C04"], "release": 8.0,
        "item": "응급 수혈혈액",
        "chain_common": [  # OR2 진입 전 공통 구간
            ("BB-09","N-B1","N-E1"), ("BB-05","N-E1","N-L2"),
            ("BB-04a","N-L2","N-L1"), ("BB-03","N-L1","N-S2"),
            ("BB-02","N-S2","N-S1"), ("BB-01","N-S1","N-W1"),
            ("SB-UP","N-W1","N-W2"),
            ("B2-01","N-W2","N-S3"), ("B2-02","N-S3","N-S4"),
            ("B2-03","N-S4","N-L5"), ("B2-04a","N-L5","N-L6"),
            ("B2-05","N-L6","N-E2"),
        ],
        "chain_or1": [],
        "chain_or2": [ ("B2-09","N-E2","ST-OR2") ],
        # C01~C04 전원 OR2 집결 (v3.5 확정)
        "split": {"C01":"or2","C02":"or2","C03":"or2","C04":"or2"},
        # 차두 간격(초). 21893e7: 엔진이 콘보이 피치(0.9m) 면제를 되돌려 물리 최소
        # 간격을 다시 강제함. KPI 정규화(normalize_capsule_timetable) 후 실제
        # 유효 속도는 route["speed"](1.5m/s)가 아니라 scale 보정을 거친 값이라,
        # 0.9m/speed로 단순 계산하면 안 되고 정규화 후 유효속도 기준으로 역산해야
        # 정확히 0.9m가 나온다 (측정: 유효속도 0.633m/s -> 필요 headway 1.42s)
        "headway": 1.47,
        "speed": 1.5,
    },
    # P1: 응급약품 (§6-1: C07 선배치, 발령 2.0s — scenario.py v3.6 ORDER_DEFS와 동기화)
    "O-3": {
        "priority": 1, "capsules": ["C07"], "release": 2.0,
        "item": "응급약품",
        "chain_common": [
            ("SP-PHM","ST-PHM","N-S2"), ("BB-02","N-S2","N-S1"),
            ("BB-01","N-S1","N-W1"), ("SB-UP","N-W1","N-W2"),
            ("B2-01","N-W2","N-S3"), ("B2-02","N-S3","N-S4"),
            ("SP-ICU","N-S4","ST-ICU"),
        ],
        "chain_or1": [], "chain_or2": [], "split": {}, "headway": 0,
        "speed": 1.5,
    },
    # P2: 항암제·TPN (§6-1: C06 선배치, 발령 1.0s — scenario.py v3.6 ORDER_DEFS와 동기화, due 78.0s)
    "O-2": {
        "priority": 2, "capsules": ["C06"], "release": 1.0,
        "item": "항암제·TPN 조제약",
        "chain_common": [
            ("SP-INJ","ST-INJ","N-S1"), ("BB-01","N-S1","N-W1"),
            ("SB-UP","N-W1","N-W2"), ("B2-01","N-W2","N-S3"),
            ("B2-02","N-S3","N-S4"), ("SP-ICU","N-S4","ST-ICU"),
        ],
        "chain_or1": [], "chain_or2": [], "split": {}, "headway": 0,
        "speed": 1.5,
    },
    # P3: 멸균 물품 공급 CSR->OR1 (v3.6 재정의: 구 O-1 "오염 기구 회수"는 O-5로 이동.
    # §6-1: C05 선배치, 발령 2.0s — scenario.py v3.6 ORDER_DEFS와 동기화, 저속 0.7m/s.
    # topology.py ROUTES["P3_OR1"] 그대로 이식 — 콘보이와 동방향이라 무간섭 조기 완주(36.2s))
    "O-1": {
        "priority": 3, "capsules": ["C05"], "release": 2.0,
        "item": "멸균 물품 공급",
        "chain_common": [
            ("SP-CSR","ST-CSR","N-S3"), ("B2-02","N-S3","N-S4"),
            ("B2-03","N-S4","N-L5"), ("B2-04a","N-L5","N-L6"),
            ("B2-05","N-L6","N-E2"), ("B2-06","N-E2","N-L7"),
            ("B2-07a","N-L7","N-L8"), ("B2-08","N-L8","ST-OR1"),
        ],
        "chain_or1": [], "chain_or2": [], "split": {}, "headway": 0,
        "speed": 0.7,  # 멸균 물품 저속 규정
    },
    # P3: 사용 기구 회수 OR1->CSR (v3.6 신설, topology.py ROUTES["P3_CSR"] 그대로 이식.
    # §6-1: C08 신규 배정, 발령 17.0s — scenario.py v3.6 ORDER_DEFS와 동기화.
    # 구 O-1 경로를 그대로 승계 — 공급(O-1)의 역방향 짝, 콘보이와 정면 조우해 교행/대피 장면 담당)
    "O-5": {
        "priority": 3, "capsules": ["C08"], "release": 17.0,
        "item": "사용 기구 회수",
        "chain_common": [
            ("B2-08","ST-OR1","N-L8"), ("B2-07a","N-L8","N-L7"),
            ("B2-06","N-L7","N-E2"), ("B2-05","N-E2","N-L6"),
            ("B2-04a","N-L6","N-L5"), ("B2-03","N-L5","N-S4"),
            ("B2-02","N-S4","N-S3"), ("SP-CSR","N-S3","ST-CSR"),
        ],
        "chain_or1": [], "chain_or2": [], "split": {}, "headway": 0,
        "speed": 0.7,  # 사용 기구 저속 규정
    },
}

def block_len(bid, a, b):
    if bid in CONFIRMED_LEN:
        return CONFIRMED_LEN[bid]
    if bid == "SB-UP":
        return 9.0  # B1F_z=4.0 -> 2F_z=13.0, 층간 9m
    d = dist(a, b)
    return d if d > 0 else 1.0  # 최소 안전값

BLOCK_DEFS = {
    'SB-UP': ('N-W1','N-W2'),   'SB-DN': ('N-W2','N-W1'),
    'BB-01': ('N-W1','N-S1'),   'BB-02': ('N-S1','N-S2'),
    'BB-03': ('N-S2','N-L1'),   'BB-04a':('N-L1','N-L2'),
    'BB-04b':('N-L1','N-L2'),   'BB-05': ('N-L2','N-E1'),
    'BB-06a':('N-E1','N-L3'),   'BB-06b':('N-L3','N-L4'),
    'BB-06c':('N-L3','N-L4'),   'BB-06d':('N-L4','N-D1'),
    'BB-07': ('N-D1','N-D2'),   'BB-08': ('N-D2','N-B1'),
    'BB-09': ('N-B1','N-E1'),
    'SP-INJ':('N-S1','ST-INJ'), 'SP-PHM':('N-S2','ST-PHM'),
    'B2-01': ('N-W2','N-S3'),   'B2-02': ('N-S3','N-S4'),
    'B2-03': ('N-S4','N-L5'),   'B2-04a':('N-L5','N-L6'),
    'B2-04b':('N-L5','N-L6'),   'B2-05': ('N-L6','N-E2'),
    'B2-06': ('N-E2','N-L7'),   'B2-07a':('N-L7','N-L8'),
    'B2-07b':('N-L7','N-L8'),   'B2-08': ('N-L8','ST-OR1'),
    'B2-09': ('N-E2','ST-OR2'),
    'SP-CSR':('N-S3','ST-CSR'), 'SP-ICU':('N-S4','ST-ICU'),
}

def is_forward(bid, from_node, to_node):
    if bid in BLOCK_DEFS:
        return from_node == BLOCK_DEFS[bid][0]
    return True

# ------------------------------------------------------------------
# 캡슐별 시간표 계산 — 속도 기반 물리적 진행
# R9(차두 규칙): 콘보이는 같은 경로를 캡슐마다 headway(초)만큼 늦게 출발
# split: 공통 구간 이후 OR1/OR2로 갈라지는 캡슐별 분기 처리
# ------------------------------------------------------------------
def build_capsule_timetable(route, capsule_id, extra_delay):
    t = route["release"] + extra_delay
    speed = route["speed"]
    timetable = []
    full_chain = list(route["chain_common"])
    branch = route["split"].get(capsule_id)
    if branch == "or1":
        full_chain += route["chain_or1"]
    elif branch == "or2":
        full_chain += route["chain_or2"]
    for bid, a, b in full_chain:
        length = block_len(bid, a, b)
        duration = length / speed
        fwd = is_forward(bid, a, b)
        timetable.append((round(t,2), round(t+duration,2), bid, length, fwd))
        t += duration
    return timetable, t

CAPSULE_ROUTE_OF = {}   # capsule_id -> order_id (역참조용)
CAPSULE_TIMETABLES = {} # capsule_id -> (raw timetable, naive_arrive)
for oid, route in ROUTES.items():
    headway = route.get("headway", 0)
    for i, cid in enumerate(route["capsules"]):
        tt, naive_arrive = build_capsule_timetable(route, cid, i*headway)
        CAPSULE_TIMETABLES[cid] = (tt, naive_arrive)
        CAPSULE_ROUTE_OF[cid] = oid

# ------------------------------------------------------------------
# 최신 KPI 실측값 (test_regression.py BASE_A/BASE_B, v3.6.1 9번째 동결 —
# 노드 셋백 0.4m + 대피 홀드 래치 반영으로 전 시각 재측정, 2026-08-24).
# 콘보이(P0) 4대는 마지막 캡슐이 이 시각에 도착하도록 정규화.
# P0=67.1s(모드B) / P2 due 78s 기준 +1.53s 여유(물리 하한 76.03s, RTA OFF 80.00s)
# ------------------------------------------------------------------
KPI_B_ARRIVE = {"O-4":67.07, "O-3":37.60, "O-2":76.47, "O-1":36.17, "O-5":86.77}
KPI_A_ARRIVE = {"O-4":70.77, "O-3":37.60, "O-2":48.97, "O-1":36.17, "O-5":74.97}
SIM_MAKESPAN_B = 86.77
SIM_MAKESPAN_A = 74.97

ORDER_DUE = {
    "O-4": 8.0 + 30.0,    # 38.0s (P0 마감)
    "O-3": 2.0 + 40.0,    # 42.0s (P1 마감)
    "O-2": 1.0 + 77.0,    # 78.0s (P2 마감, v3.6.1 확정 — 도착 76.47s, 여유 +1.53s, 물리 하한 76.03s)
    "O-1": 2.0 + 130.0,   # 132.0s (P3 마감, 공급)
    "O-5": 17.0 + 130.0,  # 147.0s (P3 마감, 회수)
}

BLOCK_STATIC_CAPS = {
    "SB-UP": 4, "SB-DN": 4,
    "BB-01": 1, "BB-02": 1, "BB-03": 1, "BB-04a": 1, "BB-04b": 3,
    "BB-05": 1, "BB-06a": 1, "BB-06b": 1, "BB-06c": 3, "BB-06d": 1,
    "BB-07": 10, "BB-08": 4, "BB-09": 4,
    "SP-INJ": 1, "SP-PHM": 1,
    "B2-01": 1, "B2-02": 1, "B2-03": 1, "B2-04a": 1, "B2-04b": 3,
    "B2-05": 1, "B2-06": 1, "B2-07a": 1, "B2-07b": 3, "B2-08": 1, "B2-09": 1,
    "SP-CSR": 1, "SP-ICU": 1,
}

def normalize_capsule_timetable(cid, target_kpi):
    """캡슐의 물리 이동 도착시각을, 소속 오더의 실측 KPI 도착시각(콘보이는 headway 보정)에 맞춰 스케일"""
    oid = CAPSULE_ROUTE_OF[cid]
    route = ROUTES[oid]
    tt, naive_arrive = CAPSULE_TIMETABLES[cid]
    idx = route["capsules"].index(cid)
    my_release = route["release"] + idx*route.get("headway",0)
    naive_span = naive_arrive - my_release
    # 콘보이 마지막 캡슐 기준으로 KPI 도착시각을 맞추고, 앞선 캡슐은 그보다 headway만큼 일찍 도착
    last_idx = len(route["capsules"]) - 1
    target_arrive = target_kpi[oid] - (last_idx-idx)*route.get("headway",0)
    real_span = target_arrive - my_release
    if naive_span <= 0: naive_span = 1
    scale = real_span/naive_span
    new_tt = []
    for (t0,t1,bid,length,fwd) in tt:
        nt0 = round(my_release + (t0-my_release)*scale, 2)
        nt1 = round(my_release + (t1-my_release)*scale, 2)
        new_tt.append((nt0,nt1,bid,length,fwd))
    return new_tt, my_release

def build_norm_tables(target_kpi):
    norm_tt, release = {}, {}
    for cid in CAPSULE_TIMETABLES:
        tt, rel = normalize_capsule_timetable(cid, target_kpi)
        norm_tt[cid] = tt
        release[cid] = rel
    return norm_tt, release

# 모드 B(선점형) — 실측 KPI_B_ARRIVE 기준 정규화
NORM_CAPSULE_TT, CAPSULE_RELEASE = build_norm_tables(KPI_B_ARRIVE)
# 🔴 모드 A(FCFS) — 동일 경로·동일 캡슐 배정이지만 KPI_A_ARRIVE 기준으로 정규화
NORM_CAPSULE_TT_A, CAPSULE_RELEASE_A = build_norm_tables(KPI_A_ARRIVE)

def capsule_state_at(cid, t, mode="B"):
    oid = CAPSULE_ROUTE_OF[cid]
    route = ROUTES[oid]
    if mode == "A":
        tt, release, kpi = NORM_CAPSULE_TT_A[cid], CAPSULE_RELEASE_A[cid], KPI_A_ARRIVE
    else:
        tt, release, kpi = NORM_CAPSULE_TT[cid], CAPSULE_RELEASE[cid], KPI_B_ARRIVE
    if t < release:
        first_fwd = tt[0][4] if tt else True
        return {"block_id":"", "pos_m":0.0, "forward":first_fwd, "state":"QUEUED" if route["priority"]<=2 else "STANDBY"}
    for (t0, t1, bid, length, fwd) in tt:
        if t0 <= t < t1:
            ratio = (t - t0)/(t1-t0) if t1>t0 else 0
            return {"block_id":bid, "pos_m":round(ratio*length,3), "forward":fwd, "state":"MOVING"}
    # 도착 완료
    last_item = tt[-1]
    last_bid, last_len, last_fwd = last_item[2], last_item[3], last_item[4]
    
    if cid == "C05" and 40.0 <= t < 50.0:
        return {"block_id":last_bid, "pos_m":last_len, "forward":last_fwd, "state":"SERVICING"}
        
    return {"block_id":last_bid, "pos_m":last_len,
            "forward":last_fwd, "state":"UNLOADING" if t < kpi[oid]+3 else "REMOVED"}

# ------------------------------------------------------------------
# order_event 로그 — 4대 시연 장면(§6) + 발령/도착 (캡슐 ID는 §6-1 확정 배정 그대로)
# ------------------------------------------------------------------
events = []
for oid, route in ROUTES.items():
    events.append({
        "sim_t": route["release"], "kind":"order_event", "event":"ORDER_RELEASE",
        "subject": oid, "detail": f"P{route['priority']}" if route['priority']==0 else "",
        "priority": route["priority"], "item": route["item"], "state":"EN_ROUTE",
    })

# 장면① 계단식 선점: P1(O-3,C07)이 P2(O-2,C06)를 선점 -> C06이 YIELD
events.append({
    "sim_t": 7.6, "kind":"order_event", "event":"YIELD",
    "subject":"C06", "detail":"BB-01: 상위 등급 진입 전 정차", "capsule":"C06",
    "order_id":"O-2", "priority":2, "by":"O-3",
    "reason":"P1(응급약품) 진입에 앞서 정차 후 양보 (R2)",
})
# 장면② 완주 허용: SB-UP 내부 진입해 있던 하위 캡슐(C07)은 멈추지 않고 통과
events.append({
    "sim_t": 8.07, "kind":"order_event", "event":"FINISH_ALLOWED",
    "subject":"C07", "detail":"SB-UP: 이미 구간 내 진입, 완주 허용", "capsule":"C07",
    "order_id":"O-3", "priority":1, "by":"O-4",
    "reason":"역주행 불가 구간이라 완주가 전체 지연 최소화 (R4)",
})
# 콘보이 출동: C01~C04가 BB-09에서 차두 간격으로 일괄 발진 (R9)
events.append({
    "sim_t": 8.0, "kind":"order_event", "event":"CODE_CRIMSON",
    "subject":"O-4", "detail":"BB-09: C01~C04 차두 간격 파이프라인 출동 (ST-OR2 집결)", "priority":0,
    "item": ROUTES["O-4"]["item"], "state":"EN_ROUTE", "code_red": True,
})
# 복귀
events.append({
    "sim_t": 22.0, "kind":"order_event", "event":"RESUME",
    "subject":"C06", "detail":"BB-01: 양보 해제, 본선 재개", "capsule":"C06",
    "order_id":"O-2", "priority":2, "by":"O-3",
    "reason":"선점 해제 조건 충족 (R2 종료)",
})
# 장면③ 교행(R13): 공급(O-1,C05)과 회수(O-5,C08)가 동측 가지에서 정면 조우
# -> 쌍둥이 대피 레인으로 치환해 스쳐 지나감 (v3.6.1 실측 t=26.10)
events.append({
    "sim_t": 26.10, "kind":"order_event", "event":"MEET_PASS",
    "subject":"C05", "detail":"B2-07a->B2-07b", "capsule":"C05",
    "order_id":"O-1", "priority":3, "by":"C08",
    "reason":"대향 캡슐과 교행 — 쌍둥이 대피 레인으로 치환해 스쳐 지나감 (R13)",
})
# 장면④ 물리적 대피 + 콘보이 통과: 회수(O-5,C08)가 콘보이 회랑 확보를 위해
# 루프 대피 레인으로 회피 (v3.6.1 실측 t=32.90 — 구 O-1/C05 장면을 O-5/C08로 승계)
events.append({
    "sim_t": 32.90, "kind":"order_event", "event":"EVAC_LANE",
    "subject":"C08", "detail":"B2-04a: 루프 A2 대피 레인으로 회피", "capsule":"C08",
    "order_id":"O-5", "priority":3, "by":"O-4",
    "reason":"Code Crimson 콘보이 회랑 확보를 위해 대피 레인으로 회피 (R3)",
})
# 장면⑤ RTA 슬랙 회복(R11): P2(O-2,C06)가 마감 임박으로 속도 상향.
# v_cmd는 engine.py 로그 포맷("slack=Xs v_cmd=Y")과 동일하게 실어 UI가
# 정규식으로 뽑아 "지령 속도"로 표시할 수 있게 한다.
events.append({
    "sim_t": 44.10, "kind":"order_event", "event":"RTA_ENGAGED",
    "subject":"O-2", "detail":"slack=4.0s v_cmd=1.05", "priority":2, "state":"EN_ROUTE",
})
# 점검 (SERVICING) 관련 이벤트 추가 (UI 시연용)
events.append({
    "sim_t": 40.0, "kind":"order_event", "event":"SERVICE_START",
    "subject":"C05", "detail":"ST-OR1: 정기 예방 점검(PM) 시작", "priority":None, "state":"SERVICING",
})
events.append({
    "sim_t": 45.0, "kind":"order_event", "event":"BLUE_CMD",
    "subject":"C05", "detail":"수동 제어권 인가", "priority":None,
})
events.append({
    "sim_t": 50.0, "kind":"order_event", "event":"SERVICE_DONE",
    "subject":"C05", "detail":"점검 완료, 라인 복귀 준비", "priority":None, "state":"STANDBY",
})

# 🧪 UI 테스트용 장면 (실측 v3.6.1 시나리오에는 없음): v3.6.1 기준선에서는 RTA
# 회복이 실제로 성공해 O-2가 승격 없이 완주하므로(마감 78s, 도착 76.47s),
# 실제 PRIORITY_PROMOTED가 발생하지 않는다. 등급 승격 반짝임 UI를 mock으로도
# 확인할 수 있도록, KPI/도착시각에 영향을 주지 않는 순수 로그성 데모 이벤트를
# 하나 추가한다 — 실데이터 연결 시에는 실제 PRIORITY_PROMOTED 이벤트로 자연히
# 대체된다.
events.append({
    "sim_t": 45.0, "kind":"order_event", "event":"PRIORITY_PROMOTED",
    "subject":"O-2", "detail":"P2->P1 (v_req=2.10>v_max=2.00) [UI 테스트 데모]",
    "priority":1, "state":"EN_ROUTE",
})

for oid in sorted(KPI_B_ARRIVE, key=lambda k: KPI_B_ARRIVE[k]):
    events.append({
        "sim_t": KPI_B_ARRIVE[oid], "kind":"order_event", "event":"ORDER_ARRIVE",
        "subject": oid, "detail":"", "priority": ROUTES[oid]["priority"],
        "item": ROUTES[oid]["item"], "state":"DELIVERED",
    })
events.append({
    "sim_t": SIM_MAKESPAN_B, "kind":"order_event", "event":"SIM_DONE",
    "subject":"sim", "detail": f"makespan={SIM_MAKESPAN_B}", "priority":None, "item":"", "state":None,
})
events.sort(key=lambda e: e["sim_t"])
events_b = events  # 모드 B(선점형) 이벤트 로그 — R1~R9 전 규칙 활성

# ------------------------------------------------------------------
# 🔴 모드 A(FCFS) 이벤트 로그 — R2~R7 비활성 / 선점 없음
# ------------------------------------------------------------------
events_a = []
for oid, route in ROUTES.items():
    events_a.append({
        "sim_t": route["release"], "kind":"order_event", "event":"ORDER_RELEASE",
        "subject": oid, "detail": "", "priority": route["priority"],
        "item": route["item"], "state":"EN_ROUTE",
    })
for oid in sorted(KPI_A_ARRIVE, key=lambda k: KPI_A_ARRIVE[k]):
    events_a.append({
        "sim_t": KPI_A_ARRIVE[oid], "kind":"order_event", "event":"ORDER_ARRIVE",
        "subject": oid, "detail":"", "priority": ROUTES[oid]["priority"],
        "item": ROUTES[oid]["item"], "state":"DELIVERED",
    })
events_a.append({
    "sim_t": SIM_MAKESPAN_A, "kind":"order_event", "event":"SIM_DONE",
    "subject":"sim", "detail": f"makespan={SIM_MAKESPAN_A} (FCFS, 선점 없음)", "priority":None, "item":"", "state":None,
})
events_a.sort(key=lambda e: e["sim_t"])

# ------------------------------------------------------------------
# 프레임 생성 (0.2초 간격, 0 ~ makespan+1)
# ------------------------------------------------------------------
ALL_BLOCK_IDS = list(CONFIRMED_LEN.keys())

def generate_frames(mode, kpi_arrive, makespan):
    frames = []
    t = 0.0
    T_END = makespan + 1.0
    prev_block_state_key = None
    while t <= T_END:
        orders_payload = {}
        for oid, route in ROUTES.items():
            if t < route["release"]:
                state = "CREATED"
            elif t < kpi_arrive[oid]:
                state = "EN_ROUTE"
            else:
                state = "DONE"
            arrive = kpi_arrive[oid] if t >= kpi_arrive[oid] else None
            orders_payload[oid] = {
                "priority": route["priority"], "state": state, "release": route["release"],
                "due": ORDER_DUE[oid], "arrive": arrive, "wait": 0.0,
                "capsules": route["capsules"],
            }
        control_state = {"sim_t": round(t,1), "kind":"control_state", "mode":mode,
                          "running": True, "orders": orders_payload,
                          "capsule_count": len(CAPSULE_ROUTE_OF)}

        capsules_payload = []
        for cid, oid in CAPSULE_ROUTE_OF.items():
            st = capsule_state_at(cid, t, mode=mode)
            capsules_payload.append({
                "capsule_id": cid, "block_id": st["block_id"], "pos_m": st["pos_m"],
                "forward": st["forward"], "state": st["state"], "order_id": oid,
                "x":0.0, "y":0.0, "z":4.0,
            })
        capsule_pose = {"sim_t": round(t,1), "kind":"capsule_pose", "capsules": capsules_payload}

        occ_map = {}
        for c in capsules_payload:
            if c["block_id"]:
                occ_map.setdefault(c["block_id"], []).append(c["capsule_id"])
        
        # CC(Code Crimson) 활성 여부: P0(C01~C04)가 EN_ROUTE 상태인 동안
        cc_active = (8.0 <= t < kpi_arrive["O-4"])
        
        blocks = []
        for bid in ALL_BLOCK_IDS:
            occ = occ_map.get(bid, [])
            # R9: CC 활성 중 콘보이 캡슐이 지나갈 때는 블록 점유 상한을 convoy_bunch_cap(=4)까지 개방
            has_convoy = any(cid in ["C01","C02","C03","C04"] for cid in occ)
            eff_cap = 4 if (cc_active and has_convoy) else BLOCK_STATIC_CAPS.get(bid, 1)
            blocks.append({
                "block_id": bid,
                "state": "OCCUPIED" if occ else "FREE",
                "occupancy": len(occ),
                "capacity": eff_cap,
                "locked": False,
                "corridor": "",
                "dir": 1,
                "capsule_ids": occ
            })
        bs_key = json.dumps(blocks, sort_keys=True)
        changed = (bs_key != prev_block_state_key)
        prev_block_state_key = bs_key

        frames.append({
            "t": round(t,1),
            "control_state": control_state,
            "capsule_pose": capsule_pose,
            "block_state": {"sim_t":round(t,1),"kind":"block_state","blocks":blocks} if changed else None,
        })
        t += 0.2
    return frames

frames_b = generate_frames("B", KPI_B_ARRIVE, SIM_MAKESPAN_B)
frames_a = generate_frames("A", KPI_A_ARRIVE, SIM_MAKESPAN_A)

# ------------------------------------------------------------------
# kpi 최종 요약
# ------------------------------------------------------------------
def build_kpi(mode, arrive_map, makespan):
    orders = {oid: {"priority": ROUTES[oid]["priority"], "arrive": v, "wait": 0.0}
              for oid, v in arrive_map.items()}
    arrival_order = sorted(arrive_map, key=lambda k: arrive_map[k])
    return {"kind":"kpi", "mode":mode, "makespan":makespan, "orders":orders, "arrival_order":arrival_order}

kpi_a = build_kpi("A", KPI_A_ARRIVE, SIM_MAKESPAN_A)
kpi_b = build_kpi("B", KPI_B_ARRIVE, SIM_MAKESPAN_B)

def route_end_of(oid, route):
    if route["chain_or2"]:
        return route["chain_or2"][-1][2]
    return route["chain_common"][-1][2]

order_meta = {oid: {"item": r["item"], "priority": r["priority"],
                     "capsules": r["capsules"],
                     "route_start": r["chain_common"][0][1], "route_end": route_end_of(oid, r)}
              for oid, r in ROUTES.items()}

out = {
    "_about": "v14: v3.6.1 확정 — 콘보이 ST-OR2 단일 집결 + R9 CC 용량 개방(effective_cap) "
              "+ R13 교행 + 노드 셋백(0.4m)/대피 홀드 래치 + 콘보이 0.9m 차두 복원 "
              "+ O-1 공급/O-5 회수 짝 + A74.97/B86.77",
    "frames_b": frames_b,
    "frames_a": frames_a,
    "order_events_b": events_b,
    "order_events_a": events_a,
    "kpi_a": kpi_a,
    "kpi_b": kpi_b,
    "order_meta": order_meta,
}

import os
out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mock_data_v14.json")
with open(out_path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=2)

print("생성 완료")
print("모드B 프레임 수:", len(frames_b), "/ 모드A 프레임 수:", len(frames_a))
print("모드B 이벤트 수:", len(events_b), "/ 모드A 이벤트 수:", len(events_a))
print()
print("=== 캡슐 배정 및 목적지 확인 (§6-1 대조, 모드B 기준) ===")
for oid, r in ROUTES.items():
    for cid in r["capsules"]:
        branch = r["split"].get(cid)
        if branch == "or1":
            dest = r["chain_or1"][-1][2]
        elif branch == "or2":
            dest = r["chain_or2"][-1][2]
        else:
            dest = r["chain_common"][-1][2]
        print(f"{oid} (P{r['priority']}, {r['item']}) 캡슐={cid}: {r['chain_common'][0][1]} -> {dest}, release(B)={CAPSULE_RELEASE[cid]}, release(A)={CAPSULE_RELEASE_A[cid]}")
