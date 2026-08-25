"""geometry — 논리 좌표(블록id + pos_m + 방향) -> 씬 월드 xyz 변환.

합의 사항(2026-08-21, C 수용): 위치의 진실 소스는 관제 코어이며, 코어가 xyz까지
계산해 /capsule_pose 에 포함한다. 브릿지는 /World/Capsules/C01~C10 프림 트랜스폼에
그대로 적용, UI도 동일 값 사용 — 좌표 변환 로직은 이 모듈 한 곳에만 존재한다.

규칙:
- 층 Z: B1F 레일 4.0 / 2F 레일 13.0 (레일 센터라인)
- 샤프트: SB-UP x=-7.25, SB-DN x=-6.75 (복선), y=-5.0, z는 진행률로 4.0<->13.0 보간
- 일반 블록: BLOCK_PATHS 폴리라인(없으면 노드 직선)을 따라 진행률 보간
  * NORMALIZED 블록은 폴리라인 실길이 != 실측 길이라 비율 매핑(시각화 근사)
- DOCKED 캡슐: 디포 측면 슬롯(DOCK_SLOTS)에 배치 — 관제 정의가 정본 (B 확인 2026-08-23)
"""
from . import topology as T


def _lerp(p, q, t):
    return (p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t)


def _along_polyline(pts, dist):
    """폴리라인 시작점부터 dist 지점의 XY (범위 밖은 양끝 고정)."""
    if dist <= 0:
        return pts[0]
    for p, q in zip(pts, pts[1:]):
        seg = ((q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2) ** 0.5
        if dist <= seg:
            return _lerp(p, q, dist / seg if seg > 0 else 0.0)
        dist -= seg
    return pts[-1]


def block_floor_z(block_id: str) -> float:
    a, b, *_ = T.BLOCKS[block_id]
    return T.RAIL_Z["2F"] if (a in T.FLOOR_2F or b in T.FLOOR_2F) else T.RAIL_Z["B1F"]


def pose_to_xyz(block_id: str, pos_m: float, forward: bool) -> tuple[float, float, float]:
    """블록 진입점 기준 진행거리 pos_m 의 월드 좌표."""
    a, b, length, _, _ = T.BLOCKS[block_id]
    frac = min(max(pos_m / length, 0.0), 1.0) if length > 0 else 0.0
    if block_id in ("SB-UP", "SB-DN"):
        x = T.SHAFT_X[block_id]
        y = T.NODE_XY["N-W1"][1]
        z0, z1 = (T.RAIL_Z["B1F"], T.RAIL_Z["2F"]) if block_id == "SB-UP" \
            else (T.RAIL_Z["2F"], T.RAIL_Z["B1F"])
        # 발행원 반올림 원칙: 부동소수 잔재(8.4999...)를 수신측이 처리하게 두지 않는다
        return (x, y, round(z0 + (z1 - z0) * frac, 4))
    pts = T.BLOCK_PATHS.get(block_id, [T.NODE_XY[a], T.NODE_XY[b]])
    if not forward:
        pts = list(reversed(pts))
    plen = sum(((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
               for (x1, y1), (x2, y2) in zip(pts, pts[1:]))
    x, y = _along_polyline(pts, frac * plen)
    return (round(x, 4), round(y, 4), block_floor_z(block_id))


def dock_slot_xyz(slot_index: int) -> tuple[float, float, float]:
    """디포 도크 슬롯 (DOCKED 캡슐 배치, 관제 정의가 정본 — 씬은 이 좌표에 시각물 배치)."""
    x, y = T.DOCK_SLOTS[slot_index % len(T.DOCK_SLOTS)]
    return (x, y, T.RAIL_Z["B1F"])


def service_pose_xyz(bid: str, s: float) -> tuple[float, float, float]:
    """OR 서비스 딥 경로(SERVICE_SEQ) 위 3D 자세 — 발행원 반올림 원칙 동일.
    SERVICING 캡슐은 블록 밖(레일 해제 상태)이라 pose_to_xyz 대신 이 경로를 쓴다."""
    return T.service_pose(bid, s)
