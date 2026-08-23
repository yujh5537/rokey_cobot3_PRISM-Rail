"""3FSM — 오더/캡슐/블록의 상태 정의와 데이터 구조.
FSM(유한 상태 기계) = "지금 상태"와 "다음 상태로 넘어가는 조건"의 규칙표.
"""
from dataclasses import dataclass, field
from enum import Enum


class OrderState(Enum):
    CREATED = "CREATED"        # 오더 생성(발령)
    EN_ROUTE = "EN_ROUTE"      # 운송 중
    ARRIVED = "ARRIVED"        # 목적지 도착(하역 중)
    DONE = "DONE"              # 완료


class CapsuleState(Enum):
    DOCKED = "DOCKED"          # 디포 충전 도크 대기
    QUEUED = "QUEUED"          # 출동 대기열(BB-08) 대기
    STANDBY = "STANDBY"        # 출발 스테이션 선배치 대기
    MOVING = "MOVING"          # 주행
    YIELD_WAIT = "YIELD_WAIT"  # 진입 전 양보 대기 (R2)
    EVACUATED = "EVACUATED"    # 대피 레인/지선 대피 (R3)
    FINISHING = "FINISHING"    # 완주 허용 주행 (R4)
    UNLOADING = "UNLOADING"    # 목적지 하역
    REMOVED = "REMOVED"        # 시나리오 상 트랙 이탈(회송 생략)


class BlockState(Enum):
    FREE = "FREE"              # 여유
    RESERVED = "RESERVED"      # 예약(선점 잠금 포함)
    OCCUPIED = "OCCUPIED"      # 점유


@dataclass
class Order:
    oid: str
    prio: int                  # 0=P0(Code Crimson) ... 3=P3
    route_name: str
    release_t: float           # 발령 시각
    due_t: float               # EDD 마감시각 (release + priority_due)
    capsule_ids: list[str]
    state: OrderState = OrderState.CREATED
    arrive_t: float | None = None   # 마지막 캡슐 도착 시각
    wait_total: float = 0.0         # 양보·대피 누적 대기(지연 비용 가시화용)
    speed: float | None = None      # 오더별 속도 상한(예: P3 오염 기구 저속 규정)
    v_cmd: float | None = None      # RTA 회복 지령 속도 (R11). None = 평시 순항
    promoted: bool = False          # R12 등급 승격(P2->P1) 여부 — 회복 불가 판정 시


@dataclass
class Capsule:
    cid: str
    order: Order | None = None
    route: list[tuple[str, bool]] = field(default_factory=list)
    idx: int = -1              # 현재 블록의 route 인덱스 (-1=미진입)
    block: str | None = None   # 현재 블록 id
    fwd: bool = True           # 현재 블록 내 주행 방향
    pos: float = 0.0           # 블록 진입점 기준 진행 거리(m)
    state: CapsuleState = CapsuleState.DOCKED
    detour: str | None = None  # 대피 중인 레인/지선 블록 id
    unload_until: float = 0.0
    req_t: float = float("inf")  # 현재 블록 진입 요청 시각(FCFS용)
    vel: float = 0.0           # 현재 속도(m/s) — 가속 램프(용혈 방지 0.8m/s²)용
