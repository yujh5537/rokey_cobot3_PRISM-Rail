"""
fsm.py — 3-FSM(세 개의 상태 기계) 정의.

초보자용 설명
------------
"3-FSM"은 서로 다른 세 가지 대상에 각각 상태 기계를 하나씩 두는 설계입니다.
이걸 분리해 두면 "누구의 상태를 바꾸는 코드인지"가 항상 명확해집니다.

  1) OrderFSM   — 주문(오더) 자체의 진행 상태.        "이 배송 건은 어디까지 왔나?"
  2) CapsuleFSM — 물리적인 캡슐(운송체)의 동작 상태.  "이 캡슐은 지금 뭘 하고 있나?"
  3) BlockFSM   — 레일 구간의 점유 상태.              "이 구간은 지금 비었나?"

세 FSM이 만나는 지점이 곧 관제 로직입니다.
  · 캡슐이 MOVING 이 되려면 → 다음 Block이 FREE 여야 한다
  · Block이 OCCUPIED 인데 상위 오더가 오면 → 캡슐을 YIELD_WAIT / EVACUATING 로 보낸다
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


# =====================================================================
# 1) 오더 FSM
# =====================================================================
class OrderState(str, Enum):
    CREATED   = "CREATED"      # 오더 생성됨(아직 큐에 안 들어감)
    QUEUED    = "QUEUED"       # 배차 대기 큐에 있음
    ASSIGNED  = "ASSIGNED"     # 캡슐이 배정됨
    RUNNING   = "RUNNING"      # 주행 중
    DELIVERED = "DELIVERED"    # 하역·인계 완료
    CANCELED  = "CANCELED"


# =====================================================================
# 2) 캡슐 FSM  (2차 발표 1-4. 상태 다이어그램과 1:1 대응)
# =====================================================================
class CapsuleState(str, Enum):
    IDLE        = "IDLE"          # 유휴 — 배차 대기
    LOADING     = "LOADING"       # 적재 — 스테이션 적재
    MOVING      = "MOVING"        # 주행 — 예약 경로 이동
    YIELD_WAIT  = "YIELD_WAIT"    # 양보 대기 — 구간 진입 '전' 정차   ← 선점 대응
    EVACUATING  = "EVACUATING"    # 대피 — 지선으로 회피             ← 선점 대응
    RUN_THROUGH = "RUN_THROUGH"   # 완주 허용 — 구간 내 통과 승인      ← 선점 대응
    HANDOVER    = "HANDOVER"      # 하역·인계 — 인증 후 완료
    RETURNING   = "RETURNING"     # 복귀 후 유휴로


# 선점(preemption) 때문에 멈춰 있는 상태들
PREEMPT_STATES = {CapsuleState.YIELD_WAIT, CapsuleState.EVACUATING}


# =====================================================================
# 3) 블록 FSM
# =====================================================================
class BlockState(str, Enum):
    FREE     = "FREE"       # 여유 — 아무도 안 씀
    RESERVED = "RESERVED"   # 예약됨 — 곧 들어올 캡슐이 잡아둠
    OCCUPIED = "OCCUPIED"   # 점유 — 캡슐이 실제로 안에 있음


# =====================================================================
# 데이터 모델
# =====================================================================
@dataclass
class Order:
    oid: str
    priority: int            # 0=P0(최우선) ~ 3=P3
    item: str
    origin: str
    dest: str
    release_s: float         # 발생 시각
    due_s: float             # 마감시각 (EDD 정렬 기준)
    cargo_class: str = "clean"
    code_red: bool = False
    load_s: float | None = None       # 오더별 적재시간 override

    state: OrderState = OrderState.CREATED
    assigned_capsule: str | None = None
    start_s: float | None = None      # 실제 출발 시각
    finish_s: float | None = None     # 인계 완료 시각
    wait_s: float = 0.0               # 선점 때문에 멈춰 있던 누적 시간(= 지연 비용)

    @property
    def pedd_key(self) -> tuple[int, float, str]:
        """P-EDD 정렬 키: 등급 우선, 같은 등급이면 마감 빠른 순."""
        return (self.priority, self.due_s, self.oid)

    @property
    def lead_time_s(self) -> float | None:
        if self.finish_s is None:
            return None
        return self.finish_s - self.release_s

    def slack_s(self, now: float) -> float:
        """남은 여유시간. 음수면 마감 초과(안정성 시한 소진)."""
        return self.due_s - now


@dataclass
class Capsule:
    cid: str
    node: str                              # 현재 정차 노드 (블록 안이면 직전 노드)
    state: CapsuleState = CapsuleState.IDLE
    order: Order | None = None

    route: list[str] = field(default_factory=list)   # 남은 블록 경로
    cur_block: str | None = None                     # 지금 들어가 있는 블록
    block_elapsed: float = 0.0                       # 그 블록에서 지난 시간
    timer: float = 0.0                               # LOADING/HANDOVER 등 잔여시간
    home: str | None = None                          # 복귀 지점
    evac_target: str | None = None                   # 대피 지선 노드

    def remaining_blocks(self) -> list[str]:
        """이 캡슐이 앞으로(지금 포함) 쓸 블록 전체 = 이 캡슐의 '주행 회랑(Corridor)'."""
        return ([self.cur_block] if self.cur_block else []) + list(self.route)

    def remain_in_block(self, traverse_s: float) -> float:
        """현재 블록의 잔여 통과시간. '완주 허용' 판단의 핵심 값."""
        return max(0.0, traverse_s - self.block_elapsed)
