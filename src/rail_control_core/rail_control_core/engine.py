"""관제 코어 엔진 — 4계층 알고리즘 구현 (순수 Python, ROS 무관).

계층 매핑:
  ① 배차: PEDD 정렬(우선순위→EDD) + 콘보이 슈퍼오더 (R1, R9)
  ② 경로: 정적 경로 테이블(topology.ROUTES) — 탐색 없음
  ③ 교통 제어: 블록 상호배제 + 차두 간격 + 회랑 방향 토큰 + 단방향 (R8)
  ④ 긴급 대응: P0 회랑 선점 잠금(롤링 해제) + 3분기 스윕(R2 양보/R3 대피/R4 완주허용)
모드 A(FCFS): ①의 정렬이 요청시각순, ④ 비활성. ③ 안전망은 양 모드 공통.
"""
from .fsm import Order, Capsule, OrderState, CapsuleState, BlockState
from . import topology as T

DT = 1.0 / 30.0


class Engine:
    def __init__(self, params: dict, mode: str = "B"):
        self.p = params
        self.mode = mode
        self.t = 0.0
        self.orders: dict[str, Order] = {}
        self.capsules: dict[str, Capsule] = {}
        self.occ: dict[str, list[Capsule]] = {b: [] for b in T.BLOCKS}
        self.corridor_dir: dict[str, int] = {c: 0 for c in T.CORRIDORS}
        self.events: list[tuple[float, str, str, str]] = []
        self._finish_logged: set[str] = set()
        self._stall_since: float | None = None
        self._deadlock_logged = False

    # ---------- 조회 ----------
    def speed(self, block_id: str, c=None) -> float:
        v = self.p["speed_shaft"] if block_id.startswith("SB") else self.p["speed_default"]
        if c is not None and c.order is not None and c.order.speed:
            v = min(v, c.order.speed)
        return v

    def remaining_blocks(self, c: Capsule) -> list[str]:
        start = max(c.idx, 0)
        return [b for b, _ in c.route[start:]]

    def locked_blocks(self) -> set[str]:
        """P0 선점 잠금 = ETA 기반 이동 파도(bow-wave).
        콘보이가 yield_window(15s) 이내에 도달할 블록만 잠근다.
        - R6: 미발령 오더 제외 (예지 버그 방지)
        - 롤링 해제: 지나간 블록은 잔여 경로에서 빠져 자동 해제
        - 원거리 블록은 잠그지 않아 하위 오더의 불필요한 대피/지연 방지"""
        hot: set[str] = set()
        w = self.p["yield_window_sec"]
        for o in self.orders.values():
            if o.prio == 0 and o.release_t <= self.t and o.state == OrderState.EN_ROUTE:
                for cid in o.capsule_ids:
                    c = self.capsules[cid]
                    if c.state == CapsuleState.REMOVED:
                        continue
                    eta = 0.0
                    if c.block is not None and c.idx >= 0:
                        length = T.BLOCKS[c.block][2]
                        eta += max(0.0, length - c.pos) / self.speed(c.block, c)
                        if eta <= w:
                            hot.add(c.block)
                    for bid, _ in c.route[max(c.idx + 1, 0):]:
                        if eta > w:
                            break
                        hot.add(bid)
                        eta += T.BLOCKS[bid][2] / self.speed(bid, c)
        return hot

    def corridor_count(self, cor: str) -> int:
        return sum(len(self.occ[b]) for b in T.CORRIDORS[cor])


    # ---------- 블록 FSM 스냅샷 ----------
    def block_snapshot(self, locked: set[str] | None = None) -> dict[str, dict]:
        """블록 상태 기계의 현재 상태표 — STEP 3의 /block_state 토픽 페이로드 원형.
        FREE(여유) / RESERVED(선점 잠금·양보 대기 예약) / OCCUPIED(점유)."""
        if locked is None:
            locked = self.locked_blocks() if self.mode == "B" else set()
        reserved = {c.route[c.idx + 1][0] for c in self.capsules.values()
                    if c.state == CapsuleState.YIELD_WAIT and c.idx + 1 < len(c.route)}
        snap = {}
        for bid, (_, _, _, cap, _) in T.BLOCKS.items():
            n = len(self.occ[bid])
            if n > 0:
                st = BlockState.OCCUPIED
            elif bid in locked or bid in reserved:
                st = BlockState.RESERVED
            else:
                st = BlockState.FREE
            cor = T.BLOCK_TO_CORRIDOR.get(bid)
            snap[bid] = {
                "state": st.value,
                "occupancy": n,
                "capacity": cap,
                "locked": bid in locked,
                "corridor": cor,
                "dir": self.corridor_dir[cor] if cor else 0,
                "capsules": [c.cid for c in self.occ[bid]],
            }
        return snap

    # ---------- 회랑 토큰 ----------
    def corridor_ok(self, block_id: str, fwd: bool) -> bool:
        cor = T.BLOCK_TO_CORRIDOR.get(block_id)
        if cor is None:
            return True
        want = 1 if fwd else -1
        if self.corridor_count(cor) == 0:
            return True
        return self.corridor_dir[cor] == want

    def _corridor_update_on_enter(self, block_id: str, fwd: bool):
        cor = T.BLOCK_TO_CORRIDOR.get(block_id)
        if cor is not None:
            self.corridor_dir[cor] = 1 if fwd else -1

    def on_wave(self, c: Capsule, locked: set[str]) -> bool:
        """캡슐이 선점 파도의 경로 위에 있는가 (블록 일치 또는 회랑 공유)."""
        if c.block is None:
            return False
        if c.block in locked:
            return True
        cor = T.BLOCK_TO_CORRIDOR.get(c.block)
        return cor is not None and any(T.BLOCK_TO_CORRIDOR.get(b) == cor for b in locked)

    # ---------- 진입 판정 ----------
    def can_enter(self, c: Capsule, block_id: str, fwd: bool, locked: set[str],
                  ignore_prio: bool = False) -> tuple[bool, str]:
        a, b, length, cap, oneway = T.BLOCKS[block_id]
        if oneway and not fwd:
            return False, "oneway"
        occ = self.occ[block_id]
        if len(occ) >= cap:
            return False, "capacity"
        if occ:
            same_dir = all(o.fwd == fwd for o in occ)
            if not same_dir:
                return False, "headon"
            tail = occ[-1]
            if tail.pos < self.p["pitch"]:
                return False, "headway"
        if not self.corridor_ok(block_id, fwd):
            return False, "corridor"
        if self.mode == "B" and not ignore_prio:
            prio = c.order.prio if c.order else 9
            # R4 일반화: 이미 잠금 구간 위에 있으면 다음 잠금 블록으로의 '탈출 전진' 허용
            if prio > 0 and block_id in locked and not self.on_wave(c, locked):
                return False, "locked"
            if prio > 0 and self._yield_needed(c, block_id):
                return False, "yield_window"
        return True, "ok"

    def _yield_needed(self, c: Capsule, block_id: str) -> bool:
        """R5: 상위 캡슐이 yield_window 내에 이 블록을 필요로 하면 진입 보류.
        R6: 미발령(release_t > t) 오더는 검사 제외.
        보완: 내 현재 위치(블록/회랑)가 상위의 잔여 경로에 걸치면 양보하지 않고
        계속 전진해 길을 비운다 (제자리 대기가 상위를 막는 역효과 방지)."""
        my_prio = c.order.prio if c.order else 9
        my_cor = T.BLOCK_TO_CORRIDOR.get(c.block) if c.block else None
        for other in self.capsules.values():
            if other is c or other.order is None or other.state == CapsuleState.REMOVED:
                continue
            o = other.order
            if o.prio >= my_prio or o.release_t > self.t:
                continue
            rem_all = self.remaining_blocks(other)
            if c.block in rem_all:
                continue  # 나는 상위 경로 위 -> 전진해서 비켜야 함
            if my_cor and any(T.BLOCK_TO_CORRIDOR.get(b) == my_cor for b in rem_all):
                continue  # 내 회랑을 상위가 쓸 예정 -> 전진해서 회랑을 비움
            rem = other.remaining_from_current(block_id)
            if rem is not None and self._eta(other, rem) <= self.p["yield_window_sec"]:
                return True
            # 회랑 단위 검사: 진입하려는 블록의 회랑을 상위가 창 내에 통과 예정이면 보류
            tcor = T.BLOCK_TO_CORRIDOR.get(block_id)
            if tcor:
                for b in rem_all:
                    if T.BLOCK_TO_CORRIDOR.get(b) == tcor:
                        rem2 = other.remaining_from_current(b)
                        if rem2 is not None and self._eta(other, rem2) <= self.p["yield_window_sec"]:
                            return True
                        break
        return False

    def _eta(self, c: Capsule, blocks_until: list[str]) -> float:
        eta = 0.0
        if c.block is not None and c.idx >= 0:
            _, _, length, _, _ = T.BLOCKS[c.block]
            eta += max(0.0, length - c.pos) / self.speed(c.block, c)
        for b in blocks_until:
            eta += T.BLOCKS[b][2] / self.speed(b, c)
        return eta

    def _wave_opposes(self, nbid: str, nfwd: bool) -> bool:
        """다음 블록에서 P0 파도와 역방향으로 만나는가 (True면 대피가 정답)."""
        for o in self.orders.values():
            if o.prio != 0 or o.state != OrderState.EN_ROUTE or o.release_t > self.t:
                continue
            for cid in o.capsule_ids:
                c = self.capsules[cid]
                if c.state == CapsuleState.REMOVED:
                    continue
                for bid, f in c.route[max(c.idx, 0):]:
                    if bid == nbid and f != nfwd:
                        return True
        return False

    # ---------- 전이 실행 ----------
    def _note_progress(self):
        self._stall_since = None

    def _check_stall(self):
        """전역 무진행 감시: 활성 오더가 있는데 아무 캡슐도 움직이지 못하면
        stall_timeout 후 DEADLOCK 이벤트 1회 발행 (운영 관제 알림용)."""
        if self._deadlock_logged:
            return
        active = any(o.state == OrderState.EN_ROUTE for o in self.orders.values())
        if not active:
            self._stall_since = None
            return
        if self._stall_since is None:
            self._stall_since = self.t
            return
        if self.t - self._stall_since > self.p.get("stall_timeout_sec", 20.0):
            stuck = [f"{c.cid}@{c.block}" for c in self.capsules.values()
                     if c.state in (CapsuleState.YIELD_WAIT, CapsuleState.EVACUATED)]
            self.log("DEADLOCK", "sim", "무진행 " +
                     f"{self.t - self._stall_since:.0f}s, 대기: {','.join(stuck)}")
            self._deadlock_logged = True

    def _do_enter(self, c: Capsule, block_id: str, fwd: bool):
        self._note_progress()
        if c.block is not None and c in self.occ[c.block]:
            self.occ[c.block].remove(c)
        c.block, c.fwd, c.pos = block_id, fwd, 0.0
        self.occ[block_id].append(c)
        self._corridor_update_on_enter(block_id, fwd)
        c.req_t = float("inf")

    def _try_evacuate(self, c: Capsule, next_bid: str, next_fwd: bool, locked: set[str]) -> bool:
        """R3: 다음 블록이 선점 잠금이고 내가 잠금 경로 위에 있으면 대피 시도."""
        if c.block not in locked:
            return False
        # 상위 활성 오더들의 목적지(최종 블록)는 대피지에서 제외
        my_prio = c.order.prio if c.order else 9
        superior_dest = {self.capsules[cid].route[-1][0]
                         for o in self.orders.values() if o.prio < my_prio
                         and o.state == OrderState.EN_ROUTE
                         for cid in o.capsule_ids}
        # 1순위: 다음 블록의 대피 레인(같은 분기점에서 진입 가능)
        esc = T.ESCAPE_LANE.get(next_bid)
        if esc and esc not in locked:
            ok, _ = self.can_enter(c, esc, next_fwd, locked, ignore_prio=True)
            if ok:
                c.route[c.idx + 1] = (esc, next_fwd)
                self.log("EVAC_LANE", c.cid, f"{c.block}->{esc}")
                return True
        # 2순위: 진출 노드에 접한 빈 지선/반대 레인으로 후퇴 대기 후 복귀(왕복 삽입)
        node = T.exit_node(c.block, c.fwd)
        for bid, (a, b, _, _, oneway) in T.BLOCKS.items():
            if bid in locked or oneway or bid == c.block or bid in superior_dest:
                continue
            into = True if a == node else (False if b == node else None)
            if into is None or T.BLOCK_TO_CORRIDOR.get(bid):
                continue
            if bid in (r[0] for r in c.route):
                continue
            ok, _ = self.can_enter(c, bid, into, locked, ignore_prio=True)
            if ok:
                c.route[c.idx + 1:c.idx + 1] = [(bid, into), (bid, not into)]
                self.log("EVAC_SPUR", c.cid, f"{c.block}->{bid}")
                return True
        return False

    # ---------- 메인 틱 ----------
    def tick(self):
        self.t += DT
        locked = self.locked_blocks() if self.mode == "B" else set()

        # 오더 발령
        for o in self.orders.values():
            if o.state == OrderState.CREATED and o.release_t <= self.t:
                o.state = OrderState.EN_ROUTE
                self.log("ORDER_RELEASE", o.oid, f"P{o.prio}")
                for cid in o.capsule_ids:
                    c = self.capsules[cid]
                    if c.state in (CapsuleState.QUEUED, CapsuleState.STANDBY):
                        c.state = CapsuleState.MOVING

        # 이동 + 하역
        for c in self.capsules.values():
            if c.state == CapsuleState.UNLOADING and self.t >= c.unload_until:
                c.state = CapsuleState.REMOVED
                if c.block and c in self.occ[c.block]:
                    self.occ[c.block].remove(c)
                    c.block = None
                self._check_order_done(c.order)
            if c.state not in (CapsuleState.MOVING, CapsuleState.FINISHING,
                               CapsuleState.EVACUATED):
                continue
            if c.block is None:
                continue
            _, _, length, _, _ = T.BLOCKS[c.block]
            limit = length
            occ = self.occ[c.block]
            i = occ.index(c)
            if i > 0:
                limit = min(limit, occ[i - 1].pos - self.p["pitch"])
            new_pos = min(c.pos + self.speed(c.block, c) * DT, max(limit, c.pos))
            if new_pos > c.pos + 1e-9:
                self._note_progress()
            c.pos = new_pos
            if self.mode == "B" and c.block in locked and c.order and c.order.prio > 0 \
               and c.cid not in self._finish_logged and c.pos < length:
                self._finish_logged.add(c.cid)
                c.state = CapsuleState.FINISHING
                self.log("FINISH_ALLOWED", c.cid, c.block)

        # 진입 요청 수집
        requests: list[Capsule] = []
        for c in self.capsules.values():
            if c.state in (CapsuleState.REMOVED, CapsuleState.UNLOADING,
                           CapsuleState.DOCKED, CapsuleState.STANDBY, CapsuleState.QUEUED):
                continue
            if c.block is None:
                continue
            _, _, length, _, _ = T.BLOCKS[c.block]
            at_end = c.pos >= length - 1e-9
            if not at_end:
                continue
            if c.idx + 1 >= len(c.route):
                self._arrive(c)
                continue
            if c.req_t == float("inf"):
                c.req_t = self.t
            requests.append(c)

        # 중재 정렬: 모드 B = (우선순위, EDD, 요청시각) / 모드 A = 요청시각
        if self.mode == "B":
            requests.sort(key=lambda c: (c.order.prio, c.order.due_t, c.req_t))
        else:
            requests.sort(key=lambda c: c.req_t)

        for c in requests:
            nbid, nfwd = c.route[c.idx + 1]
            # 지선 왕복: 같은 블록의 역방향 전이는 재진입이 아니라 제자리 방향 전환
            if nbid == c.block and nfwd != c.fwd:
                _, _, length, _, _ = T.BLOCKS[c.block]
                c.fwd, c.pos = nfwd, max(0.0, length - c.pos)
                c.idx += 1
                c.req_t = float("inf")
                continue
            prio = c.order.prio if c.order else 9
            # R3 우선: 잠금 경로 위에 있고 다음도 잠금이면 대피 시도 (실패 시 R4 플러시)
            if self.mode == "B" and prio > 0 and nbid in locked \
               and self.on_wave(c, locked) and c.state != CapsuleState.EVACUATED \
               and self._wave_opposes(nbid, nfwd):
                if self._try_evacuate(c, nbid, nfwd, locked):
                    nbid2, nfwd2 = c.route[c.idx + 1]
                    ok2, _ = self.can_enter(c, nbid2, nfwd2, locked, ignore_prio=True)
                    if ok2:
                        c.idx += 1
                        self._do_enter(c, nbid2, nfwd2)
                        c.state = CapsuleState.EVACUATED
                        c.detour = nbid2
                    continue
            ok, reason = self.can_enter(c, nbid, nfwd, locked)
            if ok:
                if c.state in (CapsuleState.YIELD_WAIT, CapsuleState.EVACUATED):
                    self.log("RESUME", c.cid, nbid)
                    c.detour = None
                c.idx += 1
                self._do_enter(c, nbid, nfwd)
                c.state = CapsuleState.MOVING
            else:
                if c.state == CapsuleState.MOVING:
                    c.state = CapsuleState.YIELD_WAIT
                    self.log("YIELD", c.cid, f"{nbid}:{reason}")
                if c.order:
                    c.order.wait_total += DT

        self._check_stall()

    def _arrive(self, c: Capsule):
        c.state = CapsuleState.UNLOADING
        c.unload_until = self.t + self.p["unload_sec"]
        self.log("ARRIVE", c.cid, c.block or "?")
        o = c.order
        if o and all(self.capsules[x].state in (CapsuleState.UNLOADING, CapsuleState.REMOVED)
                     for x in o.capsule_ids):
            if o.arrive_t is None:
                o.arrive_t = self.t
                o.state = OrderState.ARRIVED
                self.log("ORDER_ARRIVE", o.oid, f"t={self.t:.2f}")

    def _check_order_done(self, o: Order | None):
        if o and all(self.capsules[x].state == CapsuleState.REMOVED for x in o.capsule_ids):
            o.state = OrderState.DONE

    def log(self, ev: str, subj: str, detail: str):
        self.events.append((round(self.t, 2), ev, subj, detail))

    def run(self, until: float = 300.0) -> dict:
        while self.t < until:
            self.tick()
            if all(o.state == OrderState.DONE for o in self.orders.values()):
                break
        else:
            raise RuntimeError(f"타임아웃/교착 의심: t={self.t:.1f} "
                               + str({c.cid: (c.block, c.state.value) for c in self.capsules.values()}))
        return {
            "makespan": round(max(o.arrive_t for o in self.orders.values()), 2),
            "orders": {o.oid: {"arrive": round(o.arrive_t, 2),
                               "wait": round(o.wait_total, 2)} for o in self.orders.values()},
        }


# Capsule 헬퍼 (엔진에서 사용)
def _remaining_from_current(self: Capsule, block_id: str):
    start = max(self.idx + 1, 0)
    ids = [b for b, _ in self.route[start:]]
    if block_id not in ids:
        return None
    return ids[:ids.index(block_id)]


Capsule.remaining_from_current = _remaining_from_current
