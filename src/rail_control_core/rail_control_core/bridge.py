"""브리지 계층 — 관제 엔진과 ROS 사이의 rclpy 무관 어댑터.

설계 의도(포트-어댑터): node.py(rclpy)는 이 클래스를 30Hz로 호출해 dict 페이로드를
msg로 변환만 한다. 판단 로직 전부가 여기 있으므로 ROS 없이 단위 테스트 가능하고,
통합 시 문제가 생기면 "브리지 테스트 통과 여부"로 관제/연동 책임을 즉시 분리할 수 있다.
"""
from . import geometry as G
from . import scenario
from .engine import DT, Engine
from .fsm import CapsuleState, OrderState


class Bridge:
    def __init__(self, mode: str = "B", autostart: bool = False):
        self.mode = mode
        self._done_notified = False
        self.reset(mode)
        self.running = autostart

    # ---------- 제어 ----------
    def reset(self, mode: str | None = None):
        if mode:
            self.mode = mode
        self.eng: Engine = scenario.build(self.mode)
        self.running = False
        self._ev_idx = 0
        self._prev_blocks: dict | None = None
        self._done_notified = False

    def command(self, cmd: str) -> tuple[bool, str]:
        cmd = cmd.strip().lower()
        if cmd == "start":
            self.running = True
            return True, f"시작 (mode={self.mode}, t={self.eng.t:.2f})"
        if cmd == "pause":
            self.running = False
            return True, f"일시정지 (t={self.eng.t:.2f})"
        if cmd == "reset":
            self.reset()
            return True, f"초기화 완료 (mode={self.mode})"
        if cmd in ("mode_a", "mode_b"):
            self.reset("A" if cmd == "mode_a" else "B")
            return True, f"모드 {self.mode} 로 초기화 (start 필요)"
        if cmd == "status":
            orders = {o.oid: o.state.value for o in self.eng.orders.values()}
            return True, (f"t={self.eng.t:.2f} mode={self.mode} "
                          f"running={self.running} orders={orders}")
        return False, f"알 수 없는 명령: {cmd} (start|pause|reset|mode_a|mode_b|status)"

    def code_crimson(self, note: str = "") -> tuple[bool, str, str]:
        """P0 오더를 지금 즉시 발령. 시나리오의 예약 발령(release_t)을 현재 시각으로 당긴다."""
        for o in self.eng.orders.values():
            if o.prio == 0:
                if o.state != OrderState.CREATED or o.release_t <= self.eng.t:
                    return False, o.oid, f"{o.oid} 이미 발령됨 (state={o.state.value})"
                o.release_t = self.eng.t
                o.due_t = self.eng.t + scenario.PARAMS["priority_due_sec"][0]
                self.eng.log("CODE_CRIMSON", o.oid, note or "manual trigger")
                return True, o.oid, f"Code Crimson 발동 — 콘보이 4대 출동 (t={self.eng.t:.2f})"
        return False, "", "P0 오더가 시나리오에 없음"

    # ---------- 틱 & 페이로드 ----------
    def step(self) -> dict:
        """1틱 진행 후 발행할 페이로드 묶음을 돌려준다. 정지 상태면 빈 묶음."""
        out = {"capsules": None, "blocks": None, "events": [], "sim_t": self.eng.t}
        if self.running:
            self.eng.tick()
            out["sim_t"] = self.eng.t
            if not self._done_notified and \
               all(o.state == OrderState.DONE for o in self.eng.orders.values()):
                mk = max(o.arrive_t for o in self.eng.orders.values())
                self.eng.log("SIM_DONE", "sim", f"makespan={mk:.2f}")
                self._done_notified = True
            # v3.7: SIM_DONE(=KPI 종료) 이후에도 OR 서비스 딥 연출이 남아 있으면
            # 발행을 계속한다. 연출까지 끝나야 정지 — 기준값(makespan)은 그대로.
            if self._done_notified and self.running and not any(
                    x.state in (CapsuleState.SERVICING, CapsuleState.UNLOADING)
                    for x in self.eng.capsules.values()):
                self.running = False
        out["capsules"] = self._capsule_payload()
        blocks = self._block_payload()
        if blocks != self._prev_blocks:
            out["blocks"] = blocks
            self._prev_blocks = blocks
        ev = self.eng.events[self._ev_idx:]
        self._ev_idx = len(self.eng.events)
        out["events"] = [{"sim_t": t, "event": e, "subject": s, "detail": d}
                         for t, e, s, d in ev]
        return out

    def _capsule_payload(self) -> list[dict]:
        return [{**self._logical(c), **self._xyz(c)}
                for c in self.eng.capsules.values()]

    @staticmethod
    def _logical(c) -> dict:
        return {
            "capsule_id": c.cid,
            "block_id": c.block or "",
            "pos_m": round(c.svc_s if c.state == CapsuleState.SERVICING else c.pos, 4),
            "forward": bool(c.fwd),
            "state": c.state.value,
            "order_id": c.order.oid if c.order else "",
            # v3.8.2: 씬 연출용 우선순위. B 가 캡슐 바디 색을 이 값으로 칠한다
            #   (0=P0 crimson / 1=P1 / 2=P2 / 3=P3 / 9=미배정).
            #   이벤트가 아니라 매 프레임 상태로 보내는 이유: 이벤트를 놓치거나
            #   늦게 구독해도 다음 프레임에 자동 복구된다.
            "prio": (c.order.prio if c.order else 9),
        }

    @staticmethod
    def _xyz(c) -> dict:
        """씬 월드 좌표 — 위치의 진실 소스는 코어라는 합의(2026-08-21)에 따라
        코어가 계산해서 실어 보낸다. 브릿지는 프림 트랜스폼에 꽂기만 하면 된다.

        - DOCKED: 디포 측면 가상 슬롯 (B 도크 좌표 회신 시 geometry 만 수정)
        - SERVICING: OR 서비스 딥 경로 보간 — ⚠️ z 가 13.0 -> 11.0 으로 내려간다.
          수신측 UI 가 층 고정 높이를 가정하면 캡슐이 사라져 보이니 높이 축 확인 필요.
        - REMOVED: 블록이 없으므로 원점. 수신측은 이 상태에서 프림을 숨긴다.
        """
        if c.state == CapsuleState.DOCKED:
            x, y, z = G.dock_slot_xyz(int(c.cid[1:]) - 1)
        elif c.state == CapsuleState.SERVICING and c.svc_bid:
            x, y, z = G.service_pose_xyz(c.svc_bid, c.svc_s)
        elif c.block:
            x, y, z = G.pose_to_xyz(c.block, c.pos, bool(c.fwd))
        else:
            x, y, z = 0.0, 0.0, 0.0
        return {"x": x, "y": y, "z": z}

    def _block_payload(self) -> list[dict]:
        snap = self.eng.block_snapshot()
        return [{
            "block_id": bid,
            "state": d["state"],
            "occupancy": d["occupancy"],
            "capacity": d["capacity"],
            "locked": d["locked"],
            "corridor": d["corridor"] or "",
            "dir": d["dir"],
            "capsule_ids": d["capsules"],
        } for bid, d in snap.items()]
