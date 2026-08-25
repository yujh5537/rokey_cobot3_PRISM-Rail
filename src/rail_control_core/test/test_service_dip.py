"""OR 서비스 딥 회귀 테스트 (v3.7 — B 3차 핸드오프 "추가된 좌표", 2026-08-25).

이 파일이 지키는 것은 두 가지다:
  ① PDF 원문 좌표와 제어 규칙 5조가 코드에서 그대로 살아 있는지
     (특히 "BLUE 는 자동 선택 경로가 아님" — 승인 없이 진행되면 즉시 실패)
  ② 딥이 레일 역학을 건드리지 않는다는 설계 전제
     (SERVICING 진입 시 레일 점유가 기존 REMOVED 와 같은 타이밍에 해제됨)
실행: python3 -m pytest test/ -q
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from rail_control_core import topology as T
from rail_control_core.bridge import Bridge
from rail_control_core.fsm import CapsuleState

# B 3차 핸드오프 PDF 원문 (Scene 좌표) -> 관제 Logical z = Scene z + 0.015
SCENE_TO_LOGICAL = 0.015
PDF_OR1 = [(7.59, 4.50, 12.985), (7.59, 4.00, 12.985), (7.59, 4.00, 10.985),
           (7.59, 5.00, 10.985), (7.59, 5.00, 12.985), (7.59, 4.50, 12.985)]
PDF_OR2 = [(7.59, -0.80, 12.985), (7.59, -1.30, 12.985), (7.59, -1.30, 10.985),
           (7.59, -0.30, 10.985), (7.59, -0.30, 12.985), (7.59, -0.80, 12.985)]


def test_waypoints_match_handoff_pdf():
    """딥 웨이포인트가 PDF 실측과 일치 (ENTRY 이후 6점, 시작점은 레일 종점 노드)."""
    for bid, pdf, node in (("B2-08", PDF_OR1, "ST-OR1"), ("B2-09", PDF_OR2, "ST-OR2")):
        pts = T.SERVICE_SEQ[bid]["pts"]
        assert pts[0] == (*T.NODE_XY[node], T.RAIL_Z["2F"]), f"{bid}: 딥 시작점 != 레일 종점"
        for got, (x, y, z) in zip(pts[1:], pdf):
            assert got == (x, y, round(z + SCENE_TO_LOGICAL, 3)), f"{bid}: {got} != PDF {(x, y, z)}"


def test_work_is_the_bottom_stop():
    """WORK(=DOWN_BOTTOM)은 하단 z=11.0 이고 경로장 9.09m 중 5.59m 지점."""
    for bid in ("B2-08", "B2-09"):
        g = T.SERVICE_GEO[bid]
        assert abs(g["total"] - 9.09) < 1e-6
        assert abs(g["work_s"] - 5.59) < 1e-6
        assert T.service_pose(bid, g["work_s"])[2] == 11.0


def test_main_topology_untouched():
    """"기존 메인 토폴로지 변경 금지" — 딥은 별도 자산 계층이라 BLOCKS 에 없어야 한다."""
    for bid in T.SERVICE_SEQ:
        assert bid in T.BLOCKS, "딥 키는 기존 블록 id 여야 한다(새 블록 신설 금지)"
    assert T.validate() == []


def _run_full(mode: str):
    """딥 연출이 끝나 브리지가 스스로 멈출 때까지 돌리고 관측치를 모은다."""
    b = Bridge(mode, autostart=True)
    evs, zmin, pre_blue_z = [], {}, {}
    for _ in range(12000):
        out = b.step()
        evs += [(e["sim_t"], e["event"], e["subject"]) for e in out["events"]]
        for c in out["capsules"]:
            if c["state"] != "SERVICING":
                continue
            assert c["block_id"] == "", "SERVICING 은 레일 점유가 해제된 상태여야 한다"
            zmin[c["capsule_id"]] = min(zmin.get(c["capsule_id"], 99.0), c["z"])
            cap = b.eng.capsules[c["capsule_id"]]
            if not cap.svc_blue:
                pre_blue_z[c["capsule_id"]] = max(pre_blue_z.get(c["capsule_id"], 0.0), cap.svc_s)
        if not b.running:
            break
    else:
        raise AssertionError("딥 연출이 끝나지 않음 — 승인 대기 교착 의심")
    return b, evs, zmin, pre_blue_z


def test_blue_is_not_automatic():
    """제어 규칙 ①③: 승인 전에는 WORK 를 절대 넘지 않고, 넘으려면 BLUE_CMD 가 선행해야 한다."""
    b, evs, _, pre_blue_z = _run_full("B")
    work_s = T.SERVICE_GEO["B2-09"]["work_s"]
    for cid, s in pre_blue_z.items():
        assert s <= work_s + 1e-9, f"{cid}: 승인 없이 WORK({work_s})를 통과함 — BLUE 자동화 회귀"
    order = [(t, ev, subj) for t, ev, subj in evs if ev in
             ("SERVICE_START", "BLUE_CMD", "SERVICE_DONE")]
    seen = {}
    for t, ev, subj in order:
        seen.setdefault(subj, []).append(ev)
    for cid, seq in seen.items():
        assert seq == ["SERVICE_START", "BLUE_CMD", "SERVICE_DONE"], f"{cid}: {seq}"
    assert len(seen) == 5, "OR 도착 캡슐 5대 전원이 딥을 수행해야 한다"


def test_dip_reaches_operating_room_floor():
    """연출의 핵심: 전원이 하단 z=11.0 까지 내려갔다가 복귀한다."""
    for mode in ("A", "B"):
        _, _, zmin, _ = _run_full(mode)
        assert len(zmin) == 5
        for cid, z in zmin.items():
            assert z == 11.0, f"[{mode}] {cid}: 최저 z={z} (하단 11.0 미도달)"


def test_kpi_unaffected_by_dip():
    """설계 전제: 도착 판정은 레일 인계 시점 — 딥은 KPI 를 늦추지 않는다.
    발행만 연출 종료까지 이어지므로 브리지 정지 시각은 SIM_DONE 보다 뒤다."""
    for mode, makespan in (("A", 74.97), ("B", 86.77)):
        b, evs, _, _ = _run_full(mode)
        done = [t for t, ev, _ in evs if ev == "SIM_DONE"]
        assert done, f"[{mode}] SIM_DONE 미발생"
        mk = max(o.arrive_t for o in b.eng.orders.values())
        assert abs(mk - makespan) <= 0.2, f"[{mode}] makespan {mk} != {makespan}"
        last = max(t for t, ev, _ in evs if ev == "SERVICE_DONE")
        assert last > done[0], f"[{mode}] 연출이 SIM_DONE 전에 끝남 — 발행 유지 로직 확인"
        assert all(c.state == CapsuleState.REMOVED
                   for c in b.eng.capsules.values() if c.svc_bid is None and c.order)
