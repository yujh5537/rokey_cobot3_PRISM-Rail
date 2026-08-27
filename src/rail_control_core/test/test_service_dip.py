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
    """딥 연출이 끝나 브리지가 스스로 멈출 때까지(또는 모드 A 데드락까지) 돌리고
    관측치를 모은다. 모드 A(meet_pass OFF)는 C05/C08 이 단선에서 교착돼 완주하지
    않으므로 DEADLOCK 이벤트가 나오면 거기서 멈춘다 — 콘보이 딥은 그 전에 끝난다."""
    b = Bridge(mode, autostart=True)
    evs, zmin, pre_blue_z = [], {}, {}
    deadlocked = False
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
        if any(ev == "DEADLOCK" for _, ev, _ in evs):
            deadlocked = True
            break
        if not b.running:
            break
    else:
        raise AssertionError("딥 연출이 끝나지 않음 — 승인 대기 교착 의심")
    return b, evs, zmin, pre_blue_z, deadlocked


def test_blue_is_not_automatic():
    """제어 규칙 ①③: 승인 전에는 WORK 를 절대 넘지 않고, 넘으려면 BLUE_CMD 가 선행해야 한다."""
    b, evs, _, pre_blue_z, _ = _run_full("B")
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
    """연출의 핵심: OR 에 도착한 캡슐 전원이 하단 z=11.0 까지 내려갔다가 복귀한다.
    모드 B 는 5대(콘보이 4 + O-1 공급 C05), 모드 A(meet_pass OFF)는 C05 가 단선
    교착으로 OR1 에 못 가므로 콘보이 4대만 — 콘보이 딥은 데드락 선언 전에 완주한다."""
    for mode, n_dip in (("A", 4), ("B", 5)):
        _, _, zmin, _, deadlocked = _run_full(mode)
        assert (mode == "A") == deadlocked, f"[{mode}] 데드락 기대={mode=='A'} 실제={deadlocked}"
        assert len(zmin) == n_dip, f"[{mode}] 딥 수행 {len(zmin)} != {n_dip}"
        for cid, z in zmin.items():
            assert z == 11.0, f"[{mode}] {cid}: 최저 z={z} (하단 11.0 미도달)"


def test_kpi_unaffected_by_dip():
    """설계 전제: 도착 판정은 레일 인계 시점 — 딥은 KPI 를 늦추지 않는다.
    발행만 연출 종료까지 이어지므로 브리지 정지 시각은 SIM_DONE 보다 뒤다.
    (모드 A 는 meet_pass OFF 로 완주하지 않으므로 SIM_DONE 이 없다 — 모드 B 만 검증)"""
    b, evs, _, _, _ = _run_full("B")
    done = [t for t, ev, _ in evs if ev == "SIM_DONE"]
    assert done, "[B] SIM_DONE 미발생"
    mk = max(o.arrive_t for o in b.eng.orders.values())
    assert abs(mk - 86.77) <= 0.2, f"[B] makespan {mk} != 86.77"
    last = max(t for t, ev, _ in evs if ev == "SERVICE_DONE")
    assert last > done[0], "[B] 연출이 SIM_DONE 전에 끝남 — 발행 유지 로직 확인"
    assert all(c.state == CapsuleState.REMOVED
               for c in b.eng.capsules.values() if c.svc_bid is None and c.order)


def test_mode_a_deadlocks_without_meet_pass():
    """모드 A(FCFS 비교군)는 R13 교행 OFF 라 C05(O-1 공급)·C08(O-5 회수)이 단선에서
    정면 교착 → DEADLOCK 이벤트가 나오고 완주하지 않는다. 대시보드 데드락 시연과 일치."""
    b, evs, _, _, deadlocked = _run_full("A")
    assert deadlocked, "모드 A 인데 데드락이 발생하지 않음 (meet_pass 강제 OFF 회귀)"
    dl = [(t, s) for t, ev, s in evs if ev == "DEADLOCK"]
    assert dl, "DEADLOCK 이벤트 미발행"
    assert not any(ev == "SIM_DONE" for _, ev, _ in evs), "데드락인데 SIM_DONE 발생"
    assert not b.running or any(
        c.state.value == "YIELD_WAIT" for c in b.eng.capsules.values())
