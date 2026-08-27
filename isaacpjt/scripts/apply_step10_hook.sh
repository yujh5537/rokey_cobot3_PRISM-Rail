#!/bin/bash
# ============================================================
# [10-3] 오케스트레이터 카메라 캡처 훅 — 적용 스크립트
#
# 무엇을 하는가:
#   or_unload_orchestrator.py 의 큐에서 카메라 홀드 지점을
#   "정지 안정화 -> 촬영 -> 인식 대기" 3단계로 교체한다.
#
#   변경 전: ("goto","camera",3.0), ("hold",None,2.0)
#   변경 후: ("goto","camera",3.0), ("hold",None,1.0),   <- 팔 진동 소멸 대기
#            ("capture",None,0),                          <- 촬영 + OCR 통지
#            ("hold",None,1.5)                            <- 인식 여유
#
# 왜 캡처 전에 1초를 쉬는가:
#   내성 시험에서 모션 블러가 capsule_id 인식을 깨뜨렸다.
#   8단계의 "도착 진동" 문제와 같은 뿌리라, 팔이 완전히 멎은 뒤 찍는다.
#
# 안전장치:
#   캡처가 실패해도 하역 시퀀스는 그대로 완주한다(try/except).
#   관제 게이트는 DOOR_CLOSED 만 보므로 OCR 실패가 시연을 멈추지 않는다.
#
# 사전 조건: 9단계(파지) 완료, or_label_capture.py 가 같은 폴더에 존재
# 실행: bash apply_step10_hook.sh
# 되돌리기: cp or_unload_orchestrator.py.step10bak or_unload_orchestrator.py
# ============================================================
set -e
DIR=/home/rokey/rokey_cobot3/isaacpjt/scripts
P=$DIR/or_unload_orchestrator.py

[ -f "$P" ] || { echo "ERROR: $P 없음"; exit 1; }
[ -f "$DIR/or_label_capture.py" ] || { echo "ERROR: or_label_capture.py 없음"; exit 1; }
cp "$P" "$P.step10bak"
echo "백업: $P.step10bak"

python3 - <<'PYEOF'
P = "/home/rokey/rokey_cobot3/isaacpjt/scripts/or_unload_orchestrator.py"
s = open(P).read()

if "or_label_capture" in s:
    print("이미 적용됨 — 종료"); raise SystemExit(0)

# (1) 캡처 모듈 임포트
old = "import or_pack_attach as _A"
new = '''import or_pack_attach as _A
try:
    import or_label_capture as _C          # 10단계: 카메라 캡처 + OCR 통지
    print("[cap] label capture module linked")
except Exception as _e:
    _C = None
    print("[cap] capture module unavailable:", type(_e).__name__)'''
assert old in s, "attach import not found"
s = s.replace(old, new)

# (2) 큐: 카메라 홀드 -> 안정화 + 촬영 + 인식대기
old = '''        S["queue"] = [("goto","grasp",2.5), ("grip",True,0.8), ("attach",None,0),
                      ("goto","camera",3.0), ("hold",None,2.0),
                      ("goto","tray",3.0),   ("release",None,0), ("grip",False,0.8)]'''
new = '''        S["queue"] = [("goto","grasp",2.5), ("grip",True,0.8), ("attach",None,0),
                      ("goto","camera",3.0), ("hold",None,1.0),   # 진동 소멸 대기
                      ("capture",None,0),                          # 촬영 + OCR 통지
                      ("hold",None,1.5),                           # 인식 여유
                      ("goto","tray",3.0),   ("release",None,0), ("grip",False,0.8)]'''
assert old in s, "queue block not found (9단계 attach 패치가 적용됐는지 확인)"
s = s.replace(old, new)

# (3) capture 스텝 처리
old = '''            elif kind == "attach": _A.attach()'''
new = '''            elif kind == "capture":
                # 실패해도 시퀀스는 계속된다 — 하역 완주가 OCR 보다 우선
                try:
                    if _C is None:
                        print("[cap] skipped (module unavailable)")
                    else:
                        _C.capture2(CAP2ID.get(cap, "C05"))
                        print(f"[cap] capture requested for {CAP2ID.get(cap, cap)}")
                except Exception as e:
                    print("[cap] capture failed (sequence continues):", e)
            elif kind == "attach": _A.attach()'''
assert old in s, "attach step not found"
s = s.replace(old, new)

open(P, "w").write(s)
import py_compile; py_compile.compile(P, doraise=True)
print("10-3 훅 적용 완료, SYNTAX OK")
PYEOF

echo ""
echo "확인:"
grep -n "or_label_capture\|capture requested\|hold\",None,1" "$P" | head
