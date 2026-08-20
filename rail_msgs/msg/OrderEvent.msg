# 이벤트, 코어 → UI·Isaac
# 변경 시 D(김세은)의 PR 승인 필수

float32 sim_time
string  order_id
uint8   priority
uint8   event_type       # 0생성 1배차 2양보 3대피 4완주허용 5도착 6인계 7복귀 8완료
string  src_node         # "N1"
string  dst_node         # "N4"
float32 due_time         # 마감시각(EDD 정렬용)
string  reason           # "P1>P2, S1 충돌"
