# 이벤트, 코어 → UI·Isaac
# 변경 시 D(김세은)의 PR 승인 필수

float32 sim_time
string  block_id
uint8   state             # 0=여유 1=예약 2=점유
string  owner_capsule_id  # 점유/예약 주체 (없으면 "")
float32 reserved_from
float32 reserved_to
