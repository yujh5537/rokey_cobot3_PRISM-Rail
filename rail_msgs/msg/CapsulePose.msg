# ~30Hz, 코어 → UI·Isaac
# 변경 시 D(김세은)의 PR 승인 필수

string  capsule_id       # "O-1" ~ "O-4"
uint8   priority         # 0=P0, 1=P1, 2=P2, 3=P3
uint8   fsm_state        # 0=유휴 1=적재 2=주행 3=하역인계 4=양보대기 5=대피 6=완주허용
string  block_id         # 현재 블록, 예 "S1"
float32 progress         # 0.0~1.0 블록 내 진행률
float32 sim_time         # 시뮬 시작 후 경과초
