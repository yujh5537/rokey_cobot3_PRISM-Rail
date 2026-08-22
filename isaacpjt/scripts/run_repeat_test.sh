#!/bin/bash
# 10회 연속 안정성 자동화 테스트 스크립트

echo "🚀 [10회 연속 안정성 테스트 시작]"
# 빠른 검증을 위해 4배속 설정
ros2 param set /control_core_node speed_scale 4.0

SUCCESS_COUNT=0
TOTAL_RUNS=10

for i in $(seq 1 $TOTAL_RUNS); do
    echo "=========================================="
    echo "▶ [회차 $i / $TOTAL_RUNS] 시뮬레이션 시작"
    echo "=========================================="
    
    # 1. 시뮬레이션 시작
    ros2 service call /sim_start std_srvs/srv/Trigger "{}" > /dev/null 2>&1
    
    # 2. 4배속 기준 1사이클 완료 대기 (약 20~25초)
    # /kpi 토픽이 1회 발행될 때까지 대기
    timeout 30s ros2 topic echo /kpi --once > /dev/null 2>&1
    
    if [ $? -eq 0 ]; then
        echo "✅ [$i회차] 정상 완주 성공 (KPI 수신 완료)"
        SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
    else
        echo "❌ [$i회차] 시간 초과 또는 오류 발생!"
    fi
    
    # 3. 다음 회차를 위한 시뮬레이션 리셋
    echo "🔄 리셋 수행 중..."
    ros2 service call /sim_reset std_srvs/srv/Trigger "{}" > /dev/null 2>&1
    sleep 2
done

echo "=========================================="
echo "🏁 [테스트 결과] 총 $TOTAL_RUNS 회 중 $SUCCESS_COUNT 회 무오류 성공!"
echo "=========================================="