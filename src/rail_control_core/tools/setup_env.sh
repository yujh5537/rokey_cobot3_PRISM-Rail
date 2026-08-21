#!/usr/bin/env bash
# =============================================================================
#  setup_env.sh — 의료 레일 관제 디지털 트윈 팀 환경 설정 (개인 PC / GPU PC 공용)
#
#  팀 확정값
#    ROS2 배포판     : Jazzy (Ubuntu 24.04)
#    ROS_DOMAIN_ID   : 136
#    RMW             : rmw_fastrtps_cpp
#    유선 대역       : 10.10.0.1 ~ 10.10.0.4
#
#  사용법
#    chmod +x setup_env.sh
#    ./setup_env.sh 3            # ← 본인 조원 번호(1~4). 10.10.0.3 을 쓴다는 뜻
#    ./setup_env.sh 3 --fix      # 낡은 충돌 설정을 자동으로 주석 처리
#    ./setup_env.sh 3 --check    # 아무것도 고치지 않고 진단만
#    source ~/.bashrc
# =============================================================================
set -euo pipefail

MY_NO=""
AUTO_FIX=0
CHECK_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --fix)   AUTO_FIX=1 ;;
    --check) CHECK_ONLY=1 ;;
    [1-4])   MY_NO="$arg" ;;
    *) echo "알 수 없는 인자: $arg"; exit 1 ;;
  esac
done
if [[ -z "$MY_NO" ]]; then
  echo "사용법: $0 <조원번호 1~4> [--fix] [--check]"
  echo "  예) 3번이면:  $0 3          → 본인 IP는 10.10.0.3"
  echo "      충돌 자동정리:  $0 3 --fix"
  echo "      진단만:         $0 3 --check"
  exit 1
fi
MY_IP="10.10.0.${MY_NO}"

ROS_DISTRO_NAME="jazzy"
DOMAIN_ID=136

echo "▶ 조원 ${MY_NO}번 / 본인 IP ${MY_IP} / 도메인 ${DOMAIN_ID} / ROS2 ${ROS_DISTRO_NAME}"
echo

# -----------------------------------------------------------------------------
# 0. ROS2 설치 확인
# -----------------------------------------------------------------------------
if [[ ! -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
  echo "❌ /opt/ros/${ROS_DISTRO_NAME}/setup.bash 가 없습니다."
  echo "   ROS2 ${ROS_DISTRO_NAME} 를 먼저 설치하세요."
  exit 1
fi
echo "✅ ROS2 ${ROS_DISTRO_NAME} 확인"

# -----------------------------------------------------------------------------
# 0-B. 낡은 설정 충돌 검사  ★중요★
#
#  왜 필요한가:
#    이 스크립트는 .bashrc 끝에 자기 블록을 붙입니다. 그래서 앞쪽에
#    ROS_DOMAIN_ID=50 같은 낡은 줄이 있어도 "뒤에 오는 것이 이긴다"는
#    셸 규칙 덕분에 동작 자체는 정상입니다.
#
#    문제는 두 가지입니다.
#      (1) 파일을 열어보면 50과 136이 같이 보여서 나중에 반드시 헷갈립니다
#      (2) 다른 스크립트가 우리 블록 뒤에서 또 export 하면 그때는 낡은 값이 이깁니다
#
#    그래서 "지금 당장 고장은 아니지만 반드시 정리해야 할 것"으로 경고합니다.
# -----------------------------------------------------------------------------
BEGIN="# >>> RAIL_TWIN ROS2 SETUP >>>"
END="# <<< RAIL_TWIN ROS2 SETUP <<<"

CONFLICTS=""
CONFLICT_LINES=""

scan_conflicts() {
  [[ -f ~/.bashrc ]] || return 0

  # 우리 마커 블록을 제외한 나머지에서만 찾습니다.
  # (블록 안의 우리 설정을 충돌로 잡으면 안 되므로)
  local tmp
  tmp=$(mktemp)
  awk -v b="$BEGIN" -v e="$END" '
    index($0,b) { inblock=1 }
    !inblock    { print NR "\t" $0 }
    index($0,e) { inblock=0 }
  ' ~/.bashrc > "$tmp"

  # 주석이 아닌 줄에서 충돌 후보를 찾습니다
  local patterns='ROS_DOMAIN_ID|RMW_IMPLEMENTATION|FASTRTPS_DEFAULT_PROFILES_FILE|source[[:space:]]+/opt/ros/'
  CONFLICT_LINES=$(grep -nE "$patterns" "$tmp" \
    | sed 's/^[0-9]*://' \
    | awk -F'\t' '
        $2 ~ /^[[:space:]]*#/  { next }   # 주석 줄 제외
        $2 ~ /^[[:space:]]*echo/ { next } # 단순 출력 줄 제외 (무해)
        { print $1 "\t" $2 }
      ' || true)

  rm -f "$tmp"
  [[ -n "$CONFLICT_LINES" ]] && CONFLICTS=1
  return 0
}

scan_conflicts

if [[ -n "$CONFLICTS" ]]; then
  echo
  echo "⚠️  ~/.bashrc 에 이 스크립트 블록 '바깥'의 ROS 설정이 있습니다:"
  echo "─────────────────────────────────────────────────────────────"
  while IFS=$'\t' read -r ln txt; do
    [[ -z "$ln" ]] && continue
    # 값이 우리와 다르면 빨간 표시
    marker="  "
    if echo "$txt" | grep -qE 'ROS_DOMAIN_ID[[:space:]]*=[[:space:]]*'"$DOMAIN_ID"'([^0-9]|$)'; then
      marker="✅"   # 값은 같음 (중복일 뿐)
    elif echo "$txt" | grep -q 'ROS_DOMAIN_ID'; then
      marker="❌"   # 도메인 값이 다름 — 가장 위험
    elif echo "$txt" | grep -qE "source[[:space:]]+/opt/ros/${ROS_DISTRO_NAME}"; then
      marker="✅"
    elif echo "$txt" | grep -q 'source[[:space:]]*/opt/ros/'; then
      marker="❌"   # 다른 배포판 소싱 — Humble 등
    elif echo "$txt" | grep -q 'RMW_IMPLEMENTATION[[:space:]]*=[[:space:]]*rmw_fastrtps_cpp'; then
      marker="✅"
    elif echo "$txt" | grep -q 'RMW_IMPLEMENTATION'; then
      marker="❌"   # 다른 RMW (CycloneDDS 등) — 화이트리스트가 안 먹습니다
    elif echo "$txt" | grep -q 'fastdds_whitelist.xml'; then
      marker="✅"
    elif echo "$txt" | grep -q 'FASTRTPS_DEFAULT_PROFILES_FILE'; then
      marker="❌"   # 다른 프로파일 파일을 가리킴
    fi
    printf "  %s  %4s행 │ %s\n" "$marker" "$ln" "$txt"
  done <<< "$CONFLICT_LINES"
  echo "─────────────────────────────────────────────────────────────"
  echo "  ❌ = 우리 확정값과 다름 (반드시 정리 필요)"
  echo "  ✅ = 값은 같음 (중복일 뿐, 정리하면 깔끔)"
  echo

  DO_FIX=0
  if [[ "$CHECK_ONLY" == "1" ]]; then
    echo "  --check 모드이므로 아무것도 변경하지 않습니다."
    echo "  정리하려면:  $0 $MY_NO --fix"
  elif [[ "$AUTO_FIX" == "1" ]]; then
    DO_FIX=1
  elif [[ -t 0 ]]; then
    read -r -p "  이 줄들을 주석 처리할까요? (원본은 백업됩니다) [y/N] " ans
    [[ "$ans" =~ ^[Yy]$ ]] && DO_FIX=1
  else
    echo "  (비대화 실행이므로 자동 정리하지 않습니다. --fix 를 붙이면 정리합니다)"
  fi

  if [[ "$DO_FIX" == "1" ]]; then
    BAK=~/.bashrc.bak.$(date +%Y%m%d_%H%M%S)
    cp ~/.bashrc "$BAK"
    while IFS=$'\t' read -r ln txt; do
      [[ -z "$ln" ]] && continue
      # 해당 행 앞에 주석 + 사유를 붙입니다
      sed -i "${ln}s|^|#[RAIL_TWIN 정리 $(date +%F)] |" ~/.bashrc
    done < <(echo "$CONFLICT_LINES" | tac)   # 뒤에서부터 처리해야 행번호가 안 밀림
    echo "  ✅ 정리 완료. 백업: $BAK"
    echo "     되돌리려면:  cp $BAK ~/.bashrc"
  elif [[ "$CHECK_ONLY" != "1" ]]; then
    echo "  ℹ️  건너뜁니다. 우리 블록이 .bashrc 맨 뒤에 오므로 지금은 동작합니다."
    echo "     다만 나중에 헷갈리니 직접 정리해 두세요."
  fi
fi

if [[ "$CHECK_ONLY" == "1" ]]; then
  echo
  echo "▶ --check 모드 종료 (변경 없음)"
  exit 0
fi

# -----------------------------------------------------------------------------
# 1. FastDDS 화이트리스트 (유선망만 사용하도록 강제)
# -----------------------------------------------------------------------------
mkdir -p ~/.ros
cat > ~/.ros/fastdds_whitelist.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>wired_udp</transport_id>
        <type>UDPv4</type>
        <interfaceWhiteList>
          <address>127.0.0.1</address>
          <address>10.10.0.1</address>
          <address>10.10.0.2</address>
          <address>10.10.0.3</address>
          <address>10.10.0.4</address>
        </interfaceWhiteList>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="wired_only" is_default_profile="true">
      <rtps>
        <userTransports>
          <transport_id>wired_udp</transport_id>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
EOF
echo "✅ ~/.ros/fastdds_whitelist.xml 생성 (127.0.0.1 + 10.10.0.1~4)"

# -----------------------------------------------------------------------------
# 2. .bashrc 블록 갱신 (중복 추가 방지 — 마커 사이만 교체)
# -----------------------------------------------------------------------------
# 기존 블록 제거
if grep -qF "$BEGIN" ~/.bashrc 2>/dev/null; then
  sed -i "/${BEGIN}/,/${END}/d" ~/.bashrc
  echo "ℹ️  기존 설정 블록을 교체합니다"
fi

cat >> ~/.bashrc << EOF
${BEGIN}
# 의료 레일 관제 디지털 트윈 — 팀 공통 ROS2 환경
# ⚠️ 4명 전원이 동일해야 합니다. 하나라도 다르면 그 PC만 조용히 통신이 안 됩니다.

export ROS_DOMAIN_ID=${DOMAIN_ID}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/.ros/fastdds_whitelist.xml

source /opt/ros/${ROS_DISTRO_NAME}/setup.bash

# 워크스페이스 (아직 빌드 전이면 조용히 건너뜀)
[ -f \$HOME/cobot3_ws/install/setup.bash ] && source \$HOME/cobot3_ws/install/setup.bash

# 현재 설정 표시
rosenv() {
  echo "ROS_DISTRO            = \$ROS_DISTRO"
  echo "ROS_DOMAIN_ID         = \$ROS_DOMAIN_ID"
  echo "RMW_IMPLEMENTATION    = \$RMW_IMPLEMENTATION"
  echo "FASTRTPS_PROFILES     = \$FASTRTPS_DEFAULT_PROFILES_FILE"
  echo "내 유선 IP            = \$(hostname -I | tr ' ' '\\n' | grep '^10\\.10\\.0\\.' || echo '❌ 유선 IP 없음')"
}
${END}
EOF
echo "✅ ~/.bashrc 갱신 (마커 블록 방식 — 여러 번 실행해도 중복 안 됨)"

# -----------------------------------------------------------------------------
# 3. 방화벽 해제
# -----------------------------------------------------------------------------
if command -v ufw >/dev/null 2>&1; then
  sudo ufw disable >/dev/null 2>&1 || true
  echo "✅ 방화벽 비활성화: $(sudo ufw status | head -1)"
fi

# -----------------------------------------------------------------------------
# 4. 빌드 도구
# -----------------------------------------------------------------------------
if ! command -v colcon >/dev/null 2>&1; then
  echo "▶ colcon 설치 중..."
  sudo apt update -qq
  sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-pip
fi
python3 -c "import yaml" 2>/dev/null || pip3 install pyyaml --break-system-packages -q
echo "✅ 빌드 도구 확인"

# -----------------------------------------------------------------------------
# 5. 진단
# -----------------------------------------------------------------------------
echo
echo "═══════════════════ 진 단 ═══════════════════"
ACTUAL_IP=$(hostname -I | tr ' ' '\n' | grep '^10\.10\.0\.' || true)
if [[ -z "$ACTUAL_IP" ]]; then
  echo "❌ 유선 IP(10.10.0.x)가 없습니다."
  echo "   설정 → 네트워크 → 유선 → IPv4 → Manual"
  echo "   Address ${MY_IP} / Netmask 255.255.255.0 / Gateway 비움"
  echo "   저장 후 유선 토글을 OFF → ON 하세요."
elif [[ "$ACTUAL_IP" != "$MY_IP" ]]; then
  echo "⚠️  실제 IP(${ACTUAL_IP})와 지정한 IP(${MY_IP})가 다릅니다."
else
  echo "✅ 유선 IP: ${ACTUAL_IP}"
fi

echo
echo "▶ 다른 조원 PC 연결 확인:"
for i in 1 2 3 4; do
  [[ "$i" == "$MY_NO" ]] && continue
  if ping -c1 -W1 "10.10.0.${i}" >/dev/null 2>&1; then
    echo "   ✅ 10.10.0.${i} 응답"
  else
    echo "   ⚠️  10.10.0.${i} 무응답 (아직 안 켰거나 IP 미설정일 수 있음)"
  fi
done

echo
echo "═════════════════ 다음 단계 ═════════════════"
echo "  1) source ~/.bashrc"
echo "  2) rosenv                      # 설정값 확인"
echo "  3) 내 PC:    ros2 run demo_nodes_cpp talker"
echo "     다른 PC:  ros2 run demo_nodes_py listener"
echo "     → listener에 'I heard' 가 찍히면 통신 성공"
echo
