#!/usr/bin/env bash
# 대시보드를 file://로 직접 열지 않고 localhost 웹서버로 띄운 뒤 Chrome으로 여는 런처.
# file:// 오리진은 Chrome이 마이크 권한을 영구 저장하지 않아 매번 다시 물어보는데,
# http://localhost는 일반 사이트처럼 권한이 저장되어 최초 1회만 허용하면 된다.

set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT=8791
PAGE="${1:-rail_dashboard_v11.html}"
URL="http://localhost:${PORT}/${PAGE}"

# 이미 그 포트에 서버가 떠 있으면 새로 띄우지 않는다 (재실행/중복 방지)
if ! curl -s -o /dev/null "http://localhost:${PORT}/"; then
  cd "$DIR"
  nohup python3 -m http.server "$PORT" >/tmp/rail_dashboard_server.log 2>&1 &
  disown
  sleep 0.5
fi

if command -v google-chrome-stable >/dev/null 2>&1; then
  CHROME=google-chrome-stable
elif command -v google-chrome >/dev/null 2>&1; then
  CHROME=google-chrome
else
  echo "google-chrome을 찾지 못했습니다. 브라우저에서 직접 열어주세요: $URL"
  exit 1
fi

"$CHROME" --new-window "$URL" >/dev/null 2>&1 &
disown
