#!/usr/bin/env bash
# 다른 컴퓨터로 폴더를 옮긴 뒤 딱 한 번 실행하는 설치 스크립트.
# 지금 이 rail_ui 폴더의 실제 경로를 기준으로 .desktop 런처를 새로 만들어
# 바탕화면 + 앱 목록에 설치한다. (경로가 바뀌어도 항상 다시 이거 한 번만 돌리면 됨)

set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME="의료레일_대시보드.desktop"

command -v python3 >/dev/null 2>&1 || echo "⚠ python3이 없습니다 — 설치해주세요."
if ! command -v google-chrome >/dev/null 2>&1 && ! command -v google-chrome-stable >/dev/null 2>&1; then
  echo "⚠ google-chrome이 없습니다 — 설치해주세요."
fi

cat > "$DIR/$NAME" <<EOF
[Desktop Entry]
Type=Application
Name=의료 레일 관제 대시보드
Comment=localhost로 대시보드를 열어 마이크 권한이 매번 다시 물어보지 않게 함
Exec=$DIR/start_dashboard.sh
Icon=web-browser
Terminal=false
Categories=Utility;
EOF
chmod +x "$DIR/$NAME" "$DIR/start_dashboard.sh"

mkdir -p "$HOME/Desktop" "$HOME/.local/share/applications"
cp "$DIR/$NAME" "$HOME/Desktop/$NAME"
cp "$DIR/$NAME" "$HOME/.local/share/applications/$NAME"
chmod +x "$HOME/Desktop/$NAME" "$HOME/.local/share/applications/$NAME"

if command -v gio >/dev/null 2>&1; then
  gio set "$DIR/$NAME" "metadata::trusted" true 2>/dev/null || true
  gio set "$HOME/Desktop/$NAME" "metadata::trusted" true 2>/dev/null || true
  gio set "$HOME/.local/share/applications/$NAME" "metadata::trusted" true 2>/dev/null || true
fi
command -v update-desktop-database >/dev/null 2>&1 && update-desktop-database "$HOME/.local/share/applications" 2>/dev/null

echo "설치 완료. 바탕화면의 '$NAME' 아이콘을 더블클릭하세요."
echo "(더블클릭이 텍스트로 열리면 아이콘 우클릭 -> '실행 허용'을 먼저 한 번 눌러주세요)"
