# 대시보드 다른 PC로 전달하기

## 왜 이 폴더째로 보내야 하나
`rail_dashboard_v11.html` 를 파일로 바로 더블클릭하면(`file://`) 크롬이 마이크 권한을
**매번 다시** 물어봅니다. `start_dashboard.sh` 가 `http://localhost:8791` 로 띄우면
일반 사이트처럼 권한이 저장돼 **최초 1회만** 허용하면 됩니다.

roslibjs·웹폰트도 전부 이 폴더 안에 번들돼 있어 **인터넷 없이도** 동작합니다.
(단, 음성 *입력* = 크롬 SpeechRecognition 은 구글 서버를 써서 폐쇄망에선 안 됩니다.
 TTS·rosbridge·목데이터 재생·Isaac 연동은 전부 오프라인 OK)

## 전달물
`rail_ui/` 폴더 전체 (또는 아래 파일들):
- `rail_dashboard_v11.html`  ← 대시보드 본체
- `roslib.min.js`, `fonts/`  ← 로컬 번들 (건드리지 말 것)
- `start_dashboard.sh`, `install_launcher.sh`

## 받는 PC에서 (Ubuntu 기준)
필요: `python3`, `google-chrome` (또는 `google-chrome-stable`)

```bash
# 1) 폴더 아무 데나 복사 (경로 무관)
# 2) 런처 설치 (한 번만)
bash rail_ui/install_launcher.sh
#   → 바탕화면/앱목록에 "의료 레일 관제 대시보드" 아이콘 생성
# 3) 아이콘 더블클릭  (또는  bash rail_ui/start_dashboard.sh)
# 4) 크롬에서 마이크 권한 "허용" 1회 → 이후 안 물어봄
```

수동 실행:
```bash
cd rail_ui && python3 -m http.server 8791 &
google-chrome --new-window http://localhost:8791/rail_dashboard_v11.html
```

## 마이크 권한이 계속 뜨면
- 반드시 `http://localhost:8791` 로 열렸는지 확인 (주소창). `file://` 이면 안 됨
- 크롬: 설정 → 개인정보 → 사이트 설정 → 마이크 → `http://localhost:8791` **허용** 고정
- 포트를 바꾸면(예 8792) 다른 오리진이라 다시 물어봄 — 8791 유지
