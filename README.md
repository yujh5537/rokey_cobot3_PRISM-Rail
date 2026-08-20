# 의료 레일 관제 디지털 트윈 프로젝트

## 담당
| 파트 | 담당자 | 폴더 |
|---|---|---|---|
| A. 관제 코어 | 남현지 | `rail_control_core/` |
| B. 씬 빌더 | 한석형 | `rail_sim_assets/` |
| C. 연동 | 정희진 | `rail_bridge/` |
| D. UI·QA | 김세은 | `rail_ui/` |

## 공용 인터페이스
`rail_msgs/` 는 4개 파트가 전부 사용하는 공용 데이터 서식입니다.
**이 폴더를 변경하는 PR은 D(김세은)의 승인이 반드시 필요합니다.**
자세한 필드 정의는 `docs/interface_spec.md` 참고.

## 브랜치 규칙
1. `main`에 직접 push 금지 — 반드시 PR로만 병합
2. `rail_msgs/` 변경 PR은 D 승인 필수
3. 매일 저녁 각자 `main`을 자기 브랜치로 pull 하기

## 시작하기
```bash
git clone <이 저장소 주소>
cd medical-rail-twin
git checkout feat/ui-dashboard   # 각자 자기 브랜치로 이동
```
