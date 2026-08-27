#!/usr/bin/env python3
# ══════════════════════════════════════════════════════════════════════
# 수술팩 라벨 텍스처 생성기 (산출물 B)
#   - 물리 크기 0.075 x 0.0384 m 에서 카메라(2560x1440, 0.22m)가 읽을 수 있도록
#     텍스처 2048x1049 / 값 폰트 64px 로 역산 확정
#   - 4행 레이아웃 정본 유지, 어노테이션 JSON 동시 출력
# 실행:  python3 make_labels.py --count 200 --out ~/isaac_or_station/textures
#        python3 make_labels.py --demo   (시연용 고정 4장: 관제 오더와 값 일치)
# ══════════════════════════════════════════════════════════════════════
import argparse, json, random, os
from datetime import datetime, timedelta
from PIL import Image, ImageDraw, ImageFont

# ── 역산으로 확정된 상수 (근거는 문서 참조 — 임의 변경 금지) ──
TEX_W, TEX_H = 2048, 1280          # 라벨 종횡비 1.600:1 정확 일치 (왜: 글자 가로 왜곡 방지)
PHYS_W, PHYS_H = 0.120, 0.075      # 실물 크기(m) — 2026-08-27 규격 변경 (구 0.075x0.0384)
F_VALUE, F_FIELD, F_SUB = 77, 55, 68   # 값 / 필드명 / 2행 서브라인 폰트 px
#   실물 환산 @17.07px/mm: 값 4.51mm / 필드명 3.22mm / 서브 3.98mm
#   왜: 라벨이 커진 여유를 여백이 아니라 글자에 써서 카메라 인식 마진을 벌었다

ITEM_CODES = ["STP-GEN","STP-LAP","STP-ORT","STP-OBG","STP-SUT",
              "STP-MIN","STP-PLA","STP-URO","STP-ENT","STP-SPN"]

FONT_CANDIDATES = [   # 왜: 배포판마다 폰트 경로가 달라 순차 탐색합니다
    ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
     "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
    ("/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
     "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"),
]

def load_fonts():
    for bold, reg in FONT_CANDIDATES:
        if os.path.exists(bold) and os.path.exists(reg):
            return bold, reg
    raise SystemExit("폰트를 찾지 못했습니다. 실행: sudo apt install fonts-dejavu-core")

def make_record(rng):
    """랜덤 규칙에 따른 라벨 1건의 데이터 생성"""
    # 포장일: 최근 90일 내 랜덤 날짜 + 랜덤 시각 (분 단위)
    base = datetime(2026, 6, 1) + timedelta(
        days=rng.randint(0, 90), hours=rng.randint(6, 21), minutes=rng.randint(0, 59))
    exp = base + timedelta(days=30)                    # 왜: 규칙상 만료일 = 포장일 + 30일 (2026-08-27 정정)
    return {
        "item_code":  rng.choice(ITEM_CODES),
        "packaging":  base.strftime("%Y-%m-%d %H:%M"),
        "expiration": exp.strftime("%Y-%m-%d %H:%M"),
        "order_id":   f"O-{rng.randint(1, 100)}",
        "delivery":   f"ST-OR{rng.randint(1, 9)}",     # 향후 확장 대비 1~9
        "capsule_id": f"C{rng.randint(1, 99):02d}",    # 2자리 0패딩
    }

def draw_label(rec, bold_path, reg_path):
    """4행 레이아웃 정본대로 라벨 이미지 1장 렌더링"""
    img = Image.new("RGB", (TEX_W, TEX_H), (255, 255, 255))
    d = ImageDraw.Draw(img)
    f_val  = ImageFont.truetype(bold_path, F_VALUE)
    f_sub  = ImageFont.truetype(bold_path, F_SUB)
    f_fld  = ImageFont.truetype(reg_path,  F_FIELD)

    M = 32                                   # 외곽 여백
    d.rectangle([M, M, TEX_W-M, TEX_H-M], outline=(20, 20, 20), width=7)  # 왜: 라벨 경계 = OCR 관심영역 검출 단서
    SPLIT = 820                              # 좌(필드명) / 우(값) 분할 x (폭의 40%)

    # 행 높이: 2행만 2줄이라 더 크게 배분
    rows = [260, 420, 260, 260]
    y = M + 18
    bounds = []
    for h in rows:
        bounds.append((y, y + h)); y += h

    def row(i, field, value, sub=None):
        y0, y1 = bounds[i]
        if i:                                # 행 구분선 (첫 행 제외)
            d.line([M+12, y0, TEX_W-M-12, y0], fill=(170, 170, 170), width=3)
        d.line([SPLIT, y0, SPLIT, y1], fill=(210, 210, 210), width=2)   # 열 구분선
        cy = (y0 + y1) // 2
        d.text((M+40, cy), field, font=f_fld, fill=(90, 90, 90), anchor="lm")
        if sub is None:
            d.text((SPLIT+40, cy), value, font=f_val, fill=(10, 10, 10), anchor="lm")
        else:                                # 2줄 배치 (왜: 31자를 한 줄에 넣으면 글자가 절반으로 줄어 OCR 실패)
            d.text((SPLIT+40, cy-82), value, font=f_sub, fill=(10, 10, 10), anchor="lm")
            d.text((SPLIT+40, cy+82), sub,   font=f_sub, fill=(10, 10, 10), anchor="lm")

    row(0, "Item Code", rec["item_code"])
    row(1, "Packaging Date /\nExpiration Date", rec["packaging"], rec["expiration"])
    row(2, "Order ID", rec["order_id"])
    row(3, "Delivery Add. / Capsule ID", f'{rec["delivery"]} / {rec["capsule_id"]}')
    return img

def preview_as_camera(img, path):
    """검증용: 카메라가 실제로 보게 될 크기(756x473)로 축소 저장 — 눈으로 가독성 확인"""
    img.resize((756, 473), Image.LANCZOS).save(path)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=200)
    ap.add_argument("--out", default=os.path.expanduser("~/isaac_or_station/textures"))
    ap.add_argument("--seed", type=int, default=1136)
    ap.add_argument("--demo", action="store_true", help="관제 오더와 일치하는 시연용 4장")
    a = ap.parse_args()

    bold, reg = load_fonts()
    os.makedirs(a.out, exist_ok=True)
    rng = random.Random(a.seed)               # 왜: 시드 고정 = 언제 돌려도 같은 200장(재현성)
    manifest = []

    if a.demo:   # 시연용: 실제 관제 오더 O-1(멸균 공급, ST-OR1, C05)에 맞춘 고정 데이터
        recs = []
        for code in ["STP-GEN", "STP-LAP", "STP-ORT", "STP-SUT"]:
            r = make_record(rng)
            r.update({"item_code": code, "order_id": "O-1",
                      "delivery": "ST-OR1", "capsule_id": "C05"})
            recs.append(r)
    else:
        recs = [make_record(rng) for _ in range(a.count)]

    prefix = "demo" if a.demo else "label"
    for i, rec in enumerate(recs, 1):
        name = f"{prefix}_{i:04d}"
        img = draw_label(rec, bold, reg)
        img.save(os.path.join(a.out, name + ".png"))
        rec_out = dict(rec)
        rec_out.update({"file": name + ".png",
                        "texture_px": [TEX_W, TEX_H],
                        "physical_m": [PHYS_W, PHYS_H]})
        manifest.append(rec_out)
        if i == 1:                            # 첫 장은 카메라 시점 미리보기도 저장
            preview_as_camera(img, os.path.join(a.out, name + "_camera_preview.png"))

    # 어노테이션: 인식 결과 정답지로 그대로 사용 (10단계 검증에 씀)
    with open(os.path.join(a.out, f"{prefix}_annotations.json"), "w", encoding="utf-8") as f:
        json.dump({"count": len(manifest), "seed": a.seed, "records": manifest},
                  f, ensure_ascii=False, indent=1)
    print(f"[2단계] {len(manifest)}장 생성 -> {a.out}")
    print(f"  어노테이션: {prefix}_annotations.json")
    print(f"  카메라 시점 미리보기: {prefix}_0001_camera_preview.png (이게 읽히면 성공)")

if __name__ == "__main__":
    main()