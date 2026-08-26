#!/usr/bin/env python3
# [10-1 진단] 값열 크롭 x0 × psm × whitelist 스윕 -> 최적 조합 탐색
import sys, os, json, itertools
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cv2, pytesseract
import or_label_ocr_node as N

WL = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-:/."   # 공백 넣으면 -c 인자가 잘리니 제외
BASE = os.path.dirname(os.path.dirname(N.CFG["labels_json"]))
GT = {it["file"]: it for it in json.load(open(N.CFG["labels_json"]))["items"]
      if it["split"] == "demo"}
DBG = "/home/rokey/rokey_cobot3/isaacpjt/output/ocr_debug"
os.makedirs(DBG, exist_ok=True)

def prep(path, x0, shrink=476):
    img = cv2.imread(path)
    s = shrink / img.shape[1]
    img = cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
    roi, _ = N.crop_label(img)
    if x0 > 0:
        roi = roi[:, int(roi.shape[1] * x0):]      # 필드명 열 버리고 값 열만
    g = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    sc = max(1.0, 1600.0 / g.shape[1])
    g = cv2.resize(g, None, fx=sc, fy=sc, interpolation=cv2.INTER_CUBIC)
    g = cv2.GaussianBlur(g, (3, 3), 0)
    _, g = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return g

def run(g, psm, wl):
    cfg = f"--oem 3 --psm {psm}" + (f" -c tessedit_char_whitelist={WL}" if wl else "")
    return pytesseract.image_to_string(g, config=cfg), cfg

results = []
for x0, psm, wl in itertools.product([0.0, 0.40], [4, 6, 11, 12], [False, True]):
    hit = {k: 0 for k in N.FIELDS}
    for f, gt in GT.items():
        txt, _ = run(prep(os.path.join(BASE, "textures", f), x0), psm, wl)
        fl = N.parse(txt)
        for k in N.FIELDS:
            if fl.get(k) == gt[k]: hit[k] += 1
    key3 = hit["order_id"] + hit["delivery_add"] + hit["capsule_id"]   # 시연 핵심 3필드
    results.append((key3, sum(hit.values()), x0, psm, wl, dict(hit)))

results.sort(key=lambda r: (r[0], r[1]), reverse=True)
print(f"{'핵심3':>5} {'전체':>6} {'x0':>5} {'psm':>4} {'WL':>6}  필드별(4장 중)")
for k3, tot, x0, psm, wl, h in results:
    print(f"{k3:>3}/12 {tot:>3}/24 {x0:>5} {psm:>4} {str(wl):>6}  "
          + " ".join(f"{k[:4]}={v}" for k, v in h.items()))

k3, tot, x0, psm, wl, h = results[0]
print(f"\n=== BEST: x0={x0} psm={psm} whitelist={wl} (핵심3 {k3}/12, 전체 {tot}/24) ===")
g = prep(os.path.join(BASE, "textures", "demo/demo_01.png"), x0)
cv2.imwrite(f"{DBG}/best_prep.png", g)
txt, cfg = run(g, psm, wl)
print(txt)
print("전처리 이미지:", f"{DBG}/best_prep.png", "| config:", cfg)
