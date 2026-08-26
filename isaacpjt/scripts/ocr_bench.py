#!/usr/bin/env python3
# 200장 데이터셋에 대해 6필드 정확도 측정. --shrink 476 = 카메라 footprint 조건 흉내
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import or_label_ocr_node as N
shrink = int(sys.argv[sys.argv.index("--shrink") + 1]) if "--shrink" in sys.argv else None
d = json.load(open(N.CFG["labels_json"]))
base = os.path.dirname(os.path.dirname(N.CFG["labels_json"]))          # ~/isaac_or_station
hit = {k: 0 for k in N.FIELDS}; n = 0; perfect = 0
for it in d["items"]:
    if it["split"] != "dataset": continue
    r = N.recognize("C00", os.path.join(base, "textures", it["file"]), shrink)
    n += 1; ok_all = True
    for k in N.FIELDS:
        if r["fields"].get(k) == it[k]: hit[k] += 1
        else: ok_all = False
    perfect += ok_all
    if n % 50 == 0: print(f"  {n} done...")
print(f"=== {n}장, shrink={shrink} ===")
for k in N.FIELDS: print(f"  {k:16s} {hit[k]/n*100:5.1f}%")
print(f"  전 필드 일치: {perfect/n*100:.1f}%")
