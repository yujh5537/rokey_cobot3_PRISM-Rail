#!/usr/bin/env python3
# ============================================================
# [Step 10-1] 라벨 OCR 노드 — 시연 대상 demo_0001.png 최적화 (정본)
#
# 파이프라인:
#   검은 테두리 4점 검출 -> 원근 보정(rectify, 1600x819 정규화)
#   -> 격자선으로 4개 행 분리 -> 값 열만 -> 필드별 psm/화이트리스트로 셀 판독
#   -> 4행은 잉크 경계 트림 후 좌(배송지)/우(캡슐ID) 분리 + 크롭 투표
#   -> 빈 필드는 전체 이미지 변형 캐스케이드로 보완
#
# 검증된 작동 범위 (ocr_robust.py, 2026-08-27 / 라벨 120x75mm):
#   yaw ±30도 / pitch 15도까지 / 라벨 폭 550px 이상
#   조명(어두움·반사)·경미한 블러 내성 확보. 심한 노이즈에서 capsule_id 취약.
#   구 규격(75x38.4)은 pitch 0도 필수였으나 라벨 확대로 제약 해소.
#
# 핵심 설계 교훈:
#   4행을 통째로 읽으면 'ST-OR1'의 S 때문에 5<->S 혼동이 열려 C05가 깨진다.
#   -> 캡슐ID 전용 화이트리스트에서 S/O를 제외해 오인식 경로를 물리적으로 차단.
#
# 입력: UDP 47137  {"capsule_id": "C05", "image": "/path/x.png"}
# 출력: /label_scan (String JSON) + /or_station_event {"event":"LABEL_READ"}
# 실행(ROS): source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=136 \
#            && python3 or_label_ocr_node.py
# 실행(단발): python3 or_label_ocr_node.py --file x.png [--shrink 700] [--debug]
# ============================================================
import sys, os, json, re, socket, time
from collections import Counter
from datetime import datetime, timedelta
import cv2, numpy as np, pytesseract

CFG = json.load(open("/home/rokey/rokey_cobot3/isaacpjt/config/label_ocr.json"))
LABEL_AR = 120.0 / 75.0                 # 1.600 — 라벨 실물 종횡비 (2026-08-27 규격 변경)
RECT_W = 1600
RECT_H = int(RECT_W / LABEL_AR)         # 819
FIELDS = ("item_code", "packaging_date", "expiration_date",
          "order_id", "delivery_add", "capsule_id")
CORE = ("order_id", "delivery_add", "capsule_id")     # 시연 판정 3필드
KMAP = {"packaging_date": "packaging", "expiration_date": "expiration",
        "delivery_add": "delivery"}                   # make_labels.py 스키마 매핑
DBG = "/home/rokey/rokey_cobot3/isaacpjt/output/ocr_debug"

FALLBACK_DIV = [0.22, 0.52, 0.68]       # 격자 검출 실패 시 행 경계 비율
WL_ALPHA = "ABCDEFGHIJKLMNOPQRSTUVWXYZ-"
WL_NUM   = "0123456789-:"
WL_ORDER = "O0123456789-"
WL_DELIV = "STOR0123456789-/"           # 배송지 전용 (C 제외)
WL_CAPS  = "C0123456789"                # 캡슐 전용 (S/O 제외 -> 5,0 오인식 차단)

# ---------- 1) 라벨 검출 + 원근 보정 ----------
def _order_pts(p):
    s, d = p.sum(1), np.diff(p, axis=1).ravel()
    return np.array([p[np.argmin(s)], p[np.argmin(d)],
                     p[np.argmax(s)], p[np.argmax(d)]], dtype="float32")

def rectify(img):
    """검은 테두리 사각형을 찾아 정면으로 편다. 실패 시 (None, None)"""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    H, W = g.shape
    best, best_area = None, 0
    for th in (cv2.adaptiveThreshold(g, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                     cv2.THRESH_BINARY, 51, 10),
               cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]):
        cnts, _ = cv2.findContours(th, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < W * H * 0.01 or area <= best_area:
                continue
            ap = cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True)
            if len(ap) != 4 or not cv2.isContourConvex(ap):
                continue
            pts = _order_pts(ap.reshape(4, 2).astype("float32"))
            wid = max(np.linalg.norm(pts[1]-pts[0]), np.linalg.norm(pts[2]-pts[3]))
            hei = max(np.linalg.norm(pts[3]-pts[0]), np.linalg.norm(pts[2]-pts[1]))
            if hei < 1 or abs(wid/hei - LABEL_AR) > 0.45:
                continue
            best, best_area = pts, area
    if best is None:
        return None, None
    dst = np.array([[0, 0], [RECT_W, 0], [RECT_W, RECT_H], [0, RECT_H]], dtype="float32")
    M = cv2.getPerspectiveTransform(best, dst)
    return cv2.warpPerspective(img, M, (RECT_W, RECT_H)), best.astype(int).tolist()

def crop_label(img):
    """보정 실패 시 폴백: 밝은 사각형 바운딩박스"""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    H, W = g.shape
    _, th = cv2.threshold(g, 170, 255, cv2.THRESH_BINARY)
    cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, err_best = None, 1e9
    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        if w < W*0.05 or h < H*0.03:
            continue
        e = abs(w / max(h, 1) - LABEL_AR)
        if e < 0.6 and e < err_best:
            best, err_best = (x, y, w, h), e
    if best is None:
        return img, None
    x, y, w, h = best
    m = int(0.03 * w)
    return img[max(0, y-m):y+h+m, max(0, x-m):x+w+m], [x, y, w, h]

# ---------- 2) 전처리 · OCR 기본기 ----------
def _binz(roi, sharpen=False):
    g = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if roi.ndim == 3 else roi
    sc = max(1.0, 1600.0 / max(g.shape[1], 1))
    g = cv2.resize(g, None, fx=sc, fy=sc, interpolation=cv2.INTER_CUBIC)
    if sharpen:
        g = cv2.filter2D(g, -1, np.array([[0,-1,0], [-1,5,-1], [0,-1,0]]))
    g = cv2.GaussianBlur(g, (3, 3), 0)
    _, g = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return g

def _read(g, psm):
    return pytesseract.image_to_string(g, config=f"--oem 3 --psm {psm}")

def _conf(g, psm):
    try:
        d = pytesseract.image_to_data(g, config=f"--oem 3 --psm {psm}",
                                      output_type=pytesseract.Output.DICT)
        v = [int(c) for c in d["conf"] if c not in ("-1", -1)]
        return round(sum(v)/len(v), 1) if v else 0.0
    except Exception:
        return 0.0

def _cell_read(cell, psm, wl):
    """셀 하나를 확대·선명화·이진화 후 화이트리스트로 읽는다"""
    if cell is None or cell.size == 0:
        return ""
    g = cv2.cvtColor(cell, cv2.COLOR_BGR2GRAY) if cell.ndim == 3 else cell
    sc = max(1.0, 900.0 / max(g.shape[1], 1))
    g = cv2.resize(g, None, fx=sc, fy=sc, interpolation=cv2.INTER_CUBIC)
    g = cv2.filter2D(g, -1, np.array([[0,-1,0], [-1,5,-1], [0,-1,0]]))
    g = cv2.GaussianBlur(g, (3, 3), 0)
    _, g = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    g = cv2.copyMakeBorder(g, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255)
    return pytesseract.image_to_string(
        g, config=f"--oem 3 --psm {psm} -c tessedit_char_whitelist={wl}")

def _dates(t):
    return [f"{d} {tm}" for d, tm in
            re.findall(r"(\d{4}-\d{2}-\d{2})\s*(\d{2}:\d{2})", t)]

# ---------- 3) 셀 단위 판독 ----------
def find_grid(rect):
    """세로 구분선 x, 가로 구분선 y 3개 검출 (격자선이 회색 170이라 임계 205)"""
    g = cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY)
    H, W = g.shape
    dark = (g < 205).astype(np.uint8)
    col = dark[int(H*0.12):int(H*0.88)].sum(0)
    lo, hi = int(W*0.25), int(W*0.62)
    xdiv = lo + int(np.argmax(col[lo:hi]))
    if col[xdiv] < (H*0.76)*0.40:
        xdiv = int(W * CFG.get("value_col_x0", 0.43))
    row = dark[:, int(W*0.06):int(W*0.94)].sum(1)
    thr = (W*0.88) * 0.50
    cand = [y for y in range(int(H*0.10), int(H*0.93)) if row[y] > thr]
    groups = []
    for y in cand:
        if groups and y - groups[-1][-1] <= 5:
            groups[-1].append(y)
        else:
            groups.append([y])
    div = [int(sum(gp)/len(gp)) for gp in groups]
    ok = (len(div) == 3)
    if not ok:
        div = [int(H*f) for f in FALLBACK_DIV]
    return xdiv, div, ok

def _trim(cell, thr=200, m=4):
    """셀에서 글자 영역만 남긴다 — 비율 크롭이 여백을 읽는 것을 방지"""
    if cell is None or cell.size == 0:
        return cell
    g = cv2.cvtColor(cell, cv2.COLOR_BGR2GRAY) if cell.ndim == 3 else cell
    ink = (g < thr)
    xs, ys = np.where(ink.any(0))[0], np.where(ink.any(1))[0]
    if len(xs) == 0 or len(ys) == 0:
        return cell
    return cell[max(0, ys[0]-m):ys[-1]+m, max(0, xs[0]-m):xs[-1]+m]

def _vote(cell, crops, psms, wl, pattern, fmt, pick_last=False):
    """여러 크롭 x psm으로 읽어 최빈값 채택 (문자 단위 우연을 상쇄)"""
    if cell is None or cell.size == 0:
        return None, []
    w = cell.shape[1]
    out = []
    for a, b in crops:
        sub = cell[:, int(w*a):int(w*b)]
        if sub.shape[1] < 12:
            continue
        for psm in psms:
            ms = list(re.finditer(pattern, _cell_read(sub, psm, wl), re.I))
            if not ms:
                continue
            try:
                out.append(fmt(ms[-1] if pick_last else ms[0]))
            except Exception:
                pass
    if not out:
        return None, out
    return Counter(out).most_common(1)[0][0], out

def read_cells(rect):
    """rectify된 라벨을 행별로 읽는다. (fields, used, raw, grid)"""
    xdiv, div, ok = find_grid(rect)
    H, W = rect.shape[:2]
    bounds = [int(H*0.05)] + div + [int(H*0.95)]
    pad = 6
    f, raw = {}, []

    def cell_of(i):
        y0, y1 = bounds[i] + pad, bounds[i+1] - pad
        return _trim(rect[max(0, y0):max(y0+1, y1), min(xdiv+pad, W-2):W-pad])

    t = _cell_read(cell_of(0), 7, WL_ALPHA); raw.append("[item] " + t.strip())
    m = re.search(r"(?:5|S)TP[-\s]?([A-Z]{3})", t, re.I)
    if m: f["item_code"] = "STP-" + m.group(1).upper()

    t = _cell_read(cell_of(1), 6, WL_NUM); raw.append("[dates] " + t.strip())
    ds = _dates(t)
    if len(ds) > 0: f["packaging_date"] = ds[0]
    if len(ds) > 1: f["expiration_date"] = ds[1]

    # 3행은 단일 판독 — 투표를 붙이면 오히려 정확도가 떨어졌다(v6 실험)
    t = _cell_read(cell_of(2), 7, WL_ORDER); raw.append("[order] " + t.strip())
    m = re.search(r"[O0Q]\s*-\s*(\d{1,3})", t, re.I)
    if m: f["order_id"] = "O-%d" % int(m.group(1))

    # 4행: 트림된 폭 기준 좌(배송지) / 우(캡슐ID) 분리 후 각각 투표
    c4 = cell_of(3)
    dv, dcand = _vote(c4, [(0.0, 0.60), (0.0, 0.70), (0.0, 1.0)], (7, 6), WL_DELIV,
                      r"(?:5|S)T[-\s]?[O0Q]R\s*(\d)", lambda m: "ST-OR" + m.group(1))
    if dv: f["delivery_add"] = dv
    cv_, ccand = _vote(c4, [(0.55, 1.0), (0.45, 1.0), (0.65, 1.0), (0.0, 1.0)],
                       (7, 8, 6), WL_CAPS, r"C\s*(\d{1,2})",
                       lambda m: "C%02d" % int(m.group(1)), pick_last=True)
    if cv_: f["capsule_id"] = cv_
    raw.append("[deliv] %s  [caps] %s" % (dcand, ccand))

    got = [k for k in FIELDS if f.get(k)]
    conf = _conf(_binz(rect[:, int(W * CFG.get("value_col_x0", 0.43)):]), 4)
    used = ["cells(%s):%s" % ("grid" if ok else "ratio", "+".join(got))] if got else []
    return f, used, "\n".join(raw), {"xdiv": xdiv, "y": div, "detected": ok,
                                     "conf": conf, "caps_votes": ccand,
                                     "deliv_votes": dcand}

# ---------- 4) 전체 이미지 파싱 (캐스케이드 폴백용) ----------
def parse(text):
    t = text.replace("—", "-").replace("–", "-")
    def find(p, fix=lambda s: s):
        m = re.search(p, t, re.I)
        return fix(m.group(1)) if m else None
    dates = _dates(t)
    return {
        "item_code":       find(r"(?:5|S)TP[-\s]?([A-Z]{3})", lambda s: f"STP-{s.upper()}"),
        "packaging_date":  dates[0] if len(dates) > 0 else None,
        "expiration_date": dates[1] if len(dates) > 1 else None,
        "order_id":        find(r"\b[O0Q]\s?-\s?(\d{1,3})\b", lambda s: f"O-{int(s)}"),
        "delivery_add":    find(r"(?:5|S)T[-\s]?[O0Q]R\s?(\d)", lambda s: f"ST-OR{s}"),
        "capsule_id":      find(r"\b[C(]\s?[O0Q]?\s?(\d{1,2})\b", lambda s: f"C{int(s):02d}"),
    }

# ---------- 5) 자기 검증 · 정답지 ----------
def rule_check(f):
    """유통기한 = 포장일 + 30일 (생성 규칙) — 인식 결과의 자기 검증"""
    try:
        p = datetime.strptime(f["packaging_date"], "%Y-%m-%d %H:%M")
        e = datetime.strptime(f["expiration_date"], "%Y-%m-%d %H:%M")
        return (e - p) == timedelta(days=30)
    except Exception:
        return None

def gt_of(rec):
    return {k: rec.get(KMAP.get(k, k)) for k in FIELDS}

def ground_truth(fname=None):
    want = fname or CFG.get("expected_texture")
    for key in ("demo_json", "labels_json"):
        try:
            for it in json.load(open(CFG[key]))["records"]:
                if it["file"] == want:
                    return gt_of(it)
        except Exception:
            pass
    return None

# ---------- 6) 인식 본체 ----------
def recognize(capsule_id, image_path, shrink=None, debug=False):
    img = cv2.imread(image_path)
    if img is None:
        return {"capsule_id": capsule_id, "event": "LABEL_READ", "ok": False,
                "error": f"cannot read {image_path}"}
    if shrink:
        s = shrink / img.shape[1]
        img = cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
    t0 = time.time()

    rect, quad = rectify(img)
    if rect is not None:
        base, mode = rect, "rectified"
    else:
        base, quad = crop_label(img)
        mode = "bbox_fallback"

    f = {k: None for k in FIELDS}
    used, conf, raws, grid = [], 0.0, [], None

    if mode == "rectified":                       # 1차: 셀 판독
        cf, cused, craw, grid = read_cells(base)
        for k in FIELDS:
            if cf.get(k):
                f[k] = cf[k]
        used += cused
        raws.append(craw)
        conf = (grid or {}).get("conf", 0.0)

    if not all(f[k] for k in FIELDS):              # 2차: 빈 필드만 캐스케이드로 보완
        x0 = CFG.get("value_col_x0", 0.43)
        val = base[:, int(base.shape[1] * x0):]
        for name, g, psm in (("val/psm4", _binz(val), 4),
                             ("val/psm6", _binz(val), 6),
                             ("val/sharp/psm4", _binz(val, sharpen=True), 4),
                             ("full/psm4", _binz(base), 4),
                             ("full/psm11", _binz(base), 11)):
            txt = _read(g, psm)
            raws.append(f"[{name}]\n{txt.strip()}")
            got = parse(txt)
            filled = [k for k in FIELDS if not f[k] and got.get(k)]
            if filled:
                for k in filled:
                    f[k] = got[k]
                used.append(f"{name}:{'+'.join(filled)}")
                if not conf:
                    conf = _conf(g, psm)
            if all(f[k] for k in FIELDS):
                break

    res = {
        "capsule_id": capsule_id, "event": "LABEL_READ",
        "ok": all(f.get(k) for k in CORE),
        "fields": f,
        "label_matches_capsule": (f.get("capsule_id") == capsule_id),
        "detect_mode": mode, "quad": quad, "grid": grid, "variants_used": used,
        "confidence": conf, "date_rule_ok": rule_check(f),
        "image": image_path, "ocr_ms": int((time.time()-t0)*1000),
    }
    gt = ground_truth(os.path.basename(image_path))
    if gt is None:
        res["gt_match"] = "no-ground-truth"
    else:
        miss = [k for k in FIELDS if f.get(k) != gt[k]]
        res["gt_match"] = f"{len(FIELDS)-len(miss)}/6"
        res["gt_miss"] = miss
    if debug:
        os.makedirs(DBG, exist_ok=True)
        cv2.imwrite(f"{DBG}/1_base_{mode}.png", base)
        open(f"{DBG}/2_raw.txt", "w").write("\n\n".join(raws))
        res["debug_dir"] = DBG
    return res

# ---------- 7) 실행부 ----------
def main():
    if "--file" in sys.argv:
        path = sys.argv[sys.argv.index("--file") + 1]
        shrink = int(sys.argv[sys.argv.index("--shrink") + 1]) if "--shrink" in sys.argv else None
        print(json.dumps(recognize(CFG.get("demo_capsule_id", "C05"), path, shrink,
                                   "--debug" in sys.argv), ensure_ascii=False, indent=1))
        return

    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node("or_label_ocr")
    pub_scan = node.create_publisher(String, "/label_scan", 10)
    pub_evt = node.create_publisher(String, "/or_station_event", 10)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", CFG["udp_port"]))
    print(f"[ocr] UDP {CFG['udp_port']} -> /label_scan, /or_station_event(LABEL_READ)")
    while True:
        data, _ = sock.recvfrom(65536)
        req = json.loads(data.decode("utf-8"))
        res = recognize(req.get("capsule_id", "C??"), req["image"])
        m = String(); m.data = json.dumps(res, ensure_ascii=False); pub_scan.publish(m)
        e = String(); e.data = json.dumps({
            "capsule_id": res["capsule_id"], "event": "LABEL_READ", "ok": res["ok"],
            "order_id": res["fields"].get("order_id"),
            "delivery_add": res["fields"].get("delivery_add"),
            "label_capsule": res["fields"].get("capsule_id"),
            "match": res["label_matches_capsule"]})
        pub_evt.publish(e)
        print(f"[ocr] {res['capsule_id']} ok={res['ok']} conf={res['confidence']} "
              f"{res['detect_mode']} {res['fields']} {res['ocr_ms']}ms")

if __name__ == "__main__":
    main()
