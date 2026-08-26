#!/usr/bin/env python3
# [10-1] 시연 라벨 1장의 왜곡 내성 측정: 기울기·거리·밝기·블러·노이즈
import sys, os, json, itertools, tempfile
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cv2, numpy as np
import or_label_ocr_node as N

SRC = N.CFG["demo_texture"]
GT = N.ground_truth("demo_0001.png")
TMP = tempfile.mkdtemp(prefix="ocr_rb_")
img0 = cv2.imread(SRC)

def warp(img, yaw, pitch, pad=0.35):
    """라벨을 씬 배경 위에 놓고 yaw/pitch만큼 기울인 카메라 시점으로 변환"""
    h, w = img.shape[:2]
    px, py = int(w * pad), int(h * pad)
    canvas = np.full((h + 2*py, w + 2*px, 3), 90, np.uint8)   # 어두운 배경 = 씬 모사
    canvas[py:py+h, px:px+w] = img
    src = np.float32([[px, py], [px+w, py], [px+w, py+h], [px, py+h]])
    ky, kp = np.tan(np.radians(yaw)) * h * 0.5, np.tan(np.radians(pitch)) * w * 0.5
    dst = np.float32([[px+max(0,ky), py+max(0,kp)], [px+w-max(0,-ky), py+max(0,-kp)],
                      [px+w-max(0,-ky), py+h-max(0,kp)], [px+max(0,ky), py+h-max(0,-kp)]])
    M = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(canvas, M, (canvas.shape[1], canvas.shape[0]),
                               borderValue=(90, 90, 90))

def degrade(img, width, gamma, blur, noise):
    s = width / img.shape[1]
    im = cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
    if gamma != 1.0:
        im = np.clip(((im/255.0)**gamma)*255, 0, 255).astype(np.uint8)
    if blur:
        im = cv2.GaussianBlur(im, (blur*2+1, blur*2+1), 0)
    if noise:
        im = np.clip(im.astype(np.int16) +
                     np.random.normal(0, noise, im.shape).astype(np.int16), 0, 255).astype(np.uint8)
    return im

def trial(tag, im):
    p = os.path.join(TMP, tag.replace("/", "_") + ".png")
    cv2.imwrite(p, im)
    r = N.recognize("C05", p)
    core = all(r["fields"].get(k) == GT[k] for k in N.CORE)
    six = all(r["fields"].get(k) == GT[k] for k in N.FIELDS)
    bad = [k for k in N.CORE if r["fields"].get(k) != GT[k]]
    return core, six, r, bad

print(f"정답: {GT}\n임시: {TMP}\n")
rows = []
print("--- A. 기울기 (라벨 폭 700px 상당) ---")
for yaw, pitch in [(0,0), (10,0), (20,0), (30,0), (0,10), (0,20), (15,15), (25,20), (35,25)]:
    im = degrade(warp(img0, yaw, pitch), 700*1.7, 1.0, 0, 0)
    core, six, r, bad = trial(f"tilt_{yaw}_{pitch}", im)
    print(f"  yaw{yaw:>3} pitch{pitch:>3}: 핵심3={'OK ' if core else 'FAIL'} 6필드={'OK' if six else '--'}"
          f" mode={r['detect_mode']:<14} conf={r['confidence']:<5} {'실패:'+','.join(bad) if bad else ''}")

print("\n--- B. 거리(라벨 폭 px) @ yaw15/pitch10 ---")
for w in [1200, 900, 700, 550, 476, 400, 340, 280]:
    im = degrade(warp(img0, 15, 10), w*1.7, 1.0, 0, 0)
    core, six, r, bad = trial(f"dist_{w}", im)
    print(f"  폭 {w:>5}px: 핵심3={'OK ' if core else 'FAIL'} 6필드={'OK' if six else '--'}"
          f" mode={r['detect_mode']:<14} conf={r['confidence']:<5} {'실패:'+','.join(bad) if bad else ''}")

print("\n--- C. 조명/블러/노이즈 @ 폭700 yaw15 ---")
for tag, g, b, nz in [("정상",1.0,0,0), ("어둡게",1.8,0,0), ("밝게(반사)",0.5,0,0),
                      ("블러1",1.0,1,0), ("블러2",1.0,2,0), ("노이즈8",1.0,0,8),
                      ("노이즈15",1.0,0,15), ("복합",1.4,1,8)]:
    im = degrade(warp(img0, 15, 10), 700*1.7, g, b, nz)
    core, six, r, bad = trial(f"deg_{tag}", im)
    print(f"  {tag:<11}: 핵심3={'OK ' if core else 'FAIL'} 6필드={'OK' if six else '--'}"
          f" mode={r['detect_mode']:<14} conf={r['confidence']:<5} {'실패:'+','.join(bad) if bad else ''}")
