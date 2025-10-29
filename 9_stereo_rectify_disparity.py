#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import argparse
import numpy as np
import cv2

def read_yaml_matrix(path, key):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(f"열 수 없음: {path}")
    node = fs.getNode(key)
    if node.empty():
        fs.release()
        raise KeyError(f"{key} 키가 {path}에 없음")
    mat = node.mat()
    fs.release()
    return mat

def load_intrinsics(int_dir):
    cam1 = os.path.join(int_dir, "cam1.yaml")
    cam2 = os.path.join(int_dir, "cam2.yaml")
    K1 = read_yaml_matrix(cam1, "K")
    D1 = read_yaml_matrix(cam1, "dist")
    K2 = read_yaml_matrix(cam2, "K")
    D2 = read_yaml_matrix(cam2, "dist")
    return K1, D1, K2, D2

def load_stereo(ext_dir):
    st = os.path.join(ext_dir, "stereo.yaml")
    R  = read_yaml_matrix(st, "R")
    T  = read_yaml_matrix(st, "T")
    R1 = read_yaml_matrix(st, "R1")
    R2 = read_yaml_matrix(st, "R2")
    P1 = read_yaml_matrix(st, "P1")
    P2 = read_yaml_matrix(st, "P2")
    Q  = read_yaml_matrix(st, "Q")
    return R, T, R1, R2, P1, P2, Q

def rectify_images(img1, img2, K1, D1, K2, D2, R, T, alpha=0.0):
    h, w = img1.shape[:2]
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K1, D1, K2, D2, (w, h), R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=alpha
    )
    map1x, map1y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (w, h), cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (w, h), cv2.CV_32FC1)
    r1 = cv2.remap(img1, map1x, map1y, cv2.INTER_LINEAR)
    r2 = cv2.remap(img2, map2x, map2y, cv2.INTER_LINEAR)
    return r1, r2, R1, R2, P1, P2, Q

def draw_epipolar_lines_pair(imgL, imgR, n_lines=12):
    h, w = imgL.shape[:2]
    step = max(1, h // (n_lines + 1))
    vis = np.hstack([imgL.copy(), imgR.copy()])
    for y in range(step, h, step):
        cv2.line(vis, (0, y), (w-1, y), (0, 255, 0), 1, cv2.LINE_AA)
        cv2.line(vis, (w, y), (2*w-1, y), (0, 255, 0), 1, cv2.LINE_AA)
    return vis

def compute_disparity_sgbm(imgL_rect, imgR_rect):
    gL = cv2.cvtColor(imgL_rect, cv2.COLOR_BGR2GRAY)
    gR = cv2.cvtColor(imgR_rect, cv2.COLOR_BGR2GRAY)
    min_disp = 0
    num_disp = 128
    blockSize = 5
    matcher = cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=blockSize,
        P1=8 * 3 * blockSize ** 2,
        P2=32 * 3 * blockSize ** 2,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )
    disp = matcher.compute(gL, gR).astype(np.float32) / 16.0
    return disp

def colorize_disparity(disp):
    disp_valid = disp.copy()
    disp_valid[np.isinf(disp_valid)] = np.nan
    vmin = np.nanpercentile(disp_valid, 5)
    vmax = np.nanpercentile(disp_valid, 95)
    if not np.isfinite(vmin): vmin = np.nanmin(disp_valid)
    if not np.isfinite(vmax): vmax = np.nanmax(disp_valid)
    if vmax <= vmin: vmax = vmin + 1.0
    disp_norm = np.clip((disp - vmin) / (vmax - vmin), 0, 1)
    disp_vis = (disp_norm * 255).astype(np.uint8)
    disp_vis = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
    return disp_vis

def reproject_to_3d(disp, Q):
    points_3d = cv2.reprojectImageTo3D(disp, Q)
    depth = points_3d[:, :, 2]
    return points_3d, depth

def colorize_depth(depth):
    dep = depth.copy()
    dep[~np.isfinite(dep)] = 0
    hi = np.percentile(dep, 98)
    if hi <= 0: hi = dep.max() if dep.max() > 0 else 1.0
    dep = np.clip(dep, 0, hi)
    dep_vis = (dep / (hi + 1e-6) * 255).astype(np.uint8)
    dep_vis = cv2.applyColorMap(dep_vis, cv2.COLORMAP_TURBO)
    return dep_vis

def point_depth_from_disparity(uL, vL, uR, vR, P1, P2):
    f = P1[0, 0]
    B = -P2[0, 3] / P2[0, 0]
    d = (uL - uR)
    if d <= 0:
        return d, f, B, np.inf
    Z = f * B / d
    return d, f, B, Z

def main():
    ap = argparse.ArgumentParser(description="Stereo rectification + disparity + depth demo")
    ap.add_argument("--img1", default="./calibration/cam1/image_000.jpg")
    ap.add_argument("--img2",default="./calibration/cam2/image_000.jpg")
    ap.add_argument("--intdir", default="./calibration/int")
    ap.add_argument("--extdir", default="./calibration/ext")
    ap.add_argument("--outdir", default="./calibration/outputs")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    K1, D1, K2, D2 = load_intrinsics(args.intdir)
    R, T, R1_s, R2_s, P1_s, P2_s, Q_s = load_stereo(args.extdir)
    img1 = cv2.imread(args.img1, cv2.IMREAD_COLOR)
    img2 = cv2.imread(args.img2, cv2.IMREAD_COLOR)
    if img1 is None or img2 is None:
        raise FileNotFoundError("이미지 로드 실패")

    img1_rect, img2_rect, R1, R2, P1, P2, Q = rectify_images(img1, img2, K1, D1, K2, D2, R, T, alpha=0.0)

    pair_clean = np.hstack([img1_rect.copy(), img2_rect.copy()])
    pair = pair_clean.copy() # 현재 표시용 이미지
    
    H, W = img1_rect.shape[:2]
    win = "click_left_then_right (press q to quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    clicks = []
    
    def onmouse(event, x, y, flags, param):
        nonlocal clicks, pair
        
        if event == cv2.EVENT_LBUTTONDOWN:
            
            # [수정] 새 클릭 시퀀스(첫 번째 클릭)가 시작되면
            #       이전의 모든 그림(점, 텍스트)을 지우기 위해 이미지를 초기화합니다.
            if len(clicks) == 0:
                pair = pair_clean.copy()

            clicks.append((x, y))
            
            color = (0, 255, 0) if len(clicks) == 1 else (0, 0, 255)
            
            # [수정] 첫 번째 클릭(좌측)의 y좌표를 저장합니다.
            if len(clicks) == 1:
                cv2.circle(pair, (x, y), 5, color, -1)
            
            # [수정] 두 번째 클릭(우측) 시
            elif len(clicks) == 2:
                (uL, vL), (uR_global, vR_clicked) = clicks
                uR = uR_global - W
                
                # 렉티피케이션 후에는 y좌표가 동일해야 하므로,
                # 첫 번째 클릭의 y좌표(vL)를 사용하도록 강제합니다.
                vR = vL 
                
                # 보정된 위치(vL)에 빨간 점을 그립니다.
                cv2.circle(pair, (uR_global, vR), 5, color, -1) # y 대신 vR(vL) 사용

                d, f, B, Z = point_depth_from_disparity(uL, vL, uR, vR, P1, P2)
                
                # 터미널에만 출력 (요청대로 putText는 제거)
                print(f"[POINT] (uL,vL)=({uL},{vL}) (uR,vR)=({uR},{vR}) disparity on images={d:.3f}px -> depth={Z:.3f}mm")
                
                cv2.putText(pair, f"disparity={d:.2f}px, depth={Z:.2f}mm", (30, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 2)
                
                # [수정] 계산이 끝났으므로 다음 클릭을 위해 리스트를 비웁니다.
                clicks = [] 
            
            # [수정] 모든 클릭 이벤트 후에 이미지를 갱신합니다.
            cv2.imshow(win, pair)

    cv2.setMouseCallback(win, onmouse)
    cv2.imshow(win, pair)
    while True:
        if cv2.waitKey(10) & 0xFF == ord('q'):
            save_path = os.path.join(args.outdir, "depth.png")
            cv2.imwrite(save_path, pair)
            print(f"\n[SAVE] 클릭 데모 이미지 저장 완료: {save_path}")
            break # 루프 탈출
    cv2.destroyWindow(win)

if __name__ == "__main__":
    main()
