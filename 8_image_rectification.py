#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
extrinsics_stereo_calib.py
- 두 카메라의 외부 파라미터(Rotaion, Translation) 및 E, F, Rectification 행렬 산출
- 내부 파라미터는 /calibration/int/cam1.yaml, cam2.yaml
- 저장 경로:
    /calibration/ext/stereo.yaml
- [NEW] 캘리브레이션 직후 원본/보정본 비교 시각화 기능 추가
"""
import argparse
import glob
import os
import json
from typing import List, Tuple

import cv2
import numpy as np

cam1_path ="./calibration/cam1/*.jpg"
cam2_path ="./calibration/cam2/*.jpg"
intrinsic_param ="./calibration/int"
extrinsic_param ="./calibration/ext"
cols = 5
rows = 6
square = 80.0
    
    
def load_intrinsics(path_yaml: str):
    fs = cv2.FileStorage(path_yaml, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(f"열 수 없음: {path_yaml}")
    K = fs.getNode("K").mat()
    D = fs.getNode("dist").mat()
    fs.release()
    if K is None or D is None:
        raise RuntimeError(f"파일에 K/dist가 없습니다: {path_yaml}")
    return K, D

def build_object_points(cols: int, rows: int, square: float) -> np.ndarray:
    objp = np.zeros((cols*rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square)
    return objp

def find_corners(img: np.ndarray, cols: int, rows: int):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    ret, corners = cv2.findChessboardCorners(gray, (cols, rows))
    if not ret:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners

def main():

    os.makedirs(extrinsic_param, exist_ok=True)

    K1, D1 = load_intrinsics(os.path.join(intrinsic_param, "cam1.yaml"))
    K2, D2 = load_intrinsics(os.path.join(intrinsic_param, "cam2.yaml"))
    print("[INFO] Intrinsics loaded.")

    p1 = sorted(glob.glob(cam1_path))
    p2 = sorted(glob.glob(cam2_path))
    if not p1 or not p2:
        raise FileNotFoundError("입력 이미지가 비었습니다.")
    if len(p1) != len(p2):
        print(f"[WARN] 이미지 개수가 다릅니다. min 쌍으로만 진행합니다. ({len(p1)} vs {len(p2)})")
    n = min(len(p1), len(p2))

    objp = build_object_points(cols, rows, square)
    objpoints: List[np.ndarray] = []
    imgpoints1: List[np.ndarray] = []
    imgpoints2: List[np.ndarray] = []
    imsize = None

    # [NEW] 비교 시각화를 위해 첫 번째 유효한 이미지 쌍을 저장할 변수
    first_valid_im1 = None
    first_valid_im2 = None

    for i in range(n):
        im1 = cv2.imread(p1[i], cv2.IMREAD_COLOR)
        im2 = cv2.imread(p2[i], cv2.IMREAD_COLOR)
        if im1 is None or im2 is None:
            print(f"[WARN] 로드 실패: {p1[i]} or {p2[i]}")
            continue
        if imsize is None:
            imsize = (im1.shape[1], im1.shape[0]) # (width, height)
        
        # [NEW] 캘리브레이션과 시각화에 사용할 이미지 해상도가 동일한지 확인
        if (im1.shape[1], im1.shape[0]) != imsize:
            print(f"[WARN] 해상도가 다른 이미지 발견, 건너뜁니다: {p1[i]}")
            continue

        ret1, c1 = find_corners(im1, cols, rows)
        ret2, c2 = find_corners(im2, cols, rows)
        
        if ret1 and ret2:
            objpoints.append(objp.copy())
            imgpoints1.append(c1)
            imgpoints2.append(c2)
            
            # [NEW] 첫 번째 유효한 이미지 쌍 저장
            if first_valid_im1 is None:
                first_valid_im1 = im1.copy()
                first_valid_im2 = im2.copy()
        else:
            print(f"[MISS] 코너 미검출: {p1[i]} / {p2[i]}")

    if not objpoints:
        raise RuntimeError("스테레오 코너가 검출된 쌍이 없습니다.")

    print(f"[INFO] 유효 쌍: {len(objpoints)} / {n}")

    # 스테레오 캘리브레이션 (내부 파라미터 고정 가정)
    flags = (cv2.CALIB_FIX_INTRINSIC)
    rms, K1o, D1o, K2o, D2o, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints1, imgpoints2,
        K1, D1, K2, D2, imsize,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
    )
    print("[RESULT] Stereo RMS:", rms)
    print("[RESULT] R=\n", R)
    print("[RESULT] T=\n", T.T)

    # Rectification (유용하므로 함께 저장)
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K1, D1, K2, D2, imsize, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
    )


    if first_valid_im1 is not None:
        print("\n[INFO] 캘리브레이션 결과 시각화...")
        h, w = imsize[1], imsize[0]

        # 1. 보정 전 원본 이미지 (나란히 붙이기)
        original_pair = np.hstack((first_valid_im1, first_valid_im2))

        # 2. 보정 후 정렬된 이미지 (렉티피케이션)
        map1_x, map1_y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, imsize, cv2.CV_32FC1)
        map2_x, map2_y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, imsize, cv2.CV_32FC1)
        
        img1_rect = cv2.remap(first_valid_im1, map1_x, map1_y, cv2.INTER_LINEAR)
        img2_rect = cv2.remap(first_valid_im2, map2_x, map2_y, cv2.INTER_LINEAR)

        rectified_pair = np.hstack((img1_rect, img2_rect))

        # 3. 보정된 이미지에 정렬 테스트용 수평선 그리기
        num_lines = 25
        for i in range(1, num_lines):
            y = int(h * i / num_lines)
            cv2.line(rectified_pair, (0, y), (w * 2, y), (0, 255, 0), 1)

        # 4. 원본과 보정본을 위아래로 합치기
        #    텍스트를 추가하기 위해 경계선 영역(검은색)을 만듭니다.
        font = cv2.FONT_HERSHEY_SIMPLEX
        margin = np.zeros((50, w * 2, 3), dtype=np.uint8)
        
        cv2.putText(margin, "Original (Before Calibration)", (10, 35), font, 1.2, (255, 255, 255), 2)
        top_image = np.vstack((margin, original_pair))

        margin = np.zeros((50, w * 2, 3), dtype=np.uint8)
        cv2.putText(margin, "Rectified (After Calibration) - Check Green Lines", (10, 35), font, 1.2, (0, 255, 0), 2)
        bottom_image = np.vstack((margin, rectified_pair))
        
        comparison_image = np.vstack((top_image, bottom_image))
        
        save_img_path = "./calibration/8_result/comparison.png"
        cv2.imwrite(save_img_path, comparison_image)
        print(f"[SAVE] 비교 이미지 저장 완료: {save_img_path}")

    # 저장 (YAML + JSON)
    yaml_path = os.path.join(extrinsic_param, "stereo.yaml")
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_WRITE)
    fs.write("R", R)
    fs.write("T", T)
    fs.write("E", E)
    fs.write("F", F)
    fs.write("R1", R1); fs.write("R2", R2)
    fs.write("P1", P1); fs.write("P2", P2)
    fs.write("Q", Q)
    fs.write("rms", float(rms))
    fs.release()
    print(f"[SAVE] {yaml_path}")

    with open(os.path.join(extrinsic_param, "stereo.json"), "w", encoding="utf-8") as f:
        json.dump({
            "R": R.tolist(),
            "T": T.tolist(),
            "E": E.tolist(),
            "F": F.tolist(),
            "R1": R1.tolist(), "R2": R2.tolist(),
            "P1": P1.tolist(), "P2": P2.tolist(),
            "Q": Q.tolist(),
            "rms": float(rms),
        }, f, indent=2, ensure_ascii=False)
    print(f"[SAVE] {os.path.join(extrinsic_param, 'stereo.json')}")

if __name__ == "__main__":

    main()
