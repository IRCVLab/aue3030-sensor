#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
intrinsics_calib.py
- 두 대 카메라의 내부 파라미터(K, dist)만 추정하여 저장
- 입력 이미지:
    /calibration/cam1/*.jpg
    /calibration/cam2/*.jpg
- 저장 경로:
    /calibration/int/cam1.yaml (및 .json)
    /calibration/int/cam2.yaml (및 .json)

기본은 체스보드(내부 코너 cols x rows), 한 칸 실측 길이 square(단위 일관성 유지).
"""
import glob
import os
from typing import List, Tuple

import cv2
import numpy as np
import json

# 이미지 경로
cam1_glob = "./calibration/cam1/*.jpg"
cam2_glob = "./calibration/cam2/*.jpg"

# 코너 교차점 개수
cols = 5
rows = 6

# 체커보드 한 칸의 실제 길이(mm)
square = 80.0
outdir = "./calibration/int"

def build_object_points(cols: int, rows: int, square: float) -> np.ndarray:
    objp = np.zeros((cols*rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square)
    return objp

def find_corners(img: np.ndarray, cols: int, rows: int):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    # Q. 체커보드의 코너를 검출하는 이유가 무엇일까?
    ret, corners = cv2.findChessboardCorners(gray, (cols, rows))
    if not ret:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners

def save_intrinsics(path_yaml: str, K: np.ndarray, dist: np.ndarray, rms: float):
    # 혹시 몰라 YAML, JSON 둘 다 저장하기
    # YAML
    fs = cv2.FileStorage(path_yaml, cv2.FILE_STORAGE_WRITE)
    fs.write("K", K)
    fs.write("dist", dist)
    fs.write("reproj_error", float(rms))
    fs.release()
    # JSON
    base_json = os.path.splitext(path_yaml)[0] + ".json"
    with open(base_json, "w", encoding="utf-8") as f:
        json.dump({"K": K.tolist(), "dist": dist.tolist(), "reproj_error": float(rms)}, f, indent=2, ensure_ascii=False)
    print(f"[SAVE] {path_yaml}")
    print(f"[SAVE] {base_json}")

def calibrate_single_cam(image_glob: str, cols: int, rows: int, square: float):
    paths = sorted(glob.glob(image_glob))
    if not paths:
        raise FileNotFoundError(f"이미지 없음: {image_glob}")
    objp = build_object_points(cols, rows, square)

    objpoints: List[np.ndarray] = []
    imgpoints: List[np.ndarray] = []
    imsize: Tuple[int, int] = None

    for p in paths:
        img = cv2.imread(p, cv2.IMREAD_COLOR)
        if img is None:
            print(f"[WARN] 로드 실패: {p}")
            continue

        ret, corners = find_corners(img, cols, rows)
        if ret:
            objpoints.append(objp.copy())
            imgpoints.append(corners)
        else:
            print(f"[MISS] 코너 검출 실패: {p}")

    if not objpoints:
        raise RuntimeError("코너 검출된 이미지가 없습니다. 촬영/패턴을 확인하세요.")

    print(f"[INFO] 유효 이미지: {len(objpoints)} / {len(paths)}")
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, imsize, None, None,
        flags=0,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
    )
    return rms, K, dist, imsize, (objpoints, imgpoints)

def main():
    os.makedirs(outdir, exist_ok=True)

    print("[CAM1] 내부 파라미터 추정 중...")
    rms1, K1, D1, size1, data1 = calibrate_single_cam(cam1_glob, cols, rows, square)
    save_intrinsics(os.path.join(outdir, "cam1.yaml"), K1, D1, rms1)

    print("\n[CAM2] 내부 파라미터 추정 중...")
    rms2, K2, D2, size2, data2 = calibrate_single_cam(cam2_glob, cols, rows, square)
    save_intrinsics(os.path.join(outdir, "cam2.yaml"), K2, D2, rms2)

    # 보조로 데이터 페어 수/이미지 크기 기록(스테레오 단계에서 유용)
    meta = {
        "cam1": {"image_size": size1, "valid_images": len(data1[0])},
        "cam2": {"image_size": size2, "valid_images": len(data2[0])},
        "pattern": {"cols": cols, "rows": rows, "square": square}
    }
    with open(os.path.join(outdir, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)
    print(f"[SAVE] {os.path.join(outdir, 'meta.json')}")

if __name__ == "__main__":
    main()
