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
import argparse
import glob
import os
from typing import List, Tuple

import cv2
import numpy as np
import json

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

def save_intrinsics(path_yaml: str, K: np.ndarray, dist: np.ndarray, rms: float):
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
        if imsize is None:
            imsize = (img.shape[1], img.shape[0])

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
    ap = argparse.ArgumentParser(description="Dual-camera intrinsics calibration (pinhole)")
    ap.add_argument("--cam1-glob", default="./calibration/cam1/*.jpg")
    ap.add_argument("--cam2-glob", default="./calibration/cam2/*.jpg")
    ap.add_argument("--cols", type=int, default=5, help="체스보드 내부 코너(가로)")
    ap.add_argument("--rows", type=int, default=6, help="체스보드 내부 코너(세로)")
    ap.add_argument("--square", type=float, default=80.0, help="한 칸 물리 길이(mm 등)")
    ap.add_argument("--outdir", default="./calibration/int", help="내부 파라미터 저장 폴더")
    return_args = ap.parse_args()

    os.makedirs(return_args.outdir, exist_ok=True)

    print("[CAM1] 내부 파라미터 추정 중...")
    rms1, K1, D1, size1, data1 = calibrate_single_cam(return_args.cam1_glob, return_args.cols, return_args.rows, return_args.square)
    save_intrinsics(os.path.join(return_args.outdir, "cam1.yaml"), K1, D1, rms1)

    print("\n[CAM2] 내부 파라미터 추정 중...")
    rms2, K2, D2, size2, data2 = calibrate_single_cam(return_args.cam2_glob, return_args.cols, return_args.rows, return_args.square)
    save_intrinsics(os.path.join(return_args.outdir, "cam2.yaml"), K2, D2, rms2)

    # 보조로 데이터 페어 수/이미지 크기 기록(스테레오 단계에서 유용)
    meta = {
        "cam1": {"image_size": size1, "valid_images": len(data1[0])},
        "cam2": {"image_size": size2, "valid_images": len(data2[0])},
        "pattern": {"cols": return_args.cols, "rows": return_args.rows, "square": return_args.square}
    }
    with open(os.path.join(return_args.outdir, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)
    print(f"[SAVE] {os.path.join(return_args.outdir, 'meta.json')}")

if __name__ == "__main__":
    main()
