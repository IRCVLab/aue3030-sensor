#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_corners.py
- intrinsics_calib.py가 사용한 이미지들에서 코너가 잘 검출되었는지 시각화합니다.
- '지그재그' 선 (cv2.drawChessboardCorners)을 이미지에 그려서 저장합니다.
"""
import argparse
import glob
import os

import cv2
import numpy as np

def find_corners(img: np.ndarray, cols: int, rows: int):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    ret, corners = cv2.findChessboardCorners(gray, (cols, rows))
    if not ret:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners

def process_images(image_glob: str, out_path: str, cols: int, rows: int):
    """
    지정된 경로의 모든 이미지를 처리하여 코너를 그리고 저장합니다.
    """
    os.makedirs(out_path, exist_ok=True)
    print(f"\n[INFO] 처리 중: {image_glob}")
    print(f"[INFO] 결과 저장 경로: {out_path}")
    
    paths = sorted(glob.glob(image_glob))
    if not paths:
        print(f"❌ 이미지를 찾을 수 없습니다: {image_glob}")
        return

    success_count = 0
    for p in paths:
        # 원본 이미지 읽기
        img_original = cv2.imread(p, cv2.IMREAD_COLOR)
        if img_original is None:
            print(f"[WARN] 로드 실패: {p}")
            continue

        # 코너 찾기
        ret, corners = find_corners(img_original, cols, rows)
        
        # 찾은 코너를 원본 이미지 위에 그리기 (지그재그 선)
        # ret이 False(실패)면 아무것도 그리지 않습니다.
        img_with_corners = img_original.copy()
        cv2.drawChessboardCorners(img_with_corners, (cols, rows), corners, ret)

        # 결과 이미지 저장
        filename = os.path.basename(p)
        save_path = os.path.join(out_path, filename)
        cv2.imwrite(save_path, img_with_corners)

        if ret:
            success_count += 1
        else:
            # intrinsics_calib.py와 동일한 [MISS] 메시지 출력
            print(f"[MISS] 코너 검출 실패: {filename}")
    
    print(f"[INFO] 완료. (성공: {success_count} / {len(paths)})")

def main():
    ap = argparse.ArgumentParser(description="Check chessboard corner detection (visualizer)")
    ap.add_argument("--cam1-glob", default="./calibration/cam1/*.jpg")
    ap.add_argument("--cam2-glob", default="./calibration/cam2/*.jpg")
    ap.add_argument("--cols", type=int, default=5, help="체스보드 내부 코너(가로)")
    ap.add_argument("--rows", type=int, default=6, help="체스보드 내부 코너(세로)")
    ap.add_argument("--outdir", default="./calibration/corners_check", help="코너 검출 확인용 이미지 저장 폴더")
    args = ap.parse_args()

    # cam1 처리
    process_images(args.cam1_glob, os.path.join(args.outdir, "cam1"), args.cols, args.rows)
    
    # cam2 처리
    process_images(args.cam2_glob, os.path.join(args.outdir, "cam2"), args.cols, args.rows)
    
    print(f"\n✅ 모든 작업 완료! \n{args.outdir} 폴더에서 'cam1'과 'cam2' 폴더를 확인하세요.")

if __name__ == "__main__":
    main()