"""
순차 매칭 (마스터-슬레이브)

원리:
1. 한 카메라를 기준(Master)으로 설정
2. Master 카메라에서 프레임을 하나 가져옴
3. 성공적으로 가져온 직후, 다른 카메라(Slave)에서 다음 프레임을 즉시 가져옴
4. 두 프레임의 타임스탬프 차이와 관계없이 동기화된 쌍으로 간주
5. 모든 Master 프레임은 프레임 손실 없이 Slave 프레임과 1:1로 매칭됨

장점:
- 프레임 손실이 없음 (Pass 없음)
- 구현이 간단하고 처리 속도가 빠름

단점:
- 동기화 정확도가 하드웨어 및 시스템 상태에 따라 유동적임
- 타임스탬프 차이가 임계값 기반 방식보다 클 수 있음
"""
import os

import gc
import time

import settings
import PySpin

from collections import deque
from typing import Optional, List, Dict

# ============================================================
# 사용자 설정
# ============================================================
CAMERA_SERIALS = settings.CAMERA_SERIALS
MASTER_CAMERA_IDX = 0  # Master 카메라 인덱스 (0 또는 1)
NUM_SYNCED_IMAGES = 1  # 저장할 동기화된 이미지 수
BUFFER_SIZE = 30  # 각 카메라의 이미지 버퍼 크기 (이 코드에서는 직접 사용되지 않음)
SAVE_IMAGES = True
VERBOSE = False  # 출력 최소화: 최종 통계만 출력
# ============================


def dbg(msg: str) -> None:
    """디버그 메시지 출력 (VERBOSE가 True일 때만)"""
    if VERBOSE:
        print(msg)


def set_enum(nm: PySpin.INodeMap, node_name: str, entry_name: str) -> bool:
    """열거형 노드 설정"""
    enum_node = PySpin.CEnumerationPtr(nm.GetNode(node_name))
    if not (PySpin.IsAvailable(enum_node) and PySpin.IsWritable(enum_node)):
        return False
    entry = enum_node.GetEntryByName(entry_name)
    if not (PySpin.IsAvailable(entry) and PySpin.IsReadable(entry)):
        return False
    enum_node.SetIntValue(entry.GetValue())
    return True


def configure_continuous_acquisition(cam: PySpin.CameraPtr, cam_serial: str) -> bool:
    """연속 촬영 모드로 카메라 설정"""
    try:
        nm = cam.GetNodeMap()
        set_enum(nm, "TriggerMode", "Off")
        set_enum(nm, "AcquisitionMode", "Continuous")
        dbg(f"✅ [{cam_serial}] 연속 촬영 모드 설정 완료")
        return True
    except Exception as e:
        dbg(f"❌ [{cam_serial}] 설정 오류: {e}")
        return False

def get_unique_filename(base_path, serial, initial_count):
    """
    기본 경로와 시퀀스 번호를 사용하여 파일명이 이미 존재하는지 확인하고,
    고유한 파일명을 반환합니다.
    """
    count = initial_count
    # 초기 파일명 생성
    filename = os.path.join(base_path, f"image_{count:03d}.jpg")

    # 파일이 존재하면 count를 1씩 증가시키면서 고유한 파일명을 찾습니다.
    while os.path.exists(filename):
        count += 1
        filename = os.path.join(base_path, f"image_{count:03d}.jpg")

    return filename

def sequential_matching(cameras: List[PySpin.CameraPtr],
                        camera_serials: List[str],
                        master_idx: int,
                        num_images: int,
                        save_images: bool) -> Dict:
    """
    순차 매칭 (Master-Slave Sequential)
    Master 프레임 획득 후 즉시 Slave 프레임을 획득하여 1:1로 매칭합니다.

    Args:
        cameras: 카메라 리스트
        camera_serials: 시리얼 리스트
        master_idx: Master 카메라 인덱스
        num_images: 저장할 동기화 이미지 수
        save_images: 저장 여부

    Returns:
        Dict: 성능 통계
    """
    dbg(f"\n▶ Sequential Matching 동기화 시작...")
    dbg(f"   Master 카메라: {camera_serials[master_idx]}")

    slave_idx = 1 - master_idx

    stats = {
        'method': 'Sequential Matching',
        'synced_pairs': 0,
        'total_frames': [0, 0],
        'dropped_frames': [0, 0], # 불완전하거나 획득 실패한 프레임
        'timestamp_diffs': [],
        'processing_time': 0
    }

    start_time = time.time()

    try:
        # 카메라 초기화 및 촬영 시작
        for cam, serial in zip(cameras, camera_serials):
            cam.Init()
            configure_continuous_acquisition(cam, serial)
            cam.BeginAcquisition()
            dbg(f"✅ [{serial}] 연속 촬영 시작")

        processor = PySpin.ImageProcessor()
        processor.SetColorProcessing(PySpin.SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR)

        synced_count = 0
        dbg(f"\n목표: {num_images}개의 동기화된 이미지 쌍\n")

        # 카메라 간 타임스탬프 오프셋 계산용
        first_master_ts: Optional[int] = None
        timestamp_offset_ns: Optional[int] = None

        while synced_count < num_images:

            master_image = None
            slave_image = None

            try:
                # ===== 1. Master에서 프레임 가져오기 =====
                master_image = cameras[master_idx].GetNextImage(1000)
                if master_image.IsIncomplete():
                    dbg("   ⚠️ Master 프레임 불완전, 건너뜀")
                    stats['dropped_frames'][master_idx] += 1
                    master_image.Release()
                    continue

                stats['total_frames'][master_idx] += 1
                master_timestamp = master_image.GetTimeStamp()

                # ===== 2. Slave에서 다음 프레임 즉시 가져오기 =====
                slave_image = cameras[slave_idx].GetNextImage(1000)
                if slave_image.IsIncomplete():
                    dbg("   ⚠️ Slave 프레임 불완전, Master/Slave 모두 건너뜀")
                    stats['dropped_frames'][master_idx] += 1
                    stats['dropped_frames'][slave_idx] += 1
                    master_image.Release()
                    slave_image.Release()
                    continue

                stats['total_frames'][slave_idx] += 1
                slave_timestamp = slave_image.GetTimeStamp()

                # ===== 3. 타임스탬프 및 통계 계산 =====
                # 첫 프레임 쌍으로 오프셋 계산
                if timestamp_offset_ns is None:
                    first_master_ts = master_timestamp
                    timestamp_offset_ns = slave_timestamp - master_timestamp
                    dbg(f"   ⚙️ 타임스탬프 오프셋 설정: {timestamp_offset_ns / 1_000_000:.3f} ms")

                # 오프셋 보정
                aligned_slave_ts = slave_timestamp - timestamp_offset_ns
                diff = abs(aligned_slave_ts - master_timestamp)
                stats['timestamp_diffs'].append(diff)

                synced_count += 1

                dbg(f"✅ 동기화 성공 ({synced_count}/{num_images}) | "
                    f"TS 차이: {diff / 1_000_000:.3f} ms")

                # ===== 4. 이미지 저장 (필요 시) =====
                if save_images:
                    # 마스터 이미지 저장
                    master_converted = processor.Convert(master_image, PySpin.PixelFormat_BGR8)
                    master_img_obj = PySpin.Image.Create(
                        master_converted.GetWidth(), master_converted.GetHeight(), 0, 0,
                        PySpin.PixelFormat_BGR8, master_converted.GetData()
                    )
                    
                    # 📌 고유 파일명 생성 로직 적용
                    master_base_dir = "./calibration/cam1"
                    master_serial = camera_serials[master_idx]
                    # 'synced_count-1'를 초기 카운트로 사용합니다.
                    master_filename = get_unique_filename(master_base_dir, master_serial, synced_count - 1)
                    
                    # 디렉토리가 없으면 생성 (안전하게)
                    os.makedirs(master_base_dir, exist_ok=True)
                    master_img_obj.Save(master_filename)
                    dbg(f"   💾 저장: {master_filename}")

                    # 슬레이브 이미지 저장
                    slave_converted = processor.Convert(slave_image, PySpin.PixelFormat_BGR8)
                    slave_img_obj = PySpin.Image.Create(
                        slave_converted.GetWidth(), slave_converted.GetHeight(), 0, 0,
                        PySpin.PixelFormat_BGR8, slave_converted.GetData()
                    )

                    # 📌 고유 파일명 생성 로직 적용
                    slave_base_dir = "./calibration/cam2"
                    slave_serial = camera_serials[slave_idx]
                    # 'synced_count-1'를 초기 카운트로 사용합니다.
                    slave_filename = get_unique_filename(slave_base_dir, slave_serial, synced_count - 1)
                    
                    # 디렉토리가 없으면 생성 (안전하게)
                    os.makedirs(slave_base_dir, exist_ok=True)
                    slave_img_obj.Save(slave_filename)
                    dbg(f"   💾 저장: {slave_filename}")

            except PySpin.SpinnakerException as e:
                dbg(f"   ❌ 프레임 획득 중 오류: {e}")
                # 오류 발생 시 해당 프레임 쌍은 건너뜀
                stats['dropped_frames'][master_idx] += 1
                stats['dropped_frames'][slave_idx] += 1
                continue
            finally:
                if master_image:
                    master_image.Release()
                if slave_image:
                    slave_image.Release()

        stats['synced_pairs'] = synced_count
        stats['processing_time'] = time.time() - start_time
        return stats

    except Exception as e:
        dbg(f"❌ 심각한 오류: {e}")
        import traceback
        traceback.print_exc()
        return stats

    finally:
        for cam, serial in zip(cameras, camera_serials):
            try:
                if cam.IsInitialized():
                    cam.EndAcquisition()
                    dbg(f"   [{serial}] 촬영 종료")
                    cam.DeInit()
            except Exception as e:
                dbg(f"   ❌ [{serial}] 정리 중 오류: {e}")


def print_statistics(stats: Dict):
    """통계 출력"""
    if not stats or 'method' not in stats:
        print("\n통계 정보가 없습니다.")
        return

    print("\n" + "="*60)
    print(f"📊 {stats['method']} 성능 분석")
    print("="*60)

    print(f"\n✅ 동기화 결과:")
    print(f"   - 성공적으로 동기화된 이미지 쌍: {stats.get('synced_pairs', 0)}개")

    print(f"\n📈 프레임 통계:")
    total_frames_list = stats.get('total_frames', [0, 0])
    dropped_frames_list = stats.get('dropped_frames', [0, 0])

    for idx in range(len(total_frames_list)):
        cam_type = "Master" if idx == MASTER_CAMERA_IDX else "Slave"
        total = total_frames_list[idx]
        dropped = dropped_frames_list[idx]

        print(f"   Camera {idx} ({cam_type}):")
        print(f"     총 획득 시도: {total}장")
        print(f"     드롭 (불완전/오류): {dropped}장")

        if total > 0:
            drop_rate = (dropped / total) * 100
            print(f"     드롭률: {drop_rate:.1f}%")

    timestamp_diffs = stats.get('timestamp_diffs', [])
    if timestamp_diffs:
        print(f"\n🎯 동기화 정확도 (타임스탬프 차이):")
        avg_diff = sum(timestamp_diffs) / len(timestamp_diffs)
        max_diff = max(timestamp_diffs)
        min_diff = min(timestamp_diffs)
        std_dev = (sum([(x - avg_diff)**2 for x in timestamp_diffs]) / len(timestamp_diffs))**0.5

        print(f"   - 평균 차이: {avg_diff / 1_000_000:.3f} ms")
        print(f"   - 최대 차이: {max_diff / 1_000_000:.3f} ms")
        print(f"   - 최소 차이: {min_diff / 1_000_000:.3f} ms")
        print(f"   - 표준편차: {std_dev / 1_000_000:.3f} ms")

    processing_time = stats.get('processing_time', 0)
    synced_pairs = stats.get('synced_pairs', 0)
    if processing_time > 0:
        print(f"\n⏱️ 처리 시간:")
        print(f"   - 총 처리 시간: {processing_time:.2f}초")
        if synced_pairs > 0:
            print(f"   - 이미지 쌍당 평균 시간: {processing_time / synced_pairs:.2f}초")

    print("="*60)


def main() -> None:
    """메인 함수"""
    # Retrieve singleton reference to system object
    system = PySpin.System.GetInstance()

    # Retrieve list of cameras from the system
    cam_list = system.GetCameras()

    cnt = cam_list.GetSize()
    dbg(f"📦 감지된 카메라: {cnt}개\n")

    cameras = []
    found_serials = []
    stats = {}

    try:
        if cnt < len(CAMERA_SERIALS):
            print(f"❌ {len(CAMERA_SERIALS)}대의 카메라가 필요하지만 {cnt}대만 감지되었습니다.")
            return

        dbg("🔎 카메라 검색 중...")
        for serial in CAMERA_SERIALS:
            cam = cam_list.GetBySerial(serial)
            if cam is None:
                print(f"❌ 시리얼 번호 {serial}에 해당하는 카메라를 찾을 수 없습니다.")
                return

            tl = cam.GetTLDeviceNodeMap()
            model = PySpin.CStringPtr(tl.GetNode("DeviceModelName")).GetValue()
            dbg(f"   ✅ {model} (SN: {serial}) 발견")

            cameras.append(cam)
            found_serials.append(serial)

        # 순차 매칭 실행
        stats = sequential_matching(
            cameras,
            found_serials,
            MASTER_CAMERA_IDX,
            NUM_SYNCED_IMAGES,
            SAVE_IMAGES
        )

    except Exception as e:
        print(f"❌ 메인 함수 오류 발생: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # 리소스 정리
        cameras.clear()
        found_serials.clear()

        if 'cam_list' in locals() and cam_list.GetSize() > 0:
            cam_list.Clear()

        try:
            system.ReleaseInstance()
        except PySpin.SpinnakerException:
            pass

        gc.collect()

        # 최종 통계 출력
        print_statistics(stats)


if __name__ == "__main__":

    main()
