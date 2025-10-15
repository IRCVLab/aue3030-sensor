"""
임계값 기반 매칭 (마스터 우선 전략)

원리:
1. 한 카메라를 기준(Master)으로 설정
2. Master의 각 프레임에 대해 다른 카메라(Slave)에서
   Master 프레임 타임스탬프와 임계값 (Threshold) 이내의 차이를 보이는 프레임을 찾음
3. (선택) 임계값 이내의 프레임 중 가장 가까운 쌍을 동기화된 프레임으로 선택
4. 임계값 이내의 프레임이 없으면 동기화 실패로 간주하고 매칭된 이미지 없이 통과(Pass)
   (Master 프레임과 Slave 버퍼의 모든 프레임 버림)

장점:
- 동기화 정확도가 높음 (최대 오차가 임계값으로 제한됨)
- 동기화 실패 시 명확한 처리 가능

단점:
- 임계값 설정이 필요 (프레임 레이트, 하드웨어 동기 상태에 따라 달라짐)
- 임계값 밖의 프레임은 버려지므로 프레임 손실이 발생할 수 있음
"""

import gc
import time

import settings
import PySpin

from collections import deque
from typing import Optional, List, Dict, Tuple


# ============================================================
# 사용자 설정
# ============================================================
CAMERA_SERIALS = settings.CAMERA_SERIALS
MASTER_CAMERA_IDX = 0  # Master 카메라 인덱스 (0 또는 1)
NUM_SYNCED_IMAGES = 10  # 저장할 동기화된 이미지 수
BUFFER_SIZE = 30  # 각 카메라의 이미지 버퍼 크기
SAVE_IMAGES = True
VERBOSE = True  # 출력 최소화: 최종 PASS 개수만 출력
# ============================================================
# 임계값 설정 (1000ms)
# ============================================================
# PySpin 타임스탬프는 나노초(ns) 단위
SYNC_THRESHOLD_NS = 1_000_000_000 # 1000ms
# ============================================================


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
        set_enum(nm, "TriggerMode", "Off")              # Q. TriggerMode의 다른 옵션은 어떤 것이 있을까?
        set_enum(nm, "AcquisitionMode", "Continuous")   # Q. AcquisitionMode의 다른 옵션은 어떤 것이 있을까?
                                                        # Hint: L56 의 "GetEntryByName" 정의로 이동해서 주변 함수 살펴보기
        dbg(f"✅ [{cam_serial}] 연속 촬영 모드 설정 완료")
        return True
    except Exception as e:
        dbg(f"❌ [{cam_serial}] 설정 오류: {e}")
        return False


class TimestampBuffer:
    """타임스탬프 기반 이미지 버퍼 (threshold matching용, 상대 타임스탬프 사용)"""

    def __init__(self, max_size: int):
        self.buffer = deque(maxlen=max_size)
        self.used_ids = set()  # 사용된 이미지 고유 ID 추적
        self.first_timestamp = None  # 첫 프레임 기준점
        self.next_id = 0  # 고유 ID 카운터 (절대 중복되지 않음)

    def add(self, timestamp: int, image_data: bytes, width: int, height: int):
        """이미지 추가 (상대 타임스탬프로 변환)"""
        if self.first_timestamp is None:
            self.first_timestamp = timestamp
            relative_ts = 0
        else:
            relative_ts = timestamp - self.first_timestamp

        # 버퍼가 꽉 차면 가장 오래된 항목이 자동으로 제거됨
        if len(self.buffer) >= self.buffer.maxlen:
            removed = self.buffer[0]  # 제거될 항목
            self.used_ids.discard(removed['id'])  # used_ids에서도 제거

        unique_id = self.next_id
        self.next_id += 1

        self.buffer.append({
            'id': unique_id,      # 고유 ID (절대 중복 없음)
            'timestamp': relative_ts,  # 상대 타임스탬프
            'abs_timestamp': timestamp,  # 절대 타임스탬프
            'data': image_data,
            'width': width,
            'height': height
        })

    def find_nearest_unused_within_threshold(self, target_timestamp: int, threshold: int) -> Optional[Tuple[Dict, int]]:
        """
        타겟 타임스탬프에 가장 가까운 미사용 이미지를 임계값 내에서 찾기

        Returns:
            (image_dict, timestamp_diff) 튜플 또는 None
        """
        nearest = None
        min_diff = float('inf')

        for img in self.buffer:
            if img['id'] in self.used_ids:
                continue

            diff = abs(img['timestamp'] - target_timestamp)

            # 임계값 검사
            if diff <= threshold:
                if diff < min_diff:
                    min_diff = diff
                    nearest = img

        if nearest:
            return nearest, min_diff
        return None

    def mark_used(self, img: Dict):
        """이미지를 사용됨으로 표시"""
        self.used_ids.add(img['id'])

    def clear(self):
        """버퍼 비우기"""
        self.buffer.clear()
        self.used_ids.clear()

    def size(self) -> int:
        """버퍼 크기"""
        return len(self.buffer)

    def unused_count(self) -> int:
        """미사용 이미지 수"""
        return len(self.buffer) - len(self.used_ids)

    def get_oldest_unused(self) -> Optional[Dict]:
        """가장 오래된 미사용 이미지 (마스터 프레임 처리용)"""
        for img in self.buffer:
            if img['id'] not in self.used_ids:
                return img
        return None


def threshold_matching(cameras: List[PySpin.CameraPtr],
                               camera_serials: List[str],
                               master_idx: int,
                               num_images: int,
                               save_images: bool,
                               threshold_ns: int) -> Dict:
    """
    임계값 기반 타임스탬프 매칭 (버퍼 없이 실시간 처리)

    Args:
        cameras: 카메라 리스트
        camera_serials: 시리얼 리스트
        master_idx: Master 카메라 인덱스
        num_images: 저장할 동기화 이미지 수
        save_images: 저장 여부
        threshold_ns: 동기화 허용 임계값 (나노초)

    Returns:
        Dict: 성능 통계
    """
    dbg(f"\n▶ Threshold-Based Matching 동기화 시작 (임계값: {threshold_ns/1_000_000:.1f} ms)...")
    dbg(f"   Master 카메라: {camera_serials[master_idx]}")

    slave_idx = 1 - master_idx

    stats = {
        'method': 'Threshold-Based Matching',
        'threshold_ms': threshold_ns / 1_000_000,
        'synced_pairs': 0,
        'passed_masters': 0,
        'total_frames': [0, 0],
        'dropped_frames': [0, 0],
        'timestamp_diffs': [],
        'sync_accuracy_ns': [],
        'processing_time': 0
    }

    start_time = time.time()

    try:
        for cam, serial in zip(cameras, camera_serials):
            # Initialize camera
            cam.Init()
            configure_continuous_acquisition(cam, serial)

            #  Begin acquiring images
            #
            #  *** NOTES ***
            #  What happens when the camera begins acquiring images depends on the
            #  acquisition mode. Single frame captures only a single image, multi
            #  frame catures a set number of images, and continuous captures a
            #  continuous stream of images.
            #
            #  *** LATER ***
            #  Image acquisition must be ended when no more images are needed.
            cam.BeginAcquisition()
            dbg(f"✅ [{serial}] 연속 촬영 시작")

        # Create ImageProcessor instance for post processing images
        # spinnaker/docs/PySpinDoc.pdf 참고
        processor = PySpin.ImageProcessor()

        # Set default image processor color processing method
        #
        # *** NOTES ***
        # By default, if no specific color processing algorithm is set, the image
        # processor will default to NEAREST_NEIGHBOR method.
        #
        # 자세한 내용은 아래 링크 참고
        # https://softwareservices.flir.com/spinnaker/latest/group___spinnaker_defs.html#gab8d72f72c9674cab70d99975691eb54d
        # Q. spinnaker에서 지원하는 다른 color processing algorithm 은 어떤 것이 있는지?
        #    다른 알고리즘을 적용했을 때, 획득한 영상이 어떻게 달라졌는지?
        processor.SetColorProcessing(PySpin.SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR)

        synced_count = 0
        dbg(f"\n목표: {num_images}개의 동기화된 이미지 쌍\n")

        # 첫 번째 타임스탬프 기준점 및 카메라 간 오프셋
        first_timestamp = None
        timestamp_offset_ns: Optional[int] = None

        # 필요한 동기화 쌍을 얻을 때까지 반복
        while synced_count < num_images:

            # ===== 1. Master에서 프레임 가져오기 =====
            try:
                #  Retrieve next received image
                # Q. GetNextImage 함수에 들어간 인자 1000은 무슨 의미인가?
                master_image = cameras[master_idx].GetNextImage(1000)
                if master_image.IsIncomplete():
                    master_image.Release()
                    continue

                master_timestamp = master_image.GetTimeStamp()
                if first_timestamp is None:
                    first_timestamp = master_timestamp
                master_relative_ts = master_timestamp - first_timestamp

                master_width = master_image.GetWidth()
                master_height = master_image.GetHeight()

                # Q. PySpin.PixelFormat_BGR8 은 어떤 의미인지?
                # Q. PySpin.PixelFormat_YCbCr422_8_CbYCrY 은 어떤 차이가 있나?
                master_converted = processor.Convert(master_image, PySpin.PixelFormat_BGR8)
                master_data = master_converted.GetData()
                master_image.Release()

                stats['total_frames'][master_idx] += 1

            except PySpin.SpinnakerException:
                continue

            # ===== 2. Slave에서 매칭되는 프레임 찾기 =====
            found_match = False
            best_slave_img = None
            best_diff = float('inf')
            slave_attempts = 0
            max_slave_attempts = 20  # Slave에서 최대 20장까지 시도

            dbg(f"🔍 Master TS {master_relative_ts / 1_000_000:.3f} ms - Slave 프레임 검색 중...")

            while slave_attempts < max_slave_attempts:
                try:
                    slave_image = cameras[slave_idx].GetNextImage(1000)
                    if slave_image.IsIncomplete():
                        slave_image.Release()
                        slave_attempts += 1
                        dbg(f"   ⚠️ Slave 프레임 불완전 (시도 {slave_attempts}/{max_slave_attempts})")
                        continue

                    slave_timestamp = slave_image.GetTimeStamp()

                    if timestamp_offset_ns is None:
                        timestamp_offset_ns = slave_timestamp - master_timestamp
                        dbg(f"   ⚙️ 타임스탬프 오프셋 설정: {timestamp_offset_ns / 1_000_000:.3f} ms")

                    aligned_slave_ts = slave_timestamp - timestamp_offset_ns

                    stats['total_frames'][slave_idx] += 1
                    slave_attempts += 1

                    should_break = False

                    # 타임스탬프 차이 계산 (오프셋 보정 후)
                    diff = abs(aligned_slave_ts - master_timestamp)
                    slave_relative_ts = aligned_slave_ts - first_timestamp

                    dbg(f"   Slave(보정) TS {slave_relative_ts / 1_000_000:.3f} ms | 차이: {diff / 1_000_000:.3f} ms")

                    # Threshold 내에 있으면 매칭 성공
                    if diff <= threshold_ns:
                        # 더 나은 매칭 발견
                        if diff < best_diff:
                            # 이전 best가 있으면 버림
                            if best_slave_img is not None:
                                stats['dropped_frames'][slave_idx] += 1

                            best_diff = diff
                            slave_width = slave_image.GetWidth()
                            slave_height = slave_image.GetHeight()
                            slave_converted = processor.Convert(slave_image, PySpin.PixelFormat_BGR8)
                            best_slave_img = {
                                'data': slave_converted.GetData(),
                                'width': slave_width,
                                'height': slave_height,
                                'timestamp': slave_relative_ts,
                                'aligned_timestamp': aligned_slave_ts,
                                'raw_timestamp': slave_timestamp
                            }
                            found_match = True
                            dbg(f"   ✅ 매칭 후보 발견!")
                        else:
                            # 더 나쁜 매칭은 버림
                            stats['dropped_frames'][slave_idx] += 1
                    else:
                        # Threshold 밖이면 버림
                        stats['dropped_frames'][slave_idx] += 1
                        dbg(f"   ❌ Threshold 밖 (버림)")

                        # Master보다 너무 앞서면 계속 시도
                        if aligned_slave_ts < master_timestamp - threshold_ns:
                            dbg(f"   ⏪ Slave가 뒤처짐, 다음 프레임 계속")
                            pass  # 다음 Slave 프레임 시도
                        elif aligned_slave_ts > master_timestamp + threshold_ns:
                            dbg(f"   ⏩ Slave가 너무 앞섬, 포기")
                            should_break = True

                    slave_image.Release()
                    if should_break:
                        break

                except PySpin.SpinnakerException as e:
                    slave_attempts += 1
                    dbg(f"   ⚠️ Slave 프레임 획득 실패: {e}")
                    time.sleep(0.05)

            # Q. master와 slave의 동기화 방법을 위 코드를 활용해서 설명하시오.

            # ===== 3. 결과 처리 =====
            if found_match and best_slave_img is not None:
                # 동기화 성공
                synced_count += 1
                stats['timestamp_diffs'].append(best_diff)
                stats['sync_accuracy_ns'].append(best_diff)

                if save_images:
                    # Master 저장
                    master_img = PySpin.Image.Create(master_width, master_height, 0, 0,
                                                     PySpin.PixelFormat_BGR8, master_data)
                    master_filename = f"threshold_sync_{camera_serials[master_idx]}_{synced_count-1:03d}.jpg"
                    master_img.Save(master_filename)
                    dbg(f"   💾 저장: {master_filename}")

                    # Slave 저장
                    slave_img = PySpin.Image.Create(best_slave_img['width'], best_slave_img['height'],
                                                    0, 0, PySpin.PixelFormat_BGR8, best_slave_img['data'])
                    slave_filename = f"threshold_sync_{camera_serials[slave_idx]}_{synced_count-1:03d}.jpg"
                    slave_img.Save(slave_filename)
                    dbg(f"   💾 저장: {slave_filename}")

                dbg("-" * 20)
            else:
                # 동기화 실패 (PASS)
                stats['passed_masters'] += 1
                stats['dropped_frames'][master_idx] += 1
                # 패스 로그 생략

        stats['synced_pairs'] = synced_count
        stats['processing_time'] = time.time() - start_time

        # 완료 로그 생략
        return stats

    except Exception as e:
        dbg(f"❌ 오류: {e}")
        import traceback
        traceback.print_exc()
        return stats

    finally:
        for cam, serial in zip(cameras, camera_serials):
            try:
                #  End acquisition
                #
                #  *** NOTES ***
                #  Ending acquisition appropriately helps ensure that devices clean up
                #  properly and do not need to be power-cycled to maintain integrity.
                cam.EndAcquisition()
                dbg(f"   [{serial}] 촬영 종료")
            except Exception:
                pass
            try:
                # Deinitialize camera
                cam.DeInit()
            except Exception:
                pass


def print_statistics(stats: Dict):
    """통계 출력"""
    print("\n" + "="*60)
    print(f"📊 {stats['method']} 성능 분석 (Threshold: {stats['threshold_ms']:.1f} ms)")
    print("="*60)

    print(f"\n✅ 동기화 성공:")
    print(f"   - 동기화된 이미지 쌍: {stats['synced_pairs']}개")
    print(f"   - 임계값 불일치 (PASS): {stats['passed_masters']}개")

    print(f"\n📈 프레임 통계:")
    for idx in range(len(stats['total_frames'])):
        cam_type = "Master" if idx == MASTER_CAMERA_IDX else "Slave"
        print(f"   Camera {idx} ({cam_type}):")
        print(f"     총 촬영: {stats['total_frames'][idx]}장")
        print(f"     미사용/드롭: {stats['dropped_frames'][idx]}장")
        if stats['total_frames'][idx] > 0:
            # 마스터의 드롭률 계산 시 Passed 프레임을 포함
            unutilized = stats['dropped_frames'][idx] + (stats['passed_masters'] if idx == MASTER_CAMERA_IDX else 0)
            unutilized_rate = (unutilized / stats['total_frames'][idx]) * 100
            print(f"     미사용률: {unutilized_rate:.1f}%")

    if stats['sync_accuracy_ns']:
        print(f"\n🎯 동기화 정확도:")
        avg_diff = sum(stats['sync_accuracy_ns']) / len(stats['sync_accuracy_ns'])
        max_diff = max(stats['sync_accuracy_ns'])
        min_diff = min(stats['sync_accuracy_ns'])
        print(f"   - 평균 타임스탬프 차이: {avg_diff / 1_000_000:.3f} ms")
        print(f"   - 최대 차이: {max_diff / 1_000_000:.3f} ms (<= {stats['threshold_ms']:.1f} ms)")
        print(f"   - 최소 차이: {min_diff / 1_000_000:.3f} ms")
        print(f"   - 표준편차: {(sum([(x - avg_diff)**2 for x in stats['sync_accuracy_ns']]) / len(stats['sync_accuracy_ns']))**0.5 / 1_000_000:.3f} ms")

    print(f"\n⏱️ 처리 시간:")
    print(f"   - 총 처리 시간: {stats['processing_time']:.2f}초")
    if stats['synced_pairs'] > 0:
        print(f"   - 이미지 쌍당: {stats['processing_time'] / stats['synced_pairs']:.2f}초")

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
    stats: Dict = {'passed_masters': 0}

    try:
        if cnt < len(CAMERA_SERIALS):
            print(f"❌ {len(CAMERA_SERIALS)}대의 카메라가 필요합니다.")
            return

        dbg("🔎 카메라 검색 중...")
        for serial in CAMERA_SERIALS:
            cam = cam_list.GetBySerial(serial)
            if cam is None:
                print(f"❌ 카메라를 찾을 수 없음: {serial}")
                return

            # Retrieve TL device nodemap and print device information
            tl = cam.GetTLDeviceNodeMap()
            model = PySpin.CStringPtr(tl.GetNode("DeviceModelName")).GetValue()
            dbg(f"   ✅ {model} (SN: {serial})")

            cameras.append(cam)
            found_serials.append(serial)

        # Threshold-Based Matching 실행
        stats = threshold_matching(
            cameras,
            found_serials,
            MASTER_CAMERA_IDX,
            NUM_SYNCED_IMAGES,
            SAVE_IMAGES,
            SYNC_THRESHOLD_NS  # 1000ms (1초) 임계값 전달
        )

    except Exception as e:
        print(f"❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()

    finally:
        for i in range(len(cameras)):
            cameras[i] = None
        cameras.clear()
        found_serials.clear()

        try:
            cam_list.Clear()
            dbg("\n🧹 리소스 정리 완료")
        except Exception:
            pass

        try:
            del cameras, found_serials, cam_list
        except Exception:
            pass

        gc.collect()

        try:
            system.ReleaseInstance()
        except Exception:
            pass
        finally:
            try:
                del system
            except Exception:
                pass
    # 최종 패스 개수만 출력
    if isinstance(stats, dict) and 'passed_masters' in stats:
        print(stats['passed_masters'])


if __name__ == "__main__":
    main()