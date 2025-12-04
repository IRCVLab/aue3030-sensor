"""
BFS-only Hardware Trigger Setup (Master → Slave)

Master (BFS):
  - Line1 as Output
  - LineSource = ExposureActive (fallbacks applied)
  - Enable 3.3V on Line2
Slave (BFS):
  - TriggerSelector = FrameStart
  - TriggerSource = Line3
  - TriggerActivation = RisingEdge
  - TriggerOverlap = ReadOut
  - TriggerMode = On
"""

import gc
import PySpin
import settings

from typing import Optional, Tuple


# ======== USER CONFIG ========
MASTER_CAM_SERIAL = settings.CAMERA_SERIALS[0]
SLAVE_CAM_SERIAL  = settings.CAMERA_SERIALS[1]
SAVE_TO_USER_SET  = False         # True to persist settings
USER_SET_NAME     = "UserSet0"    # "UserSet0" or "UserSet1"
# =============================


def dbg(msg: str) -> None:
    """Print a debug message."""
    print(msg)


# ---------- Generic helpers with debug ----------
def set_enum(nm: PySpin.INodeMap, node_name: str, entry_name: str) -> bool:
    """
    Set an enumeration node to a specific entry; prints detailed debug.

    Args:
        nm (PySpin.INodeMap): Node map (camera.GetNodeMap()).
        node_name (str): Enum node name (e.g., "TriggerSource").
        entry_name (str): Enum entry (e.g., "Line3").

    Returns:
        bool: True if set successfully; False otherwise.
    """
    enum_node = PySpin.CEnumerationPtr(nm.GetNode(node_name))
    if not (PySpin.IsAvailable(enum_node) and PySpin.IsWritable(enum_node)):
        dbg(f"❌ set_enum('{node_name}','{entry_name}'): enum not available/writable")
        return False
    entry = enum_node.GetEntryByName(entry_name)
    if not (PySpin.IsAvailable(entry) and PySpin.IsReadable(entry)):
        dbg(f"❌ set_enum('{node_name}','{entry_name}'): entry not available/readable")
        return False
    enum_node.SetIntValue(entry.GetValue())
    dbg(f"✅ set_enum('{node_name}','{entry_name}')")
    return True


def set_bool(nm: PySpin.INodeMap, node_name: str, value: bool) -> bool:
    """
    Set a boolean node; prints detailed debug.

    Args:
        nm (PySpin.INodeMap): Node map.
        node_name (str): Boolean node name.
        value (bool): Desired state.

    Returns:
        bool: True if success; False otherwise.
    """
    bnode = PySpin.CBooleanPtr(nm.GetNode(node_name))
    if not (PySpin.IsAvailable(bnode) and PySpin.IsWritable(bnode)):
        dbg(f"❌ set_bool('{node_name}', {value}): bool not available/writable")
        return False
    bnode.SetValue(bool(value))
    dbg(f"✅ set_bool('{node_name}', {value})")
    return True


def get_str(nm: PySpin.INodeMap, node_name: str) -> str:
    """
    Read a CString node as string if available/readable; prints debug.

    Args:
        nm (PySpin.INodeMap): Node map (TL or device).
        node_name (str): Node name.

    Returns:
        str: Value or 'N/A'.
    """
    node = PySpin.CStringPtr(nm.GetNode(node_name))
    if PySpin.IsAvailable(node) and PySpin.IsReadable(node):
        val = node.GetValue()
        dbg(f"ℹ️ get_str('{node_name}') = '{val}'")
        return val
    dbg(f"❌ get_str('{node_name}'): not available/readable")
    return "N/A"


def exec_cmd(nm: PySpin.INodeMap, node_name: str) -> bool:
    """
    Execute a command node; prints debug.

    Args:
        nm (PySpin.INodeMap): Node map (requires cam.Init()).
        node_name (str): Command node name (e.g., "UserSetSave").

    Returns:
        bool: True if executed; False otherwise.
    """
    cmd = PySpin.CCommandPtr(nm.GetNode(node_name))
    if not (PySpin.IsAvailable(cmd) and PySpin.IsWritable(cmd)):
        dbg(f"❌ exec_cmd('{node_name}'): command not available/writable")
        return False
    try:
        cmd.Execute()
        dbg(f"✅ exec_cmd('{node_name}')")
        return True
    except Exception as e:
        dbg(f"❌ exec_cmd('{node_name}') failed: {e}")
        return False


def save_to_userset(nm: PySpin.INodeMap, userset: str) -> bool:
    """
    Save current settings to a user set and set default; prints debug.

    Args:
        nm (PySpin.INodeMap): Node map (requires cam.Init()).
        userset (str): "UserSet0" or "UserSet1".

    Returns:
        bool: True if saved and default set; False otherwise.
    """
    dbg(f"💾 Saving to userset '{userset}' ...")
    ok_sel = set_enum(nm, "UserSetSelector", userset)
    # In Spinnaker, UserSetSave is a Command node.
    ok_save = exec_cmd(nm, "UserSetSave")
    ok_def = set_enum(nm, "UserSetDefault", userset)
    ok = bool(ok_sel and ok_save and ok_def)
    dbg("✅ Userset saved & default set" if ok else "❌ Userset save/default failed")
    return ok


# ---------- BFS-specific helpers ----------
def enable_bfs_3v3_on_line2(nm: PySpin.INodeMap) -> bool:
    """
    Enable 3.3V line driving on Line2 for BFS

    Args:
        nm (PySpin.INodeMap): Device node map (requires cam.Init()).

    Returns:
        bool: True if any known node successfully enabled 3.3V on Line2; False otherwise.
    """
    dbg("⚙️ Enable 3.3V on Line2 for BFS ...")
    if not set_enum(nm, "LineSelector", "Line2"):
        dbg("❌ Cannot select Line2; skip 3.3V enable")
        return False

    candidates = [
        "V3_3Enable",        # Common BFS naming
        "Line3V3Enable",     # Alternative
        "LineVoltageEnable", # Some firmware variants
    ]
    for name in candidates:
        # FIX: this must be True to enable 3.3V
        if set_bool(nm, name, True):
            dbg(f"✅ 3.3V enabled via '{name}' on Line2")
            return True

    dbg("❌ 3.3V enable failed (no matching node)")
    return False


# ---------- Camera discovery ----------
def get_cam_by_serial(cam_list: PySpin.CameraList, serial: str) -> Optional[PySpin.CameraPtr]:
    """
    Locate a camera by serial number via TL device nodemap (no Init needed). Prints debug table.

    Args:
        cam_list (PySpin.CameraList): system.GetCameras() result.
        serial (str): Target serial number.

    Returns:
        Optional[PySpin.CameraPtr]: Camera pointer if found; None otherwise.
    """
    dbg(f"🔎 Searching for camera with Serial={serial} ...")
    found = None
    for i in range(cam_list.GetSize()):
        cam = cam_list.GetByIndex(i)
        tl = cam.GetTLDeviceNodeMap()
        model = get_str(tl, "DeviceModelName")
        sn    = get_str(tl, "DeviceSerialNumber")
        dbg(f"   - idx {i}: Model='{model}' SN={sn}")
        if sn == serial:
            found = cam
    dbg("✅ Camera found" if found else "❌ Camera not found")
    return found


# ---------- BFS Master / Slave configuration ----------
def configure_bfs_master(cam: PySpin.CameraPtr) -> Tuple[str, str]:
    """
    Configure BFS Master to output trigger signal per SpinView guide (with debug).

    Args:
        cam (PySpin.CameraPtr): BFS master camera pointer (uninitialized).

    Returns:
        Tuple[str, str]: (output_line, line_source)
            output_line  : The GPIO line used as output (e.g., "Line1").
            line_source  : The signal source driving the line (e.g., "ExposureActive").
    """
    dbg("🛠 Configuring BFS MASTER ...")
    cam.Init()
    try:
        nm = cam.GetNodeMap()

        set_enum(nm, "AcquisitionMode", "Continuous")
        set_enum(nm, "TriggerSelector", "FrameStart")
        set_enum(nm, "TriggerMode", "Off")  # Free-run master (ENUM On/Off)

        # BFS output line is Line1
        output_line = "Line1"
        set_enum(nm, "LineSelector", output_line)
        set_enum(nm, "LineMode", "Output")

        # Prefer ExposureActive; apply safe fallbacks
        line_source = "ExposureActive"
        if not set_enum(nm, "LineSource", line_source):
            for alt in ("FrameTriggerWait", "UserOutput0"):
                if set_enum(nm, "LineSource", alt):
                    line_source = alt
                    break
            else:
                dbg("❌ No valid LineSource could be set for MASTER")

        # Also enable 3.3V on Line2
        # Q. 이게 왜 필요한건지?
        _ = enable_bfs_3v3_on_line2(nm)

        if SAVE_TO_USER_SET:
            _ = save_to_userset(nm, USER_SET_NAME)

        dbg(f"✅ MASTER configured: Output={output_line}, Source={line_source}")
        return output_line, line_source

    finally:
        try:
            cam.EndAcquisition()
        except Exception:
            pass
        cam.DeInit()
        dbg("↩️ MASTER DeInit complete")


def configure_bfs_slave(cam: PySpin.CameraPtr) -> str:
    """
    Configure BFS Slave to use external trigger (from master's output), with debug.

    Args:
        cam (PySpin.CameraPtr): BFS slave camera pointer (uninitialized).

    Returns:
        str: TriggerSource line used (should be "Line3" for BFS per guide).
    """
    dbg("🛠 Configuring BFS SLAVE ...")
    cam.Init()
    try:
        nm = cam.GetNodeMap()

        # Q. TriggerActivation 다른 옵션은 어떤 것이 있을까?
        # 참고: https://softwareservices.flir.com/bfs-pge-23s3/latest/Model/public/AcquisitionControl.html
        set_enum(nm, "AcquisitionMode", "Continuous")
        set_enum(nm, "TriggerSelector", "FrameStart")
        set_enum(nm, "TriggerActivation", "RisingEdge")
        set_enum(nm, "TriggerOverlap", "ReadOut")

        # Q. 실습자료 PPT 의 GPIO Cable Diagram을 고려할 때, Line2 vs. Line3 어떤 것이 맞을까?
        trg_line = "Line2"
        trg_line = "Line3"
        set_enum(nm, "TriggerSource", trg_line)

        set_enum(nm, "TriggerMode", "On")  # Enable trigger mode (ENUM On/Off)

        if SAVE_TO_USER_SET:
            _ = save_to_userset(nm, USER_SET_NAME)

        dbg(f"✅ SLAVE configured: TriggerSource={trg_line}")
        return trg_line

    finally:
        try:
            cam.EndAcquisition()
        except Exception:
            pass
        cam.DeInit()
        dbg("↩️ SLAVE DeInit complete")


# ---------- Optional acquisition helpers ----------
def start_pair_bfs(master: PySpin.CameraPtr, slave: PySpin.CameraPtr) -> None:
    """
    Start acquisition for BFS pair: slave first (ready to catch triggers), then master.

    Args:
        master (PySpin.CameraPtr): BFS master camera (uninitialized).
        slave (PySpin.CameraPtr): BFS slave camera (uninitialized).

    Returns:
        None
    """
    dbg("▶ Starting acquisition pair (Slave → Master) ...")
    master.Init()
    slave.Init()
    try:
        slave.BeginAcquisition()
        dbg("   - SLAVE BeginAcquisition ✅")
        master.BeginAcquisition()
        dbg("   - MASTER BeginAcquisition ✅")
        dbg("▶ Acquisition started")
    except Exception as e:
        dbg(f"❌ start_pair_bfs error: {e}")
        try:
            master.EndAcquisition()
        except Exception:
            pass
        try:
            slave.EndAcquisition()
        except Exception:
            pass
        raise


def stop_pair_bfs(master: PySpin.CameraPtr, slave: PySpin.CameraPtr) -> None:
    """
    Stop acquisition and de-initialize both BFS cameras.

    Args:
        master (PySpin.CameraPtr): Master camera (initialized).
        slave (PySpin.CameraPtr): Slave camera (initialized).

    Returns:
        None
    """
    dbg("⏹ Stopping acquisition pair ...")
    for name, cam in (("MASTER", master), ("SLAVE", slave)):
        try:
            cam.EndAcquisition()
            dbg(f"   - {name} EndAcquisition ✅")
        except Exception:
            dbg(f"   - {name} EndAcquisition (skipped)")
        try:
            cam.DeInit()
            dbg(f"   - {name} DeInit ✅")
        except Exception:
            dbg(f"   - {name} DeInit (skipped)")


# ---------- Main ----------
def main() -> None:
    """
    Resolve BFS master/slave by serials and configure hardware trigger with verbose logging.

    Args:
        None

    Returns:
        None
    """
    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()
    master = None
    slave = None
    try:
        cnt = cam_list.GetSize()
        dbg(f"📦 Detected cameras: {cnt}")
        if cnt < 2:
            print(f"❌ Need at least 2 cameras; found {cnt}.")
            return

        master = get_cam_by_serial(cam_list, MASTER_CAM_SERIAL)
        slave  = get_cam_by_serial(cam_list, SLAVE_CAM_SERIAL)
        if master is None or slave is None:
            print("❌ Could not find both BFS cameras by serial.")
            return

        # Q. master와 slave setting이 어떻게 다른가?
        out_line, line_src = configure_bfs_master(master)
        in_line = configure_bfs_slave(slave)

        print("✅ BFS Master configured")
        print(f"   - Output Line : {out_line}")
        print(f"   - Line Source : {line_src}")
        print("✅ BFS Slave configured")
        print(f"   - TriggerSource: {in_line}")
        print("✨ BFS hardware trigger setup complete.")

    finally:
        # Drop references BEFORE releasing system to avoid [-1004]
        try:
            cam_list.Clear()
            dbg("🧹 cam_list.Clear()")
        except Exception:
            pass

        # Explicitly drop camera pointers and run GC
        try:
            master = None
            slave = None
            del master
            del slave
        except Exception:
            pass

        try:
            del cam_list
        except Exception:
            pass

        gc.collect()

        try:
            system.ReleaseInstance()
            dbg("🧹 system.ReleaseInstance()")
        finally:
            del system


if __name__ == "__main__":
    main()

