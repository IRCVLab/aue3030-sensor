# -*- coding: utf-8 -*-
"""
Clean camera info printer for PySpin.

Design:
- Single-responsibility functions
- Robust cleanup (Init/DeInit scoped to the function)
- Pretty, width-aware box printing

"""

from typing import Dict, List
import PySpin


def get_node_value(nodemap: PySpin.INodeMap, node_name: str) -> str:
    """
    Read a CString node as str if available/readable; otherwise 'N/A'.

    Args:
        nodemap (PySpin.INodeMap): Camera node map (requires cam.Init()).
        node_name (str): Node name to read (e.g., 'DeviceModelName').

    Returns:
        str: Node value or 'N/A' when not readable.
    """
    node = PySpin.CStringPtr(nodemap.GetNode(node_name))
    if PySpin.IsAvailable(node) and PySpin.IsReadable(node):
        return node.GetValue()
    return "N/A"


def _format_block(title: str, kv_pairs: Dict[str, str]) -> List[str]:
    """
    Build a pretty, width-aligned text block for printing.

    Args:
        title (str): Title to show at the top of the block.
        kv_pairs (Dict[str, str]): Mapping from label -> value.

    Returns:
        List[str]: Lines to print (without trailing newline).
    """
    # Prepare lines "label : value" with aligned label width
    label_width = max(len(label) for label in kv_pairs.keys()) if kv_pairs else 0
    body_lines = [f" ├─ {label:<{label_width}} : {value}" for label, value in kv_pairs.items()]

    # Compute box width based on the longest body line
    content_width = max([len(line) for line in body_lines] + [len(title) + 2])  # +2 for icon/space
    top = f"\n📷 {title}".ljust(content_width)
    bottom = " └" + "─" * (content_width - 2) + "┘"

    return [top, *body_lines, bottom]


def print_camera_info(cam: PySpin.CameraPtr, index: int) -> None:
    """
    Print a formatted info block for a single camera.

    Args:
        cam (PySpin.CameraPtr): Camera pointer (uninitialized).
        index (int): Camera index (for display only).

    Returns:
        None
    """
    cam.Init()
    try:
        nodemap = cam.GetNodeMap()

        info_nodes = {
            "Model Name":        "DeviceModelName",
            "Serial Number":     "DeviceSerialNumber",
            "Vendor Name":       "DeviceVendorName",
            "Firmware Ver.":     "DeviceFirmwareVersion",
            "Device Version":    "DeviceVersion",
        }
        kv = {label: get_node_value(nodemap, node) for label, node in info_nodes.items()}
        for line in _format_block(f"Camera [{index}]", kv):
            print(line)
    finally:
        # Ensure the camera is properly de-initialized
        try:
            cam.EndAcquisition()
        except Exception:
            pass
        try:
            cam.DeInit()
        except Exception:
            pass


def main() -> None:
    """
    Enumerate cameras and print basic information for each.

    Args:
        None

    Returns:
        None
    """
    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()

    try:
        num = cam_list.GetSize()
        if num == 0:
            print("❌ No cameras detected.")
            return

        print(f"✅ {num} camera(s) detected.")
        for i in range(num):
            cam = cam_list.GetByIndex(i)
            # Do not keep references around longer than needed
            print_camera_info(cam, i)
            del cam

    finally:
        # Proper release order: clear list -> release system
        try:
            cam_list.Clear()
        finally:
            del cam_list
            system.ReleaseInstance()
            del system


if __name__ == "__main__":
    main()
