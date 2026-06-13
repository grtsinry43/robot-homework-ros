"""HSV + depth perception → /scene_state (DESIGN.md §5.2)."""

from __future__ import annotations

import math
from typing import Any

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from pick_place_msgs.srv import TriggerScan
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scene_state_msgs.msg import ObjectPose, SceneState
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

import tf2_geometry_msgs  # noqa: F401 — registers PointStamped with tf2 buffer

import cv2
import numpy as np


# Tuned for Gazebo ogre diffuse materials (wide S/V; red uses dual hue wrap).
LABEL_TO_HSV: dict[str, tuple] = {
    "red_block": ([0, 40, 25], [30, 255, 255], [150, 40, 25], [180, 255, 255]),
    "green_block": ([35, 25, 25], [95, 255, 255]),
    "blue_plate": ([85, 30, 25], [145, 255, 255]),
}

# Track-association radius. HSV centroids under ogre's grayscale rendering jitter several
# cm frame-to-frame, and the sim-layout fallback fills a slightly different xy than HSV — at
# 0.08 m the same plate got split into two tracks (blue_plate_01 from fallback + _02 from
# HSV). 0.12 m absorbs that jitter so each physical object keeps ONE stable id. Same-label
# is also required to match, and the props are >0.18 m apart, so cross-object mismatch can't
# happen.
MATCH_DISTANCE_M = 0.12

# pick_place_desk.sdf poses in panda_link0 (static TF bridges world layout).
SIM_LAYOUT_POSES: dict[str, tuple[float, float, float]] = {
    "red_block": (0.35, 0.05, 0.06),
    "green_block": (0.42, -0.12, 0.06),
    "blue_plate": (0.42, 0.20, 0.055),
}

# The overhead depth back-projection measures the object's TOP face, while the rest of the
# pipeline (offset_resolver, grasp) expects the reported z to be the object CENTER. So we
# drop the detected z by (half-height + a small constant for the depth over-read measured on
# this sim/camera, ~1.5 cm). Tuned against ground truth: a 4 cm block top reads ~0.092 and
# must report 0.060 (Δ 0.032); the thin plate top reads ~0.081 and must report 0.055
# (Δ 0.026). Per-label so blocks and the flat plate get the right correction.
TOP_TO_CENTER_Z: dict[str, float] = {
    "red_block": 0.032,
    "green_block": 0.032,
    "blue_plate": 0.026,
}
DEFAULT_TOP_TO_CENTER_Z = 0.03


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_node")

        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("camera_frame", "camera_optical_frame")
        self.declare_parameter("use_camera_info_frame", True)
        self.declare_parameter("depth_is_range", True)
        self.declare_parameter("override_camera_intrinsics", True)
        self.declare_parameter("camera_fx", 554.38)
        self.declare_parameter("camera_fy", 554.38)
        self.declare_parameter("camera_cx", 320.0)
        self.declare_parameter("camera_cy", 240.0)
        self.declare_parameter("base_frame", "panda_link0")
        self.declare_parameter("stale_after_sec", 10.0)
        self.declare_parameter("min_contour_area", 200.0)
        self.declare_parameter("sim_layout_fallback", False)

        self._bridge = CvBridge()
        self._latest_color: Image | None = None
        self._latest_depth: Image | None = None
        self._camera_info: CameraInfo | None = None
        self._objects: dict[str, ObjectPose] = {}
        self._label_seq: dict[str, int] = {}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._scene_pub = self.create_publisher(SceneState, "/scene_state", 10)
        self.create_subscription(Image, "/camera/color/image_raw", self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self._on_camera_info, 10)
        self.create_service(TriggerScan, "/perception/trigger_scan", self._handle_trigger_scan)

        hz = self.get_parameter("publish_hz").value
        self.create_timer(1.0 / hz, self._publish_scene_state)
        if self.get_parameter("sim_layout_fallback").value:
            self.get_logger().info("perception_node ready (HSV + depth + TF + sim layout fallback)")
        else:
            self.get_logger().info("perception_node ready (HSV + depth + TF)")

    def _on_color(self, msg: Image) -> None:
        self._latest_color = msg

    def _on_depth(self, msg: Image) -> None:
        self._latest_depth = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _handle_trigger_scan(self, _request: TriggerScan.Request, response: TriggerScan.Response) -> TriggerScan.Response:
        count = self._run_detection()
        response.success = count >= 0
        response.message = f"tracked {len(self._objects)} objects"
        return response

    def _publish_scene_state(self) -> None:
        if self._latest_color is not None:
            self._run_detection()

        msg = SceneState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.objects = list(self._objects.values())
        self._scene_pub.publish(msg)

    def _run_detection(self) -> int:
        if self._latest_color is None:
            return -1

        try:
            bgr = self._bridge.imgmsg_to_cv2(self._latest_color, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"cv_bridge color failed: {exc}")
            return -1

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        now = self.get_clock().now()
        min_area = float(self.get_parameter("min_contour_area").value)
        detections: list[tuple[str, float, float, float, float]] = []

        for label, spec in LABEL_TO_HSV.items():
            if len(spec) == 4:
                la, ua, lb, ub = spec
                mask = cv2.bitwise_or(
                    cv2.inRange(hsv, np.array(la), np.array(ua)),
                    cv2.inRange(hsv, np.array(lb), np.array(ub)),
                )
            else:
                lo, hi = spec
                mask = cv2.inRange(hsv, np.array(lo), np.array(hi))

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best_contour = None
            best_area = min_area
            for contour in contours:
                area = cv2.contourArea(contour)
                if area >= best_area:
                    best_area = area
                    best_contour = contour
            if best_contour is None:
                continue
            moments = cv2.moments(best_contour)
            if moments["m00"] == 0:
                continue
            u = int(moments["m10"] / moments["m00"])
            v = int(moments["m01"] / moments["m00"])
            depth_m = self._lookup_depth_m(u, v)
            if depth_m is None or not math.isfinite(depth_m):
                continue
            xyz_cam = self._backproject_camera(u, v, depth_m)
            if xyz_cam is None:
                continue
            xyz = self._transform_to_base(xyz_cam)
            if xyz is None:
                continue
            confidence = min(0.99, 0.6 + best_area / 5000.0)
            z_center = xyz[2] - TOP_TO_CENTER_Z.get(label, DEFAULT_TOP_TO_CENTER_Z)
            detections.append((label, xyz[0], xyz[1], z_center, confidence))

        if self.get_parameter("sim_layout_fallback").value:
            detections = self._merge_layout_fallback(detections)

        self._update_tracks(detections, now)
        return len(detections)

    def _merge_layout_fallback(
        self, detections: list[tuple[str, float, float, float, float]]
    ) -> list[tuple[str, float, float, float, float]]:
        """Fill missing labels when Gazebo ogre renders blocks nearly grayscale, and snap
        the z of detected objects to the known layout height.

        The overhead depth back-projection over-reads the surface z (e.g. the thin plate
        came back ~5 cm high), which inflated the place release height. The objects rest on
        a flat table at known heights, so we trust HSV for xy but override z from the layout
        prior — keeps the visual tracking while making the place descent land correctly."""
        layout = SIM_LAYOUT_POSES
        merged: list[tuple[str, float, float, float, float]] = []
        found = {label for label, *_ in detections}
        for label, x, y, z, conf in detections:
            z_fixed = layout[label][2] if label in layout else z
            merged.append((label, x, y, z_fixed, conf))
        for label, (x, y, z) in layout.items():
            if label in found:
                continue
            merged.append((label, x, y, z, 0.72))
        return merged

    def _update_tracks(self, detections: list[tuple[str, float, float, float, float]], now: rclpy.time.Time) -> None:
        matched_ids: set[str] = set()
        base_frame = self.get_parameter("base_frame").value

        for label, x, y, z, confidence in detections:
            obj_id = self._match_or_create_id(label, x, y, z, matched_ids)
            matched_ids.add(obj_id)

            obj = self._objects.get(obj_id) or ObjectPose()
            obj.id = obj_id
            obj.label = label
            obj.confidence = float(confidence)
            obj.last_seen_at = now.to_msg()
            obj.pose = PoseStamped()
            obj.pose.header.frame_id = base_frame
            obj.pose.header.stamp = now.to_msg()
            obj.pose.pose = Pose()
            obj.pose.pose.position.x = x
            obj.pose.pose.position.y = y
            obj.pose.pose.position.z = z
            obj.pose.pose.orientation.w = 1.0
            self._objects[obj_id] = obj

        stale_ns = int(self.get_parameter("stale_after_sec").value * 1e9)
        expired = [
            oid for oid, obj in self._objects.items()
            if oid not in matched_ids
            and (now - rclpy.time.Time.from_msg(obj.last_seen_at)).nanoseconds > stale_ns
        ]
        for oid in expired:
            del self._objects[oid]

    def _match_or_create_id(self, label: str, x: float, y: float, z: float, taken: set[str]) -> str:
        best_id = None
        best_dist = MATCH_DISTANCE_M
        for oid, obj in self._objects.items():
            if obj.label != label or oid in taken:
                continue
            dx = obj.pose.pose.position.x - x
            dy = obj.pose.pose.position.y - y
            dz = obj.pose.pose.position.z - z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < best_dist:
                best_dist = dist
                best_id = oid
        if best_id:
            return best_id

        self._label_seq[label] = self._label_seq.get(label, 0) + 1
        return f"{label}_{self._label_seq[label]:02d}"

    def _lookup_depth_m(self, u: int, v: int) -> float | None:
        if self._latest_depth is None:
            return None
        try:
            depth_img = self._bridge.imgmsg_to_cv2(self._latest_depth)
        except Exception:  # noqa: BLE001
            return None

        h, w = depth_img.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return None

        raw = float(depth_img[v, u])
        encoding = self._latest_depth.encoding
        if encoding == "32FC1":
            return raw if raw > 0.0 else None
        if encoding in ("16UC1", "mono16"):
            return raw / 1000.0 if raw > 0 else None
        return raw if raw > 0.0 else None

    def _intrinsics(self) -> tuple[float, float, float, float] | None:
        if self._camera_info is None:
            return None
        if self.get_parameter("override_camera_intrinsics").value:
            return (
                float(self.get_parameter("camera_fx").value),
                float(self.get_parameter("camera_fy").value),
                float(self.get_parameter("camera_cx").value),
                float(self.get_parameter("camera_cy").value),
            )
        return (
            self._camera_info.k[0],
            self._camera_info.k[4],
            self._camera_info.k[2],
            self._camera_info.k[5],
        )

    def _backproject_camera(self, u: int, v: int, depth_m: float) -> tuple[float, float, float] | None:
        intr = self._intrinsics()
        if intr is None:
            return None
        fx, fy, cx, cy = intr
        ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=np.float64)
        if self.get_parameter("depth_is_range").value:
            ray /= np.linalg.norm(ray)
            return (float(ray[0] * depth_m), float(ray[1] * depth_m), float(ray[2] * depth_m))
        z = depth_m
        return ((u - cx) * depth_m / fx, (v - cy) * depth_m / fy, z)

    def _active_camera_frame(self) -> str:
        if self.get_parameter("use_camera_info_frame").value and self._camera_info is not None:
            frame = (self._camera_info.header.frame_id or "").strip()
            if frame:
                return frame
        return str(self.get_parameter("camera_frame").value)

    def _transform_to_base(self, xyz: tuple[float, float, float]) -> tuple[float, float, float] | None:
        camera_frame = self._active_camera_frame()
        base_frame = self.get_parameter("base_frame").value

        pt = PointStamped()
        pt.header.frame_id = camera_frame
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x, pt.point.y, pt.point.z = xyz

        try:
            out = self._tf_buffer.transform(
                pt, base_frame, timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except TransformException as exc:
            self.get_logger().warning(f"TF {camera_frame}->{base_frame}: {exc}")
            return None
        return (out.point.x, out.point.y, out.point.z)


def main() -> None:
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
