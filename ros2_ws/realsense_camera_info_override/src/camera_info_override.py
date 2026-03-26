#!/usr/bin/env python3

from pathlib import Path
from urllib.parse import unquote, urlparse

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


def _resolve_camera_info_path(camera_info_url: str) -> Path:
    """Resolve a plain path or file:// URL to a local filesystem path."""
    if not camera_info_url:
        raise ValueError("camera_info_url must not be empty")

    if camera_info_url.startswith("file://"):
        parsed = urlparse(camera_info_url)
        if parsed.scheme != "file":
            raise ValueError(
                f"camera_info_url must use file:// when a URL is provided: {camera_info_url}"
            )
        return Path(unquote(parsed.path))

    return Path(camera_info_url)


def _matrix_data(data: dict, key: str, expected_len: int) -> list[float]:
    matrix = data.get(key, {})
    values = matrix.get("data", [])
    if len(values) != expected_len:
        raise ValueError(
            f"{key} must contain {expected_len} values, found {len(values)}"
        )
    return [float(value) for value in values]


def _vector_data(data: dict, key: str) -> list[float]:
    vector = data.get(key, {})
    values = vector.get("data", [])
    if not values:
        raise ValueError(f"{key} must contain at least one value")
    return [float(value) for value in values]


def load_camera_info(camera_info_url: str) -> CameraInfo:
    """Load a ROS camera_calibration YAML file into a CameraInfo message."""
    path = _resolve_camera_info_path(camera_info_url)
    if not path.is_file():
        raise FileNotFoundError(f"Calibration file does not exist: {path}")

    with path.open("r", encoding="utf-8") as handle:
        payload = yaml.safe_load(handle) or {}

    message = CameraInfo()
    message.width = int(payload["image_width"])
    message.height = int(payload["image_height"])
    message.distortion_model = str(payload["distortion_model"])
    message.d = _vector_data(payload, "distortion_coefficients")
    message.k = _matrix_data(payload, "camera_matrix", 9)
    message.r = _matrix_data(payload, "rectification_matrix", 9)
    message.p = _matrix_data(payload, "projection_matrix", 12)
    return message


class CameraInfoOverrideNode(Node):
    """Relay CameraInfo with calibration fields overridden from YAML."""

    def __init__(self) -> None:
        super().__init__("camera_info_override")

        self.input_topic = (
            self.declare_parameter("input_topic", "").get_parameter_value().string_value
        )
        self.output_topic = (
            self.declare_parameter("output_topic", "").get_parameter_value().string_value
        )
        self.camera_info_url = (
            self.declare_parameter("camera_info_url", "")
            .get_parameter_value()
            .string_value
        )

        if not self.input_topic or not self.output_topic:
            raise ValueError("input_topic and output_topic parameters are required")

        self.override_info = load_camera_info(self.camera_info_url)
        self.logged_first_message = False
        self.warned_size_mismatch = False

        self.subscription = self.create_subscription(
            CameraInfo,
            self.input_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.publisher = self.create_publisher(
            CameraInfo,
            self.output_topic,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Loaded CameraInfo override from %s and relaying %s -> %s"
            % (self.camera_info_url, self.input_topic, self.output_topic)
        )

    def _camera_info_callback(self, message: CameraInfo) -> None:
        output = CameraInfo()
        output.header = message.header
        output.width = self.override_info.width or message.width
        output.height = self.override_info.height or message.height
        output.distortion_model = (
            self.override_info.distortion_model or message.distortion_model
        )
        output.d = list(self.override_info.d) if self.override_info.d else list(message.d)
        output.k = list(self.override_info.k) if self.override_info.k else list(message.k)
        output.r = list(self.override_info.r) if self.override_info.r else list(message.r)
        output.p = list(self.override_info.p) if self.override_info.p else list(message.p)
        output.binning_x = message.binning_x
        output.binning_y = message.binning_y
        output.roi = message.roi

        if (
            not self.warned_size_mismatch
            and message.width
            and message.height
            and (
                message.width != output.width
                or message.height != output.height
            )
        ):
            self.warned_size_mismatch = True
            self.get_logger().warn(
                "OEM CameraInfo size %sx%s differs from override size %sx%s"
                % (message.width, message.height, output.width, output.height)
            )

        if not self.logged_first_message:
            self.logged_first_message = True
            self.get_logger().info(
                "Publishing calibrated CameraInfo with frame_id=%s"
                % output.header.frame_id
            )

        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraInfoOverrideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
