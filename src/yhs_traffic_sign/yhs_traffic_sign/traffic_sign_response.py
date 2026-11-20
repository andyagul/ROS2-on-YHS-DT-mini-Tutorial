"""交通标识牌识别与响应 - ROS2主节点

整合目标检测、距离感知、车辆控制、异常处理四个核心模块，
形成完整的交通标识牌识别与自动响应系统。

订阅话题：
    - /camera/color/image_raw     (sensor_msgs/Image)  彩色图像
    - /camera/depth/image_raw     (sensor_msgs/Image)  深度图像

发布话题：
    - /traffic_sign/detection_result  (自定义)  检测结果
    - /traffic_sign/distance_result   (自定义)  距离结果
    - /traffic_sign/status            (std_msgs/String)  状态诊断
    - /ctrl_cmd                       (yhs_can_interfaces/CtrlCmd)  底盘控制

性能要求：
    - 端到端延迟 < 100ms
    - 距离误差 ≤ ±0.05m
    - 指令执行准确率 ≥ 99%
"""

import sys
import time
import logging
from dataclasses import asdict
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from yhs_can_interfaces.msg import CtrlCmd

from .config import (
    DetectionConfig,
    DistanceConfig,
    ControlConfig,
    AnomalyConfig,
    TopicNames,
    SignType,
)
from .traffic_sign_detector import TrafficSignDetector, DetectionTracker
from .distance_estimator import DistanceEstimator
from .vehicle_controller import VehicleController, ControlCommand
from .anomaly_handler import AnomalyHandler, FilteredResult


# 日志初始化
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
)
logger = logging.getLogger('traffic_sign_response')


class TrafficSignResponseNode(Node):
    """交通标识牌识别与响应ROS2节点"""

    def __init__(self):
        super().__init__('traffic_sign_response')

        # ========================================================
        # 声明参数
        # ========================================================
        self.declare_parameter('model_path', DetectionConfig.MODEL_PATH)
        self.declare_parameter('confidence_threshold', DetectionConfig.CONFIDENCE_THRESHOLD)
        self.declare_parameter('device', DetectionConfig.DEVICE)

        self.declare_parameter('trigger_distance', ControlConfig.TRIGGER_DISTANCE)
        self.declare_parameter('confirm_count', ControlConfig.TRIGGER_CONFIRM_COUNT)
        self.declare_parameter('cooldown', ControlConfig.CONTROL_COOLDOWN)

        self.declare_parameter('camera_fx', DistanceConfig.CAMERA_FX)
        self.declare_parameter('camera_fy', DistanceConfig.CAMERA_FY)
        self.declare_parameter('sign_real_width', DistanceConfig.SIGN_REAL_WIDTH)
        self.declare_parameter('sign_real_height', DistanceConfig.SIGN_REAL_HEIGHT)

        self.declare_parameter('low_confidence_threshold', AnomalyConfig.LOW_CONFIDENCE_THRESHOLD)
        self.declare_parameter('distance_method', DistanceConfig.DISTANCE_METHOD)
        self.declare_parameter('enable_control', True)  # 是否启用实际控制（调试时可关闭）

        # ========================================================
        # 初始化核心模块
        # ========================================================
        self.detector = TrafficSignDetector(
            model_path=self.get_parameter('model_path').value,
            confidence_threshold=self.get_parameter('confidence_threshold').value,
            device=self.get_parameter('device').value,
        )
        self.detector.load_model()

        self.detection_tracker = DetectionTracker(history_size=5)

        self.distance_estimator = DistanceEstimator(
            method=self.get_parameter('distance_method').value,
            fx=self.get_parameter('camera_fx').value,
            fy=self.get_parameter('camera_fy').value,
            sign_real_width=self.get_parameter('sign_real_width').value,
            sign_real_height=self.get_parameter('sign_real_height').value,
        )

        self.vehicle_controller = VehicleController(
            trigger_distance=self.get_parameter('trigger_distance').value,
            confirm_count=self.get_parameter('confirm_count').value,
            cooldown=self.get_parameter('cooldown').value,
            on_command_callback=self._on_control_command,
        )

        self.anomaly_handler = AnomalyHandler(
            low_conf_threshold=self.get_parameter('low_confidence_threshold').value,
        )

        self._enable_control = self.get_parameter('enable_control').value

        # ========================================================
        # QoS配置
        # ========================================================
        qos_image = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=3,
        )
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ========================================================
        # 订阅话题
        # ========================================================
        self.color_image_sub = self.create_subscription(
            Image,
            TopicNames.COLOR_IMAGE,
            self._color_image_callback,
            qos_image,
        )
        self.depth_image_sub = self.create_subscription(
            Image,
            TopicNames.DEPTH_IMAGE,
            self._depth_image_callback,
            qos_image,
        )

        # ========================================================
        # 发布话题
        # ========================================================
        self.detection_result_pub = self.create_publisher(
            String, TopicNames.DETECTION_RESULT, qos_reliable
        )
        self.distance_result_pub = self.create_publisher(
            String, TopicNames.DISTANCE_RESULT, qos_reliable
        )
        self.status_pub = self.create_publisher(
            String, TopicNames.DIAGNOSTIC_STATUS, qos_reliable
        )
        self.ctrl_cmd_pub = self.create_publisher(
            CtrlCmd, TopicNames.CONTROL_CMD, qos_reliable
        )

        # ========================================================
        # 内部状态
        # ========================================================
        self._color_image: Optional[Image] = None
        self._depth_image: Optional[Image] = None
        self._last_process_time: float = 0.0
        self._frame_count: int = 0

        # 帧定时器 (30Hz)
        self.process_timer = self.create_timer(
            0.033, self._process_frame
        )

        # 状态诊断定时器 (1Hz)
        self.status_timer = self.create_timer(
            1.0, self._publish_status
        )

        # 日志
        self.get_logger().info("交通标识牌识别与响应节点启动")
        self.get_logger().info(f"  触发距离: {self.get_parameter('trigger_distance').value}m")
        self.get_logger().info(f"  控制启用: {self._enable_control}")
        self.get_logger().info(f"  测距方法: {self.get_parameter('distance_method').value}")

    # ================================================================
    # 话题回调
    # ================================================================
    def _color_image_callback(self, msg: Image):
        """彩色图像回调"""
        self._color_image = msg

    def _depth_image_callback(self, msg: Image):
        """深度图像回调"""
        self._depth_image = msg

    # ================================================================
    # 帧处理主流程
    # ================================================================
    def _process_frame(self):
        """处理一帧图像：检测 -> 测距 -> 异常处理 -> 控制触发"""
        if self._color_image is None:
            return

        self._frame_count += 1
        frame_start = time.time()

        try:
            # Step 1: 图像转换 (sensor_msgs/Image -> numpy array)
            color_img = self._decode_image(self._color_image)
            if color_img is None:
                return

            # Step 2: 目标检测
            raw_detections = self.detector.detect(color_img)
            tracked_detection = self.detection_tracker.update(raw_detections)

            # 发布检测结果
            self._publish_detection_result(raw_detections, tracked_detection)

            # Step 3: 距离测量
            depth_img = None
            if self._depth_image is not None:
                depth_img = self._decode_depth(self._depth_image)

            distance_result = None
            if tracked_detection is not None:
                img_h, img_w = color_img.shape[:2]
                bbox_xyxy = tracked_detection.get_bbox_xyxy(img_w, img_h)
                bbox_pixel = tracked_detection.get_bbox_pixel(img_w, img_h)

                distance_result = self.distance_estimator.estimate(
                    bbox_xyxy=bbox_xyxy,
                    image_shape=(img_h, img_w),
                    depth_image=depth_img,
                    sign_bbox_pixel=bbox_pixel,
                )

                # 发布距离结果
                self._publish_distance_result(distance_result)

            # Step 4: 异常处理与过滤
            filtered = self.anomaly_handler.process(
                detections=raw_detections,
                distance=distance_result.distance if distance_result else 0.0,
                distance_valid=distance_result.distance_valid if distance_result else False,
                distance_confidence=distance_result.confidence if distance_result else 0.0,
            )

            # Step 5: 控制触发
            command = self.vehicle_controller.process(
                sign_type=filtered.sign_type,
                distance=filtered.distance,
                distance_valid=filtered.distance_valid,
            )

            frame_time = (time.time() - frame_start) * 1000  # ms
            self._last_process_time = frame_time

            # 性能告警
            if frame_time > 100:
                self.get_logger().warn(
                    f"处理帧耗时 {frame_time:.1f}ms 超过100ms阈值"
                )

            if self._frame_count % 100 == 0:
                self.get_logger().info(
                    f"处理 {self._frame_count} 帧, 平均耗时: {frame_time:.1f}ms, "
                    f"检测状态: {'有效' if filtered.is_valid else '无效'}, "
                    f"距离: {filtered.distance:.3f}m"
                )

        except Exception as e:
            self.get_logger().error(f"帧处理异常: {e}")

    # ================================================================
    # 图像解码
    # ================================================================
    def _decode_image(self, msg: Image) -> Optional[np.ndarray]:
        """将sensor_msgs/Image转换为numpy数组 (BGR)"""
        try:
            import cv2
            # 优先使用cv_bridge
            try:
                from cv_bridge import CvBridge
                bridge = CvBridge()
                return bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except ImportError:
                # 手动解码
                img = np.frombuffer(msg.data, dtype=np.uint8)
                h, w = msg.height, msg.width

                if msg.encoding == 'bgr8':
                    return img.reshape(h, w, 3)
                elif msg.encoding == 'rgb8':
                    img = img.reshape(h, w, 3)
                    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                elif msg.encoding == 'mono8':
                    gray = img.reshape(h, w)
                    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                else:
                    # 尝试通用解码
                    img = img.reshape(h, w, -1) if img.size == h * w * 3 else img.reshape(h, w)
                    return img if len(img.shape) == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        except Exception as e:
            self.get_logger().error(f"图像解码失败: {e}")
            return None

    def _decode_depth(self, msg: Image) -> Optional[np.ndarray]:
        """将深度图像转换为numpy数组"""
        try:
            try:
                from cv_bridge import CvBridge
                bridge = CvBridge()
                if msg.encoding in ['16UC1', 'mono16']:
                    return bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                elif msg.encoding in ['32FC1']:
                    return bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            except ImportError:
                h, w = msg.height, msg.width
                if msg.encoding in ['16UC1', 'mono16']:
                    return np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
                elif msg.encoding == '32FC1':
                    return np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
        except Exception as e:
            self.get_logger().error(f"深度图像解码失败: {e}")
            return None

    # ================================================================
    # 结果发布
    # ================================================================
    def _publish_detection_result(self, raw_detections, tracked):
        """发布检测结果"""
        msg = String()
        if tracked is not None:
            msg.data = (
                f"{{\"sign_type\":\"{tracked.sign_type}\","
                f"\"cn_name\":\"{SignType.CN_NAMES.get(tracked.sign_type, '')}\","
                f"\"confidence\":{tracked.confidence:.3f},"
                f"\"inference_ms\":{tracked.inference_time:.1f},"
                f"\"bbox\":{list(tracked.bbox)},"
                f"\"raw_count\":{len(raw_detections)}}}"
            )
        else:
            msg.data = f"{{\"sign_type\":\"none\",\"raw_count\":{len(raw_detections)}}}"
        self.detection_result_pub.publish(msg)

    def _publish_distance_result(self, distance_result):
        """发布距离结果"""
        msg = String()
        msg.data = (
            f"{{\"distance\":{distance_result.distance:.3f},"
            f"\"valid\":{distance_result.distance_valid},"
            f"\"method\":\"{distance_result.method}\","
            f"\"confidence\":{distance_result.confidence:.3f},"
            f"\"error\":{distance_result.measurement_error:.4f}}}"
        )
        self.distance_result_pub.publish(msg)

    def _publish_status(self):
        """发布状态诊断信息 (1Hz)"""
        try:
            msg = String()
            stats = {
                'frame_count': self._frame_count,
                'last_process_ms': self._last_process_time,
                'detector': self.detector.get_stats(),
                'controller': self.vehicle_controller.get_stats(),
                'distance': self.distance_estimator.get_stats(),
                'anomaly': self.anomaly_handler.get_anomaly_stats(),
            }
            import json
            msg.data = json.dumps(stats, ensure_ascii=False)
            self.status_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"状态发布异常: {e}")

    # ================================================================
    # 控制指令回调
    # ================================================================
    def _on_control_command(self, command: ControlCommand):
        """车辆控制指令回调"""
        if not self._enable_control:
            self.get_logger().info(
                f"[调试模式] 指令已生成但未发送: {command.to_dict()}"
            )
            return

        try:
            ctrl_msg = CtrlCmd()
            ctrl_msg.ctrl_cmd_gear = command.gear
            ctrl_msg.ctrl_cmd_linear = command.linear_velocity
            ctrl_msg.ctrl_cmd_angular = command.angular_velocity

            self.ctrl_cmd_pub.publish(ctrl_msg)
            self.get_logger().info(
                f"控制指令已发送: linear={command.linear_velocity:.3f}, "
                f"angular={command.angular_velocity:.3f}, gear={command.gear}"
            )
        except Exception as e:
            self.get_logger().error(f"指令发送异常: {e}")

    # ================================================================
    # 生命周期管理
    # ================================================================
    def destroy_node(self):
        """节点销毁时清理"""
        self.vehicle_controller.reset()
        self.anomaly_handler.reset()
        self.distance_estimator.reset()
        self.get_logger().info("交通标识牌识别与响应节点已销毁")
        super().destroy_node()


def main(args=None):
    """入口函数"""
    rclpy.init(args=args)

    node = TrafficSignResponseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号，正在关闭...")
    except Exception as e:
        node.get_logger().error(f"节点运行异常: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
