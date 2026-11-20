"""集成测试

端到端测试完整流程：
    检测 -> 跟踪 -> 测距 -> 异常处理 -> 控制触发
覆盖不同场景下的系统稳定性。
"""

import time
import pytest
import numpy as np

from yhs_traffic_sign.traffic_sign_detector import (
    TrafficSignDetector,
    DetectionTracker,
    DetectionResult,
)
from yhs_traffic_sign.distance_estimator import DistanceEstimator
from yhs_traffic_sign.vehicle_controller import (
    VehicleController,
    ControlState,
    ControlAction,
)
from yhs_traffic_sign.anomaly_handler import (
    AnomalyHandler,
    AnomalyType,
)
from yhs_traffic_sign.config import SignType, ControlConfig


def make_det(sign_type=SignType.STRAIGHT, confidence=0.9):
    """创建模拟检测结果"""
    class_id = {SignType.LEFT_TURN: 0, SignType.STRAIGHT: 1, SignType.RIGHT_TURN: 2}[sign_type]
    return DetectionResult(
        bbox=(0.5, 0.6, 0.12, 0.15),
        confidence=confidence,
        class_id=class_id,
        sign_type=sign_type,
    )


class TestEndToEndStraight:
    """端到端：直行标识牌响应场景"""

    def test_full_straight_response(self):
        """完整直行响应流程"""
        detector = TrafficSignDetector(model_path='/nonexistent.pt')
        detector.load_model()
        tracker = DetectionTracker()
        estimator = DistanceEstimator()
        handler = AnomalyHandler()

        commands = []
        controller = VehicleController(
            trigger_distance=0.5,
            confirm_count=3,
            cooldown=10.0,
            on_command_callback=commands.append,
        )

        # 模拟图像和深度图
        image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        depth = np.full((480, 640), 400, dtype=np.uint16)  # 0.4米，接近触发阈值

        for _ in range(10):  # 模拟10帧
            # 1. 检测
            detections = detector.detect(image)
            tracked = tracker.update(detections)

            # 2. 测距
            distance = 0.4
            if tracked:
                img_h, img_w = 480, 640
                bbox_xyxy = tracked.get_bbox_xyxy(img_w, img_h)
                dist_result = estimator.estimate(
                    bbox_xyxy=bbox_xyxy,
                    image_shape=(img_h, img_w),
                    depth_image=depth,
                )
                distance = dist_result.distance
                dist_valid = dist_result.distance_valid
            else:
                dist_valid = False

            # 3. 异常处理
            filtered = handler.process(detections, distance, dist_valid)

            # 4. 控制
            controller.process(
                filtered.sign_type if filtered.is_valid else SignType.UNKNOWN,
                filtered.distance,
                filtered.distance_valid,
            )

        # 应该触发了指令
        assert len(commands) >= 1
        assert commands[0].source_sign_type in [SignType.STRAIGHT, SignType.UNKNOWN]


class TestEndToEndLeftTurn:
    """端到端：左转标识牌响应场景"""

    def test_full_left_turn_response(self):
        """完整左转响应流程"""
        tracker = DetectionTracker()
        estimator = DistanceEstimator()
        handler = AnomalyHandler()

        commands = []
        controller = VehicleController(
            trigger_distance=0.5,
            confirm_count=3,
            cooldown=10.0,
            on_command_callback=commands.append,
        )

        # 模拟左转检测结果
        for _ in range(10):
            detections = [make_det(SignType.LEFT_TURN, 0.92)]
            tracker.update(detections)

            # 模拟有效距离
            distance = 0.35
            dist_valid = True

            filtered = handler.process(detections, distance, dist_valid)
            controller.process(
                filtered.sign_type if filtered.is_valid else SignType.UNKNOWN,
                filtered.distance,
                filtered.distance_valid,
            )

        assert len(commands) >= 1
        assert commands[0].action == ControlAction.TURN_LEFT
        assert commands[0].angular_velocity > 0  # 左转角速度为正
        assert commands[0].linear_velocity < ControlConfig.STRAIGHT_SPEED  # 转弯减速


class TestEndToEndRightTurn:
    """端到端：右转标识牌响应场景"""

    def test_full_right_turn_response(self):
        """完整右转响应流程"""
        handler = AnomalyHandler()

        commands = []
        controller = VehicleController(
            trigger_distance=0.5,
            confirm_count=3,
            cooldown=10.0,
            on_command_callback=commands.append,
        )

        for _ in range(10):
            detections = [make_det(SignType.RIGHT_TURN, 0.88)]
            distance = 0.42

            filtered = handler.process(detections, distance, True)
            controller.process(
                filtered.sign_type if filtered.is_valid else SignType.UNKNOWN,
                filtered.distance,
                filtered.distance_valid,
            )

        assert len(commands) >= 1
        assert commands[0].action == ControlAction.TURN_RIGHT
        assert commands[0].angular_velocity < 0  # 右转角速度为负


class TestEndToEndNoTriggerScenario:
    """端到端：不应触发的场景"""

    def test_too_far_no_trigger(self):
        """距离太远不应触发"""
        handler = AnomalyHandler()
        commands = []
        controller = VehicleController(
            trigger_distance=0.5,
            confirm_count=3,
            on_command_callback=commands.append,
        )

        # 距离10米远
        for _ in range(20):
            detections = [make_det(SignType.STRAIGHT, 0.9)]
            distance = 10.0
            filtered = handler.process(detections, distance, True)
            controller.process(
                filtered.sign_type if filtered.is_valid else SignType.UNKNOWN,
                filtered.distance,
                filtered.distance_valid,
            )

        assert len(commands) == 0

    def test_low_confidence_no_trigger(self):
        """低置信度不应触发"""
        handler = AnomalyHandler(low_conf_threshold=0.7, low_conf_tolerance=3)
        commands = []
        controller = VehicleController(
            trigger_distance=0.5,
            confirm_count=3,
            on_command_callback=commands.append,
        )

        # 置信度低
        for _ in range(10):
            detections = [make_det(SignType.STRAIGHT, 0.3)]
            distance = 0.3
            filtered = handler.process(detections, distance, True)
            controller.process(
                filtered.sign_type if filtered.is_valid else SignType.UNKNOWN,
                filtered.distance,
                filtered.distance_valid,
            )

        # 不应触发
        assert len(commands) == 0

    def test_insufficient_confirmations(self):
        """确认次数不足不应触发"""
        handler = AnomalyHandler()
        commands = []
        controller = VehicleController(
            trigger_distance=0.5,
            confirm_count=5,  # 需要5次确认
            on_command_callback=commands.append,
        )

        # 只连续检测3次
        for _ in range(3):
            detections = [make_det(SignType.STRAIGHT, 0.9)]
            distance = 0.3
            filtered = handler.process(detections, distance, True)
            controller.process(
                filtered.sign_type if filtered.is_valid else SignType.UNKNOWN,
                filtered.distance,
                filtered.distance_valid,
            )

        assert len(commands) == 0

    def test_lost_before_trigger(self):
        """触发前标识牌丢失不应触发"""
        handler = AnomalyHandler(max_lost_frames=3)
        commands = []
        controller = VehicleController(
            trigger_distance=0.5,
            confirm_count=3,
            on_command_callback=commands.append,
        )

        # 先检测2次
        for _ in range(2):
            detections = [make_det(SignType.STRAIGHT, 0.9)]
            distance = 0.3
            filtered = handler.process(detections, distance, True)
            controller.process(
                filtered.sign_type if filtered.is_valid else SignType.UNKNOWN,
                filtered.distance,
                filtered.distance_valid,
            )

        # 然后丢失
        for _ in range(5):
            filtered = handler.process([], 0.0, False)
            controller.process(
                filtered.sign_type if filtered.is_valid else SignType.UNKNOWN,
                filtered.distance,
                filtered.distance_valid,
            )

        # 丢失后确认计数应该重置
        assert len(commands) == 0


class TestEndToEndRecovery:
    """端到端：异常恢复场景"""

    def test_recovery_after_distance_jump(self):
        """距离跳变后的恢复"""
        estimator = DistanceEstimator(jump_threshold=0.3)
        handler = AnomalyHandler()
        commands = []
        controller = VehicleController(
            trigger_distance=0.5,
            confirm_count=10,  # 需要多帧确认
            on_command_callback=commands.append,
        )

        # 正常接近
        distance = 3.0
        for _ in range(5):
            distance = max(0.6, distance - 0.5)  # 逐渐接近
            filtered = handler.process(
                [make_det(SignType.STRAIGHT, 0.9)], distance, True
            )
            controller.process(
                filtered.sign_type if filtered.is_valid else SignType.UNKNOWN,
                filtered.distance,
                filtered.distance_valid,
            )

        # 突然跳变
        for _ in range(2):
            distance = 10.0  # 跳变到远距离
            filtered = handler.process(
                [make_det(SignType.STRAIGHT, 0.9)], distance, True
            )
            controller.process(
                filtered.sign_type if filtered.is_valid else SignType.UNKNOWN,
                filtered.distance,
                filtered.distance_valid,
            )

        # 恢复正常
        distance = 0.4
        for _ in range(10):
            filtered = handler.process(
                [make_det(SignType.STRAIGHT, 0.9)], distance, True
            )
            controller.process(
                filtered.sign_type if filtered.is_valid else SignType.UNKNOWN,
                filtered.distance,
                filtered.distance_valid,
            )

        # 最终应该触发
        assert len(commands) >= 1

    def test_recovery_after_sign_lost(self):
        """标识牌丢失后的恢复"""
        handler = AnomalyHandler(max_lost_frames=5)
        commands = []
        controller = VehicleController(
            trigger_distance=0.5,
            confirm_count=10,
            on_command_callback=commands.append,
        )

        # 正常检测
        for _ in range(8):
            filtered = handler.process(
                [make_det(SignType.LEFT_TURN, 0.9)], 0.3, True
            )
            controller.process(
                filtered.sign_type if filtered.is_valid else SignType.UNKNOWN,
                filtered.distance,
                filtered.distance_valid,
            )

        # 丢失3帧
        for _ in range(3):
            filtered = handler.process([], 0.0, False)
            controller.process(
                filtered.sign_type if filtered.is_valid else SignType.UNKNOWN,
                filtered.distance,
                filtered.distance_valid,
            )

        # 重新出现
        for _ in range(10):
            filtered = handler.process(
                [make_det(SignType.LEFT_TURN, 0.9)], 0.3, True
            )
            controller.process(
                filtered.sign_type if filtered.is_valid else SignType.UNKNOWN,
                filtered.distance,
                filtered.distance_valid,
            )

        assert len(commands) >= 1
        assert commands[0].action == ControlAction.TURN_LEFT


class TestPerformanceMetrics:
    """性能指标测试"""

    def test_inference_latency_target(self):
        """检测推理延迟应远低于100ms"""
        detector = TrafficSignDetector(model_path='/nonexistent.pt')
        detector.load_model()

        image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        times = []
        for _ in range(50):
            start = time.time()
            detector.detect(image)
            elapsed = (time.time() - start) * 1000
            times.append(elapsed)

        avg = np.mean(times)
        p95 = np.percentile(times, 95)

        print(f"\n推理延迟统计 (模拟模式):")
        print(f"  平均: {avg:.2f}ms")
        print(f"  P95:  {p95:.2f}ms")
        print(f"  最大: {max(times):.2f}ms")

        assert avg < 50, f"平均推理延迟 {avg:.1f}ms 超过50ms"
        assert p95 < 80, f"P95推理延迟 {p95:.1f}ms 超过80ms"

    def test_end_to_end_pipeline_speed(self):
        """端到端管线处理速度"""
        detector = TrafficSignDetector(model_path='/nonexistent.pt')
        detector.load_model()
        tracker = DetectionTracker()
        estimator = DistanceEstimator()
        handler = AnomalyHandler()

        image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        depth = np.full((480, 640), 2000, dtype=np.uint16)

        times = []
        for _ in range(50):
            start = time.time()

            detections = detector.detect(image)
            tracked = tracker.update(detections)

            if tracked:
                dist_result = estimator.estimate(
                    bbox_xyxy=tracked.get_bbox_xyxy(640, 480),
                    image_shape=(480, 640),
                    depth_image=depth,
                )
            else:
                dist_result = None

            handler.process(
                detections,
                dist_result.distance if dist_result else 0.0,
                dist_result.distance_valid if dist_result else False,
            )

            elapsed = (time.time() - start) * 1000
            times.append(elapsed)

        avg = np.mean(times)
        p95 = np.percentile(times, 95)

        print(f"\n端到端管线延迟统计:")
        print(f"  平均: {avg:.2f}ms")
        print(f"  P95:  {p95:.2f}ms")
        print(f"  最大: {max(times):.2f}ms")

        assert avg < 100, f"平均处理延迟 {avg:.1f}ms 超过100ms"

    def test_distance_estimation_precision(self):
        """距离测量精度测试 (±0.05米)"""
        estimator = DistanceEstimator(
            avg_window=1,  # 不使用平均滤波便于观察原始精度
            jump_threshold=100.0,  # 禁用跳变检测
        )

        true_distance = 2.0  # 真实距离2米
        depth_value = int(true_distance * 1000)  # 毫米

        errors = []
        for _ in range(100):
            # 加入轻微噪声
            noise = np.random.randint(-20, 21)  # ±20mm噪声
            depth = np.full((480, 640), depth_value + noise, dtype=np.uint16)
            result = estimator.estimate_from_depth(depth, (280, 180, 360, 240))
            errors.append(abs(result.distance - true_distance))

        avg_error = np.mean(errors)
        max_error = np.max(errors)
        std_error = np.std(errors)

        print(f"\n距离测量精度统计:")
        print(f"  真实值: {true_distance}m")
        print(f"  平均误差: {avg_error:.4f}m")
        print(f"  最大误差: {max_error:.4f}m")
        print(f"  误差标准差: {std_error:.4f}m")

        assert avg_error < 0.05, f"平均测量误差 {avg_error:.4f}m 超过±0.05m"
        assert std_error < 0.05, f"测量误差标准差 {std_error:.4f}m 超过±0.05m"
