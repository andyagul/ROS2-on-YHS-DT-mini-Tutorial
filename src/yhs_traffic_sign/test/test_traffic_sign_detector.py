"""目标检测模块单元测试

覆盖场景:
- 检测器初始化与模型加载
- 模拟模式检测 (无模型文件时)
- BBox坐标转换
- 检测跟踪器状态维护
"""

import pytest
import numpy as np

from yhs_traffic_sign.traffic_sign_detector import (
    TrafficSignDetector,
    DetectionTracker,
    DetectionResult,
)
from yhs_traffic_sign.config import SignType


class TestTrafficSignDetector:
    """目标检测器测试"""

    def test_init(self):
        """测试初始化"""
        detector = TrafficSignDetector()
        assert detector.confidence_threshold == 0.5
        assert detector.iou_threshold == 0.45
        assert detector.device == 'cpu'

    def test_load_model_mock_mode(self):
        """测试模型加载 - 模拟模式(模型文件不存在)"""
        detector = TrafficSignDetector(model_path='/nonexistent/path.pt')
        result = detector.load_model()
        assert result is True
        assert detector.is_ready is True
        # 模拟模式下_model为None
        assert detector._model is None

    def test_detect_with_mock_model(self):
        """测试模拟模式下的检测"""
        detector = TrafficSignDetector(model_path='/nonexistent/path.pt')
        detector.load_model()

        # 创建模拟图像
        image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        results = detector.detect(image)

        assert len(results) == 1
        assert isinstance(results[0], DetectionResult)
        assert results[0].sign_type in SignType.CLASS_IDS.values() or results[0].sign_type == SignType.UNKNOWN
        assert 0.0 <= results[0].confidence <= 1.0

    def test_detect_empty_image(self):
        """测试空图像输入"""
        detector = TrafficSignDetector()
        detector.load_model()
        results = detector.detect(None)
        assert results == []

    def test_detect_empty_array(self):
        """测试空数组输入"""
        detector = TrafficSignDetector()
        detector.load_model()
        results = detector.detect(np.array([]))
        assert results == []

    def test_bbox_conversion_xyxy(self):
        """测试bbox归一化坐标转像素坐标"""
        result = DetectionResult(
            bbox=(0.5, 0.7, 0.15, 0.20),
            confidence=0.9,
            class_id=0,
            sign_type=SignType.LEFT_TURN,
        )
        img_w, img_h = 640, 480
        x1, y1, x2, y2 = result.get_bbox_xyxy(img_w, img_h)

        assert 0 <= x1 < x2 < img_w
        assert 0 <= y1 < y2 < img_h
        # 中心点验证
        cx = (x1 + x2) / 2 / img_w
        cy = (y1 + y2) / 2 / img_h
        assert abs(cx - 0.5) < 0.02
        assert abs(cy - 0.7) < 0.02

    def test_bbox_pixel_conversion(self):
        """测试bbox像素单位转换"""
        result = DetectionResult(
            bbox=(0.5, 0.7, 0.15, 0.20),
            confidence=0.9,
            class_id=1,
            sign_type=SignType.STRAIGHT,
        )
        img_w, img_h = 640, 480
        cx, cy, w, h = result.get_bbox_pixel(img_w, img_h)

        assert abs(cx - 320.0) < 0.1  # 0.5 * 640
        assert abs(cy - 336.0) < 0.1  # 0.7 * 480
        assert abs(w - 96.0) < 0.1   # 0.15 * 640
        assert abs(h - 96.0) < 0.1   # 0.20 * 480

    def test_get_stats(self):
        """测试统计信息"""
        detector = TrafficSignDetector()
        detector.load_model()

        image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        for _ in range(10):
            detector.detect(image)

        stats = detector.get_stats()
        assert stats['total_frames'] == 10
        assert stats['model_loaded'] is True
        assert stats['avg_inference_time_ms'] >= 0


class TestDetectionTracker:
    """检测跟踪器测试"""

    def test_init(self):
        """测试初始化"""
        tracker = DetectionTracker(history_size=5)
        assert tracker.history_size == 5
        assert tracker.stable_sign_type == SignType.UNKNOWN

    def test_update_with_detection(self):
        """测试有检测结果时的更新"""
        tracker = DetectionTracker(history_size=3)

        det = DetectionResult(
            bbox=(0.5, 0.5, 0.2, 0.2),
            confidence=0.85,
            class_id=0,
            sign_type=SignType.LEFT_TURN,
        )
        result = tracker.update([det])
        assert result is not None
        assert tracker.stable_sign_type == SignType.LEFT_TURN
        assert tracker.stable_confidence > 0

    def test_update_no_detection(self):
        """测试无检测结果时的更新"""
        tracker = DetectionTracker(history_size=3)

        result = tracker.update([])
        assert result is None
        assert tracker.stable_sign_type == SignType.UNKNOWN

    def test_temporal_smoothing(self):
        """测试时序平滑 - 稳定类型"""
        tracker = DetectionTracker(history_size=3)

        # 连续3次相同类型
        for sign_type in [SignType.STRAIGHT] * 3:
            det = DetectionResult(
                bbox=(0.5, 0.5, 0.2, 0.2),
                confidence=0.9,
                class_id=1,
                sign_type=sign_type,
            )
            tracker.update([det])

        assert tracker.stable_sign_type == SignType.STRAIGHT

    def test_temporal_smoothing_fluctuation(self):
        """测试时序平滑 - 类型波动时的稳定性"""
        tracker = DetectionTracker(history_size=3)

        # 两次直行一次左转 -> 应该多数票直行
        det_straight = DetectionResult(
            bbox=(0.5, 0.5, 0.2, 0.2),
            confidence=0.9,
            class_id=1,
            sign_type=SignType.STRAIGHT,
        )
        det_left = DetectionResult(
            bbox=(0.5, 0.5, 0.2, 0.2),
            confidence=0.9,
            class_id=0,
            sign_type=SignType.LEFT_TURN,
        )

        tracker.update([det_straight])
        tracker.update([det_straight])
        tracker.update([det_left])

        # 加权投票：两次直行权重更高
        assert tracker.stable_sign_type in [SignType.STRAIGHT, SignType.LEFT_TURN]

    def test_reset(self):
        """测试重置"""
        tracker = DetectionTracker(history_size=3)

        det = DetectionResult(
            bbox=(0.5, 0.5, 0.2, 0.2),
            confidence=0.9,
            class_id=0,
            sign_type=SignType.LEFT_TURN,
        )
        tracker.update([det])
        tracker.update([det])
        tracker.reset()

        assert tracker.stable_sign_type == SignType.UNKNOWN
        assert tracker.stable_confidence == 0.0
