"""异常处理模块单元测试

覆盖场景:
- 低置信度检测与容错
- 标识牌丢失恢复
- 重复识别去重
- 多标识牌场景
- 无效距离处理
- 异常记录与统计
"""

import time
import pytest

from yhs_traffic_sign.anomaly_handler import (
    AnomalyHandler,
    AnomalyType,
    FilteredResult,
)
from yhs_traffic_sign.traffic_sign_detector import DetectionResult
from yhs_traffic_sign.config import SignType


def make_detection(sign_type=SignType.STRAIGHT, confidence=0.9):
    """辅助函数：创建模拟检测结果"""
    class_id = {SignType.LEFT_TURN: 0, SignType.STRAIGHT: 1, SignType.RIGHT_TURN: 2}[sign_type]
    return DetectionResult(
        bbox=(0.5, 0.5, 0.2, 0.2),
        confidence=confidence,
        class_id=class_id,
        sign_type=sign_type,
    )


class TestAnomalyHandler:
    """异常处理器测试"""

    def setup_method(self):
        """每个测试前创建处理器"""
        self.handler = AnomalyHandler()

    # ==================== 正常情况测试 ====================
    def test_normal_processing(self):
        """测试正常处理流程"""
        detections = [make_detection(SignType.STRAIGHT, 0.9)]
        result = self.handler.process(detections, 0.3, True)

        assert result.is_valid is True
        assert result.sign_type == SignType.STRAIGHT
        assert result.distance == 0.3
        assert result.distance_valid is True
        assert len(result.filtered_anomalies) == 0

    # ==================== 低置信度测试 ====================
    def test_low_confidence_tolerated(self):
        """测试低置信度但在容忍范围内"""
        handler = AnomalyHandler(low_conf_tolerance=3)

        # 单次低置信度 - 应该还在容忍范围内
        detections = [make_detection(SignType.STRAIGHT, 0.3)]
        result = handler.process(detections, 0.3, True)
        assert result.is_valid is True  # 仍然有效

    def test_low_confidence_exceeded(self):
        """测试低置信度超过容忍阈值"""
        handler = AnomalyHandler(
            low_conf_threshold=0.5,
            low_conf_tolerance=2,
        )

        # 先建立有效历史
        for _ in range(3):
            handler.process([make_detection(SignType.STRAIGHT, 0.9)], 0.3, True)

        # 连续低置信度超过容忍阈值
        result = None
        for _ in range(3):
            result = handler.process([make_detection(SignType.STRAIGHT, 0.3)], 0.3, True)

        # 超过容忍阈值后应返回无效
        assert result is not None
        assert result.is_valid is False
        # 应该记录了低置信度异常
        anomalies = handler.get_recent_anomalies(100)
        low_conf_count = sum(
            1 for a in anomalies if a.anomaly_type == AnomalyType.LOW_CONFIDENCE
        )
        assert low_conf_count > 0

    # ==================== 标识牌丢失测试 ====================
    def test_sign_lost_short_hold(self):
        """测试标识牌短时丢失保持跟踪但sign_type置为UNKNOWN"""
        handler = AnomalyHandler(max_lost_frames=5)

        # 先建立有效历史
        for _ in range(3):
            handler.process([make_detection(SignType.STRAIGHT, 0.9)], 0.3, True)

        # 短时丢失 (少于最大丢失帧数)
        for _ in range(3):
            result = handler.process([], 0.0, False)

        # 丢失hold期间：is_valid保持True表示跟踪连续，
        # 但sign_type=UNKNOWN防止controller误触发
        assert result.is_valid is True
        assert result.sign_type == SignType.UNKNOWN
        assert result.fallback_used is True

    def test_sign_lost_long_invalid(self):
        """测试标识牌长时间丢失返回无效"""
        handler = AnomalyHandler(max_lost_frames=3)

        # 先建立历史
        for _ in range(3):
            handler.process([make_detection(SignType.STRAIGHT, 0.9)], 0.3, True)

        # 丢失超过最大帧数
        for _ in range(5):
            result = handler.process([], 0.0, False)

        assert result.is_valid is False

    def test_sign_lost_no_history(self):
        """测试丢失时无历史"""
        result = self.handler.process([], 0.0, False)
        assert result.is_valid is False

    # ==================== 重复识别测试 ====================
    def test_duplicate_detection(self):
        """测试重复识别去重"""
        handler = AnomalyHandler(dedup_window=5.0)

        # 第一次检测
        result1 = handler.process(
            [make_detection(SignType.STRAIGHT, 0.9)], 0.3, True
        )
        assert result1.is_valid is True

        # 同一类型短时间内再次检测 -> 重复
        result2 = handler.process(
            [make_detection(SignType.STRAIGHT, 0.9)], 0.3, True
        )
        # 结果仍然有效，但记录了重复异常
        assert result2.is_valid is True
        # 检查是否记录了重复异常
        anomalies = handler.get_recent_anomalies(10)
        dup_count = sum(1 for a in anomalies if a.anomaly_type == AnomalyType.DUPLICATE_DETECTION)
        assert dup_count > 0

    # ==================== 多标识牌测试 ====================
    def test_multiple_signs_selection(self):
        """测试多标识牌时的选择策略"""
        # 创建两个不同类型的检测结果
        det_straight = make_detection(SignType.STRAIGHT, 0.95)
        det_left = make_detection(SignType.LEFT_TURN, 0.85)

        result = self.handler.process([det_straight, det_left], 0.3, True)

        assert result.is_valid is True
        # 应该选择置信度更高的直行
        assert result.sign_type == SignType.STRAIGHT

    def test_multiple_signs_history_preference(self):
        """测试多标识牌时优先选择历史类型"""
        # 先建立左转的历史
        for _ in range(3):
            self.handler.process([make_detection(SignType.LEFT_TURN, 0.8)], 0.3, True)

        # 同时出现直行和左转，直行置信度更高
        det_left = make_detection(SignType.LEFT_TURN, 0.8)
        det_straight = make_detection(SignType.STRAIGHT, 0.95)

        result = self.handler.process([det_left, det_straight], 0.3, True)

        # 优先选择历史中的左转类型
        assert result.sign_type == SignType.LEFT_TURN

    # ==================== 无效距离测试 ====================
    def test_invalid_distance_with_history(self):
        """测试无效距离时使用历史值"""
        # 先建立有效历史
        for _ in range(3):
            self.handler.process(
                [make_detection(SignType.STRAIGHT, 0.9)], 2.0, True
            )

        # 距离变为无效
        result = self.handler.process(
            [make_detection(SignType.STRAIGHT, 0.9)], 0.0, False
        )

        assert result.is_valid is True
        assert result.distance_valid is True  # 使用了历史值
        assert result.distance == pytest.approx(2.0, abs=0.01)

    def test_invalid_distance_no_history(self):
        """测试无效距离且无历史"""
        result = self.handler.process(
            [make_detection(SignType.STRAIGHT, 0.9)], 0.0, False
        )
        # 没有历史时距离无效，但检测本身有效
        assert result.is_valid is False or result.distance_valid is False

    # ==================== 异常记录与统计测试 ====================
    def test_anomaly_history(self):
        """测试异常历史记录"""
        # 触发低置信度异常
        for _ in range(10):
            self.handler.process([make_detection(SignType.STRAIGHT, 0.1)], 0.3, True)

        stats = self.handler.get_anomaly_stats()
        assert stats['total'] > 0
        assert 'low_confidence' in stats.get('by_type', {})

    def test_recent_anomalies(self):
        """测试最近异常获取"""
        for _ in range(5):
            self.handler.process([], 0.0, False)

        recent = self.handler.get_recent_anomalies(3)
        assert len(recent) <= 3
        for a in recent:
            assert a.anomaly_type == AnomalyType.SIGN_LOST

    def test_reset(self):
        """测试重置"""
        for _ in range(10):
            self.handler.process([make_detection(SignType.STRAIGHT, 0.9)], 0.3, True)

        self.handler.reset()
        stats = self.handler.get_anomaly_stats()
        assert stats['total'] == 0
        assert stats['last_valid_result'] is False


class TestFilteredResult:
    """过滤结果数据类测试"""

    def test_default_values(self):
        """测试默认值"""
        result = FilteredResult(is_valid=True)
        assert result.sign_type == SignType.UNKNOWN
        assert result.confidence == 0.0
        assert result.filtered_anomalies == []
        assert result.fallback_used is False

    def test_with_values(self):
        """测试带值初始化"""
        result = FilteredResult(
            is_valid=True,
            sign_type=SignType.LEFT_TURN,
            confidence=0.85,
            distance=0.45,
            distance_valid=True,
            filtered_anomalies=[AnomalyType.DUPLICATE_DETECTION],
            fallback_used=False,
        )
        assert result.is_valid is True
        assert result.sign_type == SignType.LEFT_TURN
        assert result.confidence == pytest.approx(0.85)
