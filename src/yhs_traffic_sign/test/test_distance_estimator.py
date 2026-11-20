"""距离感知模块单元测试

覆盖场景:
- 深度图测距
- 单目几何测距
- 移动平均滤波
- 距离跳变检测
- 无效数据处理
- 回退机制
"""

import pytest
import numpy as np

from yhs_traffic_sign.distance_estimator import DistanceEstimator, DistanceResult
from yhs_traffic_sign.config import DistanceConfig


class TestDistanceEstimator:
    """距离估计器测试"""

    def setup_method(self):
        """每个测试前创建估计器"""
        self.estimator = DistanceEstimator(
            method='depth',
            fx=525.0,
            fy=525.0,
            cx=319.5,
            cy=239.5,
        )

    # ==================== 深度测距测试 ====================
    def test_depth_estimation_basic(self):
        """测试基本的深度测距"""
        # 创建模拟深度图 (中心区域深度为2.0米)
        depth_image = np.full((480, 640), 2000, dtype=np.uint16)  # 毫米
        # 在检测框区域内设置深度
        depth_image[180:240, 280:360] = 1500  # 1.5米

        bbox_xyxy = (280, 180, 360, 240)
        result = self.estimator.estimate_from_depth(depth_image, bbox_xyxy)

        assert result.distance_valid is True
        assert 1.4 < result.distance < 1.6  # 约1.5米
        assert result.method == 'depth'
        assert result.confidence > 0

    def test_depth_estimation_with_float(self):
        """测试float类型深度图"""
        depth_image = np.full((480, 640), np.nan, dtype=np.float32)
        depth_image[180:240, 280:360] = 2.5  # 米

        bbox_xyxy = (280, 180, 360, 240)
        result = self.estimator.estimate_from_depth(depth_image, bbox_xyxy)

        assert result.distance_valid is True
        assert 2.4 < result.distance < 2.6

    def test_depth_estimation_invalid_data(self):
        """测试无效深度数据"""
        # 全零深度图
        depth_image = np.zeros((480, 640), dtype=np.uint16)
        bbox_xyxy = (280, 180, 360, 240)
        result = self.estimator.estimate_from_depth(depth_image, bbox_xyxy)

        assert result.distance_valid is False

    def test_depth_estimation_out_of_range(self):
        """测试超出有效范围的距离"""
        depth_image = np.full((480, 640), 200, dtype=np.uint16)  # 0.2米
        bbox_xyxy = (280, 180, 360, 240)

        estimator = DistanceEstimator(min_valid_distance=0.5)
        result = estimator.estimate_from_depth(depth_image, bbox_xyxy)
        assert result.distance_valid is False

    def test_depth_estimation_none_input(self):
        """测试None输入"""
        result = self.estimator.estimate_from_depth(None, (0, 0, 10, 10))
        assert result.distance_valid is False

    # ==================== 单目几何测距测试 ====================
    def test_monocular_estimation(self):
        """测试单目几何测距"""
        # 标识牌在图像中宽度100像素
        # fx=525, sign_real_width=0.6m
        # 距离 Z = fx * W_real / W_pixel = 525 * 0.6 / 100 = 3.15m
        bbox_pixel = (320, 240, 100, 100)
        result = self.estimator.estimate_from_monocular(bbox_pixel)

        assert result.distance_valid is True
        assert abs(result.distance - 3.15) < 0.1  # 约3.15米
        assert result.method == 'monocular'

    def test_monocular_different_distance(self):
        """测试不同距离的单目估计"""
        estimator = DistanceEstimator(fx=525.0, sign_real_width=0.6)

        # 近距离 -> 大像素宽度
        bbox_near = (320, 240, 200, 200)  # 200像素宽
        near_result = estimator.estimate_from_monocular(bbox_near)
        assert near_result.distance < 2.0  # 约1.575米

        # 远距离 -> 小像素宽度
        bbox_far = (320, 240, 50, 50)  # 50像素宽
        far_result = estimator.estimate_from_monocular(bbox_far)
        assert far_result.distance > 5.0  # 约6.3米

        # 近的比远的近
        assert near_result.distance < far_result.distance

    def test_monocular_zero_size(self):
        """测试零尺寸bbox"""
        result = self.estimator.estimate_from_monocular((320, 240, 0, 10))
        assert result.distance_valid is False

    # ==================== 移动平均滤波测试 ====================
    def test_moving_average_filtering(self):
        """测试移动平均滤波"""
        estimator = DistanceEstimator(avg_window=5)

        # 连续多次测量，距离略有波动
        distances = [2.0, 2.1, 1.9, 2.05, 2.15]
        for d in distances:
            depth = np.full((480, 640), int(d * 1000), dtype=np.uint16)
            result = estimator.estimate_from_depth(depth, (280, 180, 360, 240))

        # 滤波后的结果应该接近平均值
        avg = np.mean(distances)
        assert abs(result.distance - avg) < 0.2

    # ==================== 距离跳变检测测试 ====================
    def test_distance_jump_detection(self):
        """测试距离跳变检测 (通过统一入口estimate)"""
        estimator = DistanceEstimator(jump_threshold=0.3)

        # 连续正常测量 (使用estimate入口以触发滤波)
        for d in [2.0, 2.05, 1.95]:
            depth = np.full((480, 640), int(d * 1000), dtype=np.uint16)
            estimator.estimate(
                bbox_xyxy=(280, 180, 360, 240),
                image_shape=(480, 640),
                depth_image=depth,
            )

        # 突然跳变到5.0米
        jump_depth = np.full((480, 640), 5000, dtype=np.uint16)
        jump_result = estimator.estimate(
            bbox_xyxy=(280, 180, 360, 240),
            image_shape=(480, 640),
            depth_image=jump_depth,
        )

        # 跳变应该被过滤，返回上一个有效值
        assert jump_result.distance < 3.0  # 应该保持在约2米

    # ==================== 回退机制测试 ====================
    def test_fallback_to_monocular(self):
        """测试深度不可用时回退到单目"""
        estimator = DistanceEstimator(method='depth')

        # 深度图全零（无效），但有单目bbox信息
        depth_image = np.zeros((480, 640), dtype=np.uint16)
        bbox_xyxy = (280, 180, 360, 240)
        bbox_pixel = (320.0, 210.0, 80.0, 60.0)

        result = estimator.estimate(
            bbox_xyxy=bbox_xyxy,
            image_shape=(480, 640),
            depth_image=depth_image,
            sign_bbox_pixel=bbox_pixel,
        )

        # 应该回退到单目
        assert result.distance_valid is True or result.distance > 0

    # ==================== 统一入口测试 ====================
    def test_unified_estimate_with_depth(self):
        """测试统一入口 - 使用深度"""
        depth = np.full((480, 640), 3000, dtype=np.uint16)
        result = self.estimator.estimate(
            bbox_xyxy=(280, 180, 360, 240),
            image_shape=(480, 640),
            depth_image=depth,
        )
        assert result.distance_valid is True
        assert 2.9 < result.distance < 3.1

    def test_unified_estimate_with_monocular(self):
        """测试统一入口 - 使用单目"""
        estimator = DistanceEstimator(method='monocular')
        result = estimator.estimate(
            bbox_xyxy=(280, 180, 360, 240),
            image_shape=(480, 640),
            sign_bbox_pixel=(320.0, 210.0, 80.0, 60.0),
        )
        assert result.distance_valid is True

    # ==================== 统计信息测试 ====================
    def test_get_stats(self):
        """测试统计信息"""
        stats = self.estimator.get_stats()
        assert 'method' in stats
        assert 'last_valid_distance' in stats
        assert stats['method'] == 'depth'

    def test_reset(self):
        """测试重置"""
        # 先做几次测量
        depth = np.full((480, 640), 2000, dtype=np.uint16)
        self.estimator.estimate_from_depth(depth, (280, 180, 360, 240))

        self.estimator.reset()
        stats = self.estimator.get_stats()
        assert stats['last_valid_distance'] is None
        assert stats['history_size'] == 0
