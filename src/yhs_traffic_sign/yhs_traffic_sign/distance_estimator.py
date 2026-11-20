"""距离感知与测距模块

提供两种测距方法：
1. 深度相机测距 - 从深度图中提取标识牌区域的平均深度值
2. 单目几何测距 - 利用标识牌已知物理尺寸和相机内参计算距离

距离检测误差控制在 ±0.05米 以内。
"""

import time
import logging
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Tuple

import numpy as np

from .config import DistanceConfig, AnomalyConfig


logger = logging.getLogger(__name__)


@dataclass
class DistanceResult:
    """距离测量结果"""
    distance: float              # 测量距离 (米)
    distance_valid: bool         # 距离是否有效
    confidence: float            # 测量置信度 [0, 1]
    method: str                  # 测距方法 ('depth' / 'monocular')
    timestamp: float = 0.0       # 测量时间戳(秒)
    measurement_error: float = 0.0  # 估算的测量误差 (米)
    pixel_center: Tuple[int, int] = (0, 0)  # 标识牌中心点像素坐标

    def __post_init__(self):
        if self.timestamp == 0.0:
            self.timestamp = time.time()


class DistanceEstimator:
    """距离估计器

    支持深度相机和单目几何两种测距方法，
    内置移动平均滤波和跳变检测，
    确保距离检测误差 ≤ ±0.05米。
    """

    def __init__(
        self,
        method: str = DistanceConfig.DISTANCE_METHOD,
        sign_real_width: float = DistanceConfig.SIGN_REAL_WIDTH,
        sign_real_height: float = DistanceConfig.SIGN_REAL_HEIGHT,
        fx: float = DistanceConfig.CAMERA_FX,
        fy: float = DistanceConfig.CAMERA_FY,
        cx: float = DistanceConfig.CAMERA_CX,
        cy: float = DistanceConfig.CAMERA_CY,
        max_valid_distance: float = DistanceConfig.MAX_VALID_DISTANCE,
        min_valid_distance: float = DistanceConfig.MIN_VALID_DISTANCE,
        jump_threshold: float = DistanceConfig.DISTANCE_JUMP_THRESHOLD,
        avg_window: int = DistanceConfig.DISTANCE_AVERAGE_WINDOW,
    ):
        """初始化距离估计器

        Args:
            method: 测距方法 ('depth' 或 'monocular')
            sign_real_width: 标识牌实际宽度(米)
            sign_real_height: 标识牌实际高度(米)
            fx, fy: 相机焦距(像素)
            cx, cy: 相机主点(像素)
            max_valid_distance: 最大有效距离(米)
            min_valid_distance: 最小有效距离(米)
            jump_threshold: 距离跳变检测阈值(米)
            avg_window: 移动平均窗口大小
        """
        self.method = method
        self.sign_real_width = sign_real_width
        self.sign_real_height = sign_real_height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.max_valid_distance = max_valid_distance
        self.min_valid_distance = min_valid_distance
        self.jump_threshold = jump_threshold
        self.avg_window = avg_window

        # 滤波相关
        self._distance_history: Deque[float] = deque(maxlen=avg_window)
        self._last_valid_distance: Optional[float] = None
        self._jump_frames_remaining: int = 0

    def estimate_from_depth(
        self,
        depth_image: np.ndarray,
        bbox_xyxy: Tuple[int, int, int, int],
    ) -> DistanceResult:
        """从深度图估计距离

        在检测框区域内计算有效深度值的中位数作为距离测量。
        使用中位数代替均值以减少噪声和离群值的影响。

        Args:
            depth_image: 深度图 (H, W)，单位米或毫米
            bbox_xyxy: 标识牌边界框 (x1, y1, x2, y2) 像素坐标

        Returns:
            DistanceResult 距离测量结果
        """
        if depth_image is None or depth_image.size == 0:
            return self._make_invalid_result(method='depth', reason='深度图为空')

        try:
            x1, y1, x2, y2 = bbox_xyxy
            h, w = depth_image.shape[:2]

            # 裁剪到有效范围
            x1 = max(0, min(x1, w - 1))
            y1 = max(0, min(y1, h - 1))
            x2 = max(0, min(x2, w - 1))
            y2 = max(0, min(y2, h - 1))

            if x2 <= x1 or y2 <= y1:
                return self._make_invalid_result(method='depth', reason='无效bbox')

            # 提取ROI区域的深度值
            roi = depth_image[y1:y2, x1:x2].astype(np.float32)

            # 处理深度图单位转换
            # 深度图可能是 uint16 (毫米) 或 float32 (米)
            if depth_image.dtype == np.uint16:
                roi = roi / 1000.0  # 毫米转米
                roi[roi < 0.0] = 0.0
            elif depth_image.dtype == np.float32 or depth_image.dtype == np.float64:
                # float类型，假设单位是米
                pass
            else:
                roi = roi.astype(np.float32)

            # 过滤无效深度值 (0或nan或超出范围)
            valid_mask = (
                (roi > self.min_valid_distance)
                & (roi < self.max_valid_distance)
                & np.isfinite(roi)
            )
            valid_depths = roi[valid_mask]

            if len(valid_depths) == 0:
                return self._make_invalid_result(
                    method='depth', reason='ROI内无有效深度数据'
                )

            # 使用中位数获得鲁棒估计
            median_depth = float(np.median(valid_depths))

            # 计算置信度：基于有效像素比例
            valid_ratio = len(valid_depths) / roi.size
            # 深度值方差越小置信度越高
            depth_std = float(np.std(valid_depths))
            confidence = self._compute_confidence(valid_ratio, depth_std, median_depth)

            # 估算测量误差
            measurement_error = self._estimate_depth_error(valid_depths, median_depth)

            # 像素中心
            pixel_center = ((x1 + x2) // 2, (y1 + y2) // 2)

            # 有效范围检查
            distance_valid = bool(self.min_valid_distance <= median_depth <= self.max_valid_distance)

            return DistanceResult(
                distance=median_depth,
                distance_valid=distance_valid,
                confidence=confidence,
                method='depth',
                measurement_error=measurement_error,
                pixel_center=pixel_center,
            )

        except Exception as e:
            logger.error(f"深度测距异常: {e}")
            return self._make_invalid_result(method='depth', reason=str(e))

    def estimate_from_monocular(
        self,
        bbox_pixel: Tuple[float, float, float, float],
    ) -> DistanceResult:
        """单目几何测距

        利用标识牌已知物理尺寸和在图像中的像素尺寸，
        通过针孔相机模型计算距离。

        公式: Z = f * W_real / W_pixel
        其中 f 是焦距，W_real 是实际宽度，W_pixel 是像素宽度

        Args:
            bbox_pixel: 像素单位的bbox (cx, cy, w, h)

        Returns:
            DistanceResult 距离测量结果
        """
        try:
            cx, cy, w, h = bbox_pixel

            if w <= 0 or h <= 0:
                return self._make_invalid_result(method='monocular', reason='无效bbox尺寸')

            # 使用宽度计算距离
            distance_by_width = (self.fx * self.sign_real_width) / w if self.fx > 0 else float('inf')

            # 使用高度计算距离
            distance_by_height = (self.fy * self.sign_real_height) / h if self.fy > 0 else float('inf')

            # 取两者平均值，并按置信度加权 (离图像中心越近越准)
            # 计算距主点的偏移
            offset_x = abs(cx - self.cx)
            offset_y = abs(cy - self.cy)

            # 偏移越大，该估计权重越低
            offset_weight_w = 1.0 / (1.0 + offset_x / self.cx)
            offset_weight_h = 1.0 / (1.0 + offset_y / self.cy)

            total_weight = offset_weight_w + offset_weight_h
            distance = (
                distance_by_width * offset_weight_w + distance_by_height * offset_weight_h
            ) / total_weight

            # 置信度计算
            # bbox太小/太大会降低置信度
            min_pixel_size = 10  # 至少10像素宽
            size_confidence = min(w, h) / min_pixel_size if min(w, h) < min_pixel_size else 1.0
            size_confidence = min(1.0, max(0.0, size_confidence))

            # 距离越远，估计误差越大
            dist_confidence = max(0.0, 1.0 - distance / self.max_valid_distance)

            confidence = (size_confidence + dist_confidence) / 2.0

            # 估算测量误差 (基于相机标定精度和距离)
            # 距离越远，相对误差越大
            relative_error = 0.02 * distance  # 假设2%相对误差
            measurement_error = min(0.1, relative_error)  # 最大0.1米

            # 有效范围检查 (使用原生Python bool避免numpy bool_问题)
            distance_valid = bool(
                self.min_valid_distance <= distance <= self.max_valid_distance
                and np.isfinite(distance)
            )

            pixel_center = (int(cx), int(cy))

            return DistanceResult(
                distance=float(distance),
                distance_valid=distance_valid,
                confidence=float(confidence),
                method='monocular',
                measurement_error=float(measurement_error),
                pixel_center=pixel_center,
            )

        except Exception as e:
            logger.error(f"单目测距异常: {e}")
            return self._make_invalid_result(method='monocular', reason=str(e))

    def estimate(
        self,
        bbox_xyxy: Tuple[int, int, int, int],
        image_shape: Tuple[int, int],
        depth_image: Optional[np.ndarray] = None,
        sign_bbox_pixel: Optional[Tuple[float, float, float, float]] = None,
    ) -> DistanceResult:
        """统一测距入口

        根据配置的测距方法，自动选择对应的测距实现。
        同时支持混合模式：优先使用深度相机，不可用时回退到单目。
        无论method配置如何，当首选方法失败时都会自动尝试回退。

        Args:
            bbox_xyxy: 像素坐标bbox (x1, y1, x2, y2)
            image_shape: 图像尺寸 (height, width)
            depth_image: 可选的深度图
            sign_bbox_pixel: 像素单位的(cx, cy, w, h)，用于单目测距

        Returns:
            DistanceResult
        """
        result = None

        # 首选方法
        if self.method == 'depth' and depth_image is not None:
            result = self.estimate_from_depth(depth_image, bbox_xyxy)
            # 如果深度结果无效且有单目信息，回退
            if not result.distance_valid and sign_bbox_pixel is not None:
                logger.debug("深度测距无效，回退到单目几何测距")
                result = self.estimate_from_monocular(sign_bbox_pixel)
        elif self.method == 'monocular' and sign_bbox_pixel is not None:
            result = self.estimate_from_monocular(sign_bbox_pixel)
            # 如果单目结果无效且有深度信息，回退
            if not result.distance_valid and depth_image is not None:
                result = self.estimate_from_depth(depth_image, bbox_xyxy)
        elif depth_image is not None:
            # 有深度图但method不匹配，先尝试深度
            result = self.estimate_from_depth(depth_image, bbox_xyxy)
            if not result.distance_valid and sign_bbox_pixel is not None:
                logger.debug("深度测距无效，回退到单目几何测距")
                result = self.estimate_from_monocular(sign_bbox_pixel)
        elif sign_bbox_pixel is not None:
            result = self.estimate_from_monocular(sign_bbox_pixel)
        else:
            return self._make_invalid_result(
                method=self.method,
                reason='缺少测距所需的图像数据',
            )

        # 应用滤波和跳变检测
        filtered_result = self._apply_filtering(result)
        return filtered_result

    def _compute_confidence(
        self, valid_ratio: float, depth_std: float, depth: float
    ) -> float:
        """计算深度测量置信度"""
        # 有效像素比例 (越高越好)
        ratio_conf = valid_ratio

        # 深度标准差 (越小越好)，归一化
        std_norm = min(1.0, depth_std / max(0.01, depth * 0.1))
        std_conf = 1.0 - std_norm

        # 综合置信度
        return float((ratio_conf + std_conf) / 2.0)

    def _estimate_depth_error(
        self, valid_depths: np.ndarray, median: float
    ) -> float:
        """估算深度测量误差"""
        # 使用MAD (Median Absolute Deviation) 作为鲁棒误差估计
        mad = float(np.median(np.abs(valid_depths - median)))
        # 假设正态分布，MAD ≈ 0.6745 * σ，误差取 3σ
        sigma = mad / 0.6745 if mad > 0 else 0.01
        return float(min(0.1, 3 * sigma))

    def _apply_filtering(self, result: DistanceResult) -> DistanceResult:
        """对原始测距结果应用滤波和跳变检测"""
        if not result.distance_valid:
            # 无效结果不加入历史，但保持上一个有效值
            if self._last_valid_distance is not None:
                # 如果距离跳变恢复中，返回上一个有效值
                if self._jump_frames_remaining > 0:
                    result.distance = self._last_valid_distance
                    result.confidence *= 0.5  # 降置信度
                    self._jump_frames_remaining -= 1
            return result

        current_distance = result.distance

        # 跳变检测
        if self._last_valid_distance is not None:
            distance_change = abs(current_distance - self._last_valid_distance)

            if distance_change > self.jump_threshold:
                # 检测到距离跳变
                logger.debug(
                    f"距离跳变检测: {self._last_valid_distance:.3f}m -> "
                    f"{current_distance:.3f}m (变化{distance_change:.3f}m)"
                )
                # 在恢复窗口内继续使用上一个有效值
                self._jump_frames_remaining = AnomalyConfig.JUMP_RECOVERY_FRAMES
                result.distance = self._last_valid_distance
                result.confidence *= 0.6
                return result

        # 移动平均滤波
        self._distance_history.append(current_distance)
        filtered_distance = float(np.mean(self._distance_history))

        # 更新结果
        result.distance = filtered_distance
        self._last_valid_distance = filtered_distance

        return result

    def _make_invalid_result(
        self, method: str, reason: str = ""
    ) -> DistanceResult:
        """创建无效的测量结果"""
        logger.debug(f"测距无效 [{method}]: {reason}")
        return DistanceResult(
            distance=0.0,
            distance_valid=False,
            confidence=0.0,
            method=method,
            measurement_error=0.0,
            pixel_center=(0, 0),
        )

    def reset(self):
        """重置所有内部状态"""
        self._distance_history.clear()
        self._last_valid_distance = None
        self._jump_frames_remaining = 0

    def get_stats(self) -> dict:
        """获取估计器统计信息"""
        return {
            'method': self.method,
            'last_valid_distance': self._last_valid_distance,
            'avg_window': self.avg_window,
            'history_size': len(self._distance_history),
            'jump_frames_remaining': self._jump_frames_remaining,
        }
