"""异常处理与容错模块

针对交通标识牌识别系统中的常见异常场景提供容错处理：
1. 标识牌重复识别 - 去重机制避免同一标识牌重复触发
2. 距离跳变 - 滑动窗口+异常值过滤保证距离稳定性
3. 识别置信度不足 - 多帧确认与历史状态继承
4. 标识牌短暂丢失 - 超时恢复机制
5. 检测到多个标识牌 - 智能选择最相关的

确保整体系统稳定可靠运行。
"""

import time
import logging
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Deque, List, Optional, Tuple

from .config import AnomalyConfig, SignType
from .traffic_sign_detector import DetectionResult


logger = logging.getLogger(__name__)


class AnomalyType(Enum):
    """异常类型枚举"""
    NONE = 'none'
    LOW_CONFIDENCE = 'low_confidence'           # 置信度不足
    DISTANCE_JUMP = 'distance_jump'             # 距离跳变
    SIGN_LOST = 'sign_lost'                     # 标识牌丢失
    DUPLICATE_DETECTION = 'duplicate_detection'  # 重复识别
    MULTIPLE_SIGNS = 'multiple_signs'           # 多标识牌同时出现
    INVALID_DISTANCE = 'invalid_distance'       # 无效距离值


@dataclass
class AnomalyRecord:
    """异常记录"""
    anomaly_type: AnomalyType
    timestamp: float
    details: str = ""
    severity: str = "warning"  # 'info', 'warning', 'error'


@dataclass
class FilteredResult:
    """经过异常处理过滤后的结果"""
    is_valid: bool
    sign_type: str = SignType.UNKNOWN
    confidence: float = 0.0
    distance: float = 0.0
    distance_valid: bool = False
    filtered_anomalies: List[AnomalyType] = field(default_factory=list)
    fallback_used: bool = False  # 是否使用了回退值


class AnomalyHandler:
    """异常处理器

    整合各种容错逻辑，对检测和测距结果进行
    多层过滤和异常恢复，输出稳定可靠的结果。
    """

    def __init__(
        self,
        low_conf_threshold: float = AnomalyConfig.LOW_CONFIDENCE_THRESHOLD,
        low_conf_tolerance: int = AnomalyConfig.LOW_CONFIDENCE_TOLERANCE,
        dedup_window: float = AnomalyConfig.DEDUP_TIME_WINDOW,
        sign_lost_hold: float = AnomalyConfig.SIGN_LOST_HOLD_TIME,
        max_lost_frames: int = AnomalyConfig.MAX_LOST_FRAMES,
    ):
        """初始化异常处理器

        Args:
            low_conf_threshold: 低置信度阈值
            low_conf_tolerance: 连续低置信度容忍帧数
            dedup_window: 去重时间窗口(秒)
            sign_lost_hold: 标识牌丢失后保持时间(秒)
            max_lost_frames: 最大连续丢失帧数
        """
        self.low_conf_threshold = low_conf_threshold
        self.low_conf_tolerance = low_conf_tolerance
        self.dedup_window = dedup_window
        self.sign_lost_hold = sign_lost_hold
        self.max_lost_frames = max_lost_frames

        # 状态跟踪
        self._low_conf_frames: int = 0
        self._lost_frames: int = 0
        self._last_valid_result: Optional[FilteredResult] = None
        self._last_dedup_signs: List[Tuple[str, float]] = []  # [(sign_type, timestamp)]
        self._anomaly_history: Deque[AnomalyRecord] = deque(maxlen=100)

    def process(
        self,
        detections: List[DetectionResult],
        distance: float,
        distance_valid: bool,
        distance_confidence: float = 1.0,
    ) -> FilteredResult:
        """对检测和测距结果进行异常过滤

        Args:
            detections: 当前帧检测结果列表
            distance: 测距结果
            distance_valid: 距离是否有效
            distance_confidence: 距离置信度

        Returns:
            FilteredResult 经过异常处理的结果
        """
        now = time.time()
        filtered_anomalies: List[AnomalyType] = []

        # Step 1: 处理无检测结果
        if not detections:
            return self._handle_no_detection(now)

        # Step 2: 处理多标识牌场景
        best_detection = self._select_best_detection(detections, filtered_anomalies)

        # Step 3: 低置信度处理
        if best_detection.confidence < self.low_conf_threshold:
            self._low_conf_frames += 1
            filtered_anomalies.append(AnomalyType.LOW_CONFIDENCE)

            if self._low_conf_frames >= self.low_conf_tolerance:
                logger.warning(
                    f"连续 {self._low_conf_frames} 帧低置信度 "
                    f"(阈值 {self.low_conf_threshold})，返回无效结果"
                )
                self._record_anomaly(
                    AnomalyType.LOW_CONFIDENCE,
                    now,
                    f"置信度 {best_detection.confidence:.2f} 持续低于阈值",
                )
                # 低置信度不应该使用历史回退（和标识牌丢失不同）
                return FilteredResult(
                    is_valid=False,
                    filtered_anomalies=filtered_anomalies,
                )
        else:
            # 正常置信度，重置低置信度计数
            self._low_conf_frames = 0

        # Step 4: 重复识别处理
        if self._is_duplicate(best_detection.sign_type, now):
            filtered_anomalies.append(AnomalyType.DUPLICATE_DETECTION)
            self._record_anomaly(
                AnomalyType.DUPLICATE_DETECTION,
                now,
                f"标识牌 {best_detection.sign_type} 在去重窗口内重复出现",
            )
            # 重复不影响有效性，只去重时间窗口
            self._update_dedup(best_detection.sign_type, now)

        # Step 5: 距离异常检查
        if not distance_valid or distance <= 0:
            filtered_anomalies.append(AnomalyType.INVALID_DISTANCE)
            self._record_anomaly(
                AnomalyType.INVALID_DISTANCE,
                now,
                f"无效距离值: {distance:.3f}",
            )
            # 尝试使用历史距离
            if self._last_valid_result is not None and self._last_valid_result.distance_valid:
                distance = self._last_valid_result.distance
                distance_valid = True
                distance_confidence *= 0.5  # 降低置信度
                logger.debug(f"使用历史距离值: {distance:.3f}m")

        # Step 6: 重置丢失计数
        self._lost_frames = 0

        # 构建有效结果
        result = FilteredResult(
            is_valid=True,
            sign_type=best_detection.sign_type,
            confidence=best_detection.confidence,
            distance=distance,
            distance_valid=distance_valid,
            filtered_anomalies=filtered_anomalies,
        )

        # 更新历史
        self._last_valid_result = result
        self._update_dedup(best_detection.sign_type, now)

        return result

    def _handle_no_detection(self, now: float) -> FilteredResult:
        """处理无检测结果的情况"""
        self._lost_frames += 1
        filtered_anomalies = [AnomalyType.SIGN_LOST]

        self._record_anomaly(
            AnomalyType.SIGN_LOST,
            now,
            f"标识牌丢失 (连续 {self._lost_frames} 帧)",
        )

        # 短时丢失保持历史结果 - 但sign_type设为UNKNOWN防止控制累计
        if (
            self._last_valid_result is not None
            and self._lost_frames <= self.max_lost_frames
        ):
            logger.debug(
                f"标识牌短时丢失 ({self._lost_frames}/{self.max_lost_frames}), "
                f"保持距离值但sign_type置为UNKNOWN"
            )
            result = FilteredResult(
                is_valid=True,
                sign_type=SignType.UNKNOWN,  # 置为UNKNOWN防止controller继续累计
                confidence=self._last_valid_result.confidence * 0.5,  # 降置信度
                distance=self._last_valid_result.distance,
                distance_valid=self._last_valid_result.distance_valid,
                filtered_anomalies=filtered_anomalies,
                fallback_used=True,
            )
            return result

        # 长时间丢失，返回无效
        return FilteredResult(
            is_valid=False,
            filtered_anomalies=filtered_anomalies,
        )

    def _select_best_detection(
        self,
        detections: List[DetectionResult],
        anomalies: List[AnomalyType],
    ) -> DetectionResult:
        """从多个检测结果中选择最优的一个

        选择策略：
        1. 优先选择与历史相同的类型（连续性优先）
        2. 否则选择置信度最高的
        """
        if len(detections) == 1:
            return detections[0]

        # 多个检测结果
        anomalies.append(AnomalyType.MULTIPLE_SIGNS)
        self._record_anomaly(
            AnomalyType.MULTIPLE_SIGNS,
            time.time(),
            f"检测到 {len(detections)} 个标识牌",
        )

        # 优先选择与历史相同类型
        if self._last_valid_result is not None:
            for det in detections:
                if det.sign_type == self._last_valid_result.sign_type:
                    return det

        # 否则取置信度最高的
        return max(detections, key=lambda d: d.confidence)

    def _is_duplicate(self, sign_type: str, now: float) -> bool:
        """检查是否为重复识别"""
        # 清理过期记录
        self._last_dedup_signs = [
            (t, ts) for t, ts in self._last_dedup_signs
            if now - ts < self.dedup_window
        ]
        # 同一类型在去重窗口内已出现过
        return any(t == sign_type for t, ts in self._last_dedup_signs)

    def _update_dedup(self, sign_type: str, now: float):
        """更新去重记录"""
        # 先移除同类型的旧记录，再添加新记录
        self._last_dedup_signs = [
            (t, ts) for t, ts in self._last_dedup_signs if t != sign_type
        ]
        self._last_dedup_signs.append((sign_type, now))

    def _create_result_from_history(
        self,
        anomalies: List[AnomalyType],
        failure_type: AnomalyType,
    ) -> FilteredResult:
        """基于历史结果创建回退结果"""
        assert self._last_valid_result is not None
        return FilteredResult(
            is_valid=True,
            sign_type=self._last_valid_result.sign_type,
            confidence=self._last_valid_result.confidence * 0.7,
            distance=self._last_valid_result.distance,
            distance_valid=self._last_valid_result.distance_valid,
            filtered_anomalies=anomalies + [failure_type],
            fallback_used=True,
        )

    def _record_anomaly(
        self,
        anomaly_type: AnomalyType,
        timestamp: float,
        details: str = "",
        severity: str = "warning",
    ):
        """记录异常"""
        self._anomaly_history.append(
            AnomalyRecord(
                anomaly_type=anomaly_type,
                timestamp=timestamp,
                details=details,
                severity=severity,
            )
        )

    def reset(self):
        """重置所有状态"""
        self._low_conf_frames = 0
        self._lost_frames = 0
        self._last_valid_result = None
        self._last_dedup_signs.clear()
        self._anomaly_history.clear()

    def get_anomaly_stats(self) -> dict:
        """获取异常统计"""
        stats = {
            'total': len(self._anomaly_history),
            'by_type': {},
            'low_conf_frames': self._low_conf_frames,
            'lost_frames': self._lost_frames,
            'last_valid_result': self._last_valid_result is not None,
        }
        for record in self._anomaly_history:
            type_name = record.anomaly_type.value
            stats['by_type'][type_name] = stats['by_type'].get(type_name, 0) + 1
        return stats

    def get_recent_anomalies(self, n: int = 10) -> List[AnomalyRecord]:
        """获取最近n条异常记录"""
        return list(self._anomaly_history)[-n:]
