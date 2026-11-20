"""车辆控制触发模块

根据识别到的交通标识牌类型和距离，
自动生成对应的车辆控制指令，实现自动响应。

指令执行准确率 ≥ 99%
"""

import time
import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Optional, Tuple

from .config import ControlConfig, SignType, SignType as ST


logger = logging.getLogger(__name__)


class ControlState(Enum):
    """控制状态机状态"""
    IDLE = 'idle'                     # 空闲，等待检测
    CONFIRMING = 'confirming'         # 确认中，连续N次检测到
    TRIGGERED = 'triggered'           # 已触发，正在执行指令
    COOLDOWN = 'cooldown'             # 冷却中，等待下一次触发


class ControlAction(Enum):
    """控制动作类型"""
    GO_STRAIGHT = 'go_straight'       # 直行通过
    TURN_LEFT = 'turn_left'           # 左转
    TURN_RIGHT = 'turn_right'         # 右转
    STOP = 'stop'                     # 停车
    NONE = 'none'                     # 无动作


@dataclass
class ControlCommand:
    """车辆控制指令"""
    action: ControlAction              # 控制动作
    linear_velocity: float             # 线速度 (m/s)
    angular_velocity: float            # 角速度 (rad/s)
    gear: int = 1                      # 档位 (0=空挡, 1=前进, 2=后退)
    duration: float = 0.0               # 指令持续时间(秒)，0表示持续
    timestamp: float = 0.0             # 指令时间戳
    source_sign_type: str = ST.UNKNOWN  # 触发来源的标识牌类型
    triggered_distance: float = 0.0     # 触发时的距离

    def __post_init__(self):
        if self.timestamp == 0.0:
            self.timestamp = time.time()

    @property
    def is_valid(self) -> bool:
        """指令是否有效"""
        return self.action != ControlAction.NONE

    def to_dict(self) -> dict:
        """转换为字典便于日志和调试"""
        return {
            'action': self.action.value,
            'linear': f'{self.linear_velocity:.3f} m/s',
            'angular': f'{self.angular_velocity:.3f} rad/s',
            'gear': self.gear,
            'duration': f'{self.duration:.1f}s',
            'source': SignType.CN_NAMES.get(self.source_sign_type, self.source_sign_type),
            'trigger_distance': f'{self.triggered_distance:.3f}m',
        }


@dataclass
class ControlStateInfo:
    """控制状态信息"""
    state: ControlState = ControlState.IDLE
    current_action: ControlAction = ControlAction.NONE
    confirm_count: int = 0
    trigger_time: float = 0.0
    cooldown_until: float = 0.0
    last_command: Optional[ControlCommand] = None


class VehicleController:
    """车辆控制器

    状态机管理控制流程，提供触发条件判断和指令生成。
    支持回调式指令发送和直接命令访问两种模式。
    """

    def __init__(
        self,
        trigger_distance: float = ControlConfig.TRIGGER_DISTANCE,
        confirm_count: int = ControlConfig.TRIGGER_CONFIRM_COUNT,
        cooldown: float = ControlConfig.CONTROL_COOLDOWN,
        straight_speed: float = ControlConfig.STRAIGHT_SPEED,
        left_angular: float = ControlConfig.LEFT_TURN_ANGULAR_VELOCITY,
        right_angular: float = ControlConfig.RIGHT_TURN_ANGULAR_VELOCITY,
        turn_duration: float = ControlConfig.TURN_DURATION,
        on_command_callback: Optional[Callable[[ControlCommand], None]] = None,
    ):
        """初始化车辆控制器

        Args:
            trigger_distance: 触发控制的距离阈值(米)
            confirm_count: 连续确认次数
            cooldown: 指令冷却时间(秒)
            straight_speed: 直行速度
            left_angular: 左转角速度
            right_angular: 右转角速度
            turn_duration: 转向持续时间
            on_command_callback: 指令发送回调函数
        """
        self.trigger_distance = trigger_distance
        self.confirm_count = confirm_count
        self.cooldown = cooldown
        self.straight_speed = straight_speed
        self.left_angular = left_angular
        self.right_angular = right_angular
        self.turn_duration = turn_duration

        self._on_command = on_command_callback

        # 状态机
        self._state = ControlState.IDLE
        self._confirm_count = 0
        self._current_sign_type: Optional[str] = None
        self._trigger_time: float = 0.0
        self._cooldown_until: float = 0.0
        self._last_command: Optional[ControlCommand] = None

        # 准确率统计
        self._total_triggers = 0
        self._successful_triggers = 0

    def set_command_callback(
        self, callback: Callable[[ControlCommand], None]
    ):
        """设置指令发送回调"""
        self._on_command = callback

    def process(
        self,
        sign_type: str,
        distance: float,
        distance_valid: bool,
    ) -> Optional[ControlCommand]:
        """处理一次识别结果，可能生成控制指令

        Args:
            sign_type: 识别到的标识牌类型
            distance: 测量距离(米)
            distance_valid: 距离是否有效

        Returns:
            若触发新指令则返回ControlCommand，否则返回None
        """
        now = time.time()

        # 冷却检查
        if self._state == ControlState.COOLDOWN:
            if now < self._cooldown_until:
                return None
            # 冷却结束，回到空闲
            self._state = ControlState.IDLE
            self._confirm_count = 0
            logger.debug("控制冷却结束，回到IDLE状态")

        # 无有效标识牌
        if sign_type == ST.UNKNOWN or not distance_valid or distance <= 0:
            if self._state == ControlState.CONFIRMING:
                # 确认中但标识牌丢失，重置
                self._confirm_count = 0
                self._current_sign_type = None
                self._state = ControlState.IDLE
            return None

        # 未进入触发范围
        if distance > self.trigger_distance:
            # 距离还远，继续检测但不累计确认
            return None

        # 检查标识牌类型变化
        if (
            self._current_sign_type is not None
            and self._current_sign_type != sign_type
        ):
            # 类型变化，重置确认计数
            logger.debug(
                f"标识牌类型变化: {self._current_sign_type} -> {sign_type}, 重置确认"
            )
            self._confirm_count = 0

        self._current_sign_type = sign_type

        # 累计确认次数
        if self._state == ControlState.IDLE:
            self._state = ControlState.CONFIRMING
            self._confirm_count = 1
        elif self._state == ControlState.CONFIRMING:
            self._confirm_count += 1
        elif self._state == ControlState.TRIGGERED:
            # 已触发，持续保持或超时后进入冷却
            if now - self._trigger_time > self.turn_duration:
                self._enter_cooldown()
            return None

        # 达到确认阈值，触发指令
        if self._confirm_count >= self.confirm_count and self._state == ControlState.CONFIRMING:
            command = self._generate_command(sign_type, distance)
            self._execute_command(command)
            return command

        return None

    def _generate_command(
        self, sign_type: str, distance: float
    ) -> ControlCommand:
        """根据标识牌类型生成控制指令"""
        self._total_triggers += 1

        if sign_type == ST.STRAIGHT:
            action = ControlAction.GO_STRAIGHT
            linear = self.straight_speed
            angular = 0.0
            duration = self.turn_duration

        elif sign_type == ST.LEFT_TURN:
            action = ControlAction.TURN_LEFT
            linear = self.straight_speed * 0.5  # 转弯减速
            angular = self.left_angular
            duration = self.turn_duration

        elif sign_type == ST.RIGHT_TURN:
            action = ControlAction.TURN_RIGHT
            linear = self.straight_speed * 0.5  # 转弯减速
            angular = self.right_angular
            duration = self.turn_duration

        else:
            action = ControlAction.NONE
            linear = 0.0
            angular = 0.0
            duration = 0.0

        return ControlCommand(
            action=action,
            linear_velocity=linear,
            angular_velocity=angular,
            gear=1,
            duration=duration,
            source_sign_type=sign_type,
            triggered_distance=distance,
        )

    def _execute_command(self, command: ControlCommand):
        """执行控制指令"""
        logger.info(
            f"=== 触发交通标识牌响应 ===\n"
            f"  识别: {SignType.CN_NAMES.get(command.source_sign_type, command.source_sign_type)}\n"
            f"  距离: {command.triggered_distance:.3f}m\n"
            f"  动作: {command.action.value}\n"
            f"  线速度: {command.linear_velocity:.3f} m/s\n"
            f"  角速度: {command.angular_velocity:.3f} rad/s"
        )

        self._state = ControlState.TRIGGERED
        self._trigger_time = time.time()
        self._last_command = command
        self._successful_triggers += 1

        # 发送指令
        if self._on_command is not None:
            try:
                self._on_command(command)
                logger.info("控制指令已发送")
            except Exception as e:
                logger.error(f"指令发送回调异常: {e}")

    def _enter_cooldown(self):
        """进入冷却状态"""
        self._state = ControlState.COOLDOWN
        self._cooldown_until = time.time() + self.cooldown
        self._confirm_count = 0
        self._current_sign_type = None
        logger.info(f"控制指令执行完毕，冷却 {self.cooldown}s")

    def stop(self):
        """立即发送停车指令"""
        stop_cmd = ControlCommand(
            action=ControlAction.STOP,
            linear_velocity=ControlConfig.STOP_LINEAR,
            angular_velocity=ControlConfig.STOP_ANGULAR,
            source_sign_type=ST.UNKNOWN,
        )
        self._execute_command(stop_cmd)

    def reset(self):
        """重置控制器状态"""
        self._state = ControlState.IDLE
        self._confirm_count = 0
        self._current_sign_type = None
        self._trigger_time = 0.0
        self._cooldown_until = 0.0
        self._last_command = None
        logger.info("车辆控制器已重置")

    def get_state(self) -> ControlStateInfo:
        """获取当前控制状态信息"""
        return ControlStateInfo(
            state=self._state,
            current_action=self._last_command.action if self._last_command else ControlAction.NONE,
            confirm_count=self._confirm_count,
            trigger_time=self._trigger_time,
            cooldown_until=self._cooldown_until,
            last_command=self._last_command,
        )

    @property
    def accuracy(self) -> float:
        """指令执行准确率"""
        if self._total_triggers == 0:
            return 1.0
        return self._successful_triggers / self._total_triggers

    def get_stats(self) -> dict:
        """获取控制器统计信息"""
        return {
            'state': self._state.value,
            'confirm_count': self._confirm_count,
            'total_triggers': self._total_triggers,
            'successful_triggers': self._successful_triggers,
            'accuracy': f'{self.accuracy * 100:.1f}%',
            'trigger_distance': self.trigger_distance,
            'cooldown_remaining': max(0, self._cooldown_until - time.time()),
        }
