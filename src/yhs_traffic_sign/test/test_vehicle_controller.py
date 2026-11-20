"""车辆控制触发模块单元测试

覆盖场景:
- 状态机流转 (IDLE -> CONFIRMING -> TRIGGERED -> COOLDOWN -> IDLE)
- 触发条件判断 (距离+连续确认)
- 控制指令生成 (左转/直行/右转)
- 冷却时间机制
- 准确率统计
- 紧急停车
"""

import time
import pytest

from yhs_traffic_sign.vehicle_controller import (
    VehicleController,
    ControlCommand,
    ControlState,
    ControlAction,
)
from yhs_traffic_sign.config import SignType, ControlConfig


class TestVehicleController:
    """车辆控制器测试"""

    def setup_method(self):
        """每个测试前创建控制器"""
        self.commands_received = []
        self.controller = VehicleController(
            trigger_distance=0.5,
            confirm_count=3,
            cooldown=2.0,  # 短冷却便于测试
            on_command_callback=self.commands_received.append,
        )

    # ==================== 状态机测试 ====================
    def test_initial_state(self):
        """测试初始状态"""
        state = self.controller.get_state()
        assert state.state == ControlState.IDLE
        assert state.confirm_count == 0

    def test_state_transition_idle_to_confirming(self):
        """测试 IDLE -> CONFIRMING 转换"""
        # 距离在触发范围内
        self.controller.process(SignType.STRAIGHT, 0.3, True)
        state = self.controller.get_state()
        assert state.state == ControlState.CONFIRMING
        assert state.confirm_count == 1

    def test_state_transition_to_triggered(self):
        """测试 CONFIRMING -> TRIGGERED 转换 (达到确认次数)"""
        for _ in range(3):
            self.controller.process(SignType.STRAIGHT, 0.3, True)

        state = self.controller.get_state()
        assert state.state == ControlState.TRIGGERED
        assert len(self.commands_received) == 1

    def test_state_transition_to_cooldown(self):
        """测试 TRIGGERED -> COOLDOWN 转换"""
        # 先触发
        for _ in range(3):
            self.controller.process(SignType.LEFT_TURN, 0.3, True)

        # 等待执行时间 + 一点余量
        time.sleep(ControlConfig.TURN_DURATION + 0.1)

        # 处理一次让状态转换
        self.controller.process(SignType.LEFT_TURN, 0.3, True)

        state = self.controller.get_state()
        assert state.state == ControlState.COOLDOWN

    def test_cooldown_to_idle(self):
        """测试 COOLDOWN -> IDLE 转换"""
        # 触发并进入冷却
        for _ in range(3):
            self.controller.process(SignType.STRAIGHT, 0.3, True)
        time.sleep(ControlConfig.TURN_DURATION + 0.1)
        self.controller.process(SignType.STRAIGHT, 0.3, True)

        # 等待冷却结束 (cooldown=2.0)
        time.sleep(2.1)

        # 用UNKNOWN让状态机检查冷却过期，但不触发新确认
        self.controller.process(SignType.UNKNOWN, 0.0, False)
        state = self.controller.get_state()
        assert state.state == ControlState.IDLE

    # ==================== 触发条件测试 ====================
    def test_no_trigger_beyond_distance(self):
        """测试距离太远不触发"""
        # 距离超过阈值
        for _ in range(10):
            self.controller.process(SignType.STRAIGHT, 2.0, True)

        assert len(self.commands_received) == 0
        state = self.controller.get_state()
        # 可能进入CONFIRMING但不会TRIGGER
        assert state.state != ControlState.TRIGGERED

    def test_no_trigger_without_confirm(self):
        """测试未达确认次数不触发"""
        # 只确认2次
        for _ in range(2):
            self.controller.process(SignType.STRAIGHT, 0.3, True)

        assert len(self.commands_received) == 0
        state = self.controller.get_state()
        assert state.state == ControlState.CONFIRMING
        assert state.confirm_count == 2

    def test_confirm_reset_on_unknown(self):
        """测试标识牌丢失时确认重置"""
        # 开始确认
        self.controller.process(SignType.STRAIGHT, 0.3, True)
        self.controller.process(SignType.STRAIGHT, 0.3, True)
        assert self.controller.get_state().confirm_count == 2

        # 标识牌丢失
        self.controller.process(SignType.UNKNOWN, 0.3, False)
        assert self.controller.get_state().confirm_count == 0
        assert self.controller.get_state().state == ControlState.IDLE

    def test_confirm_reset_on_type_change(self):
        """测试标识牌类型变化时确认重置"""
        self.controller.process(SignType.STRAIGHT, 0.3, True)
        self.controller.process(SignType.STRAIGHT, 0.3, True)
        assert self.controller.get_state().confirm_count == 2

        # 类型变化 -> 重置
        self.controller.process(SignType.LEFT_TURN, 0.3, True)
        assert self.controller.get_state().confirm_count == 1  # 新计数开始

    # ==================== 指令生成测试 ====================
    def test_straight_command(self):
        """测试直行指令"""
        for _ in range(3):
            self.controller.process(SignType.STRAIGHT, 0.3, True)

        cmd = self.commands_received[0]
        assert cmd.action == ControlAction.GO_STRAIGHT
        assert cmd.linear_velocity == pytest.approx(ControlConfig.STRAIGHT_SPEED, abs=0.01)
        assert cmd.angular_velocity == 0.0
        assert cmd.source_sign_type == SignType.STRAIGHT

    def test_left_turn_command(self):
        """测试左转指令"""
        for _ in range(3):
            self.controller.process(SignType.LEFT_TURN, 0.3, True)

        cmd = self.commands_received[0]
        assert cmd.action == ControlAction.TURN_LEFT
        assert cmd.angular_velocity > 0  # 左转角速度为正
        assert cmd.source_sign_type == SignType.LEFT_TURN

    def test_right_turn_command(self):
        """测试右转指令"""
        for _ in range(3):
            self.controller.process(SignType.RIGHT_TURN, 0.3, True)

        cmd = self.commands_received[0]
        assert cmd.action == ControlAction.TURN_RIGHT
        assert cmd.angular_velocity < 0  # 右转角速度为负
        assert cmd.source_sign_type == SignType.RIGHT_TURN

    def test_command_validity(self):
        """测试指令有效性"""
        cmd = self.commands_received[0] if self.commands_received else None
        if cmd is None:
            # 先触发一次
            for _ in range(3):
                self.controller.process(SignType.STRAIGHT, 0.3, True)
            cmd = self.commands_received[0]

        assert cmd.is_valid is True
        assert cmd.timestamp > 0
        assert cmd.triggered_distance == pytest.approx(0.3, abs=0.001)

    # ==================== 冷却机制测试 ====================
    def test_cooldown_prevents_repeat(self):
        """测试冷却防止重复触发"""
        # 第一次触发
        for _ in range(3):
            self.controller.process(SignType.STRAIGHT, 0.3, True)

        time.sleep(ControlConfig.TURN_DURATION + 0.1)

        # 进入冷却
        self.controller.process(SignType.STRAIGHT, 0.3, True)
        cmd_count_after_first = len(self.commands_received)

        # 在冷却期间继续喂数据
        for _ in range(10):
            self.controller.process(SignType.STRAIGHT, 0.3, True)

        # 不应产生新指令
        assert len(self.commands_received) == cmd_count_after_first

    # ==================== 准确率测试 ====================
    def test_accuracy_initial(self):
        """测试初始准确率"""
        assert self.controller.accuracy == 1.0

    def test_stats(self):
        """测试统计信息"""
        for _ in range(3):
            self.controller.process(SignType.STRAIGHT, 0.3, True)

        stats = self.controller.get_stats()
        assert stats['total_triggers'] == 1
        assert stats['successful_triggers'] == 1
        assert stats['state'] in ['triggered', 'cooldown', 'idle', 'confirming']

    # ==================== 停车测试 ====================
    def test_stop_command(self):
        """测试紧急停车"""
        self.controller.stop()
        cmd = self.commands_received[-1]
        assert cmd.action == ControlAction.STOP
        assert cmd.linear_velocity == 0.0
        assert cmd.angular_velocity == 0.0

    # ==================== 重置测试 ====================
    def test_reset(self):
        """测试重置"""
        for _ in range(3):
            self.controller.process(SignType.STRAIGHT, 0.3, True)

        self.controller.reset()
        state = self.controller.get_state()
        assert state.state == ControlState.IDLE
        assert state.confirm_count == 0


class TestControlCommand:
    """控制指令数据类测试"""

    def test_to_dict(self):
        """测试字典转换"""
        cmd = ControlCommand(
            action=ControlAction.GO_STRAIGHT,
            linear_velocity=0.3,
            angular_velocity=0.0,
            source_sign_type=SignType.STRAIGHT,
            triggered_distance=0.45,
        )
        d = cmd.to_dict()
        assert 'action' in d
        assert 'linear' in d
        assert 'angular' in d
        assert 'source' in d

    def test_is_valid(self):
        """测试有效性"""
        valid_cmd = ControlCommand(
            action=ControlAction.GO_STRAIGHT,
            linear_velocity=0.3,
            angular_velocity=0.0,
        )
        assert valid_cmd.is_valid is True

        invalid_cmd = ControlCommand(
            action=ControlAction.NONE,
            linear_velocity=0.0,
            angular_velocity=0.0,
        )
        assert invalid_cmd.is_valid is False
