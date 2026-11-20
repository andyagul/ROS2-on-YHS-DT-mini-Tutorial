"""配置常量模块

集中定义交通标识牌识别与响应功能中的所有配置常量，
便于统一管理和动态调参。
"""

# ============================================================
# 标识牌类型定义
# ============================================================
class SignType:
    """交通标识牌类型枚举"""
    LEFT_TURN = 'left_turn'       # 左转
    STRAIGHT = 'straight'         # 直行
    RIGHT_TURN = 'right_turn'     # 右转
    UNKNOWN = 'unknown'

    # 类别ID映射（需与训练模型保持一致）
    CLASS_IDS = {
        0: LEFT_TURN,
        1: STRAIGHT,
        2: RIGHT_TURN,
    }

    # 中文名称映射
    CN_NAMES = {
        LEFT_TURN: '左转',
        STRAIGHT: '直行',
        RIGHT_TURN: '右转',
        UNKNOWN: '未知',
    }


# ============================================================
# 检测参数
# ============================================================
class DetectionConfig:
    """目标检测相关配置"""
    # YOLO模型路径
    MODEL_PATH = 'models/traffic_sign_yolov8n.pt'

    # 推理设备 ('cuda', 'cpu', '0' 等)
    DEVICE = 'cpu'

    # 输入图像尺寸
    IMG_SIZE = 640

    # 置信度阈值 (低于此值的检测结果被过滤)
    CONFIDENCE_THRESHOLD = 0.5

    # NMS IoU阈值
    NMS_IOU_THRESHOLD = 0.45

    # 检测帧间隔 (多少帧处理一次，用于降低CPU负载)
    DETECTION_FRAME_INTERVAL = 1

    # 推理超时时间(秒)
    INFERENCE_TIMEOUT = 0.05  # 50ms，确保总体延迟 < 100ms


# ============================================================
# 距离感知参数
# ============================================================
class DistanceConfig:
    """距离感知相关配置"""
    # 标识牌实际物理尺寸(米) - 标准交通指示牌尺寸
    SIGN_REAL_WIDTH = 0.6       # 标识牌实际宽度 60cm
    SIGN_REAL_HEIGHT = 0.6      # 标识牌实际高度 60cm

    # 相机内参 (需根据实际相机标定结果更新)
    # Orbbec Astra 系列典型参数
    CAMERA_FX = 525.0           # 焦距 x 方向 (像素)
    CAMERA_FY = 525.0           # 焦距 y 方向 (像素)
    CAMERA_CX = 319.5           # 主点 x (像素)
    CAMERA_CY = 239.5           # 主点 y (像素)

    # 深度相机话题
    DEPTH_TOPIC = '/camera/depth/image_raw'
    COLOR_TOPIC = '/camera/color/image_raw'
    CAMERA_INFO_TOPIC = '/camera/color/camera_info'

    # 测距方法 ('depth' 使用深度相机, 'monocular' 使用单目几何)
    DISTANCE_METHOD = 'depth'

    # 距离检测最大有效范围(米)
    MAX_VALID_DISTANCE = 15.0

    # 距离检测最小有效范围(米)
    MIN_VALID_DISTANCE = 0.1

    # 距离移动平均窗口大小
    DISTANCE_AVERAGE_WINDOW = 5

    # 距离跳变检测阈值(米) - 相邻两帧距离差超过此值视为跳变
    DISTANCE_JUMP_THRESHOLD = 0.3

    # 距离检测误差允许范围(米)
    DISTANCE_TOLERANCE = 0.05


# ============================================================
# 车辆控制参数
# ============================================================
class ControlConfig:
    """车辆控制触发相关配置"""
    # 触发控制的距离阈值(米)
    TRIGGER_DISTANCE = 0.5

    # 控制指令执行确认次数 - 连续N次检测到再触发
    TRIGGER_CONFIRM_COUNT = 3

    # 指令冷却时间(秒) - 防止重复触发
    CONTROL_COOLDOWN = 5.0

    # 直行速度 (m/s)
    STRAIGHT_SPEED = 0.3

    # 左转角速度 (rad/s) - 逆时针为正
    LEFT_TURN_ANGULAR_VELOCITY = 0.8

    # 右转角速度 (rad/s) - 顺时针为负
    RIGHT_TURN_ANGULAR_VELOCITY = -0.8

    # 转向持续时间(秒)
    TURN_DURATION = 1.5

    # 停止速度
    STOP_LINEAR = 0.0
    STOP_ANGULAR = 0.0


# ============================================================
# 异常处理参数
# ============================================================
class AnomalyConfig:
    """异常处理相关配置"""
    # 识别置信度不足阈值
    LOW_CONFIDENCE_THRESHOLD = 0.4

    # 连续低置信度容忍帧数
    LOW_CONFIDENCE_TOLERANCE = 5

    # 重复识别去重时间窗口(秒)
    DEDUP_TIME_WINDOW = 3.0

    # 标识牌丢失后保持时间(秒) - 短暂丢失不立即停止
    SIGN_LOST_HOLD_TIME = 0.5

    # 最大连续丢失帧数
    MAX_LOST_FRAMES = 10

    # 距离跳变恢复窗口 - 跳变后多少帧内继续使用上一次有效值
    JUMP_RECOVERY_FRAMES = 3


# ============================================================
# ROS话题名称
# ============================================================
class TopicNames:
    """ROS话题名称定义"""
    # 订阅话题
    COLOR_IMAGE = '/camera/color/image_raw'
    DEPTH_IMAGE = '/camera/depth/image_raw'
    CAMERA_INFO = '/camera/color/camera_info'
    CHASSIS_INFO = '/chassis_info'

    # 发布话题
    DETECTION_RESULT = '/traffic_sign/detection_result'
    DISTANCE_RESULT = '/traffic_sign/distance_result'
    CONTROL_CMD = '/ctrl_cmd'                       # yhs_can_control使用
    VISUALIZATION_MARKER = '/traffic_sign/marker'
    DIAGNOSTIC_STATUS = '/traffic_sign/status'
