#!/bin/bash

# 打印选项
echo "请选择一个车型："
echo "A: DT-mid"
echo "B: DT-mini"
echo "C: FR-mid"
echo "D: FR-mini"
echo "E: FW-mid"
echo "F: MK-mid"
echo "G: TK-mid"

# 读取用户输入
read -p "输入选项，大小写均可: " option

# 检查用户输入并执行相应命令
case "$option" in
    A|a)
        # 执行命令A
        echo "选择DT-mid"
        # 在这里添加命令A的具体内容
        cp /home/yhs/ros2_ws/src/yhs_chassis_description/launch/DT-mid.launch.py  /home/yhs/ros2_ws/src/yhs_chassis_description/launch/robot.launch.py
        ;;
    B|b)
        # 执行命令B
        echo "选择DT-mini"
        # 在这里添加命令B的具体内容
        cp /home/yhs/ros2_ws/src/yhs_chassis_description/launch/DT-mini.launch.py  /home/yhs/ros2_ws/src/yhs_chassis_description/launch/robot.launch.py
        ;;
    C|c)
        # 执行命令C
        echo "选择FR-mid"
        # 在这里添加命令C的具体内容
        cp /home/yhs/ros2_ws/src/yhs_chassis_description/launch/FR-mid.launch.py  /home/yhs/ros2_ws/src/yhs_chassis_description/launch/robot.launch.py
        ;;
    D|d)
        # 执行命令D
        echo "选择FR-mini"
        # 在这里添加命令D的具体内容
        cp /home/yhs/ros2_ws/src/yhs_chassis_description/launch/DT-mini.launch.py  /home/yhs/ros2_ws/src/yhs_chassis_description/launch/robot.launch.py
        ;;
    E|e)
        # 执行命令D
        echo "选择FW-mid"
        # 在这里添加命令D的具体内容
        cp /home/yhs/ros2_ws/src/yhs_chassis_description/launch/FW-mid.launch.py  /home/yhs/ros2_ws/src/yhs_chassis_description/launch/robot.launch.py
        ;;
    F|f)
        # 执行命令D
        echo "选择MK-mid"
        # 在这里添加命令D的具体内容
        cp /home/yhs/ros2_ws/src/yhs_chassis_description/launch/MK-mid.launch.py  /home/yhs/ros2_ws/src/yhs_chassis_description/launch/robot.launch.py
        ;;
    G|g)
        # 执行命令D
        echo "选择TK-mid"
        # 在这里添加命令D的具体内容
        cp /home/yhs/ros2_ws/src/yhs_chassis_description/launch/TK-mid.launch.py  /home/yhs/ros2_ws/src/yhs_chassis_description/launch/robot.launch.py
        ;;
    *)
        # 如果输入选项不正确，则提示错误信息
        echo "错误：无效选项，请选择 A、B、C、D、E、F 或 G."
        ;;
esac





















