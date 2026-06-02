# simulation_pkg/launch/rviz.launch.py
"""
@file rviz.launch.py
@brief RViz2 可视化启动文件 — 加载预配置的 RViz 布局文件。

该 launch 文件非常精简，仅启动 RViz2 并加载 bringup_pkg 中的
预配置文件。将 RViz 配置放在 bringup_pkg 中而非 simulation_pkg
中的原因是：无论是仿真还是真机部署，都使用同一套可视化布局。

@note 需要 bringup_pkg 包中存在 rviz/fcr_system.rviz 配置文件。
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── 定位 bringup_pkg 共享目录（跨包引用 RViz 配置） ─────────
    bringup_share = FindPackageShare("bringup_pkg")

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            # -d 指定预保存的 RViz 配置文件，包含面板布局和显示项
            arguments=["-d", PathJoinSubstitution([bringup_share, "rviz", "fcr_system.rviz"])],
            parameters=[{"use_sim_time": True}],
        ),
    ])
