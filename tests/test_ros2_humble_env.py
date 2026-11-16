"""票 S-RFLYSIM：Ubuntu 22.04 Humble（CPython 3.10）与 Python 3.12 .venv 的 ROS 2 桥接与兼容契约。"""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from yopo_l0 import ros2_env
from yopo_l0.ros2_env import _abi_compatible, ensure_ros_paths

from conftest import ROOT
from test_ros2_form import needs_ros


def test_abi_compatible_filters_mismatched_python_minor():
    assert _abi_compatible("/opt/ros/humble/local/lib/python3.10/dist-packages", "3.10") is True
    assert _abi_compatible("/opt/ros/humble/local/lib/python3.10/dist-packages", "3.12") is False
    assert _abi_compatible("/opt/ros/jazzy/lib/python3.12/site-packages", "3.12") is True
    assert _abi_compatible("/opt/ros/humble/lib/site-packages", "3.12") is True


def test_venv_python312_keeps_pinned_numpy_unpolluted_after_ensure_ros_paths():
    distro = ensure_ros_paths()
    if distro == "humble" and sys.version_info[:2] != (3, 10):
        assert all("python3.10" not in p for p in sys.path)
        assert "python3.10" not in os.environ.get("PYTHONPATH", "")
    assert np.__version__.startswith("2.")


def test_ensure_ros_paths_preserves_caller_discovery_env(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "77")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    monkeypatch.setattr(ros2_env, "_CACHED", False)
    ensure_ros_paths()
    assert os.environ["ROS_DOMAIN_ID"] == "77"
    assert os.environ["ROS_LOCALHOST_ONLY"] == "1"
    assert os.environ["RMW_IMPLEMENTATION"] == "rmw_fastrtps_cpp"


def test_humble_defaults_to_cyclonedds_when_installed(monkeypatch):
    if not Path("/opt/ros/humble/lib/librmw_cyclonedds_cpp.so").is_file():
        pytest.skip("仅在安装了 rmw_cyclonedds_cpp 的 Humble 主机验证默认 RMW")
    monkeypatch.delenv("RMW_IMPLEMENTATION", raising=False)
    monkeypatch.setattr(ros2_env, "_CACHED", False)
    assert ensure_ros_paths() == "humble"
    assert os.environ["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"


@needs_ros
def test_rclpy_repeated_init_shutdown_and_humble_compat_apis():
    ensure_ros_paths()
    import rclpy
    from rclpy.clock_type import ClockType
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String

    for cycle in (1, 2):
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        assert rclpy.ok() is True
        exe = None
        node = None
        try:
            use_sim = Parameter("use_sim_time", Parameter.Type.BOOL, True)
            node = Node(
                "compat_probe",
                namespace=f"probe{cycle}",
                parameter_overrides=[use_sim],
                enable_rosout=False,
            )
            assert node.get_fully_qualified_name() == f"/probe{cycle}/compat_probe"
            assert node.get_clock().clock_type == ClockType.ROS_TIME

            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=8,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            received: list[str] = []
            pub = node.create_publisher(String, "ping", qos)
            sub = node.create_subscription(String, "ping", lambda m: received.append(m.data), qos)
            clock_pub = node.create_publisher(
                Clock, "/clock", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            )
            exe = SingleThreadedExecutor()
            exe.add_node(node)

            for _ in range(40):
                exe.spin_once(timeout_sec=0.02)
                if pub.get_subscription_count() >= 1 and sub.get_publisher_count() >= 1:
                    break
            assert pub.get_subscription_count() >= 1
            assert sub.get_publisher_count() >= 1

            cmsg = Clock()
            cmsg.clock.sec = cycle
            cmsg.clock.nanosec = 250_000_000
            for _ in range(10):
                clock_pub.publish(cmsg)
                exe.spin_once(timeout_sec=0.02)
                if node.get_clock().now().nanoseconds == cycle * 1_000_000_000 + 250_000_000:
                    break
            assert node.get_clock().now().nanoseconds == cycle * 1_000_000_000 + 250_000_000

            pub.publish(String(data=f"hello-{cycle}"))
            for _ in range(20):
                exe.spin_once(timeout_sec=0.02)
                if received:
                    break
            assert received == [f"hello-{cycle}"]
        finally:
            if exe is not None:
                exe.shutdown()
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()
        assert rclpy.ok() is False


@needs_ros
def test_native_python310_humble_compat_when_present():
    if not Path("/opt/ros/humble/setup.bash").is_file() or not Path("/usr/bin/python3.10").is_file():
        pytest.skip("仅在安装了 /opt/ros/humble 与 /usr/bin/python3.10 的主机验证原生 3.10 路径")
    env = os.environ.copy()
    env["PYTHONPATH"] = f"{ROOT}:{ROOT / 'core' / 'src'}"
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["RMW_FASTRTPS_USE_SHM"] = "0"
    proc = subprocess.run(
        [
            "/usr/bin/python3.10",
            "-c",
            (
                "from yopo_l0.ros2_env import ensure_ros_paths; "
                "assert ensure_ros_paths() == 'humble'; "
                "import rclpy; from rclpy.clock_type import ClockType; "
                "from rclpy.node import Node; from std_msgs.msg import String; "
                "rclpy.init(); n = Node('py310_native', enable_rosout=False); "
                "p = n.create_publisher(String, 't', 4); "
                "s = n.create_subscription(String, 't', lambda m: None, 4); "
                "assert s.get_publisher_count() >= 1 and ClockType.ROS_TIME is not None; "
                "n.destroy_node(); rclpy.shutdown()"
            ),
        ],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr + proc.stdout
