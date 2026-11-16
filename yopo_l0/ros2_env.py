"""让当前解释器找得到已安装的 ROS 2，而不要求调用方先 source setup.bash。

`scripts/setup_ros2.sh` 把 `/opt/ros/<distro>/lib` 登记进 ldconfig。
- 当发行版 Python ABI 与当前解释器一致（如 Ubuntu 24.04 Jazzy + Python 3.12，
  或 Ubuntu 22.04 Humble + 系统 Python 3.10）时，直接把 ROS site-packages
  放进 `sys.path` 并在 Humble 上补齐 `rclpy.clock_type` 与
  `Subscription.get_publisher_count` 兼容层。
- 当 `.venv` 为 Python 3.12（锁死 numpy 2.5.4 / torch 2.4.1 金样）而本机安装的是
  Ubuntu 22.04 Humble（`rclpy` C 扩展绑定系统 CPython 3.10）时，自动通过本机
  CPython 3.10 子进程承载真实 `/opt/ros/humble` `rclpy` 节点、`/clock` 与话题收发，
  规划与动力学留在 `.venv` Python 3.12 进程内逐位确定性执行。
"""

from __future__ import annotations

import atexit
from enum import IntEnum
import importlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import types

_CACHED: str | None | bool = False  # False = 还没找过

_KEEP_PREFIXES = ("ROS_", "AMENT_", "COLCON_", "CMAKE_", "RMW_")
_PRESERVE_IF_SET = {
    "ROS_DOMAIN_ID",
    "ROS_LOCALHOST_ONLY",
    "ROS_AUTOMATIC_DISCOVERY_RANGE",
    "RMW_IMPLEMENTATION",
    "RMW_FASTRTPS_USE_SHM",
    "RCUTILS_LOGGING_SEVERITY",
}
# PATH 不能整段换成 setup.bash 的值，否则子进程会丢 .venv。PYTHONPATH 单独拼。
_SKIP_APPLY = {"PATH", "PYTHONPATH", "PWD", "OLDPWD", "SHLVL", "_"}
_PY_VER_RE = re.compile(r"/python(\d+\.\d+)(?:/|$)")


def _installed(*names: str) -> str | None:
    forced = os.environ.get("ROS_DISTRO")
    ordered = ([forced] if forced else []) + [n for n in names if n != forced]
    for name in ordered:
        if name and (Path("/opt/ros") / name / "setup.bash").is_file():
            return name
    return None


def _source_env(setup: Path) -> dict[str, str]:
    script = f"set +u; source '{setup}'; env -0"
    out = subprocess.check_output(["bash", "-c", script])
    env: dict[str, str] = {}
    for item in out.split(b"\0"):
        if not item or b"=" not in item:
            continue
        key, value = item.split(b"=", 1)
        name = key.decode(errors="replace")
        if not name.isidentifier():
            continue
        env[name] = value.decode(errors="replace")
    return env


def _abi_compatible(path: str, current_minor: str) -> bool:
    match = _PY_VER_RE.search(path)
    return match is None or match.group(1) == current_minor


def _apply_ros_env(distro: str, env: dict[str, str]) -> None:
    for key, value in env.items():
        if key in _SKIP_APPLY:
            continue
        if key in _PRESERVE_IF_SET and key in os.environ:
            continue
        if key == "LD_LIBRARY_PATH":
            ros_ld = [p for p in value.split(":") if p and f"/opt/ros/{distro}" in p]
            cur_ld = [p for p in os.environ.get("LD_LIBRARY_PATH", "").split(":") if p]
            merged = ros_ld + [p for p in cur_ld if p not in ros_ld]
            if merged:
                os.environ["LD_LIBRARY_PATH"] = ":".join(merged)
            continue
        if key.startswith(_KEEP_PREFIXES) or f"/opt/ros/{distro}" in value:
            os.environ[key] = value
    # Humble 的 FastDDS 2.6 在高并发 CPU 争用下同进程突发分片偶发丢首包；本机已装 CycloneDDS 时默认优先选用。
    if (
        distro == "humble"
        and "RMW_IMPLEMENTATION" not in os.environ
        and Path("/opt/ros/humble/lib/librmw_cyclonedds_cpp.so").is_file()
    ):
        os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"


def _patch_native_humble() -> None:
    """补齐 Humble rclpy 相对 Jazzy 缺少的两个薄接口（不改话题与时钟语义）。"""
    import rclpy.clock
    from rclpy.node import Node
    from rclpy.subscription import Subscription

    if "rclpy.clock_type" not in sys.modules:
        try:
            import rclpy.clock_type  # noqa: F401
        except ModuleNotFoundError:
            ct_mod = types.ModuleType("rclpy.clock_type")
            ct_mod.ClockType = rclpy.clock.ClockType  # type: ignore[attr-defined]
            sys.modules["rclpy.clock_type"] = ct_mod

    if not hasattr(Subscription, "get_publisher_count"):
        orig_create_sub = Node.create_subscription

        def _create_sub_with_owner(self, msg_type, topic, callback, qos_profile, *args, **kwargs):
            sub = orig_create_sub(self, msg_type, topic, callback, qos_profile, *args, **kwargs)
            sub._yopo_owner_node = self
            return sub

        def _get_publisher_count(self) -> int:
            owner = getattr(self, "_yopo_owner_node", None)
            if owner is None:
                return 0
            return int(owner.count_publishers(self.topic_name))

        Node.create_subscription = _create_sub_with_owner  # type: ignore[assignment]
        Subscription.get_publisher_count = _get_publisher_count  # type: ignore[attr-defined]


_WORKER_SCRIPT = r"""
import json
import os
import struct
import sys

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def _read_exact(fd: int, n: int) -> bytes | None:
    buf = bytearray()
    while len(buf) < n:
        chunk = os.read(fd, n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def _recv(fd: int) -> dict | None:
    hdr = _read_exact(fd, 4)
    if hdr is None:
        return None
    (size,) = struct.unpack("<I", hdr)
    body = _read_exact(fd, size)
    if body is None:
        return None
    return json.loads(body.decode("utf-8"))


def _send(fd: int, payload: dict) -> None:
    raw = json.dumps(payload, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    pkt = struct.pack("<I", len(raw)) + raw
    view = memoryview(pkt)
    sent = 0
    while sent < len(pkt):
        sent += os.write(fd, view[sent:])


def _make_qos(spec: dict) -> QoSProfile:
    return QoSProfile(
        history=getattr(HistoryPolicy, spec["history"]),
        depth=int(spec["depth"]),
        reliability=getattr(ReliabilityPolicy, spec["reliability"]),
        durability=getattr(DurabilityPolicy, spec["durability"]),
    )


def _main() -> None:
    r_fd = int(sys.argv[1])
    w_fd = int(sys.argv[2])
    seq = 0
    nodes: dict[int, Node] = {}
    pubs: dict[int, tuple[object, str, int]] = {}
    subs: dict[int, tuple[object, Node, int]] = {}
    executors: dict[int, SingleThreadedExecutor] = {}
    pending_events: list[list] = []

    def next_id() -> int:
        nonlocal seq
        seq += 1
        return seq

    def cleanup_all() -> None:
        for exe in list(executors.values()):
            try:
                exe.shutdown()
            except Exception:
                pass
        executors.clear()
        pubs.clear()
        subs.clear()
        for node in list(nodes.values()):
            try:
                node.destroy_node()
            except Exception:
                pass
        nodes.clear()
        pending_events.clear()

    try:
        while True:
            req = _recv(r_fd)
            if req is None:
                break
            cmd = req.get("cmd")
            try:
                if cmd == "init":
                    if not rclpy.ok():
                        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
                    _send(w_fd, {"ok": True})
                elif cmd == "shutdown":
                    cleanup_all()
                    if rclpy.ok():
                        rclpy.shutdown()
                    _send(w_fd, {"ok": False})
                    break
                elif cmd == "create_node":
                    params = [
                        Parameter(p["name"], getattr(Parameter.Type, p["type"]), p["value"])
                        for p in req.get("params", [])
                    ]
                    kw = {
                        "parameter_overrides": params,
                        "enable_rosout": bool(req.get("enable_rosout", True)),
                    }
                    ns = req.get("namespace")
                    if ns:
                        kw["namespace"] = ns
                    node = Node(req["name"], **kw)
                    nid = next_id()
                    nodes[nid] = node
                    _send(w_fd, {
                        "nid": nid,
                        "fqn": node.get_fully_qualified_name(),
                        "clock_type": node.get_clock().clock_type.name,
                        "clock_ns": int(node.get_clock().now().nanoseconds),
                    })
                elif cmd == "destroy_node":
                    nid = int(req["nid"])
                    for pid in [k for k, v in pubs.items() if v[2] == nid]:
                        pubs.pop(pid, None)
                    for sid in [k for k, v in subs.items() if v[2] == nid]:
                        subs.pop(sid, None)
                    node = nodes.pop(nid, None)
                    if node is not None:
                        node.destroy_node()
                    _send(w_fd, {})
                elif cmd == "create_publisher":
                    nid = int(req["nid"])
                    node = nodes[nid]
                    qos = _make_qos(req["qos"])
                    msg_kind = req["msg_type"]
                    mcls = Clock if msg_kind == "Clock" else String
                    pub = node.create_publisher(mcls, req["topic"], qos)
                    pid = next_id()
                    pubs[pid] = (pub, msg_kind, nid)
                    _send(w_fd, {"pid": pid, "topic_name": pub.topic_name})
                elif cmd == "create_subscription":
                    nid = int(req["nid"])
                    node = nodes[nid]
                    qos = _make_qos(req["qos"])
                    msg_kind = req["msg_type"]
                    sid = next_id()
                    if msg_kind == "Clock":
                        def _cb(msg, _sid=sid):
                            pending_events.append([_sid, [int(msg.clock.sec), int(msg.clock.nanosec)]])
                        sub = node.create_subscription(Clock, req["topic"], _cb, qos)
                    else:
                        def _cb(msg, _sid=sid):
                            pending_events.append([_sid, msg.data])
                        sub = node.create_subscription(String, req["topic"], _cb, qos)
                    subs[sid] = (sub, node, nid)
                    _send(w_fd, {"sid": sid, "topic_name": sub.topic_name})
                elif cmd == "publish":
                    pub, msg_kind, _ = pubs[int(req["pid"])]
                    if msg_kind == "Clock":
                        sec, nanosec = req["data"]
                        msg = Clock()
                        msg.clock.sec = int(sec)
                        msg.clock.nanosec = int(nanosec)
                    else:
                        msg = String(data=req["data"])
                    pub.publish(msg)
                    _send(w_fd, {})
                elif cmd == "pub_sub_count":
                    pub, _, _ = pubs[int(req["pid"])]
                    _send(w_fd, {"n": int(pub.get_subscription_count())})
                elif cmd == "sub_pub_count":
                    sub, owner, _ = subs[int(req["sid"])]
                    _send(w_fd, {"n": int(owner.count_publishers(sub.topic_name))})
                elif cmd == "node_count_pubs":
                    node = nodes[int(req["nid"])]
                    _send(w_fd, {"n": int(node.count_publishers(req["topic"]))})
                elif cmd == "node_count_subs":
                    node = nodes[int(req["nid"])]
                    _send(w_fd, {"n": int(node.count_subscribers(req["topic"]))})
                elif cmd == "create_executor":
                    eid = next_id()
                    executors[eid] = SingleThreadedExecutor()
                    _send(w_fd, {"eid": eid})
                elif cmd == "executor_add_node":
                    executors[int(req["eid"])].add_node(nodes[int(req["nid"])])
                    _send(w_fd, {})
                elif cmd == "executor_shutdown":
                    exe = executors.pop(int(req["eid"]), None)
                    if exe is not None:
                        exe.shutdown()
                    _send(w_fd, {})
                elif cmd == "spin_once":
                    pending_events.clear()
                    executors[int(req["eid"])].spin_once(timeout_sec=req["timeout_sec"])
                    clocks = {
                        str(nid): int(n.get_clock().now().nanoseconds)
                        for nid, n in nodes.items()
                    }
                    ev = list(pending_events)
                    pending_events.clear()
                    _send(w_fd, {"events": ev, "clocks": clocks})
                else:
                    _send(w_fd, {"err_type": "RuntimeError", "err": f"unknown cmd {cmd!r}"})
            except Exception as exc:
                _send(w_fd, {"err_type": type(exc).__name__, "err": str(exc)})
    finally:
        cleanup_all()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    _main()
"""


def _find_host_ros_python(distro: str, ros_py: list[str], ros_env: dict[str, str]) -> tuple[str, list[str]] | None:
    target_ver = None
    for p in ros_py:
        m = _PY_VER_RE.search(p)
        if m:
            target_ver = m.group(1)
            break
    candidates: list[str] = []
    if target_ver:
        for cand in (f"/usr/bin/python{target_ver}", shutil.which(f"python{target_ver}") or ""):
            if cand and cand not in candidates:
                candidates.append(cand)
    if "/usr/bin/python3" not in candidates:
        candidates.append("/usr/bin/python3")

    py_paths = list(ros_py)
    for extra in ("/usr/lib/python3/dist-packages", f"/usr/local/lib/python{target_ver or '3'}/dist-packages"):
        if Path(extra).is_dir() and extra not in py_paths:
            py_paths.append(extra)

    probe_env = os.environ.copy()
    for k, v in ros_env.items():
        if k not in _SKIP_APPLY and (k.startswith(_KEEP_PREFIXES) or f"/opt/ros/{distro}" in v):
            probe_env[k] = v
    probe_env["PYTHONPATH"] = ":".join(py_paths)

    for exe in candidates:
        if not Path(exe).is_file() or not os.access(exe, os.X_OK):
            continue
        proc = subprocess.run(
            [exe, "-c", "import rclpy; from rosgraph_msgs.msg import Clock; from std_msgs.msg import String"],
            env=probe_env,
            capture_output=True,
            text=True,
            check=False,
        )
        if proc.returncode == 0:
            return exe, py_paths
    return None


def _install_humble_bridge(host_python: str, worker_py_paths: list[str], distro: str, ros_env: dict[str, str]) -> None:
    """在 Python 3.12 .venv 里注册对接系统 CPython 3.10 Humble 的 rclpy 模块族。"""
    existing = sys.modules.get("rclpy")
    if existing is not None and getattr(existing, "_yopo_humble_bridge", False):
        return

    class ClockType(IntEnum):
        ROS_TIME = 1
        SYSTEM_TIME = 2
        STEADY_TIME = 3

    class SignalHandlerOptions(IntEnum):
        NO = 0
        SIGINT = 1
        SIGTERM = 2
        ALL = 3

    class ReliabilityPolicy(IntEnum):
        SYSTEM_DEFAULT = 0
        RELIABLE = 1
        BEST_EFFORT = 2

    class HistoryPolicy(IntEnum):
        SYSTEM_DEFAULT = 0
        KEEP_LAST = 1
        KEEP_ALL = 2

    class DurabilityPolicy(IntEnum):
        SYSTEM_DEFAULT = 0
        TRANSIENT_LOCAL = 1
        VOLATILE = 2

    class QoSProfile:
        def __init__(
            self,
            *,
            history: HistoryPolicy = HistoryPolicy.KEEP_LAST,
            depth: int = 10,
            reliability: ReliabilityPolicy = ReliabilityPolicy.RELIABLE,
            durability: DurabilityPolicy = DurabilityPolicy.VOLATILE,
        ) -> None:
            self.history = HistoryPolicy(history)
            self.depth = int(depth)
            self.reliability = ReliabilityPolicy(reliability)
            self.durability = DurabilityPolicy(durability)

        def _wire(self) -> dict[str, object]:
            return {
                "history": self.history.name,
                "depth": int(self.depth),
                "reliability": self.reliability.name,
                "durability": self.durability.name,
            }

    class _ParameterType(IntEnum):
        NOT_SET = 0
        BOOL = 1
        INTEGER = 2
        DOUBLE = 3
        STRING = 4
        BYTE_ARRAY = 5
        BOOL_ARRAY = 6
        INTEGER_ARRAY = 7
        DOUBLE_ARRAY = 8
        STRING_ARRAY = 9

    class Parameter:
        Type = _ParameterType

        def __init__(self, name: str, type_: _ParameterType | None = None, value: object = None) -> None:
            self.name = str(name)
            self.type_ = _ParameterType.BOOL if (type_ is None and isinstance(value, bool)) else (type_ or _ParameterType.NOT_SET)
            self.value = value

    class _TimeMsg:
        __slots__ = ("sec", "nanosec")

        def __init__(self, *, sec: int = 0, nanosec: int = 0) -> None:
            self.sec = int(sec)
            self.nanosec = int(nanosec)

    class Clock:
        __slots__ = ("clock",)

        def __init__(self, *, clock: _TimeMsg | None = None) -> None:
            self.clock = clock if clock is not None else _TimeMsg()

    class String:
        __slots__ = ("data",)

        def __init__(self, *, data: str = "") -> None:
            self.data = str(data)

        def __repr__(self) -> str:
            return f"std_msgs.msg.String(data={self.data!r})"

        def __eq__(self, other: object) -> bool:
            return isinstance(other, String) and self.data == other.data

    def _read_exact(fd: int, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = os.read(fd, n - len(buf))
            if not chunk:
                raise RuntimeError("Humble rclpy 辅助进程意外退出")
            buf.extend(chunk)
        return bytes(buf)

    class _Session:
        def __init__(self) -> None:
            self.proc: subprocess.Popen | None = None
            self.w_fd: int = -1
            self.r_fd: int = -1
            self.ok: bool = False
            self.nodes: dict[int, Node] = {}
            self.subs: dict[int, tuple[object, str]] = {}

        def start(self) -> None:
            if self.ok and self.proc is not None and self.proc.poll() is None:
                return
            self.close()
            p2c_r, p2c_w = os.pipe()
            c2p_r, c2p_w = os.pipe()
            child_env = os.environ.copy()
            for k, v in ros_env.items():
                if k in _SKIP_APPLY:
                    continue
                if k in _PRESERVE_IF_SET and k in child_env:
                    continue
                if k == "LD_LIBRARY_PATH":
                    ros_ld = [p for p in v.split(":") if p and f"/opt/ros/{distro}" in p]
                    cur_ld = [p for p in child_env.get("LD_LIBRARY_PATH", "").split(":") if p]
                    merged = ros_ld + [p for p in cur_ld if p not in ros_ld]
                    if merged:
                        child_env["LD_LIBRARY_PATH"] = ":".join(merged)
                    continue
                if k.startswith(_KEEP_PREFIXES) or f"/opt/ros/{distro}" in v:
                    child_env[k] = v
            if (
                distro == "humble"
                and "RMW_IMPLEMENTATION" not in child_env
                and Path("/opt/ros/humble/lib/librmw_cyclonedds_cpp.so").is_file()
            ):
                child_env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
            child_env["PYTHONPATH"] = ":".join(worker_py_paths)
            try:
                self.proc = subprocess.Popen(
                    [host_python, "-c", _WORKER_SCRIPT, str(p2c_r), str(c2p_w)],
                    env=child_env,
                    pass_fds=(p2c_r, c2p_w),
                    stdin=subprocess.DEVNULL,
                )
            except Exception:
                os.close(p2c_r)
                os.close(p2c_w)
                os.close(c2p_r)
                os.close(c2p_w)
                raise
            os.close(p2c_r)
            os.close(c2p_w)
            self.w_fd = p2c_w
            self.r_fd = c2p_r
            self.call({"cmd": "init"})
            self.ok = True

        def call(self, req: dict[str, object]) -> dict:
            if self.proc is None or self.w_fd < 0 or self.r_fd < 0:
                raise RuntimeError("rclpy 尚未 init")
            raw = json.dumps(req, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
            pkt = struct.pack("<I", len(raw)) + raw
            view = memoryview(pkt)
            sent = 0
            while sent < len(pkt):
                sent += os.write(self.w_fd, view[sent:])
            (size,) = struct.unpack("<I", _read_exact(self.r_fd, 4))
            resp = json.loads(_read_exact(self.r_fd, size).decode("utf-8"))
            if "err_type" in resp:
                err_cls = {"ValueError": ValueError, "TypeError": TypeError, "TimeoutError": TimeoutError}.get(
                    resp["err_type"], RuntimeError
                )
                raise err_cls(resp["err"])
            return resp

        def close(self) -> None:
            proc, w_fd, r_fd = self.proc, self.w_fd, self.r_fd
            self.proc = None
            self.w_fd = -1
            self.r_fd = -1
            self.ok = False
            self.nodes.clear()
            self.subs.clear()
            if proc is not None and proc.poll() is None and w_fd >= 0 and r_fd >= 0:
                try:
                    raw = b'{"cmd":"shutdown"}'
                    os.write(w_fd, struct.pack("<I", len(raw)) + raw)
                    (size,) = struct.unpack("<I", _read_exact(r_fd, 4))
                    _read_exact(r_fd, size)
                except Exception:
                    pass
            for fd in (w_fd, r_fd):
                if fd >= 0:
                    try:
                        os.close(fd)
                    except OSError:
                        pass
            if proc is not None:
                try:
                    proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=2.0)

    session = _Session()
    atexit.register(session.close)

    class _TimePoint:
        __slots__ = ("nanoseconds",)

        def __init__(self, nanoseconds: int) -> None:
            self.nanoseconds = int(nanoseconds)

    class _NodeClock:
        def __init__(self, node: Node, clock_type: ClockType) -> None:
            self._node = node
            self.clock_type = clock_type

        def now(self) -> _TimePoint:
            return _TimePoint(self._node._clock_ns)

    class Publisher:
        def __init__(self, pid: int, topic_name: str, msg_kind: str) -> None:
            self._pid = pid
            self.topic_name = topic_name
            self._msg_kind = msg_kind

        def publish(self, msg: Clock | String) -> None:
            if self._msg_kind == "Clock":
                data: object = [int(msg.clock.sec), int(msg.clock.nanosec)]  # type: ignore[union-attr]
            else:
                data = str(msg.data)  # type: ignore[union-attr]
            session.call({"cmd": "publish", "pid": self._pid, "data": data})

        def get_subscription_count(self) -> int:
            return int(session.call({"cmd": "pub_sub_count", "pid": self._pid})["n"])

    class Subscription:
        def __init__(self, sid: int, topic_name: str) -> None:
            self._sid = sid
            self.topic_name = topic_name

        def get_publisher_count(self) -> int:
            return int(session.call({"cmd": "sub_pub_count", "sid": self._sid})["n"])

    def _norm_qos(qos_profile: QoSProfile | int) -> dict[str, object]:
        if isinstance(qos_profile, int):
            return QoSProfile(depth=qos_profile)._wire()
        return qos_profile._wire()

    def _msg_kind(msg_type: object) -> str:
        if msg_type is Clock or getattr(msg_type, "__name__", "") == "Clock":
            return "Clock"
        if msg_type is String or getattr(msg_type, "__name__", "") == "String":
            return "String"
        raise TypeError(f"不支持的 ROS 消息类型：{msg_type!r}")

    class Node:
        def __init__(
            self,
            node_name: str,
            *,
            context: object = None,
            cli_args: object = None,
            namespace: str | None = None,
            use_global_arguments: bool = True,
            enable_rosout: bool = True,
            start_parameter_services: bool = True,
            parameter_overrides: list[Parameter] | None = None,
            allow_undeclared_parameters: bool = False,
            automatically_declare_parameters_from_overrides: bool = False,
        ) -> None:
            params = [
                {"name": p.name, "type": p.type_.name, "value": p.value}
                for p in (parameter_overrides or [])
            ]
            resp = session.call({
                "cmd": "create_node",
                "name": node_name,
                "namespace": namespace,
                "params": params,
                "enable_rosout": bool(enable_rosout),
            })
            self._nid = int(resp["nid"])
            self._fqn = str(resp["fqn"])
            self._clock_ns = int(resp["clock_ns"])
            self._clock = _NodeClock(self, ClockType[str(resp["clock_type"])])
            session.nodes[self._nid] = self

        def get_fully_qualified_name(self) -> str:
            return self._fqn

        def get_clock(self) -> _NodeClock:
            return self._clock

        def create_publisher(self, msg_type: object, topic: str, qos_profile: QoSProfile | int) -> Publisher:
            kind = _msg_kind(msg_type)
            resp = session.call({
                "cmd": "create_publisher",
                "nid": self._nid,
                "msg_type": kind,
                "topic": topic,
                "qos": _norm_qos(qos_profile),
            })
            return Publisher(int(resp["pid"]), str(resp["topic_name"]), kind)

        def create_subscription(
            self, msg_type: object, topic: str, callback: object, qos_profile: QoSProfile | int
        ) -> Subscription:
            kind = _msg_kind(msg_type)
            resp = session.call({
                "cmd": "create_subscription",
                "nid": self._nid,
                "msg_type": kind,
                "topic": topic,
                "qos": _norm_qos(qos_profile),
            })
            sid = int(resp["sid"])
            session.subs[sid] = (callback, kind)
            return Subscription(sid, str(resp["topic_name"]))

        def count_publishers(self, topic_name: str) -> int:
            return int(session.call({"cmd": "node_count_pubs", "nid": self._nid, "topic": topic_name})["n"])

        def count_subscribers(self, topic_name: str) -> int:
            return int(session.call({"cmd": "node_count_subs", "nid": self._nid, "topic": topic_name})["n"])

        def destroy_node(self) -> None:
            session.nodes.pop(self._nid, None)
            if session.ok:
                session.call({"cmd": "destroy_node", "nid": self._nid})

    class SingleThreadedExecutor:
        def __init__(self, *, context: object = None) -> None:
            self._eid = int(session.call({"cmd": "create_executor"})["eid"])

        def add_node(self, node: Node) -> bool:
            session.call({"cmd": "executor_add_node", "eid": self._eid, "nid": node._nid})
            return True

        def spin_once(self, timeout_sec: float | None = None) -> None:
            resp = session.call({"cmd": "spin_once", "eid": self._eid, "timeout_sec": timeout_sec})
            for nid_str, ns in resp["clocks"].items():
                node = session.nodes.get(int(nid_str))
                if node is not None:
                    node._clock_ns = int(ns)
            for sid, data in resp["events"]:
                entry = session.subs.get(int(sid))
                if entry is None:
                    continue
                cb, kind = entry
                if kind == "Clock":
                    msg: object = Clock(clock=_TimeMsg(sec=data[0], nanosec=data[1]))
                else:
                    msg = String(data=data)
                cb(msg)  # type: ignore[operator]

        def shutdown(self, timeout_sec: float | None = None) -> bool:
            if session.ok:
                session.call({"cmd": "executor_shutdown", "eid": self._eid})
            return True

    def _init(*, args: object = None, context: object = None, domain_id: object = None, signal_handler_options: object = None) -> None:
        if domain_id is not None:
            os.environ["ROS_DOMAIN_ID"] = str(int(domain_id))  # type: ignore[arg-type]
        session.start()

    def _ok(*, context: object = None) -> bool:
        return bool(session.ok and session.proc is not None and session.proc.poll() is None)

    def _shutdown(*, context: object = None) -> None:
        session.close()

    def _try_shutdown(*, context: object = None) -> None:
        session.close()

    def _create_node(node_name: str, **kwargs: object) -> Node:
        return Node(node_name, **kwargs)  # type: ignore[arg-type]

    def _spin_once(node: Node, *, executor: SingleThreadedExecutor | None = None, timeout_sec: float | None = None) -> None:
        if executor is not None:
            executor.spin_once(timeout_sec=timeout_sec)
            return
        tmp = SingleThreadedExecutor()
        try:
            tmp.add_node(node)
            tmp.spin_once(timeout_sec=timeout_sec)
        finally:
            tmp.shutdown()

    def _pkg(name: str) -> types.ModuleType:
        mod = types.ModuleType(name)
        mod.__path__ = []  # type: ignore[attr-defined]
        return mod

    rclpy_mod = _pkg("rclpy")
    rclpy_mod._yopo_humble_bridge = True  # type: ignore[attr-defined]
    rclpy_mod.init = _init  # type: ignore[attr-defined]
    rclpy_mod.ok = _ok  # type: ignore[attr-defined]
    rclpy_mod.shutdown = _shutdown  # type: ignore[attr-defined]
    rclpy_mod.try_shutdown = _try_shutdown  # type: ignore[attr-defined]
    rclpy_mod.create_node = _create_node  # type: ignore[attr-defined]
    rclpy_mod.spin_once = _spin_once  # type: ignore[attr-defined]

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = Node  # type: ignore[attr-defined]

    exec_mod = types.ModuleType("rclpy.executors")
    exec_mod.SingleThreadedExecutor = SingleThreadedExecutor  # type: ignore[attr-defined]

    param_mod = types.ModuleType("rclpy.parameter")
    param_mod.Parameter = Parameter  # type: ignore[attr-defined]

    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.QoSProfile = QoSProfile  # type: ignore[attr-defined]
    qos_mod.ReliabilityPolicy = ReliabilityPolicy  # type: ignore[attr-defined]
    qos_mod.HistoryPolicy = HistoryPolicy  # type: ignore[attr-defined]
    qos_mod.DurabilityPolicy = DurabilityPolicy  # type: ignore[attr-defined]

    clock_mod = types.ModuleType("rclpy.clock")
    clock_mod.ClockType = ClockType  # type: ignore[attr-defined]

    clock_type_mod = types.ModuleType("rclpy.clock_type")
    clock_type_mod.ClockType = ClockType  # type: ignore[attr-defined]

    sig_mod = types.ModuleType("rclpy.signals")
    sig_mod.SignalHandlerOptions = SignalHandlerOptions  # type: ignore[attr-defined]

    sub_mod = types.ModuleType("rclpy.subscription")
    sub_mod.Subscription = Subscription  # type: ignore[attr-defined]

    pub_mod = types.ModuleType("rclpy.publisher")
    pub_mod.Publisher = Publisher  # type: ignore[attr-defined]

    rg_pkg = _pkg("rosgraph_msgs")
    rg_msg = types.ModuleType("rosgraph_msgs.msg")
    rg_msg.Clock = Clock  # type: ignore[attr-defined]
    rg_pkg.msg = rg_msg  # type: ignore[attr-defined]

    std_pkg = _pkg("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.String = String  # type: ignore[attr-defined]
    std_pkg.msg = std_msg  # type: ignore[attr-defined]

    for name, mod in (
        ("rclpy", rclpy_mod),
        ("rclpy.node", node_mod),
        ("rclpy.executors", exec_mod),
        ("rclpy.parameter", param_mod),
        ("rclpy.qos", qos_mod),
        ("rclpy.clock", clock_mod),
        ("rclpy.clock_type", clock_type_mod),
        ("rclpy.signals", sig_mod),
        ("rclpy.subscription", sub_mod),
        ("rclpy.publisher", pub_mod),
        ("rosgraph_msgs", rg_pkg),
        ("rosgraph_msgs.msg", rg_msg),
        ("std_msgs", std_pkg),
        ("std_msgs.msg", std_msg),
    ):
        sys.modules[name] = mod


def _merge(distro: str) -> bool:
    prefix = Path("/opt/ros") / distro
    env = _source_env(prefix / "setup.bash")
    ros_py = [p for p in env.get("PYTHONPATH", "").split(":") if p.startswith(str(prefix))]
    minor = f"{sys.version_info.major}.{sys.version_info.minor}"
    compat_py = [p for p in ros_py if _abi_compatible(p, minor)]

    _apply_ros_env(distro, env)
    if getattr(sys.modules.get("rclpy"), "_yopo_humble_bridge", False):
        return True

    if compat_py:
        for p in reversed(compat_py):
            if p not in sys.path:
                sys.path.insert(0, p)
        for extra in ("/usr/lib/python3/dist-packages", f"/usr/local/lib/python{minor}/dist-packages"):
            if Path(extra).is_dir() and extra not in sys.path:
                sys.path.append(extra)
        old = os.environ.get("PYTHONPATH", "")
        parts = [p for p in compat_py if p and p not in old.split(":")]
        if parts:
            os.environ["PYTHONPATH"] = ":".join(parts + ([old] if old else []))
        importlib.invalidate_caches()
        try:
            import rclpy  # noqa: F401

            _patch_native_humble()
            return True
        except Exception:
            pass

    found = _find_host_ros_python(distro, ros_py, env)
    if found is not None:
        host_python, worker_py_paths = found
        _install_humble_bridge(host_python, worker_py_paths, distro, env)
        return True
    return False


def ensure_ros_paths() -> str | None:
    """返回发行版名（jazzy / humble / 已在路径里的 ROS_DISTRO）。没安装时返回 None。"""
    global _CACHED
    if _CACHED is not False:
        return _CACHED  # type: ignore[return-value]
    distro = _installed("jazzy", "humble")
    if distro is None:
        if importlib.util.find_spec("rclpy") is not None:
            _CACHED = os.environ.get("ROS_DISTRO") or "preloaded"
            return _CACHED
        _CACHED = None
        return None
    if not _merge(distro):
        _CACHED = None
        return None
    _CACHED = distro
    return distro
