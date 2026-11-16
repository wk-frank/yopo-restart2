#!/usr/bin/env python3
"""口径 A 第一期收口前置核验。只读，不改仓库。

默认做静态检查，不跑批跑：

- ``scenes/*.json`` 与 ``scenes/suite.txt`` 的默认策略仍是运行器的
  ``upstream`` + 影子盾开。研究开关 ``track_clearance``、``empty_brake``、
  ``peer_sphere_enabled``、``swarm_penalty``、``swarm_segment_penalty``、
  ``d435_depth`` 没有在场景卡上以 JSON true 打开。
- ``docs/results/SUMMARY.md``、``docs/results/SUMMARY.json``、
  ``docs/results/videos.md``、``docs/LIMITATIONS.md`` 里的场景卡哈希和相对链接。
  哈希与当前卡不一致、或 commit 仍停在旧提交（例如 ``f5985e4``），记入待刷新。
- ``core/VENDORED.md`` 与 ``core/VENDORED.sha256`` 存在，且清单里的字节哈希对得上。

``--with-smoke`` 额外跑 ``scenes/g2-hover.json`` 0.2 秒，确认这次进程的默认
策略和影子盾。需要上游权重，不在默认路径里。

退出码 0：没有硬破坏。待刷新和二期外延不挡退出码。
退出码 1：研究开关被场景卡打开、套件或文档断链、钉定哈希缺失或对不上、
运行器默认不再是 upstream + 影子盾开。
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import importlib.util
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from yopo_l0.experiment import RESEARCH_SWITCHES

SCHEMA = "yopo-restart/phase1-closeout/1"
STALE_EXAMPLE_PREFIX = "f5985e4"
UPSTREAM_SAFE_RADIUS_M = 0.05
# 套件里允许的任务策略。缺省（不写 @策略）等于批跑默认 upstream。
# certified / student 不是第一期默认基线。
SUITE_POLICIES = {"hover", "upstream", "formation", "crossing", "switch"}
DOC_MARKDOWN = (
    "docs/results/SUMMARY.md",
    "docs/results/videos.md",
    "docs/LIMITATIONS.md",
)
DOC_JSON = "docs/results/SUMMARY.json"
REQUIRED = (
    "yopo_l0/runner.py",
    "yopo_l0/batch.py",
    "yopo_l0/run.py",
    "scripts/regress.sh",
    "scenes/suite.txt",
    "docs/results/SUMMARY.md",
    "docs/results/SUMMARY.json",
    "docs/results/videos.md",
    "docs/LIMITATIONS.md",
    "core/VENDORED.md",
    "core/VENDORED.sha256",
)
SCENE_PATH_RE = re.compile(r"(?<![\w./])(scenes/[\w./-]+\.[A-Za-z0-9]+)")
HEX_RE = re.compile(r"^[0-9a-fA-F]+$")
FENCE_RE = re.compile(r"```.*?```|~~~.*?~~~", re.DOTALL)


def _item(id_: str, title: str, detail: str = "", **extra) -> dict:
    row = {"id": id_, "title": title, "detail": detail}
    row.update(extra)
    return row


def _load_doc_links():
    path = Path(__file__).resolve().parent / "check_doc_links.py"
    spec = importlib.util.spec_from_file_location("check_doc_links", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"无法加载 {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def git_head(root: Path) -> str | None:
    try:
        proc = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=root,
            capture_output=True,
            text=True,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    sha = proc.stdout.strip()
    return sha or None


def tracked_dirty(root: Path) -> bool:
    try:
        proc = subprocess.run(
            ["git", "status", "--porcelain", "--untracked-files=no"],
            cwd=root,
            capture_output=True,
            text=True,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return False
    return bool(proc.stdout.strip())


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _load_json(path: Path):
    return json.loads(_read(path))


def _class_defaults(source: str, class_name: str) -> dict:
    tree = ast.parse(source)
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            found = {}
            for stmt in node.body:
                if not isinstance(stmt, ast.AnnAssign) or not isinstance(stmt.target, ast.Name):
                    continue
                if stmt.value is None or not isinstance(stmt.value, ast.Constant):
                    continue
                found[stmt.target.id] = stmt.value.value
            return found
    return {}


def _commit_is_head(cited: str, head: str | None) -> bool:
    if not cited or head is None:
        return False
    cited_l = cited.lower()
    head_l = head.lower()
    if len(cited_l) < 7 or len(head_l) < 7:
        return False
    return head_l.startswith(cited_l) or cited_l.startswith(head_l)


def _docs_only_since(root: Path, cited: str, head: str | None) -> bool:
    if not cited or head is None or not HEX_RE.fullmatch(cited) or len(cited) < 7:
        return False
    if not (root / ".git").exists():
        return False
    try:
        resolved = subprocess.run(
            ["git", "rev-parse", "--verify", "--quiet", f"{cited}^{{commit}}"],
            cwd=root,
            capture_output=True,
            text=True,
            check=False,
        )
        if resolved.returncode != 0:
            return False
        full_cited = resolved.stdout.strip()
        anc = subprocess.run(
            ["git", "merge-base", "--is-ancestor", full_cited, head],
            cwd=root,
            capture_output=True,
            check=False,
        )
        if anc.returncode != 0:
            return False
        diff = subprocess.run(
            ["git", "diff", "--quiet", f"{full_cited}..{head}", "--", ".", ":(exclude)docs"],
            cwd=root,
            capture_output=True,
            check=False,
        )
        return diff.returncode == 0
    except OSError:
        return False


def _commit_is_current(root: Path, cited: str, head: str | None) -> bool:
    return _commit_is_head(cited, head) or _docs_only_since(root, cited, head)


def _is_stale_example(cited: str) -> bool:
    return cited.lower().startswith(STALE_EXAMPLE_PREFIX)


def _split_row(line: str) -> list[str]:
    text = line.strip()
    if text.startswith("|"):
        text = text[1:]
    if text.endswith("|"):
        text = text[:-1]
    return [cell.strip() for cell in text.split("|")]


def _is_rule(cells: list[str]) -> bool:
    if not cells:
        return False
    return all(re.fullmatch(r":?-{3,}:?", cell.replace(" ", "")) for cell in cells if cell)


def _code(cell: str) -> str:
    found = re.findall(r"`([^`]+)`", cell)
    return found[0].strip() if found else cell.strip()


def _scene_id(cell: str) -> str:
    text = _code(cell)
    text = re.sub(r"（[^）]*）", "", text)
    text = re.sub(r"\([^)]*\)", "", text)
    return text.strip()


def _section_lines(text: str, heading: str) -> list[tuple[int, str]]:
    lines = text.splitlines()
    start = None
    for index, line in enumerate(lines):
        if line.startswith(heading):
            start = index + 1
            break
    if start is None:
        return []
    picked = []
    for index in range(start, len(lines)):
        line = lines[index]
        if line.startswith("#") and not line.startswith(heading):
            break
        picked.append((index + 1, line))
    return picked


def _tables(text: str) -> list[list[dict]]:
    """按表头 + 分隔行识别 Markdown 表。数据行里出现 commit 字样不会被当成表头。"""
    lines = text.splitlines()
    tables = []
    index = 0
    total = len(lines)
    while index < total:
        if not lines[index].strip().startswith("|"):
            index += 1
            continue
        block: list[tuple[int, str]] = []
        while index < total and lines[index].strip().startswith("|"):
            block.append((index + 1, lines[index]))
            index += 1
        if len(block) < 2 or not _is_rule(_split_row(block[1][1])):
            continue
        header = _split_row(block[0][1])
        rows = []
        for lineno, line in block[2:]:
            cells = _split_row(line)
            if _is_rule(cells):
                continue
            row = {header[col]: cells[col] if col < len(cells) else "" for col in range(len(header))}
            row["_line"] = lineno
            rows.append(row)
        tables.append(rows)
    return tables


def _first_table(lines: list[tuple[int, str]]) -> list[dict]:
    header = None
    header_at = None
    rows = []
    for lineno, line in lines:
        if not line.strip().startswith("|"):
            if header is not None:
                break
            continue
        cells = _split_row(line)
        if _is_rule(cells):
            continue
        if header is None:
            header = cells
            header_at = lineno
            continue
        row = {header[i]: cells[i] if i < len(cells) else "" for i in range(len(header))}
        row["_line"] = lineno
        row["_header_line"] = header_at
        rows.append(row)
    return rows


def _walk_switches(obj, path: str = "$"):
    if isinstance(obj, dict):
        for key, value in obj.items():
            here = f"{path}.{key}"
            if key in RESEARCH_SWITCHES:
                yield here, value
            yield from _walk_switches(value, here)
    elif isinstance(obj, list):
        for index, value in enumerate(obj):
            yield from _walk_switches(value, f"{path}[{index}]")


def _backend_enabled(raw: dict) -> bool:
    block = (raw.get("policy_params") or {}).get("backend") if isinstance(raw, dict) else None
    return isinstance(block, dict) and block.get("enabled") is True


def _geom_fallback_enabled(raw: dict) -> bool:
    block = raw.get("geometric_fallback") if isinstance(raw, dict) else None
    return isinstance(block, dict) and block.get("enabled") is True


def _safe_radius(raw: dict):
    upstream = (raw.get("policy_params") or {}).get("upstream") if isinstance(raw, dict) else None
    if not isinstance(upstream, dict) or "safe_radius" not in upstream:
        return None
    return upstream.get("safe_radius")


def _shadow_off(raw: dict) -> bool:
    if not isinstance(raw, dict):
        return False
    if raw.get("shadow") is False:
        return True
    params = raw.get("policy_params")
    return isinstance(params, dict) and params.get("shadow") is False


def _parse_suite(text: str) -> list[dict]:
    entries = []
    for lineno, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        path, _, rest = line.partition("@")
        policy, _, seedpart = rest.partition("@")
        seeds = None
        seed_error = ""
        if seedpart:
            try:
                seeds = [int(piece) for piece in seedpart.split(",") if piece != ""]
            except ValueError:
                seed_error = seedpart
        entries.append(
            {
                "line": lineno,
                "raw": line,
                "path": path.strip(),
                "policy": policy.strip(),
                "seeds": seeds,
                "seed_error": seed_error,
            }
        )
    return entries


def _batch_defaults(text: str) -> tuple[str | None, bool | None]:
    policy = None
    matched = re.search(r'add_argument\(\s*"--policy"\s*,\s*default\s*=\s*"([^"]+)"', text)
    if matched:
        policy = matched.group(1)
    shadow = None
    matched = re.search(
        r'add_argument\(\s*"--shadow"\s*,\s*dest\s*=\s*"shadow"\s*,\s*action\s*=\s*"store_true"\s*,\s*default\s*=\s*(True|False)',
        text,
    )
    if matched:
        shadow = matched.group(1) == "True"
    return policy, shadow


def _cli_default(text: str, flag: str) -> str | None:
    """读 add_argument 的字符串默认值。忽略帮助文本里碰巧出现的同名旗标。"""
    matched = re.search(
        r"add_argument\(\s*[\"']" + re.escape(flag) + r"[\"']",
        text,
    )
    if not matched:
        return None
    window = text[matched.start():matched.start() + 500]
    found = re.search(r"default\s*=\s*[\"']([^\"']+)[\"']", window)
    if not found:
        return None
    return found.group(1)


def _suite_batch_line(text: str) -> str | None:
    for line in text.splitlines():
        if "yopo_l0.batch" in line and "${SUITE[@]}" in line:
            return line
    return None


def _hash_prefix(full: str, cited: str) -> bool:
    if not full or not cited:
        return False
    return full.lower().startswith(cited.lower())


def _scene_paths(text: str, scenes: Path) -> list[str]:
    found = []
    seen = set()
    for match in SCENE_PATH_RE.findall(text):
        if match not in seen:
            seen.add(match)
            found.append(match)
    for match in re.findall(r"`([^`]+)`", text):
        token = match.strip()
        name = token.split("@", 1)[0].strip()
        if name.startswith("scenes/"):
            path = name
        elif name.endswith(".json") and (scenes / Path(name).name).is_file():
            path = f"scenes/{Path(name).name}"
        else:
            continue
        if path not in seen:
            seen.add(path)
            found.append(path)
    return found


def _card_hash(path: Path) -> str:
    from yopo_l0.scene import load_card

    return load_card(path).hash


def _check_required(root: Path, hard: list[dict]) -> bool:
    missing = [rel for rel in REQUIRED if not (root / rel).is_file()]
    if missing:
        hard.append(_item("missing-files", "收口核验需要的文件缺失", "、".join(missing), paths=missing))
        return False
    return True


def _check_runner(root: Path, passed: list[dict], hard: list[dict]) -> None:
    runner_text = _read(root / "yopo_l0/runner.py")
    try:
        defaults = _class_defaults(runner_text, "RunConfig")
    except SyntaxError as exc:
        hard.append(_item("runner-default", "RunConfig 无法解析", str(exc)))
        return
    policy = defaults.get("policy")
    shadow = defaults.get("shadow")
    problems = []
    if policy != "upstream":
        problems.append(f"RunConfig.policy 默认是 {policy!r}，不是 upstream")
    if shadow is not True:
        problems.append(f"RunConfig.shadow 默认是 {shadow!r}，不是 True")
    batch_policy, batch_shadow = _batch_defaults(_read(root / "yopo_l0/batch.py"))
    if batch_policy != "upstream":
        problems.append(f"yopo_l0.batch --policy 默认是 {batch_policy!r}")
    if batch_shadow is not True:
        problems.append(f"yopo_l0.batch --shadow 默认是 {batch_shadow!r}")
    run_text = _read(root / "yopo_l0/run.py")
    run_policy = _cli_default(run_text, "--policy")
    run_shadow = _cli_default(run_text, "--shadow")
    if run_policy != "upstream":
        problems.append(f"yopo_l0.run --policy 默认是 {run_policy!r}")
    if run_shadow != "on":
        problems.append(f"yopo_l0.run --shadow 默认是 {run_shadow!r}")
    ros2 = root / "yopo_l0/ros2_run.py"
    if ros2.is_file():
        ros_text = _read(ros2)
        ros_policy = _cli_default(ros_text, "--policy")
        ros_shadow = _cli_default(ros_text, "--shadow")
        if ros_policy != "upstream":
            problems.append(f"yopo_l0.ros2_run --policy 默认是 {ros_policy!r}")
        if ros_shadow != "on":
            problems.append(f"yopo_l0.ros2_run --shadow 默认是 {ros_shadow!r}")
    if problems:
        hard.append(_item("runner-default", "运行器默认不再是 upstream + 影子盾开", "；".join(problems)))
        return
    passed.append(
        _item(
            "runner-default",
            "运行器默认是 upstream + 影子盾开",
            "RunConfig.policy 是 upstream，RunConfig.shadow 是 True。"
            "yopo_l0.run、批跑和 ROS 2 入口的默认与这一条一致。",
        )
    )


def _check_regress(root: Path, passed: list[dict], hard: list[dict]) -> None:
    text = _read(root / "scripts/regress.sh")
    line = _suite_batch_line(text)
    if line is None:
        hard.append(_item("regress-suite", "regress.sh 找不到套件批跑", "需要带 ${SUITE[@]} 的 yopo_l0.batch 行"))
        return
    if "--no-shadow" in line or "--shadow off" in line or "--shadow=off" in line:
        hard.append(_item("regress-suite", "套件批跑关掉了影子盾", line.strip()))
        return
    note = "套件批跑没有 --no-shadow，沿用运行器默认。"
    if "--no-shadow" in text and "@certified" in text:
        note += "紧随的 certified 冒烟显式 --no-shadow，只作对照，不改变套件默认。"
    passed.append(_item("regress-suite", "合入前套件批跑保持影子盾默认开", note))


def _check_scenes(root: Path, passed: list[dict], hard: list[dict], phase2: list[dict]) -> dict:
    scene_dir = root / "scenes"
    suite_entries = _parse_suite(_read(scene_dir / "suite.txt"))
    suite_paths = set()
    open_switches = []
    malformed = []
    shadow_off = []
    radius_off = []
    research_cards = []
    flight = []
    json_errors = []
    for path in sorted(scene_dir.glob("*.json")):
        try:
            raw = json.loads(_read(path))
        except json.JSONDecodeError as exc:
            json_errors.append(f"{path.name}: {exc}")
            continue
        if not isinstance(raw, dict):
            json_errors.append(f"{path.name}: 不是对象")
            continue
        rel = path.relative_to(root).as_posix()
        for pointer, value in _walk_switches(raw):
            if value is True:
                open_switches.append(f"{rel} {pointer}")
            elif value is not False:
                malformed.append(f"{rel} {pointer}={value!r}")
        if _shadow_off(raw):
            shadow_off.append(rel)
        radius = _safe_radius(raw)
        if radius is not None and radius != UPSTREAM_SAFE_RADIUS_M:
            radius_off.append(f"{rel} safe_radius={radius!r}")
        if isinstance(raw.get("task"), dict):
            flight.append(rel)
            flags = []
            if _backend_enabled(raw):
                flags.append("backend.enabled")
            if _geom_fallback_enabled(raw):
                flags.append("geometric_fallback.enabled")
            if flags:
                research_cards.append({"path": rel, "flags": flags, "in_suite": False})
    if json_errors:
        hard.append(_item("scene-json", "场景 JSON 无法解析", "；".join(json_errors)))
    unloadable = []
    for rel in flight:
        try:
            _card_hash(root / rel)
        except Exception as exc:
            unloadable.append(f"{rel}: {exc}")
    if unloadable:
        hard.append(_item("scene-load", "飞行场景卡无法装载", "；".join(unloadable)))
    if open_switches or malformed or shadow_off or radius_off:
        bits = []
        if open_switches:
            bits.append("研究开关为 JSON true：" + "；".join(open_switches))
        if malformed:
            bits.append("研究开关不是布尔：" + "；".join(malformed))
        if shadow_off:
            bits.append("影子盾被写成关：" + "、".join(shadow_off))
        if radius_off:
            bits.append("safe_radius 偏离 0.05 m：" + "；".join(radius_off))
        hard.append(_item("research-switches", "场景卡偏离上游默认开关", "。".join(bits)))
    elif not json_errors and not unloadable:
        passed.append(
            _item(
                "research-switches-off",
                f"场景卡没有打开{len(RESEARCH_SWITCHES)}个研究开关",
                "、".join(RESEARCH_SWITCHES) + " 都不是 JSON true。影子盾没有被写成关。"
                "写了 safe_radius 的卡仍是 0.05 m，没写的沿用上游缺省。",
                scene_json=len(list(scene_dir.glob("*.json"))),
                flight_cards=len(flight),
            )
        )
    suite_problems = []
    for entry in suite_entries:
        if entry["seed_error"]:
            suite_problems.append(f"第 {entry['line']} 行 seed 无法解析：{entry['raw']}")
            continue
        for name in RESEARCH_SWITCHES:
            if name in entry["raw"]:
                suite_problems.append(f"第 {entry['line']} 行写了研究开关 {name}")
        policy = entry["policy"] or "upstream"
        if policy not in SUITE_POLICIES:
            suite_problems.append(f"第 {entry['line']} 行策略 {policy} 不是第一期套件策略")
        rel = entry["path"]
        target = (root / rel).resolve()
        try:
            target.relative_to(root.resolve())
        except ValueError:
            suite_problems.append(f"第 {entry['line']} 行路径越出仓库：{rel}")
            continue
        if not (root / rel).is_file():
            suite_problems.append(f"第 {entry['line']} 行场景卡不存在：{rel}")
            continue
        suite_paths.add(rel.replace("\\", "/"))
    for card in research_cards:
        card["in_suite"] = card["path"] in suite_paths
        if card["in_suite"]:
            suite_problems.append(
                f"{card['path']} 在 suite.txt 里，且打开了 {'、'.join(card['flags'])}"
            )
    if suite_problems:
        hard.append(_item("suite", "suite.txt 偏离默认基线或断链", "；".join(suite_problems), entries=len(suite_entries)))
    else:
        policies = sorted({(entry["policy"] or "upstream") for entry in suite_entries})
        passed.append(
            _item(
                "suite",
                "suite.txt 指向存在的场景卡，且没有打开研究开关",
                f"共 {len(suite_entries)} 条。任务策略：{'、'.join(policies) or '（空）'}。"
                "不写 @策略时批跑默认 upstream。套件里没有 certified 或 student。",
                entries=len(suite_entries),
                policies=policies,
            )
        )
    outside = [card for card in research_cards if not card["in_suite"]]
    detail = (
        "这些研究开关的打开态、student 蒸馏和 SITL 实飞都不进第一期默认套件。"
        "后段或几何回退若只出现在套件外的研究卡上，不把它们当成默认基线。"
    )
    if outside:
        listed = "；".join(f"{card['path']}（{'、'.join(card['flags'])}）" for card in outside)
        detail += "当前套件外的研究卡：" + listed + "。"
    phase2.append(
        _item(
            "research-arms",
            "研究开关实验臂不进第一期默认基线",
            detail,
            cards=outside,
        )
    )
    return {"suite_entries": suite_entries, "suite_paths": suite_paths, "research_cards": research_cards}


def _check_vendored(root: Path, passed: list[dict], hard: list[dict]) -> None:
    doc = root / "core/VENDORED.md"
    pin = root / "core/VENDORED.sha256"
    doc_text = _read(doc)
    if "VENDORED.sha256" not in doc_text or not doc_text.strip():
        hard.append(_item("vendored-pin", "core/VENDORED.md 没有指向钉定清单", "需要写明 VENDORED.sha256"))
        return
    pinned = {}
    problems = []
    for lineno, raw in enumerate(_read(pin).splitlines(), 1):
        line = raw.strip()
        if not line:
            continue
        parts = line.split(maxsplit=1)
        if len(parts) != 2 or not HEX_RE.fullmatch(parts[0]) or len(parts[0]) != 64:
            problems.append(f"第 {lineno} 行不是 sha256 路径")
            continue
        digest, rel = parts[0].lower(), parts[1].strip()
        if rel in pinned:
            problems.append(f"重复路径 {rel}")
            continue
        target = root / "core" / rel
        if not target.is_file():
            problems.append(f"缺失 {rel}")
            continue
        actual = hashlib.sha256(target.read_bytes()).hexdigest()
        if actual != digest:
            problems.append(f"哈希不符 {rel}")
            continue
        pinned[rel] = digest
    src_root = root / "core/src"
    unpinned_src = []
    if src_root.is_dir():
        for path in sorted(src_root.rglob("*.py")):
            rel = path.relative_to(root / "core").as_posix()
            if rel not in pinned:
                unpinned_src.append(rel)
    if unpinned_src:
        problems.append("src 未钉定：" + "、".join(unpinned_src))
    if problems:
        hard.append(_item("vendored-pin", "core 钉定哈希不完好", "；".join(problems), pinned=len(pinned)))
        return
    extras = []
    for path in sorted((root / "core").rglob("*")):
        if not path.is_file():
            continue
        if "__pycache__" in path.parts or path.suffix == ".pyc":
            continue
        rel = path.relative_to(root / "core").as_posix()
        if rel in pinned or rel in {"VENDORED.md", "VENDORED.sha256"}:
            continue
        extras.append(rel)
    passed.append(
        _item(
            "vendored-pin",
            "core 钉定哈希完好",
            f"VENDORED.md 指向 VENDORED.sha256，{len(pinned)} 个文件字节一致。"
            f"清单之外还有 {len(extras)} 个文件（含票 13d 写明不在原始清单里的测试），不改写钉定字节。",
            pinned=len(pinned),
            extras=extras,
        )
    )


def _check_docs(root: Path, head: str | None, passed: list[dict], refresh: list[dict], hard: list[dict]) -> None:
    links = _load_doc_links()
    broken = []
    for rel in DOC_MARKDOWN:
        path = root / rel
        for url, reason in links.check_file(path):
            broken.append(f"{rel}: {url}（{reason}）")
    scenes = root / "scenes"
    missing_paths = []
    for rel in (*DOC_MARKDOWN, DOC_JSON):
        text = _read(root / rel)
        for cited in _scene_paths(text, scenes):
            if not (root / cited).is_file():
                missing_paths.append(f"{rel} → {cited}")
    if broken or missing_paths:
        detail = ""
        if broken:
            detail += "断链：" + "；".join(broken)
        if missing_paths:
            if detail:
                detail += "。"
            detail += "场景路径不存在：" + "；".join(missing_paths)
        hard.append(_item("doc-links", "交付文档有断链", detail))
    else:
        passed.append(
            _item(
                "doc-links",
                "交付文档的相对链接和场景路径有效",
                "SUMMARY.md、videos.md、LIMITATIONS.md 的相对链接都能落到文件。"
                "这三份加上 SUMMARY.json 里写到的 scenes/ 路径都存在。",
            )
        )

    try:
        summary = _load_json(root / DOC_JSON)
    except json.JSONDecodeError as exc:
        hard.append(_item("summary-json", "SUMMARY.json 无法解析", str(exc)))
        return
    rows = summary.get("rows")
    if not isinstance(rows, list):
        hard.append(_item("summary-json", "SUMMARY.json 没有 rows", ""))
        return
    by_scene: dict[str, set[str]] = {}
    json_commit = str(summary.get("commit") or "")
    for row in rows:
        if not isinstance(row, dict):
            hard.append(_item("summary-json", "SUMMARY.json 的 row 不是对象", ""))
            return
        scene = str(row.get("scene") or "")
        cited = str(row.get("scene_card_hash") or "")
        if not HEX_RE.fullmatch(cited) or len(cited) != 64:
            hard.append(_item("summary-hash", "SUMMARY.json 的场景卡哈希不是 64 位十六进制", f"{scene}: {cited}"))
            return
        by_scene.setdefault(scene, set()).add(cited.lower())
    disagree = [scene for scene, hashes in by_scene.items() if len(hashes) > 1]
    if disagree:
        hard.append(_item("summary-hash", "同一场景在 SUMMARY.json 里有多份哈希", "、".join(sorted(disagree))))
        return

    current = {}
    hash_problems = []
    for scene, hashes in sorted(by_scene.items()):
        path = scenes / f"{scene}.json"
        if not path.is_file():
            hash_problems.append(f"{scene} 没有 scenes/{scene}.json")
            continue
        try:
            current[scene] = _card_hash(path)
        except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            hash_problems.append(f"{scene} 无法计算场景卡哈希：{exc}")
    if hash_problems:
        hard.append(_item("summary-hash", "总表场景卡无法核对", "；".join(hash_problems)))
        return

    md_text = _read(root / "docs/results/SUMMARY.md")
    table = _first_table(_section_lines(md_text, "## 总表"))
    if not table:
        hard.append(_item("summary-md", "SUMMARY.md 没有总表", "需要 ## 总表 下的表"))
        return
    old_rows = []
    cross = []
    drifted = []
    matched = []
    for row in table:
        scene = _scene_id(row.get("场景卡", ""))
        cited_hash = _code(row.get("场景卡哈希", ""))
        cited_commit = _code(row.get("commit", ""))
        full_hash = next(iter(by_scene.get(scene, set())), "")
        if scene not in by_scene:
            cross.append(f"第 {row['_line']} 行 {scene} 不在 SUMMARY.json")
            continue
        if not _hash_prefix(full_hash, cited_hash):
            cross.append(
                f"第 {row['_line']} 行 {scene} 的短哈希 {cited_hash} 不是 SUMMARY.json {full_hash[:12]} 的前缀"
            )
        actual = current.get(scene, "")
        hash_ok = bool(actual) and actual.lower() == full_hash
        if hash_ok:
            matched.append(scene)
        elif actual:
            drifted.append(
                {
                    "scene": scene,
                    "cited": full_hash,
                    "current": actual,
                    "line": row["_line"],
                }
            )
        commit_full = json_commit if _commit_is_head(cited_commit, json_commit) or _hash_prefix(json_commit, cited_commit) else cited_commit
        on_head = _commit_is_current(root, cited_commit, head)
        if not on_head:
            old_rows.append(
                {
                    "source": "docs/results/SUMMARY.md",
                    "line": row["_line"],
                    "milestone": _code(row.get("里程碑", "")),
                    "scene": scene,
                    "policy": _code(row.get("策略", "")),
                    "shadow": _code(row.get("影子盾", "")),
                    "commit": cited_commit,
                    "commit_full": commit_full if _hash_prefix(json_commit, cited_commit) else cited_commit,
                    "scene_card_hash_cited": cited_hash,
                    "scene_card_hash_current": actual,
                    "hash_matches_current": hash_ok,
                    "matches_stale_example": _is_stale_example(cited_commit) or _is_stale_example(commit_full),
                    "head_matches": False,
                }
            )
    if cross:
        hard.append(_item("summary-cross", "SUMMARY.md 与 SUMMARY.json 的场景卡哈希对不上", "；".join(cross)))
    if drifted:
        refresh.append(
            _item(
                "scene-card-hash-drift",
                "总表场景卡哈希与当前场景卡不一致",
                "封版前要重跑或改写这些行。这不是研究开关被打开。",
                entries=drifted,
            )
        )
    if matched and not drifted and not cross:
        unique = sorted(set(matched))
        passed.append(
            _item(
                "scene-card-hashes",
                "总表场景卡哈希与当前场景卡一致",
                f"{len(unique)} 张卡的全长哈希等于 SUMMARY.json，SUMMARY.md 的短哈希是其前缀。",
                scenes=unique,
            )
        )
    elif matched and drifted:
        passed.append(
            _item(
                "scene-card-hashes",
                "部分总表场景卡哈希仍与当前卡一致",
                "一致的卡：" + "、".join(sorted(set(matched))) + "。不一致的见待刷新。",
            )
        )

    video_rows = []
    video_text = _read(root / "docs/results/videos.md")
    for table in _tables(video_text):
        if not table:
            continue
        keys = [key for key in table[0] if not str(key).startswith("_")]
        if keys == ["项", "值"]:
            for row in table:
                label = row.get("项", "")
                if "commit" not in label.lower():
                    continue
                token = _code(row.get("值", ""))
                if not HEX_RE.fullmatch(token) or _commit_is_current(root, token, head):
                    continue
                video_rows.append(
                    {
                        "source": "docs/results/videos.md",
                        "line": row.get("_line"),
                        "scene": row.get("项", "").strip(),
                        "commit": token,
                        "matches_stale_example": _is_stale_example(token),
                        "head_matches": False,
                    }
                )
            continue
        commit_key = next((key for key in keys if key.strip().lower() == "commit"), None)
        if commit_key is None:
            continue
        scene_key = next((key for key in keys if "场景" in key), None)
        for row in table:
            token = _code(row.get(commit_key, ""))
            if not HEX_RE.fullmatch(token) or _commit_is_current(root, token, head):
                continue
            video_rows.append(
                {
                    "source": "docs/results/videos.md",
                    "line": row.get("_line"),
                    "scene": (row.get(scene_key) or "").strip() if scene_key else "",
                    "commit": token,
                    "matches_stale_example": _is_stale_example(token),
                    "head_matches": False,
                }
            )
    prose_commits = []
    visible = FENCE_RE.sub("", md_text)
    prose = "\n".join(line for line in visible.splitlines() if not line.strip().startswith("|"))
    for token in re.findall(r"`([0-9a-fA-F]{7,40})`", prose):
        if _hash_prefix(json_commit, token) or _is_stale_example(token):
            if token not in prose_commits:
                prose_commits.append(token)
    checkout = []
    for token in re.findall(r"\bgit checkout\s+([0-9a-fA-F]{7,40})", md_text):
        if token not in checkout:
            checkout.append(token)
    if old_rows or video_rows or any(not _commit_is_current(root, token, head) for token in prose_commits + checkout):
        stale_n = sum(1 for row in old_rows if row["matches_stale_example"])
        detail = (
            f"SUMMARY.md 总表有 {len(old_rows)} 行的 commit 不是当前 HEAD"
            f"（其中 {stale_n} 行匹配旧提交示例 {STALE_EXAMPLE_PREFIX}）。"
        )
        if json_commit and not _commit_is_current(root, json_commit, head):
            detail += f"SUMMARY.json 的 commit 是 `{json_commit}`。"
        if prose_commits or checkout:
            detail += "正文或复现命令还引用了 " + "、".join(f"`{token}`" for token in prose_commits + checkout) + "。"
        if video_rows:
            detail += f"videos.md 有 {len(video_rows)} 条仍停在旧 commit。"
        if old_rows and all(row["hash_matches_current"] for row in old_rows):
            detail += "这些行的场景卡哈希仍与当前卡一致，所以要刷新的是出处 commit，不是哈希断档。"
        refresh.append(
            _item(
                "stale-commits",
                "交付索引仍引用旧 commit",
                detail,
                summary_rows=old_rows,
                video_rows=video_rows,
                prose_commits=prose_commits,
                reproduce_checkouts=checkout,
                summary_commit=json_commit,
            )
        )
    elif head:
        passed.append(
            _item(
                "commits-current",
                "总表和视频索引的 commit 就是当前 HEAD",
                head,
            )
        )


def _phase2_fixed() -> list[dict]:
    return [
        _item(
            "sitl-flight",
            "SITL 实飞归二期",
            "口径 A 的第一期是 L0 可回归闭环和 M0–M5 交付包。"
            "实飞，以及把本仓库 upstream 接到 PX4 SITL 的闭环抽检，归二期。"
            "票 09 已关闭。S1–S4 的冒烟和桥只报数，不作为本次硬门。",
        ),
        _item(
            "distill",
            "蒸馏归二期",
            "student 策略默认不选，不进 scenes/suite.txt，也不改 RunConfig 的默认。"
            "权重和教师数据在仓库外。第一期封版不把蒸馏跑数当成硬门。",
        ),
    ]


def _smoke(root: Path) -> dict:
    scene = root / "scenes/g2-hover.json"
    if not scene.is_file():
        return {"ok": False, "error": "缺少 scenes/g2-hover.json", "policy": None, "shadow": None}
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "smoke"
        proc = subprocess.run(
            [
                sys.executable,
                "-m",
                "yopo_l0.run",
                "--scene",
                str(scene),
                "--duration",
                "0.2",
                "--out",
                str(out),
            ],
            cwd=root,
            capture_output=True,
            text=True,
            timeout=180,
        )
        result_path = out / "result.json"
        if not result_path.is_file():
            tail = (proc.stderr or proc.stdout or "")[-800:]
            return {"ok": False, "error": tail, "returncode": proc.returncode, "policy": None, "shadow": None}
        result = json.loads(result_path.read_text(encoding="utf-8"))
        policy = result.get("policy")
        shadow = result.get("shadow")
        ok = policy == "upstream" and shadow is True
        return {
            "ok": ok,
            "policy": policy,
            "shadow": shadow,
            "returncode": proc.returncode,
            "scene": "scenes/g2-hover.json",
            "duration_s": 0.2,
            "error": "" if ok else "短跑结果不是 upstream + 影子盾开",
        }


def audit(root: Path | str, *, head: str | None = None, with_smoke: bool = False, smoke_fn=None) -> dict:
    """返回核对清单。不写文件。"""
    root = Path(root).resolve()
    if head is None:
        head = git_head(root)
    passed: list[dict] = []
    refresh: list[dict] = []
    hard: list[dict] = []
    phase2 = _phase2_fixed()
    smoke = None
    if not _check_required(root, hard):
        return _report(root, head, with_smoke, passed, refresh, phase2, hard, smoke)
    _check_runner(root, passed, hard)
    _check_regress(root, passed, hard)
    _check_scenes(root, passed, hard, phase2)
    _check_vendored(root, passed, hard)
    _check_docs(root, head, passed, refresh, hard)
    if with_smoke:
        try:
            smoke = (smoke_fn or _smoke)(root)
        except (OSError, subprocess.TimeoutExpired, json.JSONDecodeError) as exc:
            smoke = {"ok": False, "error": str(exc), "policy": None, "shadow": None}
        if smoke.get("ok"):
            passed.append(
                _item(
                    "smoke",
                    "短跑确认默认仍是 upstream + 影子盾开",
                    f"{smoke.get('scene', 'scenes/g2-hover.json')} {smoke.get('duration_s', 0.2)} 秒，"
                    f"policy={smoke.get('policy')}，shadow={smoke.get('shadow')}。",
                )
            )
        else:
            hard.append(_item("smoke", "短跑没有保持 upstream + 影子盾开", str(smoke.get("error") or smoke)))
    return _report(root, head, with_smoke, passed, refresh, phase2, hard, smoke)


def _report(root, head, with_smoke, passed, refresh, phase2, hard, smoke) -> dict:
    old_rows = []
    video_rows = []
    for item in refresh:
        old_rows.extend(item.get("summary_rows") or [])
        video_rows.extend(item.get("video_rows") or [])
    return {
        "schema": SCHEMA,
        "ok": not hard,
        "exit_code": 0 if not hard else 1,
        "head": head,
        "tracked_dirty": tracked_dirty(root) if (root / ".git").exists() or (root / ".git").is_file() else False,
        "mode": "smoke" if with_smoke else "static",
        "root": "." if root == ROOT else str(root),
        "note": "退出码 0 表示没有硬破坏。待刷新（旧 commit、哈希漂移）和二期外延不挡退出码。",
        "passed": passed,
        "refresh": refresh,
        "phase2": phase2,
        "hard_failures": hard,
        "summary_rows_on_old_commit": old_rows,
        "video_rows_on_old_commit": video_rows,
        "smoke": smoke,
        "counts": {
            "passed": len(passed),
            "refresh": len(refresh),
            "phase2": len(phase2),
            "hard": len(hard),
            "summary_rows_on_old_commit": len(old_rows),
            "video_rows_on_old_commit": len(video_rows),
        },
    }


def _bullet(item: dict) -> list[str]:
    lines = [f"- **{item['title']}**（`{item['id']}`）。{item.get('detail') or ''}".rstrip()]
    rows = item.get("summary_rows") or []
    if rows:
        lines.append("")
        lines.append("| 行 | 里程碑 | 场景 | commit | 哈希仍匹配 | 旧提交示例 f5985e4 |")
        lines.append("|---|---|---|---|---|---|")
        for row in rows:
            lines.append(
                "| {line} | {milestone} | {scene} | `{commit}` | {hash_ok} | {stale} |".format(
                    line=row.get("line"),
                    milestone=row.get("milestone"),
                    scene=row.get("scene"),
                    commit=row.get("commit"),
                    hash_ok="是" if row.get("hash_matches_current") else "否",
                    stale="是" if row.get("matches_stale_example") else "否",
                )
            )
        lines.append("")
    videos = item.get("video_rows") or []
    if videos:
        lines.append("")
        lines.append("| 行 | 场景 | commit | 旧提交示例 f5985e4 |")
        lines.append("|---|---|---|---|")
        for row in videos:
            lines.append(
                "| {line} | {scene} | `{commit}` | {stale} |".format(
                    line=row.get("line"),
                    scene=row.get("scene") or "",
                    commit=row.get("commit"),
                    stale="是" if row.get("matches_stale_example") else "否",
                )
            )
        lines.append("")
    drifted = item.get("entries") or []
    if drifted and item["id"] == "scene-card-hash-drift":
        lines.append("")
        lines.append("| 场景 | 总表哈希 | 当前哈希 |")
        lines.append("|---|---|---|")
        for row in drifted:
            lines.append(f"| {row['scene']} | `{row['cited'][:12]}` | `{row['current'][:12]}` |")
        lines.append("")
    return lines


def render_markdown(report: dict) -> str:
    head = report.get("head") or "未知"
    mode = {"static": "静态", "smoke": "短跑"}.get(report["mode"], report["mode"])
    lines = [
        "# 口径 A 第一期收口核验",
        "",
        f"模式：{mode}。HEAD：`{head}`。退出码：{report['exit_code']}。",
        report["note"],
        "只读检查，不改场景卡、规划器、`core/` 和交付文档。",
        "",
        "## 通过项",
        "",
    ]
    if report["passed"]:
        for item in report["passed"]:
            lines.extend(_bullet(item))
    else:
        lines.append("- （无）")
    lines.extend(["", "## 待刷新项", ""])
    if report["refresh"]:
        for item in report["refresh"]:
            lines.extend(_bullet(item))
    else:
        lines.append("- （无）")
    lines.extend(["", "## 二期外延项", ""])
    if report["phase2"]:
        for item in report["phase2"]:
            lines.extend(_bullet(item))
    else:
        lines.append("- （无）")
    lines.extend(["", "## 硬破坏", ""])
    if report["hard_failures"]:
        for item in report["hard_failures"]:
            lines.extend(_bullet(item))
    else:
        lines.append("- （无）")
    lines.append("")
    return "\n".join(lines)


def render_json(report: dict) -> str:
    return json.dumps(report, ensure_ascii=False, indent=2) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT, help="仓库根目录")
    parser.add_argument("--with-smoke", action="store_true", help="额外短跑 g2-hover 0.2 秒，确认默认策略")
    parser.add_argument("--format", choices=("both", "json", "md"), default="both")
    parser.add_argument("--json", type=Path, default=None, help="把 JSON 写到这个路径")
    parser.add_argument("--md", type=Path, default=None, help="把 Markdown 写到这个路径")
    args = parser.parse_args(argv)
    report = audit(args.root, with_smoke=args.with_smoke)
    markdown = render_markdown(report)
    payload = render_json(report)
    if args.md:
        args.md.write_text(markdown, encoding="utf-8")
    if args.json:
        args.json.write_text(payload, encoding="utf-8")
    if args.format in ("md", "both"):
        sys.stdout.write(markdown)
        if not markdown.endswith("\n"):
            sys.stdout.write("\n")
    if args.format == "both":
        sys.stdout.write("---\n")
    if args.format in ("json", "both"):
        sys.stdout.write(payload)
    return int(report["exit_code"])


if __name__ == "__main__":
    sys.exit(main())
