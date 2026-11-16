"""口径 A 第一期收口核验：默认静态通过，硬破坏才非 0。"""

import importlib.util
import json
from pathlib import Path

import pytest

from conftest import ROOT

HEAD = "a" * 40
STALE = "f5985e4335fcc00a4e19c861dcd1a05e0113f797"


def load_mod():
    path = ROOT / "scripts/check_phase1_closeout.py"
    spec = importlib.util.spec_from_file_location("check_phase1_closeout", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MOD = load_mod()


def _card(name="hover", **extra):
    card = {
        "name": name,
        "geometry": "geom.json",
        "ground_z": 0.0,
        "drones": [{"id": "uav0", "start_enu": [0.0, 0.0, 1.5], "hover_enu": [0.0, 0.0, 1.5]}],
        "task": {"type": "hover", "duration_s": 1.0},
    }
    card.update(extra)
    return card


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def make_tree(tmp_path: Path, *, commit: str = HEAD, card=None, cited_hash: str | None = None) -> Path:
    root = tmp_path
    raw = card if card is not None else _card()
    _write(root / "scenes/geom.json", json.dumps({"boxes": [], "cylinders": []}))
    _write(root / "scenes/hover.json", json.dumps(raw))
    _write(root / "scenes/suite.txt", "scenes/hover.json@hover\n")
    digest = MOD._card_hash(root / "scenes/hover.json")
    shown = cited_hash if cited_hash is not None else digest
    _write(
        root / "docs/results/SUMMARY.md",
        "\n".join(
            [
                "# 总表",
                "",
                f"对应 commit `{commit[:12]}`。",
                "",
                "见 [视频](videos.md) 和 [限制](../LIMITATIONS.md)。",
                "",
                "## 总表",
                "",
                "| 里程碑 | 场景卡 | 策略 | 层级 | 形态 / 档位 | 影子盾 | commit | 场景卡哈希 |",
                "|---|---|---|---|---|---|---|---|",
                f"| M0 | hover | hover | L0 | 进程内 | 开 | `{commit[:12]}` | `{shown[:12]}` |",
                "",
                "```bash",
                f"git checkout {commit}",
                ".venv/bin/python -m yopo_l0.batch scenes/hover.json@hover",
                "```",
                "",
            ]
        ),
    )
    summary = {
        "commit": commit,
        "rows": [
            {
                "scene": "hover",
                "commit": commit,
                "scene_card_hash": shown,
                "shadow": True,
                "policy": "hover",
            }
        ],
    }
    _write(root / "docs/results/SUMMARY.json", json.dumps(summary))
    _write(
        root / "docs/results/videos.md",
        "\n".join(
            [
                "# 交付视频",
                "",
                "| 项 | 值 |",
                "|---|---|",
                f"| 代码 commit | `{commit}` |",
                "",
                "| 文件 | 场景 | commit |",
                "|---|---|---|",
                f"| `hover.mp4` | scenes/hover.json@hover | `{commit}` |",
                "",
            ]
        ),
    )
    _write(root / "docs/LIMITATIONS.md", "见 [总表](results/SUMMARY.md)。\n")
    _write(
        root / "yopo_l0/runner.py",
        "class RunConfig:\n    policy: str = \"upstream\"\n    shadow: bool = True\n",
    )
    _write(
        root / "yopo_l0/batch.py",
        "\n".join(
            [
                'ap.add_argument("--policy", default="upstream")',
                'ap.add_argument("--shadow", dest="shadow", action="store_true", default=True)',
                "",
            ]
        ),
    )
    _write(
        root / "yopo_l0/run.py",
        "\n".join(
            [
                'ap.add_argument("--policy", default="upstream")',
                'ap.add_argument("--shadow", choices=("on", "off"), default="on")',
                "",
            ]
        ),
    )
    _write(
        root / "scripts/regress.sh",
        '$PY -m yopo_l0.batch "${SUITE[@]}" "$@" --jobs "$JOBS" --out "$OUT"\n',
    )
    payload = b"pinned\n"
    rel = "src/pin.txt"
    pin = __import__("hashlib").sha256(payload).hexdigest()
    _write(root / "core" / rel, payload.decode())
    _write(root / "core/VENDORED.sha256", f"{pin}  {rel}\n")
    _write(root / "core/VENDORED.md", "清单见 VENDORED.sha256。\n")
    return root


def _ids(report, key):
    return {item["id"] for item in report[key]}


def test_clean_tree_exits_zero(tmp_path):
    report = MOD.audit(make_tree(tmp_path), head=HEAD)
    assert report["exit_code"] == 0
    assert report["hard_failures"] == []
    assert report["summary_rows_on_old_commit"] == []
    assert "research-switches-off" in _ids(report, "passed")
    assert "scene-card-hashes" in _ids(report, "passed")
    assert "vendored-pin" in _ids(report, "passed")
    assert "doc-links" in _ids(report, "passed")
    assert "runner-default" in _ids(report, "passed")
    assert {item["id"] for item in report["phase2"]} >= {"sitl-flight", "distill", "research-arms"}
    text = MOD.render_markdown(report)
    assert "## 通过项" in text and "## 待刷新项" in text and "## 二期外延项" in text
    assert "（无）" in text.split("## 硬破坏", 1)[1]


def test_stale_f5985e4_is_refresh_not_hard(tmp_path):
    root = make_tree(tmp_path, commit=STALE)
    report = MOD.audit(root, head=HEAD)
    assert report["exit_code"] == 0
    assert report["hard_failures"] == []
    rows = report["summary_rows_on_old_commit"]
    assert len(rows) == 1
    assert rows[0]["scene"] == "hover"
    assert rows[0]["matches_stale_example"] is True
    assert rows[0]["hash_matches_current"] is True
    assert rows[0]["head_matches"] is False
    assert report["video_rows_on_old_commit"]
    assert "stale-commits" in _ids(report, "refresh")


def test_hash_drift_is_refresh_when_docs_agree(tmp_path):
    root = make_tree(tmp_path)
    real = MOD._card_hash(root / "scenes/hover.json")
    fake = "b" * 64
    assert not real.startswith(fake[:12])
    text = (root / "docs/results/SUMMARY.md").read_text(encoding="utf-8")
    text = text.replace(real[:12], fake[:12])
    (root / "docs/results/SUMMARY.md").write_text(text, encoding="utf-8")
    payload = json.loads((root / "docs/results/SUMMARY.json").read_text(encoding="utf-8"))
    payload["rows"][0]["scene_card_hash"] = fake
    (root / "docs/results/SUMMARY.json").write_text(json.dumps(payload), encoding="utf-8")
    report = MOD.audit(root, head=HEAD)
    assert report["exit_code"] == 0
    assert "scene-card-hash-drift" in _ids(report, "refresh")
    assert report["hard_failures"] == []


def test_md_json_hash_disagreement_is_hard(tmp_path):
    root = make_tree(tmp_path)
    text = (root / "docs/results/SUMMARY.md").read_text(encoding="utf-8")
    real = MOD._card_hash(root / "scenes/hover.json")
    text = text.replace(f"`{real[:12]}`", "`cccccccccccc`")
    (root / "docs/results/SUMMARY.md").write_text(text, encoding="utf-8")
    report = MOD.audit(root, head=HEAD)
    assert report["exit_code"] == 1
    assert "summary-cross" in _ids(report, "hard_failures")


@pytest.mark.parametrize(
    "switch_path",
    [
        {"policy_params": {"upstream": {"track_clearance": True}}},
        {"policy_params": {"upstream": {"empty_brake": True}}},
        {"policy_params": {"backend": {"enabled": True, "swarm_penalty": True}}},
        {"policy_params": {"backend": {"enabled": True, "swarm_segment_penalty": True}}},
        {"policy_params": {"backend": {"enabled": True, "peer_sphere_enabled": True}}},
    ],
)
def test_research_switch_on_is_hard(tmp_path, switch_path):
    root = make_tree(tmp_path, card=_card(**switch_path))
    report = MOD.audit(root, head=HEAD)
    assert report["exit_code"] == 1
    assert "research-switches" in _ids(report, "hard_failures")


def test_d435_depth_on_is_hard(tmp_path):
    card = _card()
    card["task"] = {**card["task"], "d435_depth": True}
    root = make_tree(tmp_path, card=card)
    report = MOD.audit(root, head=HEAD)
    assert report["exit_code"] == 1
    assert "research-switches" in _ids(report, "hard_failures")
    assert "d435_depth" in report["hard_failures"][0]["detail"]


def test_non_bool_switch_is_hard(tmp_path):
    root = make_tree(tmp_path, card=_card(policy_params={"upstream": {"track_clearance": "true"}}))
    report = MOD.audit(root, head=HEAD)
    assert report["exit_code"] == 1
    assert "不是布尔" in report["hard_failures"][0]["detail"]


def test_explicit_false_switch_still_passes(tmp_path):
    root = make_tree(tmp_path, card=_card(policy_params={"upstream": {"empty_brake": False}}))
    report = MOD.audit(root, head=HEAD)
    assert report["exit_code"] == 0


def test_backend_outside_suite_is_phase2(tmp_path):
    root = make_tree(tmp_path)
    extra = _card(name="bspline")
    extra["policy_params"] = {"backend": {"enabled": True}}
    (root / "scenes/bspline.json").write_text(json.dumps(extra), encoding="utf-8")
    report = MOD.audit(root, head=HEAD)
    assert report["exit_code"] == 0
    arms = next(item for item in report["phase2"] if item["id"] == "research-arms")
    assert any(card["path"].endswith("bspline.json") for card in arms["cards"])


def test_backend_inside_suite_is_hard(tmp_path):
    raw = _card()
    raw["policy_params"] = {"backend": {"enabled": True}}
    root = make_tree(tmp_path, card=raw)
    report = MOD.audit(root, head=HEAD)
    assert report["exit_code"] == 1
    assert "suite" in _ids(report, "hard_failures")


def test_broken_link_is_hard(tmp_path):
    root = make_tree(tmp_path)
    (root / "docs/LIMITATIONS.md").write_text("见 [缺失](missing-page.md)。\n", encoding="utf-8")
    report = MOD.audit(root, head=HEAD)
    assert report["exit_code"] == 1
    assert "doc-links" in _ids(report, "hard_failures")


def test_missing_suite_card_is_hard(tmp_path):
    root = make_tree(tmp_path)
    (root / "scenes/suite.txt").write_text("scenes/missing.json@upstream\n", encoding="utf-8")
    report = MOD.audit(root, head=HEAD)
    assert report["exit_code"] == 1
    assert "suite" in _ids(report, "hard_failures")


def test_student_in_suite_is_hard(tmp_path):
    root = make_tree(tmp_path)
    (root / "scenes/suite.txt").write_text("scenes/hover.json@student\n", encoding="utf-8")
    report = MOD.audit(root, head=HEAD)
    assert report["exit_code"] == 1
    assert "student" in report["hard_failures"][0]["detail"]


def test_vendored_mismatch_is_hard(tmp_path):
    root = make_tree(tmp_path)
    (root / "core/src/pin.txt").write_text("changed\n", encoding="utf-8")
    report = MOD.audit(root, head=HEAD)
    assert report["exit_code"] == 1
    assert "vendored-pin" in _ids(report, "hard_failures")


def test_runner_shadow_off_is_hard(tmp_path):
    root = make_tree(tmp_path)
    (root / "yopo_l0/runner.py").write_text(
        'class RunConfig:\n    policy: str = "upstream"\n    shadow: bool = False\n',
        encoding="utf-8",
    )
    report = MOD.audit(root, head=HEAD)
    assert report["exit_code"] == 1
    assert "runner-default" in _ids(report, "hard_failures")


def test_regress_no_shadow_on_suite_is_hard(tmp_path):
    root = make_tree(tmp_path)
    (root / "scripts/regress.sh").write_text(
        '$PY -m yopo_l0.batch "${SUITE[@]}" --no-shadow --out "$OUT"\n',
        encoding="utf-8",
    )
    report = MOD.audit(root, head=HEAD)
    assert report["exit_code"] == 1
    assert "regress-suite" in _ids(report, "hard_failures")


def test_safe_radius_deviation_is_hard(tmp_path):
    root = make_tree(tmp_path, card=_card(policy_params={"upstream": {"safe_radius": 0.2}}))
    report = MOD.audit(root, head=HEAD)
    assert report["exit_code"] == 1
    assert "safe_radius" in report["hard_failures"][0]["detail"]


def test_smoke_failure_is_hard_only_when_requested(tmp_path):
    root = make_tree(tmp_path)
    calls = {"n": 0}

    def smoke(_root):
        calls["n"] += 1
        return {"ok": False, "error": "boom", "policy": "hover", "shadow": False}

    quiet = MOD.audit(root, head=HEAD, smoke_fn=smoke)
    assert quiet["exit_code"] == 0
    assert calls["n"] == 0
    loud = MOD.audit(root, head=HEAD, with_smoke=True, smoke_fn=smoke)
    assert loud["exit_code"] == 1
    assert calls["n"] == 1
    assert loud["mode"] == "smoke"


def test_smoke_success_is_a_pass(tmp_path):
    root = make_tree(tmp_path)

    def smoke(_root):
        return {"ok": True, "policy": "upstream", "shadow": True, "scene": "scenes/g2-hover.json", "duration_s": 0.2}

    report = MOD.audit(root, head=HEAD, with_smoke=True, smoke_fn=smoke)
    assert report["exit_code"] == 0
    assert "smoke" in _ids(report, "passed")


def test_default_cli_does_not_write(tmp_path, capsys):
    root = make_tree(tmp_path)
    before = {path.relative_to(root).as_posix() for path in root.rglob("*") if path.is_file()}
    code = MOD.main(["--root", str(root)])
    after = {path.relative_to(root).as_posix() for path in root.rglob("*") if path.is_file()}
    captured = capsys.readouterr()
    assert code == 0
    assert before == after
    assert "## 通过项" in captured.out
    assert "\n---\n" in captured.out
    payload = captured.out.split("\n---\n", 1)[1]
    assert json.loads(payload)["exit_code"] == 0


def test_current_main_static_check():
    report = MOD.audit(ROOT)
    assert report["exit_code"] == 0, report["hard_failures"]
    assert report["mode"] == "static"
    assert report["hard_failures"] == []
    assert "research-switches-off" in _ids(report, "passed")
    assert "scene-card-hashes" in _ids(report, "passed")
    assert "vendored-pin" in _ids(report, "passed")
    assert "doc-links" in _ids(report, "passed")
    assert "runner-default" in _ids(report, "passed")
    assert "suite" in _ids(report, "passed")
    text = (ROOT / "docs/results/SUMMARY.md").read_text(encoding="utf-8")
    cited = []
    in_table = False
    for line in text.splitlines():
        if line.startswith("## 总表"):
            in_table = True
            continue
        if in_table and line.startswith("## "):
            break
        if in_table and line.startswith("| M"):
            cited.append(line)
    assert cited
    # 基准提交从 SUMMARY.json 读，不写死某个提交号；总表每行都要引用它。
    summary = json.loads((ROOT / "docs/results/SUMMARY.json").read_text(encoding="utf-8"))
    base = summary["commit"]
    assert len(base) == 40 and int(base, 16) >= 0
    for line in cited:
        cell = line.split("|")[7].strip().strip("`")
        assert len(cell) >= 7 and base.startswith(cell), line
    old_rows = report["summary_rows_on_old_commit"]
    if MOD._commit_is_current(ROOT, base, report["head"]):
        assert old_rows == []
        assert "commits-current" in _ids(report, "passed")
    else:
        # 结果提交之后只合了文档或脚本时，HEAD 会领先基准提交：只记待刷新，场景卡哈希须仍一致。
        assert len(old_rows) == len(cited)
        assert all(base.startswith(row["commit"]) for row in old_rows)
        assert all(row["hash_matches_current"] for row in old_rows)
        assert "stale-commits" in _ids(report, "refresh")
    phase2 = " ".join(item["title"] + item["detail"] for item in report["phase2"])
    assert "SITL" in phase2 and "蒸馏" in phase2
    arms = next(item for item in report["phase2"] if item["id"] == "research-arms")
    names = {Path(card["path"]).name for card in arms["cards"]}
    assert "g2-pillar-bspline.json" in names
    assert "crossing-forest-dense-bspline.json" in names
    assert "swap-3-geom-fallback.json" in names


def test_docs_only_advance_keeps_commits_current_until_code_changes(tmp_path):
    import subprocess

    root = make_tree(tmp_path)
    git = lambda *args: subprocess.run(
        ["git", *args], cwd=root, check=True, capture_output=True, text=True
    ).stdout.strip()
    git("init", "-q")
    git("config", "user.name", "test")
    git("config", "user.email", "test@example.com")
    git("add", ".")
    git("commit", "-q", "-m", "code base")
    c_code = git("rev-parse", "HEAD")

    make_tree(root, commit=c_code)
    (root / "docs/LIMITATIONS.md").write_text("见 [总表](results/SUMMARY.md)。刷新说明。\n", encoding="utf-8")
    git("add", "docs")
    git("commit", "-q", "-m", "docs refresh")
    c_docs = git("rev-parse", "HEAD")
    assert c_docs != c_code

    fresh = MOD.audit(root)
    assert fresh["exit_code"] == 0
    assert fresh["counts"]["hard"] == 0
    assert fresh["counts"]["refresh"] == 0
    assert "commits-current" in _ids(fresh, "passed")

    (root / "yopo_l0/runner.py").write_text(
        'class RunConfig:\n    policy: str = "upstream"\n    shadow: bool = True\n# code touched\n',
        encoding="utf-8",
    )
    git("add", "yopo_l0/runner.py")
    git("commit", "-q", "-m", "touch yopo_l0")

    stale = MOD.audit(root)
    assert stale["exit_code"] == 0
    assert stale["counts"]["hard"] == 0
    assert stale["counts"]["refresh"] == 1
    assert "stale-commits" in _ids(stale, "refresh")
