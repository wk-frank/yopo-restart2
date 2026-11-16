"""票 C2：状态脚本只读，四段输出互不影响，并动态解析 origin 仓库 slug。"""

import os
import re
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
STATUS = ROOT / "scripts" / "status.sh"


def _snapshot(root: Path) -> dict[str, bytes | None]:
    return {
        str(path.relative_to(root)): path.read_bytes() if path.is_file() else None
        for path in root.rglob("*")
    }


def _run(
    tmp_path: Path,
    ops: Path,
    gh: Path,
    wt: Path,
    *,
    repo_dir: Path | None = None,
    set_repo_env: bool = True,
    cwd: Path | None = None,
) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env.pop("YOPO_GH_REPO", None)
    env.pop("YOPO_REPO_DIR", None)
    env.update(
        YOPO_OPS_DIR=str(ops),
        GH=str(gh),
        WT_YOPO=str(wt),
    )
    if set_repo_env:
        env["YOPO_REPO_DIR"] = str(repo_dir if repo_dir is not None else tmp_path / "repo")
    return subprocess.run(
        [str(STATUS)],
        cwd=cwd if cwd is not None else tmp_path,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )


def _section(output: str, heading: str, next_heading: str) -> str:
    return output.split(heading + "\n", 1)[1].split(next_heading + "\n", 1)[0]


def test_status_snapshot_is_read_only(tmp_path):
    ops = tmp_path / "ops"
    nested = ops / "nested"
    nested.mkdir(parents=True)
    (tmp_path / "repo").mkdir()

    queue = [f"队列-{i:02d}" for i in range(1, 13)]
    results = [f"结果-{i:02d}" for i in range(1, 13)]
    (ops / "queue.txt").write_text("\n".join(queue) + "\n", encoding="utf-8")
    (ops / "results.tsv").write_text("\n".join(results) + "\n", encoding="utf-8")

    alive_pid = os.getpid()
    dead = subprocess.Popen(["/bin/true"])
    dead.wait()
    dead_pid = dead.pid
    (ops / "daemon.pid").write_text(f"{alive_pid}\n", encoding="utf-8")
    (ops / "daemon.log").write_text("旧行\n活日志末行\n", encoding="utf-8")
    (nested / "x-premerge.pid").write_text(f"{dead_pid}\n", encoding="utf-8")
    (nested / "x-premerge.log").write_text("旧行\n死日志末行\n", encoding="utf-8")
    (ops / "no-log.pid").write_text(f"{dead_pid}\n", encoding="utf-8")

    gh = tmp_path / "fake-gh"
    gh.write_text(
        "#!/usr/bin/env bash\n"
        "[[ \"$*\" == 'pr list -R unununnnn/yopo-restart --state open --json number,title,headRefOid' ]] || exit 2\n"
        "printf '%s\\n' '[{\"number\":42,\"title\":\"排队检查\",\"headRefOid\":\"abcdef0123456789\"}]'\n",
        encoding="utf-8",
    )
    gh.chmod(0o755)
    wt = tmp_path / "fake-wt"
    wt.write_text(
        "#!/usr/bin/env bash\n"
        '[[ "$#" -eq 3 && "$1" == -C && "$2" == "$YOPO_REPO_DIR" && "$3" == list ]] || exit 2\n'
        "printf '%s\\n' '工作树记录'\n",
        encoding="utf-8",
    )
    wt.chmod(0o755)

    before = _snapshot(tmp_path)
    proc = _run(tmp_path, ops, gh, wt)
    assert _snapshot(tmp_path) == before
    assert proc.returncode == 0, proc.stderr
    assert re.search(r"^生成时间：\d{4}-\d\d-\d\d \d\d:\d\d:\d\d UTC-3$", proc.stdout, re.M)
    assert proc.stdout.count("\n## ") == 4

    prs = _section(proc.stdout, "## 开放 PR", "## 队列与结果")
    assert "| 号 | 标题 | head sha | CI |" in prs
    assert "| #42 | 排队检查 | abcdef0 | 无 |" in prs

    files = _section(proc.stdout, "## 队列与结果", "## 进程")
    queue_part = files.split("### queue.txt\n", 1)[1].split("### results.tsv\n", 1)[0]
    results_part = files.split("### results.tsv\n", 1)[1]
    assert [line for line in queue_part.splitlines() if line.startswith("队列-")] == queue[-8:]
    assert [line for line in results_part.splitlines() if line.startswith("结果-")] == results[-8:]

    pids = _section(proc.stdout, "## 进程", "## 工作树")
    assert f"{ops / 'daemon.pid'} | PID：{alive_pid} | 活 | 日志：活日志末行" in pids
    assert f"{nested / 'x-premerge.pid'} | PID：{dead_pid} | 死 | 日志：死日志末行" in pids
    assert f"{ops / 'no-log.pid'} | PID：{dead_pid} | 死 | 日志：无日志" in pids
    assert "工作树记录" in proc.stdout
    assert "pgrep" not in STATUS.read_text(encoding="utf-8")


@pytest.mark.parametrize(
    ("remote_url", "expected_slug"),
    [
        ("https://github.com/wk-frank/yopo-restart2", "wk-frank/yopo-restart2"),
        ("https://github.com/wk-frank/yopo-restart2.git", "wk-frank/yopo-restart2"),
        ("git@github.com:wk-frank/yopo-restart2.git", "wk-frank/yopo-restart2"),
        ("ssh://git@github.com/wk-frank/yopo-restart2.git", "wk-frank/yopo-restart2"),
        ("/tmp/local-bare-origin", "unununnnn/yopo-restart"),
    ],
)
def test_status_resolves_repo_slug_from_origin_with_fallback(tmp_path, remote_url, expected_slug):
    ops = tmp_path / "ops"
    ops.mkdir()
    repo = tmp_path / "repo"
    subprocess.run(["git", "init", "-q", str(repo)], check=True)
    subprocess.run(["git", "-C", str(repo), "remote", "add", "origin", remote_url], check=True)

    gh = tmp_path / "fake-gh"
    gh.write_text(
        "#!/usr/bin/env bash\n"
        f"[[ \"$*\" == 'pr list -R {expected_slug} --state open --json number,title,headRefOid' ]] || exit 2\n"
        "printf '%s\\n' '[{\"number\":7,\"title\":\"动态仓库\",\"headRefOid\":\"1234567890abcdef\"}]'\n",
        encoding="utf-8",
    )
    gh.chmod(0o755)
    wt = tmp_path / "missing-wt"

    before = _snapshot(tmp_path)
    proc = _run(tmp_path, ops, gh, wt, repo_dir=repo)
    assert _snapshot(tmp_path) == before
    assert proc.returncode == 0, proc.stderr
    prs = _section(proc.stdout, "## 开放 PR", "## 队列与结果")
    assert "| #7 | 动态仓库 | 1234567 | 无 |" in prs


def test_status_resolves_checkout_origin_when_repo_dir_env_unset(tmp_path):
    ops = tmp_path / "ops"
    ops.mkdir()
    checkout = tmp_path / "checkout"
    subprocess.run(["git", "init", "-q", str(checkout)], check=True)
    subprocess.run(
        ["git", "-C", str(checkout), "remote", "add", "origin", "https://github.com/wk-frank/yopo-restart2.git"],
        check=True,
    )

    gh = tmp_path / "fake-gh"
    gh.write_text(
        "#!/usr/bin/env bash\n"
        "[[ \"$*\" == 'pr list -R wk-frank/yopo-restart2 --state open --json number,title,headRefOid' ]] || exit 2\n"
        "printf '%s\\n' '[]'\n",
        encoding="utf-8",
    )
    gh.chmod(0o755)
    wt = tmp_path / "missing-wt"

    before = _snapshot(tmp_path)
    proc = _run(tmp_path, ops, gh, wt, set_repo_env=False, cwd=checkout)
    assert _snapshot(tmp_path) == before
    assert proc.returncode == 0, proc.stderr
    prs = _section(proc.stdout, "## 开放 PR", "## 队列与结果")
    assert "（无）" in prs


@pytest.mark.parametrize(
    ("gh_name", "wt_name", "gh_message", "wt_message"),
    [
        ("missing-gh", "/bin/false", "（gh 不可用）", "（wt-yopo-restart 查询失败）"),
        ("/bin/false", "missing-wt", "（gh 查询失败）", "（wt-yopo-restart 不可用）"),
    ],
)
def test_status_continues_when_tools_or_files_are_missing(
    tmp_path, gh_name, wt_name, gh_message, wt_message
):
    ops = tmp_path / "ops"
    ops.mkdir()
    gh = Path(gh_name) if gh_name.startswith("/") else tmp_path / gh_name
    wt = Path(wt_name) if wt_name.startswith("/") else tmp_path / wt_name
    before = _snapshot(tmp_path)
    proc = _run(tmp_path, ops, gh, wt)
    assert _snapshot(tmp_path) == before
    assert proc.returncode == 0, proc.stderr
    assert gh_message in proc.stdout
    assert wt_message in proc.stdout
    assert proc.stdout.count("（无）") == 3
    assert proc.stdout.count("\n## ") == 4
