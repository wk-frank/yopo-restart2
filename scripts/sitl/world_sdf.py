#!/usr/bin/env python3
"""票 S8：从场景卡生成含多圆柱（及林带倾斜树干 / 矮枝）的 Gazebo Harmonic 世界 SDF。

只生成静态碰撞/视觉几何，不写任何飞控参数。供 ``in_container.sh`` 在 ``YOPO_SITL_SCENE_WORLD=1``
时把场景障碍注入 ``default.sdf``，也可离线单独调用做校验。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from yopo_l0.scene import load_card
from yopo_l0.sitl_bridge import build_scene_world_sdf, write_scene_world_sdf

_REPO = Path(__file__).resolve().parents[2]


def _resolve_scene(raw: str) -> Path:
    path = Path(raw)
    if path.is_file():
        return path
    alt = _REPO / raw
    if alt.is_file():
        return alt
    raise ValueError(f"找不到场景卡：{raw}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scene", required=True, help="场景卡 JSON 路径")
    ap.add_argument("--base-sdf", default=None, help="基础 default.sdf 路径（缺省用内置最小模板）")
    ap.add_argument("--out-dir", default=None, help="写出 <world>.sdf 的目录；省略时只输出摘要或 SDF")
    ap.add_argument("--world", default="default", help="SDF world 名称（默认 default）")
    ap.add_argument("--summary", default=None, help="把摘要 JSON 写到该路径")
    ap.add_argument("--print-sdf", action="store_true", help="把生成的完整 SDF 打印到 stdout")
    ap.add_argument("--no-branches", action="store_true", help="跳过矮枝等非墙体盒子，只放圆柱")
    args = ap.parse_args(argv)
    try:
        card = load_card(_resolve_scene(args.scene))
        include_branches = not bool(args.no_branches)
        if args.out_dir is not None:
            summary = write_scene_world_sdf(
                card,
                args.out_dir,
                base_sdf_path=args.base_sdf,
                world_name=args.world,
                include_branches=include_branches,
            )
            sdf_text = Path(summary["sdf_path"]).read_text(encoding="utf-8")
        else:
            base_text = None if args.base_sdf is None else Path(args.base_sdf).read_text(encoding="utf-8")
            sdf_text, summary = build_scene_world_sdf(
                card,
                base_sdf=base_text,
                world_name=args.world,
                include_branches=include_branches,
            )
    except (OSError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2

    if args.summary:
        out = Path(args.summary)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(summary, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    if args.print_sdf:
        sys.stdout.write(sdf_text)
    else:
        sys.stdout.write(json.dumps(summary, indent=1, ensure_ascii=False) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
