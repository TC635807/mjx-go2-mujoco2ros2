#!/usr/bin/env python3
"""跑齐本仓库的所有自测（不需要 pytest）。

    python tests/run_tests.py            # 全部
    python tests/run_tests.py obs safety # 只跑名字里含这些关键字的
"""

from __future__ import annotations

import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS = os.path.join(ROOT, "tests")


def discover(filters=None):
    out = []
    for name in sorted(os.listdir(TESTS)):
        if not (name.startswith("test_") and name.endswith(".py")):
            continue
        if filters and not any(f in name for f in filters):
            continue
        out.append(name)
    return out


def main() -> int:
    filters = sys.argv[1:]
    names = discover(filters)
    if not names:
        print("没有匹配的测试")
        return 1
    print("发现 %d 个测试: %s" % (len(names), ", ".join(names)))
    results = []
    for name in names:
        t0 = time.time()
        p = subprocess.run([sys.executable, os.path.join(TESTS, name)],
                           cwd=ROOT, capture_output=True, text=True)
        dt = time.time() - t0
        tail = [l for l in (p.stdout or "").strip().splitlines() if l.strip()]
        last = tail[-1][:140] if tail else ""
        ok = p.returncode == 0
        results.append((name, ok, dt, last))
        print("%-6s %-34s %6.1fs  %s" % ("PASS" if ok else "FAIL", name, dt, last))
        if not ok:
            print("---- %s 失败输出（最后 30 行）----" % name)
            print("\n".join(tail[-30:]))
            if p.stderr:
                print("[stderr]", p.stderr.strip()[-1500:])
    n_ok = sum(1 for _, ok, _, _ in results if ok)
    print("\n===== %d/%d 个测试文件通过 =====" % (n_ok, len(results)))
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
