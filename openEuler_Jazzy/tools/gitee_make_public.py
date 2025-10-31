#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse, json, os, sys, urllib.parse, urllib.request, pathlib, time

API_BASE = "https://gitee.com/api/v5"

def patch_public(ns: str, repo: str, token: str) -> tuple[bool, str]:
    """
    调用 PATCH /repos/{ns}/{repo} ，必须带 name，private=false
    返回 (是否成功, 消息)
    """
    url = f"{API_BASE}/repos/{ns}/{repo}"
    data = {
        "access_token": token,
        "name": repo,          # 关键：Gitee 要求 PATCH 时必须带 name
        "private": "false",
    }
    body = urllib.parse.urlencode(data).encode()
    req = urllib.request.Request(url, data=body, method="PATCH",
                                 headers={"Content-Type": "application/x-www-form-urlencoded"})
    try:
        with urllib.request.urlopen(req, timeout=20) as resp:
            raw = resp.read()
            j = json.loads(raw.decode("utf-8"))
            if j.get("private") is False:
                return True, "public"
            return False, f"unexpected:{j.get('private')}"
    except urllib.error.HTTPError as e:
        try:
            j = json.loads(e.read().decode("utf-8"))
        except Exception:
            j = {"error": e.reason}
        return False, f"HTTP {e.code} {j}"
    except Exception as e:
        return False, f"exc:{e}"

def main():
    ap = argparse.ArgumentParser(description="Batch make Gitee repos public")
    ap.add_argument("--ns", required=True, help="Gitee 命名空间（用户名/组织名，必须全小写）")
    ap.add_argument("--token", default=os.environ.get("GITEE_TOKEN"),
                    help="Gitee API token，也可以用环境变量 GITEE_TOKEN")
    ap.add_argument("--dir", default=str(pathlib.Path.home()/ "ros-rpm" / "repos"),
                    help="本地包根目录（默认 ~/ros-rpm/repos）")
    ap.add_argument("--sleep", type=float, default=0.25, help="每次请求后的停顿秒数，避免限流")
    ap.add_argument("--dry-run", action="store_true", help="仅打印将要修改的仓库，不实际调用")
    args = ap.parse_args()

    if not args.token:
        print("ERROR: 需要 --token 或环境变量 GITEE_TOKEN", file=sys.stderr)
        sys.exit(2)

    base = pathlib.Path(args.dir)
    if not base.is_dir():
        print(f"ERROR: 目录不存在：{base}", file=sys.stderr)
        sys.exit(2)

    names = sorted([p.name for p in base.iterdir() if p.is_dir() and not p.name.startswith(".")])
    ok = fail = 0
    print(f"命名空间: {args.ns}")
    print(f"目录:     {base}  （仓库数={len(names)}）")
    if args.dry_run:
        for n in names:
            print(f"[DRY] would PATCH {args.ns}/{n} -> public")
        return

    for n in names:
        ok1, msg = patch_public(args.ns, n, args.token)
        if ok1:
            ok += 1
            print(f"OK    {n}: public")
        else:
            fail += 1
            print(f"FAIL  {n}: {msg}")
        time.sleep(args.sleep)

    print(f"\n完成：OK={ok}  FAIL={fail}")

if __name__ == "__main__":
    main()