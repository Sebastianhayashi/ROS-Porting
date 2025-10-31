#!/usr/bin/env python3
import os
import sys
import time
import requests

API = "https://gitee.com/api/v5"
TOKEN = os.getenv("GITEE_TOKEN") or input("请输入 Gitee Access Token: ").strip()
SESSION = requests.Session()
SESSION.headers.update({"Authorization": f"Bearer {TOKEN}"})

def fetch_user_owned_repos():
    """分页获取当前用户**拥有**的仓库（避免误删协作/组织的仓库）"""
    repos = []
    page = 1
    per_page = 100
    # Gitee 的 OpenAPI 支持分页；SDK 文档也展示了用于过滤归属的参数（affiliation 等）。 [oai_citation:2‡Gitee](https://gitee.com/sdk/gitee5j/blob/main/docs/RepositoriesApi.md?utm_source=chatgpt.com)
    while True:
        params = {
            "page": page,
            "per_page": per_page,
            # 仅取自己拥有的仓库，避免误删协作或组织下但非本人所有的项目
            "affiliation": "owner"
        }
        r = SESSION.get(f"{API}/user/repos", params=params, timeout=30)
        r.raise_for_status()
        batch = r.json()
        if not batch:
            break
        repos.extend(batch)
        page += 1
    return repos

def delete_repo(full_name):
    # full_name 形如 "owner/repo"，DELETE /repos/{owner}/{repo}。 [oai_citation:3‡Gitee](https://gitee.com/sdk/gitee5j/blob/main/docs/RepositoriesApi.md?utm_source=chatgpt.com)
    url = f"{API}/repos/{full_name}"
    r = SESSION.delete(url, timeout=30)
    # 文档与社区实践通常返回 204 表示成功。 [oai_citation:4‡Cnblogs](https://www.cnblogs.com/BeiBao03/p/17985230?utm_source=chatgpt.com)
    if r.status_code == 204:
        print(f"✔ deleted: {full_name}")
        return True
    else:
        print(f"✘ failed:  {full_name}  status={r.status_code}  body={r.text[:200]}")
        return False

def main():
    repos = fetch_user_owned_repos()
    if not repos:
        print("未发现可删除的个人仓库（affiliation=owner）。已退出。")
        return

    to_delete = [r.get("full_name") or f"{(r.get('owner') or {}).get('login')}/{r.get('name')}" for r in repos]
    print("\n将要删除以下仓库（共 %d 个）：" % len(to_delete))
    for name in to_delete:
        print("  -", name)

    print("\n⚠️ 以上操作不可恢复！10 秒后继续，或 Ctrl+C 终止。")
    try:
        time.sleep(10)
    except KeyboardInterrupt:
        print("\n用户取消。")
        return

    ok = 0
    for name in to_delete:
        if delete_repo(name):
            ok += 1
    print(f"\n完成：成功 {ok} / 共 {len(to_delete)}")

if __name__ == "__main__":
    main()