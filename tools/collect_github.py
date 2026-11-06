"""Collect relevant public GitHub repositories and shallow clone to referensi/github.

All artifacts remain within RCauto. The catalog records failed clones transparently.
"""

import argparse
import concurrent.futures
import csv
import json
import os
from pathlib import Path
import re
import subprocess
import time

import requests

QUERIES = [
    "ros2 lane detection autonomous driving",
    "autonomous rc car ROS2",
    "f1tenth ros2",
    "donkeycar raspberry pi autonomous",
    "hailo rpi5 detection",
    "yolov8 traffic light detection",
    "ros2 traffic light detection",
    "ros2 ackermann steering",
    "ros2 fuzzy pid line follower",
    "raspberry pi autonomous lane following",
    "ros2 object detection",
    "self driving autonomous car lane following",
    "ros2 navigation autonomous driving",
    "ros2 foxglove visualization",
]
PINNED = [
    "hailo-ai/hailo-rpi5-examples",
    "hailo-ai/hailo-apps",
    "foxglove/foxglove-sdk",
    "ROBOTIS-GIT/turtlebot3_autorace",
    "f1tenth/f1tenth_gym_ros",
    "f1tenth/f1tenth_system",
    "autowarefoundation/autoware",
    "autowarefoundation/autoware.universe",
    "donkeycar/donkeycar",
]


def github_get(session, url, params=None):
    for retry in range(4):
        try:
            r = session.get(url, params=params, timeout=25)
            if r.status_code in (403, 429):
                reset = int(r.headers.get("X-RateLimit-Reset", 0))
                if retry == 3:
                    return None
                pause = max(6, min(60, reset - int(time.time()) + 1))
                time.sleep(pause)
                continue
            r.raise_for_status()
            return r.json()
        except requests.RequestException:
            time.sleep(3 * (retry + 1))
    return None


def repo_record(item, pinned=False, query=""):
    return {
        "repo": item.get("full_name", ""),
        "url": item.get("html_url", ""),
        "description": (item.get("description") or "").replace("\n", " ")[:400],
        "stars": item.get("stargazers_count", 0),
        "size_kb": item.get("size", 0),
        "language": item.get("language") or "",
        "license": (item.get("license") or {}).get("spdx_id") or "",
        "updated_at": item.get("updated_at") or "",
        "default_branch": item.get("default_branch") or "",
        "query": query,
        "pinned": pinned,
        "clone": "not_attempted",
    }


def clone_repo(base, record):
    name = record["repo"]
    if not re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", name):
        record["clone"] = "invalid_name"
        return record
    owner, repo = name.split("/")
    parent = base / "repositories" / owner
    parent.mkdir(parents=True, exist_ok=True)
    target = parent / repo
    if (target / ".git").exists():
        record["clone"] = "already_present"
        return record
    if record["size_kb"] > 180000:
        record["clone"] = "metadata_only_large_repo"
        return record
    if target.exists():
        record["clone"] = "target_exists_no_overwrite"
        return record
    url = f"https://github.com/{name}.git"
    try:
        run = subprocess.run(
            ["git", "-c", "http.lowSpeedLimit=1024", "-c", "http.lowSpeedTime=20",
             "clone", "--depth=1", "--filter=blob:none", "--single-branch",
             "--quiet", url, str(target)],
            capture_output=True, text=True, timeout=95,
            env=dict(os.environ, GIT_TERMINAL_PROMPT="0"),
        )
        if run.returncode == 0 and (target / ".git").is_dir():
            record["clone"] = "ok"
        else:
            record["clone"] = "failed_" + (run.stderr.strip()[:110] or str(run.returncode))
            if target.exists() and not (target / ".git").exists():
                import shutil
                shutil.rmtree(target)
    except subprocess.TimeoutExpired:
        record["clone"] = "timeout"
    return record


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--clone-limit", type=int, default=40)
    args = ap.parse_args()
    base = args.root.resolve() / "referensi" / "github"
    base.mkdir(parents=True, exist_ok=True)
    session = requests.Session()
    session.headers["Accept"] = "application/vnd.github+json"
    session.headers["User-Agent"] = "RCauto-academic-research"
    if os.getenv("GITHUB_TOKEN"):
        session.headers["Authorization"] = "Bearer " + os.getenv("GITHUB_TOKEN")
    selected = {}
    for full in PINNED:
        obj = github_get(session, "https://api.github.com/repos/" + full)
        if obj and obj.get("full_name"):
            selected[obj["full_name"].lower()] = repo_record(obj, True, "official")
    for idx, query in enumerate(QUERIES):
        obj = github_get(session, "https://api.github.com/search/repositories",
                         {"q": query + " in:name,description", "sort": "stars",
                          "order": "desc", "per_page": 35})
        if obj:
            for item in obj.get("items", []):
                if item.get("fork") or item.get("archived") or not item.get("full_name"):
                    continue
                key = item["full_name"].lower()
                if key not in selected:
                    selected[key] = repo_record(item, False, query)
        print(f"github query {idx+1}/{len(QUERIES)}; catalog {len(selected)}", flush=True)
        # Avoid hitting the unauthenticated GitHub search-rate limit.
        time.sleep(6)
    rows = list(selected.values())
    # Pinned repositories first; star count and last update reward active work.
    rows.sort(key=lambda r: (not r["pinned"], -min(20000, r["stars"]), r["repo"]))
    selected_clones = [x for x in rows if x["size_kb"] <= 180000][:max(0, args.clone_limit)]
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        futures = [executor.submit(clone_repo, base, x) for x in selected_clones]
        for idx, result in enumerate(concurrent.futures.as_completed(futures), 1):
            row = result.result()
            print(f"clone {idx}/{len(selected_clones)} {row['repo']}: {row['clone']}", flush=True)
    (base / "github_catalog.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    fields = list(rows[0]) if rows else []
    with (base / "github_catalog.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print("SUMMARY", json.dumps({
        "found": len(rows),
        "cloned": sum(x["clone"] in ("ok", "already_present") for x in rows),
        "failed": sum(x["clone"].startswith("failed") or x["clone"] == "timeout" for x in rows),
        "path": str(base),
    }), flush=True)


if __name__ == "__main__":
    main()
