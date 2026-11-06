"""Curated, DOI-deduplicated academic bibliography with SJR-2025 quartiles.

Quartile describes the JOURNAL in SJR 2025, not an individual paper.
Only open/license-permitted accessible PDFs are downloaded. Missing PDFs remain
metadata entries with verifiable DOI; never substitute fabricated text.
"""

import argparse
import concurrent.futures
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import time
from urllib.parse import quote

import pandas as pd
import requests

TOPICS = [
    "lane detection autonomous driving",
    "lane keeping automotive steering control",
    "lane following robot vision",
    "lane segmentation deep learning autonomous",
    "end to end autonomous driving steering",
    "autonomous RC vehicle Raspberry Pi",
    "visual navigation intelligent mobile robot",
    "traffic light detection recognition deep learning",
    "traffic signal recognition autonomous vehicle",
    "intersection detection autonomous vehicle",
    "pedestrian obstacle detection self driving",
    "object detection embedded autonomous driving",
    "path tracking PID controller autonomous vehicle",
    "fuzzy logic autonomous vehicle steering",
    "adaptive neuro fuzzy lane following",
    "deep reinforcement learning lane keeping",
    "finite state machine autonomous driving safety",
    "behavior planning autonomous vehicles intersection",
    "trajectory following Ackermann mobile robot",
    "computer vision stop line detection",
    "model predictive control autonomous vehicles",
    "real time semantic segmentation autonomous driving",
    "embedded edge AI object detection traffic",
    "monocular collision avoidance robotic cars",
    "vehicle autonomous navigation deep learning",
]
KEYWORDS = (
    "lane", "steer", "traffic", "driv", "intersection", "obstacle", "pedestrian",
    "robot", "vehicle", "autonom", "fuzzy", "navigation", "road", "trajectory",
    "motor", "path tracking", "reinforcement learning", "segmentation",
    "edge ai", "embedded", "visual perception",
)
HEADERS = {"User-Agent": "RCautoLiteratureCollection/1.0 (academic research; DOI citation)"}


def clean_issn(value):
    return re.sub(r"[^0-9X]", "", str(value or "").upper())


def quartiles(path):
    if not path.exists():
        print("SJR reference unavailable: journal quartiles left blank", flush=True)
        return {}
    df = pd.read_parquet(path, columns=[
        "year", "issn_print", "issn_online", "issn_raw", "title_raw",
        "best_quartile_raw", "category_quartile_numeric", "source_type_standardized",
    ])
    df = df[(df.year == 2025) & (df.source_type_standardized == "journal")].copy()
    df["quartile_number"] = pd.to_numeric(df.category_quartile_numeric, errors="coerce")
    def convert(row):
        best = str(row.best_quartile_raw or "").upper()
        if best in ("Q1", "Q2", "Q3", "Q4"):
            return best
        if pd.notna(row.quartile_number) and 1 <= int(row.quartile_number) <= 4:
            return "Q" + str(int(row.quartile_number))
        return ""
    df["q"] = df.apply(convert, axis=1)
    df = df[df.q != ""]
    rank = {}
    for row in df.itertuples():
        for raw in (row.issn_print, row.issn_online):
            issn = clean_issn(raw)
            if len(issn) == 8:
                found = rank.get(issn)
                if found is None or int(row.q[1]) < int(found[0][1]):
                    rank[issn] = (row.q, str(row.title_raw))
    print("SJR 2025 ISSN map size:", len(rank), flush=True)
    return rank


def pub_year(work):
    dates = work.get("published", {}).get("date-parts") or []
    return int(dates[0][0]) if dates and dates[0] else None


def paper_from_crossref(item, topic, sjr):
    title = (item.get("title") or [""])[0]
    title = re.sub(r"<[^>]+>", "", title).strip()
    if len(title) < 20 or not any(k in title.casefold() for k in KEYWORDS):
        return None
    year = pub_year(item)
    if year is None or year < 2019 or year > 2026:
        return None
    doi = (item.get("DOI") or "").strip().lower()
    if not doi or " " in doi:
        return None
    matched = [sjr[clean_issn(i)] for i in item.get("ISSN", [])
               if clean_issn(i) in sjr]
    if not matched:
        return None
    best = min(matched, key=lambda x: int(x[0][1]))
    if not best[0]:
        return None
    links = item.get("link", [])
    licenses = [x.get("URL", "") for x in item.get("license", [])]
    cc_license = any("creativecommons.org/" in x.lower() for x in licenses)
    pdf = ""
    if cc_license:
        for link in links:
            if link.get("content-type", "").lower() == "application/pdf" and link.get("URL"):
                pdf = link["URL"]
                break
    return {
        "doi": doi,
        "title": title,
        "year": year,
        "journal": (item.get("container-title") or [""])[0],
        "sjr2025_quartile": best[0],
        "sjr2025_journal": best[1],
        "issn": "; ".join(item.get("ISSN", [])),
        "citations_crossref": int(item.get("is-referenced-by-count") or 0),
        "topic": topic,
        "doi_url": "https://doi.org/" + quote(doi, safe="/"),
        "oa_pdf_url": pdf,
        "license_url": "; ".join(licenses)[:250],
        "pdf_status": "not_attempted",
        "file": "",
    }


def crossref_search(session, topic, rows=160):
    params = {
        "query.title": topic,
        "filter": "from-pub-date:2019-01-01,until-pub-date:2026-11-06,type:journal-article",
        "rows": rows,
        "select": "DOI,title,published,container-title,ISSN,is-referenced-by-count,URL,link,license",
    }
    for i in range(3):
        try:
            response = session.get("https://api.crossref.org/works", params=params, timeout=28)
            if response.status_code == 200:
                return response.json()["message"]["items"]
            print("Crossref HTTP", response.status_code, topic, flush=True)
        except requests.RequestException as exc:
            print("Crossref retry", str(exc)[:90], flush=True)
        time.sleep((i + 1) * 2)
    return []


def enrich_oa(rows):
    """Best-effort anonymous OpenAlex OA enrichment; short timeout on 429."""
    by_doi = {row["doi"]: row for row in rows}
    topic_queries = ["lane detection autonomous driving", "traffic light detection autonomous",
                     "fuzzy steering autonomous vehicles", "pedestrian detection edge ai"]
    sess = requests.Session()
    sess.headers.update(HEADERS)
    for topic in topic_queries:
        try:
            res = sess.get("https://api.openalex.org/works",
                           params={"search": topic,
                                   "filter": "from_publication_date:2019-01-01,is_oa:true,type:article",
                                   "per-page": 100},
                           timeout=18)
            if res.status_code == 429:
                print("OpenAlex rate limit: OA enrichment skipped (no fabricated PDFs)", flush=True)
                return
            if res.status_code != 200:
                continue
            for work in res.json().get("results", []):
                doi = (work.get("doi") or "").lower().removeprefix("https://doi.org/")
                if doi not in by_doi or by_doi[doi]["oa_pdf_url"]:
                    continue
                loc = work.get("best_oa_location") or {}
                candidate = loc.get("pdf_url") or ""
                if candidate.startswith("https://") or candidate.startswith("http://"):
                    by_doi[doi]["oa_pdf_url"] = candidate
            time.sleep(2)
        except requests.RequestException:
            continue


def download_pdf(folder, row):
    url = row.get("oa_pdf_url", "")
    if not url:
        row["pdf_status"] = "no_verified_oa_pdf_url"
        return row
    slug = re.sub(r"[^a-zA-Z0-9]+", "_", row["title"])[:65].strip("_")
    suffix = hashlib.sha256(row["doi"].encode()).hexdigest()[:10]
    target = folder / f"{row['year']}_{slug}_{suffix}.pdf"
    if target.exists() and target.stat().st_size >= 3000:
        row["pdf_status"] = "present"
        row["file"] = target.name
        return row
    try:
        response = requests.get(url, headers=HEADERS, timeout=(9, 22), stream=True,
                                allow_redirects=True)
        if response.status_code != 200:
            row["pdf_status"] = "http_" + str(response.status_code)
            return row
        data = bytearray()
        for chunk in response.iter_content(262144):
            data.extend(chunk)
            if len(data) > 24 * 1024 * 1024:
                row["pdf_status"] = "too_large"
                return row
        if not data.lstrip().startswith(b"%PDF-") or len(data) < 3000:
            row["pdf_status"] = "response_not_pdf"
            return row
        target.write_bytes(data)
        row["pdf_status"] = "downloaded"
        row["file"] = target.name
        return row
    except requests.RequestException as exc:
        row["pdf_status"] = "download_failed_" + type(exc).__name__
        return row


def export(rows, target):
    target.mkdir(parents=True, exist_ok=True)
    (target / "paper_catalog.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")
    with (target / "paper_catalog.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--target", type=int, default=300)
    ap.add_argument("--pdf-limit", type=int, default=300)
    args = ap.parse_args()
    folder = args.root.resolve() / "referensi" / "paper"
    folder.mkdir(parents=True, exist_ok=True)
    pdf_folder = folder / "pdf"
    pdf_folder.mkdir(parents=True, exist_ok=True)
    sjr = quartiles(folder / "sjr_category_panel.parquet")
    session = requests.Session()
    session.headers.update(HEADERS)
    found = {}
    for index, topic in enumerate(TOPICS, 1):
        work_items = crossref_search(session, topic, rows=160)
        for item in work_items:
            row = paper_from_crossref(item, topic, sjr)
            if row and row["doi"] not in found:
                found[row["doi"]] = row
        print(f"Crossref {index}/{len(TOPICS)} {topic}: {len(work_items)} raw, {len(found)} SJR verified", flush=True)
        if len(found) > args.target * 3 and index >= 12:
            break
        time.sleep(0.5)
    all_rows = list(found.values())
    # SJR quartiles are source-level rankings; diversify by use case and quartile.
    # Strong Q1 majority, with verified Q2/Q3/Q4 samples for comparison.
    (folder / "paper_candidates.json").write_text(
        json.dumps(all_rows, ensure_ascii=False, indent=2), encoding="utf-8")

    def relevance(row):
        title = row["title"].lower()
        strong = ("lane", "steer", "traffic light", "intersection", "pedestrian",
                  "line follow", "fuzzy", "ackermann", "stop line")
        return (
            sum(4 for term in strong if term in title) +
            sum(1 for term in ("autonomous", "vehicle", "robot", "driving",
                               "real-time", "edge", "vision") if term in title) +
            math.log1p(row["citations_crossref"]) * 2.0 +
            max(0, row["year"] - 2020) * 1.0
        )

    by_q = {f"Q{i}": sorted(
        (r for r in all_rows if r["sjr2025_quartile"] == f"Q{i}"),
        key=relevance, reverse=True,
    ) for i in range(1, 5)}
    print("SJR pool", {k: len(v) for k, v in by_q.items()}, flush=True)
    ratios = {"Q1": .63, "Q2": .24, "Q3": .10, "Q4": .03}
    quota = {q: int(args.target * fraction) for q, fraction in ratios.items()}
    quota["Q1"] += args.target - sum(quota.values())
    rows = []
    picked = set()
    topic_counts = {}
    for q in ("Q1", "Q2", "Q3", "Q4"):
        for row in by_q[q]:
            if len([r for r in rows if r["sjr2025_quartile"] == q]) >= quota[q]:
                break
            if topic_counts.get(row["topic"], 0) >= 40:
                continue
            rows.append(row)
            picked.add(row["doi"])
            topic_counts[row["topic"]] = topic_counts.get(row["topic"], 0) + 1
    if len(rows) < args.target:
        leftovers = sorted((r for r in all_rows if r["doi"] not in picked),
                           key=lambda r: (5-int(r["sjr2025_quartile"][1]), relevance(r)),
                           reverse=True)
        rows.extend(leftovers[:args.target-len(rows)])
    if not rows:
        raise RuntimeError("No verifiable SJR-2025 articles; no fabricated catalog")
    export(rows, folder)
    print("CURATED", len(rows), "unique DOI, SJR 2025 verified", flush=True)
    enrich_oa(rows)
    papers_to_download = [row for row in rows if row["oa_pdf_url"]][:args.pdf_limit]
    print("OPEN_PDF_CANDIDATES", len(papers_to_download), flush=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
        for idx, future in enumerate(concurrent.futures.as_completed(
                [executor.submit(download_pdf, pdf_folder, r) for r in papers_to_download]), 1):
            r = future.result()
            if idx % 10 == 0:
                print(f"PDF {idx}/{len(papers_to_download)} {r['pdf_status']}", flush=True)
    for row in rows:
        if row["pdf_status"] == "not_attempted":
            row["pdf_status"] = "no_verified_oa_pdf_url"
    export(rows, folder)
    print("SUMMARY", json.dumps({
        "articles": len(rows),
        "downloaded": sum(r["pdf_status"] in ("downloaded", "present") for r in rows),
        "quartiles": {f"Q{i}": sum(r["sjr2025_quartile"] == f"Q{i}" for r in rows)
                      for i in range(1, 5)},
        "path": str(folder),
    }), flush=True)


if __name__ == "__main__":
    main()
