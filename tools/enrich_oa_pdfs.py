"""Use Semantic Scholar batch OA metadata to download legally accessible PDF copies.

The public API may rate-limit; all metadata and failures are recorded without
guessing paywalled download URLs. Restricted publisher downloads are not bypassed.
"""

from concurrent.futures import ThreadPoolExecutor, as_completed
import argparse
import json
from pathlib import Path
import time

import requests

from collect_papers import download_pdf


def batched(values, n):
    for offset in range(0, len(values), n):
        yield values[offset:offset + n]


def gather_oa(rows):
    by_doi = {row["doi"].lower(): row for row in rows}
    dois = sorted(by_doi)
    session = requests.Session()
    session.headers.update({"User-Agent": "RCautoResearch/1.0 (academic noncommercial)",
                            "Accept": "application/json"})
    matched = 0
    for batch in batched(dois, 90):
        for attempt in range(3):
            try:
                r = session.post(
                    "https://api.semanticscholar.org/graph/v1/paper/batch",
                    params={"fields": "title,openAccessPdf,externalIds,year"},
                    json={"ids": ["DOI:" + doi for doi in batch]},
                    timeout=40,
                )
                if r.status_code == 200:
                    break
                if r.status_code in (429, 500, 502, 503):
                    time.sleep((attempt + 1) * 12)
                    continue
                print("Semantic Scholar HTTP", r.status_code, flush=True)
                r = None
                break
            except requests.RequestException:
                r = None
                time.sleep(5 * (attempt + 1))
        if r is None or r.status_code != 200:
            continue
        for item in r.json():
            if not item:
                continue
            doi = str((item.get("externalIds") or {}).get("DOI") or "").lower()
            row = by_doi.get(doi)
            pdf = item.get("openAccessPdf") or {}
            url = str(pdf.get("url") or "")
            if row and url.startswith(("https://", "http://")):
                row["oa_pdf_url"] = url
                if not row.get("license_url") and pdf.get("license"):
                    row["license_url"] = "Semantic Scholar OA license: " + str(pdf["license"])
                matched += 1
        print(f"OA lookup: {len(batch)} DOIs; source URLs matched cumulative {matched}", flush=True)
        time.sleep(4)
    return matched


def run(root, catalog):
    folder = root / "referensi" / "paper"
    path = folder / catalog
    if not path.exists():
        raise FileNotFoundError(path)
    rows = json.loads(path.read_text(encoding="utf-8"))
    matched = gather_oa(rows)
    pdf_dir = folder / "pdf"
    pdf_dir.mkdir(parents=True, exist_ok=True)
    candidates = [r for r in rows if r["oa_pdf_url"] and r["pdf_status"] != "downloaded"]
    print("OA URLs", matched, "download candidates", len(candidates), flush=True)
    with ThreadPoolExecutor(max_workers=5) as pool:
        jobs = [pool.submit(download_pdf, pdf_dir, row) for row in candidates]
        for idx, fut in enumerate(as_completed(jobs), 1):
            row = fut.result()
            if idx % 20 == 0:
                print("PDF", idx, "/", len(jobs), row["pdf_status"], flush=True)
    prefix = catalog.removesuffix(".json")
    output_csv = folder / (prefix + ".csv")
    path.write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")
    import csv
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print("SUMMARY", json.dumps({
        "catalog": catalog,
        "papers": len(rows),
        "pdf_downloaded_or_present": sum(r["pdf_status"] in ("downloaded", "present") for r in rows),
        "pdf_failed": sum(r["pdf_status"].startswith(("http_", "download_failed", "response_not_pdf")) for r in rows),
    }), flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--catalog", default="paper_catalog.json")
    args = ap.parse_args()
    run(args.root.resolve(), args.catalog)
