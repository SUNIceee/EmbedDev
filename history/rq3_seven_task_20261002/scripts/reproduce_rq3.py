#!/usr/bin/env python3
"""Recompute the final RQ3 table and optionally replot it, entirely offline."""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[1]
SPECIAL_ID = "gpio_output__deepseek-v4-pro__C4_structured__r03"


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def close(a, b):
    return math.isclose(a, b, rel_tol=0, abs_tol=1e-9)


def load_and_recompute():
    provenance = read(ROOT / "rq3_export_provenance.json")
    for relative, expected in provenance["exported_files"].items():
        require(sha(ROOT / relative) == expected, "Bundled data hash mismatch: " + relative)
    data = read(ROOT / "data/rq3/display_cells.json")
    positions = read(ROOT / "data/rq3/position_scores.json")["positions"]
    protocol = read(ROOT / "data/rq3/protocol.json")
    source_index = read(ROOT / "data/rq3/candidate_sources/index.json")
    require(source_index["candidate_count"] == len(source_index["candidates"]) == 59,
            "Expected 59 fixed candidates for the four new tasks.")
    code_files = [f for c in source_index["candidates"] for f in c["files"]]
    require(len(code_files) == 118, "Expected 118 C/header files.")
    for item in code_files:
        require(sha(ROOT / item["path"]) == item["original_sha256"] == item["exported_sha256"],
                "Exported candidate bytes differ from their fixed packet hash.")
    review_counts = read(ROOT / "data/rq3/new_task_review_counts.json")
    checklists = {task: read(ROOT / ("data/rq3/requirements/" + task + ".json"))
                  for task in data["task_order"][:4]}
    require(sum(len(c["items"]) for c in checklists.values()) == 58, "Expected 58 fixed requirements.")
    require(len(review_counts["rows"]) == 60, "Expected 60 new-task position records.")
    by_id = {p["identity"]: p for p in positions}
    judgment_total = 0
    for review in review_counts["rows"]:
        p = by_id[review["identity"]]
        if review["artifact_available"]:
            require([j["id"] for j in review["judgments"]]
                    == [i["id"] for i in checklists[review["task"]]["items"]],
                    "The retained verdict list differs from the fixed requirement order.")
            count = Counter(j["verdict"] for j in review["judgments"])
            require({k: count[k] for k in ("P", "F", "U", "N/A")} == review["counts"] == p["counts"],
                    "Retained item verdicts do not match position counts.")
            judgment_total += len(review["judgments"])
        else:
            require(review["judgments"] == [] and review["counts"] is None,
                    "The local no-code position cannot acquire fabricated item verdicts.")
    require(judgment_total == 862, "Expected 862 retained, not regenerated, item verdicts.")
    require(len(positions) == len({p["identity"] for p in positions}) == 105,
            "Expected 105 unique task/method/repeat positions.")
    require(len(data["cells"]) == 35, "Expected 35 task/method cells.")
    require(len(data["task_order"]) == 7 and len(data["methods"]) == 5,
            "Expected seven tasks and five methods.")
    author_reports = []
    for p in positions:
        kind, score = p["source_type"], p["display_score_percent"]
        require(p["model"] == data["model"], "Generation model mismatch.")
        require(math.isfinite(score) and 0 <= score <= 100, "Invalid position score.")
        if kind == "code_bound_static_review":
            counts = p["counts"]
            require(set(counts) == {"P", "F", "U", "N/A"}, "Unexpected count fields.")
            require(all(isinstance(v, int) and v >= 0 for v in counts.values()),
                    "Counts must be nonnegative integers.")
            den = counts["P"] + counts["F"] + counts["U"]
            require(den > 0 and den == p["denominator"], "Review denominator mismatch.")
            require(close(score, 100 * counts["P"] / den), "Review score mismatch.")
            require(p["review_binding"] is not None and bool(p["candidate_code_sha256"]),
                    "A reviewed position must retain review and code hashes.")
        elif kind == "no_code_position":
            require(score == 0 and not p["artifact_available"] and p["counts"] is None,
                    "A no-code position cannot contain fabricated judgments.")
        elif kind == "author_reported_rerun_score":
            require(p["identity"] == SPECIAL_ID and score == 100,
                    "Only the recorded author-reported GPIO replacement is permitted.")
            require(p["review_binding"] is None and p["counts"] is None
                    and p["locally_bound_review"] is False
                    and p["author_report"]["local_verification_completed"] is False,
                    "The author-reported score must not claim local verification.")
            author_reports.append(p["identity"])
        else:
            raise ValueError("Unknown score source: " + kind)
    require(author_reports == [SPECIAL_ID], "Expected one explicit author-reported score.")
    lookup = {(c["task"], c["method"]): c for c in data["cells"]}
    require(len(lookup) == 35, "Duplicate task/method cell.")
    rows = []
    for task in data["task_order"]:
        for method in data["methods"]:
            fixed = lookup[task, method]
            group = sorted((p for p in positions if p["task"] == task and p["method"] == method),
                           key=lambda p: p["repeat"])
            require([p["repeat"] for p in group] == [1, 2, 3], "Missing repeat position.")
            values = [p["display_score_percent"] for p in group]
            mean, sd = statistics.mean(values), statistics.stdev(values)
            require(fixed["n"] == 3 and all(close(a, b) for a, b in zip(values, fixed["values_percent"])),
                    "Displayed position scores do not match the fixed source.")
            require(close(mean, fixed["mean_percent"]) and close(sd, fixed["sample_sd_percentage_points"]),
                    "Mean/sample SD differs from the final published display.")
            rows.append({"task": task, "method": method, "protocol": fixed["protocol"], "n": 3,
                         "repeat_1_percent": values[0], "repeat_2_percent": values[1],
                         "repeat_3_percent": values[2], "mean_percent": mean,
                         "sample_sd_percentage_points": sd,
                         "author_reported_positions": sum(p["identity"] in author_reports for p in group)})
    return data, protocol, positions, rows


def plot(rows, data, protocol, output):
    try:
        from reportlab.graphics import renderPDF, renderSVG
        from reportlab.graphics.shapes import Drawing, Line, Rect, String
        from reportlab.lib.colors import HexColor
    except ImportError as exc:
        raise RuntimeError("Plotting needs ReportLab. Install it with: python -m pip install reportlab. "
                           "Table recomputation needs only the Python standard library.") from exc
    colors = ["#777777", "#9467BD", "#0072B2", "#009E73", "#D55E00"]
    labels = ["GPIO\noutput", "Button\ndebounce", "Balance\ncontrol", "Motor\ntracking",
              "DiscoBot", "OnStep", "Crazyflie"]
    drawing = Drawing(620, 370)
    left, right, bottom, top = 48, 609, 96, 278
    y = lambda value: bottom + (top - bottom) * value / 110
    lookup = {(r["task"], r["method"]): r for r in rows}
    for value in (0, 25, 50, 75, 100):
        yy = y(value)
        drawing.add(Line(left, yy, right, yy, strokeColor=HexColor("#D7DBE0"), strokeWidth=.6))
        drawing.add(String(left - 8, yy - 3.5, str(value), fontName="Helvetica", fontSize=10.5, textAnchor="end"))
    drawing.add(Line(left, bottom, left, top, strokeColor=HexColor("#555555"), strokeWidth=.8))
    drawing.add(String(left, 295, "Static support (%)", fontName="Helvetica-Bold", fontSize=12.5))
    sd_count = zero_count = 0
    geometry = []
    for j, task in enumerate(data["task_order"]):
        center = left + 34 + j * (right - left - 68) / 6
        for i, method in enumerate(data["methods"]):
            row = lookup[task, method]
            mean, sd = row["mean_percent"], row["sample_sd_percentage_points"]
            x, width = center - 56.2 / 2 + i * 11.6, 9.8
            middle, color = x + width / 2, HexColor(colors[i])
            drawing.add(Rect(x, bottom, width, y(mean) - bottom, fillColor=color, strokeColor=color, strokeWidth=.45))
            if mean == 0:
                zero_count += 1
                drawing.add(String(middle, bottom + 4, "0", fontName="Helvetica", fontSize=7.5,
                                   textAnchor="middle", fillColor=color))
            if sd > 0:
                sd_count += 1
                lo, hi = y(mean - sd), y(mean + sd)
                require(bottom <= lo < hi <= top, "An SD interval would be clipped.")
                for a, b, c, dd in [(middle, lo, middle, hi), (middle-2.5, lo, middle+2.5, lo),
                                    (middle-2.5, hi, middle+2.5, hi)]:
                    drawing.add(Line(a, b, c, dd, strokeColor=HexColor("#222222"), strokeWidth=.65))
            geometry.append({"task": task, "method": method, "x": x, "width": width,
                             "height": y(mean)-bottom, "mean_percent": mean, "sd_percentage_points": sd})
        for k, label in enumerate(labels[j].split("\n")):
            drawing.add(String(center, 79-k*12, label, fontName="Helvetica", fontSize=10.5, textAnchor="middle"))
    for i, (method, (x, yy)) in enumerate(zip(data["methods"], [(52,350), (245,350), (435,350), (160,327), (365,327)])):
        color = HexColor(colors[i])
        drawing.add(Rect(x, yy-4, 13, 9, fillColor=color, strokeColor=color))
        label = method + ("*" if i in (1, 2, 3) else "")
        drawing.add(String(x+20, yy-4, label, fontName="Helvetica-Bold" if i == 4 else "Helvetica", fontSize=11.5))
    notes = ["DeepSeek-V4-Pro; n=3. Bars: mean; error bars: sample SD (not a confidence interval).",
             "Static support: P/(P+F+U); N/A excluded. * Adapted baseline.",
             "Review item granularity differs by task; no pooled overall score."]
    for i, note in enumerate(notes):
        drawing.add(String(14, 43-i*12, note, fontName="Helvetica", fontSize=9.7))
    renderPDF.drawToFile(drawing, str(output / "rq3_static_support.pdf"))
    renderSVG.drawToFile(drawing, str(output / "rq3_static_support.svg"))
    return {"bars": len(rows), "sd_intervals": sd_count, "zero_annotations": zero_count,
            "sd_clipped": False, "geometry": geometry}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/rq3",
                        help="New or empty output directory. Existing result files are not overwritten.")
    parser.add_argument("--plot", action="store_true", help="Also produce PDF and SVG using optional ReportLab.")
    args = parser.parse_args()
    data, protocol, positions, rows = load_and_recompute()
    output = args.output.resolve()
    names = ["rq3_table.csv", "rq3_table.md", "rq3_recomputed.json", "rq3_validation.json"]
    if args.plot:
        names += ["rq3_static_support.pdf", "rq3_static_support.svg"]
    require(not any((output / name).exists() for name in names), "Output exists; choose a new --output directory.")
    output.mkdir(parents=True, exist_ok=True)
    with (output / "rq3_table.csv").open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    with (output / "rq3_table.md").open("x", encoding="utf-8", newline="\n") as stream:
        stream.write("| Task | Method | Mean (%) | Sample SD (pp) | n |\n|---|---|---:|---:|---:|\n")
        for row in rows:
            stream.write(f"| {row['task']} | {row['method']} | {row['mean_percent']:.2f} | "
                         f"{row['sample_sd_percentage_points']:.2f} | 3 |\n")
    with (output / "rq3_recomputed.json").open("x", encoding="utf-8") as stream:
        json.dump({"cells": rows, "protocol": protocol}, stream, indent=2); stream.write("\n")
    geometry = plot(rows, data, protocol, output) if args.plot else None
    validation = {"status": "offline_recomputed_from_bundled_scores", "cells": 35, "positions": 105,
                  "means_and_sample_sd_match_fixed_display": True,
                  "counts_checked_for_locally_bound_reviews": 95, "retained_no_code_positions": 9,
                  "fixed_new_task_candidates_verified": 59, "exact_code_files_verified": 118,
                  "fixed_whole_requirement_items_verified": 58, "retained_item_verdicts_checked": 862,
                  "author_reported_replacement_positions": 1,
                  "author_reported_position_locally_verified": False,
                  "new_model_calls": 0, "new_host_runs": 0, "new_semantic_judgments": 0,
                  "input_sha256": {p.relative_to(ROOT).as_posix(): sha(p) for p in
                                    [ROOT/"data/rq3/display_cells.json", ROOT/"data/rq3/position_scores.json",
                                     ROOT/"data/rq3/protocol.json", ROOT/"rq3_export_provenance.json"]},
                  "plot": geometry}
    with (output / "rq3_validation.json").open("x", encoding="utf-8") as stream:
        json.dump(validation, stream, indent=2); stream.write("\n")
    print(json.dumps({"status": validation["status"], "output": str(output), "cells": 35,
                      "positions": 105, "plot_created": args.plot, "new_experiments": 0}))


if __name__ == "__main__":
    main()
