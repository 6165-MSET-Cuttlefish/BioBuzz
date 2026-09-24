#!/usr/bin/env python3
"""Stamps limelight/cell_tip_snapscript.py out once per alliance into limelight/pipelines/.

Pipeline indices and tag ids are read from LimelightCamera.java, so the scripts and the hub agree.
"""

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
TEMPLATE = ROOT / "limelight" / "cell_tip_snapscript.py"
OUT_DIR = ROOT / "limelight" / "pipelines"
MODULE = ROOT / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/modules/LimelightCamera.java"

PIPELINE_RE = r"public static int %sPipeline = (\d+);"
IDS_RE = r"private static final int\[\] %s_%s_IDS = \{([0-9, ]+)\};"

ALLIANCE_RE = re.compile(r"^ALLIANCE = .*$", re.M)
INDEX_RE = re.compile(r"^PIPELINE_INDEX = .*$", re.M)
CLUSTERS_RE = re.compile(r"^CLUSTERS = .*$", re.M)

TITLE_TEXT = "— template.\n"
SOURCE_TEXT = "TEMPLATE: edit this file, run limelight/generate_pipelines.py, and upload both files it writes."
GENERATED_TEXT = "GENERATED from limelight/cell_tip_snapscript.py by limelight/generate_pipelines.py; do not edit."


def alliances():
    """(ALLIANCE, pipeline index, (("SCORING", ids), ("AUDIENCE", ids))), scoring side first."""
    java = MODULE.read_text(encoding="utf-8")
    out = []
    for name in ("red", "blue"):
        pipeline = re.search(PIPELINE_RE % name, java)
        if not pipeline:
            sys.exit("Could not find %sPipeline in %s" % (name, MODULE))
        clusters = []
        for side in ("SCORING", "AUDIENCE"):
            found = re.search(IDS_RE % (name.upper(), side), java)
            if not found:
                sys.exit("Could not find %s_%s_IDS in %s" % (name.upper(), side, MODULE))
            ids = tuple(int(t) for t in found.group(1).split(","))
            if len(ids) != 4:
                sys.exit("%s_%s_IDS should have 4 ids, found %d" % (name.upper(), side, len(ids)))
            clusters.append((side, ids))
        out.append((name.upper(), int(pipeline.group(1)), tuple(clusters)))
    return out


def main():
    template = TEMPLATE.read_text(encoding="utf-8")
    for label, pattern in (("ALLIANCE", ALLIANCE_RE), ("PIPELINE_INDEX", INDEX_RE),
                           ("CLUSTERS", CLUSTERS_RE)):
        if not pattern.search(template):
            sys.exit("No %s line to replace in %s" % (label, TEMPLATE))
    for label, text in (("title suffix", TITLE_TEXT), ("source-of-truth paragraph", SOURCE_TEXT)):
        if text not in template:
            sys.exit("No %s to replace in %s; update this script to match the docstring" % (label, TEMPLATE))
    pipelines = alliances()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for stale in OUT_DIR.glob("pipeline*.py"):
        stale.unlink()

    for name, pipeline, clusters in pipelines:
        body = ALLIANCE_RE.sub('ALLIANCE = "%s"' % name, template, count=1)
        body = INDEX_RE.sub("PIPELINE_INDEX = %d" % pipeline, body, count=1)
        body = CLUSTERS_RE.sub(
            "CLUSTERS = (%s)" % ", ".join('("%s", %s)' % (side, ids) for side, ids in clusters),
            body, count=1)
        body = body.replace(TITLE_TEXT, "— pipeline %d, %s.\n" % (pipeline, name), 1)
        body = body.replace(SOURCE_TEXT, GENERATED_TEXT, 1)
        out = OUT_DIR / ("pipeline%d_%s.py" % (pipeline, name.lower()))
        out.write_text(body, encoding="utf-8")
        print("wrote %s  %s" % (out.relative_to(ROOT),
                                 "  ".join("%s %d-%d" % (s, i[0], i[-1]) for s, i in clusters)))


if __name__ == "__main__":
    main()
