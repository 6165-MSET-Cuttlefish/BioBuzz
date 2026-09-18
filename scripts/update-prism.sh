#!/usr/bin/env bash
# Refreshes the vendored goBILDA Prism driver from goBILDA's official Add-Prism branch.
# Copies the four driver files verbatim except for the package line, and records the upstream commit.
set -euo pipefail

REF="${1:-Add-Prism}"
REPO="https://github.com/goBILDA-Official/FtcRobotController-Add-Prism.git"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="$ROOT/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/prism"
SRC_PATH="TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Prism"
FILES=(Color.java Direction.java GoBildaPrismDriver.java PrismAnimations.java)
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

git clone -q --depth 1 --branch "$REF" "$REPO" "$WORK/prism"
SHA="$(git -C "$WORK/prism" rev-parse HEAD)"
DATE="$(git -C "$WORK/prism" log -1 --format=%cs)"

mkdir -p "$DEST"
for f in "${FILES[@]}"; do
  sed -e 's/^package org\.firstinspires\.ftc\.teamcode\.Prism;/package org.firstinspires.ftc.teamcode.architecture.prism;/' \
      -e 's/org\.firstinspires\.ftc\.teamcode\.Prism\./org.firstinspires.ftc.teamcode.architecture.prism./g' \
      "$WORK/prism/$SRC_PATH/$f" > "$DEST/$f"
done
printf 'repo=%s\nref=%s\ncommit=%s\ndate=%s\nfiles=%s\nnote=package line rewritten from teamcode.Prism to teamcode.architecture.prism; otherwise verbatim\n' \
  "$REPO" "$REF" "$SHA" "$DATE" "${FILES[*]}" > "$DEST/PRISM_REV"

echo "Vendored ${#FILES[@]} Prism files from $REF @ ${SHA:0:7} ($DATE)."
echo "Review with: git -C \"$ROOT\" diff -- TeamCode/src/main/java/org/firstinspires/ftc/teamcode/architecture/prism"
