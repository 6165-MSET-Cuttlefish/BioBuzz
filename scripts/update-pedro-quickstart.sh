#!/usr/bin/env bash
# Refreshes the vendored Pedro Pathing tuning procedures from the upstream Quickstart.
# Copies only TeamCode/.../pedro/procedures/ and records the upstream commit; never touches
# anything else under pedro/, which is ours.
set -euo pipefail

REF="${1:-master}"
REPO="https://github.com/Pedro-Pathing/Quickstart.git"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="$ROOT/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures"
SRC_PATH="TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

git clone -q --depth 1 --branch "$REF" "$REPO" "$WORK/qs"
SHA="$(git -C "$WORK/qs" rev-parse HEAD)"
DATE="$(git -C "$WORK/qs" log -1 --format=%cs)"

rm -rf "$DEST"
mkdir -p "$DEST"
cp "$WORK/qs/$SRC_PATH"/*.java "$DEST/"
printf 'repo=%s\nref=%s\ncommit=%s\ndate=%s\n' "$REPO" "$REF" "$SHA" "$DATE" > "$DEST/QUICKSTART_REV"

echo "Vendored $(ls "$DEST"/*.java | wc -l | tr -d ' ') procedures from Quickstart $REF @ ${SHA:0:7} ($DATE)."
echo "Review with: git -C \"$ROOT\" diff -- $SRC_PATH"
