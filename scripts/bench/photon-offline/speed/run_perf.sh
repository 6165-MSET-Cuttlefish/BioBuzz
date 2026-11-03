#!/bin/sh
set -e
P=$(cd "$(dirname "$0")" && pwd)
H="$P/../harness"
VARS=$(python3 -I "$P/../classpath.py")
eval "$VARS"
B="$OUT/speed"
rm -rf "${B:?}"
mkdir -p "$B/fakes" "$B/perf"
find "$H/fakes" -name '*.java' -exec javac -nowarn -d "$B/fakes" -cp "$HW:$RC:$ANDROID" {} +
find "$P/src" -name '*.java' -exec javac -nowarn -d "$B/perf" -cp "$B/fakes:$TC:$HW:$RC:$FC:$SIN:$ANN:$ANDROID" \
    "$H/src/com/qualcomm/hardware/lynx/PhotonHarness.java" {} +
if [ -n "$STOCK" ]; then
    find "$P/stock" -name '*.java' -exec javac -nowarn -d "$B/perf" -cp "$B/fakes:$B/perf:$TC:$HW:$RC:$FC:$SIN:$ANN:$STOCK:$ANDROID" {} +
fi
java "$@" -Dstock="${STOCK:+true}" -cp "$B/fakes:$B/perf:$TC:$HW:$RC:$FC:$SIN:$ANN:$KOT:${STOCK:+$STOCK:}$ANDROID" \
    com.qualcomm.hardware.lynx.PerfHarness
