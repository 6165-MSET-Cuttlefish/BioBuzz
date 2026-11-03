#!/bin/sh
set -e
L=$(cd "$(dirname "$0")" && pwd)
H="$L/../harness"
VARS=$(python3 -I "$L/../classpath.py")
eval "$VARS"
B="$OUT/lockleak"
rm -rf "${B:?}"
mkdir -p "$B/fakes" "$B/classes"
find "$H/fakes" -name '*.java' -exec javac -nowarn -d "$B/fakes" -cp "$HW:$RC:$ANDROID" {} +
find "$L/src" -name '*.java' -exec javac -nowarn -d "$B/classes" -cp "$B/fakes:$TC:$HW:$RC:$FC:$SIN:$ANN:$ANDROID" \
    "$H/src/com/qualcomm/hardware/lynx/PhotonHarness.java" {} +
java -cp "$B/fakes:$B/classes:$TC:$HW:$RC:$FC:$SIN:$ANN:$KOT:$ANDROID" com.qualcomm.hardware.lynx.LockLeakHarness
