#!/bin/sh
set -e
T=$(cd "$(dirname "$0")" && pwd)
VARS=$(python3 -I "$T/../classpath.py")
eval "$VARS"
B="$OUT/harness"
rm -rf "${B:?}"
mkdir -p "$B/fakes" "$B/classes"
find "$T/fakes" -name '*.java' -exec javac -nowarn -d "$B/fakes" -cp "$HW:$RC:$ANDROID" {} +
find "$T/src" -name '*.java' -exec javac -nowarn -d "$B/classes" -cp "$B/fakes:$TC:$HW:$RC:$FC:$SIN:$ANN:$ANDROID" {} +
java -cp "$B/fakes:$B/classes:$TC:$HW:$RC:$FC:$SIN:$ANN:$KOT:$ANDROID" com.qualcomm.hardware.lynx.PhotonHarness
