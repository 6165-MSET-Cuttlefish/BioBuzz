# BioBuzz

FTC team 6165 MSET Cuttlefish, 2026–2027 season.

Single-module Android app on FTC SDK 12.0.0, Pedro Pathing 3.0.1, Sloth hot reload, and slothboard.

## Build

Requires JDK 25 (the Gradle daemon is pinned to it) and Android Studio Narwhal 3 Feature Drop or newer.

```bash
./gradlew :TeamCode:installDebug      # first deploy to a Control Hub
./gradlew :TeamCode:deploySloth       # hot reload after that
```

`CLAUDE.md` is the full description of the build, the framework, and the conventions.
