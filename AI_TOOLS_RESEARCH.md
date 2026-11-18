# AI agent tools: CLI-Anything and REA for team TeamCode

Research note, 2026-11-17. It covers two agent toolkits:

- **[CLI-Anything](https://github.com/HKUDS/CLI-Anything)** (Apache-2.0) wraps real desktop programs (FreeCAD, Blender, 3MF and others) in CLIs that agents can drive.
- **[REA](https://github.com/morluto/rea)** (MIT) is an MCP server and CLI that gives agents reverse-engineering tools.

This file covers only what matters for **this** repo.

How to read the labels:

- **Confirmed**: checked against the tool's own repo or docs.
- **Recommendation**: a suggestion, not tested.
- Nothing below was run end to end on this project yet.

## What could help here

| Tool | Use in this repo | Value |
|---|---|---|
| REA **ADB tools** (needs Android platform-tools) | At practice, connect to the Control Hub (`adb connect 192.168.43.1:5555`). REA can then read logcat, take screenshots, and pull the logs in `/sdcard/FIRST`, so an agent can explain what the robot did | Medium |
| REA **APK inspection** (needs JADX + JDK 17+) | Decompile **our own** built TeamCode APK to confirm the OpModes and constants that were actually deployed | Low |

## Not for this repo

- Don't use REA on REV or FIRST closed-source parts beyond what their licenses allow.
- REA doesn't write or test Java. Keep the Gradle / Android Studio workflow.

## Proposed changes

None to the code. This is a reference note only.

## Status on the dev PC (2026-11-17)

- **Installed:** the agent skills `cli-hub-meta-skill` and `reverse-engineer-anything`, plus `cli-anything-hub` 0.4.1 (pip).
- **Not usable yet:**
  - The REA CLI/MCP needs Node 24.22.1 or newer, and the PC has 24.15.0.
  - The FreeCAD/Blender harnesses need those programs installed. Neither is.

## Rules for REA

Use REA only on code and builds we made, or on devices we own. Reverse engineering commercial software can break its EULA or DMCA §1201, and §1201(f) covers only narrow interoperability work. REA's tool results are sent to the AI model provider, so never point it at keys, `.env` files or credentials.
