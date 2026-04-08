# Algorithm Layer Reorganization Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reorganize the algorithm layer in both `Gimbal` and `Chassis` by controller family and responsibility, while preserving all existing runtime behavior and build success.

**Architecture:** Split mixed top-level controller files into focused controller-family files, move estimation and utility code into dedicated subdirectories, then update all include paths and build scripts to point to the new layout.

**Tech Stack:** STM32 C, CMake, Makefile, PowerShell regression scripts.

---

### Completed Work

- [x] Wrote `tests/algorithm_layer_layout_regression.ps1` and verified it failed before restructuring.
- [x] Created the new `controllers/`, `estimation/`, and `utils/` subdirectory layout in both codebases.
- [x] Split mixed controller definitions into dedicated `pid`, `lqr`, and `rls` files.
- [x] Moved business control algorithms, estimation code, and utility code into categorized subdirectories.
- [x] Updated runtime includes, Makefiles, and `Gimbal/CMakeLists.txt`.
- [x] Deleted legacy top-level algorithm source/header files.
- [x] Updated `algorithm.md` summaries to document the new layout.
- [x] Verified layout, boundary, build-entry, and unused-module regression scripts all pass.
- [x] Verified both `Gimbal/compile.ps1 Debug` and `Chassis/compile.ps1 Debug` succeed.
