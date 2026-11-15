# Implementation Plan

## Task summary

Source: GitHub issue #25
Base branch: develop

Issue #25 "Update dependencies" asks to bump the five `com.irurueta` compile dependencies in `pom.xml`:
- `irurueta-statistics`, `irurueta-sorting`, `irurueta-algebra`: 1.4.0 -> 1.4.1
- `irurueta-numerical`, `irurueta-geometry`: 1.6.0 -> 1.6.1

Choices made on the user's behalf:
- No "What's new" / README / installation docs change: those track the library's own released version and are
  updated by the release flow (`iru-release`), not by a dependency bump on the unreleased 1.7.0-SNAPSHOT line.
- The change is verified by running the full unit test suite, since patch releases could still affect behavior.

Open clarifications: none

## Acceptance criteria coverage

| # | Criterion | Covered by tasks |
|---|---|---|
| 1 | "Update dependencies in pom.xl file to latest versions" (statistics, sorting, algebra -> 1.4.1) | Task 1.1 |
| 2 | "Update dependencies in pom.xl file to latest versions" (numerical, geometry -> 1.6.1) | Task 1.2 |
| 3 | derived: the project still compiles and all existing tests pass with the new versions | Task 2 |

## Current code state

- `pom.xml` (Maven, single module, Java 21, project version `1.7.0-SNAPSHOT`), lines ~43-72: five
  `com.irurueta` dependencies, scope `compile`: statistics/sorting/algebra at `1.4.0`, numerical/geometry at `1.6.0`.
- Versions are hard-coded in each `<dependency>` (no version properties).
- Java sources under `src/main/java` and tests under `src/test/java` are not expected to change unless the patch
  releases introduce API changes.
- Docs (`docs/`, `README.md`) mention only the library's own released version (1.6.0); out of scope.

## Implementation steps

### Group 1 — Bump versions (Parallelizable: yes)

- [x] Task 1. Update dependency versions in `pom.xml` _(java)_ — pom.xml only; no Java sources or tests changed (compile OK with JDK 21; full test run deferred to Group 2); no new quality issues possible.
  - [x] Task 1.1. Change `irurueta-statistics`, `irurueta-sorting` and `irurueta-algebra` `<version>` from `1.4.0` to `1.4.1`
  - [x] Task 1.2. Change `irurueta-numerical` and `irurueta-geometry` `<version>` from `1.6.0` to `1.6.1`

### Group 2 — Verify (Parallelizable: yes)

- [x] Task 2. Verify the build with the new dependencies _(java)_
  - [x] Task 2.1. Delegate to `iru-gate-runner` (`Skill iru-java-test`, full suite) to confirm compilation and all tests pass; if the 1.4.1/1.6.1 artifacts fail to resolve or break the API, fix the affected sources or report the blocker
    - Verified: full suite (JDK 21) 2288 tests, 0 failures/errors/skipped; no sources changed.
