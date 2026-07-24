# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.6.0] - 2026-07-25

### Added

- Published new project documentation as an Antora site (`docs/`), covering camera calibration, self-calibration,
  epipolar geometry (essential/fundamental matrices, point correction), robust vs. non-robust estimation, and
  structure-from-motion/SLAM reconstruction, plus an expanded `README.md` with a usage example and updated
  project/documentation links.

### Changed

- Raised the minimum Java version from 17 to 21 (compiler source/target, CI workflows, and SonarCloud analysis).
- Updated `irurueta-numerical` and `irurueta-geometry` dependencies to 1.6.0, and `irurueta-statistics`,
  `irurueta-sorting`, and `irurueta-algebra` dependencies to 1.4.0.

## [1.5.0] - 2026-03-04

### Changed

- Updated `irurueta-numerical` and `irurueta-geometry` dependencies to 1.5.0.

## [1.4.0] - 2025-12-18

### Changed

- Updated `irurueta-numerical` and `irurueta-geometry` dependencies to 1.4.0.

## [1.3.2] - 2025-09-22

### Changed

- Updated `irurueta-algebra`, `irurueta-numerical`, and `irurueta-geometry` dependencies to 1.3.2.

## [1.3.1] - 2025-09-20

### Changed

- Migrated Maven Central publishing from the legacy OSSRH/Nexus staging plugin (`nexus-staging-maven-plugin`) to
  the `central-publishing-maven-plugin`; `mvnsettings.xml`'s server/profile id was renamed from `ossrh` to
  `central`.
- Bumped build/quality plugin versions (`maven-gpg-plugin`, `maven-source-plugin`, `maven-javadoc-plugin`,
  `maven-surefire`/`maven-failsafe-plugin`, `jacoco`, `spotbugs`, `maven-site-plugin`, `maven-checkstyle-plugin`,
  `maven-pmd-plugin`, `maven-jxr-plugin`) and dependency versions (`junit-jupiter` 5.13.4, `irurueta-statistics`
  1.3.4, `irurueta-sorting` 1.3.2, `irurueta-algebra`/`irurueta-numerical`/`irurueta-geometry` 1.3.1).
- Added a manual-trigger CI workflow and general test-suite improvements.

## [1.3.0] - 2025-01-03

### Changed

- Raised the minimum Java version from 1.7 to 17.
- Migrated the entire test suite from JUnit 4 to JUnit 5 (Jupiter).
- Bumped dependency versions: `irurueta-statistics` to 1.3.2, `irurueta-sorting` to 1.3.1, `irurueta-algebra` and
  `irurueta-numerical` to 1.3.0, `irurueta-geometry` to 1.3.0.
- Replaced `findbugs-maven-plugin` with `spotbugs-maven-plugin` and updated other build/quality plugin versions;
  raised the Checkstyle line-length limit to 120.
- Internal, non-functional source refactor across the `calibration`, `epipolar`, `sfm`, and `slam` packages
  (removed Hungarian-notation field prefixes, adopted `var`/diamond operators and lambdas, reformatted code) —
  verified to carry no public API or behavior changes.

## [1.2.0] - 2023-12-17

### Changed

- Updated dependencies to `irurueta-statistics`, `irurueta-sorting`, `irurueta-algebra`, `irurueta-geometry` 1.2.0
  and `irurueta-numerical` 1.2.1. As a consequence, the `RobustEstimatorMethod` enum constants used throughout the
  calibration/epipolar/sfm APIs are now named `LMEDS` and `PROMEDS` (previously `LMedS`/`PROMedS`) — a breaking
  rename for code that references these constants by name.

## [1.1.0] - 2021-12-11

Initial release.

### Added

- Camera calibration from planar patterns (circles grid, QR pattern) via `AlternatingCameraCalibrator` and
  `ErrorOptimizationCameraCalibrator`.
- Radial lens distortion estimation with LMSE, weighted, and robust (RANSAC/LMedS/MSAC/PROSAC/PROMedS) estimators.
- Image/Dual Image/Dual Absolute Quadric of the Absolute Conic estimation (linear, weighted, Kruppa's equations,
  and robust variants) for self-calibration of intrinsic camera parameters.
- Fundamental matrix estimation (8-point, 7-point, affine, planar) with RANSAC/LMedS/MSAC/PROSAC/PROMedS robust
  estimators and refiners.
- Fundamental matrix comparators (algebraic and epipolar-distance based) and homography decomposition into camera
  pose candidates.
- Essential matrix support and epipolar geometry correctors (Sampson and gold-standard, single-point and full).
- Initial pair-of-cameras estimation from the essential matrix, DIAC, or dual absolute quadric.
- Robust single 3D point triangulation from multiple views (homogeneous/inhomogeneous, weighted,
  RANSAC/LMedS/MSAC/PROSAC/PROMedS).
- Sparse 3D reconstruction pipelines for two views, paired views, and general multi-view sequences, with optional
  known-baseline scale resolution.
- SLAM-assisted sparse reconstructors combining visual structure-from-motion with inertial/positional state
  estimation, including constant-velocity-model and absolute-orientation variants.
- Standalone SLAM estimators and calibrators (position/orientation/velocity state prediction and calibration data)
  for fusing accelerometer/gyroscope samples with camera-based motion estimates.

[Unreleased]: https://github.com/albertoirurueta/irurueta-ar/compare/1.6.0...HEAD
[1.6.0]: https://github.com/albertoirurueta/irurueta-ar/compare/1.5.0...1.6.0
[1.5.0]: https://github.com/albertoirurueta/irurueta-ar/compare/1.4.0...1.5.0
[1.4.0]: https://github.com/albertoirurueta/irurueta-ar/compare/1.3.2...1.4.0
[1.3.2]: https://github.com/albertoirurueta/irurueta-ar/compare/1.3.1...1.3.2
[1.3.1]: https://github.com/albertoirurueta/irurueta-ar/compare/1.3.0...1.3.1
[1.3.0]: https://github.com/albertoirurueta/irurueta-ar/compare/1.2.0...1.3.0
[1.2.0]: https://github.com/albertoirurueta/irurueta-ar/compare/1.1.0...1.2.0
[1.1.0]: https://github.com/albertoirurueta/irurueta-ar/releases/tag/1.1.0
