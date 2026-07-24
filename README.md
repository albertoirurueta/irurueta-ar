# irurueta-ar

Augmented Reality and 3D reconstruction library

[![Build Status](https://github.com/albertoirurueta/irurueta-ar/actions/workflows/master.yml/badge.svg)](https://github.com/albertoirurueta/irurueta-ar/actions)
[![Build Status](https://github.com/albertoirurueta/irurueta-ar/actions/workflows/develop.yml/badge.svg)](https://github.com/albertoirurueta/irurueta-ar/actions)
[![Build Status](https://github.com/albertoirurueta/irurueta-ar/actions/workflows/manual_develop.yml/badge.svg)](https://github.com/albertoirurueta/irurueta-ar/actions)

[![Bugs](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-ar&metric=bugs)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-ar)
[![Code Smells](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-ar&metric=code_smells)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-ar)
[![Coverage](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-ar&metric=coverage)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-ar)

[![Duplicated lines](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-ar&metric=duplicated_lines_density)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-ar)
[![Lines of code](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-ar&metric=ncloc)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-ar)

[![Maintainability](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-ar&metric=sqale_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-ar)
[![Quality gate](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-ar&metric=alert_status)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-ar)
[![Reliability](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-ar&metric=reliability_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-ar)

[![Security](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-ar&metric=security_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-ar)
[![Technical debt](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-ar&metric=sqale_index)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-ar)
[![Vulnerabilities](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-ar&metric=vulnerabilities)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-ar)

## Project Status

| | |
|---|---|
| Language | Java 21 |
| Build tool | Maven |
| Current development version | 1.6.0-SNAPSHOT |
| Latest release | 1.5.0 |
| License | Apache License 2.0 |
| CI | GitHub Actions — builds/tests on `develop` and `master`, plus a manual on-demand `develop` build |
| Quality | SonarCloud, JaCoCo (coverage), Checkstyle, SpotBugs, PMD |

## Documentation

- [Maven Site Report](http://albertoirurueta.github.io/irurueta-ar) — Javadoc, coverage, and code-quality reports
- [SonarCloud dashboard](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-ar)
- [CHANGELOG](CHANGELOG.md)

## Installation

Add the following dependency to your project:

Latest release:
```xml
<dependency>
    <groupId>com.irurueta</groupId>
    <artifactId>irurueta-ar</artifactId>
    <version>1.5.0</version>
    <scope>compile</scope>
</dependency>
```

Latest snapshot:
```xml
<dependency>
    <groupId>com.irurueta</groupId>
    <artifactId>irurueta-ar</artifactId>
    <version>1.6.0-SNAPSHOT</version>
    <scope>compile</scope>
</dependency>
```

## How It Works

`irurueta-ar` estimates a scene's 3D structure and camera geometry from point correspondences observed across
multiple images. Given matched 2D points between two views, `TwoViewsSparseReconstructor` drives the full
pipeline — fundamental matrix estimation, essential matrix recovery, camera pose estimation, self-calibration,
and triangulation — and reports results back through a listener:

```java
var configuration = new TwoViewsSparseReconstructorConfiguration();
var listener = new TwoViewsSparseReconstructorListener() {
    @Override
    public boolean hasMoreViewsAvailable(TwoViewsSparseReconstructor reconstructor) {
        // return true until both views have been supplied
        return !bothViewsSupplied;
    }

    @Override
    public void onRequestSamplesForCurrentView(TwoViewsSparseReconstructor reconstructor, int viewId,
            List<Sample2D> samples) {
        // supply the 2D points detected in this view
    }

    @Override
    public void onRequestMatches(TwoViewsSparseReconstructor reconstructor, List<Sample2D> samples1,
            List<Sample2D> samples2, int viewId1, int viewId2, List<MatchedSamples> matches) {
        // supply the point correspondences between the two views
    }

    @Override
    public void onReconstructedPointsEstimated(TwoViewsSparseReconstructor reconstructor,
            List<MatchedSamples> matches, List<ReconstructedPoint3D> points) {
        // consume the reconstructed 3D points here
    }

    // onSamplesAccepted, onSamplesRejected, onFundamentalMatrixEstimated, onCamerasEstimated,
    // onStart, onFinish, onCancel and onFail complete the listener contract
};

var reconstructor = new TwoViewsSparseReconstructor(configuration, listener);
reconstructor.start();
```

See the [Documentation](#documentation) links above for the full API reference, including `SLAM`-fused
reconstructors that combine visual estimation with accelerometer/gyroscope data.

## License

This project is licensed under the [Apache License 2.0](LICENSE.txt).
