/*
 * Copyright (C) 2015 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.ar.calibration;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.PointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class CameraCalibratorSampleTest {

    private static final double MIN_RANDOM_VALUE = -1.0;
    private static final double MAX_RANDOM_VALUE = 1.0;

    private static final int MIN_NUM_MARKERS = 4;
    private static final int MAX_NUM_MARKERS = 50;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-4;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-1;
    private static final double ULTRA_LARGE_ABSOLUTE_ERROR = 3.0;

    private static final double MIN_FOCAL_LENGTH = 3.0;
    private static final double MAX_FOCAL_LENGTH = 10.0;

    private static final double MIN_ANGLE_DEGREES = -10.0;
    private static final double MAX_ANGLE_DEGREES = 10.0;

    private static final int INHOM_3D_COORDS = 3;

    private static final int TIMES = 1000;

    @Test
    void testConstructor() {
        // test constructor without arguments
        var sample = new CameraCalibratorSample();

        // check correctness
        assertNull(sample.getPattern());
        assertNull(sample.getSampledMarkers());
        assertNull(sample.getSampledMarkersQualityScores());
        assertNull(sample.getUndistortedMarkers());
        assertNull(sample.getHomography());
        assertNull(sample.getRotation());
        assertNull(sample.getCameraCenter());
        assertNull(sample.getCamera());

        // test constructor with sampled markers
        final var sampledMarkers = new ArrayList<Point2D>();
        for (var i = 0; i < 4; i++) {
            sampledMarkers.add(Point2D.create());
        }
        sample = new CameraCalibratorSample(sampledMarkers);

        // check correctness
        assertNull(sample.getPattern());
        assertSame(sampledMarkers, sample.getSampledMarkers());
        assertNull(sample.getSampledMarkersQualityScores());
        assertNull(sample.getUndistortedMarkers());
        assertNull(sample.getHomography());
        assertNull(sample.getRotation());
        assertNull(sample.getCameraCenter());
        assertNull(sample.getCamera());

        // Force IllegalArgumentException
        final var emptyMarkers = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class, () -> new CameraCalibratorSample(emptyMarkers));

        // test constructor with sampled markers and quality scores
        final var qualityScores = new double[4];
        sample = new CameraCalibratorSample(sampledMarkers, qualityScores);

        // check correctness
        assertNull(sample.getPattern());
        assertSame(sampledMarkers, sample.getSampledMarkers());
        assertSame(qualityScores, sample.getSampledMarkersQualityScores());
        assertNull(sample.getUndistortedMarkers());
        assertNull(sample.getHomography());
        assertNull(sample.getRotation());
        assertNull(sample.getCameraCenter());
        assertNull(sample.getCamera());

        // Force IllegalArgumentException
        final var shortQualityScores = new double[1];
        assertThrows(IllegalArgumentException.class,
                () -> new CameraCalibratorSample(sampledMarkers, shortQualityScores));

        // test constructor with pattern and sampled markers
        final var pattern = Pattern2D.create(Pattern2DType.QR);
        sample = new CameraCalibratorSample(pattern, sampledMarkers);

        // check correctness
        assertSame(sample.getPattern(), pattern);
        assertSame(sampledMarkers, sample.getSampledMarkers());
        assertNull(sample.getSampledMarkersQualityScores());
        assertNull(sample.getUndistortedMarkers());
        assertNull(sample.getHomography());
        assertNull(sample.getRotation());
        assertNull(sample.getCameraCenter());
        assertNull(sample.getCamera());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new CameraCalibratorSample(pattern, emptyMarkers));

        // test constructor with sampled markers and quality scores
        sample = new CameraCalibratorSample(pattern, sampledMarkers, qualityScores);

        // check correctness
        assertSame(pattern, sample.getPattern());
        assertSame(sampledMarkers, sample.getSampledMarkers());
        assertSame(qualityScores, sample.getSampledMarkersQualityScores());
        assertNull(sample.getUndistortedMarkers());
        assertNull(sample.getHomography());
        assertNull(sample.getRotation());
        assertNull(sample.getCameraCenter());
        assertNull(sample.getCamera());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new CameraCalibratorSample(pattern, sampledMarkers, shortQualityScores));
    }

    @Test
    void testGetSetPattern() {
        final var sample = new CameraCalibratorSample();

        assertNull(sample.getPattern());

        // set new value
        final var pattern = Pattern2D.create(Pattern2DType.QR);
        sample.setPattern(pattern);

        // check correctness
        assertSame(pattern, sample.getPattern());
    }

    @Test
    void testGetSetSampledMarkers() {
        final var sample = new CameraCalibratorSample();

        // check default value
        assertNull(sample.getSampledMarkers());

        // set new value
        final var sampledMarkers = new ArrayList<Point2D>();
        for (var i = 0; i < 4; i++) {
            sampledMarkers.add(Point2D.create());
        }
        sample.setSampledMarkers(sampledMarkers);

        // check correctness
        assertSame(sample.getSampledMarkers(), sampledMarkers);

        // Force IllegalArgumentException
        final var emptyMarkers = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class, () -> sample.setSampledMarkers(emptyMarkers));
    }

    @Test
    void testGetSetSampledMarkersQualityScores() {
        final var sample = new CameraCalibratorSample();

        // check default value
        assertNull(sample.getSampledMarkersQualityScores());

        // set new value
        final var qualityScores = new double[4];
        sample.setSampledMarkersQualityScores(qualityScores);

        // check correctness
        assertSame(qualityScores, sample.getSampledMarkersQualityScores());

        // Force IllegalArgumentException
        final var shortScores = new double[1];
        assertThrows(IllegalArgumentException.class, () -> sample.setSampledMarkersQualityScores(shortScores));
    }

    @Test
    void testComputeSampledMarkersQualityScores() {
        final var randomizer = new UniformRandomizer();

        final var numMarkers = randomizer.nextInt(MIN_NUM_MARKERS, MAX_NUM_MARKERS);

        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var sampledMarkers = new ArrayList<Point2D>();
        Point2D marker;
        final var scoresWithCenter = new double[numMarkers];
        final var scoresNoCenter = new double[numMarkers];
        double distance;
        for (var i = 0; i < numMarkers; i++) {
            marker = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            sampledMarkers.add(marker);

            // distance with center at origin of coordinates
            distance = Math.sqrt(Math.pow(marker.getInhomX(), 2.0) + Math.pow(marker.getInhomY(), 2.0));
            scoresNoCenter[i] = 1.0 / (1.0 + distance);

            // distance respect to center
            distance = Math.sqrt(Math.pow(marker.getInhomX() - center.getInhomX(), 2.0)
                    + Math.pow(marker.getInhomY() - center.getInhomY(), 2.0));
            scoresWithCenter[i] = 1.0 / (1.0 + distance);
        }

        // check correctness
        assertArrayEquals(scoresNoCenter, CameraCalibratorSample.computeSampledMarkersQualityScores(sampledMarkers),
                ABSOLUTE_ERROR);

        assertArrayEquals(scoresWithCenter, CameraCalibratorSample.computeSampledMarkersQualityScores(sampledMarkers,
                        center), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetUndistortedMarkers() {
        final var sampledMarkers = new ArrayList<Point2D>();
        for (var i = 0; i < 4; i++) {
            sampledMarkers.add(Point2D.create());
        }
        final var sample = new CameraCalibratorSample(sampledMarkers);

        // check default value
        assertNull(sample.getUndistortedMarkers());

        // set new value
        final var undistortedMarkers = new ArrayList<Point2D>();
        for (var i = 0; i < 4; i++) {
            undistortedMarkers.add(Point2D.create());
        }
        sample.setUndistortedMarkers(undistortedMarkers);

        // check correctness
        assertSame(undistortedMarkers, sample.getUndistortedMarkers());
    }

    @Test
    void testGetSetHomography() {
        final var sample = new CameraCalibratorSample();

        // check default value
        assertNull(sample.getHomography());

        // set new value
        final var homography = new ProjectiveTransformation2D();
        sample.setHomography(homography);

        // check correctness
        assertSame(homography, sample.getHomography());
    }

    @Test
    void testGetSetRotation() {
        final var sample = new CameraCalibratorSample();

        // check default value
        assertNull(sample.getRotation());

        // set new value
        final var r = Rotation3D.create();
        sample.setRotation(r);

        // check correctness
        assertSame(r, sample.getRotation());
    }

    @Test
    void testGetSetCameraCenter() {
        final var sample = new CameraCalibratorSample();

        // check default value
        assertNull(sample.getCameraCenter());

        // set new value
        final var center = Point3D.create();
        sample.setCameraCenter(center);

        // check correctness
        assertSame(center, sample.getCameraCenter());
    }

    @Test
    void testGetSetCamera() {
        final var sample = new CameraCalibratorSample();

        // check default value
        assertNull(sample.getCamera());

        // set new value
        final var camera = new PinholeCamera();
        sample.setCamera(camera);

        // check correctness
        assertSame(camera, sample.getCamera());
    }

    @Test
    void testEstimateHomographyCirclesPattern() throws LockedException, NotReadyException, RobustEstimatorException,
            CoincidentPointsException {

        var totalPoints = 0;
        var avgTotalError = 0.0;
        for (var j = 0; j < 2 * TIMES; j++) {
            final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final var patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final var points3D = new ArrayList<Point3D>();
            for (final var patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                        1.0));
            }

            // create random camera to project 3D points
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // rotation
            final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // camera center
            final var cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // create camera with intrinsic parameters, rotation and camera
            // center
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
            camera.normalize();

            // project 3D pattern points
            final var projectedPatternPoints = camera.project(points3D);

            // create sample with projected pattern markers
            final var sample = new CameraCalibratorSample(projectedPatternPoints, CameraCalibratorSample.
                    computeSampledMarkersQualityScores(projectedPatternPoints));

            // estimate homography using ideal markers as reference
            final var estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create();
            final var homography = sample.estimateHomography(estimator, patternPoints);

            // check that points are properly transformed
            double distance;
            for (var i = 0; i < patternPoints.size(); i++) {
                distance = projectedPatternPoints.get(i).distanceTo(
                        homography.transformAndReturnNew(patternPoints.get(i)));
                avgTotalError += distance;
                totalPoints++;
            }
        }

        avgTotalError /= totalPoints;
        assertEquals(0.0, avgTotalError, 5.0 * VERY_LARGE_ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateHomographyQRPattern() throws LockedException, NotReadyException, RobustEstimatorException {

        var totalPoints = 0;
        var avgTotalError = 0.0;
        for (var j = 0; j < 2 * TIMES; j++) {
            final var pattern = Pattern2D.create(Pattern2DType.QR);
            final var patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final var points3D = new ArrayList<Point3D>();
            for (final var patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                        1.0));
            }

            // create random camera to project 3D points
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // rotation
            final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // camera center
            final var cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // create camera with intrinsic parameters, rotation and camera
            // center
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
            camera.normalize();

            // project 3D pattern points
            final var projectedPatternPoints = camera.project(points3D);

            // create sample with projected pattern markers
            final var sample = new CameraCalibratorSample(projectedPatternPoints,
                    CameraCalibratorSample.computeSampledMarkersQualityScores(projectedPatternPoints));

            // estimate homography using ideal markers as reference
            final var estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create();
            final Transformation2D homography;
            try {
                homography = sample.estimateHomography(estimator, patternPoints);
            } catch (final CoincidentPointsException e) {
                continue;
            }

            // check that points are properly transformed
            double distance;
            for (var i = 0; i < patternPoints.size(); i++) {
                distance = projectedPatternPoints.get(i).distanceTo(homography.transformAndReturnNew(
                        patternPoints.get(i)));
                avgTotalError += distance;
                totalPoints++;
            }
        }

        avgTotalError /= totalPoints;
        assertEquals(0.0, avgTotalError, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    void testComputeCameraPose() throws LockedException, NotReadyException, RobustEstimatorException,
            CoincidentPointsException, CalibrationException, NotAvailableException, WrongSizeException {

        var totalPoints = 0;
        var avgProjectionError = 0.0;
        var avgCenterError = 0.0;
        for (var j = 0; j < TIMES; j++) {
            final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final var patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0)
            final var points3D = new ArrayList<Point3D>();
            for (final var patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                        1.0));
            }

            // create random camera to project 3D points
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // rotation
            final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // camera center
            final var cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // create camera with intrinsic parameters, rotation and camera
            // center
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
            camera.normalize();

            // project 3D pattern points
            final var projectedPatternPoints = camera.project(points3D);

            // create sample with projected pattern markers
            final var sample = new CameraCalibratorSample(projectedPatternPoints,
                    CameraCalibratorSample.computeSampledMarkersQualityScores(projectedPatternPoints));

            // estimate homography using ideal markers as reference
            final var estimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create();
            final var homography = sample.estimateHomography(estimator, patternPoints);

            // set homography
            sample.setHomography(homography);

            // initially rotation, camera center and camera are null
            assertNull(sample.getRotation());
            assertNull(sample.getCameraCenter());
            assertNull(sample.getCamera());

            // compute camera pose
            sample.computeCameraPose(intrinsic);

            // check correctness

            // compare rotation
            final var sRotMat = sample.getRotation().asInhomogeneousMatrix();
            final var rotMat = rotation.asInhomogeneousMatrix();
            final var rotDiff = new Matrix(3, 3);
            for (var r = 0; r < 3; r++) {
                for (var c = 0; c < 3; c++) {
                    // signs of columns 1,2 and column 3 of rotation might
                    // be reversed and rotation would still be equal
                    rotDiff.setElementAt(r, c, Math.abs(sRotMat.getElementAt(r, c))
                            - Math.abs(rotMat.getElementAt(r, c)));
                }
            }
            assertEquals(0.0, Utils.normF(rotDiff), LARGE_ABSOLUTE_ERROR);

            // compare center
            avgCenterError += sample.getCameraCenter().distanceTo(cameraCenter);

            // compare camera parameters
            assertSame(sample.getCamera().getIntrinsicParameters(), intrinsic);
            assertSame(sample.getCamera().getCameraRotation(), sample.getRotation());
            assertEquals(0.0, sample.getCamera().getCameraCenter().distanceTo(sample.getCameraCenter()),
                    ABSOLUTE_ERROR);

            // project ideal pattern points using estimated camera and
            // compare against sampled points
            final var projectedPatternPoints2 = sample.getCamera().project(points3D);
            double distance;
            for (var i = 0; i < patternPoints.size(); i++) {
                distance = projectedPatternPoints.get(i).distanceTo(projectedPatternPoints2.get(i));
                avgProjectionError += distance;
                totalPoints++;
            }
        }

        avgProjectionError /= totalPoints;
        avgCenterError /= TIMES;
        assertEquals(0.0, avgProjectionError, ULTRA_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgCenterError, ULTRA_LARGE_ABSOLUTE_ERROR);
    }
}
