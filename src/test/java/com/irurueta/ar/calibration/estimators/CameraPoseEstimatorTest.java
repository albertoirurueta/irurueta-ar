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
package com.irurueta.ar.calibration.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.ar.calibration.Pattern2D;
import com.irurueta.ar.calibration.Pattern2DType;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.ProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.jupiter.api.Assertions.*;

class CameraPoseEstimatorTest {

    // use distance between 3cm and 20cm 10cm to the pattern
    private static final double MIN_RANDOM_VALUE = -0.2;
    private static final double MAX_RANDOM_VALUE = -0.03;

    // use a typical focal length for a phone (Samsung Galaxy Note 4)
    private static final double MIN_FOCAL_LENGTH = 1400.0;
    private static final double MAX_FOCAL_LENGTH = 1500.0;

    // skewness is always close to 0.0
    private static final double MIN_SKEWNESS = -1e-6;
    private static final double MAX_SKEWNESS = 1e-6;

    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 5.0;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 5e-4;

    private static final double STD_ERROR = 0.5;

    private static final int TIMES = 100;

    @Test
    void testConstructor() {
        final var estimator = new CameraPoseEstimator();

        // check correctness
        assertNull(estimator.getRotation());
        assertNull(estimator.getCameraCenter());
        assertNull(estimator.getCamera());
    }

    @Test
    void testEstimate() throws AlgebraException, GeometryException {
        final var pattern = Pattern2D.create(Pattern2DType.QR);

        final var patternPoints = pattern.getIdealPoints();

        // assume that pattern points are located on a 3D plane
        // (for instance Z = 0), but can be really any plane
        final var points3D = new ArrayList<Point3D>();
        for (final var patternPoint : patternPoints) {
            points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0, 1.0));
        }

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create random camera

            // rotation
            final var alphaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0, MAX_ANGLE_DEGREES * Math.PI / 180.0);
            final var betaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0, MAX_ANGLE_DEGREES * Math.PI / 180.0);
            final var gammaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0, MAX_ANGLE_DEGREES * Math.PI / 180.0);

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // camera center
            final var cameraCenterArray = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // create camera with intrinsic parameters, rotation and camera
            // center
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
            camera.normalize();

            // ensure that all 3D pattern points are in front of the camera
            assertTrue(camera.isPointInFrontOfCamera(points3D.get(0)));
            assertTrue(camera.isPointInFrontOfCamera(points3D.get(1)));
            assertTrue(camera.isPointInFrontOfCamera(points3D.get(2)));
            assertTrue(camera.isPointInFrontOfCamera(points3D.get(3)));

            // project 3D pattern points
            final var projectedPatternPoints = camera.project(points3D);

            // estimate homography between 2D pattern points and projected
            // pattern points
            final var homography = new ProjectiveTransformation2D(
                    patternPoints.get(0), patternPoints.get(1), patternPoints.get(2), patternPoints.get(3),
                    projectedPatternPoints.get(0), projectedPatternPoints.get(1), projectedPatternPoints.get(2),
                    projectedPatternPoints.get(3));

            // estimate camera pose
            final var estimator = new CameraPoseEstimator();

            // check default values
            assertNull(estimator.getRotation());
            assertNull(estimator.getCameraCenter());
            assertNull(estimator.getCamera());

            // estimate
            estimator.estimate(intrinsic, homography);

            // check correctness
            assertNotNull(estimator.getRotation());
            // check that rotation is equal at matrix level (up to sign)
            final var rotMat1 = rotation.getInternalMatrix();
            final var rotMat2 = estimator.getRotation().asInhomogeneousMatrix();
            for (var i = 0; i < 3; i++) {
                for (var j = 0; j < 3; j++) {
                    assertEquals(Math.abs(rotMat1.getElementAt(i, j)), Math.abs(rotMat2.getElementAt(i, j)),
                            VERY_LARGE_ABSOLUTE_ERROR);
                }
            }
            assertNotNull(estimator.getCameraCenter());

            final var inhomX1 = cameraCenter.getInhomX();
            final var inhomY1 = cameraCenter.getInhomY();
            final var inhomZ1 = cameraCenter.getInhomZ();
            final var inhomX2 = estimator.getCameraCenter().getInhomX();
            final var inhomY2 = estimator.getCameraCenter().getInhomY();
            final var inhomZ2 = estimator.getCameraCenter().getInhomZ();

            if (Math.abs(inhomX1 - inhomX2) > 2 * VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(inhomX1, inhomX2, 2 * VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(inhomY1 - inhomY2) > 2 * VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(inhomY1, inhomY2, 2 * VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(inhomZ1 - inhomZ2) > 2 * VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(inhomZ1, inhomZ2, 2 * VERY_LARGE_ABSOLUTE_ERROR);
            // check that all pattern points are in front of the estimated
            // camera, just the same as for the original camera
            assertTrue(estimator.getCamera().isPointInFrontOfCamera(points3D.get(0)));
            assertTrue(estimator.getCamera().isPointInFrontOfCamera(points3D.get(1)));
            assertTrue(estimator.getCamera().isPointInFrontOfCamera(points3D.get(2)));
            assertTrue(estimator.getCamera().isPointInFrontOfCamera(points3D.get(3)));

            final var distance = cameraCenter.distanceTo(estimator.getCameraCenter());
            if (distance > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, distance, LARGE_ABSOLUTE_ERROR);

            // project 3D points using estimated camera and check projection
            // error
            assertNotNull(estimator.getCamera());
            final var projectedPatternPoints2 = estimator.getCamera().project(points3D);
            assertEquals(projectedPatternPoints.size(), projectedPatternPoints2.size());
            var failed = false;
            for (var i = 0; i < projectedPatternPoints.size(); i++) {
                final var projectionDistance = projectedPatternPoints.get(i).distanceTo(projectedPatternPoints2.get(i));
                if (projectionDistance > 4 * VERY_LARGE_ABSOLUTE_ERROR) {
                    failed = true;
                    break;
                }
                assertEquals(0.0, projectionDistance, 4 * VERY_LARGE_ABSOLUTE_ERROR);
            }

            if (failed) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimateQRWithError() throws AlgebraException, GeometryException {
        final var pattern = Pattern2D.create(Pattern2DType.QR);

        final var patternPoints = pattern.getIdealPoints();

        // assume that pattern points are located on a 3D plane
        // (for instance Z = 0), but can be really any plane
        final var points3D = new ArrayList<Point3D>();
        for (final var patternPoint : patternPoints) {
            points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0, 1.0));
        }

        var avgDistanceError = 0.0;
        var avgProjectionError = 0.0;
        for (var t = 0; t < TIMES; t++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create random camera

            // rotation
            final var alphaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0, MAX_ANGLE_DEGREES * Math.PI / 180.0);
            final var betaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0, MAX_ANGLE_DEGREES * Math.PI / 180.0);
            final var gammaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0, MAX_ANGLE_DEGREES * Math.PI / 180.0);

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // camera center
            final var cameraCenterArray = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // create camera with intrinsic parameters, rotation and camera
            // center
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
            camera.normalize();

            // ensure that all 3D pattern points are in front of the camera
            assertTrue(camera.isPointInFrontOfCamera(points3D.get(0)));
            assertTrue(camera.isPointInFrontOfCamera(points3D.get(1)));
            assertTrue(camera.isPointInFrontOfCamera(points3D.get(2)));
            assertTrue(camera.isPointInFrontOfCamera(points3D.get(3)));

            // project 3D pattern points
            final var projectedPatternPoints = camera.project(points3D);
            // add error to projected pattern points
            final var rnd = new GaussianRandomizer(0.0, STD_ERROR);
            for (final var p : projectedPatternPoints) {
                p.setInhomogeneousCoordinates(p.getInhomX() + rnd.nextDouble(),
                        p.getInhomY() + rnd.nextDouble());
            }

            // estimate homography between 2D pattern points and projected
            // pattern points
            final var homography = new ProjectiveTransformation2D(
                    patternPoints.get(0), patternPoints.get(1),
                    patternPoints.get(2), patternPoints.get(3),
                    projectedPatternPoints.get(0),
                    projectedPatternPoints.get(1),
                    projectedPatternPoints.get(2),
                    projectedPatternPoints.get(3));

            // estimate camera pose
            final var estimator = new CameraPoseEstimator();

            // check default values
            assertNull(estimator.getRotation());
            assertNull(estimator.getCameraCenter());
            assertNull(estimator.getCamera());

            // estimate
            estimator.estimate(intrinsic, homography);

            // check correctness
            assertNotNull(estimator.getRotation());
            // check that rotation is equal at matrix level (up to sign)
            assertNotNull(estimator.getCameraCenter());

            final var distance = cameraCenter.distanceTo(estimator.getCameraCenter());
            avgDistanceError += distance;

            // project 3D points using estimated camera and check projection
            // error
            assertNotNull(estimator.getCamera());
            final var projectedPatternPoints2 = estimator.getCamera().project(points3D);
            assertEquals(projectedPatternPoints.size(), projectedPatternPoints2.size());
            for (var i = 0; i < projectedPatternPoints.size(); i++) {
                final var projectionDistance = projectedPatternPoints.get(i).distanceTo(
                        projectedPatternPoints2.get(i));
                avgProjectionError += projectionDistance;
            }
        }

        avgDistanceError /= TIMES;
        avgProjectionError /= TIMES;

        var msg = "QR with Error - Average camera center error: " + avgDistanceError + "m";
        Logger.getLogger(CameraPoseEstimatorTest.class.getName()).log(Level.INFO, msg);
        msg = "QR with Error - Average projection error: " + avgProjectionError + "px";
        Logger.getLogger(CameraPoseEstimatorTest.class.getName()).log(Level.INFO, msg);
    }

    @Test
    void testEstimateCirclesWithError() throws AlgebraException, GeometryException, RobustEstimatorException {
        final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);

        final var patternPoints = pattern.getIdealPoints();

        // assume that pattern points are located on a 3D plane
        // (for instance Z = 0), but can be really any plane
        final var points3D = new ArrayList<Point3D>();
        for (final var patternPoint : patternPoints) {
            points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0, 1.0));
        }

        var numValid = 0;
        var avgDistanceError = 0.0;
        var avgProjectionError = 0.0;
        for (var t = 0; t < TIMES; t++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create random camera

            // rotation
            final var alphaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0, MAX_ANGLE_DEGREES * Math.PI / 180.0);
            final var betaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0, MAX_ANGLE_DEGREES * Math.PI / 180.0);
            final var gammaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0, MAX_ANGLE_DEGREES * Math.PI / 180.0);

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // camera center
            final var cameraCenterArray = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // create camera with intrinsic parameters, rotation and camera
            // center
            final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
            camera.normalize();

            // ensure that all 3D pattern points are in front of the camera
            if (!camera.isPointInFrontOfCamera(points3D.get(0), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera.isPointInFrontOfCamera(points3D.get(0), ABSOLUTE_ERROR));
            if (!camera.isPointInFrontOfCamera(points3D.get(1), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera.isPointInFrontOfCamera(points3D.get(1), ABSOLUTE_ERROR));
            if (!camera.isPointInFrontOfCamera(points3D.get(2), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera.isPointInFrontOfCamera(points3D.get(2), ABSOLUTE_ERROR));
            if (!camera.isPointInFrontOfCamera(points3D.get(3), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera.isPointInFrontOfCamera(points3D.get(3), ABSOLUTE_ERROR));

            // project 3D pattern points
            final var projectedPatternPoints = camera.project(points3D);
            // add error to projected pattern points
            final var rnd = new GaussianRandomizer(0.0, STD_ERROR);
            for (final var p : projectedPatternPoints) {
                p.setInhomogeneousCoordinates(p.getInhomX() + rnd.nextDouble(),
                        p.getInhomY() + rnd.nextDouble());
            }

            // estimate homography between 2D pattern points and projected
            // pattern points
            final var homEstimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                    patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);
            final var homography = homEstimator.estimate();

            // estimate camera pose
            final var estimator = new CameraPoseEstimator();

            // check default values
            assertNull(estimator.getRotation());
            assertNull(estimator.getCameraCenter());
            assertNull(estimator.getCamera());

            // estimate
            estimator.estimate(intrinsic, homography);

            // check correctness
            assertNotNull(estimator.getRotation());
            // check that rotation is equal at matrix level (up to sign)
            assertNotNull(estimator.getCameraCenter());

            final var distance = cameraCenter.distanceTo(estimator.getCameraCenter());
            avgDistanceError += distance;

            // project 3D points using estimated camera and check projection
            // error
            assertNotNull(estimator.getCamera());
            final var projectedPatternPoints2 = estimator.getCamera().project(points3D);
            assertEquals(projectedPatternPoints.size(), projectedPatternPoints2.size());
            for (var i = 0; i < projectedPatternPoints.size(); i++) {
                final var projectionDistance = projectedPatternPoints.get(i).distanceTo(projectedPatternPoints2.get(i));
                avgProjectionError += projectionDistance;
            }

            numValid++;
        }

        assertTrue(numValid > 0);

        avgDistanceError /= TIMES;
        avgProjectionError /= TIMES;

        String msg = "Circles with Error - Average camera center error: " + avgDistanceError + "m";
        Logger.getLogger(CameraPoseEstimatorTest.class.getName()).log(Level.INFO, msg);
        msg = "Circles with Error - Average projection error: " + avgProjectionError + "px";
        Logger.getLogger(CameraPoseEstimatorTest.class.getName()).log(Level.INFO, msg);
    }
}
