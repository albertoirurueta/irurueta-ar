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
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class CameraCalibratorSampleTest {

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
    public void testConstructor() {
        // test constructor without arguments
        CameraCalibratorSample sample = new CameraCalibratorSample();

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
        final List<Point2D> sampledMarkers = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
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
        final List<Point2D> emptyMarkers = new ArrayList<>();
        sample = null;
        try {
            sample = new CameraCalibratorSample(emptyMarkers);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(sample);

        // test constructor with sampled markers and quality scores
        final double[] qualityScores = new double[4];
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
        final double[] shortQualityScores = new double[1];
        sample = null;
        try {
            sample = new CameraCalibratorSample(sampledMarkers, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(sample);

        // test constructor with pattern and sampled markers
        final Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
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
        sample = null;
        try {
            sample = new CameraCalibratorSample(pattern, emptyMarkers);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(sample);

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
        sample = null;
        try {
            sample = new CameraCalibratorSample(pattern, sampledMarkers, shortQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(sample);
    }

    @Test
    public void testGetSetPattern() {
        final CameraCalibratorSample sample = new CameraCalibratorSample();

        assertNull(sample.getPattern());

        // set new value
        final Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
        sample.setPattern(pattern);

        // check correctness
        assertSame(pattern, sample.getPattern());
    }

    @Test
    public void testGetSetSampledMarkers() {
        final CameraCalibratorSample sample = new CameraCalibratorSample();

        // check default value
        assertNull(sample.getSampledMarkers());

        // set new value
        final List<Point2D> sampledMarkers = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            sampledMarkers.add(Point2D.create());
        }
        sample.setSampledMarkers(sampledMarkers);

        // check correctness
        assertSame(sample.getSampledMarkers(), sampledMarkers);

        // Force IllegalArgumentException
        final List<Point2D> emptyMarkers = new ArrayList<>();
        try {
            sample.setSampledMarkers(emptyMarkers);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetSampledMarkersQualityScores() {
        final CameraCalibratorSample sample = new CameraCalibratorSample();

        // check default value
        assertNull(sample.getSampledMarkersQualityScores());

        // set new value
        final double[] qualityScores = new double[4];
        sample.setSampledMarkersQualityScores(qualityScores);

        // check correctness
        assertSame(qualityScores, sample.getSampledMarkersQualityScores());

        // Force IllegalArgumentException
        final double[] shortScores = new double[1];
        try {
            sample.setSampledMarkersQualityScores(shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testComputeSampledMarkersQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final int numMarkers = randomizer.nextInt(MIN_NUM_MARKERS, MAX_NUM_MARKERS);

        final Point2D center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final List<Point2D> sampledMarkers = new ArrayList<>();
        Point2D marker;
        final double[] scoresWithCenter = new double[numMarkers];
        final double[] scoresNoCenter = new double[numMarkers];
        double distance;
        for (int i = 0; i < numMarkers; i++) {
            marker = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            sampledMarkers.add(marker);

            // distance with center at origin of coordinates
            distance = Math.sqrt(Math.pow(marker.getInhomX(), 2.0) +
                    Math.pow(marker.getInhomY(), 2.0));
            scoresNoCenter[i] = 1.0 / (1.0 + distance);

            // distance respect to center
            distance = Math.sqrt(Math.pow(marker.getInhomX() - center.getInhomX(), 2.0) +
                    Math.pow(marker.getInhomY() - center.getInhomY(), 2.0));
            scoresWithCenter[i] = 1.0 / (1.0 + distance);
        }

        // check correctness
        assertArrayEquals(scoresNoCenter,
                CameraCalibratorSample.computeSampledMarkersQualityScores(sampledMarkers), ABSOLUTE_ERROR);

        assertArrayEquals(scoresWithCenter,
                CameraCalibratorSample.computeSampledMarkersQualityScores(sampledMarkers, center),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSetUndistortedMarkers() {
        final List<Point2D> sampledMarkers = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            sampledMarkers.add(Point2D.create());
        }
        final CameraCalibratorSample sample = new CameraCalibratorSample(sampledMarkers);

        // check default value
        assertNull(sample.getUndistortedMarkers());

        // set new value
        final List<Point2D> undistortedMarkers = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            undistortedMarkers.add(Point2D.create());
        }
        sample.setUndistortedMarkers(undistortedMarkers);

        // check correctness
        assertSame(undistortedMarkers, sample.getUndistortedMarkers());
    }

    @Test
    public void testGetSetHomography() {
        final CameraCalibratorSample sample = new CameraCalibratorSample();

        // check default value
        assertNull(sample.getHomography());

        // set new value
        final ProjectiveTransformation2D homography = new ProjectiveTransformation2D();
        sample.setHomography(homography);

        // check correctness
        assertSame(homography, sample.getHomography());
    }

    @Test
    public void testGetSetRotation() {
        final CameraCalibratorSample sample = new CameraCalibratorSample();

        // check default value
        assertNull(sample.getRotation());

        // set new value
        final Rotation3D r = Rotation3D.create();
        sample.setRotation(r);

        // check correctness
        assertSame(r, sample.getRotation());
    }

    @Test
    public void testGetSetCameraCenter() {
        final CameraCalibratorSample sample = new CameraCalibratorSample();

        // check default value
        assertNull(sample.getCameraCenter());

        // set new value
        final Point3D center = Point3D.create();
        sample.setCameraCenter(center);

        // check correctness
        assertSame(center, sample.getCameraCenter());
    }

    @Test
    public void testGetSetCamera() {
        final CameraCalibratorSample sample = new CameraCalibratorSample();

        // check default value
        assertNull(sample.getCamera());

        // set new value
        final PinholeCamera camera = new PinholeCamera();
        sample.setCamera(camera);

        // check correctness
        assertSame(camera, sample.getCamera());
    }

    @Test
    public void testEstimateHomographyCirclesPattern() throws LockedException, NotReadyException,
            RobustEstimatorException, CoincidentPointsException {

        int totalPoints = 0;
        double avgTotalError = 0.0;
        for (int j = 0; j < TIMES; j++) {
            final Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final List<Point2D> patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final List<Point3D> points3D = new ArrayList<>();
            for (final Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            // create random camera to project 3D points
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double horizontalPrincipalPoint = 0.0;
            final double verticalPrincipalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            // rotation
            final double alphaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0,
                    MAX_ANGLE_DEGREES * Math.PI / 180.0);
            final double betaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0,
                    MAX_ANGLE_DEGREES * Math.PI / 180.0);
            final double gammaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0,
                    MAX_ANGLE_DEGREES * Math.PI / 180.0);

            final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                    betaEuler, gammaEuler);

            // camera center
            final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // create camera with intrinsic parameters, rotation and camera
            // center
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
            camera.normalize();

            // project 3D pattern points
            final List<Point2D> projectedPatternPoints = camera.project(points3D);

            // create sample with projected pattern markers
            final CameraCalibratorSample sample = new CameraCalibratorSample(
                    projectedPatternPoints, CameraCalibratorSample.
                    computeSampledMarkersQualityScores(projectedPatternPoints));

            // estimate homography using ideal markers as reference
            final PointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                    PointCorrespondenceProjectiveTransformation2DRobustEstimator.create();
            final Transformation2D homography = sample.estimateHomography(estimator, patternPoints);

            // check that points are properly transformed
            double distance;
            for (int i = 0; i < patternPoints.size(); i++) {
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
    public void testEstimateHomographyQRPattern() throws LockedException, NotReadyException,
            RobustEstimatorException {

        int totalPoints = 0;
        double avgTotalError = 0.0;
        for (int j = 0; j < 2 * TIMES; j++) {
            final Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
            final List<Point2D> patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final List<Point3D> points3D = new ArrayList<>();
            for (final Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            // create random camera to project 3D points
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double horizontalPrincipalPoint = 0.0;
            final double verticalPrincipalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            // rotation
            final double alphaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0,
                    MAX_ANGLE_DEGREES * Math.PI / 180.0);
            final double betaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0,
                    MAX_ANGLE_DEGREES * Math.PI / 180.0);
            final double gammaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0,
                    MAX_ANGLE_DEGREES * Math.PI / 180.0);

            final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // camera center
            final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // create camera with intrinsic parameters, rotation and camera
            // center
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
            camera.normalize();

            // project 3D pattern points
            final List<Point2D> projectedPatternPoints = camera.project(points3D);

            // create sample with projected pattern markers
            final CameraCalibratorSample sample = new CameraCalibratorSample(
                    projectedPatternPoints, CameraCalibratorSample.computeSampledMarkersQualityScores(
                            projectedPatternPoints));

            // estimate homography using ideal markers as reference
            final PointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                    PointCorrespondenceProjectiveTransformation2DRobustEstimator.create();
            final Transformation2D homography;
            try {
                homography = sample.estimateHomography(estimator, patternPoints);
            } catch (final CoincidentPointsException e) {
                continue;
            }

            // check that points are properly transformed
            double distance;
            for (int i = 0; i < patternPoints.size(); i++) {
                distance = projectedPatternPoints.get(i).distanceTo(
                        homography.transformAndReturnNew(patternPoints.get(i)));
                avgTotalError += distance;
                totalPoints++;
            }
        }

        avgTotalError /= totalPoints;
        assertEquals(0.0, avgTotalError, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testComputeCameraPose() throws LockedException, NotReadyException, RobustEstimatorException,
            CoincidentPointsException, CalibrationException, NotAvailableException, WrongSizeException {

        int totalPoints = 0;
        double avgProjectionError = 0.0;
        double avgCenterError = 0.0;
        for (int j = 0; j < TIMES; j++) {
            final Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final List<Point2D> patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0)
            final List<Point3D> points3D = new ArrayList<>();
            for (final Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            // create random camera to project 3D points
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double horizontalPrincipalPoint = 0.0;
            final double verticalPrincipalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            // rotation
            final double alphaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0,
                    MAX_ANGLE_DEGREES * Math.PI / 180.0);
            final double betaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0,
                    MAX_ANGLE_DEGREES * Math.PI / 180.0);
            final double gammaEuler = randomizer.nextDouble(
                    MIN_ANGLE_DEGREES * Math.PI / 180.0,
                    MAX_ANGLE_DEGREES * Math.PI / 180.0);

            final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            // camera center
            final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

            // create camera with intrinsic parameters, rotation and camera
            // center
            final PinholeCamera camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
            camera.normalize();

            // project 3D pattern points
            final List<Point2D> projectedPatternPoints = camera.project(points3D);

            // create sample with projected pattern markers
            final CameraCalibratorSample sample = new CameraCalibratorSample(
                    projectedPatternPoints, CameraCalibratorSample.computeSampledMarkersQualityScores(
                            projectedPatternPoints));

            // estimate homography using ideal markers as reference
            final PointCorrespondenceProjectiveTransformation2DRobustEstimator estimator =
                    PointCorrespondenceProjectiveTransformation2DRobustEstimator.create();
            final Transformation2D homography = sample.estimateHomography(estimator, patternPoints);

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
            final Matrix sRotMat = sample.getRotation().asInhomogeneousMatrix();
            final Matrix rotMat = rotation.asInhomogeneousMatrix();
            final Matrix rotDiff = new Matrix(3, 3);
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    // signs of columns 1,2 and column 3 of rotation might
                    // be reversed and rotation would still be equal
                    rotDiff.setElementAt(r, c,
                            Math.abs(sRotMat.getElementAt(r, c)) -
                                    Math.abs(rotMat.getElementAt(r, c)));
                }
            }
            assertEquals(0.0, Utils.normF(rotDiff), LARGE_ABSOLUTE_ERROR);

            // compare center
            avgCenterError += sample.getCameraCenter().distanceTo(cameraCenter);

            // compare camera parameters
            assertSame(sample.getCamera().getIntrinsicParameters(), intrinsic);
            assertSame(sample.getCamera().getCameraRotation(), sample.getRotation());
            assertEquals(0.0, sample.getCamera().getCameraCenter().distanceTo(
                    sample.getCameraCenter()), ABSOLUTE_ERROR);

            // project ideal pattern points using estimated camera and
            // compare against sampled points
            final List<Point2D> projectedPatternPoints2 = sample.getCamera().project(points3D);
            double distance;
            for (int i = 0; i < patternPoints.size(); i++) {
                distance = projectedPatternPoints.get(i).distanceTo(
                        projectedPatternPoints2.get(i));
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
