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
package com.irurueta.ar.epipolar.estimators;

import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidFundamentalMatrixException;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class EightPointsFundamentalMatrixEstimatorTest implements
        FundamentalMatrixEstimatorListener {

    private static final int MIN_POINTS = 8;
    private static final int MAX_POINTS = 500;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-4;
    private static final double ULTRA_LARGE_ABSOLUTE_ERROR = 1e-1;
    private static final double EXTREME_LARGE_ABSOLUTE_ERROR = 1.0;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = -50.0;

    private static final double MIN_FOCAL_LENGTH = 110.0;
    private static final double MAX_FOCAL_LENGTH = 130.0;

    private static final double MIN_SKEWNESS = -0.001;
    private static final double MAX_SKEWNESS = 0.001;

    private static final double MIN_PRINCIPAL_POINT = 90.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;

    private static final double MIN_ANGLE_DEGREES = 10.0;
    private static final double MAX_ANGLE_DEGREES = 15.0;

    private static final double MIN_CAMERA_SEPARATION = 130.0;
    private static final double MAX_CAMERA_SEPARATION = 150.0;

    private static final int TIMES = 100;

    private int estimateStart;
    private int estimateEnd;

    @Test
    public void testConstructor() {
        // test constructor without arguments
        EightPointsFundamentalMatrixEstimator estimator =
                new EightPointsFundamentalMatrixEstimator();

        // check correctness
        assertEquals(estimator.isLMSESolutionAllowed(),
                EightPointsFundamentalMatrixEstimator.
                        DEFAULT_ALLOW_LMSE_SOLUTION);
        assertEquals(estimator.arePointsNormalized(),
                EightPointsFundamentalMatrixEstimator.
                        DEFAULT_NORMALIZE_POINT_CORRESPONDENCES);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMethod(),
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(),
                EightPointsFundamentalMatrixEstimator.MIN_REQUIRED_POINTS);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());

        // test constructor with points
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final List<Point2D> leftPoints = new ArrayList<>();
        final List<Point2D> rightPoints = new ArrayList<>();
        for (int i = 0; i < nPoints; i++) {
            leftPoints.add(Point2D.create());
            rightPoints.add(Point2D.create());
        }

        estimator = new EightPointsFundamentalMatrixEstimator(leftPoints,
                rightPoints);

        // check correctness
        assertEquals(estimator.isLMSESolutionAllowed(),
                EightPointsFundamentalMatrixEstimator.
                        DEFAULT_ALLOW_LMSE_SOLUTION);
        assertEquals(estimator.arePointsNormalized(),
                EightPointsFundamentalMatrixEstimator.
                        DEFAULT_NORMALIZE_POINT_CORRESPONDENCES);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMethod(),
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(),
                EightPointsFundamentalMatrixEstimator.MIN_REQUIRED_POINTS);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();
        estimator = null;
        try {
            estimator = new EightPointsFundamentalMatrixEstimator(emptyPoints,
                    rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EightPointsFundamentalMatrixEstimator(leftPoints,
                    emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testIsSetLMSESolutionAllowed() throws LockedException {
        final EightPointsFundamentalMatrixEstimator estimator =
                new EightPointsFundamentalMatrixEstimator();

        // check default value
        assertEquals(estimator.isLMSESolutionAllowed(),
                EightPointsFundamentalMatrixEstimator.
                        DEFAULT_ALLOW_LMSE_SOLUTION);

        // set new value
        estimator.setLMSESolutionAllowed(
                !EightPointsFundamentalMatrixEstimator.
                        DEFAULT_ALLOW_LMSE_SOLUTION);

        // check correctness
        assertEquals(estimator.isLMSESolutionAllowed(),
                !EightPointsFundamentalMatrixEstimator.
                        DEFAULT_ALLOW_LMSE_SOLUTION);
    }

    @Test
    public void testAreSetPointsNormalized() throws LockedException {
        final EightPointsFundamentalMatrixEstimator estimator =
                new EightPointsFundamentalMatrixEstimator();

        // check default value
        assertEquals(estimator.arePointsNormalized(),
                EightPointsFundamentalMatrixEstimator.
                        DEFAULT_NORMALIZE_POINT_CORRESPONDENCES);

        // set new value
        estimator.setPointsNormalized(
                !EightPointsFundamentalMatrixEstimator.
                        DEFAULT_NORMALIZE_POINT_CORRESPONDENCES);

        // check correctness
        assertEquals(estimator.arePointsNormalized(),
                !EightPointsFundamentalMatrixEstimator.
                        DEFAULT_NORMALIZE_POINT_CORRESPONDENCES);
    }

    @Test
    public void testGetSetPoints() throws LockedException {
        final EightPointsFundamentalMatrixEstimator estimator =
                new EightPointsFundamentalMatrixEstimator();

        // check default value
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final List<Point2D> leftPoints = new ArrayList<>();
        final List<Point2D> rightPoints = new ArrayList<>();
        for (int i = 0; i < nPoints; i++) {
            leftPoints.add(Point2D.create());
            rightPoints.add(Point2D.create());
        }

        estimator.setPoints(leftPoints, rightPoints);

        // check correctness
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();
        try {
            estimator.setPoints(emptyPoints, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setPoints(leftPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final EightPointsFundamentalMatrixEstimator estimator =
                new EightPointsFundamentalMatrixEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testEstimateNoLMSENoNormalization() throws LockedException,
            NotReadyException, FundamentalMatrixEstimatorException,
            InvalidFundamentalMatrixException, NotAvailableException {

        EightPointsFundamentalMatrixEstimator estimator;

        double leftEpipoleError;
        double rightEpipoleError;
        double avgLeftEpipoleError = 0.0;
        double avgRightEpipoleError = 0.0;
        int numValid = 0;
        for (int j = 0; j < TIMES; j++) {
            // randomly create two pinhole cameras
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS,
                    MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS,
                    MAX_SKEWNESS);

            final double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final Point3D center1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                    gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                    gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);

            // generate a random list of 3D points
            final List<Point3D> points3D = new ArrayList<>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE), randomizer.nextDouble(
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final List<Point2D> leftPoints = camera1.project(points3D);
            final List<Point2D> rightPoints = camera2.project(points3D);

            // estimate fundamental matrix
            estimator = new EightPointsFundamentalMatrixEstimator(leftPoints,
                    rightPoints);
            estimator.setLMSESolutionAllowed(false);
            estimator.setPointsNormalized(false);
            estimator.setListener(this);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            // estimate
            final FundamentalMatrix fundMatrix = estimator.estimate();

            // check correctness
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            reset();

            // compute epipoles
            final Point2D epipole1a = camera1.project(center2);
            final Point2D epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            final Point2D epipole1b = fundMatrix.getLeftEpipole();
            final Point2D epipole2b = fundMatrix.getRightEpipole();

            // check correctness of epipoles
            leftEpipoleError = epipole1a.distanceTo(epipole1b);
            rightEpipoleError = epipole2a.distanceTo(epipole2b);
            if (leftEpipoleError > EXTREME_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(leftEpipoleError, 0.0,
                    EXTREME_LARGE_ABSOLUTE_ERROR);
            if (rightEpipoleError > EXTREME_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rightEpipoleError, 0.0,
                    EXTREME_LARGE_ABSOLUTE_ERROR);

            avgLeftEpipoleError += leftEpipoleError;
            avgRightEpipoleError += rightEpipoleError;

            // check that all points lie within their corresponding epipolar
            // lines
            for (int i = 0; i < nPoints; i++) {
                final Point2D leftPoint = leftPoints.get(i);
                final Point2D rightPoint = rightPoints.get(i);
                final Point3D point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ULTRA_LARGE_ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint, ULTRA_LARGE_ABSOLUTE_ERROR));

                // obtain epipolar planes
                final Plane epipolarPlane1 = camera1.backProject(line1);
                final Plane epipolarPlane2 = camera2.backProject(line2);

                // check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2,
                        ULTRA_LARGE_ABSOLUTE_ERROR));

                // check that point3D and camera centers belong to epipolar plane
                if (!epipolarPlane1.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(point3D,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center1, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center1,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center2, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center2,
                        EXTREME_LARGE_ABSOLUTE_ERROR));

                if (!epipolarPlane2.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(point3D,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center1, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center1,
                        EXTREME_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center2, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center2,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
            }

            numValid++;
        }

        assertTrue(numValid > 0);

        avgLeftEpipoleError /= TIMES;
        avgRightEpipoleError /= TIMES;

        assertEquals(avgLeftEpipoleError, 0.0, ULTRA_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgRightEpipoleError, 0.0, ULTRA_LARGE_ABSOLUTE_ERROR);

        // Force NotReadyException
        estimator = new EightPointsFundamentalMatrixEstimator();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateNoLMSENormalization() throws LockedException,
            NotReadyException, FundamentalMatrixEstimatorException,
            InvalidFundamentalMatrixException, NotAvailableException {

        EightPointsFundamentalMatrixEstimator estimator;

        double leftEpipoleError, rightEpipoleError;
        double avgLeftEpipoleError = 0.0, avgRightEpipoleError = 0.0;
        for (int j = 0; j < 2 * TIMES; j++) {
            // randomly create two pinhole cameras
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS,
                    MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS,
                    MAX_SKEWNESS);

            final double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final Point3D center1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                    gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                    gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);

            // generate a random list of 3D points
            final List<Point3D> points3D = new ArrayList<>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(randomizer.nextDouble(
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE), randomizer.nextDouble(
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final List<Point2D> leftPoints = camera1.project(points3D);
            final List<Point2D> rightPoints = camera2.project(points3D);

            // estimate fundamental matrix
            estimator = new EightPointsFundamentalMatrixEstimator(leftPoints,
                    rightPoints);
            estimator.setLMSESolutionAllowed(false);
            estimator.setPointsNormalized(true);
            estimator.setListener(this);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            // estimate
            final FundamentalMatrix fundMatrix = estimator.estimate();

            // check correctness
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            reset();

            // compute epipoles
            final Point2D epipole1a = camera1.project(center2);
            final Point2D epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            final Point2D epipole1b = fundMatrix.getLeftEpipole();
            final Point2D epipole2b = fundMatrix.getRightEpipole();

            // check correctness of epipoles
            leftEpipoleError = epipole1a.distanceTo(epipole1b);
            rightEpipoleError = epipole2a.distanceTo(epipole2b);
            assertEquals(leftEpipoleError, 0.0,
                    2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(rightEpipoleError, 0.0,
                    2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);

            avgLeftEpipoleError += leftEpipoleError;
            avgRightEpipoleError += rightEpipoleError;

            // check that all points lie within their corresponding epipolar
            // lines
            for (int i = 0; i < nPoints; i++) {
                final Point2D leftPoint = leftPoints.get(i);
                final Point2D rightPoint = rightPoints.get(i);
                final Point3D point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint,
                        10.0 * VERY_LARGE_ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint,
                        VERY_LARGE_ABSOLUTE_ERROR));

                // obtain epipolar planes
                final Plane epipolarPlane1 = camera1.backProject(line1);
                final Plane epipolarPlane2 = camera2.backProject(line2);

                // check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2,
                        VERY_LARGE_ABSOLUTE_ERROR));

                // check that poin3D and camera centers belong to epipolar plane
                assertTrue(epipolarPlane1.isLocus(point3D,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center1,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center2,
                        ULTRA_LARGE_ABSOLUTE_ERROR));

                assertTrue(epipolarPlane2.isLocus(point3D,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center1,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center2,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
            }
        }

        avgLeftEpipoleError /= TIMES;
        avgRightEpipoleError /= TIMES;

        assertEquals(avgLeftEpipoleError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgRightEpipoleError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);

        // Force NotReadyException
        estimator = new EightPointsFundamentalMatrixEstimator();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateLMSENoNormalization() throws LockedException,
            NotReadyException, FundamentalMatrixEstimatorException,
            InvalidFundamentalMatrixException, NotAvailableException {

        EightPointsFundamentalMatrixEstimator estimator;

        double leftEpipoleError;
        double rightEpipoleError;
        double avgLeftEpipoleError = 0.0;
        double avgRightEpipoleError = 0.0;
        for (int j = 0; j < TIMES; j++) {
            // randomly create two pinhole cameras
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS,
                    MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS,
                    MAX_SKEWNESS);

            final double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final Point3D center1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                    gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                    gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);

            // generate a random list of 3D points
            final List<Point3D> points3D = new ArrayList<>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE), randomizer.nextDouble(
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final List<Point2D> leftPoints = camera1.project(points3D);
            final List<Point2D> rightPoints = camera2.project(points3D);

            // estimate fundamental matrix
            estimator = new EightPointsFundamentalMatrixEstimator(leftPoints,
                    rightPoints);
            estimator.setLMSESolutionAllowed(true);
            estimator.setPointsNormalized(false);
            estimator.setListener(this);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            // estimate
            final FundamentalMatrix fundMatrix = estimator.estimate();

            // check correctness
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            reset();

            // compute epipoles
            final Point2D epipole1a = camera1.project(center2);
            final Point2D epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            final Point2D epipole1b = fundMatrix.getLeftEpipole();
            final Point2D epipole2b = fundMatrix.getRightEpipole();

            // check correctness of epipoles
            leftEpipoleError = epipole1a.distanceTo(epipole1b);
            rightEpipoleError = epipole2a.distanceTo(epipole2b);
            assertEquals(leftEpipoleError, 0.0,
                    EXTREME_LARGE_ABSOLUTE_ERROR);
            assertEquals(rightEpipoleError, 0.0,
                    EXTREME_LARGE_ABSOLUTE_ERROR);

            avgLeftEpipoleError += leftEpipoleError;
            avgRightEpipoleError += rightEpipoleError;

            // check that all points lie within their corresponding epipolar
            // lines
            for (int i = 0; i < nPoints; i++) {
                final Point2D leftPoint = leftPoints.get(i);
                final Point2D rightPoint = rightPoints.get(i);
                final Point3D point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ULTRA_LARGE_ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint, ULTRA_LARGE_ABSOLUTE_ERROR));

                // obtain epipolar planes
                final Plane epipolarPlane1 = camera1.backProject(line1);
                final Plane epipolarPlane2 = camera2.backProject(line2);

                // check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2,
                        ULTRA_LARGE_ABSOLUTE_ERROR));

                // check that point3D and camera centers belong to epipolar plane
                assertTrue(epipolarPlane1.isLocus(point3D,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center1,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center2,
                        EXTREME_LARGE_ABSOLUTE_ERROR));

                assertTrue(epipolarPlane2.isLocus(point3D,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center1,
                        EXTREME_LARGE_ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center2,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
            }
        }

        avgLeftEpipoleError /= TIMES;
        avgRightEpipoleError /= TIMES;

        assertEquals(avgLeftEpipoleError, 0.0, ULTRA_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgRightEpipoleError, 0.0, ULTRA_LARGE_ABSOLUTE_ERROR);

        // Force NotReadyException
        estimator = new EightPointsFundamentalMatrixEstimator();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateLMSENormalization() throws LockedException,
            NotReadyException, FundamentalMatrixEstimatorException,
            InvalidFundamentalMatrixException, NotAvailableException {

        EightPointsFundamentalMatrixEstimator estimator;

        double leftEpipoleError;
        double rightEpipoleError;
        double avgLeftEpipoleError = 0.0;
        double avgRightEpipoleError = 0.0;
        for (int j = 0; j < TIMES; j++) {
            // randomly create two pinhole cameras
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS,
                    MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS,
                    MAX_SKEWNESS);

            final double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final Point3D center1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                    gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                    gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);

            // generate a random list of 3D points
            final List<Point3D> points3D = new ArrayList<>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(randomizer.nextDouble(
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE,
                                MAX_RANDOM_VALUE), randomizer.nextDouble(
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final List<Point2D> leftPoints = camera1.project(points3D);
            final List<Point2D> rightPoints = camera2.project(points3D);

            // estimate fundamental matrix
            estimator = new EightPointsFundamentalMatrixEstimator(leftPoints,
                    rightPoints);
            estimator.setLMSESolutionAllowed(true);
            estimator.setPointsNormalized(true);
            estimator.setListener(this);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            // estimate
            final FundamentalMatrix fundMatrix = estimator.estimate();

            // check correctness
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            reset();

            // compute epipoles
            final Point2D epipole1a = camera1.project(center2);
            final Point2D epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            final Point2D epipole1b = fundMatrix.getLeftEpipole();
            final Point2D epipole2b = fundMatrix.getRightEpipole();

            // check correctness of epipoles
            leftEpipoleError = epipole1a.distanceTo(epipole1b);
            rightEpipoleError = epipole2a.distanceTo(epipole2b);
            assertEquals(leftEpipoleError, 0.0,
                    ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(rightEpipoleError, 0.0,
                    ULTRA_LARGE_ABSOLUTE_ERROR);

            avgLeftEpipoleError += leftEpipoleError;
            avgRightEpipoleError += rightEpipoleError;

            // check that all points lie within their corresponding epipolar
            // lines
            for (int i = 0; i < nPoints; i++) {
                final Point2D leftPoint = leftPoints.get(i);
                final Point2D rightPoint = rightPoints.get(i);
                final Point3D point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, VERY_LARGE_ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint,
                        VERY_LARGE_ABSOLUTE_ERROR));

                // obtain epipolar planes
                final Plane epipolarPlane1 = camera1.backProject(line1);
                final Plane epipolarPlane2 = camera2.backProject(line2);

                // check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2,
                        LARGE_ABSOLUTE_ERROR));

                // check that point3D and camera centers belong to epipolar plane
                assertTrue(epipolarPlane1.isLocus(point3D,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center1,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center2,
                        ULTRA_LARGE_ABSOLUTE_ERROR));

                assertTrue(epipolarPlane2.isLocus(point3D,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center1,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center2,
                        ULTRA_LARGE_ABSOLUTE_ERROR));
            }
        }

        avgLeftEpipoleError /= TIMES;
        avgRightEpipoleError /= TIMES;

        assertEquals(avgLeftEpipoleError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgRightEpipoleError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);

        // Force NotReadyException
        estimator = new EightPointsFundamentalMatrixEstimator();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Override
    public void onEstimateStart(final FundamentalMatrixEstimator estimator) {
        estimateStart++;
        testLocked((EightPointsFundamentalMatrixEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final FundamentalMatrixEstimator estimator,
                              final FundamentalMatrix fundamentalMatrix) {
        estimateEnd++;
        testLocked((EightPointsFundamentalMatrixEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    private void testLocked(final EightPointsFundamentalMatrixEstimator estimator) {
        assertTrue(estimator.isLocked());
        try {
            estimator.setLMSESolutionAllowed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPointsNormalized(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPoints(null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final FundamentalMatrixEstimatorException | NotReadyException e) {
            fail("LockedException expected but not thrown");
        }
    }
}
