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

public class SevenPointsFundamentalMatrixEstimatorTest implements FundamentalMatrixEstimatorListener {

    private static final int MIN_POINTS = 7;
    private static final int MAX_POINTS = 500;

    private static final double ABSOLUTE_ERROR = 1e-6;
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
        SevenPointsFundamentalMatrixEstimator estimator = new SevenPointsFundamentalMatrixEstimator();

        // check correctness
        assertEquals(SevenPointsFundamentalMatrixEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());
        assertEquals(SevenPointsFundamentalMatrixEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES,
                estimator.arePointsNormalized());
        assertFalse(estimator.isReady());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getMethod());
        assertEquals(SevenPointsFundamentalMatrixEstimator.MIN_REQUIRED_POINTS,
                estimator.getMinRequiredPoints());
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

        estimator = new SevenPointsFundamentalMatrixEstimator(leftPoints, rightPoints);

        // check correctness
        assertEquals(SevenPointsFundamentalMatrixEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());
        assertEquals(SevenPointsFundamentalMatrixEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES,
                estimator.arePointsNormalized());
        assertTrue(estimator.isReady());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getMethod());
        assertEquals(SevenPointsFundamentalMatrixEstimator.MIN_REQUIRED_POINTS,
                estimator.getMinRequiredPoints());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();
        estimator = null;
        try {
            estimator = new SevenPointsFundamentalMatrixEstimator(emptyPoints, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new SevenPointsFundamentalMatrixEstimator(leftPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testIsSetLMSESolutionAllowed() throws LockedException {
        final SevenPointsFundamentalMatrixEstimator estimator =
                new SevenPointsFundamentalMatrixEstimator();

        // check default value
        assertEquals(SevenPointsFundamentalMatrixEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());

        // set new value
        estimator.setLMSESolutionAllowed(
                !SevenPointsFundamentalMatrixEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);

        // check correctness
        assertEquals(!SevenPointsFundamentalMatrixEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());
    }

    @Test
    public void testAreSetPointsNormalized() throws LockedException {
        final SevenPointsFundamentalMatrixEstimator estimator =
                new SevenPointsFundamentalMatrixEstimator();

        // check default value
        assertEquals(SevenPointsFundamentalMatrixEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES,
                estimator.arePointsNormalized());

        // set new value
        estimator.setPointsNormalized(
                !SevenPointsFundamentalMatrixEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES);

        // check correctness
        assertEquals(!SevenPointsFundamentalMatrixEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES,
                estimator.arePointsNormalized());
    }

    @Test
    public void testGetSetPoints() throws LockedException {
        final SevenPointsFundamentalMatrixEstimator estimator =
                new SevenPointsFundamentalMatrixEstimator();

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
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());

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
        final SevenPointsFundamentalMatrixEstimator estimator =
                new SevenPointsFundamentalMatrixEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testEstimateAllNoLMSENoNormalization() throws LockedException, NotReadyException,
            FundamentalMatrixEstimatorException, InvalidFundamentalMatrixException, NotAvailableException {

        SevenPointsFundamentalMatrixEstimator estimator;

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

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

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

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            // generate a random list of 3D points
            final List<Point3D> points3D = new ArrayList<>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final List<Point2D> leftPoints = camera1.project(points3D);
            final List<Point2D> rightPoints = camera2.project(points3D);

            // estimate fundamental matrix
            estimator = new SevenPointsFundamentalMatrixEstimator(leftPoints, rightPoints);
            estimator.setLMSESolutionAllowed(false);
            estimator.setPointsNormalized(false);
            estimator.setListener(this);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate all
            final List<FundamentalMatrix> fundMatrixList = estimator.estimateAll();

            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            reset();

            if (fundMatrixList.size() > 1) {
                // we have more than one solution, so estimate method should
                // fail!
                try {
                    estimator.estimate();
                    fail("FundamentalMatrixEstimatorException expected but not thrown");
                } catch (FundamentalMatrixEstimatorException ignore) {
                }

                assertEquals(1, estimateStart);
                assertEquals(1, estimateEnd);
                reset();
            }

            // check correctness
            // (at least one of the fundamental matrix solutions must be valid)
            boolean epipoleLeftCorrect, epipoleRightCorrect;
            for (final FundamentalMatrix fundMatrix : fundMatrixList) {

                // compute epipoles
                final Point2D epipole1a = camera1.project(center2);
                final Point2D epipole2a = camera2.project(center1);

                fundMatrix.computeEpipoles();

                final Point2D epipole1b = fundMatrix.getLeftEpipole();
                final Point2D epipole2b = fundMatrix.getRightEpipole();

                // check correctness of epipoles
                leftEpipoleError = epipole1a.distanceTo(epipole1b);
                rightEpipoleError = epipole2a.distanceTo(epipole2b);
                epipoleLeftCorrect = leftEpipoleError <= EXTREME_LARGE_ABSOLUTE_ERROR;
                epipoleRightCorrect = rightEpipoleError <= EXTREME_LARGE_ABSOLUTE_ERROR;

                boolean validPoints = true;
                if (epipoleLeftCorrect && epipoleRightCorrect) {
                    // check that all points lie within their corresponding epipolar
                    // lines
                    for (int i = 0; i < nPoints; i++) {
                        final Point2D leftPoint = leftPoints.get(i);
                        final Point2D rightPoint = rightPoints.get(i);
                        final Point3D point3D = points3D.get(i);

                        // obtain epipolar line on left view using 2D point on
                        // right view
                        final Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                        // obtain epipolar line on right view using 2D point on
                        // left view
                        final Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                        // check that 2D point on left view belongs to left
                        // epipolar line
                        if (!line1.isLocus(leftPoint, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                            validPoints = false;
                            break;
                        }
                        assertTrue(line1.isLocus(leftPoint, ULTRA_LARGE_ABSOLUTE_ERROR));

                        // check that 2D point on right view belongs to right
                        // epipolar line
                        if (!line2.isLocus(rightPoint, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                            validPoints = false;
                            break;
                        }
                        assertTrue(line2.isLocus(rightPoint, ULTRA_LARGE_ABSOLUTE_ERROR));

                        // obtain epipolar planes
                        final Plane epipolarPlane1 = camera1.backProject(line1);
                        final Plane epipolarPlane2 = camera2.backProject(line2);

                        // check that both planes are the same
                        if (!epipolarPlane1.equals(epipolarPlane2,
                                2.0 * ULTRA_LARGE_ABSOLUTE_ERROR)) {
                            validPoints = false;
                            break;
                        }
                        assertTrue(epipolarPlane1.equals(epipolarPlane2,
                                2.0 * ULTRA_LARGE_ABSOLUTE_ERROR));

                        // check that point3D and camera centers belong to
                        // epipolar plane
                        if (!epipolarPlane1.isLocus(point3D, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                            validPoints = false;
                            break;
                        }
                        assertTrue(epipolarPlane1.isLocus(point3D, EXTREME_LARGE_ABSOLUTE_ERROR));
                        if (!epipolarPlane1.isLocus(center1, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                            validPoints = false;
                            break;
                        }
                        assertTrue(epipolarPlane1.isLocus(center1, EXTREME_LARGE_ABSOLUTE_ERROR));
                        if (!epipolarPlane1.isLocus(center2, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                            validPoints = false;
                            break;
                        }
                        assertTrue(epipolarPlane1.isLocus(center2, EXTREME_LARGE_ABSOLUTE_ERROR));

                        if (!epipolarPlane2.isLocus(point3D, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                            validPoints = false;
                            break;
                        }
                        assertTrue(epipolarPlane2.isLocus(point3D, EXTREME_LARGE_ABSOLUTE_ERROR));
                        if (!epipolarPlane2.isLocus(center1, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                            validPoints = false;
                            break;
                        }
                        assertTrue(epipolarPlane2.isLocus(center1, EXTREME_LARGE_ABSOLUTE_ERROR));
                        if (!epipolarPlane2.isLocus(center2, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                            validPoints = false;
                            break;
                        }
                        assertTrue(epipolarPlane2.isLocus(center2, EXTREME_LARGE_ABSOLUTE_ERROR));
                    }
                }

                if (epipoleLeftCorrect && epipoleRightCorrect && validPoints) {
                    avgLeftEpipoleError += leftEpipoleError;
                    avgRightEpipoleError += rightEpipoleError;
                    numValid++;
                }
            }
        }

        assertTrue(numValid > 0);
        avgLeftEpipoleError /= numValid;
        avgRightEpipoleError /= numValid;


        assertEquals(0.0, avgLeftEpipoleError, ULTRA_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgRightEpipoleError, ULTRA_LARGE_ABSOLUTE_ERROR);

        // Force NotReadyException
        estimator = new SevenPointsFundamentalMatrixEstimator();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateAllNoLMSENormalization() throws LockedException, NotReadyException,
            FundamentalMatrixEstimatorException, InvalidFundamentalMatrixException,
            NotAvailableException {

        SevenPointsFundamentalMatrixEstimator estimator;

        double leftEpipoleError;
        double rightEpipoleError;
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

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

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

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            // generate a random list of 3D points
            final List<Point3D> points3D = new ArrayList<>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final List<Point2D> leftPoints = camera1.project(points3D);
            final List<Point2D> rightPoints = camera2.project(points3D);

            // estimate fundamental matrix
            estimator = new SevenPointsFundamentalMatrixEstimator(leftPoints, rightPoints);
            estimator.setLMSESolutionAllowed(false);
            estimator.setPointsNormalized(true);
            estimator.setListener(this);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate all
            final List<FundamentalMatrix> fundMatrixList = estimator.estimateAll();

            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            reset();

            if (fundMatrixList.size() > 1) {
                // we have more than one solution, so estimate method should
                // fail!
                try {
                    estimator.estimate();
                    fail("FundamentalMatrixEstimatorException expected but not thrown");
                } catch (final FundamentalMatrixEstimatorException ignore) {
                }

                assertEquals(1, estimateStart);
                assertEquals(1, estimateEnd);
                reset();
            }

            // check correctness
            // (at least one of the fundamental matrix solutions must be valid)
            boolean epipoleLeftCorrect;
            boolean epipoleRightCorrect;
            for (final FundamentalMatrix fundMatrix : fundMatrixList) {
                // compute epipoles
                final Point2D epipole1a = camera1.project(center2);
                final Point2D epipole2a = camera2.project(center1);

                fundMatrix.computeEpipoles();

                final Point2D epipole1b = fundMatrix.getLeftEpipole();
                final Point2D epipole2b = fundMatrix.getRightEpipole();

                // check correctness of epipoles
                leftEpipoleError = epipole1a.distanceTo(epipole1b);
                rightEpipoleError = epipole2a.distanceTo(epipole2b);
                epipoleLeftCorrect = leftEpipoleError <= ULTRA_LARGE_ABSOLUTE_ERROR;
                epipoleRightCorrect = rightEpipoleError <= ULTRA_LARGE_ABSOLUTE_ERROR;

                if (epipoleLeftCorrect && epipoleRightCorrect) {
                    // check that all points lie within their corresponding epipolar
                    // lines
                    for (int i = 0; i < nPoints; i++) {
                        final Point2D leftPoint = leftPoints.get(i);
                        final Point2D rightPoint = rightPoints.get(i);
                        final Point3D point3D = points3D.get(i);

                        // obtain epipolar line on left view using 2D point on
                        // right view
                        final Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                        // obtain epipolar line on right view using 2D point on
                        // left view
                        final Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                        // check that 2D point on left view belongs to left
                        // epipolar line
                        assertTrue(line1.isLocus(leftPoint, ULTRA_LARGE_ABSOLUTE_ERROR));
                        // check that 2D point on right view belongs to right
                        // epipolar line
                        assertTrue(line2.isLocus(rightPoint, ULTRA_LARGE_ABSOLUTE_ERROR));

                        // obtain epipolar planes
                        final Plane epipolarPlane1 = camera1.backProject(line1);
                        final Plane epipolarPlane2 = camera2.backProject(line2);

                        // check that both planes are the same
                        assertTrue(epipolarPlane1.equals(epipolarPlane2, VERY_LARGE_ABSOLUTE_ERROR));

                        // check that point3D and camera centers belong to
                        // epipolar plane
                        assertTrue(epipolarPlane1.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR));
                        assertTrue(epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR));
                        assertTrue(epipolarPlane1.isLocus(center2, ULTRA_LARGE_ABSOLUTE_ERROR));

                        assertTrue(epipolarPlane2.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR));
                        assertTrue(epipolarPlane2.isLocus(center1, ULTRA_LARGE_ABSOLUTE_ERROR));
                        assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));
                    }
                }

                if (epipoleLeftCorrect && epipoleRightCorrect) {
                    numValid++;
                }
            }

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);

        // Force NotReadyException
        estimator = new SevenPointsFundamentalMatrixEstimator();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateAllLMSENoNormalization() throws LockedException, NotReadyException,
            FundamentalMatrixEstimatorException, InvalidFundamentalMatrixException, NotAvailableException {

        SevenPointsFundamentalMatrixEstimator estimator;

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

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

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

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            // generate a random list of 3D points
            final List<Point3D> points3D = new ArrayList<>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final List<Point2D> leftPoints = camera1.project(points3D);
            final List<Point2D> rightPoints = camera2.project(points3D);

            // estimate fundamental matrix
            estimator = new SevenPointsFundamentalMatrixEstimator(leftPoints, rightPoints);
            estimator.setLMSESolutionAllowed(true);
            estimator.setPointsNormalized(false);
            estimator.setListener(this);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate all
            final List<FundamentalMatrix> fundMatrixList = estimator.estimateAll();

            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            reset();

            if (fundMatrixList.size() > 1) {
                // we have more than one solution, so estimate method should
                // fail!
                try {
                    estimator.estimate();
                    fail("FundamentalMatrixEstimatorException expected but not thrown");
                } catch (final FundamentalMatrixEstimatorException ignore) {
                }

                assertEquals(1, estimateStart);
                assertEquals(1, estimateEnd);
                reset();
            }

            // check correctness
            // (at least one of the fundamental matrix solutions must be valid)
            boolean epipoleLeftCorrect;
            boolean epipoleRightCorrect;
            for (final FundamentalMatrix fundMatrix : fundMatrixList) {

                // compute epipoles
                final Point2D epipole1a = camera1.project(center2);
                final Point2D epipole2a = camera2.project(center1);

                fundMatrix.computeEpipoles();

                final Point2D epipole1b = fundMatrix.getLeftEpipole();
                final Point2D epipole2b = fundMatrix.getRightEpipole();

                // check correctness of epipoles
                leftEpipoleError = epipole1a.distanceTo(epipole1b);
                rightEpipoleError = epipole2a.distanceTo(epipole2b);
                epipoleLeftCorrect = leftEpipoleError <= EXTREME_LARGE_ABSOLUTE_ERROR;
                epipoleRightCorrect = rightEpipoleError <= EXTREME_LARGE_ABSOLUTE_ERROR;

                if (epipoleLeftCorrect && epipoleRightCorrect) {
                    // check that all points lie within their corresponding
                    // epipolar lines
                    boolean failed = false;
                    for (int i = 0; i < nPoints; i++) {
                        final Point2D leftPoint = leftPoints.get(i);
                        final Point2D rightPoint = rightPoints.get(i);
                        final Point3D point3D = points3D.get(i);

                        // obtain epipolar line on left view using 2D point on
                        // right view
                        final Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                        // obtain epipolar line on right view using 2D point on
                        // left view
                        final Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                        // check that 2D point on left view belongs to left
                        // epipolar line
                        if (!line1.isLocus(leftPoint, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(line1.isLocus(leftPoint, ULTRA_LARGE_ABSOLUTE_ERROR));
                        // check that 2D point on right view belongs to right
                        // epipolar line
                        if (!line2.isLocus(rightPoint, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(line2.isLocus(rightPoint, ULTRA_LARGE_ABSOLUTE_ERROR));

                        // obtain epipolar planes
                        final Plane epipolarPlane1 = camera1.backProject(line1);
                        final Plane epipolarPlane2 = camera2.backProject(line2);

                        // check that both planes are the same
                        if (!epipolarPlane1.equals(epipolarPlane2, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(epipolarPlane1.equals(epipolarPlane2, ULTRA_LARGE_ABSOLUTE_ERROR));

                        // check that point3D and camera centers belong to
                        // epipolar plane
                        if (!epipolarPlane1.isLocus(point3D, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(epipolarPlane1.isLocus(point3D, EXTREME_LARGE_ABSOLUTE_ERROR));
                        if (!epipolarPlane1.isLocus(center1, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(epipolarPlane1.isLocus(center1, EXTREME_LARGE_ABSOLUTE_ERROR));
                        if (!epipolarPlane1.isLocus(center2, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(epipolarPlane1.isLocus(center2, EXTREME_LARGE_ABSOLUTE_ERROR));

                        if (!epipolarPlane2.isLocus(point3D, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(epipolarPlane2.isLocus(point3D, EXTREME_LARGE_ABSOLUTE_ERROR));
                        if (!epipolarPlane2.isLocus(center1, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(epipolarPlane2.isLocus(center1, EXTREME_LARGE_ABSOLUTE_ERROR));
                        if (!epipolarPlane2.isLocus(center2, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(epipolarPlane2.isLocus(center2, EXTREME_LARGE_ABSOLUTE_ERROR));
                    }

                    if (failed) {
                        continue;
                    }
                }

                if (epipoleLeftCorrect && epipoleRightCorrect) {
                    avgLeftEpipoleError += leftEpipoleError;
                    avgRightEpipoleError += rightEpipoleError;
                    numValid++;
                }
            }
        }

        assertTrue(numValid > 0);
        avgLeftEpipoleError /= numValid;
        avgRightEpipoleError /= numValid;


        assertEquals(0.0, avgLeftEpipoleError, ULTRA_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgRightEpipoleError, ULTRA_LARGE_ABSOLUTE_ERROR);

        // Force NotReadyException
        estimator = new SevenPointsFundamentalMatrixEstimator();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateAllLMSENormalization() throws LockedException, NotReadyException,
            FundamentalMatrixEstimatorException, InvalidFundamentalMatrixException, NotAvailableException {

        SevenPointsFundamentalMatrixEstimator estimator;

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

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

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

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            // generate a random list of 3D points
            final List<Point3D> points3D = new ArrayList<>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final List<Point2D> leftPoints = camera1.project(points3D);
            final List<Point2D> rightPoints = camera2.project(points3D);

            // estimate fundamental matrix
            estimator = new SevenPointsFundamentalMatrixEstimator(leftPoints, rightPoints);
            estimator.setLMSESolutionAllowed(true);
            estimator.setPointsNormalized(true);
            estimator.setListener(this);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate all
            final List<FundamentalMatrix> fundMatrixList = estimator.estimateAll();

            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            reset();

            if (fundMatrixList.size() > 1) {
                // we have more than one solution, so estimate method should
                // fail!
                try {
                    estimator.estimate();
                    fail("FundamentalMatrixEstimatorException expected but not thrown");
                } catch (final FundamentalMatrixEstimatorException ignore) {
                }

                assertEquals(1, estimateStart);
                assertEquals(1, estimateEnd);
                reset();
            }

            // check correctness
            // (at least one of the fundamental matrix solutions must be valid)
            boolean epipoleLeftCorrect;
            boolean epipoleRightCorrect;
            for (final FundamentalMatrix fundMatrix : fundMatrixList) {

                // compute epipoles
                final Point2D epipole1a = camera1.project(center2);
                final Point2D epipole2a = camera2.project(center1);

                fundMatrix.computeEpipoles();

                final Point2D epipole1b = fundMatrix.getLeftEpipole();
                final Point2D epipole2b = fundMatrix.getRightEpipole();

                // check correctness of epipoles
                leftEpipoleError = epipole1a.distanceTo(epipole1b);
                rightEpipoleError = epipole2a.distanceTo(epipole2b);
                epipoleLeftCorrect = leftEpipoleError <= ULTRA_LARGE_ABSOLUTE_ERROR;
                epipoleRightCorrect = rightEpipoleError <= ULTRA_LARGE_ABSOLUTE_ERROR;

                if (epipoleLeftCorrect && epipoleRightCorrect) {
                    // check that all points lie within their corresponding
                    // epipolar lines
                    boolean failed = false;
                    for (int i = 0; i < nPoints; i++) {
                        final Point2D leftPoint = leftPoints.get(i);
                        final Point2D rightPoint = rightPoints.get(i);
                        final Point3D point3D = points3D.get(i);

                        // obtain epipolar line on left view using 2D point on
                        // right view
                        final Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                        // obtain epipolar line on right view using 2D point on
                        // left view
                        final Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                        // check that 2D point on left view belongs to left
                        // epipolar line
                        if (!line1.isLocus(leftPoint, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(line1.isLocus(leftPoint, ULTRA_LARGE_ABSOLUTE_ERROR));
                        // check that 2D point on right view belongs to right
                        // epipolar line
                        if (!line2.isLocus(rightPoint, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(line2.isLocus(rightPoint, ULTRA_LARGE_ABSOLUTE_ERROR));

                        // obtain epipolar planes
                        final Plane epipolarPlane1 = camera1.backProject(line1);
                        final Plane epipolarPlane2 = camera2.backProject(line2);

                        // check that both planes are the same
                        if (!epipolarPlane1.equals(epipolarPlane2, VERY_LARGE_ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(epipolarPlane1.equals(epipolarPlane2, VERY_LARGE_ABSOLUTE_ERROR));

                        // check that point3D and camera centers belong to
                        // epipolar plane
                        if (!epipolarPlane1.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(epipolarPlane1.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR));
                        if (!epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR));
                        if (!epipolarPlane1.isLocus(center2, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(epipolarPlane1.isLocus(center2, ULTRA_LARGE_ABSOLUTE_ERROR));

                        if (!epipolarPlane2.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(epipolarPlane2.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR));
                        if (!epipolarPlane2.isLocus(center1, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(epipolarPlane2.isLocus(center1, ULTRA_LARGE_ABSOLUTE_ERROR));
                        if (!epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR)) {
                            failed = true;
                            break;
                        }
                        assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));
                    }

                    if (failed) {
                        continue;
                    }
                }

                if (epipoleLeftCorrect && epipoleRightCorrect) {
                    avgLeftEpipoleError += leftEpipoleError;
                    avgRightEpipoleError += rightEpipoleError;
                    numValid++;
                }
            }
        }

        assertTrue(numValid > 0);
        avgLeftEpipoleError /= numValid;
        avgRightEpipoleError /= numValid;

        assertEquals(0.0, avgLeftEpipoleError, 10.0 * VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgRightEpipoleError, 10.0 * VERY_LARGE_ABSOLUTE_ERROR);

        // Force NotReadyException
        estimator = new SevenPointsFundamentalMatrixEstimator();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Override
    public void onEstimateStart(final FundamentalMatrixEstimator estimator) {
        estimateStart++;
        testLocked((SevenPointsFundamentalMatrixEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final FundamentalMatrixEstimator estimator,
                              final FundamentalMatrix fundamentalMatrix) {
        estimateEnd++;
        testLocked((SevenPointsFundamentalMatrixEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    private void testLocked(final SevenPointsFundamentalMatrixEstimator estimator) {
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
