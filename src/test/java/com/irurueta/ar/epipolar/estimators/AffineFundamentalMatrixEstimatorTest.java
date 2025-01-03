/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.algebra.WrongSizeException;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidFundamentalMatrixException;
import com.irurueta.ar.epipolar.InvalidPairOfCamerasException;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class AffineFundamentalMatrixEstimatorTest implements FundamentalMatrixEstimatorListener {

    private static final int MIN_POINTS = 4;
    private static final int MAX_POINTS = 500;

    private static final double ABSOLUTE_ERROR = 1e-6;
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
    void testConstructor() {
        // test constructor without arguments
        var estimator = new AffineFundamentalMatrixEstimator();

        // check correctness
        assertEquals(AffineFundamentalMatrixEstimator.DEFAULT_ALLOW_LMSE_SOLUTION, estimator.isLMSESolutionAllowed());
        assertEquals(AffineFundamentalMatrixEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES,
                estimator.arePointsNormalized());
        assertFalse(estimator.isReady());
        assertEquals(FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM, estimator.getMethod());
        assertEquals(AffineFundamentalMatrixEstimator.MIN_REQUIRED_POINTS, estimator.getMinRequiredPoints());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());

        // test constructor with points
        final var randomizer = new UniformRandomizer();
        final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final var leftPoints = new ArrayList<Point2D>();
        final var rightPoints = new ArrayList<Point2D>();
        for (var i = 0; i < nPoints; i++) {
            leftPoints.add(Point2D.create());
            rightPoints.add(Point2D.create());
        }

        estimator = new AffineFundamentalMatrixEstimator(leftPoints, rightPoints);

        // check correctness
        assertEquals(AffineFundamentalMatrixEstimator.DEFAULT_ALLOW_LMSE_SOLUTION, estimator.isLMSESolutionAllowed());
        assertEquals(AffineFundamentalMatrixEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES,
                estimator.arePointsNormalized());
        assertTrue(estimator.isReady());
        assertEquals(FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM, estimator.getMethod());
        assertEquals(AffineFundamentalMatrixEstimator.MIN_REQUIRED_POINTS, estimator.getMinRequiredPoints());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class,
                () -> new AffineFundamentalMatrixEstimator(emptyPoints, rightPoints));
        assertThrows(IllegalArgumentException.class,
                () -> new AffineFundamentalMatrixEstimator(leftPoints, emptyPoints));
    }

    @Test
    void testIsSetLMSESolutionAllowed() throws LockedException {
        final var estimator = new AffineFundamentalMatrixEstimator();

        // check default value
        assertEquals(AffineFundamentalMatrixEstimator.DEFAULT_ALLOW_LMSE_SOLUTION, estimator.isLMSESolutionAllowed());

        // set new value
        estimator.setLMSESolutionAllowed(!AffineFundamentalMatrixEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);

        // check correctness
        assertEquals(!AffineFundamentalMatrixEstimator.DEFAULT_ALLOW_LMSE_SOLUTION, estimator.isLMSESolutionAllowed());
    }

    @Test
    void testAreSetPointsNormalized() throws LockedException {
        final var estimator = new AffineFundamentalMatrixEstimator();

        // check default value
        assertEquals(AffineFundamentalMatrixEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES,
                estimator.arePointsNormalized());

        // set new value
        estimator.setPointsNormalized(!AffineFundamentalMatrixEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES);

        assertEquals(!AffineFundamentalMatrixEstimator.DEFAULT_NORMALIZE_POINT_CORRESPONDENCES,
                estimator.arePointsNormalized());
    }

    @Test
    void testGetSetPoints() throws LockedException {
        final var estimator = new AffineFundamentalMatrixEstimator();

        // check default value
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        final var leftPoints = new ArrayList<Point2D>();
        final var rightPoints = new ArrayList<Point2D>();
        for (var i = 0; i < nPoints; i++) {
            leftPoints.add(Point2D.create());
            rightPoints.add(Point2D.create());
        }

        estimator.setPoints(leftPoints, rightPoints);

        // check correctness
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(emptyPoints, rightPoints));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(leftPoints, emptyPoints));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new AffineFundamentalMatrixEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
    }

    @Test
    void testEstimateNoLMSENoNormalization() throws LockedException, NotReadyException,
            FundamentalMatrixEstimatorException, InvalidFundamentalMatrixException, NotAvailableException,
            WrongSizeException, InvalidPairOfCamerasException {

        AffineFundamentalMatrixEstimator estimator;

        var numValid = 0;
        for (var j = 0; j < TIMES; j++) {
            // randomly create two pinhole cameras
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final var skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final var horizontalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var horizontalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final var center1 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                    horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
            final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                    horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

            final var camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            // convert cameras into affine cameras
            final var cameraMatrix1 = camera1.getInternalMatrix();
            cameraMatrix1.setElementAt(2, 0, 0.0);
            cameraMatrix1.setElementAt(2, 1, 0.0);
            cameraMatrix1.setElementAt(2, 2, 0.0);
            cameraMatrix1.setElementAt(2, 3, 1.0);
            camera1.setInternalMatrix(cameraMatrix1);

            final var cameraMatrix2 = camera2.getInternalMatrix();
            cameraMatrix2.setElementAt(2, 0, 0.0);
            cameraMatrix2.setElementAt(2, 1, 0.0);
            cameraMatrix2.setElementAt(2, 2, 0.0);
            cameraMatrix2.setElementAt(2, 3, 1.0);
            camera2.setInternalMatrix(cameraMatrix2);

            // generate a random list of 3D points
            final var points3D = new ArrayList<Point3D>();
            for (var i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final var leftPoints = camera1.project(points3D);
            final var rightPoints = camera2.project(points3D);

            // estimate fundamental matrix
            estimator = new AffineFundamentalMatrixEstimator(leftPoints, rightPoints);
            estimator.setLMSESolutionAllowed(false);
            estimator.setPointsNormalized(false);
            estimator.setListener(this);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            final var fundMatrix = estimator.estimate();
            final var fundMatrix2 = new FundamentalMatrix(camera1, camera2);

            // check correctness
            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            reset();

            // compute epipoles
            fundMatrix2.computeEpipoles();
            final var epipole1a = fundMatrix2.getLeftEpipole();
            final var epipole2a = fundMatrix2.getRightEpipole();

            fundMatrix.computeEpipoles();

            final var epipole1b = fundMatrix.getLeftEpipole();
            final var epipole2b = fundMatrix.getRightEpipole();

            // check correctness of epipoles
            if (!epipole1a.equals(epipole1b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipole1a.equals(epipole1b, ABSOLUTE_ERROR));
            if (!epipole2a.equals(epipole2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipole2a.equals(epipole2b, ABSOLUTE_ERROR));

            fundMatrix.normalize();
            fundMatrix2.normalize();

            // check that both matrices are equal up to scale (i.e. sign)
            if (!fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                    .multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                            .multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR));

            // check that all points lie within their corresponding epipolar
            // lines
            for (var i = 0; i < nPoints; i++) {
                final var leftPoint = leftPoints.get(i);
                final var rightPoint = rightPoints.get(i);
                final var point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final var line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final var line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ULTRA_LARGE_ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint, ULTRA_LARGE_ABSOLUTE_ERROR));

                // obtain epipolar planes
                final var epipolarPlane1 = camera1.backProject(line1);
                final var epipolarPlane2 = camera2.backProject(line2);

                // check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, ULTRA_LARGE_ABSOLUTE_ERROR));

                // check that point3D and camera centers belong to epipolar plane
                if (!epipolarPlane1.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center1, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center1, ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center2, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center2, EXTREME_LARGE_ABSOLUTE_ERROR));

                if (!epipolarPlane2.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center1, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center1, EXTREME_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center2, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center2, ULTRA_LARGE_ABSOLUTE_ERROR));
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // Force NotReadyException
        estimator = new AffineFundamentalMatrixEstimator();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateNoLMSENormalization() throws LockedException, NotReadyException,
            FundamentalMatrixEstimatorException, InvalidFundamentalMatrixException, NotAvailableException,
            InvalidPairOfCamerasException, WrongSizeException {

        AffineFundamentalMatrixEstimator estimator;

        var numValid = 0;
        for (var j = 0; j < TIMES; j++) {
            // randomly create two pinhole cameras
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final var skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final var horizontalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var horizontalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final var center1 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                    horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
            final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                    horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

            final var camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            // convert cameras into affine cameras
            final var cameraMatrix1 = camera1.getInternalMatrix();
            cameraMatrix1.setElementAt(2, 0, 0.0);
            cameraMatrix1.setElementAt(2, 1, 0.0);
            cameraMatrix1.setElementAt(2, 2, 0.0);
            cameraMatrix1.setElementAt(2, 3, 1.0);
            camera1.setInternalMatrix(cameraMatrix1);

            final var cameraMatrix2 = camera2.getInternalMatrix();
            cameraMatrix2.setElementAt(2, 0, 0.0);
            cameraMatrix2.setElementAt(2, 1, 0.0);
            cameraMatrix2.setElementAt(2, 2, 0.0);
            cameraMatrix2.setElementAt(2, 3, 1.0);
            camera2.setInternalMatrix(cameraMatrix2);

            // generate a random list of 3D points
            final var points3D = new ArrayList<Point3D>();
            for (var i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final var leftPoints = camera1.project(points3D);
            final var rightPoints = camera2.project(points3D);

            // estimate fundamental matrix
            estimator = new AffineFundamentalMatrixEstimator(leftPoints, rightPoints);
            estimator.setLMSESolutionAllowed(false);
            estimator.setPointsNormalized(true);
            estimator.setListener(this);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            final var fundMatrix = estimator.estimate();
            final var fundMatrix2 = new FundamentalMatrix(camera1, camera2);

            // check correctness
            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            reset();

            // compute epipoles
            fundMatrix2.computeEpipoles();
            final var epipole1a = fundMatrix2.getLeftEpipole();
            final var epipole2a = fundMatrix2.getRightEpipole();

            fundMatrix.computeEpipoles();

            final var epipole1b = fundMatrix.getLeftEpipole();
            final var epipole2b = fundMatrix.getRightEpipole();

            // check correctness of epipoles
            if (!epipole1a.equals(epipole1b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipole1a.equals(epipole1b, ABSOLUTE_ERROR));
            if (!epipole2a.equals(epipole2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipole2a.equals(epipole2b, ABSOLUTE_ERROR));

            fundMatrix.normalize();
            fundMatrix2.normalize();

            // check that both matrices are equal up to scale (i.e. sign)
            if (!fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                    .multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                    .multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR));

            // check that all points lie within their corresponding epipolar
            // lines
            for (var i = 0; i < nPoints; i++) {
                final var leftPoint = leftPoints.get(i);
                final var rightPoint = rightPoints.get(i);
                final var point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final var line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final var line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, 10.0 * VERY_LARGE_ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint, VERY_LARGE_ABSOLUTE_ERROR));

                // obtain epipolar planes
                final var epipolarPlane1 = camera1.backProject(line1);
                final var epipolarPlane2 = camera2.backProject(line2);

                // check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, VERY_LARGE_ABSOLUTE_ERROR));

                // check that point3D and camera centers belong to epipolar plane
                if (!epipolarPlane1.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center1, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center1, ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center2, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center2, ULTRA_LARGE_ABSOLUTE_ERROR));

                if (!epipolarPlane2.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center1, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center1, ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center2, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center2, ULTRA_LARGE_ABSOLUTE_ERROR));
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // Force NotReadyException
        estimator = new AffineFundamentalMatrixEstimator();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateLMSENoNormalization() throws LockedException, NotReadyException,
            FundamentalMatrixEstimatorException, InvalidFundamentalMatrixException, NotAvailableException,
            InvalidPairOfCamerasException, WrongSizeException {

        AffineFundamentalMatrixEstimator estimator;

        var numValid = 0;
        for (var j = 0; j < TIMES; j++) {
            // randomly create two pinhole cameras
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final var skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final var horizontalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var horizontalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final var center1 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                    horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
            final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                    horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

            final var camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            // convert cameras into affine cameras
            final var cameraMatrix1 = camera1.getInternalMatrix();
            cameraMatrix1.setElementAt(2, 0, 0.0);
            cameraMatrix1.setElementAt(2, 1, 0.0);
            cameraMatrix1.setElementAt(2, 2, 0.0);
            cameraMatrix1.setElementAt(2, 3, 1.0);
            camera1.setInternalMatrix(cameraMatrix1);

            final var cameraMatrix2 = camera2.getInternalMatrix();
            cameraMatrix2.setElementAt(2, 0, 0.0);
            cameraMatrix2.setElementAt(2, 1, 0.0);
            cameraMatrix2.setElementAt(2, 2, 0.0);
            cameraMatrix2.setElementAt(2, 3, 1.0);
            camera2.setInternalMatrix(cameraMatrix2);

            // generate a random list of 3D points
            final var points3D = new ArrayList<Point3D>();
            for (var i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final var leftPoints = camera1.project(points3D);
            final var rightPoints = camera2.project(points3D);

            // estimate fundamental matrix
            estimator = new AffineFundamentalMatrixEstimator(leftPoints, rightPoints);
            estimator.setLMSESolutionAllowed(true);
            estimator.setPointsNormalized(false);
            estimator.setListener(this);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            final var fundMatrix = estimator.estimate();
            final var fundMatrix2 = new FundamentalMatrix(camera1, camera2);

            // check correctness
            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            reset();

            // compute epipoles
            fundMatrix2.computeEpipoles();
            final var epipole1a = fundMatrix2.getLeftEpipole();
            final var epipole2a = fundMatrix2.getRightEpipole();

            fundMatrix.computeEpipoles();

            final var epipole1b = fundMatrix.getLeftEpipole();
            final var epipole2b = fundMatrix.getRightEpipole();

            // check correctness of epipoles
            if (!epipole1a.equals(epipole1b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipole1a.equals(epipole1b, ABSOLUTE_ERROR));
            if (!epipole2a.equals(epipole2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipole2a.equals(epipole2b, ABSOLUTE_ERROR));

            fundMatrix.normalize();
            fundMatrix2.normalize();

            // check that both matrices are equal up to scale (i.e. sign)
            if (!fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                            .multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                    .multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR));

            // check that all points lie within their corresponding epipolar
            // lines
            for (var i = 0; i < nPoints; i++) {
                final var leftPoint = leftPoints.get(i);
                final var rightPoint = rightPoints.get(i);
                final var point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final var line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final var line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ULTRA_LARGE_ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint, ULTRA_LARGE_ABSOLUTE_ERROR));

                // obtain epipolar planes
                final var epipolarPlane1 = camera1.backProject(line1);
                final var epipolarPlane2 = camera2.backProject(line2);

                // check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, ULTRA_LARGE_ABSOLUTE_ERROR));

                // check that point3D and camera centers belong to epipolar plane
                if (!epipolarPlane1.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center1, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center1, ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center2, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center2, EXTREME_LARGE_ABSOLUTE_ERROR));

                if (!epipolarPlane2.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center1, EXTREME_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center1, EXTREME_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center2, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center2, ULTRA_LARGE_ABSOLUTE_ERROR));
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // Force NotReadyException
        estimator = new AffineFundamentalMatrixEstimator();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateLMSENormalization() throws LockedException, NotReadyException, FundamentalMatrixEstimatorException,
            InvalidFundamentalMatrixException, NotAvailableException, InvalidPairOfCamerasException,
            WrongSizeException {

        AffineFundamentalMatrixEstimator estimator;

        var numValid = 0;
        for (var j = 0; j < TIMES; j++) {
            // randomly create two pinhole cameras
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final var skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final var horizontalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var horizontalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            final var center1 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                    horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
            final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                    horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

            final var camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            // convert cameras into affine cameras
            final var cameraMatrix1 = camera1.getInternalMatrix();
            cameraMatrix1.setElementAt(2, 0, 0.0);
            cameraMatrix1.setElementAt(2, 1, 0.0);
            cameraMatrix1.setElementAt(2, 2, 0.0);
            cameraMatrix1.setElementAt(2, 3, 1.0);
            camera1.setInternalMatrix(cameraMatrix1);

            final var cameraMatrix2 = camera2.getInternalMatrix();
            cameraMatrix2.setElementAt(2, 0, 0.0);
            cameraMatrix2.setElementAt(2, 1, 0.0);
            cameraMatrix2.setElementAt(2, 2, 0.0);
            cameraMatrix2.setElementAt(2, 3, 1.0);
            camera2.setInternalMatrix(cameraMatrix2);

            // generate a random list of 3D points
            final var points3D = new ArrayList<Point3D>();
            for (var i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final var leftPoints = camera1.project(points3D);
            final var rightPoints = camera2.project(points3D);

            // estimate fundamental matrix
            estimator = new AffineFundamentalMatrixEstimator(leftPoints, rightPoints);
            estimator.setLMSESolutionAllowed(true);
            estimator.setPointsNormalized(true);
            estimator.setListener(this);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            // estimate
            final var fundMatrix = estimator.estimate();
            final var fundMatrix2 = new FundamentalMatrix(camera1, camera2);

            // check correctness
            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            reset();

            // compute epipoles
            fundMatrix2.computeEpipoles();
            final var epipole1a = fundMatrix2.getLeftEpipole();
            final var epipole2a = fundMatrix2.getRightEpipole();

            fundMatrix.computeEpipoles();

            final var epipole1b = fundMatrix.getLeftEpipole();
            final var epipole2b = fundMatrix.getRightEpipole();

            // check correctness of epipoles
            if (!epipole1a.equals(epipole1b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipole1a.equals(epipole1b, ABSOLUTE_ERROR));
            if (!epipole2a.equals(epipole2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipole2a.equals(epipole2b, ABSOLUTE_ERROR));

            fundMatrix.normalize();
            fundMatrix2.normalize();

            // check that both matrices are equal up to scale (i.e. sign)
            if (!fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                            .multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                    .multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR));

            // check that all points lie within their corresponding epipolar
            // lines
            for (var i = 0; i < nPoints; i++) {
                final var leftPoint = leftPoints.get(i);
                final var rightPoint = rightPoints.get(i);
                final var point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final var line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final var line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, VERY_LARGE_ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint, VERY_LARGE_ABSOLUTE_ERROR));

                // obtain epipolar planes
                final var epipolarPlane1 = camera1.backProject(line1);
                final var epipolarPlane2 = camera2.backProject(line2);

                // check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, LARGE_ABSOLUTE_ERROR));

                // check that point3D and camera centers belong to epipolar plane
                if (!epipolarPlane1.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center1, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center1, ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center2, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center2, ULTRA_LARGE_ABSOLUTE_ERROR));

                if (!epipolarPlane2.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(point3D, ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center1, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center1, ULTRA_LARGE_ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center2, ULTRA_LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center2, ULTRA_LARGE_ABSOLUTE_ERROR));
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // Force NotReadyException
        estimator = new AffineFundamentalMatrixEstimator();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Override
    public void onEstimateStart(final FundamentalMatrixEstimator estimator) {
        estimateStart++;
        testLocked((AffineFundamentalMatrixEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final FundamentalMatrixEstimator estimator, final FundamentalMatrix fundamentalMatrix) {
        estimateEnd++;
        testLocked((AffineFundamentalMatrixEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    private void testLocked(final AffineFundamentalMatrixEstimator estimator) {
        assertTrue(estimator.isLocked());
        assertThrows(LockedException.class, () -> estimator.setLMSESolutionAllowed(true));
        assertThrows(LockedException.class, () -> estimator.setPointsNormalized(true));
        assertThrows(LockedException.class, () -> estimator.setPoints(null, null));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, estimator::estimate);
    }
}
