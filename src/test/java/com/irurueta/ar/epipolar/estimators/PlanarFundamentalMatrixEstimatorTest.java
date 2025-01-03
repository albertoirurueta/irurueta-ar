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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidPairOfCamerasException;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.ProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class PlanarFundamentalMatrixEstimatorTest implements PlanarFundamentalMatrixEstimatorListener {

    private static final double MIN_FOCAL_LENGTH = 750.0;
    private static final double MAX_FOCAL_LENGTH = 1500.0;

    private static final double MIN_ANGLE_DEGREES = -30.0;
    private static final double MAX_ANGLE_DEGREES = -15.0;

    private static final double MIN_CAMERA_SEPARATION = 500.0;
    private static final double MAX_CAMERA_SEPARATION = 1000.0;

    private static final int MIN_NUM_POINTS = 25;
    private static final int MAX_NUM_POINTS = 50;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_RANDOM_VALUE_PLANAR = -1500.0;
    private static final double MAX_RANDOM_VALUE_PLANAR = 1500.0;

    private static final int TIMES = 500;
    private static final int MAX_TRIES = 5000;

    private static final double HOMOGRAPHY_ESTIMATOR_THRESHOLD = 1e-8;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;

    private int mEstimateStart;
    private int mEstimateEnd;

    @Test
    void testConstructor() {
        // empty constructor
        var estimator = new PlanarFundamentalMatrixEstimator();

        // check default values
        assertNull(estimator.getHomography());
        assertNull(estimator.getLeftIntrinsics());
        assertNull(estimator.getRightIntrinsics());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());

        // constructor with input data
        final ProjectiveTransformation2D homography = new ProjectiveTransformation2D();
        final PinholeCameraIntrinsicParameters leftIntrinsics = new PinholeCameraIntrinsicParameters();
        final PinholeCameraIntrinsicParameters rightIntrinsics = new PinholeCameraIntrinsicParameters();
        estimator = new PlanarFundamentalMatrixEstimator(homography, leftIntrinsics, rightIntrinsics);

        // check correctness
        assertSame(homography, estimator.getHomography());
        assertSame(leftIntrinsics, estimator.getLeftIntrinsics());
        assertSame(rightIntrinsics, estimator.getRightIntrinsics());
        assertNull(estimator.getListener());

        // constructor with input data and listener
        estimator = new PlanarFundamentalMatrixEstimator(homography, leftIntrinsics, rightIntrinsics, this);

        // check correctness
        assertSame(homography, estimator.getHomography());
        assertSame(leftIntrinsics, estimator.getLeftIntrinsics());
        assertSame(rightIntrinsics, estimator.getRightIntrinsics());
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetHomography() throws LockedException {
        final var estimator = new PlanarFundamentalMatrixEstimator();

        // check default value
        assertNull(estimator.getHomography());

        // set new value
        final var homography = new ProjectiveTransformation2D();
        estimator.setHomography(homography);

        // check correctness
        assertSame(homography, estimator.getHomography());
    }

    @Test
    void testGetSetLeftIntrinsics() throws LockedException {
        final var estimator = new PlanarFundamentalMatrixEstimator();

        // check default value
        assertNull(estimator.getLeftIntrinsics());

        // set new value
        final var leftIntrinsics = new PinholeCameraIntrinsicParameters();
        estimator.setLeftIntrinsics(leftIntrinsics);

        // check correctness
        assertSame(leftIntrinsics, estimator.getLeftIntrinsics());
    }

    @Test
    void testGetSetRightIntrinsics() throws LockedException {
        final var estimator = new PlanarFundamentalMatrixEstimator();

        // check default value
        assertNull(estimator.getRightIntrinsics());

        // set new value
        final var rightIntrinsics = new PinholeCameraIntrinsicParameters();
        estimator.setRightIntrinsics(rightIntrinsics);

        // check correctness
        assertSame(rightIntrinsics, estimator.getRightIntrinsics());
    }

    @Test
    void testGetSetListener() {
        final var estimator = new PlanarFundamentalMatrixEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
    }

    @Test
    void testEstimate() throws InvalidPairOfCamerasException, AlgebraException, CameraException, LockedException,
            NotReadyException, FundamentalMatrixEstimatorException, NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var focalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var focalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var principalPointX = 0.0;
            final var principalPointY = 0.0;

            final var intrinsic1 = new PinholeCameraIntrinsicParameters(focalLength1, focalLength1, principalPointX,
                    principalPointY, skewness);
            final var intrinsic2 = new PinholeCameraIntrinsicParameters(focalLength2, focalLength2, principalPointX,
                    principalPointY, skewness);

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras and laying in
            // a plane

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final var decomposer = new SingularValueDecomposer(planesIntersectionMatrix);
            decomposer.decompose();
            final var v = decomposer.getV();
            final var centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            final var principalAxis1 = camera1.getPrincipalAxisArray();
            final var principalAxis2 = camera2.getPrincipalAxisArray();
            final var avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2), 0.5);

            final var plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();

            final var planeA = plane.getA();
            final var planeB = plane.getB();
            final var planeC = plane.getC();
            final var planeD = plane.getD();

            final var numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            for (var i = 0; i < numPoints; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final var homW = 1.0;
                    final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) / planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) / planeA;
                    }

                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                    assertTrue(plane.isLocus(point3D));

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front1 || !front2);

                // at this line 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new HomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoint1.normalize();

                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new HomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoint2.normalize();

                projectedPoints2.add(projectedPoint2);
            }

            // estimate homography
            final var homographyEstimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(projectedPoints1,
                    projectedPoints2, RobustEstimatorMethod.RANSAC);
            ((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator).setThreshold(
                    HOMOGRAPHY_ESTIMATOR_THRESHOLD);
            ((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator)
                    .setComputeAndKeepInliersEnabled(true);
            ((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator)
                    .setComputeAndKeepResidualsEnabled(true);
            homographyEstimator.setResultRefined(false);
            ProjectiveTransformation2D homography;
            try {
                homography = homographyEstimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            homography.normalize();

            // check errors on estimated homography
            var failed = false;
            for (var i = 0; i < numPoints; i++) {
                projectedPoint1 = new InhomogeneousPoint2D(projectedPoints1.get(i));
                projectedPoint2 = new InhomogeneousPoint2D(projectedPoints2.get(i));

                final var projectedPoint2b = new InhomogeneousPoint2D();
                homography.transform(projectedPoint1, projectedPoint2b);

                if (!projectedPoint2.equals(projectedPoint2b, LARGE_ABSOLUTE_ERROR)) {
                    failed = true;
                    break;
                }
                assertTrue(projectedPoint2.equals(projectedPoint2b, LARGE_ABSOLUTE_ERROR));
            }

            if (failed) {
                continue;
            }

            final var estimator = new PlanarFundamentalMatrixEstimator(homography, intrinsic1, intrinsic2,
                    this);

            reset();
            assertEquals(0, mEstimateStart);
            assertEquals(0, mEstimateEnd);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            final var result1 = estimator.estimate();
            final var result2 = new ArrayList<FundamentalMatrix>();
            estimator.estimate(result2);

            // check correctness
            assertEquals(2, mEstimateStart);
            assertEquals(2, mEstimateEnd);

            assertEquals(result1.size(), result2.size());
            assertTrue(result1.size() == 2 || result1.size() == 4);

            var atLeastOneValidFundamentalMatrix = false;
            for (var i = 0; i < result1.size(); i++) {

                assertTrue(areEqualUpToScale(result1.get(i), result2.get(i)));

                if (areEqualUpToScale(fundamentalMatrix, result1.get(i))) {
                    atLeastOneValidFundamentalMatrix = true;
                }
            }

            if (!atLeastOneValidFundamentalMatrix) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private static boolean areEqualUpToScale(
            final FundamentalMatrix fundamentalMatrix1, final FundamentalMatrix fundamentalMatrix2)
            throws NotAvailableException, NotReadyException {

        // normalize to increase accuracy
        fundamentalMatrix1.normalize();
        fundamentalMatrix2.normalize();

        final var f1 = fundamentalMatrix1.getInternalMatrix();
        final var f2a = fundamentalMatrix2.getInternalMatrix();
        final var f2b = f2a.multiplyByScalarAndReturnNew(-1.0);

        return f1.equals(f2a, PlanarFundamentalMatrixEstimatorTest.ABSOLUTE_ERROR) ||
                f1.equals(f2b, PlanarFundamentalMatrixEstimatorTest.ABSOLUTE_ERROR);
    }

    @Override
    public void onEstimateStart(final PlanarFundamentalMatrixEstimator estimator) {
        mEstimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(final PlanarFundamentalMatrixEstimator estimator,
                              final List<FundamentalMatrix> fundamentalMatrices) {
        mEstimateEnd++;
        checkLocked(estimator);
    }

    private void reset() {
        mEstimateStart = mEstimateEnd = 0;
    }

    private static void checkLocked(final PlanarFundamentalMatrixEstimator estimator) {
        assertTrue(estimator.isLocked());
        assertThrows(LockedException.class, () -> estimator.setHomography(null));
        assertThrows(LockedException.class, () -> estimator.setLeftIntrinsics(null));
        assertThrows(LockedException.class, () -> estimator.setRightIntrinsics(null));
        assertThrows(LockedException.class, estimator::estimate);
        assertThrows(LockedException.class, () -> estimator.estimate(null));
    }
}
