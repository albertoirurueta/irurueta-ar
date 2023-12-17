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
package com.irurueta.ar.calibration.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.refiners.HomogeneousRightEpipoleRefiner;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.ProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class SingleHomographyPinholeCameraEstimatorTest implements
        SingleHomographyPinholeCameraEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -1500.0;
    private static final double MAX_RANDOM_VALUE = 1500.0;

    private static final double MIN_FOCAL_LENGTH = 750.0;
    private static final double MAX_FOCAL_LENGTH = 1500.0;

    private static final double MIN_ANGLE_DEGREES = -30.0;
    private static final double MAX_ANGLE_DEGREES = -15.0;

    private static final double MIN_CAMERA_SEPARATION = 500.0;
    private static final double MAX_CAMERA_SEPARATION = 1000.0;

    private static final int MIN_NUM_POINTS = 25;
    private static final int MAX_NUM_POINTS = 50;

    private static final int TIMES = 50;
    private static final int MAX_TRIES = 5000;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;

    private int estimateStart;
    private int estimateEnd;

    @Test
    public void testConstructor() {
        // test empty constructor
        SingleHomographyPinholeCameraEstimator estimator = new SingleHomographyPinholeCameraEstimator();

        // check default value
        assertEquals(SingleHomographyPinholeCameraEstimator.DEFAULT_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertNull(estimator.getHomography());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());

        // test constructor with listener
        estimator = new SingleHomographyPinholeCameraEstimator(this);

        // check default value
        assertEquals(SingleHomographyPinholeCameraEstimator.DEFAULT_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertNull(estimator.getHomography());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());

        // test constructor with homography
        final ProjectiveTransformation2D homography = new ProjectiveTransformation2D();
        estimator = new SingleHomographyPinholeCameraEstimator(homography);

        // check default value
        assertEquals(SingleHomographyPinholeCameraEstimator.DEFAULT_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertSame(homography, estimator.getHomography());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());

        // test constructor with homography and listener
        estimator = new SingleHomographyPinholeCameraEstimator(homography, this);

        // check default value
        assertEquals(SingleHomographyPinholeCameraEstimator.DEFAULT_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertSame(homography, estimator.getHomography());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testGetSetFocalDistanceAspectRatio() throws LockedException {
        final SingleHomographyPinholeCameraEstimator estimator =
                new SingleHomographyPinholeCameraEstimator();

        // check default value
        assertEquals(SingleHomographyPinholeCameraEstimator.DEFAULT_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);

        // set new value
        estimator.setFocalDistanceAspectRatio(-SingleHomographyPinholeCameraEstimator.DEFAULT_ASPECT_RATIO);

        // check correctness
        assertEquals(-SingleHomographyPinholeCameraEstimator.DEFAULT_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
    }

    @Test
    public void testGetSetHomographyAndIsReady() throws LockedException {
        final SingleHomographyPinholeCameraEstimator estimator =
                new SingleHomographyPinholeCameraEstimator();

        // check default value
        assertNull(estimator.getHomography());
        assertFalse(estimator.isReady());

        // set new value
        final ProjectiveTransformation2D homography = new ProjectiveTransformation2D();
        estimator.setHomography(homography);

        // check correctness
        assertSame(estimator.getHomography(), homography);
        assertTrue(estimator.isReady());
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final SingleHomographyPinholeCameraEstimator estimator =
                new SingleHomographyPinholeCameraEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testEstimate() throws AlgebraException, GeometryException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double aspectRatio = 1.0;
            final double skewness = 0.0;
            final double principalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras and in a plane
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            final double[] principalAxis1 = camera1.getPrincipalAxisArray();
            final double[] principalAxis2 = camera2.getPrincipalAxisArray();
            final double[] avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2), 0.5);

            final Plane plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();

            final double planeA = plane.getA();
            final double planeB = plane.getB();
            final double planeC = plane.getC();
            final double planeD = plane.getD();

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1;
            boolean front2;
            for (int i = 0; i < numPoints; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final double homW = 1.0;
                    final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) /
                                planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) /
                                planeA;
                    }

                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                    assertTrue(plane.isLocus(point3D));

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                        break;
                    }
                    numTry++;
                } while (!front1 || !front2);

                // check that 3D point is in front of both cameras
                assertTrue(front1);
                assertTrue(front2);

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            // estimate homography
            final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                    ProjectiveTransformation2DRobustEstimator.createFromPoints(
                            projectedPoints1, projectedPoints2, RobustEstimatorMethod.LMEDS);

            final Transformation2D homography;
            try {
                homography = homographyEstimator.estimate();
            } catch (final RobustEstimatorException e) {
                continue;
            }

            final SingleHomographyPinholeCameraEstimator estimator =
                    new SingleHomographyPinholeCameraEstimator(homography, this);

            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            final PinholeCamera camera;
            try {
                camera = estimator.estimate();
            } catch (final SingleHomographyPinholeCameraEstimatorException e) {
                continue;
            }

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);

            camera.decompose();

            final PinholeCameraIntrinsicParameters estimatedIntrinsic = camera.getIntrinsicParameters();
            final Rotation3D estimatedRotation = camera.getCameraRotation();
            final Point3D estimatedCenter = camera.getCameraCenter();

            final PinholeCamera estimatedCamera1 = new PinholeCamera(
                    estimatedIntrinsic, new MatrixRotation3D(), new InhomogeneousPoint3D());
            final PinholeCamera estimatedCamera2 = new PinholeCamera(
                    estimatedIntrinsic, estimatedRotation, estimatedCenter);

            final FundamentalMatrix estimatedFundamentalMatrix =
                    new FundamentalMatrix(estimatedCamera1, estimatedCamera2);

            fundamentalMatrix.normalize();
            estimatedFundamentalMatrix.normalize();

            final BitSet inliers = new BitSet();
            inliers.set(0, numPoints);
            final double[] residuals = new double[numPoints];
            Arrays.fill(residuals, 1.0);

            estimatedFundamentalMatrix.computeEpipoles();
            final Point2D estimatedEpipoleRight = estimatedFundamentalMatrix.getRightEpipole();
            final HomogeneousRightEpipoleRefiner refiner = new HomogeneousRightEpipoleRefiner(
                    estimatedEpipoleRight, true, inliers, residuals, numPoints,
                    projectedPoints1, projectedPoints2, 1.0, homography);

            final HomogeneousPoint2D refinedEpipole = new HomogeneousPoint2D();
            refiner.refine(refinedEpipole);

            final FundamentalMatrix refinedFundamentalMatrix = new FundamentalMatrix();
            HomogeneousRightEpipoleRefiner.computeFundamentalMatrix(homography,
                    refinedEpipole, refinedFundamentalMatrix);

            refinedFundamentalMatrix.normalize();

            if (!fundamentalMatrix.getInternalMatrix().equals(
                    refinedFundamentalMatrix.getInternalMatrix(),
                    LARGE_ABSOLUTE_ERROR) && !fundamentalMatrix.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1.0).equals(
                    refinedFundamentalMatrix.getInternalMatrix(),
                    LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            numValid++;
            break;
        }
        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(
            final SingleHomographyPinholeCameraEstimator estimator) {
        estimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(
            final SingleHomographyPinholeCameraEstimator estimator) {
        estimateEnd++;
        checkLocked(estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    private void checkLocked(final SingleHomographyPinholeCameraEstimator estimator) {
        try {
            estimator.setFocalDistanceAspectRatio(1.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setHomography(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.estimate(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        assertTrue(estimator.isLocked());
    }
}
