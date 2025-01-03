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
package com.irurueta.ar.epipolar.refiners;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.ProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.refiners.Refiner;
import com.irurueta.geometry.refiners.RefinerListener;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class HomogeneousRightEpipoleRefinerTest implements RefinerListener<Point2D> {

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

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double ERROR_STD = 1e-3;

    private static final int MAX_TRIES = 5000;

    private static final int TIMES = 500;

    private int refineStart;
    private int refineEnd;

    @Test
    void testConstructor() throws GeometryException, AlgebraException, RobustEstimatorException {
        final var refinementStandardDeviation = 1.0;
        final var camera1 = new PinholeCamera();
        final var camera2 = new PinholeCamera();
        final var fundamentalMatrix = new FundamentalMatrix();
        final var samples1 = new ArrayList<Point2D>();
        final var samples2 = new ArrayList<Point2D>();
        final var homography = generateHomography(camera1, camera2, fundamentalMatrix, samples1, samples2);

        final var numPoints = samples1.size();
        final var inliers = new BitSet(numPoints);
        final var res = new double[numPoints];
        final var inliersData = new InliersData() {
            @Override
            public BitSet getInliers() {
                return inliers;
            }

            @Override
            public double[] getResiduals() {
                return res;
            }

            @Override
            public int getNumInliers() {
                return numPoints;
            }
        };
        final var numInliers = inliersData.getNumInliers();

        fundamentalMatrix.normalize();
        fundamentalMatrix.computeEpipoles();
        final var rightEpipole = fundamentalMatrix.getRightEpipole();

        var refiner = new HomogeneousRightEpipoleRefiner();

        // check default values
        assertEquals(0.0, refiner.getRefinementStandardDeviation(), 0.0);
        assertNull(refiner.getHomography());
        assertNull(refiner.getSamples1());
        assertNull(refiner.getSamples2());
        assertFalse(refiner.isReady());
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(0, refiner.getNumInliers());
        assertEquals(0, refiner.getTotalSamples());
        assertNull(refiner.getInitialEstimation());
        assertFalse(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        // test non-empty constructor
        refiner = new HomogeneousRightEpipoleRefiner(rightEpipole, true, inliers, res, numInliers,
                samples1, samples2, refinementStandardDeviation, homography);

        // check default values
        assertEquals(refinementStandardDeviation, refiner.getRefinementStandardDeviation(), 0.0);
        assertSame(homography, refiner.getHomography());
        assertSame(samples1, refiner.getSamples1());
        assertSame(samples2, refiner.getSamples2());
        assertTrue(refiner.isReady());
        assertSame(inliers, refiner.getInliers());
        assertSame(res, refiner.getResiduals());
        assertEquals(numPoints, refiner.getNumInliers());
        assertEquals(numPoints, refiner.getTotalSamples());
        assertSame(rightEpipole, refiner.getInitialEstimation());
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        // test non-empty constructor with InliersData
        refiner = new HomogeneousRightEpipoleRefiner(rightEpipole, true, inliersData, samples1, samples2,
                refinementStandardDeviation, homography);

        // check default values
        assertEquals(refinementStandardDeviation, refiner.getRefinementStandardDeviation(), 0.0);
        assertSame(homography, refiner.getHomography());
        assertSame(samples1, refiner.getSamples1());
        assertSame(samples2, refiner.getSamples2());
        assertTrue(refiner.isReady());
        assertSame(inliers, refiner.getInliers());
        assertSame(res, refiner.getResiduals());
        assertEquals(numPoints, refiner.getNumInliers());
        assertEquals(numPoints, refiner.getTotalSamples());
        assertSame(rightEpipole, refiner.getInitialEstimation());
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());
    }

    @Test
    void testGetSetListener() {
        final var refiner = new HomogeneousRightEpipoleRefiner();

        // check default value
        assertNull(refiner.getListener());

        // set new value
        refiner.setListener(this);

        // check correctness
        assertSame(this, refiner.getListener());
    }

    @Test
    void testGetSetRefinementStandardDeviation() throws LockedException {
        final var refiner = new HomogeneousRightEpipoleRefiner();

        // check default value
        assertEquals(0.0, refiner.getRefinementStandardDeviation(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var refinementStandardDeviation = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        refiner.setRefinementStandardDeviation(refinementStandardDeviation);

        // check correctness
        assertEquals(refinementStandardDeviation, refiner.getRefinementStandardDeviation(), 0.0);
    }

    @Test
    void testGetSetHomography() throws LockedException {
        final var refiner = new HomogeneousRightEpipoleRefiner();

        // check default value
        assertNull(refiner.getHomography());

        // set new value
        final var homography = new ProjectiveTransformation2D();
        refiner.setHomography(homography);

        // check correctness
        assertSame(homography, refiner.getHomography());
    }

    @Test
    void testGetSetSamples1() throws LockedException {
        final var refiner = new HomogeneousRightEpipoleRefiner();

        // check default value
        assertNull(refiner.getSamples1());

        // set new value
        final var samples1 = new ArrayList<Point2D>();
        refiner.setSamples1(samples1);

        // check correctness
        assertSame(samples1, refiner.getSamples1());
    }

    @Test
    void testGetSetSamples2() throws LockedException {
        final var refiner = new HomogeneousRightEpipoleRefiner();

        // check default value
        assertNull(refiner.getSamples2());

        // set new value
        final var samples2 = new ArrayList<Point2D>();
        refiner.setSamples2(samples2);

        // check correctness
        assertSame(samples2, refiner.getSamples2());
    }

    @Test
    void testGetSetInliers() throws LockedException {
        final var refiner = new HomogeneousRightEpipoleRefiner();

        // check default value
        assertNull(refiner.getInliers());

        // set new value
        final var inliers = new BitSet(100);
        refiner.setInliers(inliers);

        // check correctness
        assertSame(inliers, refiner.getInliers());
    }

    @Test
    void testGetSetResiduals() throws LockedException {
        final var refiner = new HomogeneousRightEpipoleRefiner();

        // check default value
        assertNull(refiner.getResiduals());

        // set new value
        final var residuals = new double[100];
        refiner.setResiduals(residuals);

        // check correctness
        assertSame(residuals, refiner.getResiduals());
    }

    @Test
    void testGetSetNumInliers() throws LockedException {
        final var refiner = new HomogeneousRightEpipoleRefiner();

        // check default value
        assertEquals(0, refiner.getNumInliers());

        // set new value
        refiner.setNumInliers(10);

        // check correctness
        assertEquals(10, refiner.getNumInliers());
    }

    @Test
    void testSetInliersData() throws LockedException {
        final var numPoints = MIN_NUM_POINTS;
        final var inliers = new BitSet(numPoints);
        final var res = new double[numPoints];
        final var inliersData = new InliersData() {
            @Override
            public BitSet getInliers() {
                return inliers;
            }

            @Override
            public double[] getResiduals() {
                return res;
            }

            @Override
            public int getNumInliers() {
                return numPoints;
            }
        };
        final var numInliers = inliersData.getNumInliers();

        final var refiner = new HomogeneousRightEpipoleRefiner();

        // check default values
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(0, refiner.getNumInliers());

        // set new value
        refiner.setInliersData(inliersData);

        // check correctness
        assertSame(inliers, refiner.getInliers());
        assertSame(res, refiner.getResiduals());
        assertEquals(numInliers, refiner.getNumInliers());
    }

    @Test
    void testGetSetInitialEstimation() throws LockedException {
        final var refiner = new HomogeneousRightEpipoleRefiner();

        // check default value
        assertNull(refiner.getInitialEstimation());

        // set new value
        final var initialEstimation = new HomogeneousPoint2D();
        refiner.setInitialEstimation(initialEstimation);

        // check correctness
        assertSame(initialEstimation, refiner.getInitialEstimation());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var refiner = new HomogeneousRightEpipoleRefiner();

        // check default value
        assertFalse(refiner.isCovarianceKept());

        // set new value
        refiner.setCovarianceKept(true);

        // check correctness
        assertTrue(refiner.isCovarianceKept());
    }

    @Test
    void testRefine() throws GeometryException, AlgebraException, RobustEstimatorException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var camera1 = new PinholeCamera();
            final var camera2 = new PinholeCamera();
            final var groundTruthFundamentalMatrix = new FundamentalMatrix();
            final var samples1 = new ArrayList<Point2D>();
            final var samples2 = new ArrayList<Point2D>();
            final var homography = generateHomography(camera1, camera2, groundTruthFundamentalMatrix, samples1,
                    samples2);

            final var numPoints = samples1.size();
            final var inliers = new BitSet(numPoints);
            inliers.set(0, numPoints);
            final var residuals = new double[numPoints];
            Arrays.fill(residuals, 1.0);

            groundTruthFundamentalMatrix.normalize();
            groundTruthFundamentalMatrix.computeEpipoles();
            final var groundTruthRightEpipole = groundTruthFundamentalMatrix.getRightEpipole();

            // add noise to epipole
            final var noiseRandomizer = new GaussianRandomizer(0.0, ERROR_STD);
            final var noiseX = noiseRandomizer.nextDouble();
            final var noiseY = noiseRandomizer.nextDouble();
            final var noisyRightEpipole = new HomogeneousPoint2D(groundTruthRightEpipole.getInhomX() + noiseX,
                    groundTruthRightEpipole.getInhomY() + noiseY, 1.0);
            noisyRightEpipole.normalize();

            final var refiner = new HomogeneousRightEpipoleRefiner(noisyRightEpipole, true, inliers,
                    residuals, numPoints, samples1, samples2, ERROR_STD, homography);
            refiner.setListener(this);

            reset();
            assertEquals(0, refineStart);
            assertEquals(0, refineEnd);

            final var refinedEpipole1 = new HomogeneousPoint2D();
            refiner.refine(refinedEpipole1);
            final var refinedEpipole2 = refiner.refine();

            assertEquals(2, refineStart);
            assertEquals(2, refineEnd);

            refinedEpipole1.normalize();
            refinedEpipole2.normalize();

            if (!refinedEpipole1.equals(groundTruthRightEpipole, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(refinedEpipole1.equals(groundTruthRightEpipole, ABSOLUTE_ERROR));
            if (!refinedEpipole2.equals(groundTruthRightEpipole, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(refinedEpipole2.equals(groundTruthRightEpipole, ABSOLUTE_ERROR));

            final var refinedFundamentalMatrix1 = new FundamentalMatrix();
            HomogeneousRightEpipoleRefiner.computeFundamentalMatrix(homography, refinedEpipole1,
                    refinedFundamentalMatrix1);
            refinedFundamentalMatrix1.normalize();

            final var refinedFundamentalMatrix2 = new FundamentalMatrix();
            HomogeneousRightEpipoleRefiner.computeFundamentalMatrix(homography, new HomogeneousPoint2D(refinedEpipole2),
                    refinedFundamentalMatrix2);
            refinedFundamentalMatrix2.normalize();

            // check correctness
            if (!groundTruthFundamentalMatrix.getInternalMatrix().equals(
                    refinedFundamentalMatrix1.getInternalMatrix(), ABSOLUTE_ERROR)
                    && !groundTruthFundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1.0).equals(
                            refinedFundamentalMatrix1.getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(groundTruthFundamentalMatrix.getInternalMatrix().equals(
                    refinedFundamentalMatrix1.getInternalMatrix(), ABSOLUTE_ERROR)
                    || groundTruthFundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1.0)
                    .equals(refinedFundamentalMatrix1.getInternalMatrix(), ABSOLUTE_ERROR));

            if (!groundTruthFundamentalMatrix.getInternalMatrix().equals(
                    refinedFundamentalMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)
                    && !groundTruthFundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1.0)
                    .equals(refinedFundamentalMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(groundTruthFundamentalMatrix.getInternalMatrix().equals(
                    refinedFundamentalMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)
                    || groundTruthFundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1.0)
                    .equals(refinedFundamentalMatrix2.getInternalMatrix(), ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private static Transformation2D generateHomography(
            final PinholeCamera camera1, final PinholeCamera camera2, final FundamentalMatrix fundamentalMatrix,
            final List<Point2D> projectedPoints1, final List<Point2D> projectedPoints2) throws GeometryException,
            AlgebraException, RobustEstimatorException {

        final var randomizer = new UniformRandomizer();
        final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var aspectRatio = 1.0;
        final var skewness = 0.0;
        final var principalPoint = 0.0;

        final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPoint,
                principalPoint, skewness);
        intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

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

        camera1.setIntrinsicAndExtrinsicParameters(intrinsic, rotation1, center1);
        camera2.setIntrinsicAndExtrinsicParameters(intrinsic, rotation2, center2);

        fundamentalMatrix.setFromPairOfCameras(camera1, camera2);

        // create 3D points laying in front of both cameras an in a plane
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
        projectedPoints1.clear();
        projectedPoints2.clear();
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
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(planeA * homX + planeC * homZ + planeD * homW) / planeB;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(planeB * homY + planeC * homZ + planeD * homW) / planeA;
                }

                point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(point3D));

                front1 = camera1.isPointInFrontOfCamera(point3D);
                front2 = camera2.isPointInFrontOfCamera(point3D);
                if (numTry > MAX_TRIES) {
                    return null;
                }
                numTry++;
            } while (!front1 || !front2);

            // check that 3D point is in front of both cameras
            assertTrue(front1);
            assertTrue(front2);

            // project 3D point into both cameras
            projectedPoints1.add(camera1.project(point3D));
            projectedPoints2.add(camera2.project(point3D));
        }

        // estimate homography
        final var homographyEstimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(projectedPoints1,
                projectedPoints2, RobustEstimatorMethod.LMEDS);

        return homographyEstimator.estimate();
    }

    @Override
    public void onRefineStart(final Refiner<Point2D> refiner, final Point2D initialEstimation) {
        refineStart++;
        checkLocked((HomogeneousRightEpipoleRefiner) refiner);
    }

    @Override
    public void onRefineEnd(final Refiner<Point2D> refiner, final Point2D initialEstimation, final Point2D result,
                            final boolean errorDecreased) {
        refineEnd++;
        checkLocked((HomogeneousRightEpipoleRefiner) refiner);
    }

    private void reset() {
        refineStart = refineEnd = 0;
    }

    private static void checkLocked(final HomogeneousRightEpipoleRefiner refiner) {
        assertTrue(refiner.isLocked());
        assertThrows(LockedException.class, () -> refiner.setRefinementStandardDeviation(0.0));
        assertThrows(LockedException.class, () -> refiner.setHomography(null));
        assertThrows(LockedException.class, refiner::refine);
        assertThrows(LockedException.class, () -> refiner.refine(new HomogeneousPoint2D()));
    }
}
