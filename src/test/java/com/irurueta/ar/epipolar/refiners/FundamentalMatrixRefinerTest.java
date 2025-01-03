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

import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.estimators.RANSACFundamentalMatrixRobustEstimator;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.refiners.Refiner;
import com.irurueta.geometry.refiners.RefinerException;
import com.irurueta.geometry.refiners.RefinerListener;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class FundamentalMatrixRefinerTest implements RefinerListener<FundamentalMatrix> {

    private static final int MIN_POINTS = 100;
    private static final int MAX_POINTS = 500;

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

    private static final int PERCENTAGE_OUTLIERS = 20;

    private static final double STD_ERROR = 10.0;

    private static final double THRESHOLD = 1e-6;

    private static final int TIMES = 100;

    private int mRefineStart;
    private int mRefineEnd;

    @Test
    void testConstructor() throws LockedException, NotReadyException, RobustEstimatorException {
        final var estimator = createRobustEstimator();
        final var fundamentalMatrix = estimator.estimate();
        final var inliersData = estimator.getInliersData();
        final var inliers = inliersData.getInliers();
        final var residuals = inliersData.getResiduals();
        final var numInliers = inliersData.getNumInliers();
        final var refinementStandardDeviation = estimator.getThreshold();
        final var samples1 = estimator.getLeftPoints();
        final var samples2 = estimator.getRightPoints();

        assertNotNull(fundamentalMatrix);
        assertNotNull(inliersData);

        // test empty constructor
        var refiner = new FundamentalMatrixRefiner();

        // check default values
        assertEquals(0.0, refiner.getRefinementStandardDeviation(), 0.0);
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
        refiner = new FundamentalMatrixRefiner(fundamentalMatrix, true, inliers, residuals, numInliers,
                samples1, samples2, refinementStandardDeviation);

        // check default values
        assertEquals(refinementStandardDeviation, refiner.getRefinementStandardDeviation(), 0.0);
        assertSame(samples1, refiner.getSamples1());
        assertSame(samples2, refiner.getSamples2());
        assertTrue(refiner.isReady());
        assertSame(inliers, refiner.getInliers());
        assertSame(residuals, refiner.getResiduals());
        assertEquals(numInliers, refiner.getNumInliers());
        assertEquals(samples1.size(), refiner.getTotalSamples());
        assertSame(fundamentalMatrix, refiner.getInitialEstimation());
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        // test non-empty constructor with InliersData
        refiner = new FundamentalMatrixRefiner(fundamentalMatrix, true, inliersData, samples1, samples2,
                refinementStandardDeviation);

        // check default values
        assertEquals(refinementStandardDeviation, refiner.getRefinementStandardDeviation(), 0.0);
        assertSame(samples1, refiner.getSamples1());
        assertSame(samples2, refiner.getSamples2());
        assertTrue(refiner.isReady());
        assertSame(inliers, refiner.getInliers());
        assertSame(residuals, refiner.getResiduals());
        assertEquals(numInliers, refiner.getNumInliers());
        assertEquals(samples1.size(), refiner.getTotalSamples());
        assertSame(fundamentalMatrix, refiner.getInitialEstimation());
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());
    }

    @Test
    void testGetSetListener() {
        final var refiner = new FundamentalMatrixRefiner();

        // check default value
        assertNull(refiner.getListener());

        // set new value
        refiner.setListener(this);

        // check correctness
        assertSame(this, refiner.getListener());
    }

    @Test
    void testGetSetRefinementStandardDeviation() throws LockedException {
        final var refiner = new FundamentalMatrixRefiner();

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
    void testGetSetSamples1() throws LockedException {
        final var estimator = createRobustEstimator();
        final var samples1 = estimator.getLeftPoints();

        final var refiner = new FundamentalMatrixRefiner();

        // check default value
        assertNull(refiner.getSamples1());

        // set new value
        refiner.setSamples1(samples1);

        // check correctness
        assertSame(samples1, refiner.getSamples1());
    }

    @Test
    void testGetSetSamples2() throws LockedException {
        final var estimator = createRobustEstimator();
        final var samples2 = estimator.getRightPoints();

        final var refiner = new FundamentalMatrixRefiner();

        // check default value
        assertNull(refiner.getSamples2());

        // set new value
        refiner.setSamples2(samples2);

        // check correctness
        assertSame(samples2, refiner.getSamples2());
    }

    @Test
    void testGetSetInliers() throws LockedException, NotReadyException, RobustEstimatorException {
        final var estimator = createRobustEstimator();

        assertNotNull(estimator.estimate());
        final var inliersData = estimator.getInliersData();
        final var inliers = inliersData.getInliers();

        final var refiner = new FundamentalMatrixRefiner();

        // check default value
        assertNull(refiner.getInliers());

        // set new value
        refiner.setInliers(inliers);

        // check correctness
        assertSame(inliers, refiner.getInliers());
    }

    @Test
    void testGetSetResiduals() throws LockedException, NotReadyException, RobustEstimatorException {
        final var estimator = createRobustEstimator();

        assertNotNull(estimator.estimate());
        final var inliersData = estimator.getInliersData();
        final var residuals = inliersData.getResiduals();

        final var refiner = new FundamentalMatrixRefiner();

        // check default value
        assertNull(refiner.getResiduals());

        // set new value
        refiner.setResiduals(residuals);

        // check correctness
        assertSame(residuals, refiner.getResiduals());
    }

    @Test
    void testGetSetNumInliers() throws LockedException, NotReadyException, RobustEstimatorException {
        final var estimator = createRobustEstimator();

        assertNotNull(estimator.estimate());
        final var inliersData = estimator.getInliersData();
        final var numInliers = inliersData.getNumInliers();

        final var refiner = new FundamentalMatrixRefiner();

        // check default value
        assertEquals(0, refiner.getNumInliers());

        // set new value
        refiner.setNumInliers(numInliers);

        // check correctness
        assertEquals(numInliers, refiner.getNumInliers());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> refiner.setNumInliers(0));
    }

    @Test
    void testSetInliersData() throws LockedException, NotReadyException, RobustEstimatorException {
        final var estimator = createRobustEstimator();

        assertNotNull(estimator.estimate());
        final var inliersData = estimator.getInliersData();

        final var refiner = new FundamentalMatrixRefiner();

        // check default values
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(0, refiner.getNumInliers());

        // set new value
        refiner.setInliersData(inliersData);

        // check correctness
        assertSame(inliersData.getInliers(), refiner.getInliers());
        assertSame(inliersData.getResiduals(), refiner.getResiduals());
        assertEquals(inliersData.getNumInliers(), refiner.getNumInliers());
    }

    @Test
    void testGetSetInitialEstimation() throws LockedException {
        final var refiner = new FundamentalMatrixRefiner();

        // check default value
        assertNull(refiner.getInitialEstimation());

        // set new value
        final var fundamentalMatrix = new FundamentalMatrix();
        refiner.setInitialEstimation(fundamentalMatrix);

        // check correctness
        assertSame(fundamentalMatrix, refiner.getInitialEstimation());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var refiner = new FundamentalMatrixRefiner();

        // check default value
        assertFalse(refiner.isCovarianceKept());

        // set new value
        refiner.setCovarianceKept(true);

        // check correctness
        assertTrue(refiner.isCovarianceKept());
    }

    @Test
    void testRefine() throws LockedException, NotReadyException, RobustEstimatorException, RefinerException,
            NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var estimator = createRobustEstimator();

            final var fundamentalMatrix = estimator.estimate();
            final var inliersData = estimator.getInliersData();
            final var refinementStandardDeviation = estimator.getThreshold();
            final var samples1 = estimator.getLeftPoints();
            final var samples2 = estimator.getRightPoints();

            final var refiner = new FundamentalMatrixRefiner(fundamentalMatrix, true, inliersData,
                    samples1, samples2, refinementStandardDeviation);
            refiner.setListener(this);

            final var result1 = new FundamentalMatrix();

            reset();
            assertEquals(0, mRefineStart);
            assertEquals(0, mRefineEnd);

            if (!refiner.refine(result1)) {
                continue;
            }

            final var result2 = refiner.refine();

            assertEquals(2, mRefineStart);
            assertEquals(2, mRefineEnd);

            result1.normalize();
            result2.normalize();

            assertEquals(result1.getInternalMatrix(), result2.getInternalMatrix());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }


    private RANSACFundamentalMatrixRobustEstimator createRobustEstimator() throws LockedException {
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

        // add outliers
        final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);

        final var leftPointsWithError = new ArrayList<Point2D>();
        for (final var leftPoint : leftPoints) {
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                // outlier
                final var errorX = errorRandomizer.nextDouble();
                final var errorY = errorRandomizer.nextDouble();
                leftPointsWithError.add(new HomogeneousPoint2D(leftPoint.getInhomX() + errorX,
                        leftPoint.getInhomY() + errorY, 1.0));
            } else {
                // inlier
                leftPointsWithError.add(leftPoint);
            }
        }

        final var rightPointsWithError = new ArrayList<Point2D>();
        for (final var rightPoint : rightPoints) {
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                // outlier
                final var errorX = errorRandomizer.nextDouble();
                final var errorY = errorRandomizer.nextDouble();
                rightPointsWithError.add(new HomogeneousPoint2D(rightPoint.getInhomX() + errorX,
                        rightPoint.getInhomY() + errorY, 1.0));
            } else {
                // inlier
                rightPointsWithError.add(rightPoint);
            }
        }

        // create fundamental matrix estimator
        final var estimator = new RANSACFundamentalMatrixRobustEstimator(leftPointsWithError, rightPointsWithError);
        estimator.setThreshold(THRESHOLD);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);

        return estimator;
    }

    @Override
    public void onRefineStart(final Refiner<FundamentalMatrix> refiner, final FundamentalMatrix initialEstimation) {
        mRefineStart++;
        checkLocked((FundamentalMatrixRefiner) refiner);
    }

    @Override
    public void onRefineEnd(final Refiner<FundamentalMatrix> refiner, final FundamentalMatrix initialEstimation,
                            final FundamentalMatrix result, final boolean errorDecreased) {
        mRefineEnd++;
        checkLocked((FundamentalMatrixRefiner) refiner);
    }

    private void reset() {
        mRefineStart = mRefineEnd = 0;
    }

    private static void checkLocked(final FundamentalMatrixRefiner refiner) {
        assertTrue(refiner.isLocked());
        assertThrows(LockedException.class, () -> refiner.setInitialEstimation(null));
        assertThrows(LockedException.class, () -> refiner.setCovarianceKept(true));
        assertThrows(LockedException.class, () -> refiner.refine(null));
        assertThrows(LockedException.class, refiner::refine);
        assertThrows(LockedException.class, () -> refiner.setInliers(null));
        assertThrows(LockedException.class, () -> refiner.setResiduals(null));
        assertThrows(LockedException.class, () -> refiner.setNumInliers(0));
        assertThrows(LockedException.class, () -> refiner.setInliersData(null));
        assertThrows(LockedException.class, () -> refiner.setSamples1(null));
        assertThrows(LockedException.class, () -> refiner.setSamples2(null));
        assertThrows(LockedException.class, () -> refiner.setRefinementStandardDeviation(0.0));
    }
}
