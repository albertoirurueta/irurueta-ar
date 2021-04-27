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
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.refiners.Refiner;
import com.irurueta.geometry.refiners.RefinerException;
import com.irurueta.geometry.refiners.RefinerListener;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class FundamentalMatrixRefinerTest implements
        RefinerListener<FundamentalMatrix> {

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
    public void testConstructor() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final RANSACFundamentalMatrixRobustEstimator estimator =
                createRobustEstimtor();
        final FundamentalMatrix fundamentalMatrix = estimator.estimate();
        final InliersData inliersData = estimator.getInliersData();
        final BitSet inliers = inliersData.getInliers();
        final double[] residuals = inliersData.getResiduals();
        final int numInliers = inliersData.getNumInliers();
        final double refinementStandardDeviation = estimator.getThreshold();
        final List<Point2D> samples1 = estimator.getLeftPoints();
        final List<Point2D> samples2 = estimator.getRightPoints();

        assertNotNull(fundamentalMatrix);
        assertNotNull(inliersData);

        // test empty constructor
        FundamentalMatrixRefiner refiner = new FundamentalMatrixRefiner();

        // check default values
        assertEquals(refiner.getRefinementStandardDeviation(), 0.0, 0.0);
        assertNull(refiner.getSamples1());
        assertNull(refiner.getSamples2());
        assertFalse(refiner.isReady());
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(refiner.getNumInliers(), 0);
        assertEquals(refiner.getTotalSamples(), 0);
        assertNull(refiner.getInitialEstimation());
        assertFalse(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        // test non-empty constructor
        refiner = new FundamentalMatrixRefiner(fundamentalMatrix, true, inliers,
                residuals, numInliers, samples1, samples2,
                refinementStandardDeviation);

        // check default values
        assertEquals(refiner.getRefinementStandardDeviation(),
                refinementStandardDeviation, 0.0);
        assertSame(refiner.getSamples1(), samples1);
        assertSame(refiner.getSamples2(), samples2);
        assertTrue(refiner.isReady());
        assertSame(refiner.getInliers(), inliers);
        assertSame(refiner.getResiduals(), residuals);
        assertEquals(refiner.getNumInliers(), numInliers);
        assertEquals(refiner.getTotalSamples(), samples1.size());
        assertSame(refiner.getInitialEstimation(), fundamentalMatrix);
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());

        // test non-empty constructor with InliersData
        refiner = new FundamentalMatrixRefiner(fundamentalMatrix, true,
                inliersData, samples1, samples2, refinementStandardDeviation);

        // check default values
        assertEquals(refiner.getRefinementStandardDeviation(),
                refinementStandardDeviation, 0.0);
        assertSame(refiner.getSamples1(), samples1);
        assertSame(refiner.getSamples2(), samples2);
        assertTrue(refiner.isReady());
        assertSame(refiner.getInliers(), inliers);
        assertSame(refiner.getResiduals(), residuals);
        assertEquals(refiner.getNumInliers(), numInliers);
        assertEquals(refiner.getTotalSamples(), samples1.size());
        assertSame(refiner.getInitialEstimation(), fundamentalMatrix);
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());
    }

    @Test
    public void testGetSetListener() {
        final FundamentalMatrixRefiner refiner = new FundamentalMatrixRefiner();

        // check default value
        assertNull(refiner.getListener());

        // set new value
        refiner.setListener(this);

        // check correctness
        assertSame(refiner.getListener(), this);
    }

    @Test
    public void testGetSetRefinementStandardDeviation() throws LockedException {
        final FundamentalMatrixRefiner refiner = new FundamentalMatrixRefiner();

        // check default value
        assertEquals(refiner.getRefinementStandardDeviation(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double refinementStandardDeviation = randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        refiner.setRefinementStandardDeviation(refinementStandardDeviation);

        // check correctness
        assertEquals(refiner.getRefinementStandardDeviation(),
                refinementStandardDeviation, 0.0);
    }

    @Test
    public void testGetSetSamples1() throws LockedException {
        final RANSACFundamentalMatrixRobustEstimator estimator =
                createRobustEstimtor();
        final List<Point2D> samples1 = estimator.getLeftPoints();

        final FundamentalMatrixRefiner refiner = new FundamentalMatrixRefiner();

        // check default value
        assertNull(refiner.getSamples1());

        // set new value
        refiner.setSamples1(samples1);

        // check correctness
        assertSame(refiner.getSamples1(), samples1);
    }

    @Test
    public void testGetSetSamples2() throws LockedException {
        final RANSACFundamentalMatrixRobustEstimator estimator =
                createRobustEstimtor();
        final List<Point2D> samples2 = estimator.getRightPoints();

        final FundamentalMatrixRefiner refiner = new FundamentalMatrixRefiner();

        // check default value
        assertNull(refiner.getSamples2());

        // set new value
        refiner.setSamples2(samples2);

        // check correctness
        assertSame(refiner.getSamples2(), samples2);
    }

    @Test
    public void testGetSetInliers() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final RANSACFundamentalMatrixRobustEstimator estimator =
                createRobustEstimtor();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final BitSet inliers = inliersData.getInliers();

        final FundamentalMatrixRefiner refiner = new FundamentalMatrixRefiner();

        // check default value
        assertNull(refiner.getInliers());

        // set new value
        refiner.setInliers(inliers);

        // check correctness
        assertSame(refiner.getInliers(), inliers);
    }

    @Test
    public void testGetSetResiduals() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final RANSACFundamentalMatrixRobustEstimator estimator =
                createRobustEstimtor();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final double[] residuals = inliersData.getResiduals();

        final FundamentalMatrixRefiner refiner = new FundamentalMatrixRefiner();

        // check default value
        assertNull(refiner.getResiduals());

        // set new value
        refiner.setResiduals(residuals);

        // check correctness
        assertSame(refiner.getResiduals(), residuals);
    }

    @Test
    public void testGetSetNumInliers() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final RANSACFundamentalMatrixRobustEstimator estimator =
                createRobustEstimtor();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();
        final int numInliers = inliersData.getNumInliers();

        final FundamentalMatrixRefiner refiner = new FundamentalMatrixRefiner();

        // check default value
        assertEquals(refiner.getNumInliers(), 0);

        // set new value
        refiner.setNumInliers(numInliers);

        // check correctness
        assertEquals(refiner.getNumInliers(), numInliers);

        // Force IllegalArgumentException
        try {
            refiner.setNumInliers(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSetInliersData() throws LockedException, NotReadyException,
            RobustEstimatorException {
        final RANSACFundamentalMatrixRobustEstimator estimator =
                createRobustEstimtor();

        assertNotNull(estimator.estimate());
        final InliersData inliersData = estimator.getInliersData();

        final FundamentalMatrixRefiner refiner = new FundamentalMatrixRefiner();

        // check default values
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(refiner.getNumInliers(), 0);

        // set new value
        refiner.setInliersData(inliersData);

        // check correctness
        assertSame(refiner.getInliers(), inliersData.getInliers());
        assertSame(refiner.getResiduals(), inliersData.getResiduals());
        assertEquals(refiner.getNumInliers(), inliersData.getNumInliers());
    }

    @Test
    public void testGetSetInitialEstimation() throws LockedException {
        final FundamentalMatrixRefiner refiner = new FundamentalMatrixRefiner();

        // check default value
        assertNull(refiner.getInitialEstimation());

        // set new value
        final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
        refiner.setInitialEstimation(fundamentalMatrix);

        // check correctness
        assertSame(refiner.getInitialEstimation(), fundamentalMatrix);
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final FundamentalMatrixRefiner refiner = new FundamentalMatrixRefiner();

        // check default value
        assertFalse(refiner.isCovarianceKept());

        // set new value
        refiner.setCovarianceKept(true);

        // check correctness
        assertTrue(refiner.isCovarianceKept());
    }

    @Test
    public void testRefine() throws LockedException, NotReadyException,
            RobustEstimatorException, RefinerException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final RANSACFundamentalMatrixRobustEstimator estimator =
                    createRobustEstimtor();

            final FundamentalMatrix fundamentalMatrix = estimator.estimate();
            final InliersData inliersData = estimator.getInliersData();
            final double refinementStandardDeviation = estimator.getThreshold();
            final List<Point2D> samples1 = estimator.getLeftPoints();
            final List<Point2D> samples2 = estimator.getRightPoints();

            final FundamentalMatrixRefiner refiner = new FundamentalMatrixRefiner(
                    fundamentalMatrix, true, inliersData, samples1, samples2,
                    refinementStandardDeviation);
            refiner.setListener(this);

            final FundamentalMatrix result1 = new FundamentalMatrix();

            reset();
            assertEquals(mRefineStart, 0);
            assertEquals(mRefineEnd, 0);

            if (!refiner.refine(result1)) {
                continue;
            }

            final FundamentalMatrix result2 = refiner.refine();

            assertEquals(mRefineStart, 2);
            assertEquals(mRefineEnd, 2);

            result1.normalize();
            result2.normalize();

            assertEquals(result1.getInternalMatrix(),
                    result2.getInternalMatrix());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }


    private RANSACFundamentalMatrixRobustEstimator createRobustEstimtor()
            throws LockedException {
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

        // add outliers
        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                new Random(), 0.0, STD_ERROR);

        final List<Point2D> leftPointsWithError = new ArrayList<>();
        for (final Point2D leftPoint : leftPoints) {
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                // outlier
                final double errorX = errorRandomizer.nextDouble();
                final double errorY = errorRandomizer.nextDouble();
                leftPointsWithError.add(new HomogeneousPoint2D(
                        leftPoint.getInhomX() + errorX,
                        leftPoint.getInhomY() + errorY, 1.0));
            } else {
                // inlier
                leftPointsWithError.add(leftPoint);
            }
        }

        final List<Point2D> rightPointsWithError = new ArrayList<>();
        for (final Point2D rightPoint : rightPoints) {
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                // outlier
                final double errorX = errorRandomizer.nextDouble();
                final double errorY = errorRandomizer.nextDouble();
                rightPointsWithError.add(new HomogeneousPoint2D(
                        rightPoint.getInhomX() + errorX,
                        rightPoint.getInhomY() + errorY, 1.0));
            } else {
                // inlier
                rightPointsWithError.add(rightPoint);
            }
        }

        // create fundamental matrix estimator
        final RANSACFundamentalMatrixRobustEstimator estimator =
                new RANSACFundamentalMatrixRobustEstimator(
                        leftPointsWithError, rightPointsWithError);
        estimator.setThreshold(THRESHOLD);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);

        return estimator;
    }

    @Override
    public void onRefineStart(final Refiner<FundamentalMatrix> refiner,
                              final FundamentalMatrix initialEstimation) {
        mRefineStart++;
        checkLocked((FundamentalMatrixRefiner) refiner);
    }

    @Override
    public void onRefineEnd(final Refiner<FundamentalMatrix> refiner,
                            final FundamentalMatrix initialEstimation,
                            final FundamentalMatrix result,
                            final boolean errorDecreased) {
        mRefineEnd++;
        checkLocked((FundamentalMatrixRefiner) refiner);
    }

    private void reset() {
        mRefineStart = mRefineEnd = 0;
    }

    private void checkLocked(final FundamentalMatrixRefiner refiner) {
        assertTrue(refiner.isLocked());
        try {
            refiner.setInitialEstimation(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.refine(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            refiner.refine();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            refiner.setInliers(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setResiduals(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setNumInliers(0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setInliersData(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSamples1(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setSamples2(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            refiner.setRefinementStandardDeviation(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }
}
