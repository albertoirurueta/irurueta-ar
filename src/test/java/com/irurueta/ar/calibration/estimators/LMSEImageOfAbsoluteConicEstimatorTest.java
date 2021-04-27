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

import com.irurueta.ar.calibration.ImageOfAbsoluteConic;
import com.irurueta.ar.calibration.Pattern2D;
import com.irurueta.ar.calibration.Pattern2DType;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.ProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class LMSEImageOfAbsoluteConicEstimatorTest implements
        ImageOfAbsoluteConicEstimatorListener {

    private static final double MIN_FOCAL_LENGTH = 3.0;
    private static final double MAX_FOCAL_LENGTH = 10.0;

    private static final double MIN_SKEWNESS = -0.01;
    private static final double MAX_SKEWNESS = 0.01;

    private static final double MIN_PRINCIPAL_POINT = -0.2;
    private static final double MAX_PRINCIPAL_POINT = 0.2;

    private static final double MIN_ANGLE_DEGREES = -10.0;
    private static final double MAX_ANGLE_DEGREES = 10.0;

    private static final int INHOM_3D_COORDS = 3;

    private static final double MIN_RANDOM_VALUE = -50.0;
    private static final double MAX_RANDOM_VALUE = -10.0;

    private static final double ABSOLUTE_ERROR = 5e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-3;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 5e-1;
    private static final double ULTRA_LARGE_ABSOLUTE_ERROR = 3.0;

    private static final int TIMES = 50;

    private int estimateStart;
    private int estimateEnd;
    private int estimationProgressChange;

    @Test
    public void testConstructor() {
        // test constructor without arguments
        LMSEImageOfAbsoluteConicEstimator estimator =
                new LMSEImageOfAbsoluteConicEstimator();

        // check default values
        assertEquals(estimator.isZeroSkewness(),
                LMSEImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertNull(estimator.getHomographies());
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(),
                ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);
        assertEquals(estimator.isLMSESolutionAllowed(),
                LMSEImageOfAbsoluteConicEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);

        // test constructor with listener
        estimator = new LMSEImageOfAbsoluteConicEstimator(this);

        // check default values
        assertEquals(estimator.isZeroSkewness(),
                LMSEImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getHomographies());
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(),
                ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);
        assertEquals(estimator.isLMSESolutionAllowed(),
                LMSEImageOfAbsoluteConicEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);

        // test constructor with homographies
        final List<Transformation2D> homographies = new ArrayList<>();
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());

        estimator = new LMSEImageOfAbsoluteConicEstimator(homographies);

        // check default values
        assertEquals(estimator.isZeroSkewness(),
                LMSEImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getType(),
                ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);
        assertEquals(estimator.isLMSESolutionAllowed(),
                LMSEImageOfAbsoluteConicEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMSEImageOfAbsoluteConicEstimator(
                    (List<Transformation2D>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        final List<Transformation2D> emptyHomographies =
                new ArrayList<>();
        try {
            estimator = new LMSEImageOfAbsoluteConicEstimator(
                    emptyHomographies);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with homographies and listener
        estimator = new LMSEImageOfAbsoluteConicEstimator(homographies, this);

        // check default values
        assertEquals(estimator.isZeroSkewness(),
                LMSEImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getType(),
                ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);
        assertEquals(estimator.isLMSESolutionAllowed(),
                LMSEImageOfAbsoluteConicEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMSEImageOfAbsoluteConicEstimator(
                    (List<Transformation2D>) null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LMSEImageOfAbsoluteConicEstimator(
                    emptyHomographies);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testIsSetZeroSkewness() throws LockedException {
        final LMSEImageOfAbsoluteConicEstimator estimator =
                new LMSEImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(estimator.isZeroSkewness(),
                LMSEImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);

        // set new value
        estimator.setZeroSkewness(
                !LMSEImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);

        // check correctness
        assertEquals(estimator.isZeroSkewness(),
                !LMSEImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
    }

    @Test
    public void testIsSetPrincipalPointAtOrigin() throws LockedException {
        final LMSEImageOfAbsoluteConicEstimator estimator =
                new LMSEImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);

        // set new value
        estimator.setPrincipalPointAtOrigin(!LMSEImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);

        // check correctness
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                !LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
    }

    @Test
    public void testIsFocalDistanceAspectRatioKnown() throws LockedException {
        final LMSEImageOfAbsoluteConicEstimator estimator =
                new LMSEImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);

        // set new value
        estimator.setFocalDistanceAspectRatioKnown(
                !LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
    }

    @Test
    public void testGetSetFocalDistanceAspectRatio() throws LockedException {
        final LMSEImageOfAbsoluteConicEstimator estimator =
                new LMSEImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                LMSEImageOfAbsoluteConicEstimator.
                        DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);

        // set new value
        estimator.setFocalDistanceAspectRatio(0.5);

        // check correctness
        assertEquals(estimator.getFocalDistanceAspectRatio(), 0.5, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setFocalDistanceAspectRatio(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final LMSEImageOfAbsoluteConicEstimator estimator =
                new LMSEImageOfAbsoluteConicEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetHomographiesAndIsReady() throws LockedException {
        final LMSEImageOfAbsoluteConicEstimator estimator =
                new LMSEImageOfAbsoluteConicEstimator();

        // check default value
        assertNull(estimator.getHomographies());
        assertFalse(estimator.isReady());

        // set new value
        final List<Transformation2D> homographies = new ArrayList<>();
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());

        estimator.setHomographies(homographies);

        // check correctness
        assertSame(estimator.getHomographies(), homographies);
        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        try {
            estimator.setHomographies(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        final List<Transformation2D> emptyHomographies =
                new ArrayList<>();
        try {
            estimator.setHomographies(emptyHomographies);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetLMSESolutionAllowed() throws LockedException {
        final LMSEImageOfAbsoluteConicEstimator estimator =
                new LMSEImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(estimator.isLMSESolutionAllowed(),
                LMSEImageOfAbsoluteConicEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);

        // set new value
        estimator.setLMSESolutionAllowed(
                !LMSEImageOfAbsoluteConicEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);

        // check correctness
        assertEquals(estimator.isLMSESolutionAllowed(),
                !LMSEImageOfAbsoluteConicEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);
    }

    @Test
    public void testEstimateNoContraintsAndNoLMSE()
            throws InvalidPinholeCameraIntrinsicParametersException,
            LockedException, NotReadyException, RobustEstimatorException,
            ImageOfAbsoluteConicEstimatorException {

        boolean succeededAtLeastOnce = false;
        int failed = 0;
        int succeeded = 0;
        int total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (int j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final List<Point2D> patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final List<Point3D> points3D = new ArrayList<>();
            for (final Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final List<Transformation2D> homographies =
                    new ArrayList<>();
            for (int i = 0; i < 3; i++) {
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
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                        cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                                createFromPoints(patternPoints, projectedPatternPoints,
                                        RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography =
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final LMSEImageOfAbsoluteConicEstimator estimator =
                    new LMSEImageOfAbsoluteConicEstimator(homographies, this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setZeroSkewness(false);
            estimator.setPrincipalPointAtOrigin(false);
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 3);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            // estimate
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            if (Math.abs(Math.abs(iac.getA()) - Math.abs(iac2.getA())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(iac.getB()) - Math.abs(iac2.getB())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getB()), Math.abs(iac2.getB()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(iac.getC()) - Math.abs(iac2.getC())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(iac.getD()) - Math.abs(iac2.getD())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getD()), Math.abs(iac2.getD()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(iac.getE()) - Math.abs(iac2.getE())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getE()), Math.abs(iac2.getE()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(iac.getF()) - Math.abs(iac2.getF())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()),
                    VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final PinholeCameraIntrinsicParameters intrinsic2 =
                        iac2.getIntrinsicParameters();

                if (Math.abs(intrinsic.getHorizontalFocalLength() - intrinsic2.getHorizontalFocalLength()) >
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalFocalLength(),
                        intrinsic2.getHorizontalFocalLength(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalFocalLength() - intrinsic2.getVerticalFocalLength()) >
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness()) > VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getSkewness(),
                        intrinsic2.getSkewness(), VERY_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getHorizontalPrincipalPoint() - intrinsic2.getHorizontalPrincipalPoint()) >
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                        intrinsic2.getHorizontalPrincipalPoint(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalPrincipalPoint() - intrinsic2.getVerticalPrincipalPoint()) >
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalPrincipalPoint(),
                        intrinsic2.getVerticalPrincipalPoint(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);

                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                                intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                                intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                                intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                                intrinsic2.getVerticalPrincipalPoint());

                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;

                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                succeededAtLeastOnce = true;
                succeeded++;
            } catch (final InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }

        final double failedRatio = (double) failed / (double) total;
        final double succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);

        final String msg = "NO LMSE, No contraints - failed: " +
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 +
                "% avg horizontal focal distance error: " +
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " vertical principal point error: " +
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);

        assertTrue(failedRatio < 0.6);
        assertTrue(succeededRatio >= 0.4);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    public void testEstimateNoContraintsAndLMSE()
            throws InvalidPinholeCameraIntrinsicParametersException,
            LockedException, NotReadyException, RobustEstimatorException,
            ImageOfAbsoluteConicEstimatorException {

        boolean succeededAtLeastOnce = false;
        int failed = 0;
        int succeeded = 0;
        int total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (int j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);

            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final List<Point2D> patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final List<Point3D> points3D = new ArrayList<>();
            for (final Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            // create 50 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 50 homographies to
            // estimate the IAC
            final List<Transformation2D> homographies =
                    new ArrayList<>();
            for (int i = 0; i < 50; i++) {
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
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                        cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                                createFromPoints(patternPoints, projectedPatternPoints,
                                        RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography =
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final LMSEImageOfAbsoluteConicEstimator estimator =
                    new LMSEImageOfAbsoluteConicEstimator(homographies, this);
            estimator.setLMSESolutionAllowed(true);
            estimator.setZeroSkewness(false);
            estimator.setPrincipalPointAtOrigin(false);
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 3);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            // estimate
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getB()), Math.abs(iac2.getB()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getD()), Math.abs(iac2.getD()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getE()), Math.abs(iac2.getE()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()),
                    VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final PinholeCameraIntrinsicParameters intrinsic2 =
                        iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(),
                        intrinsic2.getHorizontalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(),
                        intrinsic2.getSkewness(), VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                        intrinsic2.getHorizontalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalPrincipalPoint(),
                        intrinsic2.getVerticalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);

                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                                intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                                intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                                intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                                intrinsic2.getVerticalPrincipalPoint());

                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;

                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                succeededAtLeastOnce = true;
                succeeded++;
            } catch (final InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }

        final double failedRatio = (double) failed / (double) total;
        final double succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0,
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0,
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, LARGE_ABSOLUTE_ERROR);

        final String msg = "LMSE, No contraints - failed: " +
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 +
                "% avg horizontal focal distance error: " +
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);

        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    public void testEstimateZeroSkewnessAndNoLMSE()
            throws InvalidPinholeCameraIntrinsicParametersException,
            LockedException, NotReadyException, RobustEstimatorException,
            ImageOfAbsoluteConicEstimatorException {

        boolean succeededAtLeastOnce = false;
        int failed = 0;
        int succeeded = 0;
        int total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (int j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final List<Point2D> patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final List<Point3D> points3D = new ArrayList<>();
            for (final Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final List<Transformation2D> homographies =
                    new ArrayList<>();
            for (int i = 0; i < 2; i++) {
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
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                        cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                                createFromPoints(patternPoints, projectedPatternPoints,
                                        RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography =
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final LMSEImageOfAbsoluteConicEstimator estimator =
                    new LMSEImageOfAbsoluteConicEstimator(this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(false);
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 2);
            estimator.setHomographies(homographies);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            // estimate
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getB(), 0.0, 0.0);
            assertEquals(iac2.getB(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getD()), Math.abs(iac2.getD()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getE()), Math.abs(iac2.getE()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()),
                    VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final PinholeCameraIntrinsicParameters intrinsic2 =
                        iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
                if (Math.abs(intrinsic.getHorizontalPrincipalPoint() -
                        intrinsic2.getHorizontalPrincipalPoint()) > ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                        intrinsic2.getHorizontalPrincipalPoint(),
                        ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalPrincipalPoint() -
                        intrinsic2.getHorizontalPrincipalPoint()) > ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalPrincipalPoint(),
                        intrinsic2.getVerticalPrincipalPoint(),
                        ULTRA_LARGE_ABSOLUTE_ERROR);

                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                                intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                                intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                                intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                                intrinsic2.getVerticalPrincipalPoint());

                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;

                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                succeededAtLeastOnce = true;
                succeeded++;
            } catch (final InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }

        final double failedRatio = (double) failed / (double) total;
        final double succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);

        final String msg = "NO LMSE, Zero skewness - failed: " +
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 +
                "% avg horizontal focal distance error: " +
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);

        assertTrue(failedRatio < 0.6);
        assertTrue(succeededRatio >= 0.4);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    public void testEstimateZeroSkewnessAndLMSE()
            throws InvalidPinholeCameraIntrinsicParametersException,
            LockedException, NotReadyException, RobustEstimatorException,
            ImageOfAbsoluteConicEstimatorException {

        boolean succeededAtLeastOnce = false;
        int failed = 0;
        int succeeded = 0;
        int total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (int j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final List<Point2D> patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final List<Point3D> points3D = new ArrayList<>();
            for (final Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final List<Transformation2D> homographies =
                    new ArrayList<>();
            for (int i = 0; i < 50; i++) {
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
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                        cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                                createFromPoints(patternPoints, projectedPatternPoints,
                                        RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography =
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final LMSEImageOfAbsoluteConicEstimator estimator =
                    new LMSEImageOfAbsoluteConicEstimator(homographies, this);
            estimator.setLMSESolutionAllowed(true);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(false);
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 2);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            // estimate
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getB(), 0.0, 0.0);
            assertEquals(iac2.getB(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getD()), Math.abs(iac2.getD()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getE()), Math.abs(iac2.getE()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()),
                    VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final PinholeCameraIntrinsicParameters intrinsic2 =
                        iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(),
                        intrinsic2.getHorizontalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                        intrinsic2.getHorizontalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalPrincipalPoint(),
                        intrinsic2.getVerticalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);

                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                                intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                                intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                                intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                                intrinsic2.getVerticalPrincipalPoint());

                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;

                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                succeededAtLeastOnce = true;
                succeeded++;
            } catch (final InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }

        final double failedRatio = (double) failed / (double) total;
        final double succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0,
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0,
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, LARGE_ABSOLUTE_ERROR);

        final String msg = "LMSE, Zero skewness - failed: " +
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 +
                "% avg horizontal focal distance error: " +
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);

        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    public void testEstimatePrincipalPointAtOriginAndNoLMSE()
            throws InvalidPinholeCameraIntrinsicParametersException,
            LockedException, NotReadyException, RobustEstimatorException,
            ImageOfAbsoluteConicEstimatorException {

        boolean succeededAtLeastOnce = false;
        int failed = 0;
        int succeeded = 0;
        int total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (int j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);

            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final double horizontalPrincipalPoint = 0.0;
            final double verticalPrincipalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final List<Point2D> patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final List<Point3D> points3D = new ArrayList<>();
            for (final Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final List<Transformation2D> homographies =
                    new ArrayList<>();
            for (int i = 0; i < 2; i++) {
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
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                        cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                                createFromPoints(patternPoints, projectedPatternPoints,
                                        RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography =
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final LMSEImageOfAbsoluteConicEstimator estimator =
                    new LMSEImageOfAbsoluteConicEstimator(this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setZeroSkewness(false);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 2);
            estimator.setHomographies(homographies);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            // estimate
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            if (Math.abs(Math.abs(iac.getA()) - Math.abs(iac2.getA())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(iac.getB()) - Math.abs(iac2.getB())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getB()), Math.abs(iac2.getB()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(iac.getC()) - Math.abs(iac2.getC())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getD(), 0.0, 0.0);
            assertEquals(iac2.getD(), 0.0, 0.0);
            assertEquals(iac.getE(), 0.0, 0.0);
            assertEquals(iac2.getE(), 0.0, 0.0);
            if (Math.abs(Math.abs(iac.getF()) - Math.abs(iac2.getF())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()),
                    VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final PinholeCameraIntrinsicParameters intrinsic2 =
                        iac2.getIntrinsicParameters();

                if (Math.abs(intrinsic.getHorizontalFocalLength() - intrinsic2.getHorizontalFocalLength()) >
                        3 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalFocalLength(),
                        intrinsic2.getHorizontalFocalLength(),
                        3 * ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalFocalLength() - intrinsic2.getVerticalFocalLength()) >
                        3 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(),
                        3 * ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness()) > VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getSkewness(),
                        intrinsic2.getSkewness(), VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0,
                        0.0);
                assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);

                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                                intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                                intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                                intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                                intrinsic2.getVerticalPrincipalPoint());

                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;

                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                succeededAtLeastOnce = true;
                succeeded++;
            } catch (final InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }

        final double failedRatio = (double) failed / (double) total;
        final double succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);

        final String msg = "NO LMSE, Principal point at origin - failed: " +
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 +
                "% avg horizontal focal distance error: " +
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);

        assertTrue(failedRatio < 0.5);
        assertTrue(succeededRatio >= 0.5);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    public void testEstimatePrincipalPointAtOriginAndLMSE()
            throws InvalidPinholeCameraIntrinsicParametersException,
            LockedException, NotReadyException, RobustEstimatorException,
            ImageOfAbsoluteConicEstimatorException {

        boolean succeededAtLeastOnce = false;
        int failed = 0;
        int succeeded = 0;
        int total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (int j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = 0.0;
            final double verticalPrincipalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final List<Point2D> patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final List<Point3D> points3D = new ArrayList<>();
            for (final Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final List<Transformation2D> homographies =
                    new ArrayList<>();
            for (int i = 0; i < 50; i++) {
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
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                        cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                                createFromPoints(patternPoints, projectedPatternPoints,
                                        RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography =
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final LMSEImageOfAbsoluteConicEstimator estimator =
                    new LMSEImageOfAbsoluteConicEstimator(homographies, this);
            estimator.setLMSESolutionAllowed(true);
            estimator.setZeroSkewness(false);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 2);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            // estimate
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getB()), Math.abs(iac2.getB()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getD(), 0.0, 0.0);
            assertEquals(iac2.getD(), 0.0, 0.0);
            assertEquals(iac.getE(), 0.0, 0.0);
            assertEquals(iac2.getE(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()),
                    VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final PinholeCameraIntrinsicParameters intrinsic2 =
                        iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(),
                        intrinsic2.getHorizontalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(),
                        intrinsic2.getSkewness(), VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0,
                        0.0);
                assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);

                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                                intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                                intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                                intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                                intrinsic2.getVerticalPrincipalPoint());

                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;

                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                succeededAtLeastOnce = true;
                succeeded++;
            } catch (final InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }

        final double failedRatio = (double) failed / (double) total;
        final double succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0,
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0,
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, LARGE_ABSOLUTE_ERROR);

        final String msg = "LMSE, Principal point at origin - failed: " +
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 +
                "% avg horizontal focal distance error: " +
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);

        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    public void testEstimateZeroSkewnessPrincipalPointAtOriginAndNoLMSE()
            throws InvalidPinholeCameraIntrinsicParametersException,
            LockedException, NotReadyException, RobustEstimatorException,
            ImageOfAbsoluteConicEstimatorException {

        boolean succeededAtLeastOnce = false;
        int failed = 0;
        int succeeded = 0;
        int total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (int j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double horizontalPrincipalPoint = 0.0;
            final double verticalPrincipalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final List<Point2D> patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final List<Point3D> points3D = new ArrayList<>();
            for (final Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final List<Transformation2D> homographies =
                    new ArrayList<>();
            for (int i = 0; i < 1; i++) {
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
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                        cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                                createFromPoints(patternPoints, projectedPatternPoints,
                                        RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography =
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final LMSEImageOfAbsoluteConicEstimator estimator =
                    new LMSEImageOfAbsoluteConicEstimator(this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
            estimator.setHomographies(homographies);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            // estimate
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()),
                    ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getB(), 0.0, 0.0);
            assertEquals(iac2.getB(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()),
                    ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getD(), 0.0, 0.0);
            assertEquals(iac2.getD(), 0.0, 0.0);
            assertEquals(iac.getE(), 0.0, 0.0);
            assertEquals(iac2.getE(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()),
                    ULTRA_LARGE_ABSOLUTE_ERROR);

            try {
                final PinholeCameraIntrinsicParameters intrinsic2 =
                        iac2.getIntrinsicParameters();

                if (Math.abs(intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength()) > 2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalFocalLength(),
                        intrinsic2.getHorizontalFocalLength(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalFocalLength() -
                        intrinsic2.getVerticalFocalLength()) > 2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0,
                        0.0);
                assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);

                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                                intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                                intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                                intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                                intrinsic2.getVerticalPrincipalPoint());

                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;

                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                succeededAtLeastOnce = true;
                succeeded++;
            } catch (final InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }

        final double failedRatio = (double) failed / (double) total;
        final double succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);

        final String msg = "NO LMSE, No skewness and Principal point at origin - failed: " +
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 +
                "% avg horizontal focal distance error: " +
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);

        assertTrue(failedRatio < 0.5);
        assertTrue(succeededRatio >= 0.5);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    public void testEstimateZeroSkewnessPrincipalPointAtOriginAndLMSE()
            throws InvalidPinholeCameraIntrinsicParametersException,
            LockedException, NotReadyException, RobustEstimatorException,
            ImageOfAbsoluteConicEstimatorException {

        boolean succeededAtLeastOnce = false;
        int failed = 0;
        int succeeded = 0;
        int total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (int j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double horizontalPrincipalPoint = 0.0;
            final double verticalPrincipalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final List<Point2D> patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final List<Point3D> points3D = new ArrayList<>();
            for (final Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final List<Transformation2D> homographies =
                    new ArrayList<>();
            for (int i = 0; i < 50; i++) {
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
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                        cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                                createFromPoints(patternPoints, projectedPatternPoints,
                                        RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography =
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final LMSEImageOfAbsoluteConicEstimator estimator =
                    new LMSEImageOfAbsoluteConicEstimator(homographies, this);
            estimator.setLMSESolutionAllowed(true);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            // estimate
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getB(), 0.0, 0.0);
            assertEquals(iac2.getB(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getD(), 0.0, 0.0);
            assertEquals(iac2.getD(), 0.0, 0.0);
            assertEquals(iac.getE(), 0.0, 0.0);
            assertEquals(iac2.getE(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()),
                    VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final PinholeCameraIntrinsicParameters intrinsic2 =
                        iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(),
                        intrinsic2.getHorizontalFocalLength(),
                        ULTRA_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(),
                        ULTRA_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0,
                        0.0);
                assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);

                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                                intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                                intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                                intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                                intrinsic2.getVerticalPrincipalPoint());

                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;

                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                succeededAtLeastOnce = true;
                succeeded++;
            } catch (final InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }

        final double failedRatio = (double) failed / (double) total;
        final double succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0,
                VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0,
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, LARGE_ABSOLUTE_ERROR);

        final String msg = "LMSE, No skewness and Principal point at origin - failed: " +
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 +
                "% avg horizontal focal distance error: " +
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);

        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    public void testEstimateZeroSkewnessAspectRatioKnownAndNoLMSE()
            throws InvalidPinholeCameraIntrinsicParametersException,
            LockedException, NotReadyException, RobustEstimatorException,
            ImageOfAbsoluteConicEstimatorException {

        boolean succeededAtLeastOnce = false;
        int failed = 0;
        int succeeded = 0;
        int total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (int j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final List<Point2D> patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final List<Point3D> points3D = new ArrayList<>();
            for (final Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final List<Transformation2D> homographies =
                    new ArrayList<>();
            for (int i = 0; i < 2; i++) {
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
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                        cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                                createFromPoints(patternPoints, projectedPatternPoints,
                                        RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography =
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final LMSEImageOfAbsoluteConicEstimator estimator =
                    new LMSEImageOfAbsoluteConicEstimator(this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(false);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(1.0);

            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 2);
            estimator.setHomographies(homographies);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            // estimate
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            if (Math.abs(Math.abs(iac.getA()) - Math.abs(iac2.getA())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()),
                    VERY_LARGE_ABSOLUTE_ERROR);

            assertEquals(iac.getB(), 0.0, 0.0);
            assertEquals(iac2.getB(), 0.0, 0.0);

            if (Math.abs(Math.abs(iac.getC()) - Math.abs(iac2.getC())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()),
                    VERY_LARGE_ABSOLUTE_ERROR);

            if (Math.abs(Math.abs(iac.getD()) - Math.abs(iac2.getD())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getD()), Math.abs(iac2.getD()),
                    VERY_LARGE_ABSOLUTE_ERROR);

            if (Math.abs(Math.abs(iac.getE()) - Math.abs(iac2.getE())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getE()), Math.abs(iac2.getE()),
                    VERY_LARGE_ABSOLUTE_ERROR);

            if (Math.abs(Math.abs(iac.getF()) - Math.abs(iac2.getF())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()),
                    VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final PinholeCameraIntrinsicParameters intrinsic2 =
                        iac2.getIntrinsicParameters();

                if (Math.abs(intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength()) >
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalFocalLength(),
                        intrinsic2.getHorizontalFocalLength(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalFocalLength() -
                        intrinsic2.getVerticalFocalLength()) >
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
                if (Math.abs(intrinsic.getHorizontalPrincipalPoint() -
                        intrinsic2.getHorizontalPrincipalPoint()) >
                        2.0 * VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                        intrinsic2.getHorizontalPrincipalPoint(),
                        2.0 * VERY_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalPrincipalPoint() -
                        intrinsic2.getVerticalPrincipalPoint()) >
                        2.0 * VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalPrincipalPoint(),
                        intrinsic2.getVerticalPrincipalPoint(),
                        2.0 * VERY_LARGE_ABSOLUTE_ERROR);

                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                                intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                                intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                                intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                                intrinsic2.getVerticalPrincipalPoint());

                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;

                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                succeededAtLeastOnce = true;
                succeeded++;
            } catch (final InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }

        final double failedRatio = (double) failed / (double) total;
        final double succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);

        final String msg = "NO LMSE, Zero skewness, known aspect ratio - failed: " +
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 +
                "% avg horizontal focal distance error: " +
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);

        assertTrue(failedRatio < 0.6);
        assertTrue(succeededRatio >= 0.4);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    public void testEstimateZeroSkewnessAspectRatioKnownAndLMSE()
            throws InvalidPinholeCameraIntrinsicParametersException,
            LockedException, NotReadyException, RobustEstimatorException,
            ImageOfAbsoluteConicEstimatorException {

        boolean succeededAtLeastOnce = false;
        int failed = 0;
        int succeeded = 0;
        int total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (int j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final List<Point2D> patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final List<Point3D> points3D = new ArrayList<>();
            for (final Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final List<Transformation2D> homographies =
                    new ArrayList<>();
            for (int i = 0; i < 50; i++) {
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
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                        cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                                createFromPoints(patternPoints, projectedPatternPoints,
                                        RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography =
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final LMSEImageOfAbsoluteConicEstimator estimator =
                    new LMSEImageOfAbsoluteConicEstimator(homographies, this);
            estimator.setLMSESolutionAllowed(true);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(false);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(1.0);

            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 2);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            // estimate
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getB(), 0.0, 0.0);
            assertEquals(iac2.getB(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getD()), Math.abs(iac2.getD()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getE()), Math.abs(iac2.getE()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()),
                    VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final PinholeCameraIntrinsicParameters intrinsic2 =
                        iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(),
                        intrinsic2.getHorizontalFocalLength(),
                        2.0 * VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(),
                        2.0 * VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                        intrinsic2.getHorizontalPrincipalPoint(),
                        2.0 * VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalPrincipalPoint(),
                        intrinsic2.getVerticalPrincipalPoint(),
                        2.0 * VERY_LARGE_ABSOLUTE_ERROR);

                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                                intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                                intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                                intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                                intrinsic2.getVerticalPrincipalPoint());

                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;

                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                succeededAtLeastOnce = true;
                succeeded++;
            } catch (final InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }

        final double failedRatio = (double) failed / (double) total;
        final double succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0,
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0,
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, LARGE_ABSOLUTE_ERROR);

        final String msg = "LMSE, Zero skewness, known aspect ratio - failed: " +
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 +
                "% avg horizontal focal distance error: " +
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);

        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    public void testEstimateZeroSkewnessPrincipalPointAtOriginAspectRatioKnownAndNoLMSE()
            throws InvalidPinholeCameraIntrinsicParametersException,
            LockedException, NotReadyException, RobustEstimatorException,
            ImageOfAbsoluteConicEstimatorException {

        int failed = 0;
        int succeeded = 0;
        int total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (int j = 0; j < TIMES; j++) {
            // create intrinsic parameters
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

            final ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final List<Point2D> patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final List<Point3D> points3D = new ArrayList<>();
            for (final Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final List<Transformation2D> homographies =
                    new ArrayList<>();
            for (int i = 0; i < 1; i++) {
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
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                        cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                                createFromPoints(patternPoints, projectedPatternPoints,
                                        RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography =
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final LMSEImageOfAbsoluteConicEstimator estimator =
                    new LMSEImageOfAbsoluteConicEstimator(this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(1.0);

            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
            estimator.setHomographies(homographies);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            // estimate
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()),
                    ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getB(), 0.0, 0.0);
            assertEquals(iac2.getB(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()),
                    ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getD(), 0.0, 0.0);
            assertEquals(iac2.getD(), 0.0, 0.0);
            assertEquals(iac.getE(), 0.0, 0.0);
            assertEquals(iac2.getE(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()),
                    ULTRA_LARGE_ABSOLUTE_ERROR);

            try {
                final PinholeCameraIntrinsicParameters intrinsic2 =
                        iac2.getIntrinsicParameters();

                if (Math.abs(intrinsic.getHorizontalFocalLength() - intrinsic2.getHorizontalFocalLength()) >
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalFocalLength(),
                        intrinsic2.getHorizontalFocalLength(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalFocalLength() - intrinsic2.getVerticalFocalLength()) >
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0,
                        0.0);
                assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);

                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                                intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                                intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                                intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                                intrinsic2.getVerticalPrincipalPoint());

                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;

                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                succeeded++;
            } catch (final InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }

        assertTrue(succeeded > 0);

        final double failedRatio = (double) failed / (double) total;
        final double succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);

        final String msg = "NO LMSE, No skewness, Principal point at origin, known aspect ratio - failed: " +
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 +
                "% avg horizontal focal distance error: " +
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);

        assertTrue(failedRatio < 0.5);
        assertTrue(succeededRatio >= 0.5);
    }

    @Test
    public void testEstimateZeroSkewnessPrincipalPointAtOriginAspectRatioKnownAndLMSE()
            throws InvalidPinholeCameraIntrinsicParametersException,
            LockedException, NotReadyException, RobustEstimatorException,
            ImageOfAbsoluteConicEstimatorException {

        boolean succeededAtLeastOnce = false;
        int failed = 0;
        int succeeded = 0;
        int total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (int j = 0; j < TIMES; j++) {
            // create intrinsic parameters
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

            final ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final List<Point2D> patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final List<Point3D> points3D = new ArrayList<>();
            for (final Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final List<Transformation2D> homographies =
                    new ArrayList<>();
            for (int i = 0; i < 50; i++) {
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
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation,
                        cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                                createFromPoints(patternPoints, projectedPatternPoints,
                                        RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography =
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final LMSEImageOfAbsoluteConicEstimator estimator =
                    new LMSEImageOfAbsoluteConicEstimator(homographies, this);
            estimator.setLMSESolutionAllowed(true);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(1.0);

            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            // estimate
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getB(), 0.0, 0.0);
            assertEquals(iac2.getB(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getD(), 0.0, 0.0);
            assertEquals(iac2.getD(), 0.0, 0.0);
            assertEquals(iac.getE(), 0.0, 0.0);
            assertEquals(iac2.getE(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()),
                    VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final PinholeCameraIntrinsicParameters intrinsic2 =
                        iac2.getIntrinsicParameters();

                if (Math.abs(intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength()) > ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalFocalLength(),
                        intrinsic2.getHorizontalFocalLength(),
                        ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalFocalLength() -
                        intrinsic2.getVerticalFocalLength()) > ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(),
                        ULTRA_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0,
                        0.0);
                assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);

                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                                intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                                intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                                intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                                intrinsic2.getVerticalPrincipalPoint());

                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;

                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                succeededAtLeastOnce = true;
                succeeded++;
            } catch (final InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }

        final double failedRatio = (double) failed / (double) total;
        final double succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0,
                VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0,
                VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);

        final String msg = "LMSE, No skewness, Principal point at origin, known aspect ratio - failed: " +
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 +
                "% avg horizontal focal distance error: " +
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);

        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    public void testEstimateRealData() throws CoincidentPointsException,
            LockedException, NotReadyException,
            ImageOfAbsoluteConicEstimatorException,
            InvalidPinholeCameraIntrinsicParametersException {
        // For a QR pattern, assuming zero skewness, equal focal lengths and
        // principal point at origin on a nexus 5 device
        final Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
        final List<Point2D> patternPoints = pattern.getIdealPoints();
                
        /*
        Sampled data (before and after centering coordinates and setting correct y axis 
        direction)
        Point[0] = 382.5, 701.5 | -385.5, 322.5
        Point[1] = 351.0, 473.5 | -417.0, 550.5
        Point[2] = 585.0, 451.5 | -183.0, 572.5
        Point[3] = 592.0, 653.5 | -176.0, 370.5
        */
        final List<Point2D> sampledPoints = new ArrayList<>();
        sampledPoints.add(new InhomogeneousPoint2D(-385.5, 322.5));
        sampledPoints.add(new InhomogeneousPoint2D(-417.0, 550.5));
        sampledPoints.add(new InhomogeneousPoint2D(-183.0, 572.5));
        sampledPoints.add(new InhomogeneousPoint2D(-176.0, 370.5));


        // obtain homography
        final ProjectiveTransformation2D homography = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1),
                patternPoints.get(2), patternPoints.get(3),
                sampledPoints.get(0), sampledPoints.get(1),
                sampledPoints.get(2), sampledPoints.get(3));

        final List<Transformation2D> homographies = new ArrayList<>();
        homographies.add(homography);

        // estimate IAC
        final LMSEImageOfAbsoluteConicEstimator estimator =
                new LMSEImageOfAbsoluteConicEstimator(homographies);

        assertFalse(estimator.isLMSESolutionAllowed());
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);

        final ImageOfAbsoluteConic iac = estimator.estimate();

        assertNotNull(iac);

        // obtain intrinsic parameters
        final PinholeCameraIntrinsicParameters intrinsic =
                iac.getIntrinsicParameters();

        assertNotNull(intrinsic);

        final double horizontalFocalLength = intrinsic.getHorizontalFocalLength();
        final double verticalFocalLength = intrinsic.getVerticalFocalLength();
        final double skewness = intrinsic.getSkewness();
        final double horizontalPrincipalPoint = intrinsic.getHorizontalPrincipalPoint();
        final double verticalPrincipalPoint = intrinsic.getVerticalPrincipalPoint();
        assertTrue(horizontalFocalLength > 0);
        assertTrue(verticalFocalLength > 0);
        assertEquals(horizontalFocalLength, verticalFocalLength,
                ABSOLUTE_ERROR);
        assertEquals(skewness, 0.0, 0.0);
        assertEquals(horizontalPrincipalPoint, 0.0, 0.0);
        assertEquals(verticalPrincipalPoint, 0.0, 0.0);

        final String msg = "Real data focal length: " + horizontalFocalLength;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);
    }

    @Override
    public void onEstimateStart(final ImageOfAbsoluteConicEstimator estimator) {
        estimateStart++;
        testLocked((LMSEImageOfAbsoluteConicEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final ImageOfAbsoluteConicEstimator estimator) {
        estimateEnd++;
        testLocked((LMSEImageOfAbsoluteConicEstimator) estimator);
    }

    @Override
    public void onEstimationProgressChange(
            final ImageOfAbsoluteConicEstimator estimator, final float progress) {
        estimationProgressChange++;
        testLocked((LMSEImageOfAbsoluteConicEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimationProgressChange = 0;
    }

    private void testLocked(final LMSEImageOfAbsoluteConicEstimator estimator) {
        try {
            estimator.setZeroSkewness(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPrincipalPointAtOrigin(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setHomographies(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setLMSESolutionAllowed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }
}
