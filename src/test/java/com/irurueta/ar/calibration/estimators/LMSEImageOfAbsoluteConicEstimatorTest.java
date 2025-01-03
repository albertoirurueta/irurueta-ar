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
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.jupiter.api.Assertions.*;

class LMSEImageOfAbsoluteConicEstimatorTest implements ImageOfAbsoluteConicEstimatorListener {

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
    void testConstructor() {
        // test constructor without arguments
        var estimator = new LMSEImageOfAbsoluteConicEstimator();

        // check default values
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertNull(estimator.getHomographies());
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
        assertFalse(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR, estimator.getType());
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_ALLOW_LMSE_SOLUTION, estimator.isLMSESolutionAllowed());

        // test constructor with listener
        estimator = new LMSEImageOfAbsoluteConicEstimator(this);

        // check default values
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getHomographies());
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
        assertFalse(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR, estimator.getType());
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_ALLOW_LMSE_SOLUTION, estimator.isLMSESolutionAllowed());

        // test constructor with homographies
        final var homographies = new ArrayList<Transformation2D>();
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());

        estimator = new LMSEImageOfAbsoluteConicEstimator(homographies);

        // check default values
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertSame(homographies, estimator.getHomographies());
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
        assertTrue(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR, estimator.getType());
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_ALLOW_LMSE_SOLUTION, estimator.isLMSESolutionAllowed());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new LMSEImageOfAbsoluteConicEstimator((List<Transformation2D>) null));

        final var emptyHomographies = new ArrayList<Transformation2D>();
        assertThrows(IllegalArgumentException.class, () -> new LMSEImageOfAbsoluteConicEstimator(emptyHomographies));

        // test constructor with homographies and listener
        estimator = new LMSEImageOfAbsoluteConicEstimator(homographies, this);

        // check default values
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertSame(this, estimator.getListener());
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
        assertTrue(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR, estimator.getType());
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_ALLOW_LMSE_SOLUTION, estimator.isLMSESolutionAllowed());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new LMSEImageOfAbsoluteConicEstimator((List<Transformation2D>) null));
        assertThrows(IllegalArgumentException.class, () -> new LMSEImageOfAbsoluteConicEstimator(emptyHomographies));
    }

    @Test
    void testIsSetZeroSkewness() throws LockedException {
        final var estimator = new LMSEImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());

        // set new value
        estimator.setZeroSkewness(!LMSEImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);

        // check correctness
        assertEquals(!LMSEImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
    }

    @Test
    void testIsSetPrincipalPointAtOrigin() throws LockedException {
        final var estimator = new LMSEImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());

        // set new value
        estimator.setPrincipalPointAtOrigin(!LMSEImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);

        // check correctness
        assertEquals(!LMSEImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
    }

    @Test
    void testIsFocalDistanceAspectRatioKnown() throws LockedException {
        final var estimator = new LMSEImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());

        // set new value
        estimator.setFocalDistanceAspectRatioKnown(
                !LMSEImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
    }

    @Test
    void testGetSetFocalDistanceAspectRatio() throws LockedException {
        final var estimator = new LMSEImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);

        // set new value
        estimator.setFocalDistanceAspectRatio(0.5);

        // check correctness
        assertEquals(0.5, estimator.getFocalDistanceAspectRatio(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setFocalDistanceAspectRatio(0.0));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new LMSEImageOfAbsoluteConicEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetHomographiesAndIsReady() throws LockedException {
        final var estimator = new LMSEImageOfAbsoluteConicEstimator();

        // check default value
        assertNull(estimator.getHomographies());
        assertFalse(estimator.isReady());

        // set new value
        final var homographies = new ArrayList<Transformation2D>();
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());

        estimator.setHomographies(homographies);

        // check correctness
        assertSame(homographies, estimator.getHomographies());
        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setHomographies(null));

        final var emptyHomographies = new ArrayList<Transformation2D>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setHomographies(emptyHomographies));
    }

    @Test
    void testIsSetLMSESolutionAllowed() throws LockedException {
        final var estimator = new LMSEImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(LMSEImageOfAbsoluteConicEstimator.DEFAULT_ALLOW_LMSE_SOLUTION, estimator.isLMSESolutionAllowed());

        // set new value
        estimator.setLMSESolutionAllowed(!LMSEImageOfAbsoluteConicEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);

        // check correctness
        assertEquals(!LMSEImageOfAbsoluteConicEstimator.DEFAULT_ALLOW_LMSE_SOLUTION, estimator.isLMSESolutionAllowed());
    }

    @Test
    void testEstimateNoConstraintsAndNoLMSE() throws InvalidPinholeCameraIntrinsicParametersException, LockedException,
            NotReadyException, RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

        var succeededAtLeastOnce = false;
        var failed = 0;
        var succeeded = 0;
        var total = 0;
        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final var patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final var points3D = new ArrayList<Point3D>();
            for (final var patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                        1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final var homographies = new ArrayList<Transformation2D>();
            for (var i = 0; i < 3; i++) {
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final var projectedPatternPoints = camera.project(points3D);

                final var homographyEstimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                        patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                final var homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final var estimator = new LMSEImageOfAbsoluteConicEstimator(homographies, this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setZeroSkewness(false);
            estimator.setPrincipalPointAtOrigin(false);
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(3, estimator.getMinNumberOfRequiredHomographies());

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimationProgressChange);

            // estimate
            final var iac2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            if (Math.abs(Math.abs(iac.getA()) - Math.abs(iac2.getA())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(iac.getB()) - Math.abs(iac2.getB())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getB()), Math.abs(iac2.getB()), VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(iac.getC()) - Math.abs(iac2.getC())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(iac.getD()) - Math.abs(iac2.getD())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getD()), Math.abs(iac2.getD()), VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(iac.getE()) - Math.abs(iac2.getE())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getE()), Math.abs(iac2.getE()), VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(iac.getF()) - Math.abs(iac2.getF())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final var intrinsic2 = iac2.getIntrinsicParameters();

                if (Math.abs(intrinsic.getHorizontalFocalLength() - intrinsic2.getHorizontalFocalLength())
                        > 2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalFocalLength() - intrinsic2.getVerticalFocalLength())
                        > 2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness()) > VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), VERY_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getHorizontalPrincipalPoint() - intrinsic2.getHorizontalPrincipalPoint())
                        > 2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalPrincipalPoint() - intrinsic2.getVerticalPrincipalPoint())
                        > 2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                        - intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                        - intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                        - intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                        - intrinsic2.getVerticalPrincipalPoint());

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

        final var failedRatio = (double) failed / (double) total;
        final var succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgHorizontalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);

        final var msg = "NO LMSE, No constraints - failed: "
                + failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0
                + "% avg horizontal focal distance error: "
                + avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: "
                + avgVerticalFocalDistanceError + " avg skewness error: "
                + avgSkewnessError + " horizontal principal point error: "
                + avgHorizontalPrincipalPointError
                + " vertical principal point error: "
                + avgVerticalPrincipalPointError
                + " min horizontal focal distance error: "
                + minHorizontalFocalDistanceError
                + " min vertical focal distance error: "
                + minVerticalFocalDistanceError + " min skewness error: "
                + minSkewnessError + " min horizontal principal point error: "
                + minHorizontalPrincipalPointError
                + " min vertical principal point error: "
                + minVerticalPrincipalPointError
                + " max horizontal focal distance error: "
                + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: "
                + maxVerticalFocalDistanceError + " max skewness error: "
                + maxSkewnessError + " max horizontal principal point error: "
                + maxHorizontalPrincipalPointError
                + " max vertical principal point error: "
                + maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).log(Level.INFO, msg);

        assertTrue(failedRatio < 0.6);
        assertTrue(succeededRatio >= 0.4);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    void testEstimateNoConstraintsAndLMSE() throws InvalidPinholeCameraIntrinsicParametersException, LockedException,
            NotReadyException, RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

        var succeededAtLeastOnce = false;
        var failed = 0;
        var succeeded = 0;
        var total = 0;
        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final var patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final var points3D = new ArrayList<Point3D>();
            for (final var patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                        1.0));
            }

            // create 50 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 50 homographies to
            // estimate the IAC
            final var homographies = new ArrayList<Transformation2D>();
            for (var i = 0; i < 50; i++) {
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final var projectedPatternPoints = camera.project(points3D);

                final var homographyEstimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                        patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                final var homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final var estimator = new LMSEImageOfAbsoluteConicEstimator(homographies, this);
            estimator.setLMSESolutionAllowed(true);
            estimator.setZeroSkewness(false);
            estimator.setPrincipalPointAtOrigin(false);
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(3, estimator.getMinNumberOfRequiredHomographies());

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimationProgressChange);

            // estimate
            final var iac2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getB()), Math.abs(iac2.getB()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getD()), Math.abs(iac2.getD()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getE()), Math.abs(iac2.getE()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final var intrinsic2 = iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                        - intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                        - intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                        - intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                        - intrinsic2.getVerticalPrincipalPoint());

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

        final var failedRatio = (double) failed / (double) total;
        final var succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgHorizontalFocalDistanceError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, LARGE_ABSOLUTE_ERROR);

        final var msg = "LMSE, No constraints - failed: "
                + failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0
                + "% avg horizontal focal distance error: "
                + avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: "
                + avgVerticalFocalDistanceError + " avg skewness error: "
                + avgSkewnessError + " avg horizontal principal point error: "
                + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: "
                + avgVerticalPrincipalPointError
                + " min horizontal focal distance error: "
                + minHorizontalFocalDistanceError
                + " min vertical focal distance error: "
                + minVerticalFocalDistanceError + " min skewness error: "
                + minSkewnessError + " min horizontal principal point error: "
                + minHorizontalPrincipalPointError
                + " min vertical principal point error: "
                + minVerticalPrincipalPointError
                + " max horizontal focal distance error: "
                + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: "
                + maxVerticalFocalDistanceError + " max skewness error: "
                + maxSkewnessError + " max horizontal principal point error: "
                + maxHorizontalPrincipalPointError
                + " max vertical principal point error: "
                + maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).log(Level.INFO, msg);

        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    void testEstimateZeroSkewnessAndNoLMSE() throws InvalidPinholeCameraIntrinsicParametersException, LockedException,
            NotReadyException, RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

        var succeededAtLeastOnce = false;
        var failed = 0;
        var succeeded = 0;
        var total = 0;
        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final var patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final var points3D = new ArrayList<Point3D>();
            for (final var patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                        1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final var homographies = new ArrayList<Transformation2D>();
            for (var i = 0; i < 2; i++) {
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final var projectedPatternPoints = camera.project(points3D);

                final var homographyEstimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                        patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                final var homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final var estimator = new LMSEImageOfAbsoluteConicEstimator(this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(false);
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(2, estimator.getMinNumberOfRequiredHomographies());
            estimator.setHomographies(homographies);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimationProgressChange);

            // estimate
            final var iac2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, iac.getB(), 0.0);
            assertEquals(0.0, iac2.getB(), 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getD()), Math.abs(iac2.getD()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getE()), Math.abs(iac2.getE()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final var intrinsic2 = iac2.getIntrinsicParameters();

                assertEquals(0.0, intrinsic.getSkewness(), 0.0);
                assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
                if (Math.abs(intrinsic.getHorizontalPrincipalPoint() - intrinsic2.getHorizontalPrincipalPoint())
                        > ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(),
                        ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalPrincipalPoint() - intrinsic2.getHorizontalPrincipalPoint())
                        > ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(),
                        ULTRA_LARGE_ABSOLUTE_ERROR);

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                        - intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                        - intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                        - intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                        - intrinsic2.getVerticalPrincipalPoint());

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

        final var failedRatio = (double) failed / (double) total;
        final var succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgHorizontalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);

        final var msg = "NO LMSE, Zero skewness - failed: "
                + failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0
                + "% avg horizontal focal distance error: " + avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).log(Level.INFO, msg);

        assertTrue(failedRatio < 0.6);
        assertTrue(succeededRatio >= 0.4);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    void testEstimateZeroSkewnessAndLMSE() throws InvalidPinholeCameraIntrinsicParametersException, LockedException,
            NotReadyException, RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

        var succeededAtLeastOnce = false;
        var failed = 0;
        var succeeded = 0;
        var total = 0;
        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final var patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final var points3D = new ArrayList<Point3D>();
            for (final var patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                        1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final var homographies = new ArrayList<Transformation2D>();
            for (var i = 0; i < 50; i++) {
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final var projectedPatternPoints = camera.project(points3D);

                final var homographyEstimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                        patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                final var homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final var estimator = new LMSEImageOfAbsoluteConicEstimator(homographies, this);
            estimator.setLMSESolutionAllowed(true);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(false);
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(2, estimator.getMinNumberOfRequiredHomographies());

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimationProgressChange);

            // estimate
            final var iac2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, iac.getB(), 0.0);
            assertEquals(0.0, iac2.getB(), 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getD()), Math.abs(iac2.getD()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getE()), Math.abs(iac2.getE()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final var intrinsic2 = iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(0.0, intrinsic.getSkewness(), 0.0);
                assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                        - intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                        - intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                        - intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                        - intrinsic2.getVerticalPrincipalPoint());

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

        final var failedRatio = (double) failed / (double) total;
        final var succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgHorizontalFocalDistanceError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, LARGE_ABSOLUTE_ERROR);

        final var msg = "LMSE, Zero skewness - failed: " + failedRatio * 100.0
                + "% succeeded: " + succeededRatio * 100.0
                + "% avg horizontal focal distance error: " + avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).log(Level.INFO, msg);

        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    void testEstimatePrincipalPointAtOriginAndNoLMSE() throws InvalidPinholeCameraIntrinsicParametersException,
            LockedException, NotReadyException, RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

        var succeededAtLeastOnce = false;
        var failed = 0;
        var succeeded = 0;
        var total = 0;
        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final var patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final var points3D = new ArrayList<Point3D>();
            for (final var patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                        1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final var homographies = new ArrayList<Transformation2D>();
            for (var i = 0; i < 2; i++) {
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final var projectedPatternPoints = camera.project(points3D);

                final var homographyEstimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                        patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                final var homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final var estimator = new LMSEImageOfAbsoluteConicEstimator(this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setZeroSkewness(false);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(2, estimator.getMinNumberOfRequiredHomographies());
            estimator.setHomographies(homographies);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimationProgressChange);

            // estimate
            final var iac2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            if (Math.abs(Math.abs(iac.getA()) - Math.abs(iac2.getA())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(iac.getB()) - Math.abs(iac2.getB())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getB()), Math.abs(iac2.getB()), VERY_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(iac.getC()) - Math.abs(iac2.getC())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, iac.getD(), 0.0);
            assertEquals(0.0, iac2.getD(), 0.0);
            assertEquals(0.0, iac.getE(), 0.0);
            assertEquals(0.0, iac2.getE(), 0.0);
            if (Math.abs(Math.abs(iac.getF()) - Math.abs(iac2.getF())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final var intrinsic2 = iac2.getIntrinsicParameters();

                if (Math.abs(intrinsic.getHorizontalFocalLength() - intrinsic2.getHorizontalFocalLength())
                        > 3 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                        3 * ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalFocalLength() - intrinsic2.getVerticalFocalLength())
                        > 3 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                        3 * ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness()) > VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic2.getHorizontalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic2.getVerticalPrincipalPoint(), 0.0);

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                        - intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                        - intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                        - intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                        - intrinsic2.getVerticalPrincipalPoint());

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

        final var failedRatio = (double) failed / (double) total;
        final var succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgHorizontalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);

        final var msg = "NO LMSE, Principal point at origin - failed: " + failedRatio * 100.0
                + "% succeeded: " + succeededRatio * 100.0
                + "% avg horizontal focal distance error: " + avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).log(Level.INFO, msg);

        assertTrue(failedRatio < 0.5);
        assertTrue(succeededRatio >= 0.5);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    void testEstimatePrincipalPointAtOriginAndLMSE() throws InvalidPinholeCameraIntrinsicParametersException,
            LockedException, NotReadyException, RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

        var succeededAtLeastOnce = false;
        var failed = 0;
        var succeeded = 0;
        var total = 0;
        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final var patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final var points3D = new ArrayList<Point3D>();
            for (final var patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                        1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final var homographies = new ArrayList<Transformation2D>();
            for (var i = 0; i < 50; i++) {
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final var projectedPatternPoints = camera.project(points3D);

                final var homographyEstimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                        patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                final var homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final var estimator = new LMSEImageOfAbsoluteConicEstimator(homographies, this);
            estimator.setLMSESolutionAllowed(true);
            estimator.setZeroSkewness(false);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(2, estimator.getMinNumberOfRequiredHomographies());

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimationProgressChange);

            // estimate
            final var iac2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getB()), Math.abs(iac2.getB()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, iac.getD(), 0.0);
            assertEquals(0.0, iac2.getD(), 0.0);
            assertEquals(0.0, iac.getE(), 0.0);
            assertEquals(0.0, iac2.getE(), 0.0);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final var intrinsic2 = iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic2.getHorizontalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic2.getVerticalPrincipalPoint(), 0.0);

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                        - intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                        - intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                        - intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                        - intrinsic2.getVerticalPrincipalPoint());

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

        final var failedRatio = (double) failed / (double) total;
        final var succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgHorizontalFocalDistanceError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, LARGE_ABSOLUTE_ERROR);

        final var msg = "LMSE, Principal point at origin - failed: " + failedRatio * 100.0
                + "% succeeded: " + succeededRatio * 100.0
                + "% avg horizontal focal distance error: " + avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).log(Level.INFO, msg);

        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    void testEstimateZeroSkewnessPrincipalPointAtOriginAndNoLMSE()
            throws InvalidPinholeCameraIntrinsicParametersException, LockedException, NotReadyException,
            RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

        var succeededAtLeastOnce = false;
        var failed = 0;
        var succeeded = 0;
        var total = 0;
        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final var patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final var points3D = new ArrayList<Point3D>();
            for (final var patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                        1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final var homographies = new ArrayList<Transformation2D>();
            for (var i = 0; i < 1; i++) {
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final var projectedPatternPoints = camera.project(points3D);

                final var homographyEstimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                        patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                final var homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final var estimator = new LMSEImageOfAbsoluteConicEstimator(this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
            estimator.setHomographies(homographies);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimationProgressChange);

            // estimate
            final var iac2 = estimator.estimate();

            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, iac.getB(), 0.0);
            assertEquals(0.0, iac2.getB(), 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, iac.getD(), 0.0);
            assertEquals(0.0, iac2.getD(), 0.0);
            assertEquals(0.0, iac.getE(), 0.0);
            assertEquals(0.0, iac2.getE(), 0.0);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), ULTRA_LARGE_ABSOLUTE_ERROR);

            try {
                final var intrinsic2 = iac2.getIntrinsicParameters();

                if (Math.abs(intrinsic.getHorizontalFocalLength() - intrinsic2.getHorizontalFocalLength())
                        > 2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalFocalLength() - intrinsic2.getVerticalFocalLength())
                        > 2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                assertEquals(0.0, intrinsic.getSkewness(), 0.0);
                assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
                assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic2.getHorizontalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic2.getVerticalPrincipalPoint(), 0.0);

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                        - intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                        - intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                        - intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                        - intrinsic2.getVerticalPrincipalPoint());

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

        final var failedRatio = (double) failed / (double) total;
        final var succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgHorizontalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);

        final var msg = "NO LMSE, No skewness and Principal point at origin - failed: " + failedRatio * 100.0
                + "% succeeded: " + succeededRatio * 100.0
                + "% avg horizontal focal distance error: " + avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).log(Level.INFO, msg);

        assertTrue(failedRatio < 0.5);
        assertTrue(succeededRatio >= 0.5);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    void testEstimateZeroSkewnessPrincipalPointAtOriginAndLMSE()
            throws InvalidPinholeCameraIntrinsicParametersException, LockedException, NotReadyException,
            RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

        var succeededAtLeastOnce = false;
        var failed = 0;
        var succeeded = 0;
        var total = 0;
        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final var patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final var points3D = new ArrayList<Point3D>();
            for (final var patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                        1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final var homographies = new ArrayList<Transformation2D>();
            for (var i = 0; i < 50; i++) {
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final var projectedPatternPoints = camera.project(points3D);

                final var homographyEstimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                        patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                final var homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final var estimator = new LMSEImageOfAbsoluteConicEstimator(homographies, this);
            estimator.setLMSESolutionAllowed(true);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(1, estimator.getMinNumberOfRequiredHomographies());

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimationProgressChange);

            // estimate
            final var iac2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, iac.getB(), 0.0);
            assertEquals(0.0, iac2.getB(), 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, iac.getD(), 0.0);
            assertEquals(0.0, iac2.getD(), 0.0);
            assertEquals(0.0, iac.getE(), 0.0);
            assertEquals(0.0, iac2.getE(), 0.0);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final var intrinsic2 = iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                        ULTRA_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                        ULTRA_LARGE_ABSOLUTE_ERROR);
                assertEquals(0.0, intrinsic.getSkewness(), 0.0);
                assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
                assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic2.getHorizontalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic2.getVerticalPrincipalPoint(), 0.0);

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                        - intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                        - intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                        - intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                        - intrinsic2.getVerticalPrincipalPoint());

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

        final var failedRatio = (double) failed / (double) total;
        final var succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgHorizontalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, LARGE_ABSOLUTE_ERROR);

        final var msg = "LMSE, No skewness and Principal point at origin - failed: " + failedRatio * 100.0
                + "% succeeded: " + succeededRatio * 100.0
                + "% avg horizontal focal distance error: " + avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError +
                " min vertical principal point error: " + minVerticalPrincipalPointError +
                " max horizontal focal distance error: " + maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).log(Level.INFO, msg);

        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    void testEstimateZeroSkewnessAspectRatioKnownAndNoLMSE() throws InvalidPinholeCameraIntrinsicParametersException,
            LockedException, NotReadyException, RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

        var succeededAtLeastOnce = false;
        var failed = 0;
        var succeeded = 0;
        var total = 0;
        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final var patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final var points3D = new ArrayList<Point3D>();
            for (final var patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                        1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final var homographies = new ArrayList<Transformation2D>();
            for (var i = 0; i < 2; i++) {
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final var projectedPatternPoints = camera.project(points3D);

                final var homographyEstimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                        patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                final var homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final var estimator = new LMSEImageOfAbsoluteConicEstimator(this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(false);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(1.0);

            assertEquals(2, estimator.getMinNumberOfRequiredHomographies());
            estimator.setHomographies(homographies);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimationProgressChange);

            // estimate
            final var iac2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            if (Math.abs(Math.abs(iac.getA()) - Math.abs(iac2.getA())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), VERY_LARGE_ABSOLUTE_ERROR);

            assertEquals(0.0, iac.getB(), 0.0);
            assertEquals(0.0, iac2.getB(), 0.0);

            if (Math.abs(Math.abs(iac.getC()) - Math.abs(iac2.getC())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), VERY_LARGE_ABSOLUTE_ERROR);

            if (Math.abs(Math.abs(iac.getD()) - Math.abs(iac2.getD())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getD()), Math.abs(iac2.getD()), VERY_LARGE_ABSOLUTE_ERROR);

            if (Math.abs(Math.abs(iac.getE()) - Math.abs(iac2.getE())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getE()), Math.abs(iac2.getE()), VERY_LARGE_ABSOLUTE_ERROR);

            if (Math.abs(Math.abs(iac.getF()) - Math.abs(iac2.getF())) > VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final var intrinsic2 = iac2.getIntrinsicParameters();

                if (Math.abs(intrinsic.getHorizontalFocalLength() - intrinsic2.getHorizontalFocalLength())
                        > 2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalFocalLength() - intrinsic2.getVerticalFocalLength())
                        > 2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                assertEquals(0.0, intrinsic.getSkewness(), 0.0);
                assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
                if (Math.abs(intrinsic.getHorizontalPrincipalPoint() - intrinsic2.getHorizontalPrincipalPoint())
                        > 2.0 * VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(),
                        2.0 * VERY_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalPrincipalPoint() - intrinsic2.getVerticalPrincipalPoint())
                        > 2.0 * VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(),
                        2.0 * VERY_LARGE_ABSOLUTE_ERROR);

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                        - intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                        - intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                        - intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                        - intrinsic2.getVerticalPrincipalPoint());

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

        final var failedRatio = (double) failed / (double) total;
        final var succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgHorizontalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);

        final var msg = "NO LMSE, Zero skewness, known aspect ratio - failed: " + failedRatio * 100.0
                + "% succeeded: " + succeededRatio * 100.0
                + "% avg horizontal focal distance error: " + avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError +
                " max vertical principal point error: " + maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).log(Level.INFO, msg);

        assertTrue(failedRatio < 0.6);
        assertTrue(succeededRatio >= 0.4);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    void testEstimateZeroSkewnessAspectRatioKnownAndLMSE() throws InvalidPinholeCameraIntrinsicParametersException,
            LockedException, NotReadyException, RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

        var succeededAtLeastOnce = false;
        var failed = 0;
        var succeeded = 0;
        var total = 0;
        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final var patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final var points3D = new ArrayList<Point3D>();
            for (final var patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                        1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final var homographies = new ArrayList<Transformation2D>();
            for (var i = 0; i < 50; i++) {
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final var projectedPatternPoints = camera.project(points3D);

                final var homographyEstimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                        patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                final var homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final var estimator = new LMSEImageOfAbsoluteConicEstimator(homographies, this);
            estimator.setLMSESolutionAllowed(true);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(false);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(1.0);

            assertEquals(2, estimator.getMinNumberOfRequiredHomographies());

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimationProgressChange);

            // estimate
            final var iac2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, iac.getB(), 0.0);
            assertEquals(0.0, iac2.getB(), 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getD()), Math.abs(iac2.getD()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getE()), Math.abs(iac2.getE()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final var intrinsic2 = iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                        2.0 * VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                        2.0 * VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(0.0, intrinsic.getSkewness(), 0.0);
                assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(),
                        2.0 * VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(),
                        2.0 * VERY_LARGE_ABSOLUTE_ERROR);

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                        - intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                        - intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                        - intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                        - intrinsic2.getVerticalPrincipalPoint());

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

        final var failedRatio = (double) failed / (double) total;
        final var succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgHorizontalFocalDistanceError, 2.0 * LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, 2.0 * LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, LARGE_ABSOLUTE_ERROR);

        final var msg = "LMSE, Zero skewness, known aspect ratio - failed: " + failedRatio * 100.0
                + "% succeeded: " + succeededRatio * 100.0
                + "% avg horizontal focal distance error: " + avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).log(Level.INFO, msg);

        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    void testEstimateZeroSkewnessPrincipalPointAtOriginAspectRatioKnownAndNoLMSE()
            throws InvalidPinholeCameraIntrinsicParametersException, LockedException, NotReadyException,
            RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

        var failed = 0;
        var succeeded = 0;
        var total = 0;
        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final var patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final var points3D = new ArrayList<Point3D>();
            for (final var patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                        1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final var homographies = new ArrayList<Transformation2D>();
            for (var i = 0; i < 1; i++) {
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final var projectedPatternPoints = camera.project(points3D);

                final var homographyEstimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                        patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                final var homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final var estimator = new LMSEImageOfAbsoluteConicEstimator(this);
            estimator.setLMSESolutionAllowed(false);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(1.0);

            assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
            estimator.setHomographies(homographies);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimationProgressChange);

            // estimate
            final var iac2 = estimator.estimate();

            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, iac.getB(), 0.0);
            assertEquals(0.0, iac2.getB(), 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, iac.getD(), 0.0);
            assertEquals(0.0, iac2.getD(), 0.0);
            assertEquals(0.0, iac.getE(), 0.0);
            assertEquals(0.0, iac2.getE(), 0.0);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), ULTRA_LARGE_ABSOLUTE_ERROR);

            try {
                final var intrinsic2 = iac2.getIntrinsicParameters();

                if (Math.abs(intrinsic.getHorizontalFocalLength() - intrinsic2.getHorizontalFocalLength())
                        > 2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalFocalLength() - intrinsic2.getVerticalFocalLength())
                        > 2.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                        2.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                assertEquals(0.0, intrinsic.getSkewness(), 0.0);
                assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
                assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic2.getHorizontalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic2.getVerticalPrincipalPoint(), 0.0);

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                        - intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                        - intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                        - intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                        - intrinsic2.getVerticalPrincipalPoint());

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

        final var failedRatio = (double) failed / (double) total;
        final var succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgHorizontalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);

        final var msg = "NO LMSE, No skewness, Principal point at origin, known aspect ratio - failed: "
                + failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0
                + "% avg horizontal focal distance error: " + avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).log(Level.INFO, msg);

        assertTrue(failedRatio < 0.5);
        assertTrue(succeededRatio >= 0.5);
    }

    @Test
    void testEstimateZeroSkewnessPrincipalPointAtOriginAspectRatioKnownAndLMSE()
            throws InvalidPinholeCameraIntrinsicParametersException, LockedException, NotReadyException,
            RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

        var succeededAtLeastOnce = false;
        var failed = 0;
        var succeeded = 0;
        var total = 0;
        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var iac = new ImageOfAbsoluteConic(intrinsic);

            // create pattern to estimate homography
            final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            final var patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final var points3D = new ArrayList<Point3D>();
            for (final var patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                        1.0));
            }

            // create 3 random cameras having random rotation and translation but
            // created intrinsic parameters in order to obtain 3 homographies to
            // estimate the IAC
            final var homographies = new ArrayList<Transformation2D>();
            for (var i = 0; i < 50; i++) {
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final var projectedPatternPoints = camera.project(points3D);

                final var homographyEstimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(
                        patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                final var homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final var estimator = new LMSEImageOfAbsoluteConicEstimator(homographies, this);
            estimator.setLMSESolutionAllowed(true);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(1.0);

            assertEquals(1, estimator.getMinNumberOfRequiredHomographies());

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimationProgressChange);

            // estimate
            final var iac2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimationProgressChange >= 0);
            reset();

            // check that estimated iac corresponds to the initial one (up to
            // scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, iac.getB(), 0.0);
            assertEquals(0.0, iac2.getB(), 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, iac.getD(), 0.0);
            assertEquals(0.0, iac2.getD(), 0.0);
            assertEquals(0.0, iac.getE(), 0.0);
            assertEquals(0.0, iac2.getE(), 0.0);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), VERY_LARGE_ABSOLUTE_ERROR);

            try {
                final var intrinsic2 = iac2.getIntrinsicParameters();

                if (Math.abs(intrinsic.getHorizontalFocalLength() - intrinsic2.getHorizontalFocalLength())
                        > ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                        ULTRA_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalFocalLength() - intrinsic2.getVerticalFocalLength())
                        > ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                        ULTRA_LARGE_ABSOLUTE_ERROR);
                assertEquals(0.0, intrinsic.getSkewness(), 0.0);
                assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
                assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic2.getHorizontalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic2.getVerticalPrincipalPoint(), 0.0);

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                        - intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                        - intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                        - intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                        - intrinsic2.getVerticalPrincipalPoint());

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

        final var failedRatio = (double) failed / (double) total;
        final var succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= succeeded;
        avgVerticalFocalDistanceError /= succeeded;
        avgSkewnessError /= succeeded;
        avgHorizontalPrincipalPointError /= succeeded;
        avgVerticalPrincipalPointError /= succeeded;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgHorizontalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);

        final var msg = "LMSE, No skewness, Principal point at origin, known aspect ratio - failed: "
                + failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0
                + "% avg horizontal focal distance error: " + avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).log(Level.INFO, msg);

        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    void testEstimateRealData() throws CoincidentPointsException, LockedException, NotReadyException,
            ImageOfAbsoluteConicEstimatorException, InvalidPinholeCameraIntrinsicParametersException {
        // For a QR pattern, assuming zero skewness, equal focal lengths and
        // principal point at origin on a nexus 5 device
        final var pattern = Pattern2D.create(Pattern2DType.QR);
        final var patternPoints = pattern.getIdealPoints();
                
        /*
        Sampled data (before and after centering coordinates and setting correct y-axis
        direction)
        Point[0] = 382.5, 701.5 | -385.5, 322.5
        Point[1] = 351.0, 473.5 | -417.0, 550.5
        Point[2] = 585.0, 451.5 | -183.0, 572.5
        Point[3] = 592.0, 653.5 | -176.0, 370.5
        */
        final var sampledPoints = new ArrayList<Point2D>();
        sampledPoints.add(new InhomogeneousPoint2D(-385.5, 322.5));
        sampledPoints.add(new InhomogeneousPoint2D(-417.0, 550.5));
        sampledPoints.add(new InhomogeneousPoint2D(-183.0, 572.5));
        sampledPoints.add(new InhomogeneousPoint2D(-176.0, 370.5));


        // obtain homography
        final var homography = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1),
                patternPoints.get(2), patternPoints.get(3),
                sampledPoints.get(0), sampledPoints.get(1),
                sampledPoints.get(2), sampledPoints.get(3));

        final var homographies = new ArrayList<Transformation2D>();
        homographies.add(homography);

        // estimate IAC
        final var estimator = new LMSEImageOfAbsoluteConicEstimator(homographies);

        assertFalse(estimator.isLMSESolutionAllowed());
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);

        final var iac = estimator.estimate();

        assertNotNull(iac);

        // obtain intrinsic parameters
        final var intrinsic = iac.getIntrinsicParameters();

        assertNotNull(intrinsic);

        final var horizontalFocalLength = intrinsic.getHorizontalFocalLength();
        final var verticalFocalLength = intrinsic.getVerticalFocalLength();
        final var skewness = intrinsic.getSkewness();
        final var horizontalPrincipalPoint = intrinsic.getHorizontalPrincipalPoint();
        final var verticalPrincipalPoint = intrinsic.getVerticalPrincipalPoint();
        assertTrue(horizontalFocalLength > 0);
        assertTrue(verticalFocalLength > 0);
        assertEquals(horizontalFocalLength, verticalFocalLength, ABSOLUTE_ERROR);
        assertEquals(0.0, skewness, 0.0);
        assertEquals(0.0, horizontalPrincipalPoint, 0.0);
        assertEquals(0.0, verticalPrincipalPoint, 0.0);

        final var msg = "Real data focal length: " + horizontalFocalLength;
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).log(Level.INFO, msg);
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
    public void onEstimationProgressChange(final ImageOfAbsoluteConicEstimator estimator, final float progress) {
        estimationProgressChange++;
        testLocked((LMSEImageOfAbsoluteConicEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimationProgressChange = 0;
    }

    private void testLocked(final LMSEImageOfAbsoluteConicEstimator estimator) {
        assertThrows(LockedException.class, () -> estimator.setZeroSkewness(true));
        assertThrows(LockedException.class, () -> estimator.setPrincipalPointAtOrigin(true));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, () -> estimator.setHomographies(null));
        assertThrows(LockedException.class, () -> estimator.setLMSESolutionAllowed(true));
    }
}
