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

public class WeightedImageOfAbsoluteConicEstimatorTest implements ImageOfAbsoluteConicEstimatorListener {

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

    private static final double ABSOLUTE_ERROR = 1e-6;
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
        WeightedImageOfAbsoluteConicEstimator estimator = new WeightedImageOfAbsoluteConicEstimator();

        // check default values
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS,
                estimator.isZeroSkewness());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertNull(estimator.getHomographies());
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
        assertFalse(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR, estimator.getType());
        assertNull(estimator.getWeights());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_MAX_HOMOGRAPHIES,
                estimator.getMaxHomographies());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());

        // test constructor with listener
        estimator = new WeightedImageOfAbsoluteConicEstimator(this);

        // check default values
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS,
                estimator.isZeroSkewness());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getHomographies());
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
        assertFalse(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR, estimator.getType());
        assertNull(estimator.getWeights());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_MAX_HOMOGRAPHIES,
                estimator.getMaxHomographies());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());

        // test constructor with homographies
        final List<Transformation2D> homographies = new ArrayList<>();
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        final double[] weights = new double[3];

        estimator = new WeightedImageOfAbsoluteConicEstimator(homographies, weights);

        // check default values
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS,
                estimator.isZeroSkewness());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertSame(homographies, estimator.getHomographies());
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
        assertTrue(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR, estimator.getType());
        assertSame(weights, estimator.getWeights());
        assertTrue(estimator.areWeightsAvailable());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_MAX_HOMOGRAPHIES,
                estimator.getMaxHomographies());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());

        // Force IllegalArgumentException
        final List<Transformation2D> emptyHomographies = new ArrayList<>();
        final double[] shortWeights = new double[1];

        estimator = null;
        try {
            estimator = new WeightedImageOfAbsoluteConicEstimator(null, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new WeightedImageOfAbsoluteConicEstimator(homographies, shortWeights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new WeightedImageOfAbsoluteConicEstimator(emptyHomographies, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new WeightedImageOfAbsoluteConicEstimator(homographies, shortWeights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with homographies and listener
        estimator = new WeightedImageOfAbsoluteConicEstimator(homographies, weights, this);

        // check default values
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS,
                estimator.isZeroSkewness());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertSame(this, estimator.getListener());
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
        assertTrue(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR,
                estimator.getType());
        assertSame(weights, estimator.getWeights());
        assertTrue(estimator.areWeightsAvailable());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_MAX_HOMOGRAPHIES,
                estimator.getMaxHomographies());
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new WeightedImageOfAbsoluteConicEstimator(null, weights, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new WeightedImageOfAbsoluteConicEstimator(homographies, shortWeights, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new WeightedImageOfAbsoluteConicEstimator(emptyHomographies, weights, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new WeightedImageOfAbsoluteConicEstimator(homographies, shortWeights, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testIsSetZeroSkewness() throws LockedException {
        final WeightedImageOfAbsoluteConicEstimator estimator = new WeightedImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS,
                estimator.isZeroSkewness());

        // set new value
        estimator.setZeroSkewness(!WeightedImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);

        // check correctness
        assertEquals(!WeightedImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS,
                estimator.isZeroSkewness());
    }

    @Test
    public void testIsSetPrincipalPointAtOrigin() throws LockedException {
        final WeightedImageOfAbsoluteConicEstimator estimator = new WeightedImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());

        // set new value
        estimator.setPrincipalPointAtOrigin(
                !WeightedImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);

        // check correctness
        assertEquals(!WeightedImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
    }

    @Test
    public void testIsFocalDistanceAspectRatioKnown() throws LockedException {
        final WeightedImageOfAbsoluteConicEstimator estimator = new WeightedImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());

        // set new value
        estimator.setFocalDistanceAspectRatioKnown(
                !WeightedImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);

        assertEquals(!WeightedImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
    }

    @Test
    public void testGetSetFocalDistanceAspectRatio() throws LockedException {
        final WeightedImageOfAbsoluteConicEstimator estimator = new WeightedImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);

        // set new value
        estimator.setFocalDistanceAspectRatio(0.5);

        // check correctness
        assertEquals(0.5, estimator.getFocalDistanceAspectRatio(), 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setFocalDistanceAspectRatio(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final WeightedImageOfAbsoluteConicEstimator estimator = new WeightedImageOfAbsoluteConicEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testGetSetHomographiesAndWeightsAndIsReady() throws LockedException {
        final WeightedImageOfAbsoluteConicEstimator estimator = new WeightedImageOfAbsoluteConicEstimator();

        // check default value
        assertNull(estimator.getHomographies());
        assertNull(estimator.getWeights());
        assertFalse(estimator.isReady());

        // set new value
        final List<Transformation2D> homographies = new ArrayList<>();
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        final double[] weights = new double[3];

        estimator.setHomographiesAndWeights(homographies, weights);

        // check correctness
        assertSame(estimator.getHomographies(), homographies);
        assertSame(estimator.getWeights(), weights);
        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final List<Transformation2D> emptyHomographies = new ArrayList<>();
        final double[] shortWeights = new double[1];

        try {
            estimator.setHomographiesAndWeights(null, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setHomographiesAndWeights(homographies, shortWeights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setHomographiesAndWeights(emptyHomographies, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setHomographiesAndWeights(homographies, shortWeights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // setting only homographies always fails because weights must be
        // provided too
        try {
            estimator.setHomographies(homographies);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxHomographies() throws LockedException {
        final WeightedImageOfAbsoluteConicEstimator estimator = new WeightedImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_MAX_HOMOGRAPHIES,
                estimator.getMaxHomographies());

        // set new value
        estimator.setMaxHomographies(estimator.getMinNumberOfRequiredHomographies());

        // check correctness
        assertEquals(1, estimator.getMaxHomographies());

        // Force IllegalArgumentException
        try {
            estimator.setMaxHomographies(
                    estimator.getMinNumberOfRequiredHomographies() - 1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetSortWeightsEnabled() throws LockedException {
        final WeightedImageOfAbsoluteConicEstimator estimator = new WeightedImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(WeightedImageOfAbsoluteConicEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());

        // set new value
        estimator.setSortWeightsEnabled(
                !WeightedImageOfAbsoluteConicEstimator.DEFAULT_SORT_WEIGHTS);

        // check correctness
        assertEquals(!WeightedImageOfAbsoluteConicEstimator.DEFAULT_SORT_WEIGHTS,
                estimator.isSortWeightsEnabled());
    }

    @Test
    public void testEstimateNoConstraints()
            throws InvalidPinholeCameraIntrinsicParametersException, LockedException, NotReadyException,
            RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
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
                final double verticalFocalLength = randomizer.nextDouble(
                        MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
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
                final List<Transformation2D> homographies = new ArrayList<>();
                final double[] weights = new double[50];
                randomizer.fill(weights, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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
                    randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                    // create camera with intrinsic parameters, rotation and camera
                    // center
                    final PinholeCamera camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                    camera.normalize();

                    // project 3D pattern points in a plane
                    final List<Point2D> projectedPatternPoints = camera.project(points3D);

                    final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                            ProjectiveTransformation2DRobustEstimator.createFromPoints(
                                    patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                    final ProjectiveTransformation2D homography = homographyEstimator.estimate();
                    homographies.add(homography);
                }

                // Estimate  IAC
                final WeightedImageOfAbsoluteConicEstimator estimator =
                        new WeightedImageOfAbsoluteConicEstimator(homographies, weights, this);
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
                final ImageOfAbsoluteConic iac2 = estimator.estimate();

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
                    final PinholeCameraIntrinsicParameters intrinsic2 = iac2.getIntrinsicParameters();

                    assertEquals(intrinsic.getHorizontalFocalLength(),
                            intrinsic2.getHorizontalFocalLength(), VERY_LARGE_ABSOLUTE_ERROR);
                    assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                            VERY_LARGE_ABSOLUTE_ERROR);
                    assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(),
                            VERY_LARGE_ABSOLUTE_ERROR);
                    assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                            intrinsic2.getHorizontalPrincipalPoint(), VERY_LARGE_ABSOLUTE_ERROR);
                    assertEquals(intrinsic.getVerticalPrincipalPoint(),
                            intrinsic2.getVerticalPrincipalPoint(), VERY_LARGE_ABSOLUTE_ERROR);

                    horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength() -
                            intrinsic2.getHorizontalFocalLength());
                    verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength() -
                            intrinsic2.getVerticalFocalLength());
                    skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                    horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint() -
                            intrinsic2.getHorizontalPrincipalPoint());
                    verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint() -
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
            if (Math.abs(avgHorizontalFocalDistanceError) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(avgHorizontalPrincipalPointError) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(0.0, avgHorizontalFocalDistanceError, LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, avgVerticalFocalDistanceError, LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, avgSkewnessError, LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, avgHorizontalPrincipalPointError, LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, avgVerticalPrincipalPointError, LARGE_ABSOLUTE_ERROR);

            final String msg = "No constraints - failed: " +
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
            Logger.getLogger(WeightedImageOfAbsoluteConicEstimatorTest.class.getName()).
                    log(Level.INFO, msg);

            assertTrue(failedRatio < 0.75);
            assertTrue(succeededRatio >= 0.25);
            assertTrue(succeededAtLeastOnce);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateZeroSkewness()
            throws InvalidPinholeCameraIntrinsicParametersException, LockedException, NotReadyException,
            RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

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
        double horizontalFocalDistanceError, verticalFocalDistanceError,
                skewnessError, horizontalPrincipalPointError,
                verticalPrincipalPointError;
        for (int j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
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
            final List<Transformation2D> homographies = new ArrayList<>();
            final double[] weights = new double[50];
            randomizer.fill(weights, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.createFromPoints(
                                patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final WeightedImageOfAbsoluteConicEstimator estimator =
                    new WeightedImageOfAbsoluteConicEstimator(homographies, weights, this);
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
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

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
                final PinholeCameraIntrinsicParameters intrinsic2 = iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(0.0, intrinsic.getSkewness(), 0.0);
                assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                        intrinsic2.getHorizontalPrincipalPoint(), VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength() -
                        intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint() -
                        intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint() -
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
        assertEquals(0.0, avgHorizontalFocalDistanceError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, LARGE_ABSOLUTE_ERROR);

        final String msg = "Zero skewness - failed: " +
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
    public void testEstimatePrincipalPointAtOrigin()
            throws InvalidPinholeCameraIntrinsicParametersException, LockedException, NotReadyException,
            RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

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
            final double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
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
            final double[] weights = new double[50];
            randomizer.fill(weights, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.createFromPoints(
                                patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final WeightedImageOfAbsoluteConicEstimator estimator =
                    new WeightedImageOfAbsoluteConicEstimator(homographies, weights, this);
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
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

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
                final PinholeCameraIntrinsicParameters intrinsic2 = iac2.getIntrinsicParameters();

                if (Math.abs(intrinsic.getHorizontalFocalLength() - intrinsic2.getHorizontalFocalLength()) >
                        VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalFocalLength() - intrinsic2.getVerticalFocalLength()) >
                        VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness()) > VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic2.getHorizontalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
                assertEquals(0.0, intrinsic2.getVerticalPrincipalPoint(), 0.0);

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength() -
                        intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint() -
                        intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint() -
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
            } catch (final InvalidPinholeCameraIntrinsicParametersException ignore) {
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
        assertEquals(0.0, avgHorizontalFocalDistanceError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, LARGE_ABSOLUTE_ERROR);

        final String msg = "Principal point at origin - failed: " +
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
    public void testEstimateZeroSkewnessPrincipalPointAtOrigin()
            throws InvalidPinholeCameraIntrinsicParametersException, LockedException, NotReadyException,
            RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

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
            final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
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
            final List<Transformation2D> homographies = new ArrayList<>();
            final double[] weights = new double[50];
            randomizer.fill(weights, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

                final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.createFromPoints(
                                patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final WeightedImageOfAbsoluteConicEstimator estimator =
                    new WeightedImageOfAbsoluteConicEstimator(homographies, weights, this);
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
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

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
                final PinholeCameraIntrinsicParameters intrinsic2 = iac2.getIntrinsicParameters();

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

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength() -
                        intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint() -
                        intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint() -
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
        assertEquals(0.0, avgHorizontalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, LARGE_ABSOLUTE_ERROR);

        final String msg = "No skewness and Principal point at origin - failed: " +
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
    public void testEstimateZeroSkewnessAspectRatioKnown()
            throws InvalidPinholeCameraIntrinsicParametersException, LockedException, NotReadyException,
            RobustEstimatorException, ImageOfAbsoluteConicEstimatorException {

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
            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
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
            final List<Transformation2D> homographies = new ArrayList<>();
            final double[] weights = new double[50];
            randomizer.fill(weights, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

                final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                                createFromPoints(patternPoints, projectedPatternPoints,
                                        RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate  IAC
            final WeightedImageOfAbsoluteConicEstimator estimator =
                    new WeightedImageOfAbsoluteConicEstimator(homographies, weights, this);
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
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

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
                final PinholeCameraIntrinsicParameters intrinsic2 = iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                        2.0 * VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                        2.0 * VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(0.0, intrinsic.getSkewness(), 0.0);
                assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                        intrinsic2.getHorizontalPrincipalPoint(), VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(),
                        VERY_LARGE_ABSOLUTE_ERROR);

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength() -
                        intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint() -
                        intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint() -
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
        assertEquals(0.0, avgHorizontalFocalDistanceError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, LARGE_ABSOLUTE_ERROR);

        final String msg = "Zero skewness, aspect ratio known - failed: " +
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
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).log(Level.INFO, msg);

        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    public void testEstimateZeroSkewnessPrincipalPointAtOriginAspectRatioKnown()
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
            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
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
            final List<Transformation2D> homographies = new ArrayList<>();
            final double[] weights = new double[50];
            randomizer.fill(weights, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

                final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points in a plane
                final List<Point2D> projectedPatternPoints = camera.project(points3D);

                final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.createFromPoints(
                                patternPoints, projectedPatternPoints, RobustEstimatorMethod.RANSAC);

                final ProjectiveTransformation2D homography = homographyEstimator.estimate();
                homographies.add(homography);
            }

            // Estimate IAC
            final WeightedImageOfAbsoluteConicEstimator estimator =
                    new WeightedImageOfAbsoluteConicEstimator(homographies, weights, this);
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
            final ImageOfAbsoluteConic iac2 = estimator.estimate();

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
                final PinholeCameraIntrinsicParameters intrinsic2 = iac2.getIntrinsicParameters();

                if (Math.abs(intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength()) > ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
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

                horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength() -
                        intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint() -
                        intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint() -
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
        assertEquals(0.0, avgHorizontalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);

        final String msg = "No skewness, Principal point at origin, aspect ratio known - failed: " +
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
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).log(Level.INFO, msg);

        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    public void testEstimateRealData() throws CoincidentPointsException,
            InvalidPinholeCameraIntrinsicParametersException, LockedException, NotReadyException,
            ImageOfAbsoluteConicEstimatorException {
        // For a QR pattern, assuming zero skewness, equal focal lengths and
        // principal point at origin on a nexus 5 device
        final Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
        final List<Point2D> patternPoints = pattern.getIdealPoints();
        
        /*
        Sampled data (before and after centering coordinates and setting correct y-axis
        direction)
        Point[0] = 774.5, 1084.5 | 6.5, -60.5
        Point[1] = 791.5, 840.0 | 23.5, 184.0
        Point[2] = 1037.5, 854.0 | 269.5, 170.0
        Point[3] = 999.0, 1074.0 | 231.0, -50.0        
        */
        final List<Point2D> sampledPoints1 = new ArrayList<>();
        sampledPoints1.add(new InhomogeneousPoint2D(6.5, -60.5));
        sampledPoints1.add(new InhomogeneousPoint2D(23.5, 184.0));
        sampledPoints1.add(new InhomogeneousPoint2D(269.5, 170.0));
        sampledPoints1.add(new InhomogeneousPoint2D(231.0, -50.0));
        
        /*
        Sampled data (before and after centering coordinates and setting correct y-axis
        direction)
        Point[0] = 382.5, 701.5 | -385.5, 322.5
        Point[1] = 351.0, 473.5 | -417.0, 550.5
        Point[2] = 585.0, 451.5 | -183.0, 572.5
        Point[3] = 592.0, 653.5 | -176.0, 370.5
        */
        final List<Point2D> sampledPoints2 = new ArrayList<>();
        sampledPoints2.add(new InhomogeneousPoint2D(-385.5, 322.5));
        sampledPoints2.add(new InhomogeneousPoint2D(-417.0, 550.5));
        sampledPoints2.add(new InhomogeneousPoint2D(-183.0, 572.5));
        sampledPoints2.add(new InhomogeneousPoint2D(-176.0, 370.5));

        /*
        Sampled data (before and after centering coordinates and setting correct y-axis
        direction)
        Point[0] = 988.0, 486.5 | 220.0, 537.5
        Point[1] = 1028.5, 278.5 | 260.5, 745.5
        Point[2] = 1241.0, 316.5 | 473.0, 707.5
        Point[3] = 1185.5, 498.5 | 417.5, 525.5
        */
        final List<Point2D> sampledPoints3 = new ArrayList<>();
        sampledPoints3.add(new InhomogeneousPoint2D(220.0, 537.5));
        sampledPoints3.add(new InhomogeneousPoint2D(260.5, 745.5));
        sampledPoints3.add(new InhomogeneousPoint2D(473.0, 707.5));
        sampledPoints3.add(new InhomogeneousPoint2D(417.5, 525.5));
        
        /*
        Sampled data (before and after centering coordinates and setting correct y-axis
        direction)
        Point[0] = 576.0, 1404.4166 | -192.0, -380.4166259765625
        Point[1] = 544.5, 1151.5 | -223.5, -127.5
        Point[2] = 792.0, 1117.5 | 24.0, -93.5
        Point[3] = 798.5, 1347.0 | 30.5, -323.0
        */
        final List<Point2D> sampledPoints4 = new ArrayList<>();
        sampledPoints4.add(new InhomogeneousPoint2D(-192.0, -380.4166259765625));
        sampledPoints4.add(new InhomogeneousPoint2D(-223.5, -127.5));
        sampledPoints4.add(new InhomogeneousPoint2D(24.0, -93.5));
        sampledPoints4.add(new InhomogeneousPoint2D(30.5, -323.0));

        /*
        Sampled data (before and after centering coordinates and setting correct y-axis
        direction)
        Point[0] = 913.5, 1596.0 | 145.5, -572.0
        Point[1] = 939.5, 1360.7 | 171.5, -336.699951171875
        Point[2] = 1170.5, 1391.0 | 402.5, -367.0
        Point[3] = 1126.5, 1600.5 | 358.5, -576.5 
        */
        final List<Point2D> sampledPoints5 = new ArrayList<>();
        sampledPoints5.add(new InhomogeneousPoint2D(145.5, -572.0));
        sampledPoints5.add(new InhomogeneousPoint2D(171.5, -336.699951171875));
        sampledPoints5.add(new InhomogeneousPoint2D(402.5, -367.0));
        sampledPoints5.add(new InhomogeneousPoint2D(358.5, -576.5));

        //obtain homographies
        final ProjectiveTransformation2D homography1 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1),
                patternPoints.get(2), patternPoints.get(3),
                sampledPoints1.get(0), sampledPoints1.get(1),
                sampledPoints1.get(2), sampledPoints1.get(3));

        final ProjectiveTransformation2D homography2 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1),
                patternPoints.get(2), patternPoints.get(3),
                sampledPoints2.get(0), sampledPoints2.get(1),
                sampledPoints2.get(2), sampledPoints2.get(3));

        final ProjectiveTransformation2D homography3 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1),
                patternPoints.get(2), patternPoints.get(3),
                sampledPoints3.get(0), sampledPoints3.get(1),
                sampledPoints3.get(2), sampledPoints3.get(3));

        final ProjectiveTransformation2D homography4 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1),
                patternPoints.get(2), patternPoints.get(3),
                sampledPoints4.get(0), sampledPoints4.get(1),
                sampledPoints4.get(2), sampledPoints4.get(3));

        final ProjectiveTransformation2D homography5 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1),
                patternPoints.get(2), patternPoints.get(3),
                sampledPoints5.get(0), sampledPoints5.get(1),
                sampledPoints5.get(2), sampledPoints5.get(3));

        final List<Transformation2D> homographies = new ArrayList<>();
        homographies.add(homography1);
        homographies.add(homography2);
        homographies.add(homography3);
        homographies.add(homography4);
        homographies.add(homography5);

        final double[] weights = new double[]{0.001, 1.0, 0.1, 0.003, 0.002};

        // estimate IAC
        final WeightedImageOfAbsoluteConicEstimator estimator =
                new WeightedImageOfAbsoluteConicEstimator(homographies, weights);

        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);

        final ImageOfAbsoluteConic iac = estimator.estimate();

        assertNotNull(iac);

        // obtain intrinsic parameters
        final PinholeCameraIntrinsicParameters intrinsic = iac.getIntrinsicParameters();

        assertNotNull(intrinsic);

        final double horizontalFocalLength = intrinsic.getHorizontalFocalLength();
        final double verticalFocalLength = intrinsic.getVerticalFocalLength();
        final double skewness = intrinsic.getSkewness();
        final double horizontalPrincipalPoint = intrinsic.getHorizontalPrincipalPoint();
        final double verticalPrincipalPoint = intrinsic.getVerticalPrincipalPoint();
        assertTrue(horizontalFocalLength > 0);
        assertTrue(verticalFocalLength > 0);
        assertEquals(horizontalFocalLength, verticalFocalLength, ABSOLUTE_ERROR);
        assertEquals(0.0, skewness, 0.0);
        assertEquals(0.0, horizontalPrincipalPoint, 0.0);
        assertEquals(0.0, verticalPrincipalPoint, 0.0);

        final String msg = "Real data focal length: " + horizontalFocalLength;
        Logger.getLogger(WeightedImageOfAbsoluteConicEstimatorTest.class.getName()).log(Level.INFO, msg);
    }

    @Override
    public void onEstimateStart(final ImageOfAbsoluteConicEstimator estimator) {
        estimateStart++;
        testLocked((WeightedImageOfAbsoluteConicEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final ImageOfAbsoluteConicEstimator estimator) {
        estimateEnd++;
        testLocked((WeightedImageOfAbsoluteConicEstimator) estimator);
    }

    @Override
    public void onEstimationProgressChange(
            final ImageOfAbsoluteConicEstimator estimator, final float progress) {
        estimationProgressChange++;
        testLocked((WeightedImageOfAbsoluteConicEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimationProgressChange = 0;
    }

    private void testLocked(final WeightedImageOfAbsoluteConicEstimator estimator) {
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
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setHomographiesAndWeights(null, null);
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMaxHomographies(1);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setSortWeightsEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }
}
