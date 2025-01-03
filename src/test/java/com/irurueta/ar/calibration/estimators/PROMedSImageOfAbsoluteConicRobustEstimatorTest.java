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
import com.irurueta.geometry.estimators.RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.jupiter.api.Assertions.*;

class PROMedSImageOfAbsoluteConicRobustEstimatorTest implements ImageOfAbsoluteConicRobustEstimatorListener {

    private static final double MIN_FOCAL_LENGTH = 3.0;
    private static final double MAX_FOCAL_LENGTH = 10.0;

    private static final double MIN_ANGLE_DEGREES = -10.0;
    private static final double MAX_ANGLE_DEGREES = 10.0;

    private static final int INHOM_3D_COORDS = 3;

    private static final double MIN_RANDOM_VALUE = -50.0;
    private static final double MAX_RANDOM_VALUE = -10.0;

    private static final double ABSOLUTE_ERROR = 5e-6;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 5e-1;
    private static final double ULTRA_LARGE_ABSOLUTE_ERROR = 3.0;

    private static final int MIN_NUM_HOMOGRAPHIES = 100;
    private static final int MAX_NUM_HOMOGRAPHIES = 500;

    private static final double STOP_THRESHOLD = 1e-9;

    private static final int TIMES = 50;

    private static final int PERCENTAGE_OUTLIERS = 20;

    private static final double STD_ERROR = 1.0;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    // point to be reused when computing homographies residuals
    private final Point2D testPoint = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);

    @Test
    void testConstructor() {
        // test constructor without parameters
        var estimator = new PROMedSImageOfAbsoluteConicRobustEstimator();

        // check default values
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getHomographies());
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(PROMedSImageOfAbsoluteConicRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());

        // test constructor with listener
        estimator = new PROMedSImageOfAbsoluteConicRobustEstimator(this);

        // check default values
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getHomographies());
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(PROMedSImageOfAbsoluteConicRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());

        // test constructor with homographies
        final var homographies = new ArrayList<Transformation2D>();
        homographies.add(new ProjectiveTransformation2D());

        estimator = new PROMedSImageOfAbsoluteConicRobustEstimator(homographies);

        // check default values
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(homographies, estimator.getHomographies());
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(PROMedSImageOfAbsoluteConicRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());

        // Force IllegalArgumentException
        final var emptyHomographies = new ArrayList<Transformation2D>();
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSImageOfAbsoluteConicRobustEstimator((List<Transformation2D>) null));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSImageOfAbsoluteConicRobustEstimator(emptyHomographies));

        // test constructor with homographies and listener
        estimator = new PROMedSImageOfAbsoluteConicRobustEstimator(homographies, this);

        // check default values
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(homographies, estimator.getHomographies());
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(PROMedSImageOfAbsoluteConicRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSImageOfAbsoluteConicRobustEstimator((List<Transformation2D>) null));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSImageOfAbsoluteConicRobustEstimator(emptyHomographies));

        // test constructor with quality scores
        final var qualityScores = new double[1];
        estimator = new PROMedSImageOfAbsoluteConicRobustEstimator(qualityScores);

        // check default values
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getHomographies());
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(PROMedSImageOfAbsoluteConicRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());

        // force IllegalArgumentException
        final var emptyScores = new double[0];
        assertThrows(IllegalArgumentException.class, () -> new PROMedSImageOfAbsoluteConicRobustEstimator(emptyScores));

        // test constructor with quality scores and listener
        estimator = new PROMedSImageOfAbsoluteConicRobustEstimator(qualityScores, this);

        // check default values
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getHomographies());
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(PROMedSImageOfAbsoluteConicRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSImageOfAbsoluteConicRobustEstimator(emptyScores, this));

        // test constructor with homographies and quality scores
        estimator = new PROMedSImageOfAbsoluteConicRobustEstimator(homographies, qualityScores);

        // check default values
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(homographies, estimator.getHomographies());
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(PROMedSImageOfAbsoluteConicRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSImageOfAbsoluteConicRobustEstimator(null, qualityScores));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSImageOfAbsoluteConicRobustEstimator(emptyHomographies, qualityScores));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSImageOfAbsoluteConicRobustEstimator(homographies, emptyScores));

        // test constructor with homographies, quality scores and listener
        estimator = new PROMedSImageOfAbsoluteConicRobustEstimator(homographies, qualityScores, this);

        // check default values
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(homographies, estimator.getHomographies());
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(PROMedSImageOfAbsoluteConicRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSImageOfAbsoluteConicRobustEstimator(
                null, qualityScores, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSImageOfAbsoluteConicRobustEstimator(
                emptyHomographies, qualityScores, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSImageOfAbsoluteConicRobustEstimator(
                homographies, emptyScores, this));
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var estimator = new PROMedSImageOfAbsoluteConicRobustEstimator();

        // check default value
        assertEquals(PROMedSImageOfAbsoluteConicRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);

        // set new value
        estimator.setStopThreshold(1.0);

        // check correctness
        assertEquals(1.0, estimator.getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setStopThreshold(0.0));
    }

    @Test
    void testIsSetZeroSkewness() throws LockedException {
        final var estimator = new PROMedSImageOfAbsoluteConicRobustEstimator();

        // check default value
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());

        // set new value
        estimator.setZeroSkewness(!ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);

        // check correctness
        assertEquals(!ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
    }

    @Test
    void testIsSetPrincipalPointAtOrigin() throws LockedException {
        final var estimator = new PROMedSImageOfAbsoluteConicRobustEstimator();

        // check default value
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());

        // set new value
        estimator.setPrincipalPointAtOrigin(!ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);

        // check correctness
        assertEquals(!ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
    }

    @Test
    void testIsSetFocalDistanceAspectRatioKnown() throws LockedException {
        final var estimator = new PROMedSImageOfAbsoluteConicRobustEstimator();

        // check default value
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());

        // set new value
        estimator.setFocalDistanceAspectRatioKnown(
                !ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);

        // check correctness
        assertEquals(!ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
    }

    @Test
    void testGetSetFocalDistanceAspectRatio() throws LockedException {
        final var estimator = new PROMedSImageOfAbsoluteConicRobustEstimator();

        // check default value
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
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
        final var estimator = new PROMedSImageOfAbsoluteConicRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var estimator = new PROMedSImageOfAbsoluteConicRobustEstimator();

        // check default value
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5, estimator.getProgressDelta(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(2.0f));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var estimator = new PROMedSImageOfAbsoluteConicRobustEstimator();

        // check default value
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.5);

        // check correctness
        assertEquals(0.5, estimator.getConfidence(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(2.0));
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var estimator = new PROMedSImageOfAbsoluteConicRobustEstimator();

        // check default value
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(100);

        // check correctness
        assertEquals(100, estimator.getMaxIterations());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testGetSetHomographiesAndIsReady() throws LockedException {
        final var estimator = new PROMedSImageOfAbsoluteConicRobustEstimator();

        // check default value
        assertNull(estimator.getHomographies());
        assertFalse(estimator.isReady());
        assertEquals(1, estimator.getMinNumberOfRequiredHomographies());

        // set new value
        final var randomizer = new UniformRandomizer();
        final int numHomographies = randomizer.nextInt(MIN_NUM_HOMOGRAPHIES, MAX_NUM_HOMOGRAPHIES);
        final var qualityScores = new double[numHomographies];
        final var homographies = new ArrayList<Transformation2D>();
        for (var i = 0; i < numHomographies; i++) {
            homographies.add(new ProjectiveTransformation2D());
        }

        estimator.setHomographies(homographies);

        // check correctness
        assertSame(homographies, estimator.getHomographies());
        assertFalse(estimator.isReady());

        // if we also set quality scores, then estimator becomes ready
        estimator.setQualityScores(qualityScores);

        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final var emptyHomographies = new ArrayList<Transformation2D>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setHomographies(null));
        assertThrows(IllegalArgumentException.class, () -> estimator.setHomographies(emptyHomographies));
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var estimator = new PROSACImageOfAbsoluteConicRobustEstimator();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        final var qualityScores = new double[1];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        final var emptyScores = new double[0];
        assertThrows(IllegalArgumentException.class, () -> estimator.setQualityScores(emptyScores));
    }

    @Test
    void testEstimateCirclesPattern() throws InvalidPinholeCameraIntrinsicParametersException, LockedException,
            NotReadyException, RobustEstimatorException {

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

            // create homographies.
            // Create one random camera for each homography having a random
            // rotation and translation but all having the same intrinsic
            // parameters
            final var nHomographies = randomizer.nextInt(MIN_NUM_HOMOGRAPHIES, MAX_NUM_HOMOGRAPHIES);
            final var homographies = new ArrayList<Transformation2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            final var qualityScores = new double[nHomographies];
            for (var i = 0; i < nHomographies; i++) {
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

                // project 3D pattern points
                final var projectedPatternPoints = camera.project(points3D);

                // add random noise to projected points with a certain outlier
                // proportion
                final var projectedPatternPointsWithError = new ArrayList<Point2D>();
                for (final var projectedPatternPoint : projectedPatternPoints) {
                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier sample
                        final var errorX = errorRandomizer.nextDouble();
                        final var errorY = errorRandomizer.nextDouble();
                        projectedPatternPointsWithError.add(new HomogeneousPoint2D(
                                projectedPatternPoint.getInhomX() + errorX,
                                projectedPatternPoint.getInhomY() + errorY,
                                1.0));
                    } else {
                        // inlier
                        projectedPatternPointsWithError.add(projectedPatternPoint);
                    }
                }

                // estimate homography. Because a robust estimator is used,
                // probably the effect of outliers will already be removed by
                // the time the homography is obtained
                final var homographyEstimator = new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                        patternPoints, projectedPatternPointsWithError);
                // enforce a strict threshold to estimate homographies
                homographyEstimator.setThreshold(STOP_THRESHOLD);

                final var homography = homographyEstimator.estimate();
                homographies.add(homography);

                // in real situations projectedPatternPoints is unknown, and
                // hence quality scores must be computed in a different way
                // (for instance using projectedPatternPointsWithError, or
                // distance from center of image, where supposedly center of
                // radial distortion is expected to be, and hence the farther
                // measures are from the center more distorted will be) or
                // quality of the features used to detect calibration markers
                final var avgHomographyResidual = avgHomographyResidual(homography, patternPoints,
                        projectedPatternPoints);
                qualityScores[i] = 1.0 / (1.0 + avgHomographyResidual);
            }

            // Estimate IAC
            final var estimator = new PROMedSImageOfAbsoluteConicRobustEstimator(homographies, qualityScores,
                    this);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setStopThreshold(STOP_THRESHOLD);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            // estimate
            final var iac2 = estimator.estimate();

            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
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
                // check intrinsic parameters
                final var intrinsic2 = iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                        VERY_LARGE_ABSOLUTE_ERROR);
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

        final var msg = "Circles pattern - failed: " + failedRatio * 100.0
                + "% succeeded: " + succeededRatio * 100.0
                + "% avg horizontal focal distance error: " + avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " avg min horizontal focal distance error: " + minHorizontalFocalDistanceError +
                " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError +
                " max vertical principal point error: " + maxVerticalPrincipalPointError;
        Logger.getLogger(PROMedSImageOfAbsoluteConicRobustEstimatorTest.class.getName()).log(Level.INFO, msg);

        assertTrue(failedRatio < 0.6);
        assertTrue(succeededRatio >= 0.4);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    void testEstimateQRPattern() throws InvalidPinholeCameraIntrinsicParametersException, LockedException,
            NotReadyException, RobustEstimatorException {

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
            final var pattern = Pattern2D.create(Pattern2DType.QR);
            final var patternPoints = pattern.getIdealPoints();

            // assume that pattern points are located on a 3D plane
            // (for instance Z = 0), but can be really any plane
            final var points3D = new ArrayList<Point3D>();
            for (final var patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                        1.0));
            }

            // create homographies.
            // Create one random camera for each homography having a random
            // rotation and translation but all having the same intrinsic
            // parameters
            final var nHomographies = randomizer.nextInt(MIN_NUM_HOMOGRAPHIES, MAX_NUM_HOMOGRAPHIES);
            final var homographies = new ArrayList<Transformation2D>();
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            final var tmpQualityScores = new double[nHomographies];
            var validHomographiesCounter = 0;
            for (var i = 0; i < nHomographies; i++) {
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

                // project 3D pattern points
                final var projectedPatternPoints = camera.project(points3D);

                // add random noise to projected points with a certain outlier
                // proportion
                final var projectedPatternPointsWithError = new ArrayList<Point2D>();
                // we use average error of samples to determine quality scores
                // of homographies, this could be equivalent to the measure of
                // the quality of the features used to detect calibration
                // markers
                var avgError = 0.0;
                for (final var projectedPatternPoint : projectedPatternPoints) {
                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier sample
                        final var errorX = errorRandomizer.nextDouble();
                        final var errorY = errorRandomizer.nextDouble();

                        projectedPatternPointsWithError.add(new HomogeneousPoint2D(
                                projectedPatternPoint.getInhomX() + errorX,
                                projectedPatternPoint.getInhomY() + errorY,
                                1.0));

                        avgError += Math.sqrt(errorX * errorX + errorY * errorY);
                    } else {
                        // inlier
                        projectedPatternPointsWithError.add(projectedPatternPoint);
                    }
                }

                avgError /= projectedPatternPointsWithError.size();

                // estimate projective homography using 4 points of qr pattern.
                // Affine homography is not robustly estimated but effect of
                // errors will be removed during robust IAC estimation
                try {
                    final var homography = new ProjectiveTransformation2D(patternPoints.get(0), patternPoints.get(1),
                            patternPoints.get(2), patternPoints.get(3), projectedPatternPointsWithError.get(0),
                            projectedPatternPointsWithError.get(1), projectedPatternPointsWithError.get(2),
                            projectedPatternPointsWithError.get(3));
                    homographies.add(homography);

                    tmpQualityScores[validHomographiesCounter] = 1.0 / (1.0 + avgError);
                    validHomographiesCounter++;
                } catch (final CoincidentPointsException ignore) {
                    // no action needed
                }
            }

            final var qualityScores = Arrays.copyOf(tmpQualityScores, validHomographiesCounter);

            // Estimate IAC
            final var estimator = new PROMedSImageOfAbsoluteConicRobustEstimator(homographies, qualityScores,
                    this);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setStopThreshold(STOP_THRESHOLD);

            // check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            // estimate
            final var iac2 = estimator.estimate();

            assertFalse(estimator.isLocked());
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
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
                // check intrinsic parameters
                final var intrinsic2 = iac2.getIntrinsicParameters();

                final var validHorizontalFocalLength = Math.abs(intrinsic.getHorizontalFocalLength()
                        - intrinsic2.getHorizontalFocalLength()) < 3.0 * ULTRA_LARGE_ABSOLUTE_ERROR;
                final var validVerticalFocalLength = Math.abs(intrinsic.getVerticalFocalLength()
                        - intrinsic2.getVerticalFocalLength()) < 3.0 * ULTRA_LARGE_ABSOLUTE_ERROR;
                if (validHorizontalFocalLength && validVerticalFocalLength) {
                    assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                            3.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
                    assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                            3.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
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
                } else {
                    failed++;
                }
            } catch (final InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }

        final var failedRatio = (double) failed / (double) total;
        final var succeededRatio = (double) succeeded / (double) total;

        avgHorizontalFocalDistanceError /= total;
        avgVerticalFocalDistanceError /= total;
        avgSkewnessError /= total;
        avgHorizontalPrincipalPointError /= total;
        avgVerticalPrincipalPointError /= total;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgSkewnessError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);

        final var msg = "QR pattern - failed: " + failedRatio * 100.0
                + "% succeeded: " + succeededRatio * 100.0
                + "% avg horizontal focal distance error: " + avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " avg min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError;
        Logger.getLogger(PROMedSImageOfAbsoluteConicRobustEstimatorTest.class.getName()).log(Level.INFO, msg);

        assertTrue(failedRatio < 0.8);
        assertTrue(succeededRatio >= 0.2);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    void testEstimateRealData() throws CoincidentPointsException, LockedException, NotReadyException,
            RobustEstimatorException, InvalidPinholeCameraIntrinsicParametersException {
        // For a QR pattern, assuming zero skewness, equal focal lengths and
        // principal point at origin on a nexus 5 device
        final var pattern = Pattern2D.create(Pattern2DType.QR);
        final var patternPoints = pattern.getIdealPoints();
        
        /*
        Sampled data (before and after centering coordinates and setting correct y-axis
        direction)
        Point[0] = 774.5, 1084.5 | 6.5, -60.5
        Point[1] = 791.5, 840.0 | 23.5, 184.0
        Point[2] = 1037.5, 854.0 | 269.5, 170.0
        Point[3] = 999.0, 1074.0 | 231.0, -50.0        
        */
        final var sampledPoints1 = new ArrayList<Point2D>();
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
        final var sampledPoints2 = new ArrayList<Point2D>();
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
        final var sampledPoints3 = new ArrayList<Point2D>();
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
        final var sampledPoints4 = new ArrayList<Point2D>();
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
        final var sampledPoints5 = new ArrayList<Point2D>();
        sampledPoints5.add(new InhomogeneousPoint2D(145.5, -572.0));
        sampledPoints5.add(new InhomogeneousPoint2D(171.5, -336.699951171875));
        sampledPoints5.add(new InhomogeneousPoint2D(402.5, -367.0));
        sampledPoints5.add(new InhomogeneousPoint2D(358.5, -576.5));

        // obtain homographies
        final var homography1 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1),
                patternPoints.get(2), patternPoints.get(3),
                sampledPoints1.get(0), sampledPoints1.get(1),
                sampledPoints1.get(2), sampledPoints1.get(3));

        final var homography2 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1),
                patternPoints.get(2), patternPoints.get(3),
                sampledPoints2.get(0), sampledPoints2.get(1),
                sampledPoints2.get(2), sampledPoints2.get(3));

        final var homography3 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1),
                patternPoints.get(2), patternPoints.get(3),
                sampledPoints3.get(0), sampledPoints3.get(1),
                sampledPoints3.get(2), sampledPoints3.get(3));

        final var homography4 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1),
                patternPoints.get(2), patternPoints.get(3),
                sampledPoints4.get(0), sampledPoints4.get(1),
                sampledPoints4.get(2), sampledPoints4.get(3));

        final var homography5 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1),
                patternPoints.get(2), patternPoints.get(3),
                sampledPoints5.get(0), sampledPoints5.get(1),
                sampledPoints5.get(2), sampledPoints5.get(3));

        final var homographies = new ArrayList<Transformation2D>();
        homographies.add(homography1);
        homographies.add(homography2);
        homographies.add(homography3);
        homographies.add(homography4);
        homographies.add(homography5);

        final var qualityScores = new double[]{0.001, 1.0, 0.1, 0.003, 0.002};

        // estimate IAC
        final var estimator = new PROMedSImageOfAbsoluteConicRobustEstimator(homographies, qualityScores, this);
        estimator.setZeroSkewness(true);
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setFocalDistanceAspectRatioKnown(true);
        estimator.setFocalDistanceAspectRatio(1.0);
        estimator.setStopThreshold(STOP_THRESHOLD);

        // check initial state
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(0, estimateStart);
        assertEquals(0, estimateEnd);
        assertEquals(0, estimateNextIteration);
        assertEquals(0, estimateProgressChange);

        // estimate
        final var iac = estimator.estimate();

        assertFalse(estimator.isLocked());
        assertEquals(1, estimateStart);
        assertEquals(1, estimateEnd);
        assertTrue(estimateNextIteration > 0);
        assertTrue(estimateProgressChange >= 0);
        reset();

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
        Logger.getLogger(PROSACImageOfAbsoluteConicRobustEstimatorTest.class.getName()).log(Level.INFO, msg);
    }

    private double homographyResidual(final ProjectiveTransformation2D homography, final Point2D inputPoint,
            final Point2D outputPoint) {
        homography.transform(inputPoint, testPoint);
        return outputPoint.distanceTo(testPoint);
    }

    private double avgHomographyResidual(final ProjectiveTransformation2D homography, final List<Point2D> inputPoints,
            final List<Point2D> outputPoints) {
        final var size = inputPoints.size();
        var avg = 0.0;
        for (var i = 0; i < size; i++) {
            avg += homographyResidual(homography, inputPoints.get(i), outputPoints.get(i));
        }

        return avg / (double) size;
    }

    @Override
    public void onEstimateStart(final ImageOfAbsoluteConicRobustEstimator estimator) {
        estimateStart++;
        testLocked((PROMedSImageOfAbsoluteConicRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final ImageOfAbsoluteConicRobustEstimator estimator) {
        estimateEnd++;
        testLocked((PROMedSImageOfAbsoluteConicRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final ImageOfAbsoluteConicRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        testLocked((PROMedSImageOfAbsoluteConicRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final ImageOfAbsoluteConicRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        testLocked((PROMedSImageOfAbsoluteConicRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private void testLocked(final PROMedSImageOfAbsoluteConicRobustEstimator estimator) {
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setHomographies(null));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(100));
        assertThrows(LockedException.class, () -> estimator.setPrincipalPointAtOrigin(true));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> estimator.setStopThreshold(1e-6));
        assertThrows(LockedException.class, () -> estimator.setZeroSkewness(true));
        assertThrows(LockedException.class, () -> estimator.setQualityScores(null));
        assertThrows(LockedException.class, estimator::estimate);
        assertTrue(estimator.isLocked());
    }
}
