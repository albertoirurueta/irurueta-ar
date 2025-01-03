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

import com.irurueta.ar.calibration.DistortionException;
import com.irurueta.ar.calibration.RadialDistortion;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.NotSupportedException;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class PROMedSRadialDistortionRobustEstimatorTest implements RadialDistortionRobustEstimatorListener {

    private static final double MIN_POINT_VALUE = -1.0;
    private static final double MAX_POINT_VALUE = 1.0;

    private static final double MIN_PARAM_VALUE = -1e-4;
    private static final double MAX_PARAM_VALUE = 1e-4;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final int MIN_NUM_POINTS = 500;
    private static final int MAX_NUM_POINTS = 1000;

    private static final double THRESHOLD = 1e-8;

    private static final double MIN_SCORE_ERROR = -0.3;
    private static final double MAX_SCORE_ERROR = 0.3;

    private static final double STD_ERROR = 100.0;

    private static final int PERCENTAGE_OUTLIER = 20;

    private static final int TIMES = 100;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    void testConstructor() {
        // test constructor without parameters
        var estimator = new PROMedSRadialDistortionRobustEstimator();

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());

        // test constructor with listener
        estimator = new PROMedSRadialDistortionRobustEstimator(this);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());

        // test constructor with points
        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        for (var i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints, undistortedPoints);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRadialDistortionRobustEstimator(emptyPoints, undistortedPoints));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRadialDistortionRobustEstimator(emptyPoints, emptyPoints));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRadialDistortionRobustEstimator(null, undistortedPoints));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRadialDistortionRobustEstimator(distortedPoints, null));

        // test constructor with points and listener
        estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints, undistortedPoints, this);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                undistortedPoints, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                emptyPoints, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(
                null, undistortedPoints, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(distortedPoints,
                null, this));

        // test constructor with points and center
        final var center = Point2D.create();

        estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints, undistortedPoints, center);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(center.getInhomX(), estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(center.getInhomY(), estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                undistortedPoints, center));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                emptyPoints, center));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(
                null, undistortedPoints, center));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(distortedPoints,
                null, center));

        // test constructor with points, center and listener
        estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints, undistortedPoints, center,
                this);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(center.getInhomX(), estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(center.getInhomY(), estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                undistortedPoints, center, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                emptyPoints, center, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(
                null, undistortedPoints, center, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(distortedPoints,
                null, center, this));

        // test constructor with quality scores
        final var qualityScores = new double[PROMedSRadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS];
        estimator = new PROMedSRadialDistortionRobustEstimator(qualityScores);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        final var emptyScores = new double[0];
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(emptyScores));

        // test constructor with quality scores and listener
        estimator = new PROMedSRadialDistortionRobustEstimator(qualityScores, this);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRadialDistortionRobustEstimator(emptyScores, this));

        // test constructor with points and quality scores
        estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints, undistortedPoints, qualityScores);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                undistortedPoints, qualityScores));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                emptyPoints, qualityScores));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(
                null, undistortedPoints, qualityScores));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(distortedPoints,
                null, qualityScores));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(distortedPoints,
                undistortedPoints, emptyScores));

        // test constructor with points, quality scores and listener
        estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints, undistortedPoints, qualityScores,
                this);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                undistortedPoints, qualityScores, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                emptyPoints, qualityScores, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(
                null, undistortedPoints, qualityScores, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(distortedPoints,
                null, qualityScores, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(
                distortedPoints, undistortedPoints, emptyScores, this));

        // test constructor with points, quality scores and center
        estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints, undistortedPoints, qualityScores,
                center);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(center.getInhomX(), estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0);
        assertEquals(center.getInhomY(), estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                undistortedPoints, qualityScores, center));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                emptyPoints, qualityScores, center));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(
                null, undistortedPoints, qualityScores, center));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(distortedPoints,
                null, qualityScores, center));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(distortedPoints,
                undistortedPoints, emptyScores, center));

        // test constructor with points, center and listener
        estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints, undistortedPoints, qualityScores,
                center, this);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(center.getInhomX(), estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(center.getInhomY(), estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                undistortedPoints, qualityScores, center, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                emptyPoints, qualityScores, center, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(
                null, undistortedPoints, qualityScores, center, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(distortedPoints,
                null, qualityScores, center, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRadialDistortionRobustEstimator(distortedPoints,
                undistortedPoints, emptyScores, center, this));
    }

    @Test
    void testGetSetStopThreshold() throws LockedException {
        final var estimator = new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD, estimator.getStopThreshold(),
                0.0);

        // set new value
        estimator.setStopThreshold(10.0);

        // check correctness
        assertEquals(10.0, estimator.getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setStopThreshold(0.0));
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var estimator = new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        final var qualityScores = new double[PROMedSRadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        var emptyScores = new double[1];
        assertThrows(IllegalArgumentException.class, () -> estimator.setQualityScores(emptyScores));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // Set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var estimator = new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5, estimator.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(2.0f));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var estimator = new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.75);

        // check correctness
        assertEquals(0.75, estimator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(2.0));
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var estimator = new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(10);

        // check correctness
        assertEquals(10, estimator.getMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testGetSetPoints() throws LockedException {
        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        for (var i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        final var estimator = new PROMedSRadialDistortionRobustEstimator();

        // check default values
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());

        // set new value
        estimator.setPoints(distortedPoints, undistortedPoints);

        // check correctness
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());

        // if we set quality scores, then estimator becomes ready
        final var qualityScores = new double[distortedPoints.size()];
        estimator.setQualityScores(qualityScores);

        assertTrue(estimator.isReady());

        // clearing list makes instance not ready
        distortedPoints.clear();

        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(emptyPoints, undistortedPoints));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(emptyPoints, emptyPoints));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(null, undistortedPoints));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(distortedPoints, null));
    }

    @Test
    void testGetSetDistortionCenter() throws LockedException {
        final var estimator = new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertNull(estimator.getDistortionCenter());

        // set new value
        final var center = Point2D.create();
        estimator.setDistortionCenter(center);

        // check correctness
        assertSame(center, estimator.getDistortionCenter());
    }

    @Test
    void testGetSetHorizontalFocalLength() throws LockedException {
        final var estimator = new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);

        // set new value
        estimator.setHorizontalFocalLength(2.0);

        // check correctness
        assertEquals(2.0, estimator.getHorizontalFocalLength(), 0.0);
    }

    @Test
    void testGetSetVerticalFocalLength() throws LockedException {
        final var estimator = new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);

        // set new value
        estimator.setVerticalFocalLength(2.0);

        // check correctness
        assertEquals(2.0, estimator.getVerticalFocalLength(), 0.0);
    }

    @Test
    void testGetSetSkew() throws LockedException {
        final var estimator = new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);

        // set new value
        estimator.setSkew(1.0);

        // check correctness
        assertEquals(1.0, estimator.getSkew(), 0.0);
    }

    @Test
    void testGetSetIntrinsic() throws LockedException {
        final var estimator = new PROMedSRadialDistortionRobustEstimator();

        // check default value
        var intrinsic = estimator.getIntrinsic();

        assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, intrinsic.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, intrinsic.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, intrinsic.getSkewness(), 0.0);

        // set new value
        intrinsic = new PinholeCameraIntrinsicParameters(2.0, 3.0,
                4.0, 5.0, 6.0);
        estimator.setIntrinsic(intrinsic);

        // check correctness
        assertEquals(4.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(5.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
        assertEquals(2.0, intrinsic.getHorizontalFocalLength(), 0.0);
        assertEquals(3.0, intrinsic.getVerticalFocalLength(), 0.0);
        assertEquals(6.0, intrinsic.getSkewness(), 0.0);

        // set again
        estimator.setIntrinsic(new InhomogeneousPoint2D(6.0, 5.0), 4.0, 3.0,
                2.0);

        // check correctness
        assertEquals(6.0, estimator.getDistortionCenter().getInhomX(), 0.0);
        assertEquals(5.0, estimator.getDistortionCenter().getInhomY(), 0.0);
        assertEquals(4.0, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(3.0, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(2.0, estimator.getSkew(), 0.0);
    }

    @Test
    void testGetSetNumKParams() throws LockedException {
        final var estimator = new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());

        // set new value
        estimator.setNumKParams(3);

        // check correctness
        assertEquals(3, estimator.getNumKParams());
    }

    @Test
    void testEstimate() throws NotSupportedException, LockedException, NotReadyException, RobustEstimatorException,
            DistortionException {
        final var randomizer = new UniformRandomizer();

        var numValid = 0;
        for (var j = 0; j < TIMES; j++) {
            final var k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
            final var k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);

            final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

            final var distortion = new RadialDistortion(k1, k2, center);

            final var nPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            final var distortedPointsWithError = new ArrayList<Point2D>();
            final var distortedPoints = new ArrayList<Point2D>();
            final var undistortedPoints = new ArrayList<Point2D>();
            final var qualityScores = new double[nPoints];
            Point2D distortedPoint;
            Point2D distortedPointWithError;
            Point2D undistortedPoint;
            for (var i = 0; i < nPoints; i++) {
                undistortedPoint = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                        randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

                distortedPoint = distortion.distort(undistortedPoint);
                final var scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    distortedPointWithError = new InhomogeneousPoint2D(distortedPoint.getInhomX() + errorX,
                            distortedPoint.getInhomY() + errorY);

                    final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    // point is inlier
                    distortedPointWithError = distortedPoint;
                }
                distortedPoints.add(distortedPoint);
                distortedPointsWithError.add(distortedPointWithError);
                undistortedPoints.add(undistortedPoint);
            }

            final var estimator = new PROMedSRadialDistortionRobustEstimator(distortedPointsWithError,
                    undistortedPoints, qualityScores, this);

            estimator.setIntrinsic(distortion.getIntrinsic());
            estimator.setStopThreshold(THRESHOLD);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var distortion2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation
            assertEquals(distortion2.getK1(), k1, ABSOLUTE_ERROR);
            assertEquals(distortion2.getK2(), k2, ABSOLUTE_ERROR);
            assertEquals(distortion2.getCenter(), center);

            var failed = false;
            for (var i = 0; i < nPoints; i++) {
                if (distortedPoints.get(i).distanceTo(distortion2.distort(undistortedPoints.get(i))) > ABSOLUTE_ERROR) {
                    failed = true;
                    break;
                }
                assertEquals(0.0,
                        distortedPoints.get(i).distanceTo(distortion2.distort(undistortedPoints.get(i))),
                        ABSOLUTE_ERROR);
            }

            if (!failed) {
                numValid++;
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    @Override
    public void onEstimateStart(final RadialDistortionRobustEstimator estimator) {
        estimateStart++;
        testLocked((PROMedSRadialDistortionRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final RadialDistortionRobustEstimator estimator) {
        estimateEnd++;
        testLocked((PROMedSRadialDistortionRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final RadialDistortionRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        testLocked((PROMedSRadialDistortionRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final RadialDistortionRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        testLocked((PROMedSRadialDistortionRobustEstimator) estimator);
    }

    private void testLocked(final PROMedSRadialDistortionRobustEstimator estimator) {
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setDistortionCenter(null));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(10));
        assertThrows(LockedException.class, () -> estimator.setPoints(null, null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> estimator.setStopThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setQualityScores(null));
        assertThrows(LockedException.class, estimator::estimate);
        assertTrue(estimator.isLocked());
    }
}
