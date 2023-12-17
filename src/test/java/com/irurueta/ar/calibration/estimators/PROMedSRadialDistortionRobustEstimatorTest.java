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
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PROMedSRadialDistortionRobustEstimatorTest implements
        RadialDistortionRobustEstimatorListener {

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
    public void testConstructor() {
        // test constructor without parameters
        PROMedSRadialDistortionRobustEstimator estimator =
                new PROMedSRadialDistortionRobustEstimator();

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());

        // test constructor with listener
        estimator = new PROMedSRadialDistortionRobustEstimator(this);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS,
                estimator.getNumKParams());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());

        // test constructor with points
        final List<Point2D> distortedPoints = new ArrayList<>();
        final List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints, undistortedPoints);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS,
                estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();
        estimator = null;
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyPoints, undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(null, undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with points and listener
        estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints,
                undistortedPoints, this);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                    undistortedPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyPoints, emptyPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    null, undistortedPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    distortedPoints, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with points and center
        final Point2D center = Point2D.create();

        estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints, undistortedPoints, center);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(center.getInhomX(), estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0);
        assertEquals(center.getInhomY(), estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyPoints, undistortedPoints, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyPoints, emptyPoints, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    null, undistortedPoints, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    distortedPoints, null, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with points, center and listener
        estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints,
                undistortedPoints, center, this);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(center.getInhomX(), estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0);
        assertEquals(center.getInhomY(), estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                    undistortedPoints, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                    emptyPoints, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    null, undistortedPoints, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    distortedPoints, null, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores
        final double[] qualityScores = new double[
                PROMedSRadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS];
        estimator = new PROMedSRadialDistortionRobustEstimator(qualityScores);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS,
                estimator.getNumKParams());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        final double[] emptyScores = new double[0];
        estimator = null;
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores and listener
        estimator = new PROMedSRadialDistortionRobustEstimator(qualityScores, this);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
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
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with points and quality scores
        estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints,
                undistortedPoints, qualityScores);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS,
                estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                    undistortedPoints, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                    emptyPoints, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    null, undistortedPoints, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    distortedPoints, null, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    distortedPoints, undistortedPoints, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with points, quality scores and listener
        estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints,
                undistortedPoints, qualityScores, this);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS,
                estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                    undistortedPoints, qualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                    emptyPoints, qualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    null, undistortedPoints, qualityScores,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    distortedPoints, null, qualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    distortedPoints, undistortedPoints, emptyScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with points, quality scores and center
        estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints,
                undistortedPoints, qualityScores, center);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(center.getInhomX(), estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0);
        assertEquals(center.getInhomY(), estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                    undistortedPoints, qualityScores, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                    emptyPoints, qualityScores, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    null, undistortedPoints, qualityScores, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    distortedPoints, null, qualityScores, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    distortedPoints, undistortedPoints, emptyScores, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with points, center and listener
        estimator = new PROMedSRadialDistortionRobustEstimator(distortedPoints,
                undistortedPoints, qualityScores, center, this);

        // check correctness
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(center.getInhomX(), estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0);
        assertEquals(center.getInhomY(), estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS,
                estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                    undistortedPoints, qualityScores, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(emptyPoints,
                    emptyPoints, qualityScores, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    null, undistortedPoints, qualityScores,
                    center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    distortedPoints, null, qualityScores, center,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSRadialDistortionRobustEstimator(
                    distortedPoints, undistortedPoints, emptyScores, center,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        final PROMedSRadialDistortionRobustEstimator estimator =
                new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);

        // set new value
        estimator.setStopThreshold(10.0);

        // check correctness
        assertEquals(10.0, estimator.getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final PROMedSRadialDistortionRobustEstimator estimator =
                new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        final double[] qualityScores = new double[
                PROMedSRadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        double[] emptyScores = new double[1];
        try {
            estimator.setQualityScores(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final PROMedSRadialDistortionRobustEstimator estimator =
                new PROMedSRadialDistortionRobustEstimator();

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
    public void testGetSetProgressDelta() throws LockedException {
        final PROMedSRadialDistortionRobustEstimator estimator =
                new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5, estimator.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        final PROMedSRadialDistortionRobustEstimator estimator =
                new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.75);

        // check correctness
        assertEquals(0.75, estimator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxIterations() throws LockedException {
        final PROMedSRadialDistortionRobustEstimator estimator =
                new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertEquals(PROMedSRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(10);

        // check correctness
        assertEquals(10, estimator.getMaxIterations());

        // Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetPoints() throws LockedException {
        final List<Point2D> distortedPoints = new ArrayList<>();
        final List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        final PROMedSRadialDistortionRobustEstimator estimator =
                new PROMedSRadialDistortionRobustEstimator();

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
        final double[] qualityScores = new double[distortedPoints.size()];
        estimator.setQualityScores(qualityScores);

        assertTrue(estimator.isReady());

        // clearing list makes instance not ready
        distortedPoints.clear();

        assertFalse(estimator.isReady());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();
        try {
            estimator.setPoints(emptyPoints, undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setPoints(emptyPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setPoints(null, undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setPoints(distortedPoints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetDistortionCenter() throws LockedException {
        final PROMedSRadialDistortionRobustEstimator estimator =
                new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertNull(estimator.getDistortionCenter());

        // set new value
        final Point2D center = Point2D.create();
        estimator.setDistortionCenter(center);

        // check correctness
        assertSame(center, estimator.getDistortionCenter());
    }

    @Test
    public void testGetSetHorizontalFocalLength() throws LockedException {
        final PROMedSRadialDistortionRobustEstimator estimator =
                new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);

        // set new value
        estimator.setHorizontalFocalLength(2.0);

        // check correctness
        assertEquals(2.0, estimator.getHorizontalFocalLength(), 0.0);
    }

    @Test
    public void testGetSetVerticalFocalLength() throws LockedException {
        final PROMedSRadialDistortionRobustEstimator estimator =
                new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);

        // set new value
        estimator.setVerticalFocalLength(2.0);

        // check correctness
        assertEquals(2.0, estimator.getVerticalFocalLength(), 0.0);
    }

    @Test
    public void testGetSetSkew() throws LockedException {
        final PROMedSRadialDistortionRobustEstimator estimator =
                new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);

        // set new value
        estimator.setSkew(1.0);

        // check correctness
        assertEquals(1.0, estimator.getSkew(), 0.0);
    }

    @Test
    public void testGetSetIntrinsic() throws LockedException {
        final PROMedSRadialDistortionRobustEstimator estimator =
                new PROMedSRadialDistortionRobustEstimator();

        // check default value
        PinholeCameraIntrinsicParameters intrinsic = estimator.getIntrinsic();

        assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                intrinsic.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                intrinsic.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, intrinsic.getSkewness(), 0.0);

        // set new value
        intrinsic = new PinholeCameraIntrinsicParameters(
                2.0, 3.0,
                4.0, 5.0, 6.0);
        estimator.setIntrinsic(intrinsic);

        // check correctness
        assertEquals(4.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(5.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
        assertEquals(2.0, intrinsic.getHorizontalFocalLength(), 0.0);
        assertEquals(3.0, intrinsic.getVerticalFocalLength(), 0.0);
        assertEquals(6.0, intrinsic.getSkewness(), 0.0);

        // set again
        estimator.setIntrinsic(new InhomogeneousPoint2D(6.0, 5.0),
                4.0, 3.0, 2.0);

        // check correctness
        assertEquals(6.0, estimator.getDistortionCenter().getInhomX(), 0.0);
        assertEquals(5.0, estimator.getDistortionCenter().getInhomY(), 0.0);
        assertEquals(4.0, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(3.0, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(2.0, estimator.getSkew(), 0.0);
    }

    @Test
    public void testGetSetNumKParams() throws LockedException {
        final PROMedSRadialDistortionRobustEstimator estimator =
                new PROMedSRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS,
                estimator.getNumKParams());

        // set new value
        estimator.setNumKParams(3);

        // check correctness
        assertEquals(3, estimator.getNumKParams());
    }

    @Test
    public void testEstimate() throws NotSupportedException, LockedException,
            NotReadyException, RobustEstimatorException, DistortionException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        int numValid = 0;
        for (int j = 0; j < TIMES; j++) {
            final double k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
            final double k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);

            final Point2D center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

            final RadialDistortion distortion = new RadialDistortion(k1, k2, center);

            final int nPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final List<Point2D> distortedPointsWithError = new ArrayList<>();
            final List<Point2D> distortedPoints = new ArrayList<>();
            final List<Point2D> undistortedPoints = new ArrayList<>();
            final double[] qualityScores = new double[nPoints];
            Point2D distortedPoint;
            Point2D distortedPointWithError;
            Point2D undistortedPoint;
            for (int i = 0; i < nPoints; i++) {
                undistortedPoint = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                        randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

                distortedPoint = distortion.distort(undistortedPoint);
                final double scoreError = randomizer.nextDouble(MIN_SCORE_ERROR, MAX_SCORE_ERROR);
                qualityScores[i] = 1.0 + scoreError;
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    distortedPointWithError = new InhomogeneousPoint2D(
                            distortedPoint.getInhomX() + errorX,
                            distortedPoint.getInhomY() + errorY);

                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    qualityScores[i] = 1.0 / (1.0 + error) + scoreError;
                } else {
                    // point is inlier
                    distortedPointWithError = distortedPoint;
                }
                distortedPoints.add(distortedPoint);
                distortedPointsWithError.add(distortedPointWithError);
                undistortedPoints.add(undistortedPoint);
            }

            final PROMedSRadialDistortionRobustEstimator estimator =
                    new PROMedSRadialDistortionRobustEstimator(distortedPointsWithError,
                            undistortedPoints, qualityScores, this);

            estimator.setIntrinsic(distortion.getIntrinsic());
            estimator.setStopThreshold(THRESHOLD);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final RadialDistortion distortion2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation
            assertEquals(distortion2.getK1(), k1, ABSOLUTE_ERROR);
            assertEquals(distortion2.getK2(), k2, ABSOLUTE_ERROR);
            assertEquals(distortion2.getCenter(), center);

            boolean failed = false;
            for (int i = 0; i < nPoints; i++) {
                if (distortedPoints.get(i).distanceTo(
                        distortion2.distort(undistortedPoints.get(i))) > ABSOLUTE_ERROR) {
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
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
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
    public void onEstimateNextIteration(
            final RadialDistortionRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        testLocked((PROMedSRadialDistortionRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final RadialDistortionRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        testLocked((PROMedSRadialDistortionRobustEstimator) estimator);
    }

    private void testLocked(final PROMedSRadialDistortionRobustEstimator estimator) {
        try {
            estimator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setDistortionCenter(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMaxIterations(10);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPoints(null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setStopThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setQualityScores(null);
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
        assertTrue(estimator.isLocked());
    }
}
