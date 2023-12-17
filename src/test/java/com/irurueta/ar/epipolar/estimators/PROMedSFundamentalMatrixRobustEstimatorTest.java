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
package com.irurueta.ar.epipolar.estimators;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidFundamentalMatrixException;
import com.irurueta.ar.epipolar.InvalidPairOfCamerasException;
import com.irurueta.geometry.*;
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

public class PROMedSFundamentalMatrixRobustEstimatorTest implements
        FundamentalMatrixRobustEstimatorListener {

    private static final int MIN_REQUIRED_POINTS_AFFINE = 4;
    private static final int MIN_REQUIRED_POINTS_7 = 7;
    private static final int MIN_REQUIRED_POINTS_8 = 8;

    private static final int MIN_POINTS = 100;
    private static final int MAX_POINTS = 500;

    private static final double ABSOLUTE_ERROR = 5e-6;

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

    private static final int TIMES = 100;

    private static final int PERCENTAGE_OUTLIERS = 10;

    private static final double STD_ERROR = 10.0;

    private static final double STOP_THRESHOLD = 1e-9;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    public void testConstructor() {
        // test constructor without arguments
        PROMedSFundamentalMatrixRobustEstimator estimator = new PROMedSFundamentalMatrixRobustEstimator();

        // check correctness
        assertEquals(PROMedSFundamentalMatrixRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with listener
        estimator = new PROMedSFundamentalMatrixRobustEstimator(this);

        // check correctness
        assertEquals(PROMedSFundamentalMatrixRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with left and right points
        final List<Point2D> leftPoints = new ArrayList<>();
        final List<Point2D> rightPoints = new ArrayList<>();
        for (int i = 0; i < MIN_REQUIRED_POINTS_8; i++) {
            leftPoints.add(Point2D.create());
            rightPoints.add(Point2D.create());
        }

        estimator = new PROMedSFundamentalMatrixRobustEstimator(leftPoints, rightPoints);

        // check correctness
        assertEquals(PROMedSFundamentalMatrixRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();
        estimator = null;
        try {
            estimator = new PROMedSFundamentalMatrixRobustEstimator(emptyPoints, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSFundamentalMatrixRobustEstimator(leftPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSFundamentalMatrixRobustEstimator(emptyPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with left and right points and listener
        estimator = new PROMedSFundamentalMatrixRobustEstimator(leftPoints, rightPoints, this);

        // check correctness
        assertEquals(PROMedSFundamentalMatrixRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSFundamentalMatrixRobustEstimator(emptyPoints,
                    rightPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSFundamentalMatrixRobustEstimator(leftPoints,
                    emptyPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSFundamentalMatrixRobustEstimator(emptyPoints,
                    emptyPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with quality scores
        final double[] qualityScores = new double[MIN_REQUIRED_POINTS_8];
        estimator = new PROMedSFundamentalMatrixRobustEstimator(qualityScores);

        // check correctness
        assertEquals(PROMedSFundamentalMatrixRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        final double[] emptyScores = new double[0];
        estimator = null;
        try {
            estimator = new PROMedSFundamentalMatrixRobustEstimator(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with listener
        estimator = new PROMedSFundamentalMatrixRobustEstimator(qualityScores, this);

        // check correctness
        assertEquals(PROMedSFundamentalMatrixRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertFalse(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSFundamentalMatrixRobustEstimator(emptyScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with left and right points and quality scores
        estimator = new PROMedSFundamentalMatrixRobustEstimator(qualityScores, leftPoints, rightPoints);

        // check correctness
        assertEquals(PROMedSFundamentalMatrixRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSFundamentalMatrixRobustEstimator(emptyScores, leftPoints, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
        try {
            estimator = new PROMedSFundamentalMatrixRobustEstimator(emptyScores, emptyPoints, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSFundamentalMatrixRobustEstimator(emptyScores, leftPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSFundamentalMatrixRobustEstimator(emptyScores, emptyPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with left and right points, listener and quality
        // scores
        estimator = new PROMedSFundamentalMatrixRobustEstimator(qualityScores,
                leftPoints, rightPoints, this);

        // check correctness
        assertEquals(PROMedSFundamentalMatrixRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROMedSFundamentalMatrixRobustEstimator(emptyScores, leftPoints, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSFundamentalMatrixRobustEstimator(emptyScores,
                    emptyPoints, rightPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSFundamentalMatrixRobustEstimator(emptyScores,
                    leftPoints, emptyPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROMedSFundamentalMatrixRobustEstimator(emptyScores,
                    emptyPoints, emptyPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetNonRobustFundamentalMatrixEstimatorMethod() throws LockedException {
        final PROMedSFundamentalMatrixRobustEstimator estimator =
                new PROMedSFundamentalMatrixRobustEstimator();

        // check default value
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());

        // set new value
        estimator.setNonRobustFundamentalMatrixEstimatorMethod(
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);

        // check correctness
        assertEquals(FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_8, estimator.getMinRequiredPoints());
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        final PROMedSFundamentalMatrixRobustEstimator estimator =
                new PROMedSFundamentalMatrixRobustEstimator();

        // check default value
        assertEquals(PROMedSFundamentalMatrixRobustEstimator.DEFAULT_STOP_THRESHOLD,
                estimator.getStopThreshold(), 0.0);

        // set new value
        estimator.setStopThreshold(0.5);

        // check correctness
        assertEquals(0.5, estimator.getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetPointsAndIsReady() throws LockedException {
        final PROMedSFundamentalMatrixRobustEstimator estimator =
                new PROMedSFundamentalMatrixRobustEstimator();

        // check default values
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertFalse(estimator.isReady());

        // sets new values
        final List<Point2D> leftPoints = new ArrayList<>();
        final List<Point2D> rightPoints = new ArrayList<>();
        for (int i = 0; i < MIN_REQUIRED_POINTS_8; i++) {
            leftPoints.add(Point2D.create());
            rightPoints.add(Point2D.create());
        }

        estimator.setPoints(leftPoints, rightPoints);

        // check correctness
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertFalse(estimator.isReady());

        // if we provide quality scores, then estimator becomes ready
        final double[] qualityScores = new double[MIN_REQUIRED_POINTS_8];
        estimator.setQualityScores(qualityScores);

        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();
        try {
            estimator.setPoints(emptyPoints, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setPoints(leftPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setPoints(emptyPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListenerAndIsListenerAvailable() throws LockedException {
        final PROMedSFundamentalMatrixRobustEstimator estimator =
                new PROMedSFundamentalMatrixRobustEstimator();

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
    public void testGetSetProgressDelta() throws LockedException {
        final PROMedSFundamentalMatrixRobustEstimator estimator =
                new PROMedSFundamentalMatrixRobustEstimator();

        // check default value
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5f, estimator.getProgressDelta(), 0.0);

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
        final PROMedSFundamentalMatrixRobustEstimator estimator =
                new PROMedSFundamentalMatrixRobustEstimator();

        // check default value
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.5);

        // check correctness
        assertEquals(0.5, estimator.getConfidence(), 0.0);

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
        final PROMedSFundamentalMatrixRobustEstimator estimator =
                new PROMedSFundamentalMatrixRobustEstimator();

        // check default value
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS,
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
    public void testIsSetResultRefined() throws LockedException {
        final PROMedSFundamentalMatrixRobustEstimator estimator =
                new PROMedSFundamentalMatrixRobustEstimator();

        // check default value
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(
                !FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT);

        // check correctness
        assertEquals(!FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT,
                estimator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final PROMedSFundamentalMatrixRobustEstimator estimator =
                new PROMedSFundamentalMatrixRobustEstimator();

        // check default value
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(
                !FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE);

        // check correctness
        assertEquals(!FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE,
                estimator.isCovarianceKept());
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final PROMedSFundamentalMatrixRobustEstimator estimator =
                new PROMedSFundamentalMatrixRobustEstimator();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        final double[] qualityScores = new double[MIN_REQUIRED_POINTS_8];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        final double[] emptyScores = new double[0];
        try {
            estimator.setQualityScores(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testEstimateSevenPointsWithRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, InvalidFundamentalMatrixException, NotAvailableException {
        double leftEpipoleError;
        double rightEpipoleError;
        double avgLeftEpipoleError = 0.0;
        double avgRightEpipoleError = 0.0;
        int numCovars = 0;
        int numValid = 0;
        for (int j = 0; j < TIMES; j++) {
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

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

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

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            // generate a random list of 3D points
            final List<Point3D> points3D = new ArrayList<>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final List<Point2D> leftPoints = camera1.project(points3D);
            final List<Point2D> rightPoints = camera2.project(points3D);

            // add outliers
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final double[] qualityScores = new double[nPoints];
            final List<Point2D> leftPointsWithError = new ArrayList<>();
            int pos = 0;
            for (final Point2D leftPoint : leftPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    leftPointsWithError.add(new HomogeneousPoint2D(
                            leftPoint.getInhomX() + errorX,
                            leftPoint.getInhomY() + errorY, 1.0));

                    qualityScores[pos] = 1.0 / (1.0 + error);
                } else {
                    // inlier
                    leftPointsWithError.add(leftPoint);
                    qualityScores[pos] = 1.0;
                }
                pos++;
            }

            final List<Point2D> rightPointsWithError = new ArrayList<>();
            pos = 0;
            for (final Point2D rightPoint : rightPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    rightPointsWithError.add(new HomogeneousPoint2D(
                            rightPoint.getInhomX() + errorX,
                            rightPoint.getInhomY() + errorY, 1.0));
                    qualityScores[pos] += 1.0 / (1.0 + error);
                } else {
                    // inlier
                    rightPointsWithError.add(rightPoint);
                    qualityScores[pos] += 1.0;
                }
                pos++;
            }

            // estimate fundamental matrix
            final PROMedSFundamentalMatrixRobustEstimator estimator =
                    new PROMedSFundamentalMatrixRobustEstimator(qualityScores,
                            leftPointsWithError, rightPointsWithError, this);
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            final FundamentalMatrix fundMatrix = estimator.estimate();
            assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            if (estimator.getCovariance() != null) {
                numCovars++;
            }

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // compute epipoles
            final Point2D epipole1a = camera1.project(center2);
            final Point2D epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            final Point2D epipole1b = fundMatrix.getLeftEpipole();
            final Point2D epipole2b = fundMatrix.getRightEpipole();

            // check correctness of epipoles
            leftEpipoleError = epipole1a.distanceTo(epipole1b);
            rightEpipoleError = epipole2a.distanceTo(epipole2b);
            if (leftEpipoleError > ABSOLUTE_ERROR) {
                continue;
            }
            if (rightEpipoleError > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, leftEpipoleError, ABSOLUTE_ERROR);
            assertEquals(0.0, rightEpipoleError, ABSOLUTE_ERROR);

            avgLeftEpipoleError += leftEpipoleError;
            avgRightEpipoleError += rightEpipoleError;

            // check that all points lie within their corresponding epipolar
            // lines
            for (int i = 0; i < nPoints; i++) {
                final Point2D leftPoint = leftPoints.get(i);
                final Point2D rightPoint = rightPoints.get(i);
                final Point3D point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

                // obtain epipolar planes
                final Plane epipolarPlane1 = camera1.backProject(line1);
                final Plane epipolarPlane2 = camera2.backProject(line2);

                // check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, ABSOLUTE_ERROR));

                // check that point3D and camera centers belong to epipolar plane
                assertTrue(epipolarPlane1.isLocus(point3D, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center2, ABSOLUTE_ERROR));

                assertTrue(epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));
            }

            numValid++;
        }

        assertTrue(numValid > 0);

        avgLeftEpipoleError /= numValid;
        avgRightEpipoleError /= numValid;

        assertEquals(0.0, avgLeftEpipoleError, ABSOLUTE_ERROR);
        assertEquals(0.0, avgRightEpipoleError, ABSOLUTE_ERROR);
        assertTrue(numCovars > 0);

        // Force NotReadyException
        final PROMedSFundamentalMatrixRobustEstimator estimator =
                new PROMedSFundamentalMatrixRobustEstimator();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateEightPointsWithRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, InvalidFundamentalMatrixException, NotAvailableException {
        double leftEpipoleError;
        double rightEpipoleError;
        double avgLeftEpipoleError = 0.0;
        double avgRightEpipoleError = 0.0;
        int numValid = 0;
        for (int j = 0; j < TIMES; j++) {
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

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

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

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            // generate a random list of 3D points
            final List<Point3D> points3D = new ArrayList<>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final List<Point2D> leftPoints = camera1.project(points3D);
            final List<Point2D> rightPoints = camera2.project(points3D);

            // add outliers
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final double[] qualityScores = new double[nPoints];
            final List<Point2D> leftPointsWithError = new ArrayList<>();
            int pos = 0;
            for (final Point2D leftPoint : leftPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    leftPointsWithError.add(new HomogeneousPoint2D(
                            leftPoint.getInhomX() + errorX,
                            leftPoint.getInhomY() + errorY, 1.0));

                    qualityScores[pos] = 1.0 / (1.0 + error);
                } else {
                    // inlier
                    leftPointsWithError.add(leftPoint);
                    qualityScores[pos] = 1.0;
                }
                pos++;
            }

            final List<Point2D> rightPointsWithError = new ArrayList<>();
            pos = 0;
            for (final Point2D rightPoint : rightPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    rightPointsWithError.add(new HomogeneousPoint2D(
                            rightPoint.getInhomX() + errorX,
                            rightPoint.getInhomY() + errorY, 1.0));
                    qualityScores[pos] += 1.0 / (1.0 + error);
                } else {
                    // inlier
                    rightPointsWithError.add(rightPoint);
                    qualityScores[pos] += 1.0;
                }
                pos++;
            }

            // estimate fundamental matrix
            final PROMedSFundamentalMatrixRobustEstimator estimator =
                    new PROMedSFundamentalMatrixRobustEstimator(
                            FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM,
                            qualityScores, leftPointsWithError, rightPointsWithError, this);
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            final FundamentalMatrix fundMatrix = estimator.estimate();
            assertEquals(MIN_REQUIRED_POINTS_8, estimator.getMinRequiredPoints());
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNotNull(estimator.getCovariance());

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // compute epipoles
            final Point2D epipole1a = camera1.project(center2);
            final Point2D epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            final Point2D epipole1b = fundMatrix.getLeftEpipole();
            final Point2D epipole2b = fundMatrix.getRightEpipole();

            // check correctness of epipoles
            leftEpipoleError = epipole1a.distanceTo(epipole1b);
            rightEpipoleError = epipole2a.distanceTo(epipole2b);
            if (Math.abs(leftEpipoleError) > ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(rightEpipoleError) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, leftEpipoleError, ABSOLUTE_ERROR);
            assertEquals(0.0, rightEpipoleError, ABSOLUTE_ERROR);

            avgLeftEpipoleError += leftEpipoleError;
            avgRightEpipoleError += rightEpipoleError;

            // check that all points lie within their corresponding epipolar
            // lines
            for (int i = 0; i < nPoints; i++) {
                final Point2D leftPoint = leftPoints.get(i);
                final Point2D rightPoint = rightPoints.get(i);
                final Point3D point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

                // obtain epipolar planes
                final Plane epipolarPlane1 = camera1.backProject(line1);
                final Plane epipolarPlane2 = camera2.backProject(line2);

                // check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, ABSOLUTE_ERROR));

                // check that point3D and camera centers belong to epipolar plane
                assertTrue(epipolarPlane1.isLocus(point3D, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center2, ABSOLUTE_ERROR));

                assertTrue(epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));
            }

            numValid++;
        }

        assertTrue(numValid > 0);

        avgLeftEpipoleError /= numValid;
        avgRightEpipoleError /= numValid;

        assertEquals(0.0, avgLeftEpipoleError, ABSOLUTE_ERROR);
        assertEquals(0.0, avgRightEpipoleError, ABSOLUTE_ERROR);

        // Force NotReadyException
        final PROMedSFundamentalMatrixRobustEstimator estimator =
                new PROMedSFundamentalMatrixRobustEstimator();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateAffineWithRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, InvalidFundamentalMatrixException, NotAvailableException,
            WrongSizeException, InvalidPairOfCamerasException {

        int numValid = 0;
        for (int j = 0; j < TIMES; j++) {
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

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

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

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            // convert cameras into affine cameras
            final Matrix cameraMatrix1 = camera1.getInternalMatrix();
            cameraMatrix1.setElementAt(2, 0, 0.0);
            cameraMatrix1.setElementAt(2, 1, 0.0);
            cameraMatrix1.setElementAt(2, 2, 0.0);
            cameraMatrix1.setElementAt(2, 3, 1.0);
            camera1.setInternalMatrix(cameraMatrix1);

            final Matrix cameraMatrix2 = camera2.getInternalMatrix();
            cameraMatrix2.setElementAt(2, 0, 0.0);
            cameraMatrix2.setElementAt(2, 1, 0.0);
            cameraMatrix2.setElementAt(2, 2, 0.0);
            cameraMatrix2.setElementAt(2, 3, 1.0);
            camera2.setInternalMatrix(cameraMatrix2);

            // generate a random list of 3D points
            final List<Point3D> points3D = new ArrayList<>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final List<Point2D> leftPoints = camera1.project(points3D);
            final List<Point2D> rightPoints = camera2.project(points3D);

            // add outliers
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final double[] qualityScores = new double[nPoints];
            final List<Point2D> leftPointsWithError = new ArrayList<>();
            int pos = 0;
            for (final Point2D leftPoint : leftPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    leftPointsWithError.add(new HomogeneousPoint2D(
                            leftPoint.getInhomX() + errorX,
                            leftPoint.getInhomY() + errorY, 1.0));

                    qualityScores[pos] = 1.0 / (1.0 + error);
                } else {
                    // inlier
                    leftPointsWithError.add(leftPoint);
                    qualityScores[pos] = 1.0;
                }
                pos++;
            }

            final List<Point2D> rightPointsWithError = new ArrayList<>();
            pos = 0;
            for (final Point2D rightPoint : rightPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    rightPointsWithError.add(new HomogeneousPoint2D(
                            rightPoint.getInhomX() + errorX,
                            rightPoint.getInhomY() + errorY, 1.0));
                    qualityScores[pos] += 1.0 / (1.0 + error);
                } else {
                    // inlier
                    rightPointsWithError.add(rightPoint);
                    qualityScores[pos] += 1.0;
                }
                pos++;
            }

            // estimate fundamental matrix
            final PROMedSFundamentalMatrixRobustEstimator estimator =
                    new PROMedSFundamentalMatrixRobustEstimator(
                            FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM,
                            qualityScores, leftPointsWithError, rightPointsWithError,
                            this);
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            final FundamentalMatrix fundMatrix = estimator.estimate();
            assertEquals(MIN_REQUIRED_POINTS_AFFINE, estimator.getMinRequiredPoints());
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // compute epipoles
            final FundamentalMatrix fundMatrix2 = new FundamentalMatrix(camera1, camera2);
            fundMatrix2.computeEpipoles();

            final Point2D epipole1a = fundMatrix2.getLeftEpipole();
            final Point2D epipole2a = fundMatrix2.getRightEpipole();

            fundMatrix.computeEpipoles();

            final Point2D epipole1b = fundMatrix.getLeftEpipole();
            final Point2D epipole2b = fundMatrix.getRightEpipole();

            // check correctness of epipoles
            if (!epipole1a.equals(epipole1b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipole1a.equals(epipole1b, ABSOLUTE_ERROR));
            if (!epipole2a.equals(epipole2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipole2a.equals(epipole2b, ABSOLUTE_ERROR));

            fundMatrix.normalize();
            fundMatrix2.normalize();

            // check that both matrices are equal up to scale (i.e. sign)
            if (!fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                    .multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundMatrix.getInternalMatrix().equals(
                    fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                    .multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR));

            // check that all points lie within their corresponding epipolar
            // lines
            for (int i = 0; i < nPoints; i++) {
                final Point2D leftPoint = leftPoints.get(i);
                final Point2D rightPoint = rightPoints.get(i);
                final Point3D point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

                // obtain epipolar planes
                final Plane epipolarPlane1 = camera1.backProject(line1);
                final Plane epipolarPlane2 = camera2.backProject(line2);

                // check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, ABSOLUTE_ERROR));

                // check that point3D and camera centers belong to epipolar plane
                if (!epipolarPlane1.isLocus(point3D, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(point3D, ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center2, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center2, ABSOLUTE_ERROR));

                if (!epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // Force NotReadyException
        final LMedSFundamentalMatrixRobustEstimator estimator =
                new LMedSFundamentalMatrixRobustEstimator();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateSevenPointsWithoutRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, InvalidFundamentalMatrixException, NotAvailableException {
        double leftEpipoleError;
        double rightEpipoleError;
        double avgLeftEpipoleError = 0.0;
        double avgRightEpipoleError = 0.0;
        int numValid = 0;
        for (int j = 0; j < TIMES; j++) {
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

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

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

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            // generate a random list of 3D points
            final List<Point3D> points3D = new ArrayList<>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final List<Point2D> leftPoints = camera1.project(points3D);
            final List<Point2D> rightPoints = camera2.project(points3D);

            // add outliers
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final double[] qualityScores = new double[nPoints];
            final List<Point2D> leftPointsWithError = new ArrayList<>();
            int pos = 0;
            for (final Point2D leftPoint : leftPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    leftPointsWithError.add(new HomogeneousPoint2D(
                            leftPoint.getInhomX() + errorX,
                            leftPoint.getInhomY() + errorY, 1.0));

                    qualityScores[pos] = 1.0 / (1.0 + error);
                } else {
                    // inlier
                    leftPointsWithError.add(leftPoint);
                    qualityScores[pos] = 1.0;
                }
                pos++;
            }

            final List<Point2D> rightPointsWithError = new ArrayList<>();
            pos = 0;
            for (final Point2D rightPoint : rightPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    rightPointsWithError.add(new HomogeneousPoint2D(
                            rightPoint.getInhomX() + errorX,
                            rightPoint.getInhomY() + errorY, 1.0));
                    qualityScores[pos] += 1.0 / (1.0 + error);
                } else {
                    // inlier
                    rightPointsWithError.add(rightPoint);
                    qualityScores[pos] += 1.0;
                }
            }

            // estimate fundamental matrix
            final PROMedSFundamentalMatrixRobustEstimator estimator =
                    new PROMedSFundamentalMatrixRobustEstimator(qualityScores,
                            leftPointsWithError, rightPointsWithError, this);
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            final FundamentalMatrix fundMatrix = estimator.estimate();
            assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNull(estimator.getCovariance());

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // compute epipoles
            final Point2D epipole1a = camera1.project(center2);
            final Point2D epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            final Point2D epipole1b = fundMatrix.getLeftEpipole();
            final Point2D epipole2b = fundMatrix.getRightEpipole();

            // check correctness of epipoles
            leftEpipoleError = epipole1a.distanceTo(epipole1b);
            rightEpipoleError = epipole2a.distanceTo(epipole2b);
            if (leftEpipoleError > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, leftEpipoleError, ABSOLUTE_ERROR);
            if (rightEpipoleError > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, rightEpipoleError, ABSOLUTE_ERROR);

            avgLeftEpipoleError += leftEpipoleError;
            avgRightEpipoleError += rightEpipoleError;

            // check that all points lie within their corresponding epipolar
            // lines
            for (int i = 0; i < nPoints; i++) {
                final Point2D leftPoint = leftPoints.get(i);
                final Point2D rightPoint = rightPoints.get(i);
                final Point3D point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

                // obtain epipolar planes
                final Plane epipolarPlane1 = camera1.backProject(line1);
                final Plane epipolarPlane2 = camera2.backProject(line2);

                // check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, ABSOLUTE_ERROR));

                // check that point3D and camera centers belong to epipolar plane
                assertTrue(epipolarPlane1.isLocus(point3D, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center2, ABSOLUTE_ERROR));

                assertTrue(epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));
            }
            numValid++;
        }

        assertTrue(numValid > 0);

        avgLeftEpipoleError /= TIMES;
        avgRightEpipoleError /= TIMES;

        assertEquals(0.0, avgLeftEpipoleError, ABSOLUTE_ERROR);
        assertEquals(0.0, avgRightEpipoleError, ABSOLUTE_ERROR);

        // Force NotReadyException
        final PROMedSFundamentalMatrixRobustEstimator estimator =
                new PROMedSFundamentalMatrixRobustEstimator();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateEightPointsWithoutRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, InvalidFundamentalMatrixException, NotAvailableException {
        double leftEpipoleError;
        double rightEpipoleError;
        double avgLeftEpipoleError = 0.0;
        double avgRightEpipoleError = 0.0;
        int numValid = 0;
        for (int j = 0; j < TIMES; j++) {
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

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

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

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            // generate a random list of 3D points
            final List<Point3D> points3D = new ArrayList<>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final List<Point2D> leftPoints = camera1.project(points3D);
            final List<Point2D> rightPoints = camera2.project(points3D);

            // add outliers
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final double[] qualityScores = new double[nPoints];
            final List<Point2D> leftPointsWithError = new ArrayList<>();
            int pos = 0;
            for (final Point2D leftPoint : leftPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    leftPointsWithError.add(new HomogeneousPoint2D(
                            leftPoint.getInhomX() + errorX,
                            leftPoint.getInhomY() + errorY, 1.0));

                    qualityScores[pos] = 1.0 / (1.0 + error);
                } else {
                    // inlier
                    leftPointsWithError.add(leftPoint);
                    qualityScores[pos] = 1.0;
                }
                pos++;
            }

            final List<Point2D> rightPointsWithError = new ArrayList<>();
            pos = 0;
            for (final Point2D rightPoint : rightPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    rightPointsWithError.add(new HomogeneousPoint2D(
                            rightPoint.getInhomX() + errorX,
                            rightPoint.getInhomY() + errorY, 1.0));
                    qualityScores[pos] += 1.0 / (1.0 + error);
                } else {
                    // inlier
                    rightPointsWithError.add(rightPoint);
                    qualityScores[pos] += 1.0;
                }
            }

            // estimate fundamental matrix
            final PROMedSFundamentalMatrixRobustEstimator estimator =
                    new PROMedSFundamentalMatrixRobustEstimator(
                            FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM,
                            qualityScores, leftPointsWithError, rightPointsWithError, this);
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            final FundamentalMatrix fundMatrix = estimator.estimate();
            assertEquals(MIN_REQUIRED_POINTS_8, estimator.getMinRequiredPoints());
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNull(estimator.getCovariance());

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // compute epipoles
            final Point2D epipole1a = camera1.project(center2);
            final Point2D epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            final Point2D epipole1b = fundMatrix.getLeftEpipole();
            final Point2D epipole2b = fundMatrix.getRightEpipole();

            // check correctness of epipoles
            leftEpipoleError = epipole1a.distanceTo(epipole1b);
            rightEpipoleError = epipole2a.distanceTo(epipole2b);
            if (leftEpipoleError > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, leftEpipoleError, ABSOLUTE_ERROR);
            if (rightEpipoleError > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, rightEpipoleError, ABSOLUTE_ERROR);

            avgLeftEpipoleError += leftEpipoleError;
            avgRightEpipoleError += rightEpipoleError;

            // check that all points lie within their corresponding epipolar
            // lines
            for (int i = 0; i < nPoints; i++) {
                final Point2D leftPoint = leftPoints.get(i);
                final Point2D rightPoint = rightPoints.get(i);
                final Point3D point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

                // obtain epipolar planes
                final Plane epipolarPlane1 = camera1.backProject(line1);
                final Plane epipolarPlane2 = camera2.backProject(line2);

                // check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, ABSOLUTE_ERROR));

                // check that point3D and camera centers belong to epipolar plane
                assertTrue(epipolarPlane1.isLocus(point3D, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1.isLocus(center2, ABSOLUTE_ERROR));

                assertTrue(epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));
            }

            numValid++;
        }

        assertTrue(numValid > 0);

        avgLeftEpipoleError /= TIMES;
        avgRightEpipoleError /= TIMES;

        assertEquals(0.0, avgLeftEpipoleError, ABSOLUTE_ERROR);
        assertEquals(0.0, avgRightEpipoleError, ABSOLUTE_ERROR);

        // Force NotReadyException
        final PROMedSFundamentalMatrixRobustEstimator estimator =
                new PROMedSFundamentalMatrixRobustEstimator();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Test
    public void testEstimateAffineWithoutRefinement() throws LockedException, NotReadyException,
            RobustEstimatorException, InvalidFundamentalMatrixException, NotAvailableException,
            WrongSizeException, InvalidPairOfCamerasException {

        int numValid = 0;
        for (int j = 0; j < TIMES; j++) {
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

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

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

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            // convert cameras into affine cameras
            final Matrix cameraMatrix1 = camera1.getInternalMatrix();
            cameraMatrix1.setElementAt(2, 0, 0.0);
            cameraMatrix1.setElementAt(2, 1, 0.0);
            cameraMatrix1.setElementAt(2, 2, 0.0);
            cameraMatrix1.setElementAt(2, 3, 1.0);
            camera1.setInternalMatrix(cameraMatrix1);

            final Matrix cameraMatrix2 = camera2.getInternalMatrix();
            cameraMatrix2.setElementAt(2, 0, 0.0);
            cameraMatrix2.setElementAt(2, 1, 0.0);
            cameraMatrix2.setElementAt(2, 2, 0.0);
            cameraMatrix2.setElementAt(2, 3, 1.0);
            camera2.setInternalMatrix(cameraMatrix2);

            // generate a random list of 3D points
            final List<Point3D> points3D = new ArrayList<>();
            for (int i = 0; i < nPoints; i++) {
                points3D.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }

            // project 3D points with both cameras
            final List<Point2D> leftPoints = camera1.project(points3D);
            final List<Point2D> rightPoints = camera2.project(points3D);

            // add outliers
            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final double[] qualityScores = new double[nPoints];
            final List<Point2D> leftPointsWithError = new ArrayList<>();
            int pos = 0;
            for (final Point2D leftPoint : leftPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    leftPointsWithError.add(new HomogeneousPoint2D(
                            leftPoint.getInhomX() + errorX,
                            leftPoint.getInhomY() + errorY, 1.0));

                    qualityScores[pos] = 1.0 / (1.0 + error);
                } else {
                    // inlier
                    leftPointsWithError.add(leftPoint);
                    qualityScores[pos] = 1.0;
                }
                pos++;
            }

            final List<Point2D> rightPointsWithError = new ArrayList<>();
            pos = 0;
            for (final Point2D rightPoint : rightPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    rightPointsWithError.add(new HomogeneousPoint2D(
                            rightPoint.getInhomX() + errorX,
                            rightPoint.getInhomY() + errorY, 1.0));

                    qualityScores[pos] += 1.0 / (1.0 + error);
                } else {
                    // inlier
                    rightPointsWithError.add(rightPoint);
                    qualityScores[pos] += 1.0;
                }
            }

            // estimate fundamental matrix
            final PROMedSFundamentalMatrixRobustEstimator estimator =
                    new PROMedSFundamentalMatrixRobustEstimator(
                            FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM,
                            qualityScores, leftPointsWithError, rightPointsWithError, this);
            estimator.setStopThreshold(STOP_THRESHOLD);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            final FundamentalMatrix fundMatrix = estimator.estimate();
            assertEquals(MIN_REQUIRED_POINTS_AFFINE,
                    estimator.getMinRequiredPoints());
            assertNotNull(estimator.getInliersData());
            assertNotNull(estimator.getInliersData().getInliers());
            assertNotNull(estimator.getInliersData().getResiduals());
            assertTrue(estimator.getInliersData().getNumInliers() > 0);
            assertNull(estimator.getCovariance());

            // check correctness
            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // compute epipoles
            final FundamentalMatrix fundMatrix2 = new FundamentalMatrix(camera1, camera2);
            fundMatrix2.computeEpipoles();

            final Point2D epipole1a = fundMatrix2.getLeftEpipole();
            final Point2D epipole2a = fundMatrix2.getRightEpipole();

            fundMatrix.computeEpipoles();

            final Point2D epipole1b = fundMatrix.getLeftEpipole();
            final Point2D epipole2b = fundMatrix.getRightEpipole();

            // check correctness of epipoles
            if (!epipole1a.equals(epipole1b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipole1a.equals(epipole1b, ABSOLUTE_ERROR));
            if (!epipole2a.equals(epipole2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipole2a.equals(epipole2b, ABSOLUTE_ERROR));

            fundMatrix.normalize();
            fundMatrix2.normalize();

            // check that both matrices are equal up to scale (i.e. sign)
            if (!fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                    .multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundMatrix.getInternalMatrix().equals(
                    fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                    .multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR));

            // check that all points lie within their corresponding epipolar 
            // lines
            for (int i = 0; i < nPoints; i++) {
                final Point2D leftPoint = leftPoints.get(i);
                final Point2D rightPoint = rightPoints.get(i);
                final Point3D point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final Line2D line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final Line2D line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar 
                // line
                assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

                // obtain epipolar planes
                final Plane epipolarPlane1 = camera1.backProject(line1);
                final Plane epipolarPlane2 = camera2.backProject(line2);

                // check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, ABSOLUTE_ERROR));

                // check that point3D and camera centers belong to epipolar plane
                if (!epipolarPlane1.isLocus(point3D, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(point3D, ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center2, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center2, ABSOLUTE_ERROR));

                if (!epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // Force NotReadyException
        final LMedSFundamentalMatrixRobustEstimator estimator =
                new LMedSFundamentalMatrixRobustEstimator();
        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        }
    }

    @Override
    public void onEstimateStart(final FundamentalMatrixRobustEstimator estimator) {
        estimateStart++;
        checkLocked((PROMedSFundamentalMatrixRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final FundamentalMatrixRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((PROMedSFundamentalMatrixRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(
            final FundamentalMatrixRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((PROMedSFundamentalMatrixRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final FundamentalMatrixRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((PROMedSFundamentalMatrixRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void checkLocked(final PROMedSFundamentalMatrixRobustEstimator estimator) {
        try {
            estimator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(null);
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
            estimator.setNonRobustFundamentalMatrixEstimatorMethod(
                    FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final NotReadyException | RobustEstimatorException e) {
            fail("LockedException expected but not thrown");
        }
        assertTrue(estimator.isLocked());
    }
}
