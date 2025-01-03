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
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class PROSACFundamentalMatrixRobustEstimatorTest implements FundamentalMatrixRobustEstimatorListener {

    private static final int MIN_REQUIRED_POINTS_AFFINE = 4;
    private static final int MIN_REQUIRED_POINTS_7 = 7;
    private static final int MIN_REQUIRED_POINTS_8 = 8;

    private static final int MIN_POINTS = 100;
    private static final int MAX_POINTS = 500;

    private static final double ABSOLUTE_ERROR = 5e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;

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

    private static final int PERCENTAGE_OUTLIERS = 20;

    private static final double STD_ERROR = 10.0;

    private static final double THRESHOLD = 1e-6;

    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;

    @Test
    void testConstructor() {
        // test constructor without arguments
        var estimator = new PROSACFundamentalMatrixRobustEstimator();

        // check correctness
        assertEquals(PROSACFundamentalMatrixRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertNull(estimator.getQualityScores());

        // test constructor with listener
        estimator = new PROSACFundamentalMatrixRobustEstimator(this);

        // check correctness
        assertEquals(PROSACFundamentalMatrixRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertNull(estimator.getQualityScores());

        // test constructor with left and right points
        final var leftPoints = new ArrayList<Point2D>();
        final var rightPoints = new ArrayList<Point2D>();
        for (var i = 0; i < MIN_REQUIRED_POINTS_7; i++) {
            leftPoints.add(Point2D.create());
            rightPoints.add(Point2D.create());
        }

        estimator = new PROSACFundamentalMatrixRobustEstimator(leftPoints, rightPoints);

        // check correctness
        assertEquals(PROSACFundamentalMatrixRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACFundamentalMatrixRobustEstimator(emptyPoints, rightPoints));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACFundamentalMatrixRobustEstimator(leftPoints, emptyPoints));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACFundamentalMatrixRobustEstimator(emptyPoints, emptyPoints));

        // test constructor with left and right points and listener
        estimator = new PROSACFundamentalMatrixRobustEstimator(leftPoints, rightPoints, this);

        // check correctness
        assertEquals(PROSACFundamentalMatrixRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACFundamentalMatrixRobustEstimator(emptyPoints, rightPoints, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACFundamentalMatrixRobustEstimator(leftPoints, emptyPoints, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACFundamentalMatrixRobustEstimator(emptyPoints, emptyPoints, this));

        // test constructor with quality scores
        final var qualityScores = new double[MIN_REQUIRED_POINTS_7];
        estimator = new PROSACFundamentalMatrixRobustEstimator(qualityScores);

        // check correctness
        assertEquals(PROSACFundamentalMatrixRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertSame(estimator.getQualityScores(), qualityScores);

        // Force IllegalArgumentException
        final var emptyScores = new double[0];
        assertThrows(IllegalArgumentException.class, () -> new PROSACFundamentalMatrixRobustEstimator(emptyScores));

        // test constructor with listener
        estimator = new PROSACFundamentalMatrixRobustEstimator(qualityScores, this);

        // check correctness
        assertEquals(PROSACFundamentalMatrixRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertFalse(estimator.isReady());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACFundamentalMatrixRobustEstimator(emptyScores, this));

        // test constructor with left and right points and quality scores
        estimator = new PROSACFundamentalMatrixRobustEstimator(qualityScores, leftPoints, rightPoints);

        // check correctness
        assertEquals(PROSACFundamentalMatrixRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACFundamentalMatrixRobustEstimator(emptyScores, leftPoints, rightPoints));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACFundamentalMatrixRobustEstimator(emptyScores, emptyPoints, rightPoints));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACFundamentalMatrixRobustEstimator(emptyScores, leftPoints, emptyPoints));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACFundamentalMatrixRobustEstimator(emptyScores, emptyPoints, emptyPoints));

        // test constructor with left and right points, listener and quality
        // scores
        estimator = new PROSACFundamentalMatrixRobustEstimator(qualityScores, leftPoints, rightPoints, this);

        // check correctness
        assertEquals(PROSACFundamentalMatrixRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        assertTrue(estimator.isReady());
        assertFalse(estimator.isComputeAndKeepInliersEnabled());
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACFundamentalMatrixRobustEstimator(emptyScores,
                leftPoints, rightPoints, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACFundamentalMatrixRobustEstimator(emptyScores,
                emptyPoints, rightPoints, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACFundamentalMatrixRobustEstimator(emptyScores,
                leftPoints, emptyPoints, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACFundamentalMatrixRobustEstimator(emptyScores,
                emptyPoints, emptyPoints, this));
    }

    @Test
    void testGetSetNonRobustFundamentalMatrixEstimatorMethod() throws LockedException {
        final var estimator = new PROSACFundamentalMatrixRobustEstimator();

        // check default value
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());

        // set new value
        estimator.setNonRobustFundamentalMatrixEstimatorMethod(FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);

        // check correctness
        assertEquals(FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_8, estimator.getMinRequiredPoints());
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var estimator = new PROSACFundamentalMatrixRobustEstimator();

        // check default value
        assertEquals(PROSACFundamentalMatrixRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);

        // set new value
        estimator.setThreshold(0.5);

        // check correctness
        assertEquals(0.5, estimator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setThreshold(0.0));
    }

    @Test
    void testGetSetPointsAndIsReady() throws LockedException {
        final var estimator = new PROSACFundamentalMatrixRobustEstimator();

        // check default values
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertFalse(estimator.isReady());

        // sets new values
        final var leftPoints = new ArrayList<Point2D>();
        final var rightPoints = new ArrayList<Point2D>();
        for (var i = 0; i < MIN_REQUIRED_POINTS_7; i++) {
            leftPoints.add(Point2D.create());
            rightPoints.add(Point2D.create());
        }

        estimator.setPoints(leftPoints, rightPoints);

        // check correctness
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertFalse(estimator.isReady());

        // if we provide quality scores, then estimator becomes ready
        final var qualityScores = new double[MIN_REQUIRED_POINTS_7];
        estimator.setQualityScores(qualityScores);

        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(emptyPoints, rightPoints));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(leftPoints, emptyPoints));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(emptyPoints, emptyPoints));
    }

    @Test
    void testGetSetListenerAndIsListenerAvailable() throws LockedException {
        final var estimator = new PROSACFundamentalMatrixRobustEstimator();

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
        final var estimator = new PROSACFundamentalMatrixRobustEstimator();

        // check default value
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5f, estimator.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(2.0f));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var estimator = new PROSACFundamentalMatrixRobustEstimator();

        // check default value
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.5);

        // check correctness
        assertEquals(0.5, estimator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(2.0));
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var estimator = new PROSACFundamentalMatrixRobustEstimator();

        // check default value
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(10);

        // check correctness
        assertEquals(10, estimator.getMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testIsSetResultRefined() throws LockedException {
        final var estimator = new PROSACFundamentalMatrixRobustEstimator();

        // check default value
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());

        // set new value
        estimator.setResultRefined(!FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT);

        // check correctness
        assertEquals(!FundamentalMatrixRobustEstimator.DEFAULT_REFINE_RESULT, estimator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var estimator = new PROSACFundamentalMatrixRobustEstimator();

        // check default value
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());

        // set new value
        estimator.setCovarianceKept(!FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE);

        // check correctness
        assertEquals(!FundamentalMatrixRobustEstimator.DEFAULT_KEEP_COVARIANCE, estimator.isCovarianceKept());
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var estimator = new PROSACFundamentalMatrixRobustEstimator();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        final var qualityScores = new double[MIN_REQUIRED_POINTS_7];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertSame(qualityScores, estimator.getQualityScores());

        // Force IllegalArgumentException
        final var emptyScores = new double[0];
        assertThrows(IllegalArgumentException.class, () -> estimator.setQualityScores(emptyScores));
    }

    @Test
    void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final var estimator = new PROSACFundamentalMatrixRobustEstimator();

        // check default value
        assertFalse(estimator.isComputeAndKeepInliersEnabled());

        // set new value
        estimator.setComputeAndKeepInliersEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepInliersEnabled());
    }

    @Test
    void testIsSetComputeAndKeepResidualsEnabled() throws LockedException {
        final var estimator = new PROSACFundamentalMatrixRobustEstimator();

        // check default value
        assertFalse(estimator.isComputeAndKeepResidualsEnabled());

        // set new value
        estimator.setComputeAndKeepResidualsEnabled(true);

        // check correctness
        assertTrue(estimator.isComputeAndKeepResidualsEnabled());
    }

    @Test
    void testEstimateSevenPointsWithRefinement() throws LockedException, NotReadyException, RobustEstimatorException,
            InvalidFundamentalMatrixException, NotAvailableException {
        double leftEpipoleError;
        double rightEpipoleError;

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
        final var qualityScores = new double[nPoints];
        final var leftPointsWithError = new ArrayList<Point2D>();
        var pos = 0;
        for (final var leftPoint : leftPoints) {
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                // outlier
                final var errorX = errorRandomizer.nextDouble();
                final var errorY = errorRandomizer.nextDouble();
                final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                leftPointsWithError.add(new HomogeneousPoint2D(leftPoint.getInhomX() + errorX,
                        leftPoint.getInhomY() + errorY, 1.0));

                qualityScores[pos] = 1.0 / (1.0 + error);
            } else {
                // inlier
                leftPointsWithError.add(leftPoint);
                qualityScores[pos] = 1.0;
            }
            pos++;
        }

        final var rightPointsWithError = new ArrayList<Point2D>();
        pos = 0;
        for (final var rightPoint : rightPoints) {
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                // outlier
                final var errorX = errorRandomizer.nextDouble();
                final var errorY = errorRandomizer.nextDouble();
                final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                rightPointsWithError.add(new HomogeneousPoint2D(rightPoint.getInhomX() + errorX,
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
        var estimator = new PROSACFundamentalMatrixRobustEstimator(qualityScores, leftPointsWithError,
                rightPointsWithError, this);
        estimator.setThreshold(THRESHOLD);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);
        estimator.setResultRefined(true);
        estimator.setCovarianceKept(true);

        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getCovariance());

        assertEquals(0, estimateStart);
        assertEquals(0, estimateEnd);
        assertEquals(0, estimateNextIteration);
        assertEquals(0, estimateProgressChange);

        final var fundMatrix = estimator.estimate();
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
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
        final var epipole1a = camera1.project(center2);
        final var epipole2a = camera2.project(center1);

        fundMatrix.computeEpipoles();

        final var epipole1b = fundMatrix.getLeftEpipole();
        final var epipole2b = fundMatrix.getRightEpipole();

        // check correctness of epipoles
        leftEpipoleError = epipole1a.distanceTo(epipole1b);
        rightEpipoleError = epipole2a.distanceTo(epipole2b);
        assertEquals(0.0, leftEpipoleError, 2 * ABSOLUTE_ERROR);
        assertEquals(0.0, rightEpipoleError, 2 * ABSOLUTE_ERROR);

        // check that all points lie within their corresponding epipolar
        // lines
        for (var i = 0; i < nPoints; i++) {
            final var leftPoint = leftPoints.get(i);
            final var rightPoint = rightPoints.get(i);
            final var point3D = points3D.get(i);

            // obtain epipolar line on left view using 2D point on right view
            final var line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
            // obtain epipolar line on right view using 2D point on left view
            final var line2 = fundMatrix.getRightEpipolarLine(leftPoint);

            // check that 2D point on left view belongs to left epipolar line
            assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
            // check that 2D point on right view belongs to right epipolar
            // line
            assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

            // obtain epipolar planes
            final var epipolarPlane1 = camera1.backProject(line1);
            final var epipolarPlane2 = camera2.backProject(line2);

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

        assertEquals(0.0, leftEpipoleError, ABSOLUTE_ERROR);
        assertEquals(0.0, rightEpipoleError, ABSOLUTE_ERROR);

        // Force NotReadyException
        estimator = new PROSACFundamentalMatrixRobustEstimator();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateEightPointsWithRefinement() throws LockedException, NotReadyException, RobustEstimatorException,
            InvalidFundamentalMatrixException, NotAvailableException {
        double leftEpipoleError;
        double rightEpipoleError;
        var avgLeftEpipoleError = 0.0;
        var avgRightEpipoleError = 0.0;
        var numValid = 0;
        for (var j = 0; j < TIMES; j++) {
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
            final var qualityScores = new double[nPoints];
            final var leftPointsWithError = new ArrayList<Point2D>();
            var pos = 0;
            for (final var leftPoint : leftPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                    leftPointsWithError.add(new HomogeneousPoint2D(leftPoint.getInhomX() + errorX,
                            leftPoint.getInhomY() + errorY, 1.0));

                    qualityScores[pos] = 1.0 / (1.0 + error);
                } else {
                    // inlier
                    leftPointsWithError.add(leftPoint);
                    qualityScores[pos] = 1.0;
                }
                pos++;
            }

            final var rightPointsWithError = new ArrayList<Point2D>();
            pos = 0;
            for (final var rightPoint : rightPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                    rightPointsWithError.add(new HomogeneousPoint2D(rightPoint.getInhomX() + errorX,
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
            final var estimator = new PROSACFundamentalMatrixRobustEstimator(
                    FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM, qualityScores, leftPointsWithError,
                    rightPointsWithError, this);
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            final var fundMatrix = estimator.estimate();
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
            final var epipole1a = camera1.project(center2);
            final var epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            final var epipole1b = fundMatrix.getLeftEpipole();
            final var epipole2b = fundMatrix.getRightEpipole();

            // check correctness of epipoles
            leftEpipoleError = epipole1a.distanceTo(epipole1b);
            rightEpipoleError = epipole2a.distanceTo(epipole2b);
            if (Math.abs(leftEpipoleError) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, leftEpipoleError, LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rightEpipoleError) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, rightEpipoleError, LARGE_ABSOLUTE_ERROR);

            avgLeftEpipoleError += leftEpipoleError;
            avgRightEpipoleError += rightEpipoleError;

            // check that all points lie within their corresponding epipolar
            // lines
            var validPoints = true;
            for (var i = 0; i < nPoints; i++) {
                final var leftPoint = leftPoints.get(i);
                final var rightPoint = rightPoints.get(i);
                final var point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final var line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final var line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

                // obtain epipolar planes
                final var epipolarPlane1 = camera1.backProject(line1);
                final var epipolarPlane2 = camera2.backProject(line2);

                // check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, ABSOLUTE_ERROR));

                // check that point3D and camera centers belong to epipolar plane
                if (!epipolarPlane1.isLocus(point3D, 2.0 * ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(epipolarPlane1.isLocus(point3D, 2.0 * ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center1, 2.0 * ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(epipolarPlane1.isLocus(center1, 2.0 * ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center2, 2.0 * ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(epipolarPlane1.isLocus(center2, 2.0 * ABSOLUTE_ERROR));

                if (!epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR));
                if (!epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            numValid++;
        }

        assertTrue(numValid > 0);

        avgLeftEpipoleError /= numValid;
        avgRightEpipoleError /= numValid;

        assertEquals(0.0, avgLeftEpipoleError, ABSOLUTE_ERROR);
        assertEquals(0.0, avgRightEpipoleError, ABSOLUTE_ERROR);

        // Force NotReadyException
        final var estimator = new PROSACFundamentalMatrixRobustEstimator();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateAffineWithRefinement() throws LockedException, NotReadyException, RobustEstimatorException,
            InvalidFundamentalMatrixException, NotAvailableException, WrongSizeException,
            InvalidPairOfCamerasException {

        var numValid = 0;
        for (var j = 0; j < TIMES; j++) {
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

            // convert cameras into affine cameras
            final var cameraMatrix1 = camera1.getInternalMatrix();
            cameraMatrix1.setElementAt(2, 0, 0.0);
            cameraMatrix1.setElementAt(2, 1, 0.0);
            cameraMatrix1.setElementAt(2, 2, 0.0);
            cameraMatrix1.setElementAt(2, 3, 1.0);
            camera1.setInternalMatrix(cameraMatrix1);

            final var cameraMatrix2 = camera2.getInternalMatrix();
            cameraMatrix2.setElementAt(2, 0, 0.0);
            cameraMatrix2.setElementAt(2, 1, 0.0);
            cameraMatrix2.setElementAt(2, 2, 0.0);
            cameraMatrix2.setElementAt(2, 3, 1.0);
            camera2.setInternalMatrix(cameraMatrix2);

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
            final var qualityScores = new double[nPoints];
            final var leftPointsWithError = new ArrayList<Point2D>();
            var pos = 0;
            for (final var leftPoint : leftPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                    leftPointsWithError.add(new HomogeneousPoint2D(leftPoint.getInhomX() + errorX,
                            leftPoint.getInhomY() + errorY, 1.0));

                    qualityScores[pos] = 1.0 / (1.0 + error);
                } else {
                    // inlier
                    leftPointsWithError.add(leftPoint);
                    qualityScores[pos] = 1.0;
                }
                pos++;
            }

            final var rightPointsWithError = new ArrayList<Point2D>();
            pos = 0;
            for (final var rightPoint : rightPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                    rightPointsWithError.add(new HomogeneousPoint2D(rightPoint.getInhomX() + errorX,
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
            final var estimator = new PROSACFundamentalMatrixRobustEstimator(
                    FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM, qualityScores, leftPointsWithError,
                    rightPointsWithError, this);
            estimator.setThreshold(THRESHOLD);
            estimator.setResultRefined(true);
            estimator.setCovarianceKept(true);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            final var fundMatrix = estimator.estimate();
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
            var fundMatrix2 = new FundamentalMatrix(camera1, camera2);
            fundMatrix2.computeEpipoles();

            final var epipole1a = fundMatrix2.getLeftEpipole();
            final var epipole2a = fundMatrix2.getRightEpipole();

            fundMatrix.computeEpipoles();

            final var epipole1b = fundMatrix.getLeftEpipole();
            final var epipole2b = fundMatrix.getRightEpipole();

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
            assertTrue(fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                    .multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR));

            // check that all points lie within their corresponding epipolar
            // lines
            for (var i = 0; i < nPoints; i++) {
                final var leftPoint = leftPoints.get(i);
                final var rightPoint = rightPoints.get(i);
                final var point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final var line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final var line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

                // obtain epipolar planes
                final var epipolarPlane1 = camera1.backProject(line1);
                final var epipolarPlane2 = camera2.backProject(line2);

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
        final var estimator = new LMedSFundamentalMatrixRobustEstimator();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateSevenPointsWithoutRefinement() throws LockedException, NotReadyException, RobustEstimatorException,
            InvalidFundamentalMatrixException, NotAvailableException {
        double leftEpipoleError;
        double rightEpipoleError;

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
        final var qualityScores = new double[nPoints];
        final var leftPointsWithError = new ArrayList<Point2D>();
        var pos = 0;
        for (final var leftPoint : leftPoints) {
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                // outlier
                final var errorX = errorRandomizer.nextDouble();
                final var errorY = errorRandomizer.nextDouble();
                final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                leftPointsWithError.add(new HomogeneousPoint2D(leftPoint.getInhomX() + errorX,
                        leftPoint.getInhomY() + errorY, 1.0));

                qualityScores[pos] = 1.0 / (1.0 + error);
            } else {
                // inlier
                leftPointsWithError.add(leftPoint);
                qualityScores[pos] = 1.0;
            }
            pos++;
        }

        final var rightPointsWithError = new ArrayList<Point2D>();
        pos = 0;
        for (final var rightPoint : rightPoints) {
            if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                // outlier
                final var errorX = errorRandomizer.nextDouble();
                final var errorY = errorRandomizer.nextDouble();
                final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                rightPointsWithError.add(new HomogeneousPoint2D(rightPoint.getInhomX() + errorX,
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
        var estimator = new PROSACFundamentalMatrixRobustEstimator(qualityScores, leftPointsWithError,
                rightPointsWithError, this);
        estimator.setThreshold(THRESHOLD);
        estimator.setComputeAndKeepInliersEnabled(true);
        estimator.setComputeAndKeepResidualsEnabled(true);
        estimator.setResultRefined(false);
        estimator.setCovarianceKept(false);

        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getCovariance());

        assertEquals(0, estimateStart);
        assertEquals(0, estimateEnd);
        assertEquals(0, estimateNextIteration);
        assertEquals(0, estimateProgressChange);

        final var fundMatrix = estimator.estimate();
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
        final var epipole1a = camera1.project(center2);
        final var epipole2a = camera2.project(center1);

        fundMatrix.computeEpipoles();

        final var epipole1b = fundMatrix.getLeftEpipole();
        final var epipole2b = fundMatrix.getRightEpipole();

        // check correctness of epipoles
        leftEpipoleError = epipole1a.distanceTo(epipole1b);
        rightEpipoleError = epipole2a.distanceTo(epipole2b);
        assertEquals(0.0, leftEpipoleError, 2 * ABSOLUTE_ERROR);
        assertEquals(0.0, rightEpipoleError, 2 * ABSOLUTE_ERROR);

        // check that all points lie within their corresponding epipolar
        // lines
        for (var i = 0; i < nPoints; i++) {
            final var leftPoint = leftPoints.get(i);
            final var rightPoint = rightPoints.get(i);
            final var point3D = points3D.get(i);

            // obtain epipolar line on left view using 2D point on right view
            final var line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
            // obtain epipolar line on right view using 2D point on left view
            final var line2 = fundMatrix.getRightEpipolarLine(leftPoint);

            // check that 2D point on left view belongs to left epipolar line
            assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
            // check that 2D point on right view belongs to right epipolar
            // line
            assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

            // obtain epipolar planes
            final var epipolarPlane1 = camera1.backProject(line1);
            final var epipolarPlane2 = camera2.backProject(line2);

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

        assertEquals(0.0, leftEpipoleError, ABSOLUTE_ERROR);
        assertEquals(0.0, rightEpipoleError, ABSOLUTE_ERROR);

        // Force NotReadyException
        estimator = new PROSACFundamentalMatrixRobustEstimator();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateEightPointsWithoutRefinement() throws LockedException, NotReadyException, RobustEstimatorException,
            InvalidFundamentalMatrixException, NotAvailableException {
        double leftEpipoleError;
        double rightEpipoleError;
        var avgLeftEpipoleError = 0.0;
        var avgRightEpipoleError = 0.0;
        var numValid = 0;
        for (var j = 0; j < TIMES; j++) {
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
            final var qualityScores = new double[nPoints];
            final var leftPointsWithError = new ArrayList<Point2D>();
            var pos = 0;
            for (final var leftPoint : leftPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                    leftPointsWithError.add(new HomogeneousPoint2D(leftPoint.getInhomX() + errorX,
                            leftPoint.getInhomY() + errorY, 1.0));

                    qualityScores[pos] = 1.0 / (1.0 + error);
                } else {
                    // inlier
                    leftPointsWithError.add(leftPoint);
                    qualityScores[pos] = 1.0;
                }
                pos++;
            }

            final var rightPointsWithError = new ArrayList<Point2D>();
            pos = 0;
            for (final var rightPoint : rightPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                    rightPointsWithError.add(new HomogeneousPoint2D(rightPoint.getInhomX() + errorX,
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
            final var estimator = new PROSACFundamentalMatrixRobustEstimator(
                    FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM, qualityScores, leftPointsWithError,
                    rightPointsWithError, this);
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            final var fundMatrix = estimator.estimate();
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
            final var epipole1a = camera1.project(center2);
            final var epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            final var epipole1b = fundMatrix.getLeftEpipole();
            final var epipole2b = fundMatrix.getRightEpipole();

            // check correctness of epipoles
            leftEpipoleError = epipole1a.distanceTo(epipole1b);
            rightEpipoleError = epipole2a.distanceTo(epipole2b);
            if (leftEpipoleError > 2 * LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, leftEpipoleError, 2 * LARGE_ABSOLUTE_ERROR);
            if (rightEpipoleError > 2 * LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, rightEpipoleError, 2 * LARGE_ABSOLUTE_ERROR);

            avgLeftEpipoleError += leftEpipoleError;
            avgRightEpipoleError += rightEpipoleError;

            // check that all points lie within their corresponding epipolar
            // lines
            for (var i = 0; i < nPoints; i++) {
                final var leftPoint = leftPoints.get(i);
                final var rightPoint = rightPoints.get(i);
                final var point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final var line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final var line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

                // obtain epipolar planes
                final var epipolarPlane1 = camera1.backProject(line1);
                final var epipolarPlane2 = camera2.backProject(line2);

                // check that both planes are the same
                assertTrue(epipolarPlane1.equals(epipolarPlane2, ABSOLUTE_ERROR));

                // check that point3D and camera centers belong to epipolar plane
                if (!epipolarPlane1.isLocus(point3D, 2.0 * ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(point3D, 2.0 * ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center1, 2.0 * ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center1, 2.0 * ABSOLUTE_ERROR));
                if (!epipolarPlane1.isLocus(center2, 2.0 * ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(epipolarPlane1.isLocus(center2, 2.0 * ABSOLUTE_ERROR));

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
        }

        assertTrue(numValid > 0);

        avgLeftEpipoleError /= numValid;
        avgRightEpipoleError /= numValid;

        assertEquals(0.0, avgLeftEpipoleError, ABSOLUTE_ERROR);
        assertEquals(0.0, avgRightEpipoleError, ABSOLUTE_ERROR);

        // Force NotReadyException
        final var estimator = new PROSACFundamentalMatrixRobustEstimator();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Test
    void testEstimateAffineWithoutRefinement() throws LockedException, NotReadyException, RobustEstimatorException,
            InvalidFundamentalMatrixException, NotAvailableException, WrongSizeException,
            InvalidPairOfCamerasException {

        var numValid = 0;
        for (var j = 0; j < TIMES; j++) {
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

            // convert cameras into affine cameras
            final var cameraMatrix1 = camera1.getInternalMatrix();
            cameraMatrix1.setElementAt(2, 0, 0.0);
            cameraMatrix1.setElementAt(2, 1, 0.0);
            cameraMatrix1.setElementAt(2, 2, 0.0);
            cameraMatrix1.setElementAt(2, 3, 1.0);
            camera1.setInternalMatrix(cameraMatrix1);

            final var cameraMatrix2 = camera2.getInternalMatrix();
            cameraMatrix2.setElementAt(2, 0, 0.0);
            cameraMatrix2.setElementAt(2, 1, 0.0);
            cameraMatrix2.setElementAt(2, 2, 0.0);
            cameraMatrix2.setElementAt(2, 3, 1.0);
            camera2.setInternalMatrix(cameraMatrix2);

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
            final var qualityScores = new double[nPoints];
            final var leftPointsWithError = new ArrayList<Point2D>();
            var pos = 0;
            for (final var leftPoint : leftPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                    leftPointsWithError.add(new HomogeneousPoint2D(leftPoint.getInhomX() + errorX,
                            leftPoint.getInhomY() + errorY, 1.0));

                    qualityScores[pos] = 1.0 / (1.0 + error);
                } else {
                    // inlier
                    leftPointsWithError.add(leftPoint);
                    qualityScores[pos] = 1.0;
                }
                pos++;
            }

            final var rightPointsWithError = new ArrayList<Point2D>();
            pos = 0;
            for (final var rightPoint : rightPoints) {
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                    rightPointsWithError.add(new HomogeneousPoint2D(rightPoint.getInhomX() + errorX,
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
            final var estimator = new PROSACFundamentalMatrixRobustEstimator(
                    FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM, qualityScores, leftPointsWithError,
                    rightPointsWithError, this);
            estimator.setThreshold(THRESHOLD);
            estimator.setComputeAndKeepInliersEnabled(true);
            estimator.setComputeAndKeepResidualsEnabled(true);
            estimator.setResultRefined(false);
            estimator.setCovarianceKept(false);

            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            assertNull(estimator.getCovariance());

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);

            final var fundMatrix = estimator.estimate();
            assertEquals(MIN_REQUIRED_POINTS_AFFINE, estimator.getMinRequiredPoints());
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
            final var fundMatrix2 = new FundamentalMatrix(camera1, camera2);
            fundMatrix2.computeEpipoles();

            final var epipole1a = fundMatrix2.getLeftEpipole();
            final var epipole2a = fundMatrix2.getRightEpipole();

            fundMatrix.computeEpipoles();

            final var epipole1b = fundMatrix.getLeftEpipole();
            final var epipole2b = fundMatrix.getRightEpipole();

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
            assertTrue(fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                    .multiplyByScalarAndReturnNew(-1.0), ABSOLUTE_ERROR));

            // check that all points lie within their corresponding epipolar
            // lines
            for (var i = 0; i < nPoints; i++) {
                final var leftPoint = leftPoints.get(i);
                final var rightPoint = rightPoints.get(i);
                final var point3D = points3D.get(i);

                // obtain epipolar line on left view using 2D point on right view
                final var line1 = fundMatrix.getLeftEpipolarLine(rightPoint);
                // obtain epipolar line on right view using 2D point on left view
                final var line2 = fundMatrix.getRightEpipolarLine(leftPoint);

                // check that 2D point on left view belongs to left epipolar line
                assertTrue(line1.isLocus(leftPoint, ABSOLUTE_ERROR));
                // check that 2D point on right view belongs to right epipolar
                // line
                assertTrue(line2.isLocus(rightPoint, ABSOLUTE_ERROR));

                // obtain epipolar planes
                final var epipolarPlane1 = camera1.backProject(line1);
                final var epipolarPlane2 = camera2.backProject(line2);

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
        final var estimator = new LMedSFundamentalMatrixRobustEstimator();
        assertThrows(NotReadyException.class, estimator::estimate);
    }

    @Override
    public void onEstimateStart(final FundamentalMatrixRobustEstimator estimator) {
        estimateStart++;
        checkLocked((PROSACFundamentalMatrixRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final FundamentalMatrixRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((PROSACFundamentalMatrixRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final FundamentalMatrixRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((PROSACFundamentalMatrixRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final FundamentalMatrixRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((PROSACFundamentalMatrixRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private static void checkLocked(final PROSACFundamentalMatrixRobustEstimator estimator) {
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(10));
        assertThrows(LockedException.class, () -> estimator.setPoints(null, null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> estimator.setThreshold(0.5));
        assertThrows(LockedException.class, () -> estimator.setQualityScores(null));
        assertThrows(LockedException.class, () -> estimator.setNonRobustFundamentalMatrixEstimatorMethod(
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM));
        assertThrows(LockedException.class, estimator::estimate);
        assertTrue(estimator.isLocked());
    }
}
