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

import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class FundamentalMatrixRobustEstimatorTest {

    public static final int MIN_REQUIRED_POINTS_7 = 7;
    public static final int MIN_REQUIRED_POINTS_8 = 8;

    @Test
    void testCreate() {
        // create with method

        // RANSAC
        var estimator = FundamentalMatrixRobustEstimator.create(RobustEstimatorMethod.RANSAC);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertInstanceOf(RANSACFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // LMedS
        estimator = FundamentalMatrixRobustEstimator.create(RobustEstimatorMethod.LMEDS);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertInstanceOf(LMedSFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // MSAC
        estimator = FundamentalMatrixRobustEstimator.create(RobustEstimatorMethod.MSAC);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertInstanceOf(MSACFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // PROSAC
        estimator = FundamentalMatrixRobustEstimator.create(RobustEstimatorMethod.PROSAC);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertInstanceOf(PROSACFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // PROMedS
        estimator = FundamentalMatrixRobustEstimator.create(RobustEstimatorMethod.PROMEDS);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertInstanceOf(PROMedSFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // create with left and right points
        final var leftPoints = new ArrayList<Point2D>();
        final var rightPoints = new ArrayList<Point2D>();
        for (var i = 0; i < MIN_REQUIRED_POINTS_8; i++) {
            leftPoints.add(Point2D.create());
            rightPoints.add(Point2D.create());
        }

        // RANSAC
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, rightPoints, RobustEstimatorMethod.RANSAC);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertInstanceOf(RANSACFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // LMedS
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, rightPoints, RobustEstimatorMethod.LMEDS);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertInstanceOf(LMedSFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // MSAC
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, rightPoints, RobustEstimatorMethod.MSAC);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertInstanceOf(MSACFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // PROSAC
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, rightPoints, RobustEstimatorMethod.PROSAC);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertInstanceOf(PROSACFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // PROMedS
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, rightPoints, RobustEstimatorMethod.PROMEDS);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertInstanceOf(PROMedSFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // create with left and right points, and quality scores
        final var qualityScores = new double[MIN_REQUIRED_POINTS_8];

        // RANSAC
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, rightPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertInstanceOf(RANSACFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // LMedS
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, rightPoints, qualityScores,
                RobustEstimatorMethod.LMEDS);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertInstanceOf(LMedSFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // MSAC
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, rightPoints, qualityScores,
                RobustEstimatorMethod.MSAC);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertInstanceOf(MSACFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // PROSAC
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, rightPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertInstanceOf(PROSACFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // PROMedS
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, rightPoints, qualityScores,
                RobustEstimatorMethod.PROMEDS);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertInstanceOf(PROMedSFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test create without arguments
        estimator = FundamentalMatrixRobustEstimator.create();
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertInstanceOf(PROSACFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test create with left and right points
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, rightPoints);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertInstanceOf(PROSACFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        // test create with left and right points, and quality scores
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, rightPoints, qualityScores);
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM,
                estimator.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(MIN_REQUIRED_POINTS_7, estimator.getMinRequiredPoints());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertInstanceOf(PROSACFundamentalMatrixRobustEstimator.class, estimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
    }
}
