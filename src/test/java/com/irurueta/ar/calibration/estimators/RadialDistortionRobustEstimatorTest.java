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

import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class RadialDistortionRobustEstimatorTest {

    @Test
    void testCreate() {
        // test create with method

        // test RANSAC
        var estimator = RadialDistortionRobustEstimator.create(RobustEstimatorMethod.RANSAC);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());

        // test LMedS
        estimator = RadialDistortionRobustEstimator.create(RobustEstimatorMethod.LMEDS);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());

        // test MSAC
        estimator = RadialDistortionRobustEstimator.create(RobustEstimatorMethod.MSAC);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());

        // test PROSAC
        estimator = RadialDistortionRobustEstimator.create(RobustEstimatorMethod.PROSAC);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());

        // test PROMedS
        estimator = RadialDistortionRobustEstimator.create(RobustEstimatorMethod.PROMEDS);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());

        // test create with points, quality scores, center and method
        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        final var qualityScores = new double[RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS];
        for (var i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        final var center = Point2D.create();

        // test RANSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, qualityScores, center,
                RobustEstimatorMethod.RANSAC);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());

        // test LMedS
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, qualityScores, center,
                RobustEstimatorMethod.LMEDS);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());

        // test MSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, qualityScores, center,
                RobustEstimatorMethod.MSAC);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());

        // test PROSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, qualityScores, center,
                RobustEstimatorMethod.PROSAC);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());

        // test PROMedS
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, qualityScores, center,
                RobustEstimatorMethod.PROMEDS);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());

        // test create with points, center and method

        // test RANSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, center,
                RobustEstimatorMethod.RANSAC);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());

        // test LMedS
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, center,
                RobustEstimatorMethod.LMEDS);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());

        // test MSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, center,
                RobustEstimatorMethod.MSAC);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());

        // test PROSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, center,
                RobustEstimatorMethod.PROSAC);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());

        // test PROMedS
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, center,
                RobustEstimatorMethod.PROMEDS);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());

        // test create with points, quality scores and method

        // test RANSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, qualityScores,
                RobustEstimatorMethod.RANSAC);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());

        // test LMedS
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, qualityScores,
                RobustEstimatorMethod.LMEDS);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());

        // test MSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, qualityScores,
                RobustEstimatorMethod.MSAC);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());

        // test PROSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, qualityScores,
                RobustEstimatorMethod.PROSAC);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());

        // test PROMedS
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, qualityScores,
                RobustEstimatorMethod.PROMEDS);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());

        // test create with points and method

        // test RANSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints,
                RobustEstimatorMethod.RANSAC);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());

        // test LMedS
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints,
                RobustEstimatorMethod.LMEDS);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());

        // test MSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints,
                RobustEstimatorMethod.MSAC);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());

        // test PROSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints,
                RobustEstimatorMethod.PROSAC);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());

        // test PROMedS
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints,
                RobustEstimatorMethod.PROMEDS);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());

        // test create with default method
        estimator = RadialDistortionRobustEstimator.create();

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // test create with points and quality scores
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, qualityScores);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // test create with points
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // test create with points, quality scores and center
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, qualityScores, center);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());

        // test create with points and center
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints, center);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());
    }

    @Test
    void testAreValidPoints() {
        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        for (var i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        final var emptyPoints = new ArrayList<Point2D>();

        assertTrue(RadialDistortionRobustEstimator.areValidPoints(distortedPoints, undistortedPoints));
        assertFalse(RadialDistortionRobustEstimator.areValidPoints(emptyPoints, undistortedPoints));
        assertFalse(RadialDistortionRobustEstimator.areValidPoints(emptyPoints, emptyPoints));
        assertFalse(RadialDistortionRobustEstimator.areValidPoints(null, undistortedPoints));
        assertFalse(RadialDistortionRobustEstimator.areValidPoints(distortedPoints, null));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        for (var i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        final var listener = new RadialDistortionRobustEstimatorListener() {

            @Override
            public void onEstimateStart(final RadialDistortionRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final RadialDistortionRobustEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateNextIteration(final RadialDistortionRobustEstimator estimator, final int iteration) {
                // no action needed
            }

            @Override
            public void onEstimateProgressChange(
                    final RadialDistortionRobustEstimator estimator, final float progress) {
                // no action needed
            }
        };

        final var estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints);

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // set new value
        estimator.setListener(listener);

        // check correctness
        assertSame(listener, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        for (var i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        final var estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints);

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5, estimator.getProgressDelta(), 0.0);
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        for (var i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        final var estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints);

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.5f);

        // check correctness
        assertEquals(0.5, estimator.getConfidence(), 0.0);
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        for (var i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        final var estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints);

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(10);

        // check correctness
        assertEquals(10, estimator.getMaxIterations());
    }

    @Test
    void testGetSetPoints() throws LockedException {
        final var distortedPoints1 = new ArrayList<Point2D>();
        final var undistortedPoints1 = new ArrayList<Point2D>();
        final var distortedPoints2 = new ArrayList<Point2D>();
        final var undistortedPoints2 = new ArrayList<Point2D>();
        for (var i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints1.add(Point2D.create());
            undistortedPoints1.add(Point2D.create());
            distortedPoints2.add(Point2D.create());
            undistortedPoints2.add(Point2D.create());
        }
        final var emptyPoints = new ArrayList<Point2D>();

        final var estimator = RadialDistortionRobustEstimator.create(distortedPoints1, undistortedPoints1);

        // check correctness
        assertSame(estimator.getDistortedPoints(), distortedPoints1);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints1);
        assertTrue(estimator.arePointsAvailable());

        // set new value
        estimator.setPoints(distortedPoints2, undistortedPoints2);

        // check correctness
        assertSame(distortedPoints2, estimator.getDistortedPoints());
        assertSame(undistortedPoints2, estimator.getUndistortedPoints());
        assertTrue(estimator.arePointsAvailable());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(emptyPoints, undistortedPoints2));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(emptyPoints, emptyPoints));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(null, undistortedPoints2));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(distortedPoints2, null));
    }

    @Test
    void testGetSetDistortionCenter() throws LockedException {
        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        for (var i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        final var center = Point2D.create();

        final var estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints);

        // check default value
        assertNull(estimator.getDistortionCenter());

        // set new value
        estimator.setDistortionCenter(center);

        // check correctness
        assertSame(center, estimator.getDistortionCenter());
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        final var qualityScores = new double[RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS];
        for (var i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        final var estimator = RadialDistortionRobustEstimator.create(distortedPoints, undistortedPoints,
                RobustEstimatorMethod.RANSAC);

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertNull(estimator.getQualityScores());
    }
}
