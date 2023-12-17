/*
 * Copyright (C) 2016 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class DualAbsoluteQuadricRobustEstimatorTest implements
        DualAbsoluteQuadricRobustEstimatorListener {

    @Test
    public void testCreateWithMethod() {
        // create with LMedS method
        DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create(
                RobustEstimatorMethod.LMEDS);

        // check correctness
        assertTrue(estimator instanceof LMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getCameras());
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());

        // create with MSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(RobustEstimatorMethod.MSAC);

        // check correctness
        assertTrue(estimator instanceof MSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getCameras());
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());

        // create with PROSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(RobustEstimatorMethod.PROSAC);

        // check correctness
        assertTrue(estimator instanceof
                PROSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getCameras());
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());

        // create with PROMedS method
        estimator = DualAbsoluteQuadricRobustEstimator.create(RobustEstimatorMethod.PROMEDS);

        // check correctness
        assertTrue(estimator instanceof
                PROMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getCameras());
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());

        // create with RANSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(RobustEstimatorMethod.RANSAC);

        // check correctness
        assertTrue(estimator instanceof
                RANSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getCameras());
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
    }

    @Test
    public void testCreateWithCamerasQualityScoresAndMethod() {
        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        final double[] qualityScores = new double[cameras.size()];

        // create with LMedS method
        DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create(cameras,
                qualityScores, RobustEstimatorMethod.LMEDS);

        // check correctness
        assertTrue(estimator instanceof LMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(estimator.getCameras(), cameras);
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), new double[1],
                    RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // create with MSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(cameras, qualityScores,
                RobustEstimatorMethod.MSAC);

        // check correctness
        assertTrue(estimator instanceof MSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(estimator.getCameras(), cameras);
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), new double[1],
                    RobustEstimatorMethod.MSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // create with PROSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(cameras, qualityScores,
                RobustEstimatorMethod.PROSAC);

        // check correctness
        assertTrue(estimator instanceof PROSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(estimator.getCameras(), cameras);
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(cameras,
                    new double[1],
                    RobustEstimatorMethod.PROSAC);
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), new double[1],
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // create with PROMedS method
        estimator = DualAbsoluteQuadricRobustEstimator.create(cameras, qualityScores,
                RobustEstimatorMethod.PROMEDS);

        // check correctness
        assertTrue(estimator instanceof PROMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(estimator.getCameras(), cameras);
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertSame(qualityScores, estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(cameras,
                    new double[1], RobustEstimatorMethod.PROMEDS);
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), new double[1],
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // create with RANSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(cameras,
                qualityScores, RobustEstimatorMethod.RANSAC);

        // check correctness
        assertTrue(estimator instanceof RANSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(estimator.getCameras(), cameras);
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), new double[1],
                    RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testCreateWithCamerasAndMethod() {
        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        // create with LMedS method
        DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create(cameras,
                RobustEstimatorMethod.LMEDS);

        // check correctness
        assertTrue(estimator instanceof LMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(estimator.getCameras(), cameras);
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(),
                    RobustEstimatorMethod.LMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // create with MSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(cameras, RobustEstimatorMethod.MSAC);

        // check correctness
        assertTrue(estimator instanceof MSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(estimator.getCameras(), cameras);
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(),
                    RobustEstimatorMethod.MSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // create with PROSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(cameras,
                RobustEstimatorMethod.PROSAC);

        // check correctness
        assertTrue(estimator instanceof PROSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(cameras, estimator.getCameras());
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(),
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // create with PROMedS method
        estimator = DualAbsoluteQuadricRobustEstimator.create(cameras, RobustEstimatorMethod.PROMEDS);

        // check correctness
        assertTrue(estimator instanceof PROMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(cameras, estimator.getCameras());
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(),
                    RobustEstimatorMethod.PROMEDS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // create with RANSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(cameras, RobustEstimatorMethod.RANSAC);

        // check correctness
        assertTrue(estimator instanceof RANSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(cameras, estimator.getCameras());
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(),
                    RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testCreateDefaultMethod() {
        final DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create();

        // check correctness
        assertTrue(estimator instanceof LMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getCameras());
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
    }

    @Test
    public void testCreateWithCamerasQualityScoresAndDefaultMethod() {
        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        final double[] qualityScores = new double[cameras.size()];

        DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create(cameras,
                qualityScores);

        // check correctness
        assertTrue(estimator instanceof LMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(estimator.getCameras(), cameras);
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(cameras,
                    new double[1], RobustEstimatorMethod.PROSAC);
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), new double[1],
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testCreateWithCamerasAndDefaultMethod() {
        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create(cameras);

        // check correctness
        assertTrue(estimator instanceof
                LMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(estimator.getCameras(), cameras);
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(),
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testIsSetZeroSkewness() throws LockedException {
        final DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create();

        // check default value
        assertTrue(estimator.isZeroSkewness());

        // set new value
        estimator.setZeroSkewness(false);

        // check correctness
        assertFalse(estimator.isZeroSkewness());
    }

    @Test
    public void testIsSetPrincipalPointAtOrigin() throws LockedException {
        final DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create();

        // check default value
        assertTrue(estimator.isPrincipalPointAtOrigin());

        // set new value
        estimator.setPrincipalPointAtOrigin(false);

        // check correctness
        assertFalse(estimator.isPrincipalPointAtOrigin());
    }

    @Test
    public void testIsFocalDistanceAspectRatioKnown() throws LockedException {
        final DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create();

        // check default value
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());

        // set new value
        estimator.setFocalDistanceAspectRatioKnown(false);

        // check correctness
        assertFalse(estimator.isFocalDistanceAspectRatioKnown());
    }

    @Test
    public void testGetSetFocalDistanceAspectRatio() throws LockedException {
        final DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create();

        // check default value
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);

        // set new value
        estimator.setFocalDistanceAspectRatio(0.5);

        // check correctness
        assertEquals(0.5, estimator.getFocalDistanceAspectRatio(), 0.0);
    }

    @Test
    public void testIsSetSingularityEnforced() throws LockedException {
        final DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create();

        // check default value
        assertTrue(estimator.isSingularityEnforced());

        // set new value
        estimator.setSingularityEnforced(false);

        // check correctness
        assertFalse(estimator.isSingularityEnforced());
    }

    @Test
    public void testIsSetEnforcedSingularityValidated() throws LockedException {
        final DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create();

        // check default value
        assertTrue(estimator.isEnforcedSingularityValidated());

        // set new value
        estimator.setEnforcedSingularityValidated(false);

        // check correctness
        assertFalse(estimator.isEnforcedSingularityValidated());
    }

    @Test
    public void testGetSetDeterminantThreshold() throws LockedException {
        final DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create();

        // check default value
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                estimator.getDeterminantThreshold(), 0.0);

        // set new value
        estimator.setDeterminantThreshold(1e-3);

        // check correctness
        assertEquals(1e-3, estimator.getDeterminantThreshold(), 0.0);
    }

    @Test
    public void testGetSetListenerAndIsListenerAvailable()
            throws LockedException {
        final DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create();

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
        final DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create();

        // check default value
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);

        // set new value
        estimator.setProgressDelta(0.1f);

        // check correctness
        assertEquals(0.1f, estimator.getProgressDelta(), 0.0);

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
        final DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create();

        // check default value
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.8);

        // check correctness
        assertEquals(0.8, estimator.getConfidence(), 0.0);

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
        final DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create();

        // check default value
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(100);

        // check correctness
        assertEquals(100, estimator.getMaxIterations());

        // Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetCameras() throws LockedException {
        final DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create();

        // initial value
        assertNull(estimator.getCameras());

        // set new value
        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        estimator.setCameras(cameras);

        // check correctness
        assertSame(cameras, estimator.getCameras());

        // Force IllegalArgumentException
        try {
            estimator.setCameras(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setCameras(new ArrayList<PinholeCamera>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetMinNumberOfRequiredCameras() throws LockedException {
        final DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create();

        // check default value
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());

        // disable principal point at origin
        estimator.setPrincipalPointAtOrigin(false);

        // check correctness
        assertEquals(-1, estimator.getMinNumberOfRequiredCameras());

        // disable zero skewness
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setZeroSkewness(false);

        // check correctness
        assertEquals(4, estimator.getMinNumberOfRequiredCameras());

        // disable focal distance aspect ratio known
        estimator.setZeroSkewness(true);
        estimator.setFocalDistanceAspectRatioKnown(false);

        // check correctness
        assertEquals(3, estimator.getMinNumberOfRequiredCameras());

        // disable zero skewness and singularity enforcement
        estimator.setZeroSkewness(false);
        estimator.setSingularityEnforced(false);

        // check correctness
        assertEquals(5, estimator.getMinNumberOfRequiredCameras());

        // disable focal distance aspect ratio known and singularity enforcement
        estimator.setZeroSkewness(true);

        // check correctness
        assertEquals(3, estimator.getMinNumberOfRequiredCameras());
    }

    @Test
    public void testAreValidConstraints() throws LockedException {
        final DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create();

        // check default value
        assertTrue(estimator.areValidConstraints());

        // disable principal point at origin
        estimator.setPrincipalPointAtOrigin(false);

        // check correctness
        assertFalse(estimator.areValidConstraints());

        // disable zero skewness
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setZeroSkewness(false);

        // check correctness
        assertTrue(estimator.areValidConstraints());

        // disable focal distance aspect ratio known
        estimator.setZeroSkewness(true);
        estimator.setFocalDistanceAspectRatioKnown(false);

        // check correctness
        assertFalse(estimator.areValidConstraints());

        // disable singularity enforcement
        estimator.setSingularityEnforced(false);

        // check correctness
        assertTrue(estimator.areValidConstraints());
    }

    @Test
    public void testIsReady() throws LockedException {
        final DualAbsoluteQuadricRobustEstimator estimator = DualAbsoluteQuadricRobustEstimator.create();

        // check default value
        assertNull(estimator.getCameras());
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());

        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        estimator.setCameras(cameras);
        estimator.setQualityScores(new double[cameras.size()]);

        // check correctness
        assertTrue(estimator.isReady());

        // disable principal point at origin
        estimator.setPrincipalPointAtOrigin(false);

        // check correctness
        assertFalse(estimator.areValidConstraints());
        assertFalse(estimator.isReady());

        // enable principal point at origin
        estimator.setPrincipalPointAtOrigin(true);

        // check correctness
        assertTrue(estimator.isReady());

        // clear cameras
        cameras.clear();

        // check correctness
        assertFalse(estimator.isReady());
    }


    @Override
    public void onEstimateStart(
            final DualAbsoluteQuadricRobustEstimator estimator) {
    }

    @Override
    public void onEstimateEnd(final DualAbsoluteQuadricRobustEstimator estimator) {
    }

    @Override
    public void onEstimateNextIteration(
            final DualAbsoluteQuadricRobustEstimator estimator, final int iteration) {
    }

    @Override
    public void onEstimateProgressChange(
            final DualAbsoluteQuadricRobustEstimator estimator, final float progress) {
    }
}
