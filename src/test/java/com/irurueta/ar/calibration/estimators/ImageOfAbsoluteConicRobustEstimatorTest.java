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

import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.geometry.Transformation2D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class ImageOfAbsoluteConicRobustEstimatorTest {

    @Test
    void testCreate() {
        // test with method

        // RANSAC
        var estimator = ImageOfAbsoluteConicRobustEstimator.create(RobustEstimatorMethod.RANSAC);

        // check correctness
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
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertInstanceOf(RANSACImageOfAbsoluteConicRobustEstimator.class, estimator);

        // LMedS
        estimator = ImageOfAbsoluteConicRobustEstimator.create(RobustEstimatorMethod.LMEDS);

        // check correctness
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
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertInstanceOf(LMedSImageOfAbsoluteConicRobustEstimator.class, estimator);

        // MSAC
        estimator = ImageOfAbsoluteConicRobustEstimator.create(RobustEstimatorMethod.MSAC);

        // check correctness
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
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertInstanceOf(MSACImageOfAbsoluteConicRobustEstimator.class, estimator);

        // PROSAC
        estimator = ImageOfAbsoluteConicRobustEstimator.create(RobustEstimatorMethod.PROSAC);

        // check correctness
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
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertInstanceOf(PROSACImageOfAbsoluteConicRobustEstimator.class, estimator);

        // PROMedS
        estimator = ImageOfAbsoluteConicRobustEstimator.create(RobustEstimatorMethod.PROMEDS);

        // check correctness
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
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertInstanceOf(PROMedSImageOfAbsoluteConicRobustEstimator.class, estimator);

        // test with homographies, quality scores and method
        final var homographies = new ArrayList<Transformation2D>();
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        final var qualityScores = new double[3];

        // RANSAC
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, qualityScores,
                RobustEstimatorMethod.RANSAC);

        // check correctness
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
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertInstanceOf(RANSACImageOfAbsoluteConicRobustEstimator.class, estimator);

        // LMedS
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, qualityScores,
                RobustEstimatorMethod.LMEDS);

        // check correctness
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
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertInstanceOf(LMedSImageOfAbsoluteConicRobustEstimator.class, estimator);

        // MSAC
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, qualityScores, RobustEstimatorMethod.MSAC);

        // check correctness
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
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertInstanceOf(MSACImageOfAbsoluteConicRobustEstimator.class, estimator);

        // PROSAC
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, qualityScores,
                RobustEstimatorMethod.PROSAC);

        // check correctness
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
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertInstanceOf(PROSACImageOfAbsoluteConicRobustEstimator.class, estimator);

        // PROMedS
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, qualityScores,
                RobustEstimatorMethod.PROMEDS);

        // check correctness
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
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertInstanceOf(PROMedSImageOfAbsoluteConicRobustEstimator.class, estimator);

        // test with homographies and method

        // RANSAC
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, RobustEstimatorMethod.RANSAC);

        // check correctness
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
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertInstanceOf(RANSACImageOfAbsoluteConicRobustEstimator.class, estimator);

        // LMedS
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, RobustEstimatorMethod.LMEDS);

        // check correctness
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
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, estimator.getMethod());
        assertInstanceOf(LMedSImageOfAbsoluteConicRobustEstimator.class, estimator);

        // MSAC
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, RobustEstimatorMethod.MSAC);

        // check correctness
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
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertInstanceOf(MSACImageOfAbsoluteConicRobustEstimator.class, estimator);

        // PROSAC
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, RobustEstimatorMethod.PROSAC);

        // check correctness
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
        assertEquals(RobustEstimatorMethod.PROSAC, estimator.getMethod());
        assertInstanceOf(PROSACImageOfAbsoluteConicRobustEstimator.class, estimator);

        // PROMedS
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, RobustEstimatorMethod.PROMEDS);

        // check correctness
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
        assertEquals(RobustEstimatorMethod.PROMEDS, estimator.getMethod());
        assertInstanceOf(PROMedSImageOfAbsoluteConicRobustEstimator.class, estimator);

        // test without arguments
        estimator = ImageOfAbsoluteConicRobustEstimator.create();

        // check correctness
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
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());
        assertInstanceOf(PROSACImageOfAbsoluteConicRobustEstimator.class, estimator);

        // test with homographies and quality scores
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies, qualityScores);

        // check correctness
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
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());
        assertInstanceOf(PROSACImageOfAbsoluteConicRobustEstimator.class, estimator);

        // test with homographies
        estimator = ImageOfAbsoluteConicRobustEstimator.create(homographies);

        // check correctness
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
        assertEquals(ImageOfAbsoluteConicRobustEstimator.DEFAULT_ROBUST_METHOD, estimator.getMethod());
        assertInstanceOf(PROSACImageOfAbsoluteConicRobustEstimator.class, estimator);
    }
}
