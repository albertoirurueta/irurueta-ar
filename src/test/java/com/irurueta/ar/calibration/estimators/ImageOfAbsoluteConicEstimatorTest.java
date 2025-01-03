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
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class ImageOfAbsoluteConicEstimatorTest {

    @Test
    void testCreate() {
        // test without parameters
        var estimator = ImageOfAbsoluteConicEstimator.create();

        // check correctness
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertNull(estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertFalse(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ESTIMATOR_TYPE, estimator.getType());
        assertInstanceOf(LMSEImageOfAbsoluteConicEstimator.class, estimator);

        // test with listener
        final var listener = new ImageOfAbsoluteConicEstimatorListener() {

            @Override
            public void onEstimateStart(final ImageOfAbsoluteConicEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final ImageOfAbsoluteConicEstimator estimator) {
                // no action needed
            }

            @Override
            public void onEstimationProgressChange(
                    final ImageOfAbsoluteConicEstimator estimator, final float progress) {
                // no action needed
            }
        };
        estimator = ImageOfAbsoluteConicEstimator.create(listener);

        // check correctness
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertFalse(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ESTIMATOR_TYPE, estimator.getType());
        assertInstanceOf(LMSEImageOfAbsoluteConicEstimator.class, estimator);

        // test with homographies
        final var homographies = new ArrayList<Transformation2D>();
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());

        estimator = ImageOfAbsoluteConicEstimator.create(homographies);

        // check correctness
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertSame(homographies, estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertTrue(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ESTIMATOR_TYPE, estimator.getType());
        assertInstanceOf(LMSEImageOfAbsoluteConicEstimator.class, estimator);

        // Force IllegalArgumentException
        final var emptyHomographies = new ArrayList<Transformation2D>();
        assertThrows(IllegalArgumentException.class, () -> ImageOfAbsoluteConicEstimator.create(emptyHomographies));

        // test with homographies and listener
        estimator = ImageOfAbsoluteConicEstimator.create(homographies, listener);

        // check correctness
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertSame(listener, estimator.getListener());
        assertSame(homographies, estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertTrue(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ESTIMATOR_TYPE, estimator.getType());
        assertInstanceOf(LMSEImageOfAbsoluteConicEstimator.class, estimator);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> ImageOfAbsoluteConicEstimator.create(emptyHomographies, listener));

        // test with LMSE type
        estimator = ImageOfAbsoluteConicEstimator.create(ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);

        // check correctness
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertNull(estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertFalse(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR, estimator.getType());
        assertInstanceOf(LMSEImageOfAbsoluteConicEstimator.class, estimator);

        // test with Weighted type
        estimator = ImageOfAbsoluteConicEstimator.create(ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR);

        // check correctness
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertNull(estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertFalse(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR, estimator.getType());
        assertInstanceOf(WeightedImageOfAbsoluteConicEstimator.class, estimator);

        // test with LMSE type and listener
        estimator = ImageOfAbsoluteConicEstimator.create(listener,
                ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);

        // check correctness
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertFalse(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR, estimator.getType());
        assertInstanceOf(LMSEImageOfAbsoluteConicEstimator.class, estimator);

        // test with weighted type and listener
        estimator = ImageOfAbsoluteConicEstimator.create(listener,
                ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR);

        // check correctness
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertSame(listener, estimator.getListener());
        assertNull(estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertFalse(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR, estimator.getType());
        assertInstanceOf(WeightedImageOfAbsoluteConicEstimator.class, estimator);

        // test with LMSE type and homographies
        estimator = ImageOfAbsoluteConicEstimator.create(homographies,
                ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);

        // check correctness
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertSame(homographies, estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertTrue(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR, estimator.getType());
        assertInstanceOf(LMSEImageOfAbsoluteConicEstimator.class, estimator);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> ImageOfAbsoluteConicEstimator.create(emptyHomographies,
                ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR));

        // test with weighted type and homographies
        estimator = ImageOfAbsoluteConicEstimator.create(homographies,
                ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR);

        // check correctness
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertSame(homographies, estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertTrue(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR, estimator.getType());
        assertInstanceOf(WeightedImageOfAbsoluteConicEstimator.class, estimator);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> ImageOfAbsoluteConicEstimator.create(emptyHomographies,
                ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR));

        // test with LMSE type, homographies and listener
        estimator = ImageOfAbsoluteConicEstimator.create(homographies, listener,
                ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);

        // check correctness
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertSame(listener, estimator.getListener());
        assertSame(homographies, estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertTrue(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR, estimator.getType());
        assertInstanceOf(LMSEImageOfAbsoluteConicEstimator.class, estimator);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> ImageOfAbsoluteConicEstimator.create(emptyHomographies,
                listener, ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR));

        // test with weighted type, homographies and listener
        estimator = ImageOfAbsoluteConicEstimator.create(homographies, listener,
                ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR);

        // check correctness
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS, estimator.isZeroSkewness());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_PRINCIPAL_POINT_AT_ORIGIN,
                estimator.isPrincipalPointAtOrigin());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN,
                estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(ImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO,
                estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertSame(listener, estimator.getListener());
        assertSame(homographies, estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertTrue(estimator.isReady());
        assertEquals(ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR, estimator.getType());
        assertInstanceOf(WeightedImageOfAbsoluteConicEstimator.class, estimator);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> ImageOfAbsoluteConicEstimator.create(emptyHomographies,
                listener, ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR));
    }
}
