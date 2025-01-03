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
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class FundamentalMatrixEstimatorTest {

    @Test
    void testCreate() {
        // test create with method

        // 7 points
        var estimator = FundamentalMatrixEstimator.create(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);

        // check correctness
        assertInstanceOf(SevenPointsFundamentalMatrixEstimator.class, estimator);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM, estimator.getMethod());

        // 8 points
        estimator = FundamentalMatrixEstimator.create(FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);

        // check correctness
        assertInstanceOf(EightPointsFundamentalMatrixEstimator.class, estimator);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM, estimator.getMethod());

        // affine
        estimator = FundamentalMatrixEstimator.create(FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM);

        // check correctness
        assertInstanceOf(AffineFundamentalMatrixEstimator.class, estimator);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM, estimator.getMethod());

        // test create with points and method
        final var leftPoints = new ArrayList<Point2D>();
        final var rightPoints = new ArrayList<Point2D>();
        for (var i = 0; i < 8; i++) {
            leftPoints.add(Point2D.create());
            rightPoints.add(Point2D.create());
        }

        // 7 points
        estimator = FundamentalMatrixEstimator.create(leftPoints, rightPoints,
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);

        // check correctness
        assertInstanceOf(SevenPointsFundamentalMatrixEstimator.class, estimator);
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM, estimator.getMethod());

        // 8 points
        estimator = FundamentalMatrixEstimator.create(leftPoints, rightPoints,
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);

        // check correctness
        assertInstanceOf(EightPointsFundamentalMatrixEstimator.class, estimator);
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM, estimator.getMethod());

        // affine
        estimator = FundamentalMatrixEstimator.create(leftPoints, rightPoints,
                FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM);

        // check correctness
        assertInstanceOf(AffineFundamentalMatrixEstimator.class, estimator);
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM, estimator.getMethod());

        // test create with default method
        estimator = FundamentalMatrixEstimator.create();

        assertInstanceOf(SevenPointsFundamentalMatrixEstimator.class, estimator);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM, estimator.getMethod());

        // test create with points and default method
        estimator = FundamentalMatrixEstimator.create(leftPoints, rightPoints);

        // check correctness
        assertInstanceOf(SevenPointsFundamentalMatrixEstimator.class, estimator);
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM, estimator.getMethod());
    }
}
