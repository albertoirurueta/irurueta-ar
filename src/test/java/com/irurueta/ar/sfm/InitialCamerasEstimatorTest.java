/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.ar.sfm;

import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.geometry.PinholeCamera;
import org.junit.Test;

import static org.junit.Assert.*;

public class InitialCamerasEstimatorTest implements InitialCamerasEstimatorListener {

    @Test
    public void testCreate() {
        // test create with method
        InitialCamerasEstimator estimator = InitialCamerasEstimator.create(
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX, estimator.getMethod());
        assertTrue(estimator instanceof EssentialMatrixInitialCamerasEstimator);

        estimator = InitialCamerasEstimator.create(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);

        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC, estimator.getMethod());
        assertTrue(estimator instanceof DualImageOfAbsoluteConicInitialCamerasEstimator);

        estimator = InitialCamerasEstimator.create(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);

        assertEquals(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC, estimator.getMethod());
        assertTrue(estimator instanceof DualAbsoluteQuadricInitialCamerasEstimator);

        estimator = InitialCamerasEstimator.create(
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);

        assertEquals(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC, estimator.getMethod());
        assertTrue(estimator instanceof DualAbsoluteQuadricInitialCamerasEstimator);

        // test create with fundamental matrix and method
        final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
        estimator = InitialCamerasEstimator.create(fundamentalMatrix,
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX, estimator.getMethod());
        assertTrue(estimator instanceof EssentialMatrixInitialCamerasEstimator);

        estimator = InitialCamerasEstimator.create(fundamentalMatrix,
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);

        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC, estimator.getMethod());
        assertTrue(estimator instanceof DualImageOfAbsoluteConicInitialCamerasEstimator);

        estimator = InitialCamerasEstimator.create(fundamentalMatrix,
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);

        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC, estimator.getMethod());
        assertTrue(estimator instanceof DualAbsoluteQuadricInitialCamerasEstimator);

        estimator = InitialCamerasEstimator.create(fundamentalMatrix,
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);

        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC, estimator.getMethod());
        assertTrue(estimator instanceof DualAbsoluteQuadricInitialCamerasEstimator);

        // test create with listener and method
        estimator = InitialCamerasEstimator.create(this,
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

        assertSame(this, estimator.getListener());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX, estimator.getMethod());
        assertTrue(estimator instanceof EssentialMatrixInitialCamerasEstimator);

        estimator = InitialCamerasEstimator.create(this,
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);

        assertSame(this, estimator.getListener());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC, estimator.getMethod());
        assertTrue(estimator instanceof DualImageOfAbsoluteConicInitialCamerasEstimator);

        estimator = InitialCamerasEstimator.create(this,
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);

        assertSame(this, estimator.getListener());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC, estimator.getMethod());
        assertTrue(estimator instanceof DualAbsoluteQuadricInitialCamerasEstimator);

        estimator = InitialCamerasEstimator.create(this,
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);

        assertSame(this, estimator.getListener());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC, estimator.getMethod());
        assertTrue(estimator instanceof DualAbsoluteQuadricInitialCamerasEstimator);

        // test create with fundamental matrix, listener and method
        estimator = InitialCamerasEstimator.create(fundamentalMatrix, this,
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX, estimator.getMethod());
        assertTrue(estimator instanceof EssentialMatrixInitialCamerasEstimator);

        estimator = InitialCamerasEstimator.create(fundamentalMatrix, this,
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);

        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC, estimator.getMethod());
        assertTrue(estimator instanceof DualImageOfAbsoluteConicInitialCamerasEstimator);

        estimator = InitialCamerasEstimator.create(fundamentalMatrix, this,
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);

        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC, estimator.getMethod());
        assertTrue(estimator instanceof DualAbsoluteQuadricInitialCamerasEstimator);

        estimator = InitialCamerasEstimator.create(fundamentalMatrix, this,
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);

        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC, estimator.getMethod());
        assertTrue(estimator instanceof DualAbsoluteQuadricInitialCamerasEstimator);

        // test create with default method
        estimator = InitialCamerasEstimator.create();

        assertEquals(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC, estimator.getMethod());
        assertTrue(estimator instanceof DualAbsoluteQuadricInitialCamerasEstimator);

        // test create with fundamental matrix and default method
        estimator = InitialCamerasEstimator.create(fundamentalMatrix);

        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC, estimator.getMethod());
        assertTrue(estimator instanceof DualAbsoluteQuadricInitialCamerasEstimator);

        // test create with listener and default method
        estimator = InitialCamerasEstimator.create(this);

        assertSame(this, estimator.getListener());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC, estimator.getMethod());
        assertTrue(estimator instanceof DualAbsoluteQuadricInitialCamerasEstimator);

        // test create with fundamental matrix, listener and default method
        estimator = InitialCamerasEstimator.create(fundamentalMatrix, this);

        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC, estimator.getMethod());
        assertTrue(estimator instanceof DualAbsoluteQuadricInitialCamerasEstimator);
    }

    @Override
    public void onStart(final InitialCamerasEstimator estimator) {
    }

    @Override
    public void onFinish(final InitialCamerasEstimator estimator,
                         final PinholeCamera estimatedLeftCamera,
                         final PinholeCamera estimatedRightCamera) {
    }

    @Override
    public void onFail(final InitialCamerasEstimator estimator,
                       final InitialCamerasEstimationFailedException e) {
    }
}
