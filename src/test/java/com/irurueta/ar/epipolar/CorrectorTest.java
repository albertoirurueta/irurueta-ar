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
package com.irurueta.ar.epipolar;

import com.irurueta.geometry.Point2D;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class CorrectorTest {

    @Test
    public void testCreate() {
        // create with type

        // SAMPSON
        Corrector corrector = Corrector.create(CorrectorType.SAMPSON_CORRECTOR);

        // check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());
        assertTrue(corrector instanceof SampsonCorrector);

        // GOLD STANDARD
        corrector = Corrector.create(CorrectorType.GOLD_STANDARD);

        // check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.GOLD_STANDARD, corrector.getType());
        assertTrue(corrector instanceof GoldStandardCorrector);

        // create with fundamental matrix and type
        final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();

        // SAMPSON
        corrector = Corrector.create(fundamentalMatrix, CorrectorType.SAMPSON_CORRECTOR);

        // check correctness
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());
        assertTrue(corrector instanceof SampsonCorrector);

        // GOLD STANDARD
        corrector = Corrector.create(fundamentalMatrix, CorrectorType.GOLD_STANDARD);

        // check correctness
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.GOLD_STANDARD, corrector.getType());
        assertTrue(corrector instanceof GoldStandardCorrector);

        // create with points and type
        final List<Point2D> leftPoints = new ArrayList<>();
        final List<Point2D> rightPoints = new ArrayList<>();

        // SAMPSON
        corrector = Corrector.create(leftPoints, rightPoints, CorrectorType.SAMPSON_CORRECTOR);

        // check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());
        assertTrue(corrector instanceof SampsonCorrector);

        // GOLD STANDARD
        corrector = Corrector.create(leftPoints, rightPoints, CorrectorType.GOLD_STANDARD);

        // check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.GOLD_STANDARD, corrector.getType());
        assertTrue(corrector instanceof GoldStandardCorrector);

        // create with points, fundamental matrix and type

        // SAMPSON
        corrector = Corrector.create(leftPoints, rightPoints, fundamentalMatrix,
                CorrectorType.SAMPSON_CORRECTOR);

        // check correctness
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());
        assertTrue(corrector instanceof SampsonCorrector);

        // GOLD STANDARD
        corrector = Corrector.create(leftPoints, rightPoints, fundamentalMatrix,
                CorrectorType.GOLD_STANDARD);

        // check correctness
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.GOLD_STANDARD, corrector.getType());
        assertTrue(corrector instanceof GoldStandardCorrector);

        // create with listener and type
        final CorrectorListener listener = new CorrectorListener() {

            @Override
            public void onCorrectStart(final Corrector corrector) {
            }

            @Override
            public void onCorrectEnd(final Corrector corrector) {
            }

            @Override
            public void onCorrectProgressChange(final Corrector corrector, final float progress) {
            }
        };

        // SAMPSON
        corrector = Corrector.create(listener, CorrectorType.SAMPSON_CORRECTOR);

        // check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(listener, corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());
        assertTrue(corrector instanceof SampsonCorrector);

        // GOLD STANDARD
        corrector = Corrector.create(listener, CorrectorType.GOLD_STANDARD);

        // check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(listener, corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.GOLD_STANDARD, corrector.getType());
        assertTrue(corrector instanceof GoldStandardCorrector);

        // create with fundamental matrix, listener and type

        // SAMPSON
        corrector = Corrector.create(fundamentalMatrix, listener, CorrectorType.SAMPSON_CORRECTOR);

        // check correctness
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(listener, corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());
        assertTrue(corrector instanceof SampsonCorrector);

        // GOLD STANDARD
        corrector = Corrector.create(fundamentalMatrix, listener, CorrectorType.GOLD_STANDARD);

        // check correctness
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(listener, corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.GOLD_STANDARD, corrector.getType());
        assertTrue(corrector instanceof GoldStandardCorrector);

        // create with points, listener and type

        // SAMPSON
        corrector = Corrector.create(leftPoints, rightPoints, listener, CorrectorType.SAMPSON_CORRECTOR);

        // check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(listener, corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());
        assertTrue(corrector instanceof SampsonCorrector);

        // GOLD STANDARD
        corrector = Corrector.create(leftPoints, rightPoints, listener, CorrectorType.GOLD_STANDARD);

        // check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(listener, corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.GOLD_STANDARD, corrector.getType());
        assertTrue(corrector instanceof GoldStandardCorrector);

        // create with points, fundamental matrix, listener and type

        // SAMPSON
        corrector = Corrector.create(leftPoints, rightPoints, fundamentalMatrix,
                listener, CorrectorType.SAMPSON_CORRECTOR);

        // check correctness
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(listener, corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());
        assertTrue(corrector instanceof SampsonCorrector);

        // GOLD STANDARD
        corrector = Corrector.create(leftPoints, rightPoints, fundamentalMatrix,
                listener, CorrectorType.GOLD_STANDARD);

        // check correctness
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(listener, corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.GOLD_STANDARD, corrector.getType());
        assertTrue(corrector instanceof GoldStandardCorrector);

        // create without arguments
        corrector = Corrector.create();

        // check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(Corrector.DEFAULT_TYPE, corrector.getType());

        // create with fundamental matrix
        corrector = Corrector.create(fundamentalMatrix);

        // check correctness
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(Corrector.DEFAULT_TYPE, corrector.getType());

        // create with left and right points
        corrector = Corrector.create(leftPoints, rightPoints);

        // check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(Corrector.DEFAULT_TYPE, corrector.getType());

        // create with left and right points and fundamental matrix
        corrector = Corrector.create(leftPoints, rightPoints, fundamentalMatrix);

        // check correctness
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(Corrector.DEFAULT_TYPE, corrector.getType());

        // create with listener
        corrector = Corrector.create(listener);

        // check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(listener, corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(Corrector.DEFAULT_TYPE, corrector.getType());

        // create with fundamental matrix and listener
        corrector = Corrector.create(fundamentalMatrix, listener);

        // check correctness
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(listener, corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(Corrector.DEFAULT_TYPE, corrector.getType());

        // create with left and right points and listener
        corrector = Corrector.create(leftPoints, rightPoints, listener);

        // check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(listener, corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(Corrector.DEFAULT_TYPE, corrector.getType());

        // create with left and right points, fundamental matrix and listener
        corrector = Corrector.create(leftPoints, rightPoints, fundamentalMatrix, listener);

        // check correctness
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(listener, corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(Corrector.DEFAULT_TYPE, corrector.getType());
    }
}
