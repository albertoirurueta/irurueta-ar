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
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SingleCorrectorTest {

    @Test
    void testCreate() {
        // test create with method

        // Gold Standard
        var corrector = SingleCorrector.create(CorrectorType.GOLD_STANDARD);

        // check correctness
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertNull(corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(CorrectorType.GOLD_STANDARD, corrector.getType());
        assertInstanceOf(GoldStandardSingleCorrector.class, corrector);

        // Sampson
        corrector = SingleCorrector.create(CorrectorType.SAMPSON_CORRECTOR);

        // check correctness
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertNull(corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());
        assertInstanceOf(SampsonSingleCorrector.class, corrector);

        // test create with fundamental matrix and type
        final var fundamentalMatrix = new FundamentalMatrix();

        // Gold standard
        corrector = SingleCorrector.create(fundamentalMatrix, CorrectorType.GOLD_STANDARD);

        // check correctness
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(CorrectorType.GOLD_STANDARD, corrector.getType());
        assertInstanceOf(GoldStandardSingleCorrector.class, corrector);

        // Sampson
        corrector = SingleCorrector.create(fundamentalMatrix, CorrectorType.SAMPSON_CORRECTOR);

        // check correctness
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());
        assertInstanceOf(SampsonSingleCorrector.class, corrector);

        // test create with left and right points
        final var leftPoint = Point2D.create();
        final var rightPoint = Point2D.create();

        // Gold standard
        corrector = SingleCorrector.create(leftPoint, rightPoint, CorrectorType.GOLD_STANDARD);

        // check correctness
        assertSame(leftPoint, corrector.getLeftPoint());
        assertSame(rightPoint, corrector.getRightPoint());
        assertNull(corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(CorrectorType.GOLD_STANDARD, corrector.getType());
        assertInstanceOf(GoldStandardSingleCorrector.class, corrector);

        // Sampson
        corrector = SingleCorrector.create(leftPoint, rightPoint, CorrectorType.SAMPSON_CORRECTOR);

        // check correctness
        assertSame(leftPoint, corrector.getLeftPoint());
        assertSame(rightPoint, corrector.getRightPoint());
        assertNull(corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());
        assertInstanceOf(SampsonSingleCorrector.class, corrector);

        // test create with left and right points and fundamental matrix

        // Gold standard
        corrector = SingleCorrector.create(leftPoint, rightPoint, fundamentalMatrix, CorrectorType.GOLD_STANDARD);

        // check correctness
        assertSame(leftPoint, corrector.getLeftPoint());
        assertSame(rightPoint, corrector.getRightPoint());
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        // fundamental matrix is not defined
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(CorrectorType.GOLD_STANDARD, corrector.getType());
        assertInstanceOf(GoldStandardSingleCorrector.class, corrector);

        // Sampson
        corrector = SingleCorrector.create(leftPoint, rightPoint, fundamentalMatrix, CorrectorType.SAMPSON_CORRECTOR);

        // check correctness
        assertSame(leftPoint, corrector.getLeftPoint());
        assertSame(rightPoint, corrector.getRightPoint());
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        // fundamental matrix is not defined
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());
        assertInstanceOf(SampsonSingleCorrector.class, corrector);

        // test create without arguments
        corrector = SingleCorrector.create();

        // check corrector
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertNull(corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(SingleCorrector.DEFAULT_TYPE, corrector.getType());

        // test create with fundamental matrix
        corrector = SingleCorrector.create(fundamentalMatrix);

        // check correctness
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(SingleCorrector.DEFAULT_TYPE, corrector.getType());

        // test create with left and right points
        corrector = SingleCorrector.create(leftPoint, rightPoint);

        // check correctness
        assertSame(leftPoint, corrector.getLeftPoint());
        assertSame(rightPoint, corrector.getRightPoint());
        assertNull(corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(SingleCorrector.DEFAULT_TYPE, corrector.getType());

        // test create with left and right points and fundamental matrix
        corrector = SingleCorrector.create(leftPoint, rightPoint, fundamentalMatrix);

        // check correctness
        assertSame(leftPoint, corrector.getLeftPoint());
        assertSame(rightPoint, corrector.getRightPoint());
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(SingleCorrector.DEFAULT_TYPE, corrector.getType());
    }
}
