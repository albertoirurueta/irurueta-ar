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
package com.irurueta.ar.calibration;

import com.irurueta.ar.SerializationHelper;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class QRPattern2DTest {

    public static final double ABSOLUTE_ERROR = 1e-6;

    @Test
    void testConstants() {
        assertEquals(4, QRPattern2D.NUMBER_OF_POINTS);
        assertEquals(2, QRPattern2D.QR_VERSION);
        assertEquals(25, QRPattern2D.NUMBER_OF_MODULES);
        assertEquals(4, QRPattern2D.ORIGIN_OFFSET);
        assertEquals(1.1e-2, QRPattern2D.DEFAULT_QR_CODE_WIDTH, 0.0);
        assertEquals(1.1e-2, QRPattern2D.DEFAULT_QR_CODE_HEIGHT, 0.0);
    }

    @Test
    void testConstructor() {
        final var pattern = new QRPattern2D();

        // check default values
        assertEquals(QRPattern2D.DEFAULT_QR_CODE_WIDTH, pattern.getCodeWidth(), 0.0);
        assertEquals(QRPattern2D.DEFAULT_QR_CODE_HEIGHT, pattern.getCodeHeight(), 0.0);
        assertEquals(Pattern2DType.QR, pattern.getType());
        assertEquals(QRPattern2D.NUMBER_OF_POINTS, pattern.getNumberOfPoints());
    }

    @Test
    void testGetSetCodeWidth() {
        final var pattern = new QRPattern2D();

        // check default value
        assertEquals(QRPattern2D.DEFAULT_QR_CODE_WIDTH, pattern.getCodeWidth(), 0.0);

        // set new value
        pattern.setCodeWidth(5.0);

        // check correctness
        assertEquals(5.0, pattern.getCodeWidth(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> pattern.setCodeWidth(0.0));
    }

    @Test
    void testGetSetCodeHeight() {
        final var pattern = new QRPattern2D();

        // check default value
        assertEquals(QRPattern2D.DEFAULT_QR_CODE_HEIGHT, pattern.getCodeHeight(), 0.0);

        // set new value
        pattern.setCodeHeight(10.0);

        // check correctness
        assertEquals(10.0, pattern.getCodeHeight(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> pattern.setCodeHeight(0.0));
    }

    @Test
    void testGetIdealPoints() {
        final var pattern = new QRPattern2D();

        // check correctness
        final var points = pattern.getIdealPoints();

        // check correctness
        assertEquals(4, points.size());

        // by default QR codes are assumed to be 1.1 x 1.1 centimeters

        // 1st point is bottom-left finder pattern, which is located at:
        // (0cm, 0.792cm)
        assertEquals(0.0, points.get(0).getInhomX(), 0.0);
        assertEquals(0.00792, points.get(0).getInhomY(), ABSOLUTE_ERROR);

        // 2nd point is top-left finder pattern, which is located at:
        // (0cm, 0cm)
        assertEquals(0.0, points.get(1).getInhomX(), ABSOLUTE_ERROR);
        assertEquals(0.0, points.get(1).getInhomY(), ABSOLUTE_ERROR);

        // 3rd point is top-right finder pattern, which is located at:
        // (0.792cm, 0cm)
        assertEquals(0.00792, points.get(2).getInhomX(), ABSOLUTE_ERROR);
        assertEquals(0.0, points.get(2).getInhomY(), ABSOLUTE_ERROR);

        // 4th point is bottom-right finder pattern, which is located at:
        // (0.66cm, 0.66cm)
        assertEquals(0.0066, points.get(3).getInhomX(), ABSOLUTE_ERROR);
        assertEquals(0.0066, points.get(3).getInhomY(), ABSOLUTE_ERROR);

        assertEquals(points.size(), pattern.getNumberOfPoints());
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var pattern1 = new QRPattern2D();

        // set new values
        pattern1.setCodeWidth(1.0);
        pattern1.setCodeHeight(2.0);

        // check
        assertEquals(1.0, pattern1.getCodeWidth(), 0.0);
        assertEquals(2.0, pattern1.getCodeHeight(), 0.0);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(pattern1);
        final var pattern2 = SerializationHelper.<QRPattern2D>deserialize(bytes);

        // check
        assertEquals(pattern1.getCodeWidth(), pattern2.getCodeWidth(), 0.0);
        assertEquals(pattern1.getCodeHeight(), pattern2.getCodeHeight(), 0.0);
    }
}
