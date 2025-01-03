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

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class Pattern2DTest {

    @Test
    void testCreate() {
        var pattern = Pattern2D.create(Pattern2DType.QR);

        assertEquals(Pattern2DType.QR, pattern.getType());
        assertInstanceOf(QRPattern2D.class, pattern);

        pattern = Pattern2D.create(Pattern2DType.CIRCLES);

        assertEquals(Pattern2DType.CIRCLES, pattern.getType());
        assertInstanceOf(CirclesPattern2D.class, pattern);
    }
}
