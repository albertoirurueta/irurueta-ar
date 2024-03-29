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

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class Pattern2DTest {

    @Test
    public void testCreate() {
        Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);

        assertEquals(Pattern2DType.QR, pattern.getType());
        assertTrue(pattern instanceof QRPattern2D);

        pattern = Pattern2D.create(Pattern2DType.CIRCLES);

        assertEquals(Pattern2DType.CIRCLES, pattern.getType());
        assertTrue(pattern instanceof CirclesPattern2D);
    }
}
