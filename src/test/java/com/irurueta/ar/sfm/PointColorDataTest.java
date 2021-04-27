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
package com.irurueta.ar.sfm;

import org.junit.Test;

import static org.junit.Assert.*;

public class PointColorDataTest {

    @Test
    public void testConstructor() {
        final PointColorData data = new CustomPointColorData();

        // check default value
        assertNull(data.getId());
        assertEquals(data.getQualityScore(),
                PointColorData.DEFAULT_QUALITY_SCORE, 0.0);
    }

    @Test
    public void testGetSetId() {
        final PointColorData data = new CustomPointColorData();

        // check default value
        assertNull(data.getId());

        // set new value
        data.setId("id");

        // check correctness
        assertEquals(data.getId(), "id");
    }

    @Test
    public void testGetSetQualityScore() {
        final PointColorData data = new CustomPointColorData();

        // check default value
        assertEquals(data.getQualityScore(),
                PointColorData.DEFAULT_QUALITY_SCORE, 0.0);

        // set new value
        data.setQualityScore(5.0);

        // check correctness
        assertEquals(data.getQualityScore(), 5.0, 0.0);
    }

    @Test
    public void testAverage() {
        final PointColorData data1 = new CustomPointColorData();
        final PointColorData data2 = new CustomPointColorData();
        final PointColorData data3 = new CustomPointColorData();

        data1.average(data2, data3);

        assertNotNull(data1);
        assertNotNull(data2);
        assertNotNull(data3);
    }

    public static class CustomPointColorData extends PointColorData {

        @Override
        public void average(final PointColorData other, final PointColorData result) {
        }
    }
}
