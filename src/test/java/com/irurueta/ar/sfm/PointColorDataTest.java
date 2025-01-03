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

import com.irurueta.ar.SerializationHelper;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class PointColorDataTest {

    @Test
    void testConstructor() {
        final var data = new CustomPointColorData();

        // check default value
        assertNull(data.getId());
        assertEquals(PointColorData.DEFAULT_QUALITY_SCORE, data.getQualityScore(), 0.0);
    }

    @Test
    void testGetSetId() {
        final var data = new CustomPointColorData();

        // check default value
        assertNull(data.getId());

        // set new value
        data.setId("id");

        // check correctness
        assertEquals("id", data.getId());
    }

    @Test
    void testGetSetQualityScore() {
        final var data = new CustomPointColorData();

        // check default value
        assertEquals(PointColorData.DEFAULT_QUALITY_SCORE, data.getQualityScore(), 0.0);

        // set new value
        data.setQualityScore(5.0);

        // check correctness
        assertEquals(5.0, data.getQualityScore(), 0.0);
    }

    @Test
    void testAverage() {
        final var data1 = new CustomPointColorData();
        final var data2 = new CustomPointColorData();
        final var data3 = new CustomPointColorData();

        data1.average(data2, data3);

        assertNotNull(data1);
        assertNotNull(data2);
        assertNotNull(data3);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var data1 = new CustomPointColorData();

        // set new values
        data1.setId("id");
        data1.setQualityScore(5.0);

        // check correctness
        assertEquals("id", data1.getId());
        assertEquals(5.0, data1.getQualityScore(), 0.0);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(data1);
        final var data2 = SerializationHelper.<PointColorData>deserialize(bytes);

        assertEquals(data1.getId(), data2.getId());
        assertEquals(data1.getQualityScore(), data2.getQualityScore(), 0.0);
    }

    public static class CustomPointColorData extends PointColorData {

        @Override
        public void average(final PointColorData other, final PointColorData result) {
            // no action needed
        }
    }
}
