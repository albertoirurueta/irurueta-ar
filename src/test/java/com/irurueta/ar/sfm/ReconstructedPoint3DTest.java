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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.ar.SerializationHelper;
import com.irurueta.geometry.Point3D;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.*;

public class ReconstructedPoint3DTest {

    @Test
    public void testConstructor() {
        final ReconstructedPoint3D rp = new ReconstructedPoint3D();

        // check default values
        assertNull(rp.getId());
        assertNull(rp.getPoint());
        assertEquals(ReconstructedPoint3D.DEFAULT_QUALITY_SCORE, rp.getQualityScore(), 0.0);
        assertNull(rp.getCovariance());
        assertNull(rp.getColorData());
    }

    @Test
    public void testGetSetId() {
        final ReconstructedPoint3D rp = new ReconstructedPoint3D();

        // check default value
        assertNull(rp.getId());

        // set new value
        rp.setId("id");

        // check correctness
        assertEquals("id", rp.getId());
    }

    @Test
    public void testGetSetPoint() {
        final ReconstructedPoint3D rp = new ReconstructedPoint3D();

        // check default value
        assertNull(rp.getPoint());

        // set new value
        final Point3D p = Point3D.create();
        rp.setPoint(p);

        // check correctness
        assertSame(p, rp.getPoint());
    }

    @Test
    public void testGetSetQualityScore() {
        final ReconstructedPoint3D rp = new ReconstructedPoint3D();

        // check default value
        assertEquals(ReconstructedPoint3D.DEFAULT_QUALITY_SCORE, rp.getQualityScore(), 0.0);

        // set new value
        rp.setQualityScore(5.0);

        // check correctness
        assertEquals(5.0, rp.getQualityScore(), 0.0);
    }

    @Test
    public void testGetSetCovariance() throws WrongSizeException {
        final ReconstructedPoint3D rp = new ReconstructedPoint3D();

        // check default value
        assertNull(rp.getCovariance());

        // set new value
        final Matrix cov = new Matrix(3, 3);
        rp.setCovariance(cov);

        // check correctness
        assertSame(cov, rp.getCovariance());
    }

    @Test
    public void testGetSetColorData() {
        final ReconstructedPoint3D rp = new ReconstructedPoint3D();

        // check default value
        assertNull(rp.getColorData());

        // set new value
        final PointColorData data = new CustomPointColorData();
        rp.setColorData(data);

        // check correctness
        assertSame(data, rp.getColorData());
    }

    @Test
    public void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final ReconstructedPoint3D rp1 = new ReconstructedPoint3D();

        // set values
        rp1.setId("id");
        final Point3D p = Point3D.create();
        rp1.setPoint(p);
        rp1.setQualityScore(5.0);
        final Matrix cov = new Matrix(3, 3);
        rp1.setCovariance(cov);
        final PointColorData data = new CustomPointColorData();
        rp1.setColorData(data);

        // check
        assertEquals("id", rp1.getId());
        assertSame(p, rp1.getPoint());
        assertEquals(5.0, rp1.getQualityScore(), 0.0);
        assertSame(cov, rp1.getCovariance());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(rp1);
        final ReconstructedPoint3D rp2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(rp1.getId(), rp2.getId());
        assertEquals(rp1.getPoint(), rp2.getPoint());
        assertEquals(rp1.getQualityScore(), rp2.getQualityScore(), 0.0);
        assertEquals(rp1.getCovariance(), rp2.getCovariance());
    }

    public static class CustomPointColorData extends PointColorData {

        @Override
        public void average(final PointColorData other, final PointColorData result) {
        }
    }
}
