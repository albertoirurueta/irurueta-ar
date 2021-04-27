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
import com.irurueta.geometry.Point3D;
import org.junit.Test;

import static org.junit.Assert.*;

public class ReconstructedPoint3DTest {

    @Test
    public void testConstructor() {
        final ReconstructedPoint3D rp = new ReconstructedPoint3D();

        // check default values
        assertNull(rp.getId());
        assertNull(rp.getPoint());
        assertEquals(rp.getQualityScore(),
                ReconstructedPoint3D.DEFAULT_QUALITY_SCORE, 0.0);
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
        assertEquals(rp.getId(), "id");
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
        assertSame(rp.getPoint(), p);
    }

    @Test
    public void testGetSetQualityScore() {
        final ReconstructedPoint3D rp = new ReconstructedPoint3D();

        // check default value
        assertEquals(rp.getQualityScore(),
                ReconstructedPoint3D.DEFAULT_QUALITY_SCORE, 0.0);

        // set new value
        rp.setQualityScore(5.0);

        // check correctness
        assertEquals(rp.getQualityScore(), 5.0, 0.0);
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
        assertSame(rp.getCovariance(), cov);
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
        assertSame(rp.getColorData(), data);
    }

    public static class CustomPointColorData extends PointColorData {

        @Override
        public void average(final PointColorData other, final PointColorData result) {
        }
    }
}
