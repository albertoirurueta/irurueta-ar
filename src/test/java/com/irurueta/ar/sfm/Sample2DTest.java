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
import com.irurueta.geometry.Point2D;
import org.junit.Test;

import static org.junit.Assert.*;

public class Sample2DTest {

    @Test
    public void testConstructor() {
        final Sample2D s = new Sample2D();

        // check default values
        assertNull(s.getId());
        assertEquals(s.getViewId(), 0);
        assertNull(s.getPoint());
        assertNull(s.getReconstructedPoint());
        assertEquals(s.getQualityScore(), Sample2D.DEFAULT_QUALITY_SCORE, 0.0);
        assertNull(s.getCovariance());
        assertNull(s.getColorData());
    }

    @Test
    public void testGetSetId() {
        final Sample2D s = new Sample2D();

        // check default value
        assertNull(s.getId());

        // set new value
        s.setId("id");

        // check correctness
        assertEquals(s.getId(), "id");
    }

    @Test
    public void testGetSetViewId() {
        final Sample2D s = new Sample2D();

        // check default values
        assertEquals(s.getViewId(), 0);

        // set new value
        s.setViewId(5);

        // check correctness
        assertEquals(s.getViewId(), 5);
    }

    @Test
    public void testGetSetPoint() {
        final Sample2D s = new Sample2D();

        // check default value
        assertNull(s.getPoint());

        // set new value
        final Point2D p = Point2D.create();
        s.setPoint(p);

        // check correctness
        assertSame(s.getPoint(), p);
    }

    @Test
    public void testGetSetReconstructedPoint() {
        final Sample2D s = new Sample2D();

        // check default value
        assertNull(s.getReconstructedPoint());

        // set new value
        final ReconstructedPoint3D reconstructedPoint3D = new ReconstructedPoint3D();
        s.setReconstructedPoint(reconstructedPoint3D);

        // check correctness
        assertSame(s.getReconstructedPoint(), reconstructedPoint3D);
    }

    @Test
    public void testGetSetQualityScore() {
        final Sample2D s = new Sample2D();

        // check default value
        assertEquals(s.getQualityScore(), Sample2D.DEFAULT_QUALITY_SCORE, 0.0);

        // set new value
        s.setQualityScore(15.0);

        // check correctness
        assertEquals(s.getQualityScore(), 15.0, 0.0);
    }

    @Test
    public void testGetSetCovariance() throws WrongSizeException {
        final Sample2D s = new Sample2D();

        // check default value
        assertNull(s.getCovariance());

        // set new value
        final Matrix cov = new Matrix(2, 2);
        s.setCovariance(cov);

        // check correctness
        assertSame(s.getCovariance(), cov);
    }

    @Test
    public void testGetSetColorData() {
        final Sample2D s = new Sample2D();

        // check default value
        assertNull(s.getColorData());

        // set new value
        final PointColorData data = new CustomPointColorData();
        s.setColorData(data);

        // check correctness
        assertSame(s.getColorData(), data);
    }

    public static class CustomPointColorData extends PointColorData {

        @Override
        public void average(final PointColorData other, final PointColorData result) {
        }
    }
}
