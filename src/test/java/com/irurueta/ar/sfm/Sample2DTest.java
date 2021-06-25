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
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point2D;
import org.junit.Test;

import java.io.IOException;

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

    @Test
    public void testSerializeDeserialize() throws WrongSizeException, IOException,
            ClassNotFoundException {
        final Sample2D s1 = new Sample2D();

        // set new values
        s1.setId("id");
        s1.setViewId(1);
        final InhomogeneousPoint2D point = new InhomogeneousPoint2D(1.0, 2.0);
        s1.setPoint(point);
        final ReconstructedPoint3D reconstructedPoint = new ReconstructedPoint3D();
        reconstructedPoint.setPoint(new InhomogeneousPoint3D(1.0, 2.0, 3.0));
        reconstructedPoint.setId("2");
        reconstructedPoint.setInlier(true);
        s1.setReconstructedPoint(reconstructedPoint);
        s1.setQualityScore(1.0);
        final Matrix cov = Matrix.identity(2, 2);
        s1.setCovariance(cov);
        final PointColorData data = new CustomPointColorData();
        data.setId("3");
        data.setQualityScore(1.0);
        s1.setColorData(data);

        // check
        assertEquals("id", s1.getId());
        assertEquals(1, s1.getViewId());
        assertSame(point, s1.getPoint());
        assertSame(reconstructedPoint, s1.getReconstructedPoint());
        assertEquals(1.0, s1.getQualityScore(), 0.0);
        assertSame(cov, s1.getCovariance());
        assertSame(data, s1.getColorData());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(s1);
        final Sample2D s2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(s1.getId(), s2.getId());
        assertEquals(s1.getViewId(), s2.getViewId());
        assertEquals(s1.getPoint(), s2.getPoint());
        assertEquals(s1.getReconstructedPoint().getPoint(),
                s2.getReconstructedPoint().getPoint());
        assertEquals(s1.getReconstructedPoint().getId(),
                s2.getReconstructedPoint().getId());
        assertEquals(s1.getReconstructedPoint().isInlier(),
                s2.getReconstructedPoint().isInlier());
        assertEquals(s1.getQualityScore(), s2.getQualityScore(), 0.0);
        assertEquals(s1.getCovariance(), s2.getCovariance());
        assertEquals(s1.getColorData().getId(),
                s2.getColorData().getId());
        assertEquals(s1.getColorData().getQualityScore(),
                s2.getColorData().getQualityScore(), 0.0);
    }

    public static class CustomPointColorData extends PointColorData {

        @Override
        public void average(final PointColorData other, final PointColorData result) {
        }
    }
}
