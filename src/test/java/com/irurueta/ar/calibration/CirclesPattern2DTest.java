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
import com.irurueta.geometry.Point2D;
import org.junit.Test;

import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

public class CirclesPattern2DTest {

    @Test
    public void testConstructor() {
        final CirclesPattern2D pattern = new CirclesPattern2D();

        // check default values
        assertEquals(CirclesPattern2D.DEFAULT_POINT_SEPARATION, pattern.getPointSeparation(), 0.0);
        assertEquals(CirclesPattern2D.DEFAULT_COLS, pattern.getCols());
        assertEquals(CirclesPattern2D.DEFAULT_ROWS, pattern.getRows());
        assertEquals(Pattern2DType.CIRCLES, pattern.getType());
        assertEquals(CirclesPattern2D.DEFAULT_NUMBER_OF_POINTS, pattern.getNumberOfPoints());
    }

    @Test
    public void testGetSetPointSeparation() {
        final CirclesPattern2D pattern = new CirclesPattern2D();

        // check default value
        assertEquals(CirclesPattern2D.DEFAULT_POINT_SEPARATION, pattern.getPointSeparation(), 0.0);

        // set new value
        pattern.setPointSeparation(1.0);

        // check correctness
        assertEquals(1.0, pattern.getPointSeparation(), 0.0);

        // force IllegalArgumentException
        try {
            pattern.setPointSeparation(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetCols() {
        final CirclesPattern2D pattern = new CirclesPattern2D();

        // check default value
        assertEquals(CirclesPattern2D.DEFAULT_COLS, pattern.getCols());
        assertEquals(CirclesPattern2D.DEFAULT_NUMBER_OF_POINTS, pattern.getNumberOfPoints());

        // set new value
        pattern.setCols(2);

        // check correctness
        assertEquals(2, pattern.getCols());
        assertEquals(pattern.getRows() * pattern.getCols(), pattern.getNumberOfPoints());

        // force IllegalArgumentException
        try {
            pattern.setCols(1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetRows() {
        final CirclesPattern2D pattern = new CirclesPattern2D();

        // check default value
        assertEquals(CirclesPattern2D.DEFAULT_ROWS, pattern.getRows());
        assertEquals(CirclesPattern2D.DEFAULT_NUMBER_OF_POINTS, pattern.getNumberOfPoints());

        // set new value
        pattern.setRows(2);

        // check correctness
        assertEquals(2, pattern.getRows());
        assertEquals(pattern.getRows() * pattern.getCols(), pattern.getNumberOfPoints());

        // force IllegalArgumentException
        try {
            pattern.setRows(1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetIdealPoints() {
        final CirclesPattern2D pattern = new CirclesPattern2D();
        pattern.setPointSeparation(1.0);
        pattern.setCols(3);
        pattern.setRows(3);

        final List<Point2D> points = pattern.getIdealPoints();

        assertEquals(0.0, points.get(0).getInhomX(), 0.0);
        assertEquals(0.0, points.get(0).getInhomY(), 0.0);

        assertEquals(2.0, points.get(1).getInhomX(), 0.0);
        assertEquals(0.0, points.get(1).getInhomY(), 0.0);

        assertEquals(4.0, points.get(2).getInhomX(), 0.0);
        assertEquals(0.0, points.get(2).getInhomY(), 0.0);

        assertEquals(1.0, points.get(3).getInhomX(), 0.0);
        assertEquals(1.0, points.get(3).getInhomY(), 0.0);

        assertEquals(3.0, points.get(4).getInhomX(), 0.0);
        assertEquals(1.0, points.get(4).getInhomY(), 0.0);

        assertEquals(5.0, points.get(5).getInhomX(), 0.0);
        assertEquals(1.0, points.get(5).getInhomY(), 0.0);

        assertEquals(0.0, points.get(6).getInhomX(), 0.0);
        assertEquals(2.0, points.get(6).getInhomY(), 0.0);

        assertEquals(2.0, points.get(7).getInhomX(), 0.0);
        assertEquals(2.0, points.get(7).getInhomY(), 0.0);

        assertEquals(4.0, points.get(8).getInhomX(), 0.0);
        assertEquals(2.0, points.get(8).getInhomY(), 0.0);

        assertEquals(points.size(), pattern.getNumberOfPoints());
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final CirclesPattern2D pattern1 = new CirclesPattern2D();

        // set new values
        pattern1.setPointSeparation(1.0);
        pattern1.setCols(2);
        pattern1.setRows(2);

        // check
        assertEquals(1.0, pattern1.getPointSeparation(), 0.0);
        assertEquals(2, pattern1.getCols());
        assertEquals(2, pattern1.getRows());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(pattern1);
        final CirclesPattern2D pattern2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(pattern1.getPointSeparation(),
                pattern2.getPointSeparation(), 0.0);
        assertEquals(pattern1.getCols(), pattern2.getCols());
        assertEquals(pattern1.getRows(), pattern2.getRows());
    }
}
