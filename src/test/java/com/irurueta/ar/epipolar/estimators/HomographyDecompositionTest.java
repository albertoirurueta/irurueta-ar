/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.ar.epipolar.estimators;

import com.irurueta.geometry.EuclideanTransformation3D;
import org.junit.Test;

import static org.junit.Assert.*;

public class HomographyDecompositionTest {

    @Test
    public void testConstructor() {
        // empty constructor
        HomographyDecomposition decomposition = new HomographyDecomposition();

        // check default values
        assertNull(decomposition.getTransformation());
        assertNull(decomposition.getPlaneNormal());
        assertEquals(0.0, decomposition.getPlaneDistance(), 0.0);

        // test non-empty constructor
        final EuclideanTransformation3D transformation = new EuclideanTransformation3D();
        final double[] planeNormal = new double[HomographyDecomposition.PLANE_NORMAL_LENGTH];
        decomposition = new HomographyDecomposition(transformation, planeNormal, 5.0);

        // check correctness
        assertSame(transformation, decomposition.getTransformation());
        assertSame(planeNormal, decomposition.getPlaneNormal());
        assertEquals(5.0, decomposition.getPlaneDistance(), 0.0);

        // Force IllegalArgumentException
        decomposition = null;
        final double[] wrong = new double[1];
        try {
            decomposition = new HomographyDecomposition(transformation, wrong, 3.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(decomposition);
    }

    @Test
    public void testGetSetTransformation() {
        final HomographyDecomposition decomposition = new HomographyDecomposition();

        // check default value
        assertNull(decomposition.getTransformation());

        // set new value
        final EuclideanTransformation3D transformation = new EuclideanTransformation3D();
        decomposition.setTransformation(transformation);

        // check correctness
        assertSame(decomposition.getTransformation(), transformation);
    }

    @Test
    public void testGetSetPlaneNormal() {
        final HomographyDecomposition decomposition = new HomographyDecomposition();

        // check default value
        assertNull(decomposition.getPlaneNormal());

        // set new value
        final double[] planeNormal = new double[HomographyDecomposition.PLANE_NORMAL_LENGTH];
        decomposition.setPlaneNormal(planeNormal);

        // check correctness
        assertSame(planeNormal, decomposition.getPlaneNormal());

        // force IllegalArgumentException
        final double[] wrong = new double[1];
        try {
            decomposition.setPlaneNormal(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetPlaneDistance() {
        final HomographyDecomposition decomposition = new HomographyDecomposition();

        // initial value
        assertEquals(0.0, decomposition.getPlaneDistance(), 0.0);

        // set new value
        decomposition.setPlaneDistance(10.0);

        // check correctness
        assertEquals(10.0, decomposition.getPlaneDistance(), 0.0);
    }
}
