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
import org.junit.*;

import static org.junit.Assert.*;

public class HomographyDecompositionTest {
    
    public HomographyDecompositionTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testConstructor() {
        //empty constructor
        HomographyDecomposition decomposition = new HomographyDecomposition();
        
        //check default values
        assertNull(decomposition.getTransformation());
        assertNull(decomposition.getPlaneNormal());
        assertEquals(decomposition.getPlaneDistance(), 0.0, 0.0);
        
        //test non-empty constructor
        EuclideanTransformation3D transformation = 
                new EuclideanTransformation3D();
        double[] planeNormal = new double[
                HomographyDecomposition.PLANE_NORMAL_LENGTH];
        decomposition = new HomographyDecomposition(transformation, 
                planeNormal, 5.0);
        
        //check correctness
        assertSame(decomposition.getTransformation(), transformation);
        assertSame(decomposition.getPlaneNormal(), planeNormal);
        assertEquals(decomposition.getPlaneDistance(), 5.0, 0.0);
        
        //Force IllegalArgumentException
        decomposition = null;
        double[] wrong = new double[1];
        try {
            decomposition = new HomographyDecomposition(transformation, wrong, 
                    3.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(decomposition);
    }
    
    @Test
    public void testGetSetTransformation() {
        HomographyDecomposition decomposition = new HomographyDecomposition();
        
        //check default value
        assertNull(decomposition.getTransformation());
        
        //set new value
        EuclideanTransformation3D transformation =
                new EuclideanTransformation3D();
        decomposition.setTransformation(transformation);
        
        //check correctness
        assertSame(decomposition.getTransformation(), transformation);
    }
    
    @Test
    public void testGetSetPlaneNormal() {
        HomographyDecomposition decomposition = new HomographyDecomposition();
        
        //check default value
        assertNull(decomposition.getPlaneNormal());
        
        //set new value
        double[] planeNormal = new double[
                HomographyDecomposition.PLANE_NORMAL_LENGTH];
        decomposition.setPlaneNormal(planeNormal);
        
        //check correctness
        assertSame(decomposition.getPlaneNormal(), planeNormal);
        
        //force IllegalArgumentException
        double[] wrong = new double[1];
        try {
            decomposition.setPlaneNormal(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetPlaneDistance() {
        HomographyDecomposition decomposition = new HomographyDecomposition();
        
        //initial value
        assertEquals(decomposition.getPlaneDistance(), 0.0, 0.0);
        
        //set new value
        decomposition.setPlaneDistance(10.0);
        
        //check correctness
        assertEquals(decomposition.getPlaneDistance(), 10.0, 0.0);
    }
}
