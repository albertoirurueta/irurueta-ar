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
package com.irurueta.ar.epipolar.estimators;

import com.irurueta.geometry.Point2D;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class FundamentalMatrixEstimatorTest {
    
    public FundamentalMatrixEstimatorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testCreate() {
        //test create with method
        
        //7 points
        FundamentalMatrixEstimator estimator = FundamentalMatrixEstimator.
                create(FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        
        //check correctness
        assertTrue(estimator instanceof SevenPointsFundamentalMatrixEstimator);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        
        //8 points
        estimator = FundamentalMatrixEstimator.create(
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
        
        //check correctness
        assertTrue(estimator instanceof EightPointsFundamentalMatrixEstimator);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMethod(), 
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);        
        
        //affine
        estimator = FundamentalMatrixEstimator.create(
                FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM);
        
        //check correctness
        assertTrue(estimator instanceof AffineFundamentalMatrixEstimator);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMethod(), 
                FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM);        
        
        //test create with points and method
        List<Point2D> leftPoints = new ArrayList<>();
        List<Point2D> rightPoints = new ArrayList<>();
        for (int i = 0; i < 8; i++) {
            leftPoints.add(Point2D.create());
            rightPoints.add(Point2D.create());
        }
        
        //7 points
        estimator = FundamentalMatrixEstimator.create(leftPoints, rightPoints,
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        
        //check correctness
        assertTrue(estimator instanceof SevenPointsFundamentalMatrixEstimator);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMethod(),
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        
        //8 points
        estimator = FundamentalMatrixEstimator.create(leftPoints, rightPoints, 
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
        
        //check correctness
        assertTrue(estimator instanceof EightPointsFundamentalMatrixEstimator);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMethod(),
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);        
        
        //affine
        estimator = FundamentalMatrixEstimator.create(leftPoints, rightPoints, 
                FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM);
        
        //check correctness
        assertTrue(estimator instanceof AffineFundamentalMatrixEstimator);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMethod(),
                FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM);        
        
        
        //test create with default method
        estimator = FundamentalMatrixEstimator.create();
        
        assertTrue(estimator instanceof SevenPointsFundamentalMatrixEstimator);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        
        //test create with points and default method
        estimator = FundamentalMatrixEstimator.create(leftPoints, rightPoints);
        
        //check correctness
        assertTrue(estimator instanceof SevenPointsFundamentalMatrixEstimator);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getMethod(),
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);        
    }
}
