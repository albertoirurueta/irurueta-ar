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
package com.irurueta.ar.calibration.estimators;

import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RadialDistortionEstimatorTest {
    
    public RadialDistortionEstimatorTest() { }
    
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
        RadialDistortionEstimator estimator =
                RadialDistortionEstimator.create();
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR);
        
        //test with type
        estimator = RadialDistortionEstimator.create(
                RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());        
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR);        
        
        estimator = RadialDistortionEstimator.create(
                RadialDistortionEstimatorType.WEIGHTED_RADIAL_DISTORTION_ESTIMATOR);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());        
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), 
                RadialDistortionEstimatorType.WEIGHTED_RADIAL_DISTORTION_ESTIMATOR);
    }
    
    @Test
    public void testGetSetdistortedUndistortedPoints() throws LockedException {
        RadialDistortionEstimator estimator = 
                RadialDistortionEstimator.create();
        
        //check default values
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        
        //set new value
        List<Point2D> distortedPoints = new ArrayList<>();
        List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < estimator.getMinNumberOfMatchedPoints(); i++){
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        
        estimator.setPoints(distortedPoints, undistortedPoints);
        
        //check correctness
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        
        //Force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<>();
        try {
            estimator.setPoints(emptyPoints, undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setPoints(emptyPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setPoints(null, undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setPoints(distortedPoints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetDistortionCenter() throws LockedException {
        RadialDistortionEstimator estimator = 
                RadialDistortionEstimator.create();

        //check default value
        assertNull(estimator.getDistortionCenter());
        
        //set new value
        Point2D center = Point2D.create();
        estimator.setDistortionCenter(center);
        
        //check correctness
        assertSame(estimator.getDistortionCenter(), center);
    }
    
    @Test
    public void testAreValidLists() {
        RadialDistortionEstimator estimator = 
                RadialDistortionEstimator.create();
        
        List<Point2D> distortedPoints = new ArrayList<>();
        List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < estimator.getMinNumberOfMatchedPoints(); i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        List<Point2D> emptyPoints = new ArrayList<>();

        assertTrue(estimator.areValidPoints(distortedPoints, 
                undistortedPoints));
        assertFalse(estimator.areValidPoints(emptyPoints, undistortedPoints));
        assertFalse(estimator.areValidPoints(distortedPoints, emptyPoints));
        assertFalse(estimator.areValidPoints(null, undistortedPoints));
        assertFalse(estimator.areValidPoints(distortedPoints, null));
    }    
}
