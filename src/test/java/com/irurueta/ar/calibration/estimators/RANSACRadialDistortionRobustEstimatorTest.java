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

import com.irurueta.ar.calibration.DistortionException;
import com.irurueta.ar.calibration.RadialDistortion;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.NotSupportedException;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class RANSACRadialDistortionRobustEstimatorTest implements
        RadialDistortionRobustEstimatorListener {
    
    private static final double MIN_POINT_VALUE = -1.0;
    private static final double MAX_POINT_VALUE = 1.0;
    
    private static final double MIN_PARAM_VALUE = -1e-4;
    private static final double MAX_PARAM_VALUE = 1e-4;
    
    private static final double ABSOLUTE_ERROR = 1e-8;
    
    private static final int MIN_NUM_POINTS = 500;
    private static final int MAX_NUM_POINTS = 1000;
    
    private static final double THRESHOLD = 1e-8;
    
    private static final double STD_ERROR = 100.0;
    
    private static final int PERCENTAGE_OUTLIER = 20;
    
    private static final int TIMES = 100;
    
    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    public RANSACRadialDistortionRobustEstimatorTest() { }
    
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
        //test constructor without parameters
        RANSACRadialDistortionRobustEstimator estimator =
                new RANSACRadialDistortionRobustEstimator();
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(estimator.getHorizontalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getVerticalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getSkew(), 
                RadialDistortionRobustEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalPrincipalPoint(), 
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getSkewness(),
                RadialDistortionRobustEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getNumKParams(),
                RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS);
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test construtor with listener
        estimator = new RANSACRadialDistortionRobustEstimator(this);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(estimator.getHorizontalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getVerticalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getSkew(), 
                RadialDistortionRobustEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalPrincipalPoint(), 
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getSkewness(),
                RadialDistortionRobustEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getNumKParams(),
                RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS);        
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //test constructor with points
        List<Point2D> distortedPoints = new ArrayList<>();
        List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        
        estimator = new RANSACRadialDistortionRobustEstimator(distortedPoints, 
                undistortedPoints);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertNull(estimator.getDistortionCenter());
        assertEquals(estimator.getHorizontalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getVerticalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getSkew(), 
                RadialDistortionRobustEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalPrincipalPoint(), 
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getSkewness(),
                RadialDistortionRobustEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getNumKParams(),
                RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS);        
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<>();
        estimator = null;
        try {
            estimator = new RANSACRadialDistortionRobustEstimator(emptyPoints,
                    undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RANSACRadialDistortionRobustEstimator(emptyPoints,
                    emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RANSACRadialDistortionRobustEstimator(
                    null, undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RANSACRadialDistortionRobustEstimator(
                    distortedPoints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);  
        
        //test constructor with points and listener
        estimator = new RANSACRadialDistortionRobustEstimator(distortedPoints, 
                undistortedPoints, this);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertNull(estimator.getDistortionCenter());
        assertEquals(estimator.getHorizontalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getVerticalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getSkew(), 
                RadialDistortionRobustEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalPrincipalPoint(), 
                0.0, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getSkewness(),
                RadialDistortionRobustEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getNumKParams(),
                RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS);        
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RANSACRadialDistortionRobustEstimator(emptyPoints,
                    undistortedPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RANSACRadialDistortionRobustEstimator(emptyPoints,
                    emptyPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RANSACRadialDistortionRobustEstimator(
                    null, undistortedPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RANSACRadialDistortionRobustEstimator(
                    distortedPoints, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);        
        
        //test constructor with points and center
        Point2D center = Point2D.create();
        
        estimator = new RANSACRadialDistortionRobustEstimator(distortedPoints, 
                undistortedPoints, center);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getDistortionCenter(), center);
        assertEquals(estimator.getHorizontalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getVerticalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getSkew(), 
                RadialDistortionRobustEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                center.getInhomX(), 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalPrincipalPoint(), 
                center.getInhomY(), 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getSkewness(),
                RadialDistortionRobustEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getNumKParams(),
                RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS);        
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RANSACRadialDistortionRobustEstimator(emptyPoints,
                    undistortedPoints, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RANSACRadialDistortionRobustEstimator(emptyPoints,
                    emptyPoints, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try{
            estimator = new RANSACRadialDistortionRobustEstimator(
                    null, undistortedPoints, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RANSACRadialDistortionRobustEstimator(
                    distortedPoints, null, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);  

        //test constructor with points, center and listener
        estimator = new RANSACRadialDistortionRobustEstimator(distortedPoints, 
                undistortedPoints, center, this);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getDistortionCenter(), center);
        assertEquals(estimator.getHorizontalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getVerticalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getSkew(), 
                RadialDistortionRobustEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalPrincipalPoint(),
                center.getInhomX(), 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalPrincipalPoint(), 
                center.getInhomY(), 0.0);
        assertEquals(estimator.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getVerticalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(estimator.getIntrinsic().getSkewness(),
                RadialDistortionRobustEstimator.DEFAULT_SKEW, 0.0);
        assertEquals(estimator.getNumKParams(),
                RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS);        
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new RANSACRadialDistortionRobustEstimator(emptyPoints,
                    undistortedPoints, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RANSACRadialDistortionRobustEstimator(emptyPoints,
                    emptyPoints, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RANSACRadialDistortionRobustEstimator(
                    null, undistortedPoints, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new RANSACRadialDistortionRobustEstimator(
                    distortedPoints, null, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);          
    }
    
    @Test
    public void testGetSetQualityScores() throws LockedException {
        RANSACRadialDistortionRobustEstimator estimator =
                new RANSACRadialDistortionRobustEstimator();
        
        //check default value
        assertNull(estimator.getQualityScores());
        
        //set new value
        double[] qualityScores = new double[1];
        estimator.setQualityScores(qualityScores);
        
        //check correctness
        assertNull(estimator.getQualityScores());
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException {
        RANSACRadialDistortionRobustEstimator estimator =
                new RANSACRadialDistortionRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD, 0.0);
        
        //set new value
        estimator.setThreshold(10.0);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 10.0, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetListener() throws LockedException {
        RANSACRadialDistortionRobustEstimator estimator =
                new RANSACRadialDistortionRobustEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        
        //Set new value
        estimator.setListener(this);
        
        //check correctness
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
    }
    
    @Test
    public void testGetSetProgressDelta() throws LockedException {
        RANSACRadialDistortionRobustEstimator estimator =
                new RANSACRadialDistortionRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        
        //set new value
        estimator.setProgressDelta(0.5f);
        
        //check correctness
        assertEquals(estimator.getProgressDelta(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetConfidence() throws LockedException {
        RANSACRadialDistortionRobustEstimator estimator =
                new RANSACRadialDistortionRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
        //set new value
        estimator.setConfidence(0.75);
        
        //check correctness
        assertEquals(estimator.getConfidence(), 0.75, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetMaxIterations() throws LockedException {
        RANSACRadialDistortionRobustEstimator estimator =
                new RANSACRadialDistortionRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(), 
                RANSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
        //set new value
        estimator.setMaxIterations(10);
        
        //check correctness
        assertEquals(estimator.getMaxIterations(), 10);
        
        //Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetPoints() throws LockedException {
        List<Point2D> distortedPoints = new ArrayList<>();
        List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        
        RANSACRadialDistortionRobustEstimator estimator =
                new RANSACRadialDistortionRobustEstimator();
        
        //check default values
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        
        //set new value
        estimator.setPoints(distortedPoints, undistortedPoints);
        
        //check correctness
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        
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
        RANSACRadialDistortionRobustEstimator estimator =
                new RANSACRadialDistortionRobustEstimator();
        
        //check default value
        assertNull(estimator.getDistortionCenter());
        
        //set new value
        Point2D center = Point2D.create();
        estimator.setDistortionCenter(center);
        
        //check correctness
        assertSame(estimator.getDistortionCenter(), center);
    }
    
    @Test
    public void testGetSetHorizontalFocalLength() throws LockedException {
        RANSACRadialDistortionRobustEstimator estimator =
                new RANSACRadialDistortionRobustEstimator();
        
        //check default value
        assertEquals(estimator.getHorizontalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        
        //set new value
        estimator.setHorizontalFocalLength(2.0);
        
        //check correctness
        assertEquals(estimator.getHorizontalFocalLength(), 2.0, 0.0);
    }
    
    @Test
    public void testGetSetVerticalFocalLength() throws LockedException {
        RANSACRadialDistortionRobustEstimator estimator =
                new RANSACRadialDistortionRobustEstimator();
        
        //check default value
        assertEquals(estimator.getVerticalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        
        //set new value
        estimator.setVerticalFocalLength(2.0);
        
        //check correctness
        assertEquals(estimator.getVerticalFocalLength(), 2.0, 0.0);
    }
    
    @Test
    public void testGetSetSkew() throws LockedException {
        RANSACRadialDistortionRobustEstimator estimator =
                new RANSACRadialDistortionRobustEstimator();
        
        //check default value
        assertEquals(estimator.getSkew(),
                RadialDistortionRobustEstimator.DEFAULT_SKEW, 0.0);
        
        //set new value
        estimator.setSkew(1.0);
        
        //check correctness
        assertEquals(estimator.getSkew(), 1.0, 0.0);
    }
    
    @Test
    public void testGetSetIntrinsic() throws LockedException {
        RANSACRadialDistortionRobustEstimator estimator =
                new RANSACRadialDistortionRobustEstimator();
        
        //check default value
        PinholeCameraIntrinsicParameters intrinsic = estimator.getIntrinsic();
        
        assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
        assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
        assertEquals(intrinsic.getHorizontalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(intrinsic.getVerticalFocalLength(),
                RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(intrinsic.getSkewness(),
                RadialDistortionRobustEstimator.DEFAULT_SKEW, 0.0);
        
        //set new value
        intrinsic = new PinholeCameraIntrinsicParameters(2.0, 3.0, 4.0, 5.0, 
                6.0);
        estimator.setIntrinsic(intrinsic);
        
        //check correctness
        assertEquals(intrinsic.getHorizontalPrincipalPoint(), 4.0, 0.0);
        assertEquals(intrinsic.getVerticalPrincipalPoint(), 5.0, 0.0);
        assertEquals(intrinsic.getHorizontalFocalLength(), 2.0, 0.0);
        assertEquals(intrinsic.getVerticalFocalLength(), 3.0, 0.0);
        assertEquals(intrinsic.getSkewness(), 6.0, 0.0);    
        
        //set again
        estimator.setIntrinsic(new InhomogeneousPoint2D(6.0, 5.0), 4.0, 3.0, 
                2.0);
        
        //check correctness
        assertEquals(estimator.getDistortionCenter().getInhomX(), 6.0, 0.0);
        assertEquals(estimator.getDistortionCenter().getInhomY(), 5.0, 0.0);
        assertEquals(estimator.getHorizontalFocalLength(), 4.0, 0.0);
        assertEquals(estimator.getVerticalFocalLength(), 3.0, 0.0);
        assertEquals(estimator.getSkew(), 2.0, 0.0);            
    }
    
    @Test
    public void testGetSetNumKParams() throws LockedException {
        RANSACRadialDistortionRobustEstimator estimator =
                new RANSACRadialDistortionRobustEstimator();

        //check default value
        assertEquals(estimator.getNumKParams(), 
                RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS);
        
        //set new value
        estimator.setNumKParams(3);
        
        //check correctness
        assertEquals(estimator.getNumKParams(), 3);
    }
    
    @Test
    public void testEstimate() throws NotSupportedException, LockedException, 
            NotReadyException, RobustEstimatorException, DistortionException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        for (int j = 0; j < TIMES; j++) {
            double k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
            double k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        
            Point2D center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
        
            RadialDistortion distortion = new RadialDistortion(k1, k2, center);
        
            int nPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
       
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Point2D> distortedPointsWithError = new ArrayList<>();
            List<Point2D> distortedPoints = new ArrayList<>();
            List<Point2D> undistortedPoints = new ArrayList<>();
            Point2D distortedPoint, distortedPointWithError, undistortedPoint;
            for (int i = 0; i < nPoints; i++) {
                undistortedPoint = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                        randomizer.nextDouble(MIN_POINT_VALUE, 
                        MAX_POINT_VALUE));
            
                distortedPoint = distortion.distort(undistortedPoint);
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    //point is outlier
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    distortedPointWithError = new InhomogeneousPoint2D(
                            distortedPoint.getInhomX() + errorX, 
                            distortedPoint.getInhomY() + errorY);
                } else {
                    //point is inlier
                    distortedPointWithError = distortedPoint;
                }
                distortedPoints.add(distortedPoint);
                distortedPointsWithError.add(distortedPointWithError);
                undistortedPoints.add(undistortedPoint);
            }
            
            RANSACRadialDistortionRobustEstimator estimator =
                    new RANSACRadialDistortionRobustEstimator(distortedPointsWithError, 
                            undistortedPoints, this);
            
            estimator.setIntrinsic(distortion.getIntrinsic());
            estimator.setThreshold(THRESHOLD);
            
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());
            
            RadialDistortion distortion2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            //check correctness of estimation
            assertEquals(distortion2.getK1(), k1, ABSOLUTE_ERROR);
            assertEquals(distortion2.getK2(), k2, ABSOLUTE_ERROR);
            assertEquals(distortion2.getCenter(), center);
            
            for(int i = 0; i < nPoints; i++){
                assertEquals(distortedPoints.get(i).distanceTo(
                        distortion2.distort(undistortedPoints.get(i))), 0.0, 
                        ABSOLUTE_ERROR);
            }
        }
    }

    @Override
    public void onEstimateStart(RadialDistortionRobustEstimator estimator) {
        estimateStart++;
        checkLocked((RANSACRadialDistortionRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(RadialDistortionRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((RANSACRadialDistortionRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(
            RadialDistortionRobustEstimator estimator, int iteration) {
        estimateNextIteration++;
        checkLocked((RANSACRadialDistortionRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(
            RadialDistortionRobustEstimator estimator, float progress) {
        estimateProgressChange++;
        checkLocked((RANSACRadialDistortionRobustEstimator)estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }
    
    private void checkLocked(RANSACRadialDistortionRobustEstimator estimator) {
        try {
            estimator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setDistortionCenter(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setMaxIterations(10);            
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setPoints(null, null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch(Exception e) {
            fail("LockedException expected but not thrown");
        }
        assertTrue(estimator.isLocked());
    }
}
