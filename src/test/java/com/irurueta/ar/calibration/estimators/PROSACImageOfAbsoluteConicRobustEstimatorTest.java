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

import com.irurueta.ar.calibration.ImageOfAbsoluteConic;
import com.irurueta.ar.calibration.Pattern2D;
import com.irurueta.ar.calibration.Pattern2DType;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class PROSACImageOfAbsoluteConicRobustEstimatorTest implements
        ImageOfAbsoluteConicRobustEstimatorListener {
    
    private static final double MIN_FOCAL_LENGTH = 3.0;
    private static final double MAX_FOCAL_LENGTH = 10.0;

    private static final double MIN_ANGLE_DEGREES = -10.0;
    private static final double MAX_ANGLE_DEGREES = 10.0;
    
    private static final int INHOM_3D_COORDS = 3;
    
    private static final double MIN_RANDOM_VALUE = -50.0;
    private static final double MAX_RANDOM_VALUE = -10.0;
    
    private static final double ABSOLUTE_ERROR = 5e-6;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 5e-1;
    
    private static final int MIN_NUM_HOMOGRAPHIES = 100;
    private static final int MAX_NUM_HOMOGRAPHIES = 500;
    
    private static final double THRESHOLD = 1e-9;
    private static final double LARGE_THRESHOLD = 1e-8;
    
    public static final int TIMES = 50;
    
    public static final int PERCENTAGE_OUTLIERS = 20;
    
    public static final double STD_ERROR = 1.0;
    
    private int estimateStart;
    private int estimateEnd;
    private int estimateNextIteration;
    private int estimateProgressChange;
    
    //point to be reused when computing homographies residuals
    private Point2D mTestPoint = Point2D.create(
            CoordinatesType.HOMOGENEOUS_COORDINATES);
    
    public PROSACImageOfAbsoluteConicRobustEstimatorTest() { }
    
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
        PROSACImageOfAbsoluteConicRobustEstimator estimator =
                new PROSACImageOfAbsoluteConicRobustEstimator();
        
        //check default values
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getHomographies());
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getThreshold(), 
                PROSACImageOfAbsoluteConicRobustEstimator.DEFAULT_THRESHOLD, 
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        
        //test constructor with listener
        estimator = new PROSACImageOfAbsoluteConicRobustEstimator(this);
        
        //check default values
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getHomographies());
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getThreshold(), 
                PROSACImageOfAbsoluteConicRobustEstimator.DEFAULT_THRESHOLD, 
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);        
        
        //test constructor with homographies
        List<Transformation2D> homographies = new ArrayList<>();
        homographies.add(new ProjectiveTransformation2D());
        
        estimator = new PROSACImageOfAbsoluteConicRobustEstimator(homographies);
        
        //check default values
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getThreshold(), 
                RANSACImageOfAbsoluteConicRobustEstimator.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        
        //Force IllegalArgumentException
        List<Transformation2D> emptyHomographies = 
                new ArrayList<>();
        estimator = null;
        try {
            estimator = new PROSACImageOfAbsoluteConicRobustEstimator(
                    (List<Transformation2D>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROSACImageOfAbsoluteConicRobustEstimator(
                    emptyHomographies);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //test constructor with homographies and listener
        estimator = new PROSACImageOfAbsoluteConicRobustEstimator(homographies, 
                this);
        
        //check default values
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getThreshold(), 
                RANSACImageOfAbsoluteConicRobustEstimator.DEFAULT_THRESHOLD, 
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACImageOfAbsoluteConicRobustEstimator(
                    (List<Transformation2D>)null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROSACImageOfAbsoluteConicRobustEstimator(
                    emptyHomographies);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore ) { }
        assertNull(estimator);        
        
        //test constructor with quality scores
        double[] qualityScores = new double[1];
        estimator = new PROSACImageOfAbsoluteConicRobustEstimator(
                qualityScores);
        
        //check default values
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getHomographies());
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getThreshold(), 
                PROSACImageOfAbsoluteConicRobustEstimator.DEFAULT_THRESHOLD, 
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        
        //force IllegalArgumentException
        double[] emptyScores = new double[0];
        estimator = null;
        try {
            estimator = new PROSACImageOfAbsoluteConicRobustEstimator(
                    emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //test constructor with quality scores and listener
        estimator = new PROSACImageOfAbsoluteConicRobustEstimator(
                qualityScores, this);
        
        //check default values
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getHomographies());
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getThreshold(), 
                PROSACImageOfAbsoluteConicRobustEstimator.DEFAULT_THRESHOLD, 
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);        
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACImageOfAbsoluteConicRobustEstimator(
                    emptyScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //test constructor with homographies and quality scores
        estimator = new PROSACImageOfAbsoluteConicRobustEstimator(homographies,
                qualityScores);
        
        //check default values
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getThreshold(), 
                RANSACImageOfAbsoluteConicRobustEstimator.DEFAULT_THRESHOLD, 
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACImageOfAbsoluteConicRobustEstimator(
                    null, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROSACImageOfAbsoluteConicRobustEstimator(
                    emptyHomographies, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROSACImageOfAbsoluteConicRobustEstimator(
                    homographies, emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //test constructor with homographies, quality scores and listener
        estimator = new PROSACImageOfAbsoluteConicRobustEstimator(homographies, 
                qualityScores, this);
        
        //check default values
        assertEquals(estimator.isZeroSkewness(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getThreshold(), 
                RANSACImageOfAbsoluteConicRobustEstimator.DEFAULT_THRESHOLD, 
                0.0);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACImageOfAbsoluteConicRobustEstimator(
                    null, qualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROSACImageOfAbsoluteConicRobustEstimator(
                    emptyHomographies, qualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new PROSACImageOfAbsoluteConicRobustEstimator(
                    homographies, emptyScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);                
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException {
        PROSACImageOfAbsoluteConicRobustEstimator estimator =
                new PROSACImageOfAbsoluteConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.getThreshold(),
                PROSACImageOfAbsoluteConicRobustEstimator.DEFAULT_THRESHOLD, 
                0.0);
        
        //set new value
        estimator.setThreshold(1.0);
        
        //check correctness
        assertEquals(estimator.getThreshold(), 1.0, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testIsSetZeroSkewness() throws LockedException {
        PROSACImageOfAbsoluteConicRobustEstimator estimator =
                new PROSACImageOfAbsoluteConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        
        //set new value
        estimator.setZeroSkewness(
                !ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                !ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
    }
    
    @Test
    public void testIsSetPrincipalPointAtOrigin() throws LockedException {
        PROSACImageOfAbsoluteConicRobustEstimator estimator =
                new PROSACImageOfAbsoluteConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        
        //set new value
        estimator.setPrincipalPointAtOrigin(!ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        
        //check correctness
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                !ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
    }
    
    @Test
    public void testIsSetFocalDistanceAspectRatioKnown() throws LockedException {
        PROSACImageOfAbsoluteConicRobustEstimator estimator =
                new PROSACImageOfAbsoluteConicRobustEstimator();

        //check default value
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        
        //set new value
        estimator.setFocalDistanceAspectRatioKnown(
                !ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        
        //check correctness
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                !ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
    }
    
    @Test
    public void testGetSetFocalDistanceAspectRatio() throws LockedException {
        PROSACImageOfAbsoluteConicRobustEstimator estimator =
                new PROSACImageOfAbsoluteConicRobustEstimator();

        //check default value
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);
        
        //set new value
        estimator.setFocalDistanceAspectRatio(0.5);
        
        //check correctness
        assertEquals(estimator.getFocalDistanceAspectRatio(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setFocalDistanceAspectRatio(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetListener() throws LockedException {
        PROSACImageOfAbsoluteConicRobustEstimator estimator =
                new PROSACImageOfAbsoluteConicRobustEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        
        //set new value
        estimator.setListener(this);
        
        //check correctness
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
    }
    
    @Test
    public void testGetSetProgressDelta() throws LockedException {
        PROSACImageOfAbsoluteConicRobustEstimator estimator =
                new PROSACImageOfAbsoluteConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.getProgressDelta(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        
        //set new value
        estimator.setProgressDelta(0.5f);
        
        //check correctness
        assertEquals(estimator.getProgressDelta(), 0.5, 0.0);
        
        //force IllegalArgumentException
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
        PROSACImageOfAbsoluteConicRobustEstimator estimator =
                new PROSACImageOfAbsoluteConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.getConfidence(),
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
        //set new value
        estimator.setConfidence(0.5);
        
        //check correctness
        assertEquals(estimator.getConfidence(), 0.5, 0.0);
        
        //force IllegalArgumentException
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
        PROSACImageOfAbsoluteConicRobustEstimator estimator =
                new PROSACImageOfAbsoluteConicRobustEstimator();
        
        //check default value
        assertEquals(estimator.getMaxIterations(), 
                ImageOfAbsoluteConicRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
        //set new value
        estimator.setMaxIterations(100);
        
        //check correctness
        assertEquals(estimator.getMaxIterations(), 100);
        
        //force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetHomographiesAndIsReady() throws LockedException {
        PROSACImageOfAbsoluteConicRobustEstimator estimator =
                new PROSACImageOfAbsoluteConicRobustEstimator();
        
        //check default value
        assertNull(estimator.getHomographies());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numHomographies = randomizer.nextInt(MIN_NUM_HOMOGRAPHIES, 
                MAX_NUM_HOMOGRAPHIES);
        double[] qualityScores = new double[numHomographies];
        List<Transformation2D> homographies = new ArrayList<>();
        for (int i = 0; i < numHomographies; i++) {
            homographies.add(new ProjectiveTransformation2D());
        }
        
        estimator.setHomographies(homographies);
        
        //check correctness
        assertSame(estimator.getHomographies(), homographies);
        assertFalse(estimator.isReady());
        
        //if we also set quality scores, then estimator becomes ready
        estimator.setQualityScores(qualityScores);
        
        assertTrue(estimator.isReady());
        
        //Force IllegalArgumentException
        List<Transformation2D> emptyHomographies = 
                new ArrayList<>();
        try {
            estimator.setHomographies(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setHomographies(emptyHomographies);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    @SuppressWarnings("Duplicates")
    public void testGetSetQualityScores() throws LockedException {
        PROSACImageOfAbsoluteConicRobustEstimator estimator =
                new PROSACImageOfAbsoluteConicRobustEstimator();
        
        //check default value
        assertNull(estimator.getQualityScores());
        
        //set new value
        double[] qualityScores = new double[1];
        estimator.setQualityScores(qualityScores);
        
        //check correctness
        assertSame(estimator.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        double[] emptyScores = new double[0];
        try {
            estimator.setQualityScores(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    @SuppressWarnings("all")
    public void testEstimateCirclesPattern() throws 
            InvalidPinholeCameraIntrinsicParametersException, LockedException, 
            NotReadyException, RobustEstimatorException {

        boolean succeededAtLeastOnce = false;
        int failed = 0, succeeded = 0, total = 0;        
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;      
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError, verticalFocalDistanceError,
                skewnessError, horizontalPrincipalPointError,
                verticalPrincipalPointError;                
        for (int j = 0; j < TIMES; j++) {
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                    verticalPrincipalPoint, skewness);
            
            ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);
            
            //create pattern to estimate homography
            Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            List<Point2D> patternPoints = pattern.getIdealPoints();
            
            //assume that pattern points are located on a 3D plane
            //(for instance Z = 0), but can be really any plane
            List<Point3D> points3D = new ArrayList<>();
            for (Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                    patternPoint.getInhomY(), 0.0, 1.0));
            }
            
            //create homographies.
            //Create one random camera for each homography having a random
            //rotation and translation but all having the same intrinsic
            //parameters
            int nHomographies = randomizer.nextInt(MIN_NUM_HOMOGRAPHIES, 
                    MAX_NUM_HOMOGRAPHIES);
            List<Transformation2D> homographies =
                    new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            double[] qualityScores = new double[nHomographies];
            for (int i = 0; i < nHomographies; i++) {
                //rotation
                double alphaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0,
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double betaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double gammaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                
                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                        betaEuler, gammaEuler);
                
                //camera center
                double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);
                
                //create camera with intrinsic parameters, rotation and camera
                //center
                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);
                camera.normalize();
                
                //project 3D pattern points
                List<Point2D> projectedPatternPoints = camera.project(points3D);
                
                //add random noise to projected points with a certain outlier
                //proportion
                List<Point2D> projectedPatternPointsWithError = 
                        new ArrayList<>();
                for (Point2D projectedPatternPoint : projectedPatternPoints) {
                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        //outlier sample
                        double errorX = errorRandomizer.nextDouble();
                        double errorY = errorRandomizer.nextDouble();
                        projectedPatternPointsWithError.add(
                                new HomogeneousPoint2D(
                                projectedPatternPoint.getInhomX() + errorX,
                                projectedPatternPoint.getInhomY() + errorY, 
                                1.0));
                    } else {
                        //inlier
                        projectedPatternPointsWithError.add(
                                projectedPatternPoint);
                    }
                }
                
                //estimate homography. Because a robust estimator is used,
                //probably the effect of outliers will already be removed by
                //the time the homography is obtained
                RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator
                        homographyEstimator = 
                        new RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator(
                        patternPoints, projectedPatternPointsWithError);
                //enforce a strict threshold to estimate homographies
                homographyEstimator.setThreshold(THRESHOLD);
                
                ProjectiveTransformation2D homography =
                        homographyEstimator.estimate();
                homographies.add(homography);
                
                //in real situations projectedPatternPoints is unknown, and 
                //hence quality scores must be computed in a different way
                //(for instance using projectedPatternPointsWithError, or 
                //distance from center of image, where supposedly center of 
                //radial distorsion is expected to be, and hence the farther
                //measures are from the center more distorted will be) or 
                //quality of the features used to detect calibration markers
                double avgHomographyResidual = avgHomographyResidual(homography, 
                        patternPoints, projectedPatternPoints);
                qualityScores[i] = 1.0 / (1.0 + avgHomographyResidual);
            }
            
            //Estimate IAC
            PROSACImageOfAbsoluteConicRobustEstimator estimator =
                    new PROSACImageOfAbsoluteConicRobustEstimator(homographies,
                    qualityScores, this);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setThreshold(THRESHOLD);
            
            //check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            
            //estimate
            ImageOfAbsoluteConic iac2 = estimator.estimate();
            
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check that estimated iac corresponds to the initial one (up to 
            //scale)
            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getB(), 0.0, 0.0);
            assertEquals(iac2.getB(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), 
                    VERY_LARGE_ABSOLUTE_ERROR);            
            assertEquals(iac.getD(), 0.0, 0.0);
            assertEquals(iac2.getD(), 0.0, 0.0);
            assertEquals(iac.getE(), 0.0, 0.0);
            assertEquals(iac2.getE(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            
            try {
                //check intrinsic parameters
                PinholeCameraIntrinsicParameters intrinsic2 = 
                        iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(), 
                        intrinsic2.getHorizontalFocalLength(), 
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(), 
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0, 
                        0.0);
                assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);            

                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                        intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                        intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                        intrinsic2.getVerticalPrincipalPoint());

                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;

                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }
                succeededAtLeastOnce = true;
                succeeded++;
            } catch (InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }
        
        double failedRatio = (double)failed / (double)total;
        double succeededRatio = (double)succeeded / (double)total;
        
        avgHorizontalFocalDistanceError /= (double)succeeded;
        avgVerticalFocalDistanceError /= (double)succeeded;
        avgSkewnessError /= (double)succeeded;
        avgHorizontalPrincipalPointError /= (double)succeeded;
        avgVerticalPrincipalPointError /= (double)succeeded;
        
        //check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        
        String msg = "Circles pattern - failed: " + failedRatio * 100.0 +
                "% succeeded: " + succeededRatio * 100.0 +
                "% avg horizontal focal distance error: " + 
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError +
                " avg skewness error: " + avgSkewnessError +
                " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +
                avgVerticalPrincipalPointError +
                " avg min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;
        Logger.getLogger(PROSACImageOfAbsoluteConicRobustEstimatorTest.class.getName()).
                log(Level.INFO, msg);
        
        assertTrue(failedRatio < 0.6);
        assertTrue(succeededRatio >= 0.4);
        assertTrue(succeededAtLeastOnce);        
    }

    @Test
    @SuppressWarnings("all")
    public void testEstimateQRPattern() throws 
            InvalidPinholeCameraIntrinsicParametersException, LockedException, 
            NotReadyException, RobustEstimatorException {

        boolean succeededAtLeastOnce = false;
        int failed = 0, succeeded = 0, total = 0;        
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;      
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError, verticalFocalDistanceError,
                skewnessError, horizontalPrincipalPointError,
                verticalPrincipalPointError;                
        for (int j = 0; j < TIMES; j++) {
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                    verticalPrincipalPoint, skewness);
            
            ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);
            
            //create pattern to estimate homography
            Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
            List<Point2D> patternPoints = pattern.getIdealPoints();
            
            //assume that pattern points are located on a 3D plane
            //(for instance Z = 0), but can be really any plane
            List<Point3D> points3D = new ArrayList<>();
            for (Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                    patternPoint.getInhomY(), 0.0, 1.0));
            }
            
            //create homographies.
            //Create one random camera for each homography having a random
            //rotation and translation but all having the same intrinsic
            //parameters
            int nHomographies = randomizer.nextInt(MIN_NUM_HOMOGRAPHIES, 
                    MAX_NUM_HOMOGRAPHIES);
            List<Transformation2D> homographies =
                    new ArrayList<>();
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            double[] tmpQualityScores = new double[nHomographies];
            int validHomographiesCounter = 0;
            for (int i = 0; i < nHomographies; i++) {
                //rotation
                double alphaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0,
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double betaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double gammaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                
                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                        betaEuler, gammaEuler);
                
                //camera center
                double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);
                
                //create camera with intrinsic parameters, rotation and camera
                //center
                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);
                camera.normalize();
                
                //project 3D pattern points
                List<Point2D> projectedPatternPoints = camera.project(points3D);
                
                //add random noise to projected points with a certain outlier
                //proportion
                List<Point2D> projectedPatternPointsWithError = 
                        new ArrayList<>();
                //we use average error of samples to determine quality scores
                //of homographies, this could be equivalent to the measure of
                //of the quality of the features used to detect calibration
                //markers
                double avgError = 0.0;
                for (Point2D projectedPatternPoint : projectedPatternPoints) {
                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        //outlier sample
                        double errorX = errorRandomizer.nextDouble();
                        double errorY = errorRandomizer.nextDouble();
                        
                        projectedPatternPointsWithError.add(
                                new HomogeneousPoint2D(
                                projectedPatternPoint.getInhomX() + errorX,
                                projectedPatternPoint.getInhomY() + errorY, 
                                1.0));
                        
                        avgError += Math.sqrt(errorX * errorX + 
                                errorY * errorY);
                    } else {
                        //inlier
                        projectedPatternPointsWithError.add(
                                projectedPatternPoint);
                    }
                }
                
                avgError /= (double)projectedPatternPointsWithError.size();
                
                //estimate affine homography using 3 points of qr pattern.
                //Affine homography is not robustly estimated but effect of
                //errors will be removed during robust IAC estimation
                try {
                    ProjectiveTransformation2D homography = 
                        new ProjectiveTransformation2D(patternPoints.get(0), 
                        patternPoints.get(1), patternPoints.get(2), 
                        patternPoints.get(3),
                        projectedPatternPointsWithError.get(0),
                        projectedPatternPointsWithError.get(1),
                        projectedPatternPointsWithError.get(2),
                        projectedPatternPointsWithError.get(3));
                    homographies.add(homography);
                } catch (CoincidentPointsException ignore) { }
                
                tmpQualityScores[validHomographiesCounter] = 
                        1.0 / (1.0 + avgError);
                validHomographiesCounter++;
            }
            
            double[] qualityScores = Arrays.copyOf(tmpQualityScores, 
                    validHomographiesCounter);
            
            //Estimate IAC
            PROSACImageOfAbsoluteConicRobustEstimator estimator =
                    new PROSACImageOfAbsoluteConicRobustEstimator(homographies, 
                    qualityScores, this);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setThreshold(THRESHOLD);
            
            if (!estimator.isReady()) {
                continue;
            }
            
            //check initial state
            assertFalse(estimator.isLocked());            
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimateNextIteration, 0);
            assertEquals(estimateProgressChange, 0);
            
            //estimate
            ImageOfAbsoluteConic iac2 = estimator.estimate();
            
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();
            
            //check that estimated iac corresponds to the initial one (up to 
            //scale)
            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getB(), 0.0, 0.0);
            assertEquals(iac2.getB(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), 
                    VERY_LARGE_ABSOLUTE_ERROR);            
            assertEquals(iac.getD(), 0.0, 0.0);
            assertEquals(iac2.getD(), 0.0, 0.0);
            assertEquals(iac.getE(), 0.0, 0.0);
            assertEquals(iac2.getE(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            
            try {
                //check intrinsic parameters
                PinholeCameraIntrinsicParameters intrinsic2 = 
                        iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(), 
                        intrinsic2.getHorizontalFocalLength(), 
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(), 
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0, 
                        0.0);
                assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);            

                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                        intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                        intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                        intrinsic2.getVerticalPrincipalPoint());

                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;

                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }

                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }
                succeededAtLeastOnce = true;
                succeeded++;
            } catch (InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }
        
        double failedRatio = (double)failed / (double)total;
        double succeededRatio = (double)succeeded / (double)total;
        
        avgHorizontalFocalDistanceError /= (double)total;
        avgVerticalFocalDistanceError /= (double)total;
        avgSkewnessError /= (double)total;
        avgHorizontalPrincipalPointError /= (double)total;
        avgVerticalPrincipalPointError /= (double)total;
        
        //check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        
        String msg = "QR pattern - failed: " + failedRatio * 100.0 +
                "% succeeded: " + succeededRatio * 100.0 +
                "% avg horizontal focal distance error: " + 
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError +
                " avg skewness error: " + avgSkewnessError +
                " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError +
                " avg vertical principal point error: " +
                avgVerticalPrincipalPointError +
                " avg min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;
        Logger.getLogger(PROSACImageOfAbsoluteConicRobustEstimatorTest.class.getName()).
                log(Level.INFO, msg);
        
        assertTrue(failedRatio < 0.25);
        assertTrue(succeededRatio >= 0.75);
        assertTrue(succeededAtLeastOnce);        
    }
    
    @Test
    public void testEstimateRealData() throws CoincidentPointsException, 
            LockedException, NotReadyException, RobustEstimatorException, 
            InvalidPinholeCameraIntrinsicParametersException {
        //For a QR pattern, assuming zero skewness, equal focal lengths and
        //principal point at origin on a nexus 5 device
        Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
        List<Point2D> patternPoints = pattern.getIdealPoints();
        
        /*
        Sampled data (before and after centering coordinates and setting correct y axis 
        direction)
        Point[0] = 774.5, 1084.5 | 6.5, -60.5
        Point[1] = 791.5, 840.0 | 23.5, 184.0
        Point[2] = 1037.5, 854.0 | 269.5, 170.0
        Point[3] = 999.0, 1074.0 | 231.0, -50.0        
        */        
        List<Point2D> sampledPoints1 = new ArrayList<>();
        sampledPoints1.add(new InhomogeneousPoint2D(6.5, -60.5));
        sampledPoints1.add(new InhomogeneousPoint2D(23.5, 184.0));
        sampledPoints1.add(new InhomogeneousPoint2D(269.5, 170.0));
        sampledPoints1.add(new InhomogeneousPoint2D(231.0, -50.0));
        
        /*
        Sampled data (before and after centering coordinates and setting correct y axis 
        direction)
        Point[0] = 382.5, 701.5 | -385.5, 322.5
        Point[1] = 351.0, 473.5 | -417.0, 550.5
        Point[2] = 585.0, 451.5 | -183.0, 572.5
        Point[3] = 592.0, 653.5 | -176.0, 370.5
        */
        List<Point2D> sampledPoints2 = new ArrayList<>();
        sampledPoints2.add(new InhomogeneousPoint2D(-385.5, 322.5));
        sampledPoints2.add(new InhomogeneousPoint2D(-417.0, 550.5));
        sampledPoints2.add(new InhomogeneousPoint2D(-183.0, 572.5));
        sampledPoints2.add(new InhomogeneousPoint2D(-176.0, 370.5));

        /*
        Sampled data (before and after centering coordinates and setting correct y axis 
        direction)
        Point[0] = 988.0, 486.5 | 220.0, 537.5
        Point[1] = 1028.5, 278.5 | 260.5, 745.5
        Point[2] = 1241.0, 316.5 | 473.0, 707.5
        Point[3] = 1185.5, 498.5 | 417.5, 525.5
        */
        List<Point2D> sampledPoints3 = new ArrayList<>();
        sampledPoints3.add(new InhomogeneousPoint2D(220.0, 537.5));
        sampledPoints3.add(new InhomogeneousPoint2D(260.5, 745.5));
        sampledPoints3.add(new InhomogeneousPoint2D(473.0, 707.5));
        sampledPoints3.add(new InhomogeneousPoint2D(417.5, 525.5));
        
        /*
        Sampled data (before and after centering coordinates and setting correct y axis 
        direction)
        Point[0] = 576.0, 1404.4166 | -192.0, -380.4166259765625
        Point[1] = 544.5, 1151.5 | -223.5, -127.5
        Point[2] = 792.0, 1117.5 | 24.0, -93.5
        Point[3] = 798.5, 1347.0 | 30.5, -323.0
        */
        List<Point2D> sampledPoints4 = new ArrayList<>();
        sampledPoints4.add(new InhomogeneousPoint2D(-192.0, -380.4166259765625));
        sampledPoints4.add(new InhomogeneousPoint2D(-223.5, -127.5));
        sampledPoints4.add(new InhomogeneousPoint2D(24.0, -93.5));
        sampledPoints4.add(new InhomogeneousPoint2D(30.5, -323.0));

        /*
        Sampled data (before and after centering coordinates and setting correct y axis 
        direction)
        Point[0] = 913.5, 1596.0 | 145.5, -572.0
        Point[1] = 939.5, 1360.7 | 171.5, -336.699951171875
        Point[2] = 1170.5, 1391.0 | 402.5, -367.0
        Point[3] = 1126.5, 1600.5 | 358.5, -576.5 
        */   
        List<Point2D> sampledPoints5 = new ArrayList<>();
        sampledPoints5.add(new InhomogeneousPoint2D(145.5, -572.0));
        sampledPoints5.add(new InhomogeneousPoint2D(171.5, -336.699951171875));
        sampledPoints5.add(new InhomogeneousPoint2D(402.5, -367.0));
        sampledPoints5.add(new InhomogeneousPoint2D(358.5, -576.5));
        
        //obtain homographies
        ProjectiveTransformation2D homography1 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1), 
                patternPoints.get(2), patternPoints.get(3), 
                sampledPoints1.get(0), sampledPoints1.get(1), 
                sampledPoints1.get(2), sampledPoints1.get(3));

        ProjectiveTransformation2D homography2 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1), 
                patternPoints.get(2), patternPoints.get(3), 
                sampledPoints2.get(0), sampledPoints2.get(1), 
                sampledPoints2.get(2), sampledPoints2.get(3));

        ProjectiveTransformation2D homography3 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1), 
                patternPoints.get(2), patternPoints.get(3), 
                sampledPoints3.get(0), sampledPoints3.get(1), 
                sampledPoints3.get(2), sampledPoints3.get(3));
        
        ProjectiveTransformation2D homography4 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1), 
                patternPoints.get(2), patternPoints.get(3), 
                sampledPoints4.get(0), sampledPoints4.get(1), 
                sampledPoints4.get(2), sampledPoints4.get(3));
        
        ProjectiveTransformation2D homography5 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1), 
                patternPoints.get(2), patternPoints.get(3), 
                sampledPoints5.get(0), sampledPoints5.get(1), 
                sampledPoints5.get(2), sampledPoints5.get(3));
        
        
        List<Transformation2D> homographies = new ArrayList<>();
        homographies.add(homography1);
        homographies.add(homography2);
        homographies.add(homography3);
        homographies.add(homography4);
        homographies.add(homography5);
        
        double[] qualityScores = new double[]{ 0.001, 1.0, 0.1, 0.003, 0.002};
        
        //estimate IAC
        PROSACImageOfAbsoluteConicRobustEstimator estimator =
                new PROSACImageOfAbsoluteConicRobustEstimator(homographies,
                qualityScores, this);
        estimator.setZeroSkewness(true);
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setFocalDistanceAspectRatioKnown(true);
        estimator.setFocalDistanceAspectRatio(1.0);
        estimator.setThreshold(LARGE_THRESHOLD);
        
        //check initial state
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimateStart, 0);
        assertEquals(estimateEnd, 0);
        assertEquals(estimateNextIteration, 0);
        assertEquals(estimateProgressChange, 0);

        //estimate
        ImageOfAbsoluteConic iac = estimator.estimate();

        assertFalse(estimator.isLocked());
        assertEquals(estimateStart, 1);
        assertEquals(estimateEnd, 1);
        assertTrue(estimateNextIteration > 0);
        assertTrue(estimateProgressChange >= 0);
        reset();
        
        //obtain intrinsic parameters
        PinholeCameraIntrinsicParameters intrinsic = 
                iac.getIntrinsicParameters();
        
        assertNotNull(intrinsic);
        
        double horizontalFocalLength = intrinsic.getHorizontalFocalLength();
        double verticalFocalLength = intrinsic.getVerticalFocalLength();
        double skewness = intrinsic.getSkewness();
        double horizontalPrincipalPoint = intrinsic.getHorizontalPrincipalPoint();
        double verticalPrincipalPoint = intrinsic.getVerticalPrincipalPoint();
        assertTrue(horizontalFocalLength > 0);
        assertTrue(verticalFocalLength > 0);
        assertEquals(horizontalFocalLength, verticalFocalLength, 
                ABSOLUTE_ERROR);
        assertEquals(skewness, 0.0, 0.0);
        assertEquals(horizontalPrincipalPoint, 0.0, 0.0);
        assertEquals(verticalPrincipalPoint, 0.0, 0.0);        
           
        String msg = "Real data focal length: " + horizontalFocalLength;
        Logger.getLogger(PROSACImageOfAbsoluteConicRobustEstimatorTest.class.getName()).
                log(Level.INFO, msg);
    }    
    
    private double homographyResidual(
            ProjectiveTransformation2D homography, Point2D inputPoint, 
            Point2D outputPoint){
        homography.transform(inputPoint, mTestPoint);
        return outputPoint.distanceTo(mTestPoint);
    }
    
    private double avgHomographyResidual(
            ProjectiveTransformation2D homography, List<Point2D> inputPoints,
            List<Point2D> outputPoints){
        int size = inputPoints.size();
        double avg = 0.0;
        for (int i = 0; i < size; i++) {
            avg += homographyResidual(homography, inputPoints.get(i),
                    outputPoints.get(i));
        }
        
        return avg / (double)size;
    }

    @Override
    public void onEstimateStart(ImageOfAbsoluteConicRobustEstimator estimator) {
        estimateStart++;
        testLocked((PROSACImageOfAbsoluteConicRobustEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(ImageOfAbsoluteConicRobustEstimator estimator) {
        estimateEnd++;
        testLocked((PROSACImageOfAbsoluteConicRobustEstimator)estimator);
    }

    @Override
    public void onEstimateNextIteration(
            ImageOfAbsoluteConicRobustEstimator estimator, int iteration) {
        estimateNextIteration++;
        testLocked((PROSACImageOfAbsoluteConicRobustEstimator)estimator);
    }

    @Override
    public void onEstimateProgressChange(
            ImageOfAbsoluteConicRobustEstimator estimator, float progress) {
        estimateProgressChange++;
        testLocked((PROSACImageOfAbsoluteConicRobustEstimator)estimator);
    }

    private void reset(){
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void testLocked(
            PROSACImageOfAbsoluteConicRobustEstimator estimator) {
        try {
            estimator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setHomographies(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setMaxIterations(100);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setPrincipalPointAtOrigin(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setThreshold(1e-6);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setZeroSkewness(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setQualityScores(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
        assertTrue(estimator.isLocked());
    }
}
