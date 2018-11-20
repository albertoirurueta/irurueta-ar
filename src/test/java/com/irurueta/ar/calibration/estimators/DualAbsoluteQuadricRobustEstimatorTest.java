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
package com.irurueta.ar.calibration.estimators;

import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class DualAbsoluteQuadricRobustEstimatorTest implements
        DualAbsoluteQuadricRobustEstimatorListener {
    
    public DualAbsoluteQuadricRobustEstimatorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testCreateWithMethod() {
        //create with LMedS method
        DualAbsoluteQuadricRobustEstimator estimator =
                DualAbsoluteQuadricRobustEstimator.create(
                        RobustEstimatorMethod.LMedS);
        
        //check correctness
        assertTrue(estimator instanceof 
                LMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        
        //create with MSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(
                RobustEstimatorMethod.MSAC);
        
        //check correctness
        assertTrue(estimator instanceof
                MSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        
        //create with PROSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(
                RobustEstimatorMethod.PROSAC);
        
        //check correctness
        assertTrue(estimator instanceof
                PROSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);        
        
        //create with PROMedS method
        estimator = DualAbsoluteQuadricRobustEstimator.create(
                RobustEstimatorMethod.PROMedS);
        
        //check correctness
        assertTrue(estimator instanceof
                PROMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        
        //create with RANSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(
                RobustEstimatorMethod.RANSAC);
        
        //check correctness
        assertTrue(estimator instanceof
                RANSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);        
    }
    
    @Test
    public void testCreateWithCamerasQualityScoresAndMethod() {
        List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        
        double[] qualityScores = new double[cameras.size()];
        
        //create with LMedS method
        DualAbsoluteQuadricRobustEstimator estimator =
                DualAbsoluteQuadricRobustEstimator.create(cameras, 
                        qualityScores, RobustEstimatorMethod.LMedS);
        
        //check correctness
        assertTrue(estimator instanceof 
                LMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), new double[1], 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //create with MSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(cameras, 
                qualityScores, RobustEstimatorMethod.MSAC);
        
        //check correctness
        assertTrue(estimator instanceof
                MSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);        
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), new double[1], 
                    RobustEstimatorMethod.MSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //create with PROSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(cameras, 
                qualityScores, RobustEstimatorMethod.PROSAC);
        
        //check correctness
        assertTrue(estimator instanceof
                PROSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);        
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(cameras, 
                    new double[1], RobustEstimatorMethod.PROSAC);
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), new double[1], 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //create with PROMedS method
        estimator = DualAbsoluteQuadricRobustEstimator.create(cameras,
                qualityScores, RobustEstimatorMethod.PROMedS);
        
        //check correctness
        assertTrue(estimator instanceof
                PROMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(cameras, 
                    new double[1], RobustEstimatorMethod.PROMedS);
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), new double[1], 
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //create with RANSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(cameras,
                qualityScores, RobustEstimatorMethod.RANSAC);
        
        //check correctness
        assertTrue(estimator instanceof
                RANSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);        
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), new double[1], 
                    RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);        
    }
    
    @Test
    public void testCreateWithCamerasAndMethod() {
        List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());        
        
        //create with LMedS method
        DualAbsoluteQuadricRobustEstimator estimator =
                DualAbsoluteQuadricRobustEstimator.create(cameras, 
                        RobustEstimatorMethod.LMedS);
        
        //check correctness
        assertTrue(estimator instanceof 
                LMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), 
                    RobustEstimatorMethod.LMedS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        
        //create with MSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(cameras, 
                RobustEstimatorMethod.MSAC);
        
        //check correctness
        assertTrue(estimator instanceof
                MSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), 
                    RobustEstimatorMethod.MSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);        
        
        //create with PROSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(cameras, 
                RobustEstimatorMethod.PROSAC);
        
        //check correctness
        assertTrue(estimator instanceof
                PROSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);        
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);        
                
        //create with PROMedS method
        estimator = DualAbsoluteQuadricRobustEstimator.create(cameras,
                RobustEstimatorMethod.PROMedS);
        
        //check correctness
        assertTrue(estimator instanceof
                PROMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), 
                    RobustEstimatorMethod.PROMedS);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //create with RANSAC method
        estimator = DualAbsoluteQuadricRobustEstimator.create(cameras,
                RobustEstimatorMethod.RANSAC);
        
        //check correctness
        assertTrue(estimator instanceof
                RANSACDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);        
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), 
                    RobustEstimatorMethod.RANSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);        
    }
    
    @Test
    public void testCreateDefaultMethod() {
        DualAbsoluteQuadricRobustEstimator estimator = 
                DualAbsoluteQuadricRobustEstimator.create();
        
        //check correctness
        assertTrue(estimator instanceof
                LMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);        
    }
    
    @Test
    public void testCreateWithCamerasQualityScoresAndDefaultMethod() {
        List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        
        double[] qualityScores = new double[cameras.size()];

        DualAbsoluteQuadricRobustEstimator estimator = 
                DualAbsoluteQuadricRobustEstimator.create(cameras, 
                qualityScores);
        
        //check correctness
        assertTrue(estimator instanceof
                LMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);        
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(cameras, 
                    new double[1], RobustEstimatorMethod.PROSAC);
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), new double[1], 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);        
    }
    
    @Test
    public void testCreateWithCamerasAndDefaultMethod() {
        List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());        
        
        DualAbsoluteQuadricRobustEstimator estimator = 
                DualAbsoluteQuadricRobustEstimator.create(cameras);
        
        //check correctness
        assertTrue(estimator instanceof
                LMedSDualAbsoluteQuadricRobustEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);        
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = DualAbsoluteQuadricRobustEstimator.create(
                    new ArrayList<PinholeCamera>(), 
                    RobustEstimatorMethod.PROSAC);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);        
    }
    
    @Test
    public void testIsSetZeroSkewness() throws LockedException {
        DualAbsoluteQuadricRobustEstimator estimator = 
                DualAbsoluteQuadricRobustEstimator.create();
        
        //check default value
        assertTrue(estimator.isZeroSkewness());
        
        //set new value
        estimator.setZeroSkewness(false);
        
        //check correctness
        assertFalse(estimator.isZeroSkewness());
    }
    
    @Test
    public void testIsSetPrincipalPointAtOrigin() throws LockedException {
        DualAbsoluteQuadricRobustEstimator estimator =
                DualAbsoluteQuadricRobustEstimator.create();
        
        //check default value
        assertTrue(estimator.isPrincipalPointAtOrigin());
        
        //set new value
        estimator.setPrincipalPointAtOrigin(false);
        
        //check correctness
        assertFalse(estimator.isPrincipalPointAtOrigin());
    }
    
    @Test
    public void testIsFocalDistanceAspectRatioKnown() throws LockedException {
        DualAbsoluteQuadricRobustEstimator estimator =
                DualAbsoluteQuadricRobustEstimator.create();
        
        //check default value
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        
        //set new value
        estimator.setFocalDistanceAspectRatioKnown(false);
        
        //check correctness
        assertFalse(estimator.isFocalDistanceAspectRatioKnown());
    }
    
    @Test
    public void testGetSetFocalDistanceAspectRatio() throws LockedException {
        DualAbsoluteQuadricRobustEstimator estimator =
                DualAbsoluteQuadricRobustEstimator.create();
        
        //check default value
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        
        //set new value
        estimator.setFocalDistanceAspectRatio(0.5);
        
        //check correctness
        assertEquals(estimator.getFocalDistanceAspectRatio(), 0.5, 0.0);
    }
    
    @Test
    public void testIsSetSingularityEnforced() throws LockedException {
        DualAbsoluteQuadricRobustEstimator estimator =
                DualAbsoluteQuadricRobustEstimator.create();
        
        //check default value
        assertTrue(estimator.isSingularityEnforced());
        
        //set new value
        estimator.setSingularityEnforced(false);
        
        //check correctness
        assertFalse(estimator.isSingularityEnforced());
    }
    
    @Test
    public void testIsSetEnforcedSingularityValidated() throws LockedException {
        DualAbsoluteQuadricRobustEstimator estimator =
                DualAbsoluteQuadricRobustEstimator.create();
        
        //check default value
        assertTrue(estimator.isEnforcedSingularityValidated());
        
        //set new value
        estimator.setEnforcedSingularityValidated(false);
        
        //check correctness
        assertFalse(estimator.isEnforcedSingularityValidated());
    }
    
    @Test
    public void testGetSetDeterminantThreshold() throws LockedException {
        DualAbsoluteQuadricRobustEstimator estimator =
                DualAbsoluteQuadricRobustEstimator.create();
        
        //check default value
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        
        //set new value
        estimator.setDeterminantThreshold(1e-3);
        
        //check correctness
        assertEquals(estimator.getDeterminantThreshold(), 1e-3, 0.0);        
    }
    
    @Test
    public void testGetSetListenerAndIsListenerAvailable() 
            throws LockedException {
        DualAbsoluteQuadricRobustEstimator estimator =
                DualAbsoluteQuadricRobustEstimator.create();
        
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
        DualAbsoluteQuadricRobustEstimator estimator =
                DualAbsoluteQuadricRobustEstimator.create();
        
        //check default value
        assertEquals(estimator.getProgressDelta(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
        //set new value
        estimator.setProgressDelta(0.1f);
        
        //check correctness
        assertEquals(estimator.getProgressDelta(), 0.1f, 0.0);
        
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
        DualAbsoluteQuadricRobustEstimator estimator =
                DualAbsoluteQuadricRobustEstimator.create();
        
        //check default value
        assertEquals(estimator.getConfidence(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
        //set new value
        estimator.setConfidence(0.8);
        
        //check correctness
        assertEquals(estimator.getConfidence(), 0.8, 0.0);
        
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
        DualAbsoluteQuadricRobustEstimator estimator =
                DualAbsoluteQuadricRobustEstimator.create();
        
        //check default value
        assertEquals(estimator.getMaxIterations(), 
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
        //set new value
        estimator.setMaxIterations(100);
        
        //check correctness
        assertEquals(estimator.getMaxIterations(), 100);
        
        //Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetCameras() throws LockedException {
        DualAbsoluteQuadricRobustEstimator estimator =
                DualAbsoluteQuadricRobustEstimator.create();
        
        //initial value
        assertNull(estimator.getCameras());
        
        //set new value
        List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        estimator.setCameras(cameras);
        
        //check correctness
        assertSame(estimator.getCameras(), cameras);
        
        //Force IllegalArgumentException
        try {
            estimator.setCameras(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setCameras(new ArrayList<PinholeCamera>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetMinNumberOfRequiredCameras() throws LockedException {
        DualAbsoluteQuadricRobustEstimator estimator =
                DualAbsoluteQuadricRobustEstimator.create();
        
        //check default value
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        
        //disable principal point at origin
        estimator.setPrincipalPointAtOrigin(false);
        
        //check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), -1);
        
        
        //disable zero skewness
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setZeroSkewness(false);
        
        //check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 4);
        
        
        //disable focal distance aspect ratio known
        estimator.setZeroSkewness(true);
        estimator.setFocalDistanceAspectRatioKnown(false);
        
        //check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 3);
        
        
        //disable zero skewness and singularity enforcement
        estimator.setZeroSkewness(false);
        estimator.setSingularityEnforced(false);
        
        //check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 5);
        
        
        //disable focal distance aspect ratio known and singularity enforcement
        estimator.setZeroSkewness(true);
        
        //check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 3);        
    }
    
    @Test
    public void testAreValidConstraints() throws LockedException {
        DualAbsoluteQuadricRobustEstimator estimator = 
                DualAbsoluteQuadricRobustEstimator.create();
        
        //check default value
        assertTrue(estimator.areValidConstraints());
        
        //disable principal point at origin
        estimator.setPrincipalPointAtOrigin(false);
        
        //check correctness
        assertFalse(estimator.areValidConstraints());
        
        
        //disable zero skewness
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setZeroSkewness(false);
        
        //check correctness
        assertTrue(estimator.areValidConstraints());
        
        
        //disable focal distance aspect ratio known
        estimator.setZeroSkewness(true);
        estimator.setFocalDistanceAspectRatioKnown(false);
        
        //check correctness
        assertFalse(estimator.areValidConstraints());
        
        //disable singularity enforcement
        estimator.setSingularityEnforced(false);
        
        //check correctness
        assertTrue(estimator.areValidConstraints());
    }
    
    @Test
    public void testIsReady() throws LockedException {        
        DualAbsoluteQuadricRobustEstimator estimator = 
                DualAbsoluteQuadricRobustEstimator.create();
        
        //check default value
        assertNull(estimator.getCameras());
        assertTrue(estimator.areValidConstraints());        
        assertFalse(estimator.isReady());
        
        List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        
        estimator.setCameras(cameras);
        estimator.setQualityScores(new double[cameras.size()]);
        
        //check correctness
        assertTrue(estimator.isReady());

        
        //disable principal point at origin
        estimator.setPrincipalPointAtOrigin(false);
        
        //check correctness
        assertFalse(estimator.areValidConstraints());
        assertFalse(estimator.isReady());

        //enable principal point at origin
        estimator.setPrincipalPointAtOrigin(true);
        
        //check correctness
        assertTrue(estimator.isReady());
        
        //clear cameras
        cameras.clear();

        //check correctness
        assertFalse(estimator.isReady());
    }
    

    @Override
    public void onEstimateStart(
            DualAbsoluteQuadricRobustEstimator estimator) { }

    @Override
    public void onEstimateEnd(DualAbsoluteQuadricRobustEstimator estimator) { }

    @Override
    public void onEstimateNextIteration(
            DualAbsoluteQuadricRobustEstimator estimator, int iteration) { }

    @Override
    public void onEstimateProgressChange(
            DualAbsoluteQuadricRobustEstimator estimator, float progress) { }
}
