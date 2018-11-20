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
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RadialDistortionRobustEstimatorTest {
    
    public RadialDistortionRobustEstimatorTest() { }
    
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
        
        //test RANSAC
        RadialDistortionRobustEstimator estimator =
                RadialDistortionRobustEstimator.create(
                RobustEstimatorMethod.RANSAC);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.RANSAC);
        
        //test LMedS
        estimator = RadialDistortionRobustEstimator.create(
                RobustEstimatorMethod.LMedS);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.LMedS);

        //test MSAC
        estimator = RadialDistortionRobustEstimator.create(
                RobustEstimatorMethod.MSAC);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.MSAC);
        
        //test PROSAC
        estimator = RadialDistortionRobustEstimator.create(
                RobustEstimatorMethod.PROSAC);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.PROSAC);
        
        //test PROMedS
        estimator = RadialDistortionRobustEstimator.create(
                RobustEstimatorMethod.PROMedS);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.PROMedS);        
        
        //test create with points, quality scores, center and method
        List<Point2D> distortedPoints = new ArrayList<>();
        List<Point2D> undistortedPoints = new ArrayList<>();
        double[] qualityScores = new double[
                RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS];
        for (int i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        Point2D center = Point2D.create();
        
        //test RANSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, qualityScores, center,
                        RobustEstimatorMethod.RANSAC);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getDistortionCenter(), center);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.RANSAC);
        
        //test LMedS
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, qualityScores, center,
                        RobustEstimatorMethod.LMedS);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getDistortionCenter(), center);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.LMedS);
        
        //test MSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, qualityScores, center,
                        RobustEstimatorMethod.MSAC);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getDistortionCenter(), center);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.MSAC);
        
        //test PROSAC        
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, qualityScores, center,
                        RobustEstimatorMethod.PROSAC);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getDistortionCenter(), center);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.PROSAC);
        
        //test PROMedS       
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, qualityScores, center,
                        RobustEstimatorMethod.PROMedS);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getDistortionCenter(), center);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.PROMedS);        
        
        
        //test create with points, center and method
        
        //test RANSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, center, 
                        RobustEstimatorMethod.RANSAC);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getDistortionCenter(), center);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.RANSAC);
        
        //test LMedS
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, center, RobustEstimatorMethod.LMedS);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getDistortionCenter(), center);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.LMedS);
        
        //test MSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, center, RobustEstimatorMethod.MSAC);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getDistortionCenter(), center);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.MSAC);
        
        //test PROSAC        
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, center, 
                        RobustEstimatorMethod.PROSAC);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getDistortionCenter(), center);
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.PROSAC);
        
        //test PROMedS       
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, center, 
                        RobustEstimatorMethod.PROMedS);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getDistortionCenter(), center);
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.PROMedS);        
        
        //test create with points, quality scores and method
        
        //test RANSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, qualityScores, 
                        RobustEstimatorMethod.RANSAC);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.RANSAC);

        //test LMedS
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, qualityScores, 
                        RobustEstimatorMethod.LMedS);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.LMedS);

        //test MSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, qualityScores, 
                        RobustEstimatorMethod.MSAC);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.MSAC);
        
        //test PROSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, qualityScores, 
                        RobustEstimatorMethod.PROSAC);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.PROSAC);
        
        //test PROMedS
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, qualityScores, 
                        RobustEstimatorMethod.PROMedS);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.PROMedS);
        
        
        //test create with points and method
        
        //test RANSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, RobustEstimatorMethod.RANSAC);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.RANSAC);

        //test LMedS
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, RobustEstimatorMethod.LMedS);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.LMedS);

        //test MSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, RobustEstimatorMethod.MSAC);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.MSAC);
        
        //test PROSAC
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, RobustEstimatorMethod.PROSAC);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.PROSAC);
        
        //test PROMedS
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, RobustEstimatorMethod.PROMedS);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RobustEstimatorMethod.PROMedS);
        
        //test create with default method
        estimator = RadialDistortionRobustEstimator.create();
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RadialDistortionRobustEstimator.DEFAULT_ROBUST_METHOD);        
        
        //test create with points and quality scores
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, qualityScores);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), 
                RadialDistortionRobustEstimator.DEFAULT_ROBUST_METHOD);

        
        //test create with points
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertNull(estimator.getDistortionCenter());
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RadialDistortionRobustEstimator.DEFAULT_ROBUST_METHOD);
        
        
        //test create with points, quality scores and center
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, qualityScores, center);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getDistortionCenter(), center);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), 
                RadialDistortionRobustEstimator.DEFAULT_ROBUST_METHOD);
        
        
        //test create with points and center
        estimator = RadialDistortionRobustEstimator.create(distortedPoints, 
                        undistortedPoints, center);
        
        //check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getDistortedPoints(), distortedPoints);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints);
        assertSame(estimator.getDistortionCenter(), center);
        assertTrue(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), 
                RadialDistortionRobustEstimator.DEFAULT_ROBUST_METHOD);        
    }
    
    @Test
    public void testAreValidPoints() {
        List<Point2D> distortedPoints = new ArrayList<>();
        List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        List<Point2D> emptyPoints = new ArrayList<>();
        
        assertTrue(RadialDistortionRobustEstimator.areValidPoints(
                distortedPoints, undistortedPoints));
        assertFalse(RadialDistortionRobustEstimator.areValidPoints(emptyPoints, 
                undistortedPoints));
        assertFalse(RadialDistortionRobustEstimator.areValidPoints(emptyPoints, 
                emptyPoints));
        assertFalse(RadialDistortionRobustEstimator.areValidPoints(null, 
                undistortedPoints));        
        assertFalse(RadialDistortionRobustEstimator.areValidPoints(
                distortedPoints, null));                
    }
    
    @Test
    public void testGetSetListener() throws LockedException {
        List<Point2D> distortedPoints = new ArrayList<>();
        List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        
        RadialDistortionRobustEstimatorListener listener =
                new RadialDistortionRobustEstimatorListener() {

            @Override
            public void onEstimateStart(
                    RadialDistortionRobustEstimator estimator) { }

            @Override
            public void onEstimateEnd(
                    RadialDistortionRobustEstimator estimator) { }

            @Override
            public void onEstimateNextIteration(
                    RadialDistortionRobustEstimator estimator, int iteration) { }

            @Override
            public void onEstimateProgressChange(
                    RadialDistortionRobustEstimator estimator, 
                    float progress) { }
        };
        
        RadialDistortionRobustEstimator estimator = 
                RadialDistortionRobustEstimator.create(distortedPoints, 
                undistortedPoints);
        
        //check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        
        //set new value
        estimator.setListener(listener);
        
        //check correctness
        assertSame(estimator.getListener(), listener);
        assertTrue(estimator.isListenerAvailable());
    }
    
    @Test
    public void testGetSetProgressDelta() throws LockedException {
        List<Point2D> distortedPoints = new ArrayList<>();
        List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        
        RadialDistortionRobustEstimator estimator = 
                RadialDistortionRobustEstimator.create(distortedPoints, 
                undistortedPoints);

        //check default value
        assertEquals(estimator.getProgressDelta(), 
                RadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        
        //set new value
        estimator.setProgressDelta(0.5f);
        
        //check correctness
        assertEquals(estimator.getProgressDelta(), 0.5, 0.0);
    }
    
    @Test
    public void testGetSetConfidence() throws LockedException {
        List<Point2D> distortedPoints = new ArrayList<>();
        List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        
        RadialDistortionRobustEstimator estimator = 
                RadialDistortionRobustEstimator.create(distortedPoints, 
                undistortedPoints);

        //check default value
        assertEquals(estimator.getConfidence(), 
                RadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        
        //set new value
        estimator.setConfidence(0.5f);
        
        //check correctness
        assertEquals(estimator.getConfidence(), 0.5, 0.0);
    }

    @Test
    public void testGetSetMaxIterations() throws LockedException {
        List<Point2D> distortedPoints = new ArrayList<>();
        List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        
        RadialDistortionRobustEstimator estimator = 
                RadialDistortionRobustEstimator.create(distortedPoints, 
                undistortedPoints);

        //check default value
        assertEquals(estimator.getMaxIterations(), 
                RadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS);
        
        //set new value
        estimator.setMaxIterations(10);
        
        //check correctness
        assertEquals(estimator.getMaxIterations(), 10);
    }
    
    @Test
    public void testGetSetPoints() throws LockedException {
        List<Point2D> distortedPoints1 = new ArrayList<>();
        List<Point2D> undistortedPoints1 = new ArrayList<>();
        List<Point2D> distortedPoints2 = new ArrayList<>();
        List<Point2D> undistortedPoints2 = new ArrayList<>();
        for (int i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints1.add(Point2D.create());
            undistortedPoints1.add(Point2D.create());
            distortedPoints2.add(Point2D.create());
            undistortedPoints2.add(Point2D.create());            
        }
        List<Point2D> emptyPoints = new ArrayList<>();
        
        RadialDistortionRobustEstimator estimator = 
                RadialDistortionRobustEstimator.create(distortedPoints1, 
                undistortedPoints1);

        //check correctness
        assertSame(estimator.getDistortedPoints(), distortedPoints1);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints1);
        assertTrue(estimator.arePointsAvailable());
        
        //set new value
        estimator.setPoints(distortedPoints2, undistortedPoints2);
        
        //check correctness
        assertSame(estimator.getDistortedPoints(), distortedPoints2);
        assertSame(estimator.getUndistortedPoints(), undistortedPoints2);   
        assertTrue(estimator.arePointsAvailable());
        
        //Force IllegalArgumentException
        try {
            estimator.setPoints(emptyPoints, undistortedPoints2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setPoints(emptyPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setPoints(null, undistortedPoints2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setPoints(distortedPoints2, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetDistortionCenter() throws LockedException {
        List<Point2D> distortedPoints = new ArrayList<>();
        List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        Point2D center = Point2D.create();
        
        RadialDistortionRobustEstimator estimator = 
                RadialDistortionRobustEstimator.create(distortedPoints, 
                undistortedPoints);

        //check default value
        assertNull(estimator.getDistortionCenter());
        
        //set new value
        estimator.setDistortionCenter(center);
        
        //check correctness
        assertSame(estimator.getDistortionCenter(), center);
    }    
    
    @Test
    public void testGetSetQualityScores() throws LockedException {
        List<Point2D> distortedPoints = new ArrayList<>();
        List<Point2D> undistortedPoints = new ArrayList<>();
        double[] qualityScores = new double[
                RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS];
        for (int i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        
        RadialDistortionRobustEstimator estimator = 
                RadialDistortionRobustEstimator.create(distortedPoints, 
                undistortedPoints, RobustEstimatorMethod.RANSAC);        
        
        //check default value
        assertNull(estimator.getQualityScores());
        
        //set new value
        estimator.setQualityScores(qualityScores);
        
        //check correctness
        assertNull(estimator.getQualityScores());
    }
}
