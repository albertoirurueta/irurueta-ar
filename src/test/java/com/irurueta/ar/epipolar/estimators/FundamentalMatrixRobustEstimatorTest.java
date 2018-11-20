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
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class FundamentalMatrixRobustEstimatorTest {
    
    public static final int MIN_REQUIRED_POINTS_7 = 7;
    public static final int MIN_REQUIRED_POINTS_8 = 8;
    
    public FundamentalMatrixRobustEstimatorTest() { }
    
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
        //create with method
        
        //RANSAC
        FundamentalMatrixRobustEstimator estimator =
                FundamentalMatrixRobustEstimator.create(
                RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //LMedS
        estimator = FundamentalMatrixRobustEstimator.create(
                RobustEstimatorMethod.LMedS);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //MSAC
        estimator = FundamentalMatrixRobustEstimator.create(
                RobustEstimatorMethod.MSAC);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //PROSAC
        estimator = FundamentalMatrixRobustEstimator.create(
                RobustEstimatorMethod.PROSAC);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //PROMedS
        estimator = FundamentalMatrixRobustEstimator.create(
                RobustEstimatorMethod.PROMedS);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //create with left and right points
        List<Point2D> leftPoints = new ArrayList<>();
        List<Point2D> rightPoints = new ArrayList<>();
        for (int i = 0; i < MIN_REQUIRED_POINTS_8; i++) {
            leftPoints.add(Point2D.create());
            rightPoints.add(Point2D.create());
        }
        
        //RANSAC
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, 
                rightPoints, RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        //LMedS
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, 
                rightPoints, RobustEstimatorMethod.LMedS);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //MSAC
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, 
                rightPoints, RobustEstimatorMethod.MSAC);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //PROSAC
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, 
                rightPoints, RobustEstimatorMethod.PROSAC);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        //PROMedS
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, 
                rightPoints, RobustEstimatorMethod.PROMedS);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //create with left and right points, and quality scores
        double[] qualityScores = new double[MIN_REQUIRED_POINTS_8];
        
        //RANSAC
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, 
                rightPoints, qualityScores, RobustEstimatorMethod.RANSAC);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertTrue(estimator instanceof RANSACFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());

        //LMedS
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, 
                rightPoints, qualityScores, RobustEstimatorMethod.LMedS);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(estimator instanceof LMedSFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //MSAC
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, 
                rightPoints, qualityScores, RobustEstimatorMethod.MSAC);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.MSAC);
        assertTrue(estimator instanceof MSACFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //PROSAC
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, 
                rightPoints, qualityScores, RobustEstimatorMethod.PROSAC);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof PROSACFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //PROMedS
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, 
                rightPoints, qualityScores, RobustEstimatorMethod.PROMedS);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertTrue(estimator instanceof 
                PROMedSFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //test create without arguments
        estimator = FundamentalMatrixRobustEstimator.create();
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //test create with left and right points
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, 
                rightPoints);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACFundamentalMatrixRobustEstimator);
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());
        assertNull(estimator.getCovariance());
        
        //test create with left and right points, and quality scores
        estimator = FundamentalMatrixRobustEstimator.create(leftPoints, 
                rightPoints, qualityScores);
        assertEquals(estimator.getNonRobustFundamentalMatrixEstimatorMethod(), 
                FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM);
        assertEquals(estimator.getMinRequiredPoints(), MIN_REQUIRED_POINTS_7);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(), 
                FundamentalMatrixRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(), 
                FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getInliersData());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertTrue(estimator instanceof 
                PROSACFundamentalMatrixRobustEstimator);        
        assertTrue(estimator.isResultRefined());
        assertFalse(estimator.isCovarianceKept());   
        assertNull(estimator.getCovariance());
    }
}
