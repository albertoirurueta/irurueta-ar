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
package com.irurueta.ar.epipolar;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class EpipolarDistanceFundamentalMatrixComparatorTest implements
        FundamentalMatrixComparatorListener {
    
    private static final double MIN_RANDOM_VALUE = 100.0;
    private static final double MAX_RANDOM_VALUE = 500.0;
    
    private static final double MIN_FOCAL_LENGTH = 110.0;
    private static final double MAX_FOCAL_LENGTH = 130.0;
    
    private static final double MIN_SKEWNESS = -0.001;
    private static final double MAX_SKEWNESS = 0.001;
    
    private static final double MIN_PRINCIPAL_POINT = 90.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;
    
    private static final double MIN_ANGLE_DEGREES = 10.0;
    private static final double MAX_ANGLE_DEGREES = 15.0;
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    
    private int compareStart, compareEnd, compareProgressChange;
    
    public EpipolarDistanceFundamentalMatrixComparatorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testConstructor() throws AlgebraException, 
            InvalidFundamentalMatrixException {
        //test constructor without arguments
        EpipolarDistanceFundamentalMatrixComparator comparator =
                new EpipolarDistanceFundamentalMatrixComparator();
        
        //check default values
        assertNull(comparator.getGroundTruthFundamentalMatrix());
        assertNull(comparator.getOtherFundamentalMatrix());
        assertNull(comparator.getListener());
        assertFalse(comparator.isLocked());
        assertFalse(comparator.isReady());
        assertEquals(comparator.getType(), 
                FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR);
        assertEquals(comparator.getMinX(), 
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, 0.0);
        assertEquals(comparator.getMaxX(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, 0.0);
        assertEquals(comparator.getMinY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, 0.0);
        assertEquals(comparator.getMaxY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, 0.0);
        assertEquals(comparator.getNSamples(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, 
                0.0);
        assertEquals(comparator.getMinHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMinVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxIterationsFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_ITERATIONS_FACTOR, 0.0);
        assertEquals(comparator.getProgressDelta(), 
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_PROGRESS_DELTA, 0.0);
        
        //test constructor with fundamental matrices
        FundamentalMatrix emptyFundamentalMatrix1 = new FundamentalMatrix();
        FundamentalMatrix emptyFundamentalMatrix2 = new FundamentalMatrix();
        
        FundamentalMatrix fundamentalMatrix1 = createRandomFundamentalMatrix();
        FundamentalMatrix fundamentalMatrix2 = createRandomFundamentalMatrix();
        
        comparator = new EpipolarDistanceFundamentalMatrixComparator(
                emptyFundamentalMatrix1, emptyFundamentalMatrix2);
        
        //check default values
        assertSame(comparator.getGroundTruthFundamentalMatrix(), 
                emptyFundamentalMatrix1);
        assertSame(comparator.getOtherFundamentalMatrix(), 
                emptyFundamentalMatrix2);
        assertNull(comparator.getListener());
        assertFalse(comparator.isLocked());
        assertFalse(comparator.isReady()); //fundamental matrices are not defined
        assertEquals(comparator.getType(), 
                FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR);
        assertEquals(comparator.getMinX(), 
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, 0.0);
        assertEquals(comparator.getMaxX(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, 0.0);
        assertEquals(comparator.getMinY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, 0.0);
        assertEquals(comparator.getMaxY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, 0.0);
        assertEquals(comparator.getNSamples(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, 
                0.0);
        assertEquals(comparator.getMinHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMinVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxIterationsFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_ITERATIONS_FACTOR, 0.0);
        assertEquals(comparator.getProgressDelta(), 
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_PROGRESS_DELTA, 0.0);
        
        comparator = new EpipolarDistanceFundamentalMatrixComparator(
                fundamentalMatrix1, emptyFundamentalMatrix2);
        
        //check default values
        assertSame(comparator.getGroundTruthFundamentalMatrix(), 
                fundamentalMatrix1);
        assertSame(comparator.getOtherFundamentalMatrix(), 
                emptyFundamentalMatrix2);
        assertNull(comparator.getListener());
        assertFalse(comparator.isLocked());
        assertFalse(comparator.isReady()); //fundamental matrices are not defined
        assertEquals(comparator.getType(), 
                FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR);
        assertEquals(comparator.getMinX(), 
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, 0.0);
        assertEquals(comparator.getMaxX(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, 0.0);
        assertEquals(comparator.getMinY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, 0.0);
        assertEquals(comparator.getMaxY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, 0.0);
        assertEquals(comparator.getNSamples(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, 
                0.0);
        assertEquals(comparator.getMinHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMinVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxIterationsFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_ITERATIONS_FACTOR, 0.0);
        assertEquals(comparator.getProgressDelta(), 
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_PROGRESS_DELTA, 0.0);

        comparator = new EpipolarDistanceFundamentalMatrixComparator(
                emptyFundamentalMatrix1, fundamentalMatrix2);
        
        //check default values
        assertSame(comparator.getGroundTruthFundamentalMatrix(), 
                emptyFundamentalMatrix1);
        assertSame(comparator.getOtherFundamentalMatrix(), 
                fundamentalMatrix2);
        assertNull(comparator.getListener());
        assertFalse(comparator.isLocked());
        assertFalse(comparator.isReady()); //fundamental matrices are not defined
        assertEquals(comparator.getType(), 
                FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR);
        assertEquals(comparator.getMinX(), 
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, 0.0);
        assertEquals(comparator.getMaxX(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, 0.0);
        assertEquals(comparator.getMinY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, 0.0);
        assertEquals(comparator.getMaxY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, 0.0);
        assertEquals(comparator.getNSamples(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, 
                0.0);
        assertEquals(comparator.getMinHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMinVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxIterationsFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_ITERATIONS_FACTOR, 0.0);
        assertEquals(comparator.getProgressDelta(), 
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_PROGRESS_DELTA, 0.0);

        comparator = new EpipolarDistanceFundamentalMatrixComparator(
                fundamentalMatrix1, fundamentalMatrix2);
        
        //check default values
        assertSame(comparator.getGroundTruthFundamentalMatrix(), 
                fundamentalMatrix1);
        assertSame(comparator.getOtherFundamentalMatrix(), 
                fundamentalMatrix2);
        assertNull(comparator.getListener());
        assertFalse(comparator.isLocked());
        assertTrue(comparator.isReady());
        assertEquals(comparator.getType(), 
                FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR);
        assertEquals(comparator.getMinX(), 
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, 0.0);
        assertEquals(comparator.getMaxX(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, 0.0);
        assertEquals(comparator.getMinY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, 0.0);
        assertEquals(comparator.getMaxY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, 0.0);
        assertEquals(comparator.getNSamples(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, 
                0.0);
        assertEquals(comparator.getMinHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMinVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxIterationsFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_ITERATIONS_FACTOR, 0.0);
        assertEquals(comparator.getProgressDelta(), 
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_PROGRESS_DELTA, 0.0);
        
        //test constructor with listener
        comparator = new EpipolarDistanceFundamentalMatrixComparator(this);
        
        //check default values
        assertNull(comparator.getGroundTruthFundamentalMatrix());
        assertNull(comparator.getOtherFundamentalMatrix());
        assertSame(comparator.getListener(), this);
        assertFalse(comparator.isLocked());
        assertFalse(comparator.isReady());
        assertEquals(comparator.getType(), 
                FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR);
        assertEquals(comparator.getMinX(), 
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, 0.0);
        assertEquals(comparator.getMaxX(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, 0.0);
        assertEquals(comparator.getMinY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, 0.0);
        assertEquals(comparator.getMaxY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, 0.0);
        assertEquals(comparator.getNSamples(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, 
                0.0);
        assertEquals(comparator.getMinHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMinVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxIterationsFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_ITERATIONS_FACTOR, 0.0);
        assertEquals(comparator.getProgressDelta(), 
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_PROGRESS_DELTA, 0.0);
        
        //test constructor with fundamental matrices and listener
        comparator = new EpipolarDistanceFundamentalMatrixComparator(
                emptyFundamentalMatrix1, emptyFundamentalMatrix2, this);
        
        //check default values
        assertSame(comparator.getGroundTruthFundamentalMatrix(), 
                emptyFundamentalMatrix1);
        assertSame(comparator.getOtherFundamentalMatrix(), 
                emptyFundamentalMatrix2);
        assertSame(comparator.getListener(), this);
        assertFalse(comparator.isLocked());
        assertFalse(comparator.isReady()); //fundamental matrices are not defined
        assertEquals(comparator.getType(), 
                FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR);
        assertEquals(comparator.getMinX(), 
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, 0.0);
        assertEquals(comparator.getMaxX(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, 0.0);
        assertEquals(comparator.getMinY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, 0.0);
        assertEquals(comparator.getMaxY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, 0.0);
        assertEquals(comparator.getNSamples(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, 
                0.0);
        assertEquals(comparator.getMinHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMinVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxIterationsFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_ITERATIONS_FACTOR, 0.0);
        assertEquals(comparator.getProgressDelta(), 
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_PROGRESS_DELTA, 0.0);
        
        comparator = new EpipolarDistanceFundamentalMatrixComparator(
                fundamentalMatrix1, emptyFundamentalMatrix2, this);
        
        //check default values
        assertSame(comparator.getGroundTruthFundamentalMatrix(), 
                fundamentalMatrix1);
        assertSame(comparator.getOtherFundamentalMatrix(), 
                emptyFundamentalMatrix2);
        assertSame(comparator.getListener(), this);
        assertFalse(comparator.isLocked());
        assertFalse(comparator.isReady()); //fundamental matrices are not defined
        assertEquals(comparator.getType(), 
                FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR);
        assertEquals(comparator.getMinX(), 
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, 0.0);
        assertEquals(comparator.getMaxX(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, 0.0);
        assertEquals(comparator.getMinY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, 0.0);
        assertEquals(comparator.getMaxY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, 0.0);
        assertEquals(comparator.getNSamples(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, 
                0.0);
        assertEquals(comparator.getMinHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMinVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxIterationsFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_ITERATIONS_FACTOR, 0.0);
        assertEquals(comparator.getProgressDelta(), 
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_PROGRESS_DELTA, 0.0);

        comparator = new EpipolarDistanceFundamentalMatrixComparator(
                emptyFundamentalMatrix1, fundamentalMatrix2, this);
        
        //check default values
        assertSame(comparator.getGroundTruthFundamentalMatrix(), 
                emptyFundamentalMatrix1);
        assertSame(comparator.getOtherFundamentalMatrix(), 
                fundamentalMatrix2);
        assertSame(comparator.getListener(), this);
        assertFalse(comparator.isLocked());
        assertFalse(comparator.isReady()); //fundamental matrices are not defined
        assertEquals(comparator.getType(), 
                FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR);
        assertEquals(comparator.getMinX(), 
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, 0.0);
        assertEquals(comparator.getMaxX(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, 0.0);
        assertEquals(comparator.getMinY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, 0.0);
        assertEquals(comparator.getMaxY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, 0.0);
        assertEquals(comparator.getNSamples(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, 
                0.0);
        assertEquals(comparator.getMinHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMinVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxIterationsFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_ITERATIONS_FACTOR, 0.0);
        assertEquals(comparator.getProgressDelta(), 
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_PROGRESS_DELTA, 0.0);

        comparator = new EpipolarDistanceFundamentalMatrixComparator(
                fundamentalMatrix1, fundamentalMatrix2, this);
        
        //check default values
        assertSame(comparator.getGroundTruthFundamentalMatrix(), 
                fundamentalMatrix1);
        assertSame(comparator.getOtherFundamentalMatrix(), 
                fundamentalMatrix2);
        assertSame(comparator.getListener(), this);
        assertFalse(comparator.isLocked());
        assertTrue(comparator.isReady());        
        assertEquals(comparator.getType(), 
                FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR);        
        assertEquals(comparator.getMinX(), 
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, 0.0);
        assertEquals(comparator.getMaxX(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, 0.0);
        assertEquals(comparator.getMinY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, 0.0);
        assertEquals(comparator.getMaxY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, 0.0);
        assertEquals(comparator.getNSamples(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, 
                0.0);
        assertEquals(comparator.getMinHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMinVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxIterationsFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_ITERATIONS_FACTOR, 0.0);
        assertEquals(comparator.getProgressDelta(), 
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_PROGRESS_DELTA, 0.0);        
    }
    
    @Test
    public void testGetSetMinMaxX() throws LockedException {
        EpipolarDistanceFundamentalMatrixComparator comparator =
                new EpipolarDistanceFundamentalMatrixComparator();
        
        //check default values
        assertEquals(comparator.getMinX(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, 0.0);
        assertEquals(comparator.getMaxX(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, 0.0);
        
        //set new values
        comparator.setMinMaxX(5.0, 10.0);
        
        
        //check correctness
        assertEquals(comparator.getMinX(), 5.0, 0.0);
        assertEquals(comparator.getMaxX(), 10.0, 0.0);
        
        //Force IllegalArgumentException
        try {
            comparator.setMinMaxX(5.0, 5.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetMinMaxY() throws LockedException {
        EpipolarDistanceFundamentalMatrixComparator comparator =
                new EpipolarDistanceFundamentalMatrixComparator();
        
        //check default values
        assertEquals(comparator.getMinY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, 0.0);
        assertEquals(comparator.getMaxY(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, 0.0);
        
        //set new values
        comparator.setMinMaxY(5.0, 10.0);
        
        
        //check correctness
        assertEquals(comparator.getMinY(), 5.0, 0.0);
        assertEquals(comparator.getMaxY(), 10.0, 0.0);
        
        //Force IllegalArgumentException
        try {
            comparator.setMinMaxY(5.0, 5.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetNSamples() throws LockedException {
        EpipolarDistanceFundamentalMatrixComparator comparator =
                new EpipolarDistanceFundamentalMatrixComparator();
        
        //check default values
        assertEquals(comparator.getNSamples(),
                EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES);
        
        //set new value
        comparator.setNSamples(1);
        
        //check correctness
        assertEquals(comparator.getNSamples(), 1);
        
        //Force IllegalArgumentException
        try {
            comparator.setNSamples(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetMinMaxHorizontalDisparityFactor() 
            throws LockedException {
        EpipolarDistanceFundamentalMatrixComparator comparator =
                new EpipolarDistanceFundamentalMatrixComparator();
        
        //check default values
        assertEquals(comparator.getMinHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxHorizontalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        
        //set new value
        comparator.setMinMaxHorizontalDisparityFactor(-0.2, 0.2);
        
        //check correctness
        assertEquals(comparator.getMinHorizontalDisparityFactor(), -0.2, 0.0);
        assertEquals(comparator.getMaxHorizontalDisparityFactor(), 0.2, 0.0);
        
        //Force IllegalArgumentException
        try {
            comparator.setMinMaxHorizontalDisparityFactor(0.2, -0.2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetMinMaxVerticalDisparityFactor() 
            throws LockedException {
        EpipolarDistanceFundamentalMatrixComparator comparator =
                new EpipolarDistanceFundamentalMatrixComparator();
        
        //check default values
        assertEquals(comparator.getMinVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MIN_DISPARITY_FACTOR, 0.0);
        assertEquals(comparator.getMaxVerticalDisparityFactor(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_DISPARITY_FACTOR, 0.0);
        
        //set new value
        comparator.setMinMaxVerticalDisparityFactor(-0.2, 0.2);
        
        //check correctness
        assertEquals(comparator.getMinVerticalDisparityFactor(), -0.2, 0.0);
        assertEquals(comparator.getMaxVerticalDisparityFactor(), 0.2, 0.0);
        
        //Force IllegalArgumentException
        try {
            comparator.setMinMaxVerticalDisparityFactor(0.2, -0.2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetMaxIterationsFactor() throws LockedException {
        EpipolarDistanceFundamentalMatrixComparator comparator =
                new EpipolarDistanceFundamentalMatrixComparator();
        
        //check default value
        assertEquals(comparator.getMaxIterationsFactor(), 
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_MAX_ITERATIONS_FACTOR, 0.0);
        
        //set new value
        comparator.setMaxIterationsFactor(1.0);
        
        //check correctness
        assertEquals(comparator.getMaxIterationsFactor(), 1.0, 0.0);
        
        //Foce IllegalArgumentException
        try {
            comparator.setMaxIterationsFactor(0.5);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetProgressDelta() throws LockedException {
        EpipolarDistanceFundamentalMatrixComparator comparator =
                new EpipolarDistanceFundamentalMatrixComparator();
        
        //check default value
        assertEquals(comparator.getProgressDelta(),
                EpipolarDistanceFundamentalMatrixComparator.
                DEFAULT_PROGRESS_DELTA, 0.0);
        
        //set new value
        comparator.setProgressDelta(0.5f);
        
        //check correctness
        assertEquals(comparator.getProgressDelta(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            comparator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            comparator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testCompare() throws InvalidPairOfCamerasException,
            NotReadyException, LockedException, 
            FundamentalMatrixComparatorException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double alphaEuler1 = 0.0;
        double betaEuler1 = 0.0;
        double gammaEuler1 = 0.0;
        double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        double horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        double verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        double horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        double verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        
        double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        
        double horizontalPrincipalPoint1 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        double verticalPrincipalPoint1 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        double horizontalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        double verticalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        
        Point3D cameraCenter1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point3D cameraCenter2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, 
                gammaEuler1);
        Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, 
                gammaEuler2);
        
        PinholeCameraIntrinsicParameters intrinsic1 = 
                new PinholeCameraIntrinsicParameters(horizontalFocalLength1, 
                verticalFocalLength1, horizontalPrincipalPoint1, 
                verticalPrincipalPoint1, skewness1);
        PinholeCameraIntrinsicParameters intrinsic2 = 
                new PinholeCameraIntrinsicParameters(horizontalFocalLength2, 
                verticalFocalLength2, horizontalPrincipalPoint2, 
                verticalPrincipalPoint2, skewness2);
        
        PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, 
                cameraCenter1);
        PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                cameraCenter2);
        
        FundamentalMatrix fundamentalMatrix1 = new FundamentalMatrix(camera1, 
                camera2);
        FundamentalMatrix fundamentalMatrix2 = new FundamentalMatrix(camera1, 
                camera2);
        
        EpipolarDistanceFundamentalMatrixComparator comparator = 
                new EpipolarDistanceFundamentalMatrixComparator(
                fundamentalMatrix1, fundamentalMatrix2, this);
        
        //check status
        assertFalse(comparator.isLocked());
        assertTrue(comparator.isReady());
        assertEquals(compareStart, 0);
        assertEquals(compareEnd, 0);
        
        //compare
        assertEquals(comparator.compare(), 0.0, ABSOLUTE_ERROR);
        
        //check correctness
        assertTrue(comparator.isReady());
        assertFalse(comparator.isLocked());
        assertEquals(compareStart, 1);
        assertEquals(compareEnd, 1);
        assertTrue(compareProgressChange > 0);
        reset();
    }
    
    private void reset() {
        compareStart = compareEnd = compareProgressChange = 0;
    }
        
    @Override
    public void onCompareStart(FundamentalMatrixComparator comparator) {
        compareStart++;
        testLocked((EpipolarDistanceFundamentalMatrixComparator)comparator);
    }

    @Override
    public void onCompareEnd(FundamentalMatrixComparator comparator) {
        compareEnd++;
        testLocked((EpipolarDistanceFundamentalMatrixComparator)comparator);
    }

    @Override
    public void onCompareProgressChange(FundamentalMatrixComparator comparator, 
            float progress) {
        compareProgressChange++;
        testLocked((EpipolarDistanceFundamentalMatrixComparator)comparator);
    }

    private void testLocked(
            EpipolarDistanceFundamentalMatrixComparator comparator) {
        assertTrue(comparator.isLocked());
        try {
            comparator.setGroundTruthFundamentalMatrix(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            comparator.setOtherFundamentalMatrix(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            comparator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            comparator.compare();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception ignore) {
            fail("LockedException expected but not thrown");
        }
        try {
            comparator.setMinMaxX(0.0, 100.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            comparator.setMinMaxY(0.0, 100.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            comparator.setNSamples(1);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            comparator.setMinMaxHorizontalDisparityFactor(-0.2, 0.2);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            comparator.setMinMaxVerticalDisparityFactor(-0.2, 0.2);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            comparator.setMaxIterationsFactor(2.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            comparator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
    }
    
    private FundamentalMatrix createRandomFundamentalMatrix() 
            throws AlgebraException, InvalidFundamentalMatrixException {
        FundamentalMatrix fundamentalMatrix;
        int rank;
        do {
            Matrix internalMatrix = Matrix.createWithUniformRandomValues(
                    FundamentalMatrix.FUNDAMENTAL_MATRIX_ROWS, 
                    FundamentalMatrix.FUNDAMENTAL_MATRIX_COLS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            
            //ensure that internal matrix has rank 2
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    internalMatrix);
            decomposer.decompose();
            
            rank = decomposer.getRank(); //if rank is less than 2 we need to
                                         //pick another random matrix
            
            Matrix U = decomposer.getU();
            Matrix W = decomposer.getW();
            Matrix V = decomposer.getV();
            Matrix transV = V.transposeAndReturnNew();
            
            //set last element to 0 to force rank 2
            W.setElementAt(2, 2, 0.0);
            
            internalMatrix = U.multiplyAndReturnNew(W.multiplyAndReturnNew(
                    transV));
            
            fundamentalMatrix = new FundamentalMatrix(internalMatrix);
        } while (rank < 2);
        
        return fundamentalMatrix;
    }    
}
