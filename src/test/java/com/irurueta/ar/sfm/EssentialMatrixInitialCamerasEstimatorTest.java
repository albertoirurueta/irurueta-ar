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
package com.irurueta.ar.sfm;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.ar.epipolar.*;
import com.irurueta.geometry.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class EssentialMatrixInitialCamerasEstimatorTest implements
        InitialCamerasEstimatorListener {
    
    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;
    
    private static final double MIN_ANGLE_DEGREES = -30.0;
    private static final double MAX_ANGLE_DEGREES = -15.0;
    
    private static final double MIN_CAMERA_SEPARATION = 5.0;
    private static final double MAX_CAMERA_SEPARATION = 10.0;
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;
    
    private static final int MIN_POINTS = 50;
    private static final int MAX_POINTS = 100;
    
    private static final double MIN_LAMBDA = 100.0;
    private static final double MAX_LAMBDA = 500.0;
    
    private static final int TIMES = 50;
    private static final int MAX_TRIES = 2000;
    
    public EssentialMatrixInitialCamerasEstimatorTest() { }
    
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
        EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();
        
        //check default values
        assertNull(estimator.getFundamentalMatrix());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());        
        assertEquals(estimator.getMethod(), 
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        
        FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
        estimator = new EssentialMatrixInitialCamerasEstimator(
                fundamentalMatrix);
        
        //check default values
        assertSame(estimator.getFundamentalMatrix(), fundamentalMatrix);
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());        
        assertEquals(estimator.getMethod(), 
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());


        PinholeCameraIntrinsicParameters leftIntrinsic = 
                new PinholeCameraIntrinsicParameters();
        PinholeCameraIntrinsicParameters rightIntrinsic =
                new PinholeCameraIntrinsicParameters();
        estimator = new EssentialMatrixInitialCamerasEstimator(leftIntrinsic, 
                rightIntrinsic);
        
        //check default values
        assertNull(estimator.getFundamentalMatrix());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());        
        assertEquals(estimator.getMethod(), 
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        assertFalse(estimator.isReady());
        assertSame(estimator.getLeftIntrinsic(), leftIntrinsic);
        assertSame(estimator.getRightIntrinsic(), rightIntrinsic);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());
        
        
        estimator = new EssentialMatrixInitialCamerasEstimator(
                fundamentalMatrix, leftIntrinsic, rightIntrinsic);
        
        //check default values
        assertSame(estimator.getFundamentalMatrix(), fundamentalMatrix);
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());        
        assertEquals(estimator.getMethod(), 
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        assertFalse(estimator.isReady());
        assertSame(estimator.getLeftIntrinsic(), leftIntrinsic);
        assertSame(estimator.getRightIntrinsic(), rightIntrinsic);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());
        
        
        List<Point2D> leftPoints = new ArrayList<>();
        leftPoints.add(Point2D.create());
        List<Point2D> rightPoints = new ArrayList<>();
        rightPoints.add(Point2D.create());
        estimator = new EssentialMatrixInitialCamerasEstimator(leftPoints, 
                rightPoints);
        
        //check default values
        assertNull(estimator.getFundamentalMatrix());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());        
        assertEquals(estimator.getMethod(), 
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(leftPoints, 
                    null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(null, 
                    rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        
        estimator = new EssentialMatrixInitialCamerasEstimator(
                fundamentalMatrix, leftPoints, rightPoints);
        
        //check default values
        assertSame(estimator.getFundamentalMatrix(), fundamentalMatrix);
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());        
        assertEquals(estimator.getMethod(), 
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, leftPoints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, null, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        
        estimator = new EssentialMatrixInitialCamerasEstimator(leftIntrinsic,
                rightIntrinsic, leftPoints, rightPoints);
        
        //check default values
        assertNull(estimator.getFundamentalMatrix());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());        
        assertEquals(estimator.getMethod(), 
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        assertFalse(estimator.isReady());
        assertSame(estimator.getLeftIntrinsic(), leftIntrinsic);
        assertSame(estimator.getRightIntrinsic(), rightIntrinsic);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    leftIntrinsic, rightIntrinsic, leftPoints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    leftIntrinsic, rightIntrinsic, null, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        
        estimator = new EssentialMatrixInitialCamerasEstimator(
                fundamentalMatrix, leftIntrinsic, rightIntrinsic, leftPoints, 
                rightPoints);
        
        //check default values
        assertSame(estimator.getFundamentalMatrix(), fundamentalMatrix);
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());        
        assertEquals(estimator.getMethod(), 
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        assertTrue(estimator.isReady());
        assertSame(estimator.getLeftIntrinsic(), leftIntrinsic);
        assertSame(estimator.getRightIntrinsic(), rightIntrinsic);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, leftIntrinsic, rightIntrinsic, 
                    leftPoints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, leftIntrinsic, rightIntrinsic, null, 
                    rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);

        
        estimator = new EssentialMatrixInitialCamerasEstimator(this);
        
        //check default values
        assertNull(estimator.getFundamentalMatrix());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());        
        assertEquals(estimator.getMethod(), 
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        
        estimator = new EssentialMatrixInitialCamerasEstimator(
                fundamentalMatrix, this);
        
        //check default values
        assertSame(estimator.getFundamentalMatrix(), fundamentalMatrix);
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());        
        assertEquals(estimator.getMethod(), 
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());


        estimator = new EssentialMatrixInitialCamerasEstimator(leftIntrinsic, 
                rightIntrinsic, this);
        
        //check default values
        assertNull(estimator.getFundamentalMatrix());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());        
        assertEquals(estimator.getMethod(), 
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        assertFalse(estimator.isReady());
        assertSame(estimator.getLeftIntrinsic(), leftIntrinsic);
        assertSame(estimator.getRightIntrinsic(), rightIntrinsic);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());
        
        
        estimator = new EssentialMatrixInitialCamerasEstimator(
                fundamentalMatrix, leftIntrinsic, rightIntrinsic, this);
        
        //check default values
        assertSame(estimator.getFundamentalMatrix(), fundamentalMatrix);
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());        
        assertEquals(estimator.getMethod(), 
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        assertFalse(estimator.isReady());
        assertSame(estimator.getLeftIntrinsic(), leftIntrinsic);
        assertSame(estimator.getRightIntrinsic(), rightIntrinsic);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());
        
        
        estimator = new EssentialMatrixInitialCamerasEstimator(leftPoints, 
                rightPoints, this);
        
        //check default values
        assertNull(estimator.getFundamentalMatrix());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());        
        assertEquals(estimator.getMethod(), 
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(leftPoints, 
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(null, 
                    rightPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        
        estimator = new EssentialMatrixInitialCamerasEstimator(
                fundamentalMatrix, leftPoints, rightPoints, this);
        
        //check default values
        assertSame(estimator.getFundamentalMatrix(), fundamentalMatrix);
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());        
        assertEquals(estimator.getMethod(), 
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, leftPoints, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, null, rightPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        
        estimator = new EssentialMatrixInitialCamerasEstimator(leftIntrinsic,
                rightIntrinsic, leftPoints, rightPoints, this);
        
        //check default values
        assertNull(estimator.getFundamentalMatrix());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());        
        assertEquals(estimator.getMethod(), 
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        assertFalse(estimator.isReady());
        assertSame(estimator.getLeftIntrinsic(), leftIntrinsic);
        assertSame(estimator.getRightIntrinsic(), rightIntrinsic);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    leftIntrinsic, rightIntrinsic, leftPoints, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    leftIntrinsic, rightIntrinsic, null, rightPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        
        estimator = new EssentialMatrixInitialCamerasEstimator(
                fundamentalMatrix, leftIntrinsic, rightIntrinsic, leftPoints, 
                rightPoints, this);
        
        //check default values
        assertSame(estimator.getFundamentalMatrix(), fundamentalMatrix);
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());        
        assertEquals(estimator.getMethod(), 
                InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        assertTrue(estimator.isReady());
        assertSame(estimator.getLeftIntrinsic(), leftIntrinsic);
        assertSame(estimator.getRightIntrinsic(), rightIntrinsic);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, leftIntrinsic, rightIntrinsic, 
                    leftPoints, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, leftIntrinsic, rightIntrinsic, null, 
                    rightPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);        
    }
    
    @Test
    public void testGetSetFundamentalMatrix() throws 
            com.irurueta.geometry.estimators.LockedException {
        EssentialMatrixInitialCamerasEstimator estimator = 
                new EssentialMatrixInitialCamerasEstimator();
        
        //check default value
        assertNull(estimator.getFundamentalMatrix());
        
        //set new value
        FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
        estimator.setFundamentalMatrix(fundamentalMatrix);
        
        //chck correctness
        assertSame(estimator.getFundamentalMatrix(), fundamentalMatrix);
    }
    
    @Test
    public void testGetSetListener() {
        EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        
        //set new value
        estimator.setListener(this);
        
        //check correctness
        assertSame(estimator.getListener(), this);
    }
    
    @Test
    public void testGetSetLeftIntrinsic() 
            throws com.irurueta.geometry.estimators.LockedException {
        EssentialMatrixInitialCamerasEstimator estimator = 
                new EssentialMatrixInitialCamerasEstimator();
        
        //check default value
        assertNull(estimator.getLeftIntrinsic());
        
        //set new value
        PinholeCameraIntrinsicParameters leftIntrinsic = 
                new PinholeCameraIntrinsicParameters();
        estimator.setLeftIntrinsic(leftIntrinsic);
        
        //check correctness
        assertSame(estimator.getLeftIntrinsic(), leftIntrinsic);
    }
    
    @Test
    public void testGetSetRightIntrinsic() 
            throws com.irurueta.geometry.estimators.LockedException {
        EssentialMatrixInitialCamerasEstimator estimator = 
                new EssentialMatrixInitialCamerasEstimator();
        
        //check default value
        assertNull(estimator.getRightIntrinsic());
        
        //set new value
        PinholeCameraIntrinsicParameters rightIntrinsic =
                new PinholeCameraIntrinsicParameters();
        estimator.setRightIntrinsic(rightIntrinsic);
        
        //check correctness
        assertSame(estimator.getRightIntrinsic(), rightIntrinsic);
    }
    
    @Test
    public void testSetLeftAndRightIntrinsics() 
            throws com.irurueta.geometry.estimators.LockedException {
        EssentialMatrixInitialCamerasEstimator estimator = 
                new EssentialMatrixInitialCamerasEstimator();

        //check default values
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        
        //set new values
        PinholeCameraIntrinsicParameters leftIntrinsic =
                new PinholeCameraIntrinsicParameters();
        PinholeCameraIntrinsicParameters rightIntrinsic =
                new PinholeCameraIntrinsicParameters();
        estimator.setLeftAndRightIntrinsics(leftIntrinsic, rightIntrinsic);
        
        //check correctness
        assertSame(estimator.getLeftIntrinsic(), leftIntrinsic);
        assertSame(estimator.getRightIntrinsic(), rightIntrinsic);
    }
    
    @Test
    public void testSetIntrinsicsForBoth() 
            throws com.irurueta.geometry.estimators.LockedException {
        EssentialMatrixInitialCamerasEstimator estimator = 
                new EssentialMatrixInitialCamerasEstimator();

        //check default values
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());

        //set new value
        PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters();
        estimator.setIntrinsicsForBoth(intrinsic);
        
        //check correctness
        assertSame(estimator.getLeftIntrinsic(), intrinsic);
        assertSame(estimator.getRightIntrinsic(), intrinsic);
    }
    
    @Test
    public void testGetSetLeftPoints() 
            throws com.irurueta.geometry.estimators.LockedException {
        EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();
        
        //check default value
        assertNull(estimator.getLeftPoints());
        
        //set new value
        List<Point2D> leftPoints = new ArrayList<>();
        estimator.setLeftPoints(leftPoints);
        
        //check correctness
        assertSame(estimator.getLeftPoints(), leftPoints);
    }
    
    @Test
    public void testGetSetRightPoints() 
            throws com.irurueta.geometry.estimators.LockedException {
        EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();
        
        //check default value
        assertNull(estimator.getRightPoints());
        
        //set new value
        List<Point2D> rightPoints = new ArrayList<>();
        estimator.setRightPoints(rightPoints);
        
        //check correctness
        assertSame(estimator.getRightPoints(), rightPoints);
    }
    
    @Test
    public void testSetLeftAndRightPoints() 
            throws com.irurueta.geometry.estimators.LockedException {
        EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();
        
        //check default values
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        
        //set new values
        List<Point2D> leftPoints = new ArrayList<>();
        leftPoints.add(Point2D.create());
        List<Point2D> rightPoints = new ArrayList<>();
        rightPoints.add(Point2D.create());
        estimator.setLeftAndRightPoints(leftPoints, rightPoints);
        
        //check correctness
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        
        //Force IllegalArgumentException
        try {
            estimator.setLeftAndRightPoints(leftPoints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setLeftAndRightPoints(null, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        List<Point2D> wrongPoints = new ArrayList<>();
        try {
            estimator.setLeftAndRightPoints(leftPoints, wrongPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetCorrectorType() 
            throws com.irurueta.geometry.estimators.LockedException {
        EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();
        
        //check default value
        assertEquals(estimator.getCorrectorType(), Corrector.DEFAULT_TYPE);
        
        //set new value
        estimator.setCorrectorType(CorrectorType.GOLD_STANDARD);
        
        //check correctness
        assertEquals(estimator.getCorrectorType(), CorrectorType.GOLD_STANDARD);
    }
    
    @Test
    public void testAreSetPointsTriangulated() 
            throws com.irurueta.geometry.estimators.LockedException {
        EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();
        
        //check default value
        assertEquals(estimator.arePointsTriangulated(), 
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        
        //set new value
        estimator.setPointsTriangulated(!EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
        
        //check correctness
        assertEquals(estimator.arePointsTriangulated(), 
                !EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_TRIANGULATE_POINTS);
    }
    
    @Test
    public void testAreValidTriangulatedPointsMarked() 
            throws com.irurueta.geometry.estimators.LockedException {
        EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();
        
        //check default value
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        
        //set new value
        estimator.setValidTriangulatedPointsMarked(
                !EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
        
        //check correctness
        assertEquals(estimator.areValidTriangulatedPointsMarked(),
                !EssentialMatrixInitialCamerasEstimator.
                        DEFAULT_MARK_VALID_TRIANGULATED_POINTS);
    }
    
    @Test
    public void testEstimate() throws InvalidPairOfCamerasException, CameraException,
            com.irurueta.geometry.estimators.LockedException, 
            com.irurueta.geometry.estimators.NotReadyException,
            InitialCamerasEstimationFailedException, com.irurueta.geometry.NotAvailableException,
            InvalidFundamentalMatrixException, AlgebraException {
        int numValidTimes = 0;
        for (int t = 0; t < TIMES; t++) {
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
        
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            double skewness = 0.0;
            double principalPoint = 0.0;
        
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
        
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
        
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, 
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, 
                    betaEuler2, gammaEuler2);
        
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, 
                            focalLength, principalPoint, principalPoint, 
                            skewness);
        
            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
        
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, 
                    camera2);
        
            //create 3D points laying in front of both cameras
        
            //1st find an approximate central point by intersecting the axis 
            //planes of both cameras
            Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());
        
            planesIntersectionMatrix.setElementAt(1, 0, 
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, 
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, 
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, 
                    horizontalPlane1.getD());
        
            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());
        
            planesIntersectionMatrix.setElementAt(3, 0, 
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, 
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, 
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, 
                    horizontalPlane2.getD());
        
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            Matrix v = decomposer.getV();
            HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));
        
            double[] principalAxis1 = camera1.getPrincipalAxisArray();
            double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1, lambda2;
        
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
            InhomogeneousPoint3D worldPoint;
            List<InhomogeneousPoint3D> worldPoints = 
                    new ArrayList<>();
            Point2D leftPoint, rightPoint;
            List<Point2D> leftPoints = new ArrayList<>();
            List<Point2D> rightPoints = new ArrayList<>();
            boolean leftFront, rightFront;
            for (int i = 0; i < nPoints; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambda1 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);
                    lambda2 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);        
            
                    worldPoint = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() +
                            principalAxis1[0] * lambda1 +
                            principalAxis2[0] * lambda2, 
                            centralCommonPoint.getInhomY() +
                            principalAxis1[1] * lambda1 +
                            principalAxis2[1] * lambda2,
                            centralCommonPoint.getInhomZ() +
                            principalAxis1[2] * lambda1 +
                            principalAxis2[2] * lambda2);
                    leftFront = camera1.isPointInFrontOfCamera(worldPoint);
                    rightFront = camera2.isPointInFrontOfCamera(worldPoint);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!leftFront || !rightFront);
                worldPoints.add(worldPoint);
                
                //check that world point is in front of both cameras
                //noinspection all
                assertTrue(leftFront);
                //noinspection all
                assertTrue(rightFront);
            
                //projct world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);
            
                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }
        
            EssentialMatrixInitialCamerasEstimator estimator = 
                    new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, intrinsic, intrinsic, leftPoints, 
                    rightPoints, this);
            estimator.setPointsTriangulated(true);
            estimator.setValidTriangulatedPointsMarked(true);
            
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedLeftCamera());
            assertNull(estimator.getEstimatedRightCamera());
            assertNull(estimator.getTriangulatedPoints());
            assertNull(estimator.getValidTriangulatedPoints());
            
            estimator.estimate();
            
            PinholeCamera camera1b = estimator.getEstimatedLeftCamera();
            PinholeCamera camera2b = estimator.getEstimatedRightCamera();
            List<Point3D> triangulatedPoints = 
                    estimator.getTriangulatedPoints();
            BitSet validTriangulatedPoints = 
                    estimator.getValidTriangulatedPoints();
        
            camera1b.decompose();
            camera2b.decompose();
        
            PinholeCameraIntrinsicParameters intrinsic1b = 
                    camera1b.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters intrinsic2b =
                    camera2b.getIntrinsicParameters();
        
            Rotation3D rotation1b = camera1b.getCameraRotation();
            Rotation3D rotation2b = camera2b.getCameraRotation();
        
            Point3D center1b = camera1b.getCameraCenter();
            Point3D center2b = camera2b.getCameraCenter();
        
            assertEquals(intrinsic1b.getHorizontalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getVerticalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getSkewness(), 0.0, ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getHorizontalPrincipalPoint(), 0.0, 
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getVerticalPrincipalPoint(), 0.0, 
                    ABSOLUTE_ERROR);
        
            assertEquals(intrinsic2b.getHorizontalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getVerticalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getSkewness(), 0.0, ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getHorizontalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getVerticalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);  
        
            Rotation3D diffRotation = rotation2b.combineAndReturnNew(
                    rotation1b.inverseRotationAndReturnNew());
        
            assertTrue(rotation1b.asInhomogeneousMatrix().equals(
                    Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(
                    diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
        
            assertNotNull(center1b);
            assertNotNull(center2b);
        
            //compute scale factor
            double distanceA = center1.distanceTo(center2);        
            double distanceB = center1b.distanceTo(center2b);
            double scaleFactor = distanceB / distanceA;
            double invScaleFactor = distanceA / distanceB;
        
            //NOTE: distance between estimated cameras is always normalized
            assertEquals(distanceB, 1.0, ABSOLUTE_ERROR);
        
            MetricTransformation3D scaleTransformation = 
                    new MetricTransformation3D(scaleFactor);
            Transformation3D invScaleTransformation = 
                    scaleTransformation.inverseAndReturnNew();
            MetricTransformation3D invScaleTransformation2 =
                    new MetricTransformation3D(invScaleFactor);
        
            assertTrue(invScaleTransformation.asMatrix().equals(
                    invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));
        
        
            //check that estimated cameras generate the same input fundamental
            //matrix
            FundamentalMatrix fundamentalMatrixB = new FundamentalMatrix(
                    camera1b, camera2b);        
        
            //compare fundamental matrices by checking generated epipolar 
            //geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();
        
            Point2D epipole1 = camera1.project(center2);
            Point2D epipole2 = camera2.project(center1);
        
            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();
        
            Point2D epipole1a = fundamentalMatrix.getLeftEpipole();
            Point2D epipole2a = fundamentalMatrix.getRightEpipole();
        
            Point2D epipole1b = fundamentalMatrixB.getLeftEpipole();
            Point2D epipole2b = fundamentalMatrixB.getRightEpipole();
        
            assertEquals(epipole1.distanceTo(epipole1a), 0.0, ABSOLUTE_ERROR);
            assertEquals(epipole2.distanceTo(epipole2a), 0.0, ABSOLUTE_ERROR);
        
            assertEquals(epipole1.distanceTo(epipole1b), 0.0, ABSOLUTE_ERROR);
            assertEquals(epipole2.distanceTo(epipole2b), 0.0, 
                    LARGE_ABSOLUTE_ERROR);
        
            //generate epipolar lines
            Point3D scaledWorldPoint, triangulatedPoint, 
                    scaledTriangulatedPoint;            
            int numValid1a = 0, numValid2a = 0;
            int numValid1b = 0, numValid2b = 0;
            int numValidEqual = 0;
            for (int i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);
            
                triangulatedPoint = triangulatedPoints.get(i);
                assertTrue(validTriangulatedPoints.get(i));
            
                Line2D line1a = fundamentalMatrix.getLeftEpipolarLine(
                        rightPoint);
                Line2D line2a = fundamentalMatrix.getRightEpipolarLine(
                        leftPoint);
            
                Line2D line1b = fundamentalMatrixB.getLeftEpipolarLine(
                        rightPoint);
                Line2D line2b = fundamentalMatrixB.getRightEpipolarLine(
                        leftPoint);
            
                //check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));
            
                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));
            
                //backproject epipolar lines for each pair of cameras and check 
                //that each pair of lines correspond to the same epipolar plane
                Plane epipolarPlane1a = camera1.backProject(line1a);
                Plane epipolarPlane2a = camera2.backProject(line2a);
        
                Plane epipolarPlane1b = camera1b.backProject(line1b);
                Plane epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, 
                        ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, 
                        ABSOLUTE_ERROR));

                //check that 3D point and both camera centers for each pair of 
                //cameras belong to their corresponding epipolar plane
                if (epipolarPlane1a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid1a++;
                }
                assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));
  
                if (epipolarPlane2a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid2a++;
                }
                assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

                //notice that since estimated cameras have an arbitrary scale, 
                //original world point doesn't need to lie on epipolar plane 
                //because first a scale transformation needs to be done    
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(
                        worldPoint);
                if (epipolarPlane1a.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid1b++;
                }
                assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));
        
                if (epipolarPlane2b.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid2b++;
                }
                assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR)); 
            
                //recover scale in triangulated point
                scaledTriangulatedPoint = invScaleTransformation.
                        transformAndReturnNew(triangulatedPoint);
            
                //check that triangulated point after recovering scale matches 
                //original point
                if (worldPoint.equals(scaledTriangulatedPoint, 
                        LARGE_ABSOLUTE_ERROR)) {
                    numValidEqual++;
                }
            }
            
            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 && 
                    numValid2b > 0 && numValidEqual > 0) {
                numValidTimes++;
            }
        
            //recover scale of cameras by undoing their transformations
            PinholeCamera camera1c = invScaleTransformation.
                    transformAndReturnNew(camera1b);
            PinholeCamera camera2c = invScaleTransformation.
                    transformAndReturnNew(camera2b);
        
            //check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();
        
            Matrix camera1Matrix = camera1.getInternalMatrix();
            Matrix camera1cMatrix = camera1c.getInternalMatrix();
            assertTrue(camera1Matrix.equals(camera1cMatrix, ABSOLUTE_ERROR));
        
            Matrix camera2Matrix = camera2.getInternalMatrix();
            Matrix camera2cMatrix = camera2c.getInternalMatrix();
            assertTrue(camera2Matrix.equals(camera2cMatrix, ABSOLUTE_ERROR));
            
            //force NotReadyException
            estimator = new EssentialMatrixInitialCamerasEstimator();
            
            assertFalse(estimator.isReady());
            
            try {
                estimator.estimate();
                fail("NotReadyException expected but not thrown");
            } catch (com.irurueta.geometry.estimators.NotReadyException ignore) { }
        }
        
        assertTrue(numValidTimes > 0);        
    }

    @Test
    public void testGenerateInitialMetricCamerasFromEssentialMatrix1() 
            throws InvalidPairOfCamerasException, CameraException,
            InitialCamerasEstimationFailedException, 
            com.irurueta.geometry.NotAvailableException, 
            com.irurueta.geometry.estimators.NotReadyException, 
            InvalidFundamentalMatrixException, AlgebraException {
        int numValidTimes = 0;
        for (int t = 0; t < TIMES; t++) {
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
        
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            double skewness = 0.0;
            double principalPoint = 0.0;
        
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
        
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
        
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, 
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, 
                    betaEuler2, gammaEuler2);
        
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, 
                            focalLength, principalPoint, principalPoint, 
                            skewness);
        
            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
        
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, 
                    camera2);
        
            //create 3D points laying in front of both cameras
        
            //1st find an approximate central point by intersecting the axis 
            //planes of both cameras
            Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());
        
            planesIntersectionMatrix.setElementAt(1, 0, 
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, 
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, 
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, 
                    horizontalPlane1.getD());
        
            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());
        
            planesIntersectionMatrix.setElementAt(3, 0, 
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, 
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, 
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, 
                    horizontalPlane2.getD());
        
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            Matrix v = decomposer.getV();
            HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));
        
            double[] principalAxis1 = camera1.getPrincipalAxisArray();
            double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1, lambda2;
        
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
            InhomogeneousPoint3D worldPoint;
            List<InhomogeneousPoint3D> worldPoints = 
                    new ArrayList<>();
            Point2D leftPoint, rightPoint;
            List<Point2D> leftPoints = new ArrayList<>();
            List<Point2D> rightPoints = new ArrayList<>();
            boolean leftFront, rightFront;            
            for (int i = 0; i < nPoints; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambda1 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);
                    lambda2 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);        
            
                    worldPoint = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() +
                            principalAxis1[0] * lambda1 +
                            principalAxis2[0] * lambda2, 
                            centralCommonPoint.getInhomY() +
                            principalAxis1[1] * lambda1 +
                            principalAxis2[1] * lambda2,
                            centralCommonPoint.getInhomZ() +
                            principalAxis1[2] * lambda1 +
                            principalAxis2[2] * lambda2);
                    leftFront = camera1.isPointInFrontOfCamera(worldPoint);
                    rightFront = camera2.isPointInFrontOfCamera(worldPoint);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!leftFront || !rightFront);
                worldPoints.add(worldPoint);
            
                //check that world point is in front of both cameras
                //noinspection all
                assertTrue(leftFront);
                //noinspection all
                assertTrue(rightFront);
            
                //projct world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);
            
                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }
        
            PinholeCamera camera1b = new PinholeCamera();
            PinholeCamera camera2b = new PinholeCamera();
            int numValid = EssentialMatrixInitialCamerasEstimator.
                    generateInitialMetricCamerasFromEssentialMatrix(
                    fundamentalMatrix, intrinsic, intrinsic, leftPoints, 
                    rightPoints, camera1b, camera2b);
        
            //check correctness
            assertEquals(numValid, nPoints);
        
            camera1b.decompose();
            camera2b.decompose();
        
            PinholeCameraIntrinsicParameters intrinsic1b = 
                    camera1b.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters intrinsic2b =
                    camera2b.getIntrinsicParameters();
        
            Rotation3D rotation1b = camera1b.getCameraRotation();
            Rotation3D rotation2b = camera2b.getCameraRotation();
        
            Point3D center1b = camera1b.getCameraCenter();
            Point3D center2b = camera2b.getCameraCenter();
        
            assertEquals(intrinsic1b.getHorizontalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getVerticalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getSkewness(), 0.0, ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getHorizontalPrincipalPoint(), 0.0, 
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getVerticalPrincipalPoint(), 0.0, 
                    ABSOLUTE_ERROR);
        
            assertEquals(intrinsic2b.getHorizontalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getVerticalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getSkewness(), 0.0, ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getHorizontalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getVerticalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);  
        
            Rotation3D diffRotation = rotation2b.combineAndReturnNew(
                    rotation1b.inverseRotationAndReturnNew());
        
            assertTrue(rotation1b.asInhomogeneousMatrix().equals(
                    Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(
                    diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
        
            assertNotNull(center1b);
            assertNotNull(center2b);
        
            //compute scale factor
            double distanceA = center1.distanceTo(center2);        
            double distanceB = center1b.distanceTo(center2b);
            double scaleFactor = distanceB / distanceA;
            double invScaleFactor = distanceA / distanceB;
        
            //NOTE: distance between estimated cameras is always normalized
            assertEquals(distanceB, 1.0, ABSOLUTE_ERROR);
        
            MetricTransformation3D scaleTransformation = 
                    new MetricTransformation3D(scaleFactor);
            Transformation3D invScaleTransformation = 
                    scaleTransformation.inverseAndReturnNew();
            MetricTransformation3D invScaleTransformation2 =
                    new MetricTransformation3D(invScaleFactor);
        
            assertTrue(invScaleTransformation.asMatrix().equals(
                    invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));
        
        
            //check that estimated cameras generate the same input fundamental
            //matrix
            FundamentalMatrix fundamentalMatrixB = new FundamentalMatrix(
                    camera1b, camera2b);        
        
            //compare fundamental matrices by checking generated epipolar 
            //geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();
        
            Point2D epipole1 = camera1.project(center2);
            Point2D epipole2 = camera2.project(center1);
        
            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();
        
            Point2D epipole1a = fundamentalMatrix.getLeftEpipole();
            Point2D epipole2a = fundamentalMatrix.getRightEpipole();
        
            Point2D epipole1b = fundamentalMatrixB.getLeftEpipole();
            Point2D epipole2b = fundamentalMatrixB.getRightEpipole();
        
            assertEquals(epipole1.distanceTo(epipole1a), 0.0, ABSOLUTE_ERROR);
            assertEquals(epipole2.distanceTo(epipole2a), 0.0, ABSOLUTE_ERROR);
        
            assertEquals(epipole1.distanceTo(epipole1b), 0.0, ABSOLUTE_ERROR);
            assertEquals(epipole2.distanceTo(epipole2b), 0.0, 
                    LARGE_ABSOLUTE_ERROR);
        
            //generate epipolar lines
            Point3D scaledWorldPoint;
            int numValid1a = 0, numValid2a = 0;
            int numValid1b = 0, numValid2b = 0;            
            for (int i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);
            
                Line2D line1a = fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                Line2D line2a = fundamentalMatrix.getRightEpipolarLine(leftPoint);
            
                Line2D line1b = fundamentalMatrixB.getLeftEpipolarLine(rightPoint);
                Line2D line2b = fundamentalMatrixB.getRightEpipolarLine(leftPoint);
            
                //check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));
            
                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));
            
                //backproject epipolar lines for each pair of cameras and check 
                //that each pair of lines correspond to the same epipolar plane
                Plane epipolarPlane1a = camera1.backProject(line1a);
                Plane epipolarPlane2a = camera2.backProject(line2a);
        
                Plane epipolarPlane1b = camera1b.backProject(line1b);
                Plane epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, 
                        ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, 
                        ABSOLUTE_ERROR));

                //check that 3D point and both camera centers for each pair of 
                //cameras belong to their corresponding epipolar plane
                if (epipolarPlane1a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid1a++;
                }
                assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));
        
                if (epipolarPlane2a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid2a++;
                }
                assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

                //notice that since estimated cameras have an arbitrary scale, 
                //original world point doesn't need to lie on epipolar plane 
                //because first a scale transformation needs to be done    
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(
                        worldPoint);
                if (epipolarPlane1a.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid1b++;
                }
                assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));
        
                if (epipolarPlane2b.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid2b++;
                }
                assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR));                            
            }
        
            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 && 
                    numValid2b > 0) {
                numValidTimes++;
            }
            
            //recover scale of cameras by undoing their transformations
            PinholeCamera camera1c = invScaleTransformation.
                    transformAndReturnNew(camera1b);
            PinholeCamera camera2c = invScaleTransformation.
                    transformAndReturnNew(camera2b);
        
            //check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();
        
            Matrix camera1Matrix = camera1.getInternalMatrix();
            Matrix camera1cMatrix = camera1c.getInternalMatrix();
            assertTrue(camera1Matrix.equals(camera1cMatrix, ABSOLUTE_ERROR));
        
            Matrix camera2Matrix = camera2.getInternalMatrix();
            Matrix camera2cMatrix = camera2c.getInternalMatrix();
            assertTrue(camera2Matrix.equals(camera2cMatrix, ABSOLUTE_ERROR));
        }
        
        assertTrue(numValidTimes > 0);
    }    
    
    @Test
    public void testGenerateInitialMetricCamerasFromEssentialMatrix2() 
            throws InvalidPairOfCamerasException, CameraException,
            InitialCamerasEstimationFailedException, 
            com.irurueta.geometry.NotAvailableException, 
            com.irurueta.geometry.estimators.NotReadyException, 
            InvalidFundamentalMatrixException, AlgebraException {
        int numValidTimes = 0;
        for (int t = 0; t < TIMES; t++) {
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
        
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            double skewness = 0.0;
            double principalPoint = 0.0;
        
            double cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION,
                    MAX_CAMERA_SEPARATION);
        
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
        
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, 
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, 
                    betaEuler2, gammaEuler2);
        
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, 
                            focalLength, principalPoint, principalPoint, 
                            skewness);
        
            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
        
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, 
                    camera2);
        
            //create 3D points laying in front of both cameras
        
            //1st find an approximate central point by intersecting the axis 
            //planes of both cameras
            Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            Matrix planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, 
                    Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());
        
            planesIntersectionMatrix.setElementAt(1, 0, 
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, 
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, 
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, 
                    horizontalPlane1.getD());
        
            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());
        
            planesIntersectionMatrix.setElementAt(3, 0, 
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, 
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, 
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, 
                    horizontalPlane2.getD());
        
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            Matrix v = decomposer.getV();
            HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));
        
            double[] principalAxis1 = camera1.getPrincipalAxisArray();
            double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1, lambda2;
        
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
            InhomogeneousPoint3D worldPoint;
            List<InhomogeneousPoint3D> worldPoints = 
                    new ArrayList<>();
            Point2D leftPoint, rightPoint;
            List<Point2D> leftPoints = new ArrayList<>();
            List<Point2D> rightPoints = new ArrayList<>();
            boolean leftFront, rightFront;            
            for (int i = 0; i < nPoints; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambda1 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);
                    lambda2 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);        
            
                    worldPoint = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() +
                            principalAxis1[0] * lambda1 +
                            principalAxis2[0] * lambda2, 
                            centralCommonPoint.getInhomY() +
                            principalAxis1[1] * lambda1 +
                            principalAxis2[1] * lambda2,
                            centralCommonPoint.getInhomZ() +
                            principalAxis1[2] * lambda1 +
                            principalAxis2[2] * lambda2);                    
                    leftFront = camera1.isPointInFrontOfCamera(worldPoint);
                    rightFront = camera2.isPointInFrontOfCamera(worldPoint);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!leftFront || !rightFront);
                worldPoints.add(worldPoint);
            
                //check that world point is in front of both cameras
                //noinspection all
                assertTrue(leftFront);
                //noinspection all
                assertTrue(rightFront);
            
                //projct world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);
            
                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }
        
            PinholeCamera camera1b = new PinholeCamera();
            PinholeCamera camera2b = new PinholeCamera();
            int numValid = EssentialMatrixInitialCamerasEstimator.
                    generateInitialMetricCamerasFromEssentialMatrix(
                    fundamentalMatrix, intrinsic, intrinsic, leftPoints, 
                    rightPoints, null, camera1b, camera2b);
        
            //check correctness
            assertEquals(numValid, nPoints);
        
            camera1b.decompose();
            camera2b.decompose();
        
            PinholeCameraIntrinsicParameters intrinsic1b = 
                    camera1b.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters intrinsic2b =
                    camera2b.getIntrinsicParameters();
        
            Rotation3D rotation1b = camera1b.getCameraRotation();
            Rotation3D rotation2b = camera2b.getCameraRotation();
        
            Point3D center1b = camera1b.getCameraCenter();
            Point3D center2b = camera2b.getCameraCenter();
        
            assertEquals(intrinsic1b.getHorizontalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getVerticalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getSkewness(), 0.0, ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getHorizontalPrincipalPoint(), 0.0, 
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getVerticalPrincipalPoint(), 0.0, 
                    ABSOLUTE_ERROR);
        
            assertEquals(intrinsic2b.getHorizontalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getVerticalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getSkewness(), 0.0, ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getHorizontalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getVerticalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);  
        
            Rotation3D diffRotation = rotation2b.combineAndReturnNew(
                    rotation1b.inverseRotationAndReturnNew());
        
            assertTrue(rotation1b.asInhomogeneousMatrix().equals(
                    Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(
                    diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
        
            assertNotNull(center1b);
            assertNotNull(center2b);
        
            //compute scale factor
            double distanceA = center1.distanceTo(center2);        
            double distanceB = center1b.distanceTo(center2b);
            double scaleFactor = distanceB / distanceA;
            double invScaleFactor = distanceA / distanceB;
        
            //NOTE: distance between estimated cameras is always normalized
            assertEquals(distanceB, 1.0, ABSOLUTE_ERROR);
        
            MetricTransformation3D scaleTransformation = 
                    new MetricTransformation3D(scaleFactor);
            Transformation3D invScaleTransformation = 
                    scaleTransformation.inverseAndReturnNew();
            MetricTransformation3D invScaleTransformation2 =
                    new MetricTransformation3D(invScaleFactor);
        
            assertTrue(invScaleTransformation.asMatrix().equals(
                    invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));
        
        
            //check that estimated cameras generate the same input fundamental
            //matrix
            FundamentalMatrix fundamentalMatrixB = new FundamentalMatrix(
                    camera1b, camera2b);        
        
            //compare fundamental matrices by checking generated epipolar 
            //geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();
        
            Point2D epipole1 = camera1.project(center2);
            Point2D epipole2 = camera2.project(center1);
        
            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();
        
            Point2D epipole1a = fundamentalMatrix.getLeftEpipole();
            Point2D epipole2a = fundamentalMatrix.getRightEpipole();
        
            Point2D epipole1b = fundamentalMatrixB.getLeftEpipole();
            Point2D epipole2b = fundamentalMatrixB.getRightEpipole();
        
            assertEquals(epipole1.distanceTo(epipole1a), 0.0, ABSOLUTE_ERROR);
            assertEquals(epipole2.distanceTo(epipole2a), 0.0, ABSOLUTE_ERROR);
        
            assertEquals(epipole1.distanceTo(epipole1b), 0.0, ABSOLUTE_ERROR);
            assertEquals(epipole2.distanceTo(epipole2b), 0.0, 
                    LARGE_ABSOLUTE_ERROR);
        
            //generate epipolar lines
            Point3D scaledWorldPoint;
            int numValid1a = 0, numValid2a = 0;
            int numValid1b = 0, numValid2b = 0;      
            for (int i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);
            
                Line2D line1a = fundamentalMatrix.getLeftEpipolarLine(
                        rightPoint);
                Line2D line2a = fundamentalMatrix.getRightEpipolarLine(
                        leftPoint);
            
                Line2D line1b = fundamentalMatrixB.getLeftEpipolarLine(
                        rightPoint);
                Line2D line2b = fundamentalMatrixB.getRightEpipolarLine(
                        leftPoint);
            
                //check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));
            
                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));
            
                //backproject epipolar lines for each pair of cameras and check 
                //that each pair of lines correspond to the same epipolar plane
                Plane epipolarPlane1a = camera1.backProject(line1a);
                Plane epipolarPlane2a = camera2.backProject(line2a);
        
                Plane epipolarPlane1b = camera1b.backProject(line1b);
                Plane epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, 
                        ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, 
                        ABSOLUTE_ERROR));

                //check that 3D point and both camera centers for each pair of 
                //cameras belong to their corresponding epipolar plane
                if (epipolarPlane1a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid1a++;
                }
                assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));
        
                if (epipolarPlane2a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid2a++;
                }
                assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

                //notice that since estimated cameras have an arbitrary scale, 
                //original world point doesn't need to lie on epipolar plane 
                //because first a scale transformation needs to be done    
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(
                        worldPoint);
                if (epipolarPlane1a.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid1b++;
                }
                assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));
        
                if (epipolarPlane2b.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid2b++;
                }
                assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR));                
            }
        
            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 && 
                    numValid2b > 0) {
                numValidTimes++;
            }            
            
            //recover scale of cameras by undoing their transformations
            PinholeCamera camera1c = invScaleTransformation.
                    transformAndReturnNew(camera1b);
            PinholeCamera camera2c = invScaleTransformation.
                    transformAndReturnNew(camera2b);
        
            //check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();
        
            Matrix camera1Matrix = camera1.getInternalMatrix();
            Matrix camera1cMatrix = camera1c.getInternalMatrix();
            assertTrue(camera1Matrix.equals(camera1cMatrix, ABSOLUTE_ERROR));
        
            Matrix camera2Matrix = camera2.getInternalMatrix();
            Matrix camera2cMatrix = camera2c.getInternalMatrix();
            assertTrue(camera2Matrix.equals(camera2cMatrix, ABSOLUTE_ERROR));
        }
        
        assertTrue(numValidTimes > 0);
    }    
    
    @Test
    public void testGenerateInitialMetricCamerasFromEssentialMatrix3() 
            throws InvalidPairOfCamerasException, CameraException,
            InitialCamerasEstimationFailedException, 
            com.irurueta.geometry.NotAvailableException, 
            com.irurueta.geometry.estimators.NotReadyException, 
            InvalidFundamentalMatrixException, AlgebraException {
        int numValidTimes = 0;
        for(int t = 0; t < TIMES; t++) {
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
        
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            double skewness = 0.0;
            double principalPoint = 0.0;
        
            double cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION,
                    MAX_CAMERA_SEPARATION);
        
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
        
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, 
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, 
                    betaEuler2, gammaEuler2);
        
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, 
                            focalLength, principalPoint, principalPoint, 
                            skewness);
        
            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
        
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, 
                    camera2);
        
            //create 3D points laying in front of both cameras
        
            //1st find an approximate central point by intersecting the axis 
            //planes of both cameras
            Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            Matrix planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, 
                    Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());
        
            planesIntersectionMatrix.setElementAt(1, 0, 
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, 
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, 
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, 
                    horizontalPlane1.getD());
        
            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());
        
            planesIntersectionMatrix.setElementAt(3, 0, 
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, 
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, 
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, 
                    horizontalPlane2.getD());
        
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            Matrix v = decomposer.getV();
            HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));
        
            double[] principalAxis1 = camera1.getPrincipalAxisArray();
            double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1, lambda2;
        
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
            InhomogeneousPoint3D worldPoint;
            List<InhomogeneousPoint3D> worldPoints = 
                    new ArrayList<>();
            Point2D leftPoint, rightPoint;
            List<Point2D> leftPoints = new ArrayList<>();
            List<Point2D> rightPoints = new ArrayList<>();
            boolean leftFront, rightFront;            
            for (int i = 0; i < nPoints; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambda1 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);
                    lambda2 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);        
            
                    worldPoint = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() +
                            principalAxis1[0] * lambda1 +
                            principalAxis2[0] * lambda2, 
                            centralCommonPoint.getInhomY() +
                            principalAxis1[1] * lambda1 +
                            principalAxis2[1] * lambda2,
                            centralCommonPoint.getInhomZ() +
                            principalAxis1[2] * lambda1 +
                            principalAxis2[2] * lambda2);                    
                    leftFront = camera1.isPointInFrontOfCamera(worldPoint);
                    rightFront = camera2.isPointInFrontOfCamera(worldPoint);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!leftFront || !rightFront);
                worldPoints.add(worldPoint);
            
                //check that world point is in front of both cameras
                //noinspection all
                assertTrue(leftFront);
                //noinspection all
                assertTrue(rightFront);
            
                //project world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);
            
                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }
        
            PinholeCamera camera1b = new PinholeCamera();
            PinholeCamera camera2b = new PinholeCamera();
            List<Point3D> triangulatedPoints = new ArrayList<>();
            BitSet validTriangulatedPoints = new BitSet(nPoints);
            int numValid = EssentialMatrixInitialCamerasEstimator.
                    generateInitialMetricCamerasFromEssentialMatrix(
                    fundamentalMatrix, intrinsic, intrinsic, leftPoints, 
                    rightPoints, camera1b, camera2b, triangulatedPoints,
                    validTriangulatedPoints);
        
            //check correctness
            assertEquals(numValid, nPoints);
        
            camera1b.decompose();
            camera2b.decompose();
        
            PinholeCameraIntrinsicParameters intrinsic1b = 
                    camera1b.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters intrinsic2b =
                    camera2b.getIntrinsicParameters();
        
            Rotation3D rotation1b = camera1b.getCameraRotation();
            Rotation3D rotation2b = camera2b.getCameraRotation();
        
            Point3D center1b = camera1b.getCameraCenter();
            Point3D center2b = camera2b.getCameraCenter();
        
            assertEquals(intrinsic1b.getHorizontalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getVerticalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getSkewness(), 0.0, ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getHorizontalPrincipalPoint(), 0.0, 
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getVerticalPrincipalPoint(), 0.0, 
                    ABSOLUTE_ERROR);
        
            assertEquals(intrinsic2b.getHorizontalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getVerticalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getSkewness(), 0.0, ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getHorizontalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getVerticalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);  
        
            Rotation3D diffRotation = rotation2b.combineAndReturnNew(
                    rotation1b.inverseRotationAndReturnNew());
        
            assertTrue(rotation1b.asInhomogeneousMatrix().equals(
                    Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(
                    diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
        
            assertNotNull(center1b);
            assertNotNull(center2b);
        
            //compute scale factor
            double distanceA = center1.distanceTo(center2);        
            double distanceB = center1b.distanceTo(center2b);
            double scaleFactor = distanceB / distanceA;
            double invScaleFactor = distanceA / distanceB;
        
            //NOTE: distance between estimated cameras is always normalized
            assertEquals(distanceB, 1.0, ABSOLUTE_ERROR);
        
            MetricTransformation3D scaleTransformation = 
                    new MetricTransformation3D(scaleFactor);
            Transformation3D invScaleTransformation = 
                    scaleTransformation.inverseAndReturnNew();
            MetricTransformation3D invScaleTransformation2 =
                    new MetricTransformation3D(invScaleFactor);
        
            assertTrue(invScaleTransformation.asMatrix().equals(
                    invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));
        
        
            //check that estimated cameras generate the same input fundamental
            //matrix
            FundamentalMatrix fundamentalMatrixB = new FundamentalMatrix(
                    camera1b, camera2b);        
        
            //compare fundamental matrices by checking generated epipolar geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();
        
            Point2D epipole1 = camera1.project(center2);
            Point2D epipole2 = camera2.project(center1);
        
            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();
        
            Point2D epipole1a = fundamentalMatrix.getLeftEpipole();
            Point2D epipole2a = fundamentalMatrix.getRightEpipole();
        
            Point2D epipole1b = fundamentalMatrixB.getLeftEpipole();
            Point2D epipole2b = fundamentalMatrixB.getRightEpipole();
        
            assertEquals(epipole1.distanceTo(epipole1a), 0.0, ABSOLUTE_ERROR);
            assertEquals(epipole2.distanceTo(epipole2a), 0.0, ABSOLUTE_ERROR);
        
            assertEquals(epipole1.distanceTo(epipole1b), 0.0, ABSOLUTE_ERROR);
            assertEquals(epipole2.distanceTo(epipole2b), 0.0, 
                    LARGE_ABSOLUTE_ERROR);
        
            //generate epipolar lines
            Point3D scaledWorldPoint, triangulatedPoint, 
                    scaledTriangulatedPoint;
            int numValid1a = 0, numValid2a = 0;
            int numValid1b = 0, numValid2b = 0;
            int numValidEqual = 0;
            for (int i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);
            
                triangulatedPoint = triangulatedPoints.get(i);
                assertTrue(validTriangulatedPoints.get(i));
            
                Line2D line1a = fundamentalMatrix.getLeftEpipolarLine(
                        rightPoint);
                Line2D line2a = fundamentalMatrix.getRightEpipolarLine(
                        leftPoint);
            
                Line2D line1b = fundamentalMatrixB.getLeftEpipolarLine(
                        rightPoint);
                Line2D line2b = fundamentalMatrixB.getRightEpipolarLine(
                        leftPoint);
            
                //check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));
            
                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));
            
                //backproject epipolar lines for each pair of cameras and check 
                //that each pair of lines correspond to the same epipolar plane
                Plane epipolarPlane1a = camera1.backProject(line1a);
                Plane epipolarPlane2a = camera2.backProject(line2a);
        
                Plane epipolarPlane1b = camera1b.backProject(line1b);
                Plane epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, 
                        ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, 
                        ABSOLUTE_ERROR));

                //check that 3D point and both camera centers for each pair of 
                //cameras belong to their corresponding epipolar plane
                if (epipolarPlane1a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid1a++;
                }
                assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));
        
                if (epipolarPlane2a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid2a++;
                }
                assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

                //notice that since estimated cameras have an arbitrary scale, 
                //original world point doesn't need to lie on epipolar plane 
                //because first a scale transformation needs to be done    
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(
                        worldPoint);
                if (epipolarPlane1a.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid1b++;
                }
                assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));
        
                if (epipolarPlane2b.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid2b++;
                }
                assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR)); 
            
                //recover scale in triangulated point
                scaledTriangulatedPoint = invScaleTransformation.
                        transformAndReturnNew(triangulatedPoint);
            
                //check that triangulated point after recovering scale matches 
                //original point
                if (worldPoint.equals(scaledTriangulatedPoint, 
                        LARGE_ABSOLUTE_ERROR)) {
                    numValidEqual++;
                }
            }
            
            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 &&
                    numValid2b > 0 && numValidEqual > 0) {
                numValidTimes++;
            }
        
            //recover scale of cameras by undoing their transformations
            PinholeCamera camera1c = invScaleTransformation.
                    transformAndReturnNew(camera1b);
            PinholeCamera camera2c = invScaleTransformation.
                    transformAndReturnNew(camera2b);
        
            //check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();
        
            Matrix camera1Matrix = camera1.getInternalMatrix();
            Matrix camera1cMatrix = camera1c.getInternalMatrix();
            assertTrue(camera1Matrix.equals(camera1cMatrix, ABSOLUTE_ERROR));
        
            Matrix camera2Matrix = camera2.getInternalMatrix();
            Matrix camera2cMatrix = camera2c.getInternalMatrix();
            assertTrue(camera2Matrix.equals(camera2cMatrix, ABSOLUTE_ERROR));
        }
        
        assertTrue(numValidTimes > 0);
    }    

    @Test
    public void testGenerateInitialMetricCamerasFromEssentialMatrix4() 
            throws InvalidPairOfCamerasException, CameraException,
            InitialCamerasEstimationFailedException, 
            com.irurueta.geometry.NotAvailableException, 
            com.irurueta.geometry.estimators.NotReadyException, 
            InvalidFundamentalMatrixException, AlgebraException {
        int numValidTimes = 0;
        for (int t = 0; t < TIMES; t++) {
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
        
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            double skewness = 0.0;
            double principalPoint = 0.0;
        
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
        
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
        
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, 
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, 
                    betaEuler2, gammaEuler2);
        
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, 
                            focalLength, principalPoint, principalPoint, 
                            skewness);
        
            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
        
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, 
                    camera2);
        
            //create 3D points laying in front of both cameras
        
            //1st find an approximate central point by intersecting the axis 
            //planes of both cameras
            Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());
        
            planesIntersectionMatrix.setElementAt(1, 0, 
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, 
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, 
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, 
                    horizontalPlane1.getD());
        
            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());
        
            planesIntersectionMatrix.setElementAt(3, 0, 
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, 
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, 
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, 
                    horizontalPlane2.getD());
        
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            Matrix v = decomposer.getV();
            HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));
        
            double[] principalAxis1 = camera1.getPrincipalAxisArray();
            double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1, lambda2;
        
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
            InhomogeneousPoint3D worldPoint;
            List<InhomogeneousPoint3D> worldPoints = 
                    new ArrayList<>();
            Point2D leftPoint, rightPoint;
            List<Point2D> leftPoints = new ArrayList<>();
            List<Point2D> rightPoints = new ArrayList<>();
            boolean leftFront, rightFront;
            for (int i = 0; i < nPoints; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambda1 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);
                    lambda2 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);        
            
                    worldPoint = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() +
                            principalAxis1[0] * lambda1 +
                            principalAxis2[0] * lambda2, 
                            centralCommonPoint.getInhomY() +
                            principalAxis1[1] * lambda1 +
                            principalAxis2[1] * lambda2,
                            centralCommonPoint.getInhomZ() +
                            principalAxis1[2] * lambda1 +
                            principalAxis2[2] * lambda2);
                    leftFront = camera1.isPointInFrontOfCamera(worldPoint);
                    rightFront = camera2.isPointInFrontOfCamera(worldPoint);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!leftFront || !rightFront);
                worldPoints.add(worldPoint);
                
                //check that world point is in front of both cameras
                //noinspection all
                assertTrue(leftFront);
                //noinspection all
                assertTrue(rightFront);
            
                //project world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);
            
                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }
        
            PinholeCamera camera1b = new PinholeCamera();
            PinholeCamera camera2b = new PinholeCamera();
            List<Point3D> triangulatedPoints = new ArrayList<>();
            BitSet validTriangulatedPoints = new BitSet(nPoints);
            int numValid = EssentialMatrixInitialCamerasEstimator.
                    generateInitialMetricCamerasFromEssentialMatrix(
                    fundamentalMatrix, intrinsic, intrinsic, leftPoints, 
                    rightPoints, CorrectorType.SAMPSON_CORRECTOR, camera1b, 
                    camera2b, triangulatedPoints, validTriangulatedPoints);
        
            //check correctness
            assertEquals(numValid, nPoints);
        
            camera1b.decompose();
            camera2b.decompose();
        
            PinholeCameraIntrinsicParameters intrinsic1b = 
                    camera1b.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters intrinsic2b =
                    camera2b.getIntrinsicParameters();
        
            Rotation3D rotation1b = camera1b.getCameraRotation();
            Rotation3D rotation2b = camera2b.getCameraRotation();
        
            Point3D center1b = camera1b.getCameraCenter();
            Point3D center2b = camera2b.getCameraCenter();
        
            assertEquals(intrinsic1b.getHorizontalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getVerticalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getSkewness(), 0.0, ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getHorizontalPrincipalPoint(), 0.0, 
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getVerticalPrincipalPoint(), 0.0, 
                    ABSOLUTE_ERROR);
        
            assertEquals(intrinsic2b.getHorizontalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getVerticalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getSkewness(), 0.0, ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getHorizontalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2b.getVerticalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);  
        
            Rotation3D diffRotation = rotation2b.combineAndReturnNew(
                    rotation1b.inverseRotationAndReturnNew());
        
            assertTrue(rotation1b.asInhomogeneousMatrix().equals(
                    Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(
                    diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
        
            assertNotNull(center1b);
            assertNotNull(center2b);
        
            //compute scale factor
            double distanceA = center1.distanceTo(center2);        
            double distanceB = center1b.distanceTo(center2b);
            double scaleFactor = distanceB / distanceA;
            double invScaleFactor = distanceA / distanceB;
        
            //NOTE: distance between estimated cameras is always normalized
            assertEquals(distanceB, 1.0, ABSOLUTE_ERROR);
        
            MetricTransformation3D scaleTransformation = 
                    new MetricTransformation3D(scaleFactor);
            Transformation3D invScaleTransformation = 
                    scaleTransformation.inverseAndReturnNew();
            MetricTransformation3D invScaleTransformation2 =
                    new MetricTransformation3D(invScaleFactor);
        
            assertTrue(invScaleTransformation.asMatrix().equals(
                    invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));
        
        
            //check that estimated cameras generate the same input fundamental
            //matrix
            FundamentalMatrix fundamentalMatrixB = new FundamentalMatrix(
                    camera1b, camera2b);        
        
            //compare fundamental matrices by checking generated epipolar 
            //geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();
        
            Point2D epipole1 = camera1.project(center2);
            Point2D epipole2 = camera2.project(center1);
        
            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();
        
            Point2D epipole1a = fundamentalMatrix.getLeftEpipole();
            Point2D epipole2a = fundamentalMatrix.getRightEpipole();
        
            Point2D epipole1b = fundamentalMatrixB.getLeftEpipole();
            Point2D epipole2b = fundamentalMatrixB.getRightEpipole();
        
            assertEquals(epipole1.distanceTo(epipole1a), 0.0, ABSOLUTE_ERROR);
            assertEquals(epipole2.distanceTo(epipole2a), 0.0, ABSOLUTE_ERROR);
        
            assertEquals(epipole1.distanceTo(epipole1b), 0.0, ABSOLUTE_ERROR);
            assertEquals(epipole2.distanceTo(epipole2b), 0.0, 
                    LARGE_ABSOLUTE_ERROR);
        
            //generate epipolar lines
            Point3D scaledWorldPoint, triangulatedPoint, 
                    scaledTriangulatedPoint;            
            int numValid1a = 0, numValid2a = 0;
            int numValid1b = 0, numValid2b = 0;
            int numValidEqual = 0;
            for (int i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);
            
                triangulatedPoint = triangulatedPoints.get(i);
                assertTrue(validTriangulatedPoints.get(i));
            
                Line2D line1a = fundamentalMatrix.getLeftEpipolarLine(
                        rightPoint);
                Line2D line2a = fundamentalMatrix.getRightEpipolarLine(
                        leftPoint);
            
                Line2D line1b = fundamentalMatrixB.getLeftEpipolarLine(
                        rightPoint);
                Line2D line2b = fundamentalMatrixB.getRightEpipolarLine(
                        leftPoint);
            
                //check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));
            
                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));
            
                //backproject epipolar lines for each pair of cameras and check 
                //that each pair of lines correspond to the same epipolar plane
                Plane epipolarPlane1a = camera1.backProject(line1a);
                Plane epipolarPlane2a = camera2.backProject(line2a);
        
                Plane epipolarPlane1b = camera1b.backProject(line1b);
                Plane epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, 
                        ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, 
                        ABSOLUTE_ERROR));

                //check that 3D point and both camera centers for each pair of 
                //cameras belong to their corresponding epipolar plane
                if (epipolarPlane1a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid1a++;
                }
                assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));
  
                if (epipolarPlane2a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid2a++;
                }
                assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

                //notice that since estimated cameras have an arbitrary scale, 
                //original world point doesn't need to lie on epipolar plane 
                //because first a scale transformation needs to be done    
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(
                        worldPoint);
                if (epipolarPlane1a.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid1b++;
                }
                assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));
        
                if (epipolarPlane2b.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid2b++;
                }
                assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR)); 
            
                //recover scale in triangulated point
                scaledTriangulatedPoint = invScaleTransformation.
                        transformAndReturnNew(triangulatedPoint);
            
                //check that triangulated point after recovering scale matches 
                //original point
                if (worldPoint.equals(scaledTriangulatedPoint, 
                        LARGE_ABSOLUTE_ERROR)) {
                    numValidEqual++;
                }
            }
            
            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 && 
                    numValid2b > 0 && numValidEqual > 0) {
                numValidTimes++;
            }
        
            //recover scale of cameras by undoing their transformations
            PinholeCamera camera1c = invScaleTransformation.
                    transformAndReturnNew(camera1b);
            PinholeCamera camera2c = invScaleTransformation.
                    transformAndReturnNew(camera2b);
        
            //check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();
        
            Matrix camera1Matrix = camera1.getInternalMatrix();
            Matrix camera1cMatrix = camera1c.getInternalMatrix();
            assertTrue(camera1Matrix.equals(camera1cMatrix, ABSOLUTE_ERROR));
        
            Matrix camera2Matrix = camera2.getInternalMatrix();
            Matrix camera2cMatrix = camera2c.getInternalMatrix();
            assertTrue(camera2Matrix.equals(camera2cMatrix, ABSOLUTE_ERROR));
        }
        
        assertTrue(numValidTimes > 0);
    }    

    @Override
    public void onStart(InitialCamerasEstimator estimator) {
        checkLocked((EssentialMatrixInitialCamerasEstimator)estimator);
    }

    @Override
    public void onFinish(InitialCamerasEstimator estimator, 
            PinholeCamera estimatedLeftCamera, 
            PinholeCamera estimatedRightCamera) {
        checkLocked((EssentialMatrixInitialCamerasEstimator)estimator);
    }

    @Override
    public void onFail(InitialCamerasEstimator estimator, 
            InitialCamerasEstimationFailedException e) {
        checkLocked((EssentialMatrixInitialCamerasEstimator)estimator);
    }
    
    private void checkLocked(EssentialMatrixInitialCamerasEstimator estimator) {
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (com.irurueta.geometry.estimators.LockedException ignore) {
        } catch (com.irurueta.geometry.estimators.NotReadyException |
                InitialCamerasEstimationFailedException ex) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setLeftIntrinsic(null);
            fail("LockedException expected but not thrown");
        } catch (com.irurueta.geometry.estimators.LockedException ignore) { }
        try {
            estimator.setRightIntrinsic(null);
            fail("LockedException expected but not thrown");
        } catch (com.irurueta.geometry.estimators.LockedException ignore) { }
        try {
            estimator.setLeftAndRightIntrinsics(null, null);
            fail("LockedException expected but not thrown");
        } catch (com.irurueta.geometry.estimators.LockedException ignore) { }
        try {
            estimator.setIntrinsicsForBoth(null);
            fail("LockedException expected but not thrown");
        } catch (com.irurueta.geometry.estimators.LockedException ignore) { }
        try {
            estimator.setLeftPoints(null);
            fail("LockedException expected but not thrown");
        } catch (com.irurueta.geometry.estimators.LockedException ignore) { }
        try {
            estimator.setRightPoints(null);
            fail("LockedException expected but not thrown");
        } catch (com.irurueta.geometry.estimators.LockedException ignore) { }
        try {
            estimator.setLeftAndRightPoints(null, null);
            fail("LockedException expected but not thrown");
        } catch (com.irurueta.geometry.estimators.LockedException ignore) { }
        try {
            estimator.setCorrectorType(null);
            fail("LockedException expected but not thrown");
        } catch (com.irurueta.geometry.estimators.LockedException ignore) { }
        try {
            estimator.setPointsTriangulated(true);
            fail("LockedException expected but not thrown");
        } catch (com.irurueta.geometry.estimators.LockedException ignore) { }
        try {
            estimator.setValidTriangulatedPointsMarked(true);
            fail("LockedException expected but not thrown");
        } catch (com.irurueta.geometry.estimators.LockedException ignore) { }
        try {
            estimator.setFundamentalMatrix(null);
            fail("LockedException expected but not thrown");
        } catch (com.irurueta.geometry.estimators.LockedException ignore) { }
    }
}
