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
import com.irurueta.ar.epipolar.Corrector;
import com.irurueta.ar.epipolar.CorrectorType;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidFundamentalMatrixException;
import com.irurueta.ar.epipolar.InvalidPairOfCamerasException;
import com.irurueta.geometry.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

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

    @Test
    public void testConstructor() {
        EssentialMatrixInitialCamerasEstimator estimator = new EssentialMatrixInitialCamerasEstimator();

        // check default values
        assertNull(estimator.getFundamentalMatrix());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX,
                estimator.getMethod());
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
        estimator = new EssentialMatrixInitialCamerasEstimator(fundamentalMatrix);

        // check default values
        assertSame(estimator.getFundamentalMatrix(), fundamentalMatrix);
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX, estimator.getMethod());
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        final PinholeCameraIntrinsicParameters leftIntrinsic = new PinholeCameraIntrinsicParameters();
        final PinholeCameraIntrinsicParameters rightIntrinsic = new PinholeCameraIntrinsicParameters();
        estimator = new EssentialMatrixInitialCamerasEstimator(leftIntrinsic, rightIntrinsic);

        // check default values
        assertNull(estimator.getFundamentalMatrix());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX, estimator.getMethod());
        assertFalse(estimator.isReady());
        assertSame(estimator.getLeftIntrinsic(), leftIntrinsic);
        assertSame(estimator.getRightIntrinsic(), rightIntrinsic);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        estimator = new EssentialMatrixInitialCamerasEstimator(
                fundamentalMatrix, leftIntrinsic, rightIntrinsic);

        // check default values
        assertSame(estimator.getFundamentalMatrix(), fundamentalMatrix);
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX, estimator.getMethod());
        assertFalse(estimator.isReady());
        assertSame(estimator.getLeftIntrinsic(), leftIntrinsic);
        assertSame(estimator.getRightIntrinsic(), rightIntrinsic);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        final List<Point2D> leftPoints = new ArrayList<>();
        leftPoints.add(Point2D.create());
        final List<Point2D> rightPoints = new ArrayList<>();
        rightPoints.add(Point2D.create());
        estimator = new EssentialMatrixInitialCamerasEstimator(leftPoints, rightPoints);

        // check default values
        assertNull(estimator.getFundamentalMatrix());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX, estimator.getMethod());
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(leftPoints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(null, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        estimator = new EssentialMatrixInitialCamerasEstimator(fundamentalMatrix, leftPoints, rightPoints);

        // check default values
        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX, estimator.getMethod());
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, leftPoints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, null, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        estimator = new EssentialMatrixInitialCamerasEstimator(leftIntrinsic,
                rightIntrinsic, leftPoints, rightPoints);

        // check default values
        assertNull(estimator.getFundamentalMatrix());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX, estimator.getMethod());
        assertFalse(estimator.isReady());
        assertSame(estimator.getLeftIntrinsic(), leftIntrinsic);
        assertSame(estimator.getRightIntrinsic(), rightIntrinsic);
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    leftIntrinsic, rightIntrinsic, leftPoints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    leftIntrinsic, rightIntrinsic, null, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        estimator = new EssentialMatrixInitialCamerasEstimator(
                fundamentalMatrix, leftIntrinsic, rightIntrinsic, leftPoints, rightPoints);

        // check default values
        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX,
                estimator.getMethod());
        assertTrue(estimator.isReady());
        assertSame(leftIntrinsic, estimator.getLeftIntrinsic());
        assertSame(rightIntrinsic, estimator.getRightIntrinsic());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, leftIntrinsic, rightIntrinsic, leftPoints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, leftIntrinsic, rightIntrinsic, null, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        estimator = new EssentialMatrixInitialCamerasEstimator(this);

        // check default values
        assertNull(estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX, estimator.getMethod());
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        estimator = new EssentialMatrixInitialCamerasEstimator(
                fundamentalMatrix, this);

        // check default values
        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX,
                estimator.getMethod());
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        estimator = new EssentialMatrixInitialCamerasEstimator(leftIntrinsic, rightIntrinsic,
                this);

        // check default values
        assertNull(estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX,
                estimator.getMethod());
        assertFalse(estimator.isReady());
        assertSame(leftIntrinsic, estimator.getLeftIntrinsic());
        assertSame(rightIntrinsic, estimator.getRightIntrinsic());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        estimator = new EssentialMatrixInitialCamerasEstimator(
                fundamentalMatrix, leftIntrinsic, rightIntrinsic, this);

        // check default values
        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX,
                estimator.getMethod());
        assertFalse(estimator.isReady());
        assertSame(estimator.getLeftIntrinsic(), leftIntrinsic);
        assertSame(estimator.getRightIntrinsic(), rightIntrinsic);
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        estimator = new EssentialMatrixInitialCamerasEstimator(leftPoints, rightPoints, this);

        // check default values
        assertNull(estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX, estimator.getMethod());
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(leftPoints,
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(null,
                    rightPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        estimator = new EssentialMatrixInitialCamerasEstimator(
                fundamentalMatrix, leftPoints, rightPoints, this);

        // check default values
        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX, estimator.getMethod());
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());
        assertSame(estimator.getLeftPoints(), leftPoints);
        assertSame(estimator.getRightPoints(), rightPoints);
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, leftPoints, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, null, rightPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        estimator = new EssentialMatrixInitialCamerasEstimator(leftIntrinsic,
                rightIntrinsic, leftPoints, rightPoints, this);

        // check default values
        assertNull(estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX, estimator.getMethod());
        assertFalse(estimator.isReady());
        assertSame(leftIntrinsic, estimator.getLeftIntrinsic());
        assertSame(rightIntrinsic, estimator.getRightIntrinsic());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    leftIntrinsic, rightIntrinsic, leftPoints, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    leftIntrinsic, rightIntrinsic, null, rightPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        estimator = new EssentialMatrixInitialCamerasEstimator(
                fundamentalMatrix, leftIntrinsic, rightIntrinsic, leftPoints,
                rightPoints, this);

        // check default values
        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX,
                estimator.getMethod());
        assertTrue(estimator.isReady());
        assertSame(leftIntrinsic, estimator.getLeftIntrinsic());
        assertSame(rightIntrinsic, estimator.getRightIntrinsic());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, leftIntrinsic, rightIntrinsic,
                    leftPoints, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new EssentialMatrixInitialCamerasEstimator(
                    fundamentalMatrix, leftIntrinsic, rightIntrinsic, null,
                    rightPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetFundamentalMatrix() throws
            com.irurueta.geometry.estimators.LockedException {
        final EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();

        // check default value
        assertNull(estimator.getFundamentalMatrix());

        // set new value
        final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
        estimator.setFundamentalMatrix(fundamentalMatrix);

        // check correctness
        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
    }

    @Test
    public void testGetSetListener() {
        final EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testGetSetLeftIntrinsic()
            throws com.irurueta.geometry.estimators.LockedException {
        final EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();

        // check default value
        assertNull(estimator.getLeftIntrinsic());

        // set new value
        final PinholeCameraIntrinsicParameters leftIntrinsic =
                new PinholeCameraIntrinsicParameters();
        estimator.setLeftIntrinsic(leftIntrinsic);

        // check correctness
        assertSame(leftIntrinsic, estimator.getLeftIntrinsic());
    }

    @Test
    public void testGetSetRightIntrinsic()
            throws com.irurueta.geometry.estimators.LockedException {
        final EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();

        // check default value
        assertNull(estimator.getRightIntrinsic());

        // set new value
        final PinholeCameraIntrinsicParameters rightIntrinsic =
                new PinholeCameraIntrinsicParameters();
        estimator.setRightIntrinsic(rightIntrinsic);

        // check correctness
        assertSame(rightIntrinsic, estimator.getRightIntrinsic());
    }

    @Test
    public void testSetLeftAndRightIntrinsics()
            throws com.irurueta.geometry.estimators.LockedException {
        final EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();

        // check default values
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());

        // set new values
        final PinholeCameraIntrinsicParameters leftIntrinsic = new PinholeCameraIntrinsicParameters();
        final PinholeCameraIntrinsicParameters rightIntrinsic = new PinholeCameraIntrinsicParameters();
        estimator.setLeftAndRightIntrinsics(leftIntrinsic, rightIntrinsic);

        // check correctness
        assertSame(leftIntrinsic, estimator.getLeftIntrinsic());
        assertSame(rightIntrinsic, estimator.getRightIntrinsic());
    }

    @Test
    public void testSetIntrinsicsForBoth()
            throws com.irurueta.geometry.estimators.LockedException {
        final EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();

        // check default values
        assertNull(estimator.getLeftIntrinsic());
        assertNull(estimator.getRightIntrinsic());

        // set new value
        final PinholeCameraIntrinsicParameters intrinsic = new PinholeCameraIntrinsicParameters();
        estimator.setIntrinsicsForBoth(intrinsic);

        // check correctness
        assertSame(intrinsic, estimator.getLeftIntrinsic());
        assertSame(intrinsic, estimator.getRightIntrinsic());
    }

    @Test
    public void testGetSetLeftPoints()
            throws com.irurueta.geometry.estimators.LockedException {
        final EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();

        // check default value
        assertNull(estimator.getLeftPoints());

        // set new value
        final List<Point2D> leftPoints = new ArrayList<>();
        estimator.setLeftPoints(leftPoints);

        // check correctness
        assertSame(leftPoints, estimator.getLeftPoints());
    }

    @Test
    public void testGetSetRightPoints()
            throws com.irurueta.geometry.estimators.LockedException {
        final EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();

        // check default value
        assertNull(estimator.getRightPoints());

        // set new value
        final List<Point2D> rightPoints = new ArrayList<>();
        estimator.setRightPoints(rightPoints);

        // check correctness
        assertSame(rightPoints, estimator.getRightPoints());
    }

    @Test
    public void testSetLeftAndRightPoints()
            throws com.irurueta.geometry.estimators.LockedException {
        final EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();

        // check default values
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());

        // set new values
        final List<Point2D> leftPoints = new ArrayList<>();
        leftPoints.add(Point2D.create());
        final List<Point2D> rightPoints = new ArrayList<>();
        rightPoints.add(Point2D.create());
        estimator.setLeftAndRightPoints(leftPoints, rightPoints);

        // check correctness
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());

        // Force IllegalArgumentException
        try {
            estimator.setLeftAndRightPoints(leftPoints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setLeftAndRightPoints(null, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        final List<Point2D> wrongPoints = new ArrayList<>();
        try {
            estimator.setLeftAndRightPoints(leftPoints, wrongPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetCorrectorType()
            throws com.irurueta.geometry.estimators.LockedException {
        final EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();

        // check default value
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());

        // set new value
        estimator.setCorrectorType(CorrectorType.GOLD_STANDARD);

        // check correctness
        assertEquals(CorrectorType.GOLD_STANDARD, estimator.getCorrectorType());
    }

    @Test
    public void testAreSetPointsTriangulated()
            throws com.irurueta.geometry.estimators.LockedException {
        final EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();

        // check default value
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());

        // set new value
        estimator.setPointsTriangulated(
                !EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS);

        // check correctness
        assertEquals(!EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
    }

    @Test
    public void testAreValidTriangulatedPointsMarked()
            throws com.irurueta.geometry.estimators.LockedException {
        final EssentialMatrixInitialCamerasEstimator estimator =
                new EssentialMatrixInitialCamerasEstimator();

        // check default value
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());

        // set new value
        estimator.setValidTriangulatedPointsMarked(
                !EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS);

        // check correctness
        assertEquals(!EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
    }

    @Test
    public void testEstimate() throws InvalidPairOfCamerasException, CameraException,
            com.irurueta.geometry.estimators.LockedException,
            com.irurueta.geometry.estimators.NotReadyException,
            InitialCamerasEstimationFailedException, com.irurueta.geometry.NotAvailableException,
            InvalidFundamentalMatrixException, AlgebraException {
        int numValidTimes = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double principalPoint = 0.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            final double[] principalAxis1 = camera1.getPrincipalAxisArray();
            final double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1;
            double lambda2;

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            InhomogeneousPoint3D worldPoint;
            final List<InhomogeneousPoint3D> worldPoints = new ArrayList<>();
            Point2D leftPoint;
            Point2D rightPoint;
            final List<Point2D> leftPoints = new ArrayList<>();
            final List<Point2D> rightPoints = new ArrayList<>();
            boolean leftFront;
            boolean rightFront;
            for (int i = 0; i < nPoints; i++) {
                // generate points and ensure they lie in front of both cameras
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

                // check that world point is in front of both cameras
                assertTrue(leftFront);
                assertTrue(rightFront);

                // project world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);

                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }

            EssentialMatrixInitialCamerasEstimator estimator =
                    new EssentialMatrixInitialCamerasEstimator(fundamentalMatrix, intrinsic, intrinsic,
                            leftPoints, rightPoints, this);
            estimator.setPointsTriangulated(true);
            estimator.setValidTriangulatedPointsMarked(true);

            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedLeftCamera());
            assertNull(estimator.getEstimatedRightCamera());
            assertNull(estimator.getTriangulatedPoints());
            assertNull(estimator.getValidTriangulatedPoints());

            estimator.estimate();

            final PinholeCamera camera1b = estimator.getEstimatedLeftCamera();
            final PinholeCamera camera2b = estimator.getEstimatedRightCamera();
            final List<Point3D> triangulatedPoints = estimator.getTriangulatedPoints();
            final BitSet validTriangulatedPoints = estimator.getValidTriangulatedPoints();

            camera1b.decompose();
            camera2b.decompose();

            final PinholeCameraIntrinsicParameters intrinsic1b = camera1b.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters intrinsic2b = camera2b.getIntrinsicParameters();

            final Rotation3D rotation1b = camera1b.getCameraRotation();
            final Rotation3D rotation2b = camera2b.getCameraRotation();

            final Point3D center1b = camera1b.getCameraCenter();
            final Point3D center2b = camera2b.getCameraCenter();

            assertEquals(focalLength, intrinsic1b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic1b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(focalLength, intrinsic2b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic2b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            final Rotation3D diffRotation = rotation2b.combineAndReturnNew(
                    rotation1b.inverseRotationAndReturnNew());

            assertTrue(rotation1b.asInhomogeneousMatrix().equals(
                    Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(
                    diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            assertNotNull(center1b);
            assertNotNull(center2b);

            // compute scale factor
            final double distanceA = center1.distanceTo(center2);
            final double distanceB = center1b.distanceTo(center2b);
            final double scaleFactor = distanceB / distanceA;
            final double invScaleFactor = distanceA / distanceB;

            // NOTE: distance between estimated cameras is always normalized
            assertEquals(1.0, distanceB, ABSOLUTE_ERROR);

            final MetricTransformation3D scaleTransformation = new MetricTransformation3D(scaleFactor);
            final Transformation3D invScaleTransformation = scaleTransformation.inverseAndReturnNew();
            final MetricTransformation3D invScaleTransformation2 =
                    new MetricTransformation3D(invScaleFactor);

            assertTrue(invScaleTransformation.asMatrix().equals(
                    invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));


            // check that estimated cameras generate the same input fundamental
            // matrix
            FundamentalMatrix fundamentalMatrixB = new FundamentalMatrix(camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar
            // geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final Point2D epipole1 = camera1.project(center2);
            final Point2D epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final Point2D epipole1a = fundamentalMatrix.getLeftEpipole();
            final Point2D epipole2a = fundamentalMatrix.getRightEpipole();

            final Point2D epipole1b = fundamentalMatrixB.getLeftEpipole();
            final Point2D epipole2b = fundamentalMatrixB.getRightEpipole();

            assertEquals(0.0, epipole1.distanceTo(epipole1a), ABSOLUTE_ERROR);
            assertEquals(0.0, epipole2.distanceTo(epipole2a), ABSOLUTE_ERROR);

            assertEquals(0.0, epipole1.distanceTo(epipole1b), ABSOLUTE_ERROR);
            assertEquals(0.0, epipole2.distanceTo(epipole2b), LARGE_ABSOLUTE_ERROR);

            // generate epipolar lines
            Point3D scaledWorldPoint;
            Point3D triangulatedPoint;
            Point3D scaledTriangulatedPoint;
            int numValid1a = 0;
            int numValid2a = 0;
            int numValid1b = 0;
            int numValid2b = 0;
            int numValidEqual = 0;
            for (int i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);

                triangulatedPoint = triangulatedPoints.get(i);
                assertTrue(validTriangulatedPoints.get(i));

                final Line2D line1a = fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                final Line2D line2a = fundamentalMatrix.getRightEpipolarLine(leftPoint);

                final Line2D line1b = fundamentalMatrixB.getLeftEpipolarLine(rightPoint);
                final Line2D line2b = fundamentalMatrixB.getRightEpipolarLine(leftPoint);

                // check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));

                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));

                // back-project epipolar lines for each pair of cameras and check
                // that each pair of lines correspond to the same epipolar plane
                final Plane epipolarPlane1a = camera1.backProject(line1a);
                final Plane epipolarPlane2a = camera2.backProject(line2a);

                final Plane epipolarPlane1b = camera1b.backProject(line1b);
                final Plane epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

                // check that 3D point and both camera centers for each pair of
                // cameras belong to their corresponding epipolar plane
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

                // notice that since estimated cameras have an arbitrary scale,
                // original world point doesn't need to lie on epipolar plane
                // because first a scale transformation needs to be done
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(worldPoint);
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

                // recover scale in triangulated point
                scaledTriangulatedPoint = invScaleTransformation.transformAndReturnNew(triangulatedPoint);

                // check that triangulated point after recovering scale matches
                // original point
                if (worldPoint.equals(scaledTriangulatedPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidEqual++;
                }
            }

            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 && numValid2b > 0
                    && numValidEqual > 0) {
                numValidTimes++;
            }

            // recover scale of cameras by undoing their transformations
            final PinholeCamera camera1c = invScaleTransformation.transformAndReturnNew(camera1b);
            final PinholeCamera camera2c = invScaleTransformation.transformAndReturnNew(camera2b);

            // check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();

            final Matrix camera1Matrix = camera1.getInternalMatrix();
            final Matrix camera1cMatrix = camera1c.getInternalMatrix();
            assertTrue(camera1Matrix.equals(camera1cMatrix, ABSOLUTE_ERROR));

            final Matrix camera2Matrix = camera2.getInternalMatrix();
            final Matrix camera2cMatrix = camera2c.getInternalMatrix();
            assertTrue(camera2Matrix.equals(camera2cMatrix, ABSOLUTE_ERROR));

            // force NotReadyException
            estimator = new EssentialMatrixInitialCamerasEstimator();

            assertFalse(estimator.isReady());

            try {
                estimator.estimate();
                fail("NotReadyException expected but not thrown");
            } catch (final com.irurueta.geometry.estimators.NotReadyException ignore) {
            }
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
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double principalPoint = 0.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            final double[] principalAxis1 = camera1.getPrincipalAxisArray();
            final double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1;
            double lambda2;

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            InhomogeneousPoint3D worldPoint;
            final List<InhomogeneousPoint3D> worldPoints = new ArrayList<>();
            Point2D leftPoint;
            Point2D rightPoint;
            final List<Point2D> leftPoints = new ArrayList<>();
            final List<Point2D> rightPoints = new ArrayList<>();
            boolean leftFront;
            boolean rightFront;
            for (int i = 0; i < nPoints; i++) {
                // generate points and ensure they lie in front of both cameras
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

                // check that world point is in front of both cameras
                assertTrue(leftFront);
                assertTrue(rightFront);

                // project world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);

                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }

            final PinholeCamera camera1b = new PinholeCamera();
            final PinholeCamera camera2b = new PinholeCamera();
            final int numValid = EssentialMatrixInitialCamerasEstimator.
                    generateInitialMetricCamerasFromEssentialMatrix(
                            fundamentalMatrix, intrinsic, intrinsic, leftPoints, rightPoints,
                            camera1b, camera2b);

            // check correctness
            assertEquals(numValid, nPoints);

            camera1b.decompose();
            camera2b.decompose();

            final PinholeCameraIntrinsicParameters intrinsic1b = camera1b.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters intrinsic2b = camera2b.getIntrinsicParameters();

            final Rotation3D rotation1b = camera1b.getCameraRotation();
            final Rotation3D rotation2b = camera2b.getCameraRotation();

            final Point3D center1b = camera1b.getCameraCenter();
            final Point3D center2b = camera2b.getCameraCenter();

            assertEquals(focalLength, intrinsic1b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic1b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(focalLength, intrinsic2b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic2b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            final Rotation3D diffRotation = rotation2b.combineAndReturnNew(
                    rotation1b.inverseRotationAndReturnNew());

            assertTrue(rotation1b.asInhomogeneousMatrix().equals(
                    Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(
                    diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            assertNotNull(center1b);
            assertNotNull(center2b);

            // compute scale factor
            final double distanceA = center1.distanceTo(center2);
            final double distanceB = center1b.distanceTo(center2b);
            final double scaleFactor = distanceB / distanceA;
            final double invScaleFactor = distanceA / distanceB;

            // NOTE: distance between estimated cameras is always normalized
            assertEquals(1.0, distanceB, ABSOLUTE_ERROR);

            final MetricTransformation3D scaleTransformation = new MetricTransformation3D(scaleFactor);
            final Transformation3D invScaleTransformation = scaleTransformation.inverseAndReturnNew();
            final MetricTransformation3D invScaleTransformation2 =
                    new MetricTransformation3D(invScaleFactor);

            assertTrue(invScaleTransformation.asMatrix().equals(
                    invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));


            // check that estimated cameras generate the same input fundamental
            // matrix
            final FundamentalMatrix fundamentalMatrixB = new FundamentalMatrix(camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar
            // geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final Point2D epipole1 = camera1.project(center2);
            final Point2D epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final Point2D epipole1a = fundamentalMatrix.getLeftEpipole();
            final Point2D epipole2a = fundamentalMatrix.getRightEpipole();

            final Point2D epipole1b = fundamentalMatrixB.getLeftEpipole();
            final Point2D epipole2b = fundamentalMatrixB.getRightEpipole();

            assertEquals(0.0, epipole1.distanceTo(epipole1a), ABSOLUTE_ERROR);
            assertEquals(0.0, epipole2.distanceTo(epipole2a), ABSOLUTE_ERROR);

            assertEquals(0.0, epipole1.distanceTo(epipole1b), ABSOLUTE_ERROR);
            assertEquals(0.0, epipole2.distanceTo(epipole2b), LARGE_ABSOLUTE_ERROR);

            // generate epipolar lines
            Point3D scaledWorldPoint;
            int numValid1a = 0;
            int numValid2a = 0;
            int numValid1b = 0;
            int numValid2b = 0;
            for (int i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);

                final Line2D line1a = fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                final Line2D line2a = fundamentalMatrix.getRightEpipolarLine(leftPoint);

                final Line2D line1b = fundamentalMatrixB.getLeftEpipolarLine(rightPoint);
                final Line2D line2b = fundamentalMatrixB.getRightEpipolarLine(leftPoint);

                // check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));

                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));

                // back-project epipolar lines for each pair of cameras and check
                // that each pair of lines correspond to the same epipolar plane
                final Plane epipolarPlane1a = camera1.backProject(line1a);
                final Plane epipolarPlane2a = camera2.backProject(line2a);

                final Plane epipolarPlane1b = camera1b.backProject(line1b);
                final Plane epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

                // check that 3D point and both camera centers for each pair of
                // cameras belong to their corresponding epipolar plane
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

                // notice that since estimated cameras have an arbitrary scale,
                // original world point doesn't need to lie on epipolar plane
                // because first a scale transformation needs to be done
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

            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 && numValid2b > 0) {
                numValidTimes++;
            }

            // recover scale of cameras by undoing their transformations
            final PinholeCamera camera1c = invScaleTransformation.transformAndReturnNew(camera1b);
            final PinholeCamera camera2c = invScaleTransformation.transformAndReturnNew(camera2b);

            // check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();

            final Matrix camera1Matrix = camera1.getInternalMatrix();
            final Matrix camera1cMatrix = camera1c.getInternalMatrix();
            assertTrue(camera1Matrix.equals(camera1cMatrix, ABSOLUTE_ERROR));

            final Matrix camera2Matrix = camera2.getInternalMatrix();
            final Matrix camera2cMatrix = camera2c.getInternalMatrix();
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
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double principalPoint = 0.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS,
                    Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            final double[] principalAxis1 = camera1.getPrincipalAxisArray();
            final double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1;
            double lambda2;

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            InhomogeneousPoint3D worldPoint;
            final List<InhomogeneousPoint3D> worldPoints = new ArrayList<>();
            Point2D leftPoint;
            Point2D rightPoint;
            final List<Point2D> leftPoints = new ArrayList<>();
            final List<Point2D> rightPoints = new ArrayList<>();
            boolean leftFront;
            boolean rightFront;
            for (int i = 0; i < nPoints; i++) {
                // generate points and ensure they lie in front of both cameras
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

                // check that world point is in front of both cameras
                assertTrue(leftFront);
                assertTrue(rightFront);

                // project world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);

                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }

            final PinholeCamera camera1b = new PinholeCamera();
            final PinholeCamera camera2b = new PinholeCamera();
            final int numValid = EssentialMatrixInitialCamerasEstimator.
                    generateInitialMetricCamerasFromEssentialMatrix(
                            fundamentalMatrix, intrinsic, intrinsic, leftPoints,
                            rightPoints, null, camera1b, camera2b);

            // check correctness
            assertEquals(numValid, nPoints);

            camera1b.decompose();
            camera2b.decompose();

            final PinholeCameraIntrinsicParameters intrinsic1b = camera1b.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters intrinsic2b = camera2b.getIntrinsicParameters();

            final Rotation3D rotation1b = camera1b.getCameraRotation();
            final Rotation3D rotation2b = camera2b.getCameraRotation();

            final Point3D center1b = camera1b.getCameraCenter();
            final Point3D center2b = camera2b.getCameraCenter();

            assertEquals(focalLength, intrinsic1b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic1b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(focalLength, intrinsic2b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic2b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            final Rotation3D diffRotation = rotation2b.combineAndReturnNew(
                    rotation1b.inverseRotationAndReturnNew());

            assertTrue(rotation1b.asInhomogeneousMatrix().equals(
                    Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(
                    diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            assertNotNull(center1b);
            assertNotNull(center2b);

            // compute scale factor
            final double distanceA = center1.distanceTo(center2);
            final double distanceB = center1b.distanceTo(center2b);
            final double scaleFactor = distanceB / distanceA;
            final double invScaleFactor = distanceA / distanceB;

            // NOTE: distance between estimated cameras is always normalized
            assertEquals(1.0, distanceB, ABSOLUTE_ERROR);

            final MetricTransformation3D scaleTransformation = new MetricTransformation3D(scaleFactor);
            final Transformation3D invScaleTransformation = scaleTransformation.inverseAndReturnNew();
            final MetricTransformation3D invScaleTransformation2 =
                    new MetricTransformation3D(invScaleFactor);

            assertTrue(invScaleTransformation.asMatrix().equals(
                    invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));

            // check that estimated cameras generate the same input fundamental
            // matrix
            final FundamentalMatrix fundamentalMatrixB = new FundamentalMatrix(camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar
            // geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final Point2D epipole1 = camera1.project(center2);
            final Point2D epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final Point2D epipole1a = fundamentalMatrix.getLeftEpipole();
            final Point2D epipole2a = fundamentalMatrix.getRightEpipole();

            final Point2D epipole1b = fundamentalMatrixB.getLeftEpipole();
            final Point2D epipole2b = fundamentalMatrixB.getRightEpipole();

            assertEquals(0.0, epipole1.distanceTo(epipole1a), ABSOLUTE_ERROR);
            assertEquals(0.0, epipole2.distanceTo(epipole2a), ABSOLUTE_ERROR);

            assertEquals(0.0, epipole1.distanceTo(epipole1b), ABSOLUTE_ERROR);
            assertEquals(0.0, epipole2.distanceTo(epipole2b), LARGE_ABSOLUTE_ERROR);

            // generate epipolar lines
            Point3D scaledWorldPoint;
            int numValid1a = 0;
            int numValid2a = 0;
            int numValid1b = 0;
            int numValid2b = 0;
            for (int i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);

                final Line2D line1a = fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                final Line2D line2a = fundamentalMatrix.getRightEpipolarLine(leftPoint);

                final Line2D line1b = fundamentalMatrixB.getLeftEpipolarLine(rightPoint);
                final Line2D line2b = fundamentalMatrixB.getRightEpipolarLine(leftPoint);

                // check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));

                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));

                // back-project epipolar lines for each pair of cameras and check
                // that each pair of lines correspond to the same epipolar plane
                final Plane epipolarPlane1a = camera1.backProject(line1a);
                final Plane epipolarPlane2a = camera2.backProject(line2a);

                final Plane epipolarPlane1b = camera1b.backProject(line1b);
                final Plane epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

                // check that 3D point and both camera centers for each pair of
                // cameras belong to their corresponding epipolar plane
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

                // notice that since estimated cameras have an arbitrary scale,
                // original world point doesn't need to lie on epipolar plane
                // because first a scale transformation needs to be done
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(worldPoint);
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

            // recover scale of cameras by undoing their transformations
            final PinholeCamera camera1c = invScaleTransformation.transformAndReturnNew(camera1b);
            final PinholeCamera camera2c = invScaleTransformation.transformAndReturnNew(camera2b);

            // check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();

            final Matrix camera1Matrix = camera1.getInternalMatrix();
            final Matrix camera1cMatrix = camera1c.getInternalMatrix();
            assertTrue(camera1Matrix.equals(camera1cMatrix, ABSOLUTE_ERROR));

            final Matrix camera2Matrix = camera2.getInternalMatrix();
            final Matrix camera2cMatrix = camera2c.getInternalMatrix();
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
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double principalPoint = 0.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            final double[] principalAxis1 = camera1.getPrincipalAxisArray();
            final double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1;
            double lambda2;

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            InhomogeneousPoint3D worldPoint;
            final List<InhomogeneousPoint3D> worldPoints = new ArrayList<>();
            Point2D leftPoint;
            Point2D rightPoint;
            final List<Point2D> leftPoints = new ArrayList<>();
            final List<Point2D> rightPoints = new ArrayList<>();
            boolean leftFront;
            boolean rightFront;
            for (int i = 0; i < nPoints; i++) {
                // generate points and ensure they lie in front of both cameras
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

                // check that world point is in front of both cameras
                assertTrue(leftFront);
                assertTrue(rightFront);

                // project world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);

                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }

            final PinholeCamera camera1b = new PinholeCamera();
            final PinholeCamera camera2b = new PinholeCamera();
            final List<Point3D> triangulatedPoints = new ArrayList<>();
            final BitSet validTriangulatedPoints = new BitSet(nPoints);
            final int numValid = EssentialMatrixInitialCamerasEstimator.
                    generateInitialMetricCamerasFromEssentialMatrix(
                            fundamentalMatrix, intrinsic, intrinsic, leftPoints,
                            rightPoints, camera1b, camera2b, triangulatedPoints,
                            validTriangulatedPoints);

            // check correctness
            assertEquals(numValid, nPoints);

            camera1b.decompose();
            camera2b.decompose();

            final PinholeCameraIntrinsicParameters intrinsic1b = camera1b.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters intrinsic2b = camera2b.getIntrinsicParameters();

            final Rotation3D rotation1b = camera1b.getCameraRotation();
            final Rotation3D rotation2b = camera2b.getCameraRotation();

            final Point3D center1b = camera1b.getCameraCenter();
            final Point3D center2b = camera2b.getCameraCenter();

            assertEquals(focalLength, intrinsic1b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic1b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(focalLength, intrinsic2b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic2b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            final Rotation3D diffRotation = rotation2b.combineAndReturnNew(
                    rotation1b.inverseRotationAndReturnNew());

            assertTrue(rotation1b.asInhomogeneousMatrix().equals(
                    Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(
                    diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            assertNotNull(center1b);
            assertNotNull(center2b);

            // compute scale factor
            final double distanceA = center1.distanceTo(center2);
            final double distanceB = center1b.distanceTo(center2b);
            final double scaleFactor = distanceB / distanceA;
            final double invScaleFactor = distanceA / distanceB;

            // NOTE: distance between estimated cameras is always normalized
            assertEquals(1.0, distanceB, ABSOLUTE_ERROR);

            final MetricTransformation3D scaleTransformation = new MetricTransformation3D(scaleFactor);
            final Transformation3D invScaleTransformation = scaleTransformation.inverseAndReturnNew();
            final MetricTransformation3D invScaleTransformation2 =
                    new MetricTransformation3D(invScaleFactor);

            assertTrue(invScaleTransformation.asMatrix().equals(
                    invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));

            // check that estimated cameras generate the same input fundamental
            // matrix
            final FundamentalMatrix fundamentalMatrixB = new FundamentalMatrix(camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final Point2D epipole1 = camera1.project(center2);
            final Point2D epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final Point2D epipole1a = fundamentalMatrix.getLeftEpipole();
            final Point2D epipole2a = fundamentalMatrix.getRightEpipole();

            final Point2D epipole1b = fundamentalMatrixB.getLeftEpipole();
            final Point2D epipole2b = fundamentalMatrixB.getRightEpipole();

            assertEquals(0.0, epipole1.distanceTo(epipole1a), ABSOLUTE_ERROR);
            assertEquals(0.0, epipole2.distanceTo(epipole2a), ABSOLUTE_ERROR);

            assertEquals(0.0, epipole1.distanceTo(epipole1b), ABSOLUTE_ERROR);
            assertEquals(0.0, epipole2.distanceTo(epipole2b), LARGE_ABSOLUTE_ERROR);

            // generate epipolar lines
            Point3D scaledWorldPoint;
            Point3D triangulatedPoint;
            Point3D scaledTriangulatedPoint;
            int numValid1a = 0;
            int numValid2a = 0;
            int numValid1b = 0;
            int numValid2b = 0;
            int numValidEqual = 0;
            for (int i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);

                triangulatedPoint = triangulatedPoints.get(i);
                assertTrue(validTriangulatedPoints.get(i));

                final Line2D line1a = fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                final Line2D line2a = fundamentalMatrix.getRightEpipolarLine(leftPoint);

                final Line2D line1b = fundamentalMatrixB.getLeftEpipolarLine(rightPoint);
                final Line2D line2b = fundamentalMatrixB.getRightEpipolarLine(leftPoint);

                // check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));

                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));

                // back-project epipolar lines for each pair of cameras and check
                // that each pair of lines correspond to the same epipolar plane
                final Plane epipolarPlane1a = camera1.backProject(line1a);
                final Plane epipolarPlane2a = camera2.backProject(line2a);

                final Plane epipolarPlane1b = camera1b.backProject(line1b);
                final Plane epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

                // check that 3D point and both camera centers for each pair of
                // cameras belong to their corresponding epipolar plane
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

                // notice that since estimated cameras have an arbitrary scale,
                // original world point doesn't need to lie on epipolar plane
                // because first a scale transformation needs to be done
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(worldPoint);
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

                // recover scale in triangulated point
                scaledTriangulatedPoint = invScaleTransformation.transformAndReturnNew(triangulatedPoint);

                // check that triangulated point after recovering scale matches
                // original point
                if (worldPoint.equals(scaledTriangulatedPoint,
                        LARGE_ABSOLUTE_ERROR)) {
                    numValidEqual++;
                }
            }

            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 && numValid2b > 0
                    && numValidEqual > 0) {
                numValidTimes++;
            }

            // recover scale of cameras by undoing their transformations
            final PinholeCamera camera1c = invScaleTransformation.transformAndReturnNew(camera1b);
            final PinholeCamera camera2c = invScaleTransformation.transformAndReturnNew(camera2b);

            // check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();

            final Matrix camera1Matrix = camera1.getInternalMatrix();
            final Matrix camera1cMatrix = camera1c.getInternalMatrix();
            assertTrue(camera1Matrix.equals(camera1cMatrix, ABSOLUTE_ERROR));

            final Matrix camera2Matrix = camera2.getInternalMatrix();
            final Matrix camera2cMatrix = camera2c.getInternalMatrix();
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
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double principalPoint = 0.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            final double[] principalAxis1 = camera1.getPrincipalAxisArray();
            final double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1;
            double lambda2;

            final int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            InhomogeneousPoint3D worldPoint;
            final List<InhomogeneousPoint3D> worldPoints = new ArrayList<>();
            Point2D leftPoint;
            Point2D rightPoint;
            final List<Point2D> leftPoints = new ArrayList<>();
            final List<Point2D> rightPoints = new ArrayList<>();
            boolean leftFront;
            boolean rightFront;
            for (int i = 0; i < nPoints; i++) {
                // generate points and ensure they lie in front of both cameras
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

                // check that world point is in front of both cameras
                assertTrue(leftFront);
                assertTrue(rightFront);

                // project world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);

                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }

            final PinholeCamera camera1b = new PinholeCamera();
            final PinholeCamera camera2b = new PinholeCamera();
            final List<Point3D> triangulatedPoints = new ArrayList<>();
            final BitSet validTriangulatedPoints = new BitSet(nPoints);
            final int numValid = EssentialMatrixInitialCamerasEstimator.
                    generateInitialMetricCamerasFromEssentialMatrix(
                            fundamentalMatrix, intrinsic, intrinsic, leftPoints,
                            rightPoints, CorrectorType.SAMPSON_CORRECTOR, camera1b,
                            camera2b, triangulatedPoints, validTriangulatedPoints);

            // check correctness
            assertEquals(numValid, nPoints);

            camera1b.decompose();
            camera2b.decompose();

            final PinholeCameraIntrinsicParameters intrinsic1b = camera1b.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters intrinsic2b = camera2b.getIntrinsicParameters();

            final Rotation3D rotation1b = camera1b.getCameraRotation();
            final Rotation3D rotation2b = camera2b.getCameraRotation();

            final Point3D center1b = camera1b.getCameraCenter();
            final Point3D center2b = camera2b.getCameraCenter();

            assertEquals(focalLength, intrinsic1b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic1b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(focalLength, intrinsic2b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic2b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            final Rotation3D diffRotation = rotation2b.combineAndReturnNew(
                    rotation1b.inverseRotationAndReturnNew());

            assertTrue(rotation1b.asInhomogeneousMatrix().equals(
                    Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(
                    diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            assertNotNull(center1b);
            assertNotNull(center2b);

            // compute scale factor
            final double distanceA = center1.distanceTo(center2);
            final double distanceB = center1b.distanceTo(center2b);
            final double scaleFactor = distanceB / distanceA;
            final double invScaleFactor = distanceA / distanceB;

            // NOTE: distance between estimated cameras is always normalized
            assertEquals(1.0, distanceB, ABSOLUTE_ERROR);

            final MetricTransformation3D scaleTransformation = new MetricTransformation3D(scaleFactor);
            final Transformation3D invScaleTransformation = scaleTransformation.inverseAndReturnNew();
            final MetricTransformation3D invScaleTransformation2 =
                    new MetricTransformation3D(invScaleFactor);

            assertTrue(invScaleTransformation.asMatrix().equals(
                    invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));


            // check that estimated cameras generate the same input fundamental
            // matrix
            final FundamentalMatrix fundamentalMatrixB = new FundamentalMatrix(camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar
            // geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final Point2D epipole1 = camera1.project(center2);
            final Point2D epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final Point2D epipole1a = fundamentalMatrix.getLeftEpipole();
            final Point2D epipole2a = fundamentalMatrix.getRightEpipole();

            final Point2D epipole1b = fundamentalMatrixB.getLeftEpipole();
            final Point2D epipole2b = fundamentalMatrixB.getRightEpipole();

            assertEquals(0.0, epipole1.distanceTo(epipole1a), ABSOLUTE_ERROR);
            assertEquals(0.0, epipole2.distanceTo(epipole2a), ABSOLUTE_ERROR);

            assertEquals(0.0, epipole1.distanceTo(epipole1b), ABSOLUTE_ERROR);
            assertEquals(0.0, epipole2.distanceTo(epipole2b), LARGE_ABSOLUTE_ERROR);

            // generate epipolar lines
            Point3D scaledWorldPoint;
            Point3D triangulatedPoint;
            Point3D scaledTriangulatedPoint;
            int numValid1a = 0;
            int numValid2a = 0;
            int numValid1b = 0;
            int numValid2b = 0;
            int numValidEqual = 0;
            for (int i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);

                triangulatedPoint = triangulatedPoints.get(i);
                assertTrue(validTriangulatedPoints.get(i));

                final Line2D line1a = fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                final Line2D line2a = fundamentalMatrix.getRightEpipolarLine(leftPoint);

                final Line2D line1b = fundamentalMatrixB.getLeftEpipolarLine(rightPoint);
                final Line2D line2b = fundamentalMatrixB.getRightEpipolarLine(leftPoint);

                // check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));

                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));

                // back-project epipolar lines for each pair of cameras and check
                // that each pair of lines correspond to the same epipolar plane
                final Plane epipolarPlane1a = camera1.backProject(line1a);
                final Plane epipolarPlane2a = camera2.backProject(line2a);

                final Plane epipolarPlane1b = camera1b.backProject(line1b);
                final Plane epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

                // check that 3D point and both camera centers for each pair of
                // cameras belong to their corresponding epipolar plane
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

                // notice that since estimated cameras have an arbitrary scale,
                // original world point doesn't need to lie on epipolar plane
                // because first a scale transformation needs to be done
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(worldPoint);
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

                // recover scale in triangulated point
                scaledTriangulatedPoint = invScaleTransformation.transformAndReturnNew(triangulatedPoint);

                // check that triangulated point after recovering scale matches
                // original point
                if (worldPoint.equals(scaledTriangulatedPoint,
                        LARGE_ABSOLUTE_ERROR)) {
                    numValidEqual++;
                }
            }

            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 &&
                    numValid2b > 0 && numValidEqual > 0) {
                numValidTimes++;
            }

            // recover scale of cameras by undoing their transformations
            final PinholeCamera camera1c = invScaleTransformation.transformAndReturnNew(camera1b);
            final PinholeCamera camera2c = invScaleTransformation.transformAndReturnNew(camera2b);

            // check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();

            final Matrix camera1Matrix = camera1.getInternalMatrix();
            final Matrix camera1cMatrix = camera1c.getInternalMatrix();
            assertTrue(camera1Matrix.equals(camera1cMatrix, ABSOLUTE_ERROR));

            final Matrix camera2Matrix = camera2.getInternalMatrix();
            final Matrix camera2cMatrix = camera2c.getInternalMatrix();
            assertTrue(camera2Matrix.equals(camera2cMatrix, ABSOLUTE_ERROR));
        }

        assertTrue(numValidTimes > 0);
    }

    @Override
    public void onStart(final InitialCamerasEstimator estimator) {
        checkLocked((EssentialMatrixInitialCamerasEstimator) estimator);
    }

    @Override
    public void onFinish(final InitialCamerasEstimator estimator,
                         final PinholeCamera estimatedLeftCamera,
                         final PinholeCamera estimatedRightCamera) {
        checkLocked((EssentialMatrixInitialCamerasEstimator) estimator);
    }

    @Override
    public void onFail(final InitialCamerasEstimator estimator,
                       final InitialCamerasEstimationFailedException e) {
        checkLocked((EssentialMatrixInitialCamerasEstimator) estimator);
    }

    private void checkLocked(final EssentialMatrixInitialCamerasEstimator estimator) {
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (final com.irurueta.geometry.estimators.LockedException ignore) {
        } catch (final com.irurueta.geometry.estimators.NotReadyException |
                InitialCamerasEstimationFailedException ex) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setLeftIntrinsic(null);
            fail("LockedException expected but not thrown");
        } catch (final com.irurueta.geometry.estimators.LockedException ignore) {
        }
        try {
            estimator.setRightIntrinsic(null);
            fail("LockedException expected but not thrown");
        } catch (final com.irurueta.geometry.estimators.LockedException ignore) {
        }
        try {
            estimator.setLeftAndRightIntrinsics(null, null);
            fail("LockedException expected but not thrown");
        } catch (final com.irurueta.geometry.estimators.LockedException ignore) {
        }
        try {
            estimator.setIntrinsicsForBoth(null);
            fail("LockedException expected but not thrown");
        } catch (final com.irurueta.geometry.estimators.LockedException ignore) {
        }
        try {
            estimator.setLeftPoints(null);
            fail("LockedException expected but not thrown");
        } catch (final com.irurueta.geometry.estimators.LockedException ignore) {
        }
        try {
            estimator.setRightPoints(null);
            fail("LockedException expected but not thrown");
        } catch (final com.irurueta.geometry.estimators.LockedException ignore) {
        }
        try {
            estimator.setLeftAndRightPoints(null, null);
            fail("LockedException expected but not thrown");
        } catch (final com.irurueta.geometry.estimators.LockedException ignore) {
        }
        try {
            estimator.setCorrectorType(null);
            fail("LockedException expected but not thrown");
        } catch (final com.irurueta.geometry.estimators.LockedException ignore) {
        }
        try {
            estimator.setPointsTriangulated(true);
            fail("LockedException expected but not thrown");
        } catch (final com.irurueta.geometry.estimators.LockedException ignore) {
        }
        try {
            estimator.setValidTriangulatedPointsMarked(true);
            fail("LockedException expected but not thrown");
        } catch (final com.irurueta.geometry.estimators.LockedException ignore) {
        }
        try {
            estimator.setFundamentalMatrix(null);
            fail("LockedException expected but not thrown");
        } catch (final com.irurueta.geometry.estimators.LockedException ignore) {
        }
    }
}
