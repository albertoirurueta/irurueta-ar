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
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class EpipolarDistanceFundamentalMatrixComparatorTest implements FundamentalMatrixComparatorListener {

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

    private int compareStart;
    private int compareEnd;
    private int compareProgressChange;

    @Test
    void testConstructor() throws AlgebraException, InvalidFundamentalMatrixException {
        // test constructor without arguments
        var comparator = new EpipolarDistanceFundamentalMatrixComparator();

        // check default values
        assertNull(comparator.getGroundTruthFundamentalMatrix());
        assertNull(comparator.getOtherFundamentalMatrix());
        assertNull(comparator.getListener());
        assertFalse(comparator.isLocked());
        assertFalse(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR, comparator.getType());
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, comparator.getMinX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, comparator.getMaxX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, comparator.getMinY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, comparator.getMaxY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, comparator.getNSamples(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_ITERATIONS_FACTOR,
                comparator.getMaxIterationsFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_PROGRESS_DELTA, comparator.getProgressDelta(),
                0.0);

        // test constructor with fundamental matrices
        final var emptyFundamentalMatrix1 = new FundamentalMatrix();
        final var emptyFundamentalMatrix2 = new FundamentalMatrix();

        final var fundamentalMatrix1 = createRandomFundamentalMatrix();
        final var fundamentalMatrix2 = createRandomFundamentalMatrix();

        comparator = new EpipolarDistanceFundamentalMatrixComparator(emptyFundamentalMatrix1, emptyFundamentalMatrix2);

        // check default values
        assertSame(emptyFundamentalMatrix1, comparator.getGroundTruthFundamentalMatrix());
        assertSame(emptyFundamentalMatrix2, comparator.getOtherFundamentalMatrix());
        assertNull(comparator.getListener());
        assertFalse(comparator.isLocked());
        // fundamental matrices are not defined
        assertFalse(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR, comparator.getType());
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, comparator.getMinX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, comparator.getMaxX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, comparator.getMinY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, comparator.getMaxY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, comparator.getNSamples(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_ITERATIONS_FACTOR,
                comparator.getMaxIterationsFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_PROGRESS_DELTA, comparator.getProgressDelta(),
                0.0);

        comparator = new EpipolarDistanceFundamentalMatrixComparator(fundamentalMatrix1, emptyFundamentalMatrix2);

        // check default values
        assertSame(fundamentalMatrix1, comparator.getGroundTruthFundamentalMatrix());
        assertSame(emptyFundamentalMatrix2, comparator.getOtherFundamentalMatrix());
        assertNull(comparator.getListener());
        assertFalse(comparator.isLocked());
        // fundamental matrices are not defined
        assertFalse(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR, comparator.getType());
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, comparator.getMinX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, comparator.getMaxX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, comparator.getMinY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, comparator.getMaxY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, comparator.getNSamples(),
                0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_ITERATIONS_FACTOR,
                comparator.getMaxIterationsFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_PROGRESS_DELTA, comparator.getProgressDelta(),
                0.0);

        comparator = new EpipolarDistanceFundamentalMatrixComparator(emptyFundamentalMatrix1, fundamentalMatrix2);

        // check default values
        assertSame(emptyFundamentalMatrix1, comparator.getGroundTruthFundamentalMatrix());
        assertSame(fundamentalMatrix2, comparator.getOtherFundamentalMatrix());
        assertNull(comparator.getListener());
        assertFalse(comparator.isLocked());
        assertFalse(comparator.isReady()); //fundamental matrices are not defined
        assertEquals(FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR, comparator.getType());
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, comparator.getMinX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, comparator.getMaxX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, comparator.getMinY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, comparator.getMaxY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, comparator.getNSamples(),
                0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_ITERATIONS_FACTOR,
                comparator.getMaxIterationsFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_PROGRESS_DELTA, comparator.getProgressDelta(),
                0.0);

        comparator = new EpipolarDistanceFundamentalMatrixComparator(fundamentalMatrix1, fundamentalMatrix2);

        // check default values
        assertSame(fundamentalMatrix1, comparator.getGroundTruthFundamentalMatrix());
        assertSame(fundamentalMatrix2, comparator.getOtherFundamentalMatrix());
        assertNull(comparator.getListener());
        assertFalse(comparator.isLocked());
        assertTrue(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR, comparator.getType());
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, comparator.getMinX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, comparator.getMaxX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, comparator.getMinY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, comparator.getMaxY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, comparator.getNSamples(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_ITERATIONS_FACTOR,
                comparator.getMaxIterationsFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_PROGRESS_DELTA, comparator.getProgressDelta(),
                0.0);

        // test constructor with listener
        comparator = new EpipolarDistanceFundamentalMatrixComparator(this);

        // check default values
        assertNull(comparator.getGroundTruthFundamentalMatrix());
        assertNull(comparator.getOtherFundamentalMatrix());
        assertSame(this, comparator.getListener());
        assertFalse(comparator.isLocked());
        assertFalse(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR, comparator.getType());
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, comparator.getMinX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, comparator.getMaxX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, comparator.getMinY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, comparator.getMaxY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, comparator.getNSamples(),
                0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_ITERATIONS_FACTOR,
                comparator.getMaxIterationsFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_PROGRESS_DELTA, comparator.getProgressDelta(),
                0.0);

        // test constructor with fundamental matrices and listener
        comparator = new EpipolarDistanceFundamentalMatrixComparator(emptyFundamentalMatrix1, emptyFundamentalMatrix2,
                this);

        // check default values
        assertSame(emptyFundamentalMatrix1, comparator.getGroundTruthFundamentalMatrix());
        assertSame(emptyFundamentalMatrix2, comparator.getOtherFundamentalMatrix());
        assertSame(this, comparator.getListener());
        assertFalse(comparator.isLocked());
        // fundamental matrices are not defined
        assertFalse(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR, comparator.getType());
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, comparator.getMinX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, comparator.getMaxX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, comparator.getMinY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, comparator.getMaxY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, comparator.getNSamples(),
                0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_ITERATIONS_FACTOR,
                comparator.getMaxIterationsFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_PROGRESS_DELTA, comparator.getProgressDelta(),
                0.0);

        comparator = new EpipolarDistanceFundamentalMatrixComparator(fundamentalMatrix1, emptyFundamentalMatrix2,
                this);

        // check default values
        assertSame(fundamentalMatrix1, comparator.getGroundTruthFundamentalMatrix());
        assertSame(emptyFundamentalMatrix2, comparator.getOtherFundamentalMatrix());
        assertSame(this, comparator.getListener());
        assertFalse(comparator.isLocked());
        // fundamental matrices are not defined
        assertFalse(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR, comparator.getType());
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, comparator.getMinX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, comparator.getMaxX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, comparator.getMinY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, comparator.getMaxY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, comparator.getNSamples(),
                0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_ITERATIONS_FACTOR,
                comparator.getMaxIterationsFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_PROGRESS_DELTA, comparator.getProgressDelta(),
                0.0);

        comparator = new EpipolarDistanceFundamentalMatrixComparator(emptyFundamentalMatrix1, fundamentalMatrix2,
                this);

        // check default values
        assertSame(emptyFundamentalMatrix1, comparator.getGroundTruthFundamentalMatrix());
        assertSame(fundamentalMatrix2, comparator.getOtherFundamentalMatrix());
        assertSame(this, comparator.getListener());
        assertFalse(comparator.isLocked());
        // fundamental matrices are not defined
        assertFalse(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR, comparator.getType());
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, comparator.getMinX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, comparator.getMaxX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, comparator.getMinY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, comparator.getMaxY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, comparator.getNSamples(),
                0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_ITERATIONS_FACTOR,
                comparator.getMaxIterationsFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_PROGRESS_DELTA, comparator.getProgressDelta(),
                0.0);

        comparator = new EpipolarDistanceFundamentalMatrixComparator(fundamentalMatrix1, fundamentalMatrix2,
                this);

        // check default values
        assertSame(fundamentalMatrix1, comparator.getGroundTruthFundamentalMatrix());
        assertSame(fundamentalMatrix2, comparator.getOtherFundamentalMatrix());
        assertSame(this, comparator.getListener());
        assertFalse(comparator.isLocked());
        assertTrue(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR, comparator.getType());
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, comparator.getMinX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, comparator.getMaxX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, comparator.getMinY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, comparator.getMaxY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, comparator.getNSamples(),
                0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_ITERATIONS_FACTOR,
                comparator.getMaxIterationsFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_PROGRESS_DELTA, comparator.getProgressDelta(),
                0.0);
    }

    @Test
    void testGetSetMinMaxX() throws LockedException {
        final var comparator = new EpipolarDistanceFundamentalMatrixComparator();

        // check default values
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_X, comparator.getMinX(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_X, comparator.getMaxX(), 0.0);

        // set new values
        comparator.setMinMaxX(5.0, 10.0);

        // check correctness
        assertEquals(5.0, comparator.getMinX(), 0.0);
        assertEquals(10.0, comparator.getMaxX(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> comparator.setMinMaxX(5.0, 5.0));
    }

    @Test
    void testGetSetMinMaxY() throws LockedException {
        final var comparator = new EpipolarDistanceFundamentalMatrixComparator();

        // check default values
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_Y, comparator.getMinY(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_Y, comparator.getMaxY(), 0.0);

        // set new values
        comparator.setMinMaxY(5.0, 10.0);

        // check correctness
        assertEquals(5.0, comparator.getMinY(), 0.0);
        assertEquals(10.0, comparator.getMaxY(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> comparator.setMinMaxY(5.0, 5.0));
    }

    @Test
    void testGetSetNSamples() throws LockedException {
        final var comparator = new EpipolarDistanceFundamentalMatrixComparator();

        // check default values
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_N_SAMPLES, comparator.getNSamples());

        // set new value
        comparator.setNSamples(1);

        // check correctness
        assertEquals(1, comparator.getNSamples());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> comparator.setNSamples(0));
    }

    @Test
    void testGetSetMinMaxHorizontalDisparityFactor() throws LockedException {
        final var comparator = new EpipolarDistanceFundamentalMatrixComparator();

        // check default values
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinHorizontalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxHorizontalDisparityFactor(), 0.0);

        // set new value
        comparator.setMinMaxHorizontalDisparityFactor(-0.2, 0.2);

        // check correctness
        assertEquals(-0.2, comparator.getMinHorizontalDisparityFactor(), 0.0);
        assertEquals(0.2, comparator.getMaxHorizontalDisparityFactor(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> comparator.setMinMaxHorizontalDisparityFactor(0.2, -0.2));
    }

    @Test
    void testGetSetMinMaxVerticalDisparityFactor() throws LockedException {
        final var comparator = new EpipolarDistanceFundamentalMatrixComparator();

        // check default values
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MIN_DISPARITY_FACTOR,
                comparator.getMinVerticalDisparityFactor(), 0.0);
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_DISPARITY_FACTOR,
                comparator.getMaxVerticalDisparityFactor(), 0.0);

        // set new value
        comparator.setMinMaxVerticalDisparityFactor(-0.2, 0.2);

        // check correctness
        assertEquals(-0.2, comparator.getMinVerticalDisparityFactor(), 0.0);
        assertEquals(0.2, comparator.getMaxVerticalDisparityFactor(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> comparator.setMinMaxVerticalDisparityFactor(0.2, -0.2));
    }

    @Test
    void testGetSetMaxIterationsFactor() throws LockedException {
        final var comparator = new EpipolarDistanceFundamentalMatrixComparator();

        // check default value
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_MAX_ITERATIONS_FACTOR,
                comparator.getMaxIterationsFactor(), 0.0);

        // set new value
        comparator.setMaxIterationsFactor(1.0);

        // check correctness
        assertEquals(1.0, comparator.getMaxIterationsFactor(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> comparator.setMaxIterationsFactor(0.5));
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var comparator = new EpipolarDistanceFundamentalMatrixComparator();

        // check default value
        assertEquals(EpipolarDistanceFundamentalMatrixComparator.DEFAULT_PROGRESS_DELTA, comparator.getProgressDelta(),
                0.0);

        // set new value
        comparator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5, comparator.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> comparator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> comparator.setProgressDelta(2.0f));
    }

    @Test
    void testCompare() throws InvalidPairOfCamerasException, NotReadyException, LockedException,
            FundamentalMatrixComparatorException {
        final var randomizer = new UniformRandomizer();
        final var alphaEuler1 = 0.0;
        final var betaEuler1 = 0.0;
        final var gammaEuler1 = 0.0;
        final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final var skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final var skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var horizontalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var cameraCenter1 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var cameraCenter2 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
        final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

        final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
        final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

        final var camera1 = new PinholeCamera(intrinsic1, rotation1, cameraCenter1);
        final var camera2 = new PinholeCamera(intrinsic2, rotation2, cameraCenter2);

        final var fundamentalMatrix1 = new FundamentalMatrix(camera1, camera2);
        final var fundamentalMatrix2 = new FundamentalMatrix(camera1, camera2);

        final var comparator = new EpipolarDistanceFundamentalMatrixComparator(fundamentalMatrix1, fundamentalMatrix2,
                this);

        // check status
        assertFalse(comparator.isLocked());
        assertTrue(comparator.isReady());
        assertEquals(0, compareStart);
        assertEquals(0, compareEnd);

        // compare
        assertEquals(0.0, comparator.compare(), ABSOLUTE_ERROR);

        // check correctness
        assertTrue(comparator.isReady());
        assertFalse(comparator.isLocked());
        assertEquals(1, compareStart);
        assertEquals(1, compareEnd);
        assertTrue(compareProgressChange > 0);
        reset();
    }

    private void reset() {
        compareStart = compareEnd = compareProgressChange = 0;
    }

    @Override
    public void onCompareStart(final FundamentalMatrixComparator comparator) {
        compareStart++;
        testLocked((EpipolarDistanceFundamentalMatrixComparator) comparator);
    }

    @Override
    public void onCompareEnd(final FundamentalMatrixComparator comparator) {
        compareEnd++;
        testLocked((EpipolarDistanceFundamentalMatrixComparator) comparator);
    }

    @Override
    public void onCompareProgressChange(final FundamentalMatrixComparator comparator, final float progress) {
        compareProgressChange++;
        testLocked((EpipolarDistanceFundamentalMatrixComparator) comparator);
    }

    private static void testLocked(final EpipolarDistanceFundamentalMatrixComparator comparator) {
        assertTrue(comparator.isLocked());
        assertThrows(LockedException.class, () -> comparator.setGroundTruthFundamentalMatrix(null));
        assertThrows(LockedException.class, () -> comparator.setOtherFundamentalMatrix(null));
        assertThrows(LockedException.class, () -> comparator.setListener(null));
        assertThrows(LockedException.class, comparator::compare);
        assertThrows(LockedException.class, () -> comparator.setMinMaxX(0.0, 100.0));
        assertThrows(LockedException.class, () -> comparator.setMinMaxY(0.0, 100.0));
        assertThrows(LockedException.class, () -> comparator.setNSamples(1));
        assertThrows(LockedException.class, () -> comparator.setMinMaxHorizontalDisparityFactor(-0.2, 0.2));
        assertThrows(LockedException.class, () -> comparator.setMinMaxVerticalDisparityFactor(-0.2, 0.2));
        assertThrows(LockedException.class, () -> comparator.setMaxIterationsFactor(2.0));
        assertThrows(LockedException.class, () -> comparator.setProgressDelta(0.5f));
    }

    private FundamentalMatrix createRandomFundamentalMatrix() throws AlgebraException,
            InvalidFundamentalMatrixException {
        FundamentalMatrix fundamentalMatrix;
        int rank;
        do {
            var internalMatrix = Matrix.createWithUniformRandomValues(FundamentalMatrix.FUNDAMENTAL_MATRIX_ROWS,
                    FundamentalMatrix.FUNDAMENTAL_MATRIX_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            // ensure that internal matrix has rank 2
            final var decomposer = new SingularValueDecomposer(internalMatrix);
            decomposer.decompose();

            // if rank is less than 2 we need to
            // pick another random matrix
            rank = decomposer.getRank();

            final var u = decomposer.getU();
            final var w = decomposer.getW();
            final var v = decomposer.getV();
            final var transV = v.transposeAndReturnNew();

            // set last element to 0 to force rank 2
            w.setElementAt(2, 2, 0.0);

            internalMatrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));

            fundamentalMatrix = new FundamentalMatrix(internalMatrix);
        } while (rank < 2);

        return fundamentalMatrix;
    }
}
