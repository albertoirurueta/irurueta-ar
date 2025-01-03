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
import com.irurueta.ar.calibration.RadialDistortionException;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.NotSupportedException;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class WeightedRadialDistortionEstimatorTest implements RadialDistortionEstimatorListener {

    private static final double MIN_POINT_VALUE = -2.0;
    private static final double MAX_POINT_VALUE = 2.0;

    private static final double MIN_PARAM_VALUE = -1e-3;
    private static final double MAX_PARAM_VALUE = 1e-3;

    private static final double MIN_WEIGHT_VALUE = 0.5;
    private static final double MAX_WEIGHT_VALUE = 1.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final int NUM_POINTS = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimationProgressChange;

    @Test
    void testConstructor() {
        // test constructor without parameters
        var estimator = new WeightedRadialDistortionEstimator();

        // check correctness
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getIntrinsic().getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        //noinspection EqualsWithItself
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getMinNumberOfMatchedPoints());
        assertNull(estimator.getWeights());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS, estimator.getMaxPoints());
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS, estimator.isSortWeightsEnabled());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionEstimatorType.WEIGHTED_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());

        // test constructor with listener
        estimator = new WeightedRadialDistortionEstimator(this);

        // check correctness
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getIntrinsic().getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        //noinspection EqualsWithItself
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getMinNumberOfMatchedPoints());
        assertNull(estimator.getWeights());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS, estimator.getMaxPoints());
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS, estimator.isSortWeightsEnabled());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionEstimatorType.WEIGHTED_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());

        // test constructor with distorted/undistorted points
        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        final var weights = new double[estimator.getMinNumberOfMatchedPoints()];
        for (var i = 0; i < estimator.getMinNumberOfMatchedPoints(); i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        estimator = new WeightedRadialDistortionEstimator(distortedPoints, undistortedPoints, weights);

        // check correctness
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getIntrinsic().getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        //noinspection EqualsWithItself
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getMinNumberOfMatchedPoints());
        assertSame(estimator.getWeights(), weights);
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.areWeightsAvailable());
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS, estimator.getMaxPoints());
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS, estimator.isSortWeightsEnabled());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionEstimatorType.WEIGHTED_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        final var shortWeights = new double[1];
        assertThrows(IllegalArgumentException.class,
                () -> new WeightedRadialDistortionEstimator(emptyPoints, undistortedPoints, weights));
        assertThrows(IllegalArgumentException.class,
                () -> new WeightedRadialDistortionEstimator(emptyPoints, emptyPoints, weights));
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(null,
                undistortedPoints, weights));
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(distortedPoints,
                null, weights));
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(distortedPoints,
                undistortedPoints, shortWeights));

        // test constructor with distorted/undistorted points and listener
        estimator = new WeightedRadialDistortionEstimator(distortedPoints, undistortedPoints, weights, this);

        // check correctness
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getIntrinsic().getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        //noinspection EqualsWithItself
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getMinNumberOfMatchedPoints());
        assertSame(weights, estimator.getWeights());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.areWeightsAvailable());
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS, estimator.getMaxPoints());
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS, estimator.isSortWeightsEnabled());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionEstimatorType.WEIGHTED_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(emptyPoints,
                undistortedPoints, weights, this));
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(emptyPoints,
                emptyPoints, weights, this));
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(
                null, undistortedPoints, weights, this));
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(distortedPoints,
                null, weights, this));
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(distortedPoints,
                undistortedPoints, shortWeights, this));

        // test constructor with distortion center
        final var center = Point2D.create();
        estimator = new WeightedRadialDistortionEstimator(center);

        // check correctness
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getIntrinsic().getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        //noinspection EqualsWithItself
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getMinNumberOfMatchedPoints());
        assertNull(estimator.getWeights());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS, estimator.getMaxPoints());
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS, estimator.isSortWeightsEnabled());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionEstimatorType.WEIGHTED_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());

        // test constructor with distortion center and listener
        estimator = new WeightedRadialDistortionEstimator(center, this);

        // check correctness
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getIntrinsic().getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        //noinspection EqualsWithItself
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getMinNumberOfMatchedPoints());
        assertNull(estimator.getWeights());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS, estimator.getMaxPoints());
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS, estimator.isSortWeightsEnabled());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionEstimatorType.WEIGHTED_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());

        // test constructor with distorted/undistorted points and distortion
        // center
        estimator = new WeightedRadialDistortionEstimator(distortedPoints, undistortedPoints, weights, center);
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getIntrinsic().getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        //noinspection EqualsWithItself
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getMinNumberOfMatchedPoints());
        assertSame(weights, estimator.getWeights());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.areWeightsAvailable());
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS, estimator.getMaxPoints());
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS, estimator.isSortWeightsEnabled());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionEstimatorType.WEIGHTED_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(emptyPoints,
                undistortedPoints, weights, center));
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(emptyPoints,
                emptyPoints, weights, center));
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(
                null, undistortedPoints, weights, center));
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(distortedPoints,
                null, weights, center));
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(distortedPoints,
                undistortedPoints, shortWeights, center));

        // test constructor with distorted/undistorted points and distortion
        // center
        estimator = new WeightedRadialDistortionEstimator(distortedPoints, undistortedPoints, weights, center,
                this);
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getIntrinsic().getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        //noinspection EqualsWithItself
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getMinNumberOfMatchedPoints());
        assertSame(weights, estimator.getWeights());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.areWeightsAvailable());
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS, estimator.getMaxPoints());
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS, estimator.isSortWeightsEnabled());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionEstimatorType.WEIGHTED_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(emptyPoints,
                undistortedPoints, weights, center, this));
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(emptyPoints,
                emptyPoints, weights, center, this));
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(
                null, undistortedPoints, weights, center, this));
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(distortedPoints,
                null, weights, center, this));
        assertThrows(IllegalArgumentException.class, () -> new WeightedRadialDistortionEstimator(distortedPoints,
                undistortedPoints, shortWeights, center, this));
    }

    @Test
    void testGetSetListsAndWeights() throws LockedException {
        final var estimator = new WeightedRadialDistortionEstimator();

        // check default values
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getWeights());

        // set new value
        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        final var weights = new double[estimator.getMinNumberOfMatchedPoints()];
        for (var i = 0; i < estimator.getMinNumberOfMatchedPoints(); i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        estimator.setPointsAndWeights(distortedPoints, undistortedPoints, weights);

        // check correctness
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(weights, estimator.getWeights());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        final var shortWeights = new double[1];
        assertThrows(IllegalArgumentException.class,
                () -> estimator.setPointsAndWeights(emptyPoints, undistortedPoints, weights));
        assertThrows(IllegalArgumentException.class,
                () -> estimator.setPointsAndWeights(emptyPoints, emptyPoints, weights));
        assertThrows(IllegalArgumentException.class,
                () -> estimator.setPointsAndWeights(null, undistortedPoints, weights));
        assertThrows(IllegalArgumentException.class,
                () -> estimator.setPointsAndWeights(distortedPoints, null, weights));
        assertThrows(IllegalArgumentException.class,
                () -> estimator.setPointsAndWeights(distortedPoints, undistortedPoints, shortWeights));
    }

    @Test
    void testAreValidPointsAndWeights() {
        final var estimator = new WeightedRadialDistortionEstimator();

        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        final var weights = new double[estimator.getMinNumberOfMatchedPoints()];
        for (var i = 0; i < estimator.getMinNumberOfMatchedPoints(); i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        final var emptyPoints = new ArrayList<Point2D>();

        assertTrue(estimator.areValidPointsAndWeights(distortedPoints, undistortedPoints, weights));
        assertFalse(estimator.areValidPointsAndWeights(emptyPoints, undistortedPoints, weights));
        assertFalse(estimator.areValidPointsAndWeights(distortedPoints, emptyPoints, weights));
        assertFalse(estimator.areValidPointsAndWeights(null, undistortedPoints, weights));
        assertFalse(estimator.areValidPointsAndWeights(distortedPoints, null, weights));
    }

    @Test
    void testGetSetMaxPoints() throws LockedException {
        final var estimator = new WeightedRadialDistortionEstimator();

        // check default value
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_MAX_POINTS, estimator.getMaxPoints());

        // set new value
        estimator.setMaxPoints(10);

        // check correctness
        assertEquals(10, estimator.getMaxPoints());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxPoints(1));
    }

    @Test
    void testIsSetSortWeightsEnabled() throws LockedException {
        final var estimator = new WeightedRadialDistortionEstimator();

        // check default value
        assertEquals(WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS, estimator.isSortWeightsEnabled());

        // set new value
        estimator.setSortWeightsEnabled(!WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS);

        // check correctness
        assertEquals(!WeightedRadialDistortionEstimator.DEFAULT_SORT_WEIGHTS, estimator.isSortWeightsEnabled());
    }

    @Test
    void testGetSetDistortionCenter() throws LockedException {
        final var estimator = new WeightedRadialDistortionEstimator();

        // check default value
        assertNull(estimator.getDistortionCenter());
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);

        // set new value
        final var center = Point2D.create();
        center.setInhomogeneousCoordinates(1.0, 2.0);
        estimator.setDistortionCenter(center);

        // check correctness
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(center.getInhomX(), estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(center.getInhomY(), estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
    }

    @Test
    void testGetSetHorizontalFocalLength() throws LockedException, RadialDistortionException {
        final var estimator = new WeightedRadialDistortionEstimator();

        // check default value
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);

        // set new value
        estimator.setHorizontalFocalLength(2.0);

        // check correctness
        assertEquals(2.0, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(2.0, estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);

        // Force RadialDistortionException
        assertThrows(RadialDistortionException.class, () -> estimator.setHorizontalFocalLength(0.0));
    }

    @Test
    void testGetSetVerticalFocalLength() throws LockedException, RadialDistortionException {
        final var estimator = new WeightedRadialDistortionEstimator();

        // check default value
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getIntrinsic().getVerticalFocalLength(),
                0.0);

        // set new value
        estimator.setVerticalFocalLength(2.0);

        // check correctness
        assertEquals(2.0, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(2.0, estimator.getIntrinsic().getVerticalFocalLength(), 0.0);

        // Force RadialDistortionException
        assertThrows(RadialDistortionException.class, () -> estimator.setVerticalFocalLength(0.0));
    }

    @Test
    void testGetSetSkew() throws LockedException {
        final var estimator = new WeightedRadialDistortionEstimator();

        // check default value
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);

        // set new value
        estimator.setSkew(1.0);

        // check correctness
        assertEquals(1.0, estimator.getSkew(), 0.0);
        assertEquals(1.0, estimator.getIntrinsic().getSkewness(), 0.0);
    }

    @Test
    void testSetIntrinsic() throws LockedException, RadialDistortionException {
        final var estimator = new WeightedRadialDistortionEstimator();

        // check default values
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);

        // set new values
        final var center = new InhomogeneousPoint2D(1.0, 2.0);

        estimator.setIntrinsic(center, 3.0, 4.0, 5.0);

        // check correctness
        assertEquals(1.0, estimator.getDistortionCenter().getInhomX(), 0.0);
        assertEquals(2.0, estimator.getDistortionCenter().getInhomY(), 0.0);
        assertEquals(3.0, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(4.0, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(5.0, estimator.getSkew(), 0.0);

        // Force RadialDistortionException
        assertThrows(RadialDistortionException.class,
                () -> estimator.setIntrinsic(center, 0.0, 0.0, 1.0));

        // set new values
        var intrinsic = new PinholeCameraIntrinsicParameters(1.0, 2.0,
                3.0, 4.0, 5.0);
        estimator.setIntrinsic(intrinsic);

        // check correctness
        assertEquals(3.0, estimator.getDistortionCenter().getInhomX(), 0.0);
        assertEquals(4.0, estimator.getDistortionCenter().getInhomY(), 0.0);
        assertEquals(1.0, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(2.0, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(5.0, estimator.getSkew(), 0.0);

        // Force RadialDistortionException
        final var wrongIntrinsic = new PinholeCameraIntrinsicParameters(0.0, 0.0,
                3.0, 4.0, 5.0);
        assertThrows(RadialDistortionException.class, () -> estimator.setIntrinsic(wrongIntrinsic));
    }

    @Test
    void testGetSetNumKParamsAndGetMinNumberOfMatchedPoints() throws LockedException {
        final var estimator = new WeightedRadialDistortionEstimator();

        // check default values
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getNumKParams());

        // set new value
        estimator.setNumKParams(3);

        // check correctness
        assertEquals(3, estimator.getNumKParams());
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getNumKParams());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setNumKParams(0));
    }

    @Test
    void testEstimate() throws NotSupportedException, LockedException, NotReadyException,
            RadialDistortionEstimatorException, DistortionException {
        final var randomizer = new UniformRandomizer();

        final var k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        final var k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);

        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

        final var distortion = new RadialDistortion(k1, k2, center);

        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        final var weights = new double[NUM_POINTS];
        Point2D undistortedPoint;
        for (var i = 0; i < NUM_POINTS; i++) {
            undistortedPoint = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

            undistortedPoints.add(undistortedPoint);
            distortedPoints.add(distortion.distort(undistortedPoint));
            weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
        }

        final var estimator = new WeightedRadialDistortionEstimator(distortedPoints, undistortedPoints, weights, center,
                this);

        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(0, estimateStart);
        assertEquals(0, estimateEnd);
        assertEquals(0, estimationProgressChange);

        final var distortion2 = estimator.estimate();

        // check correctness
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(1, estimateStart);
        assertEquals(1, estimateEnd);
        assertTrue(estimationProgressChange >= 0);
        reset();

        assertEquals(k1, distortion2.getK1(), ABSOLUTE_ERROR);
        assertEquals(k2, distortion2.getK2(), ABSOLUTE_ERROR);
        assertEquals(distortion2.getCenter(), center);
    }

    @Override
    public void onEstimateStart(final RadialDistortionEstimator estimator) {
        estimateStart++;
        testLocked((WeightedRadialDistortionEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final RadialDistortionEstimator estimator) {
        estimateEnd++;
        testLocked((WeightedRadialDistortionEstimator) estimator);
    }

    @Override
    public void onEstimationProgressChange(final RadialDistortionEstimator estimator, final float progress) {
        estimationProgressChange++;
        testLocked((WeightedRadialDistortionEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimationProgressChange = 0;
    }

    private static void testLocked(final WeightedRadialDistortionEstimator estimator) {
        assertThrows(LockedException.class, () -> estimator.setPoints(null, null));
        assertThrows(LockedException.class, () -> estimator.setDistortionCenter(null));
        assertThrows(LockedException.class,
                () -> estimator.setPointsAndWeights(null, null, null));
        assertThrows(LockedException.class, () -> estimator.setMaxPoints(NUM_POINTS));
        assertThrows(LockedException.class, () -> estimator.setSortWeightsEnabled(true));
    }
}
