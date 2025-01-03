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
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class RANSACRadialDistortionRobustEstimatorTest implements RadialDistortionRobustEstimatorListener {

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

    @Test
    void testConstructor() {
        // test constructor without parameters
        var estimator = new RANSACRadialDistortionRobustEstimator();

        // check correctness
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with listener
        estimator = new RANSACRadialDistortionRobustEstimator(this);

        // check correctness
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with points
        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        for (var i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        estimator = new RANSACRadialDistortionRobustEstimator(distortedPoints, undistortedPoints);

        // check correctness
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class,
                () -> new RANSACRadialDistortionRobustEstimator(emptyPoints, undistortedPoints));
        assertThrows(IllegalArgumentException.class,
                () -> new RANSACRadialDistortionRobustEstimator(emptyPoints, emptyPoints));
        assertThrows(IllegalArgumentException.class,
                () -> new RANSACRadialDistortionRobustEstimator(null, undistortedPoints));
        assertThrows(IllegalArgumentException.class,
                () -> new RANSACRadialDistortionRobustEstimator(distortedPoints, null));

        // test constructor with points and listener
        estimator = new RANSACRadialDistortionRobustEstimator(distortedPoints, undistortedPoints, this);

        // check correctness
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RANSACRadialDistortionRobustEstimator(emptyPoints,
                undistortedPoints, this));
        assertThrows(IllegalArgumentException.class, () -> new RANSACRadialDistortionRobustEstimator(emptyPoints,
                emptyPoints, this));
        assertThrows(IllegalArgumentException.class, () -> new RANSACRadialDistortionRobustEstimator(null,
                undistortedPoints, this));
        assertThrows(IllegalArgumentException.class, () -> new RANSACRadialDistortionRobustEstimator(distortedPoints,
                null, this));

        // test constructor with points and center
        final var center = Point2D.create();

        estimator = new RANSACRadialDistortionRobustEstimator(distortedPoints, undistortedPoints, center);

        // check correctness
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(center.getInhomX(), estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(center.getInhomY(), estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RANSACRadialDistortionRobustEstimator(emptyPoints,
                undistortedPoints, center));
        assertThrows(IllegalArgumentException.class, () -> new RANSACRadialDistortionRobustEstimator(emptyPoints,
                emptyPoints, center));
        assertThrows(IllegalArgumentException.class, () -> new RANSACRadialDistortionRobustEstimator(null,
                undistortedPoints, center));
        assertThrows(IllegalArgumentException.class, () -> new RANSACRadialDistortionRobustEstimator(distortedPoints,
                null, center));

        // test constructor with points, center and listener
        estimator = new RANSACRadialDistortionRobustEstimator(distortedPoints, undistortedPoints, center, this);

        // check correctness
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.RANSAC, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(center.getInhomX(), estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(center.getInhomY(), estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RANSACRadialDistortionRobustEstimator(emptyPoints,
                undistortedPoints, center, this));
        assertThrows(IllegalArgumentException.class, () -> new RANSACRadialDistortionRobustEstimator(emptyPoints,
                emptyPoints, center, this));
        assertThrows(IllegalArgumentException.class, () -> new RANSACRadialDistortionRobustEstimator(null,
                undistortedPoints, center, this));
        assertThrows(IllegalArgumentException.class, () -> new RANSACRadialDistortionRobustEstimator(distortedPoints,
                null, center, this));
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var estimator = new RANSACRadialDistortionRobustEstimator();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        final var qualityScores = new double[1];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertNull(estimator.getQualityScores());
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var estimator = new RANSACRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);

        // set new value
        estimator.setThreshold(10.0);

        // check correctness
        assertEquals(10.0, estimator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setThreshold(0.0));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new RANSACRadialDistortionRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // Set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var estimator = new RANSACRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5, estimator.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(2.0f));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var estimator = new RANSACRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.75);

        // check correctness
        assertEquals(0.75, estimator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(2.0));
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var estimator = new RANSACRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RANSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(10);

        // check correctness
        assertEquals(10, estimator.getMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testGetSetPoints() throws LockedException {
        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        for (var i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        final var estimator = new RANSACRadialDistortionRobustEstimator();

        // check default values
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());

        // set new value
        estimator.setPoints(distortedPoints, undistortedPoints);

        // check correctness
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(emptyPoints, undistortedPoints));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(emptyPoints, emptyPoints));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(null, undistortedPoints));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(distortedPoints, null));
    }

    @Test
    void testGetSetDistortionCenter() throws LockedException {
        final var estimator = new RANSACRadialDistortionRobustEstimator();

        // check default value
        assertNull(estimator.getDistortionCenter());

        // set new value
        final var center = Point2D.create();
        estimator.setDistortionCenter(center);

        // check correctness
        assertSame(estimator.getDistortionCenter(), center);
    }

    @Test
    void testGetSetHorizontalFocalLength() throws LockedException {
        final var estimator = new RANSACRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getHorizontalFocalLength(),
                0.0);

        // set new value
        estimator.setHorizontalFocalLength(2.0);

        // check correctness
        assertEquals(2.0, estimator.getHorizontalFocalLength(), 0.0);
    }

    @Test
    void testGetSetVerticalFocalLength() throws LockedException {
        final var estimator = new RANSACRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, estimator.getVerticalFocalLength(),
                0.0);

        // set new value
        estimator.setVerticalFocalLength(2.0);

        // check correctness
        assertEquals(2.0, estimator.getVerticalFocalLength(), 0.0);
    }

    @Test
    void testGetSetSkew() throws LockedException {
        final var estimator = new RANSACRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);

        // set new value
        estimator.setSkew(1.0);

        // check correctness
        assertEquals(1.0, estimator.getSkew(), 0.0);
    }

    @Test
    void testGetSetIntrinsic() throws LockedException {
        final var estimator = new RANSACRadialDistortionRobustEstimator();

        // check default value
        var intrinsic = estimator.getIntrinsic();

        assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, intrinsic.getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH, intrinsic.getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, intrinsic.getSkewness(), 0.0);

        // set new value
        intrinsic = new PinholeCameraIntrinsicParameters(2.0, 3.0,
                4.0, 5.0, 6.0);
        estimator.setIntrinsic(intrinsic);

        // check correctness
        assertEquals(4.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(5.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
        assertEquals(2.0, intrinsic.getHorizontalFocalLength(), 0.0);
        assertEquals(3.0, intrinsic.getVerticalFocalLength(), 0.0);
        assertEquals(6.0, intrinsic.getSkewness(), 0.0);

        // set again
        estimator.setIntrinsic(new InhomogeneousPoint2D(6.0, 5.0), 4.0, 3.0,
                2.0);

        // check correctness
        assertEquals(6.0, estimator.getDistortionCenter().getInhomX(), 0.0);
        assertEquals(5.0, estimator.getDistortionCenter().getInhomY(), 0.0);
        assertEquals(4.0, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(3.0, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(2.0, estimator.getSkew(), 0.0);
    }

    @Test
    void testGetSetNumKParams() throws LockedException {
        final var estimator = new RANSACRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());

        // set new value
        estimator.setNumKParams(3);

        // check correctness
        assertEquals(3, estimator.getNumKParams());
    }

    @Test
    void testEstimate() throws NotSupportedException, LockedException, NotReadyException, RobustEstimatorException,
            DistortionException {
        final var randomizer = new UniformRandomizer();

        for (var j = 0; j < TIMES; j++) {
            final var k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
            final var k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);

            final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

            final var distortion = new RadialDistortion(k1, k2, center);

            final var nPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            final var distortedPointsWithError = new ArrayList<Point2D>();
            final var distortedPoints = new ArrayList<Point2D>();
            final var undistortedPoints = new ArrayList<Point2D>();
            Point2D distortedPoint;
            Point2D distortedPointWithError;
            Point2D undistortedPoint;
            for (var i = 0; i < nPoints; i++) {
                undistortedPoint = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                        randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

                distortedPoint = distortion.distort(undistortedPoint);
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    distortedPointWithError = new InhomogeneousPoint2D(distortedPoint.getInhomX() + errorX,
                            distortedPoint.getInhomY() + errorY);
                } else {
                    // point is inlier
                    distortedPointWithError = distortedPoint;
                }
                distortedPoints.add(distortedPoint);
                distortedPointsWithError.add(distortedPointWithError);
                undistortedPoints.add(undistortedPoint);
            }

            final var estimator = new RANSACRadialDistortionRobustEstimator(distortedPointsWithError, undistortedPoints,
                    this);

            estimator.setIntrinsic(distortion.getIntrinsic());
            estimator.setThreshold(THRESHOLD);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final var distortion2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation
            assertEquals(k1, distortion2.getK1(), ABSOLUTE_ERROR);
            assertEquals(k2, distortion2.getK2(), ABSOLUTE_ERROR);
            assertEquals(center, distortion2.getCenter());

            for (var i = 0; i < nPoints; i++) {
                assertEquals(0.0,
                        distortedPoints.get(i).distanceTo(distortion2.distort(undistortedPoints.get(i))),
                        ABSOLUTE_ERROR);
            }
        }
    }

    @Override
    public void onEstimateStart(final RadialDistortionRobustEstimator estimator) {
        estimateStart++;
        checkLocked((RANSACRadialDistortionRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final RadialDistortionRobustEstimator estimator) {
        estimateEnd++;
        checkLocked((RANSACRadialDistortionRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(final RadialDistortionRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        checkLocked((RANSACRadialDistortionRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(final RadialDistortionRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        checkLocked((RANSACRadialDistortionRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration = estimateProgressChange = 0;
    }

    private void checkLocked(final RANSACRadialDistortionRobustEstimator estimator) {
        assertThrows(LockedException.class, () -> estimator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> estimator.setDistortionCenter(null));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, () -> estimator.setMaxIterations(10));
        assertThrows(LockedException.class, () -> estimator.setPoints(null, null));
        assertThrows(LockedException.class, () -> estimator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> estimator.setThreshold(0.5));
        assertThrows(LockedException.class, estimator::estimate);
        assertTrue(estimator.isLocked());
    }
}
