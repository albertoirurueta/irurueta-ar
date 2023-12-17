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
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class MSACRadialDistortionRobustEstimatorTest implements RadialDistortionRobustEstimatorListener {

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
    public void testConstructor() {
        // test constructor without parameters
        MSACRadialDistortionRobustEstimator estimator =
                new MSACRadialDistortionRobustEstimator();

        // check correctness
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(0.0,
                estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0,
                estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS,
                estimator.getNumKParams());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with listener
        estimator = new MSACRadialDistortionRobustEstimator(this);

        // check correctness
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // test constructor with points
        final List<Point2D> distortedPoints = new ArrayList<>();
        final List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        estimator = new MSACRadialDistortionRobustEstimator(distortedPoints, undistortedPoints);

        // check correctness
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();
        estimator = null;
        try {
            estimator = new MSACRadialDistortionRobustEstimator(emptyPoints, undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new MSACRadialDistortionRobustEstimator(emptyPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new MSACRadialDistortionRobustEstimator(null, undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new MSACRadialDistortionRobustEstimator(distortedPoints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with points and listener
        estimator = new MSACRadialDistortionRobustEstimator(distortedPoints,
                undistortedPoints, this);

        // check correctness
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS,
                estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new MSACRadialDistortionRobustEstimator(emptyPoints,
                    undistortedPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new MSACRadialDistortionRobustEstimator(emptyPoints, emptyPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new MSACRadialDistortionRobustEstimator(
                    null, undistortedPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new MSACRadialDistortionRobustEstimator(
                    distortedPoints, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with points and center
        final Point2D center = Point2D.create();

        estimator = new MSACRadialDistortionRobustEstimator(distortedPoints, undistortedPoints, center);

        // check correctness
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(center.getInhomX(), estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(center.getInhomY(), estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new MSACRadialDistortionRobustEstimator(emptyPoints, undistortedPoints, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new MSACRadialDistortionRobustEstimator(emptyPoints, emptyPoints, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new MSACRadialDistortionRobustEstimator(
                    null, undistortedPoints, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new MSACRadialDistortionRobustEstimator(
                    distortedPoints, null, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with points, center and listener
        estimator = new MSACRadialDistortionRobustEstimator(distortedPoints,
                undistortedPoints, center, this);

        // check correctness
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(center.getInhomX(), estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(center.getInhomY(), estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS,
                estimator.getNumKParams());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new MSACRadialDistortionRobustEstimator(emptyPoints,
                    undistortedPoints, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new MSACRadialDistortionRobustEstimator(emptyPoints,
                    emptyPoints, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new MSACRadialDistortionRobustEstimator(
                    null, undistortedPoints, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new MSACRadialDistortionRobustEstimator(
                    distortedPoints, null, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final MSACRadialDistortionRobustEstimator estimator = new MSACRadialDistortionRobustEstimator();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        final double[] qualityScores = new double[1];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertNull(estimator.getQualityScores());
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final MSACRadialDistortionRobustEstimator estimator = new MSACRadialDistortionRobustEstimator();

        // check default value
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_THRESHOLD,
                estimator.getThreshold(), 0.0);

        // set new value
        estimator.setThreshold(10.0);

        // check correctness
        assertEquals(10.0, estimator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final MSACRadialDistortionRobustEstimator estimator = new MSACRadialDistortionRobustEstimator();

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
    public void testGetSetProgressDelta() throws LockedException {
        final MSACRadialDistortionRobustEstimator estimator = new MSACRadialDistortionRobustEstimator();

        // check default value
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_PROGRESS_DELTA,
                estimator.getProgressDelta(), 0.0);

        // set new value
        estimator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5, estimator.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        final MSACRadialDistortionRobustEstimator estimator = new MSACRadialDistortionRobustEstimator();

        // check default value
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_CONFIDENCE,
                estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.75);

        // check correctness
        assertEquals(0.75, estimator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxIterations() throws LockedException {
        final MSACRadialDistortionRobustEstimator estimator = new MSACRadialDistortionRobustEstimator();

        // check default value
        assertEquals(MSACRadialDistortionRobustEstimator.DEFAULT_MAX_ITERATIONS,
                estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(10);

        // check correctness
        assertEquals(10, estimator.getMaxIterations());

        // Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetPoints() throws LockedException {
        final List<Point2D> distortedPoints = new ArrayList<>();
        final List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS; i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        final MSACRadialDistortionRobustEstimator estimator = new MSACRadialDistortionRobustEstimator();

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
        final List<Point2D> emptyPoints = new ArrayList<>();
        try {
            estimator.setPoints(emptyPoints, undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setPoints(emptyPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setPoints(null, undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setPoints(distortedPoints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetDistortionCenter() throws LockedException {
        final MSACRadialDistortionRobustEstimator estimator = new MSACRadialDistortionRobustEstimator();

        // check default value
        assertNull(estimator.getDistortionCenter());

        // set new value
        final Point2D center = Point2D.create();
        estimator.setDistortionCenter(center);

        // check correctness
        assertSame(center, estimator.getDistortionCenter());
    }

    @Test
    public void testGetSetHorizontalFocalLength() throws LockedException {
        final MSACRadialDistortionRobustEstimator estimator = new MSACRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);

        // set new value
        estimator.setHorizontalFocalLength(2.0);

        // check correctness
        assertEquals(2.0, estimator.getHorizontalFocalLength(), 0.0);
    }

    @Test
    public void testGetSetVerticalFocalLength() throws LockedException {
        final MSACRadialDistortionRobustEstimator estimator = new MSACRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);

        // set new value
        estimator.setVerticalFocalLength(2.0);

        // check correctness
        assertEquals(2.0, estimator.getVerticalFocalLength(), 0.0);
    }

    @Test
    public void testGetSetSkew() throws LockedException {
        final MSACRadialDistortionRobustEstimator estimator = new MSACRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);

        // set new value
        estimator.setSkew(1.0);

        // check correctness
        assertEquals(1.0, estimator.getSkew(), 0.0);
    }

    @Test
    public void testGetSetIntrinsic() throws LockedException {
        final MSACRadialDistortionRobustEstimator estimator = new MSACRadialDistortionRobustEstimator();

        // check default value
        PinholeCameraIntrinsicParameters intrinsic = estimator.getIntrinsic();

        assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                intrinsic.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_FOCAL_LENGTH,
                intrinsic.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_SKEW,
                intrinsic.getSkewness(), 0.0);

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
        estimator.setIntrinsic(new InhomogeneousPoint2D(6.0, 5.0),
                4.0, 3.0, 2.0);

        // check correctness
        assertEquals(6.0, estimator.getDistortionCenter().getInhomX(), 0.0);
        assertEquals(5.0, estimator.getDistortionCenter().getInhomY(), 0.0);
        assertEquals(4.0, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(3.0, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(2.0, estimator.getSkew(), 0.0);
    }

    @Test
    public void testGetSetNumKParams() throws LockedException {
        final MSACRadialDistortionRobustEstimator estimator = new MSACRadialDistortionRobustEstimator();

        // check default value
        assertEquals(RadialDistortionRobustEstimator.DEFAULT_NUM_K_PARAMS,
                estimator.getNumKParams());

        // set new value
        estimator.setNumKParams(3);

        // check correctness
        assertEquals(3, estimator.getNumKParams());
    }

    @Test
    public void testEstimate() throws NotSupportedException, LockedException,
            NotReadyException, RobustEstimatorException, DistortionException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int j = 0; j < TIMES; j++) {
            final double k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
            final double k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);

            final Point2D center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

            final RadialDistortion distortion = new RadialDistortion(k1, k2, center);

            final int nPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final List<Point2D> distortedPointsWithError = new ArrayList<>();
            final List<Point2D> distortedPoints = new ArrayList<>();
            final List<Point2D> undistortedPoints = new ArrayList<>();
            Point2D distortedPoint;
            Point2D distortedPointWithError;
            Point2D undistortedPoint;
            for (int i = 0; i < nPoints; i++) {
                undistortedPoint = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                        randomizer.nextDouble(MIN_POINT_VALUE,
                                MAX_POINT_VALUE));

                distortedPoint = distortion.distort(undistortedPoint);
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIER) {
                    // point is outlier
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    distortedPointWithError = new InhomogeneousPoint2D(
                            distortedPoint.getInhomX() + errorX,
                            distortedPoint.getInhomY() + errorY);
                } else {
                    // point is inlier
                    distortedPointWithError = distortedPoint;
                }
                distortedPoints.add(distortedPoint);
                distortedPointsWithError.add(distortedPointWithError);
                undistortedPoints.add(undistortedPoint);
            }

            final MSACRadialDistortionRobustEstimator estimator = new MSACRadialDistortionRobustEstimator(
                    distortedPointsWithError, undistortedPoints, this);

            estimator.setIntrinsic(distortion.getIntrinsic());
            estimator.setThreshold(THRESHOLD);

            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertEquals(0, estimateNextIteration);
            assertEquals(0, estimateProgressChange);
            assertTrue(estimator.isReady());
            assertFalse(estimator.isLocked());

            final RadialDistortion distortion2 = estimator.estimate();

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertTrue(estimateNextIteration > 0);
            assertTrue(estimateProgressChange >= 0);
            reset();

            // check correctness of estimation
            assertEquals(k1, distortion2.getK1(), ABSOLUTE_ERROR);
            assertEquals(k2, distortion2.getK2(), ABSOLUTE_ERROR);
            assertEquals(distortion2.getCenter(), center);

            for (int i = 0; i < nPoints; i++) {
                assertEquals(distortedPoints.get(i), distortion2.distort(undistortedPoints.get(i)));
            }
        }
    }

    @Override
    public void onEstimateStart(final RadialDistortionRobustEstimator estimator) {
        estimateStart++;
        testLocked((MSACRadialDistortionRobustEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final RadialDistortionRobustEstimator estimator) {
        estimateEnd++;
        testLocked((MSACRadialDistortionRobustEstimator) estimator);
    }

    @Override
    public void onEstimateNextIteration(
            final RadialDistortionRobustEstimator estimator, final int iteration) {
        estimateNextIteration++;
        testLocked((MSACRadialDistortionRobustEstimator) estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final RadialDistortionRobustEstimator estimator, final float progress) {
        estimateProgressChange++;
        testLocked((MSACRadialDistortionRobustEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimateNextIteration =
                estimateProgressChange = 0;
    }

    private void testLocked(final MSACRadialDistortionRobustEstimator estimator) {
        try {
            estimator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setDistortionCenter(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMaxIterations(10);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPoints(null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception ignore) {
            fail("LockedException expected but not thrown");
        }
        assertTrue(estimator.isLocked());
    }
}
