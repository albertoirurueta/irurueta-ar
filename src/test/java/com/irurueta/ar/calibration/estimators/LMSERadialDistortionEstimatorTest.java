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
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.NotSupportedException;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class LMSERadialDistortionEstimatorTest implements RadialDistortionEstimatorListener {

    private static final double MIN_POINT_VALUE = -1.0;
    private static final double MAX_POINT_VALUE = 1.0;

    private static final double MIN_PARAM_VALUE = -1e-4;
    private static final double MAX_PARAM_VALUE = 1e-4;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final int NUM_POINTS = 10;

    private int estimateStart;
    private int estimateEnd;
    private int estimationProgressChange;

    @Test
    public void testConstructor() {
        // test constructor without parameters
        LMSERadialDistortionEstimator estimator = new LMSERadialDistortionEstimator();

        // check correctness
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        //noinspection EqualsWithItself
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getMinNumberOfMatchedPoints());
        assertEquals(LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());

        // test constructor with listener
        estimator = new LMSERadialDistortionEstimator(this);

        // check correctness
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        //noinspection EqualsWithItself
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getMinNumberOfMatchedPoints());
        assertEquals(LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());

        // test constructor with distorted/undistorted points
        final List<Point2D> distortedPoints = new ArrayList<>();
        final List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < estimator.getMinNumberOfMatchedPoints(); i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        estimator = new LMSERadialDistortionEstimator(distortedPoints, undistortedPoints);

        // check correctness
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        //noinspection EqualsWithItself
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getMinNumberOfMatchedPoints());
        assertEquals(LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();
        estimator = null;
        try {
            estimator = new LMSERadialDistortionEstimator(emptyPoints, undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LMSERadialDistortionEstimator(emptyPoints, emptyPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LMSERadialDistortionEstimator(null, undistortedPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LMSERadialDistortionEstimator(distortedPoints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with distorted/undistorted points and listener
        estimator = new LMSERadialDistortionEstimator(distortedPoints, undistortedPoints, this);

        // check correctness
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS,
                estimator.getNumKParams());
        //noinspection EqualsWithItself
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getMinNumberOfMatchedPoints());
        assertEquals(LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR,
                estimator.getType());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMSERadialDistortionEstimator(emptyPoints,
                    undistortedPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LMSERadialDistortionEstimator(emptyPoints,
                    emptyPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LMSERadialDistortionEstimator(null,
                    undistortedPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LMSERadialDistortionEstimator(distortedPoints,
                    null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with distortion center
        final Point2D center = Point2D.create();
        estimator = new LMSERadialDistortionEstimator(center);

        // check correctness
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertSame(estimator.getDistortionCenter(), center);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        //noinspection EqualsWithItself
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getMinNumberOfMatchedPoints());
        assertEquals(LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());

        // test constructor with distortion center and listener
        estimator = new LMSERadialDistortionEstimator(center, this);

        // check correctness
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        //noinspection EqualsWithItself
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getMinNumberOfMatchedPoints());
        assertEquals(LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());
        assertFalse(estimator.arePointsAvailable());
        assertFalse(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());

        // test constructor with distorted/undistorted points and distortion
        // center
        estimator = new LMSERadialDistortionEstimator(distortedPoints, undistortedPoints, center);
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(estimator.getDistortionCenter(), center);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS, estimator.getNumKParams());
        //noinspection EqualsWithItself
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getMinNumberOfMatchedPoints());
        assertEquals(LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR,
                estimator.getType());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMSERadialDistortionEstimator(emptyPoints, undistortedPoints, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LMSERadialDistortionEstimator(emptyPoints, emptyPoints, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LMSERadialDistortionEstimator(null, undistortedPoints, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LMSERadialDistortionEstimator(distortedPoints, null, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with distorted/undistorted points and distortion
        // center
        estimator = new LMSERadialDistortionEstimator(distortedPoints,
                undistortedPoints, center, this);
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW,
                estimator.getSkew(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS,
                estimator.getNumKParams());
        //noinspection EqualsWithItself
        assertEquals(estimator.getMinNumberOfMatchedPoints(),
                estimator.getMinNumberOfMatchedPoints());
        assertEquals(LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());
        assertTrue(estimator.arePointsAvailable());
        assertTrue(estimator.isReady());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertEquals(RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR,
                estimator.getType());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new LMSERadialDistortionEstimator(emptyPoints,
                    undistortedPoints, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LMSERadialDistortionEstimator(emptyPoints,
                    emptyPoints, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LMSERadialDistortionEstimator(null,
                    undistortedPoints, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new LMSERadialDistortionEstimator(distortedPoints,
                    null, center, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetDistortedUndistortedPoints() throws LockedException {
        final LMSERadialDistortionEstimator estimator = new LMSERadialDistortionEstimator();

        // check default values
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());

        // set new value
        final List<Point2D> distortedPoints = new ArrayList<>();
        final List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < estimator.getMinNumberOfMatchedPoints(); i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        estimator.setPoints(distortedPoints, undistortedPoints);

        // check correctness
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());

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
        final LMSERadialDistortionEstimator estimator = new LMSERadialDistortionEstimator();

        // check default value
        assertNull(estimator.getDistortionCenter());
        assertEquals(0.0, estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);

        // set new value
        final Point2D center = Point2D.create();
        center.setInhomogeneousCoordinates(1.0, 2.0);
        estimator.setDistortionCenter(center);

        // check correctness
        assertSame(center, estimator.getDistortionCenter());
        assertEquals(center.getInhomX(), estimator.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(center.getInhomY(), estimator.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
    }

    @Test
    public void testGetSetHorizontalFocalLength() throws LockedException,
            RadialDistortionException {
        final LMSERadialDistortionEstimator estimator = new LMSERadialDistortionEstimator();

        // check default value
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);

        // set new value
        estimator.setHorizontalFocalLength(2.0);

        // check correctness
        assertEquals(2.0, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(2.0, estimator.getIntrinsic().getHorizontalFocalLength(), 0.0);

        // Force RadialDistortionException
        try {
            estimator.setHorizontalFocalLength(0.0);
            fail("RadialDistortionException expected but not thrown");
        } catch (final RadialDistortionException ignore) {
        }
    }

    @Test
    public void testGetSetVerticalFocalLength() throws LockedException,
            RadialDistortionException {
        final LMSERadialDistortionEstimator estimator = new LMSERadialDistortionEstimator();

        // check default value
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getIntrinsic().getVerticalFocalLength(), 0.0);

        // set new value
        estimator.setVerticalFocalLength(2.0);

        // check correctness
        assertEquals(2.0, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(2.0, estimator.getIntrinsic().getVerticalFocalLength(), 0.0);

        // Force RadialDistortionException
        try {
            estimator.setVerticalFocalLength(0.0);
            fail("RadialDistortionException expected but not thrown");
        } catch (final RadialDistortionException ignore) {
        }
    }

    @Test
    public void testGetSetSkew() throws LockedException {
        final LMSERadialDistortionEstimator estimator = new LMSERadialDistortionEstimator();

        // check default value
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW,
                estimator.getIntrinsic().getSkewness(), 0.0);

        // set new value
        estimator.setSkew(1.0);

        // check correctness
        assertEquals(1.0, estimator.getSkew(), 0.0);
        assertEquals(1.0, estimator.getIntrinsic().getSkewness(), 0.0);
    }

    @Test
    public void testSetIntrinsic() throws LockedException,
            RadialDistortionException {
        final LMSERadialDistortionEstimator estimator = new LMSERadialDistortionEstimator();

        // check default values
        assertNull(estimator.getDistortionCenter());
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_FOCAL_LENGTH,
                estimator.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortionEstimator.DEFAULT_SKEW, estimator.getSkew(), 0.0);

        // set new values
        final Point2D center = new InhomogeneousPoint2D(1.0, 2.0);

        estimator.setIntrinsic(center, 3.0, 4.0, 5.0);

        // check correctness
        assertEquals(1.0, estimator.getDistortionCenter().getInhomX(), 0.0);
        assertEquals(2.0, estimator.getDistortionCenter().getInhomY(), 0.0);
        assertEquals(3.0, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(4.0, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(5.0, estimator.getSkew(), 0.0);

        // Force RadialDistortionException
        try {
            estimator.setIntrinsic(center, 0.0, 0.0, 1.0);
            fail("RadialDistortionException expected but not thrown");
        } catch (final RadialDistortionException ignore) {
        }

        // set new values
        PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(1.0, 2.0,
                        3.0, 4.0, 5.0);
        estimator.setIntrinsic(intrinsic);

        // check correctness
        assertEquals(3.0, estimator.getDistortionCenter().getInhomX(), 0.0);
        assertEquals(4.0, estimator.getDistortionCenter().getInhomY(), 0.0);
        assertEquals(1.0, estimator.getHorizontalFocalLength(), 0.0);
        assertEquals(2.0, estimator.getVerticalFocalLength(), 0.0);
        assertEquals(5.0, estimator.getSkew(), 0.0);

        // Force RadialDistortionException
        intrinsic = new PinholeCameraIntrinsicParameters(0.0, 0.0,
                3.0, 4.0, 5.0);
        try {
            estimator.setIntrinsic(intrinsic);
            fail("RadialDistortionException expected but not thrown");
        } catch (final RadialDistortionException ignore) {
        }
    }

    @Test
    public void testGetSetNumKParamsAndGetMinNumberOfMatchedPoints()
            throws LockedException {
        final LMSERadialDistortionEstimator estimator = new LMSERadialDistortionEstimator();

        // check default values
        assertEquals(estimator.getNumKParams(), RadialDistortionEstimator.DEFAULT_NUM_K_PARAMS);
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getNumKParams());

        // set new value
        estimator.setNumKParams(3);

        // check correctness
        assertEquals(3, estimator.getNumKParams());
        assertEquals(estimator.getMinNumberOfMatchedPoints(), estimator.getNumKParams());

        // Force IllegalArgumentException
        try {
            estimator.setNumKParams(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetLMSESolutionAllowed() throws LockedException {
        final LMSERadialDistortionEstimator estimator = new LMSERadialDistortionEstimator();

        // check default value
        assertEquals(LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());

        // set new value
        estimator.setLMSESolutionAllowed(!LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION);

        // check default value
        assertEquals(!LMSERadialDistortionEstimator.DEFAULT_ALLOW_LMSE_SOLUTION,
                estimator.isLMSESolutionAllowed());
    }

    @Test
    public void testAreValidPoints() {
        final LMSERadialDistortionEstimator estimator = new LMSERadialDistortionEstimator();

        final List<Point2D> distortedPoints = new ArrayList<>();
        final List<Point2D> undistortedPoints = new ArrayList<>();
        for (int i = 0; i < estimator.getMinNumberOfMatchedPoints(); i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        final List<Point2D> emptyPoints = new ArrayList<>();

        assertTrue(estimator.areValidPoints(distortedPoints, undistortedPoints));
        assertFalse(estimator.areValidPoints(emptyPoints, undistortedPoints));
        assertFalse(estimator.areValidPoints(distortedPoints, emptyPoints));
        assertFalse(estimator.areValidPoints(null, undistortedPoints));
        assertFalse(estimator.areValidPoints(distortedPoints, null));
    }

    @Test
    public void testEstimate() throws NotSupportedException, LockedException,
            NotReadyException, RadialDistortionEstimatorException, DistortionException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        final double k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);

        final Point2D center = new InhomogeneousPoint2D(//0.0, 0.0);
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

        final RadialDistortion distortion = new RadialDistortion(k1, k2, center);

        final List<Point2D> distortedPoints = new ArrayList<>();
        final List<Point2D> undistortedPoints = new ArrayList<>();
        Point2D undistortedPoint;
        for (int i = 0; i < NUM_POINTS; i++) {
            undistortedPoint = new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                    1.0);

            undistortedPoints.add(undistortedPoint);
            distortedPoints.add(distortion.distort(undistortedPoint));
        }

        // test estimation when LMSE is not allowed
        LMSERadialDistortionEstimator estimator = new LMSERadialDistortionEstimator(distortedPoints,
                undistortedPoints, center, this);

        assertFalse(estimator.isLMSESolutionAllowed());
        assertEquals(distortion.getCenter().getInhomX(),
                estimator.getDistortionCenter().getInhomX(), ABSOLUTE_ERROR);
        assertEquals(distortion.getCenter().getInhomY(),
                estimator.getDistortionCenter().getInhomY(), ABSOLUTE_ERROR);
        assertEquals(distortion.getHorizontalFocalLength(),
                estimator.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(distortion.getVerticalFocalLength(),
                estimator.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(distortion.getSkew(),
                estimator.getSkew(), ABSOLUTE_ERROR);

        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(0, estimateStart);
        assertEquals(0, estimateEnd);
        assertEquals(0, estimationProgressChange);

        RadialDistortion distortion2 = estimator.estimate();

        // check correctness
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(1, estimateStart);
        assertEquals(1, estimateEnd);
        assertTrue(estimationProgressChange >= 0);
        reset();

        assertEquals(k1, distortion2.getK1(), ABSOLUTE_ERROR);
        assertEquals(k2, distortion2.getK2(), ABSOLUTE_ERROR);
        assertEquals(center, distortion2.getCenter());

        // test estimation when LMSE is allowed
        estimator = new LMSERadialDistortionEstimator(distortedPoints,
                undistortedPoints, center, this);
        estimator.setLMSESolutionAllowed(true);

        assertTrue(estimator.isLMSESolutionAllowed());

        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(0, estimateStart);
        assertEquals(0, estimateEnd);
        assertEquals(0, estimationProgressChange);

        distortion2 = estimator.estimate();

        // check correctness
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(1, estimateStart);
        assertEquals(1, estimateEnd);
        assertTrue(estimationProgressChange >= 0);
        reset();

        assertEquals(k1, distortion2.getK1(), ABSOLUTE_ERROR);
        assertEquals(k2, distortion2.getK2(), ABSOLUTE_ERROR);
        assertEquals(center, distortion2.getCenter());
    }

    @Override
    public void onEstimateStart(final RadialDistortionEstimator estimator) {
        estimateStart++;
        testLocked((LMSERadialDistortionEstimator) estimator);
    }

    @Override
    public void onEstimateEnd(final RadialDistortionEstimator estimator) {
        estimateEnd++;
        testLocked((LMSERadialDistortionEstimator) estimator);
    }

    @Override
    public void onEstimationProgressChange(final RadialDistortionEstimator estimator,
                                           final float progress) {
        estimationProgressChange++;
        testLocked((LMSERadialDistortionEstimator) estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimationProgressChange = 0;
    }

    private void testLocked(final LMSERadialDistortionEstimator estimator) {
        try {
            estimator.setPoints(null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setDistortionCenter(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setLMSESolutionAllowed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }
}
