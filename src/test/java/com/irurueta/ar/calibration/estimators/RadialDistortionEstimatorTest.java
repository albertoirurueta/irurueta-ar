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
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RadialDistortionEstimatorTest {

    @Test
    public void testCreate() {
        RadialDistortionEstimator estimator = RadialDistortionEstimator.create();

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.isReady());
        assertEquals(RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());

        // test with type
        estimator = RadialDistortionEstimator.create(
                RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.isReady());
        assertEquals(RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR,
                estimator.getType());

        estimator = RadialDistortionEstimator.create(
                RadialDistortionEstimatorType.WEIGHTED_RADIAL_DISTORTION_ESTIMATOR);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.isReady());
        assertEquals(RadialDistortionEstimatorType.WEIGHTED_RADIAL_DISTORTION_ESTIMATOR,
                estimator.getType());
    }

    @Test
    public void testGetSetDistortedUndistortedPoints() throws LockedException {
        final RadialDistortionEstimator estimator = RadialDistortionEstimator.create();

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
        final RadialDistortionEstimator estimator = RadialDistortionEstimator.create();

        // check default value
        assertNull(estimator.getDistortionCenter());

        // set new value
        final Point2D center = Point2D.create();
        estimator.setDistortionCenter(center);

        // check correctness
        assertSame(center, estimator.getDistortionCenter());
    }

    @Test
    public void testAreValidLists() {
        final RadialDistortionEstimator estimator = RadialDistortionEstimator.create();

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
}
