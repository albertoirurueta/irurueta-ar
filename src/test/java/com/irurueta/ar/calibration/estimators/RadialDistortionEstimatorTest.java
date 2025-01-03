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
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class RadialDistortionEstimatorTest {

    @Test
    void testCreate() {
        var estimator = RadialDistortionEstimator.create();

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.isReady());
        assertEquals(RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());

        // test with type
        estimator = RadialDistortionEstimator.create(RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.isReady());
        assertEquals(RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());

        estimator = RadialDistortionEstimator.create(
                RadialDistortionEstimatorType.WEIGHTED_RADIAL_DISTORTION_ESTIMATOR);

        // check correctness
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());
        assertNull(estimator.getDistortionCenter());
        assertFalse(estimator.isReady());
        assertEquals(RadialDistortionEstimatorType.WEIGHTED_RADIAL_DISTORTION_ESTIMATOR, estimator.getType());
    }

    @Test
    void testGetSetDistortedUndistortedPoints() throws LockedException {
        final var estimator = RadialDistortionEstimator.create();

        // check default values
        assertNull(estimator.getDistortedPoints());
        assertNull(estimator.getUndistortedPoints());

        // set new value
        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        for (var i = 0; i < estimator.getMinNumberOfMatchedPoints(); i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }

        estimator.setPoints(distortedPoints, undistortedPoints);

        // check correctness
        assertSame(distortedPoints, estimator.getDistortedPoints());
        assertSame(undistortedPoints, estimator.getUndistortedPoints());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(emptyPoints, undistortedPoints));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(emptyPoints, emptyPoints));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(null, undistortedPoints));
        assertThrows(IllegalArgumentException.class, () -> estimator.setPoints(distortedPoints, null));
    }

    @Test
    void testGetSetDistortionCenter() throws LockedException {
        final var estimator = RadialDistortionEstimator.create();

        // check default value
        assertNull(estimator.getDistortionCenter());

        // set new value
        final var center = Point2D.create();
        estimator.setDistortionCenter(center);

        // check correctness
        assertSame(center, estimator.getDistortionCenter());
    }

    @Test
    void testAreValidLists() {
        final var estimator = RadialDistortionEstimator.create();

        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        for (var i = 0; i < estimator.getMinNumberOfMatchedPoints(); i++) {
            distortedPoints.add(Point2D.create());
            undistortedPoints.add(Point2D.create());
        }
        final var emptyPoints = new ArrayList<Point2D>();

        assertTrue(estimator.areValidPoints(distortedPoints, undistortedPoints));
        assertFalse(estimator.areValidPoints(emptyPoints, undistortedPoints));
        assertFalse(estimator.areValidPoints(distortedPoints, emptyPoints));
        assertFalse(estimator.areValidPoints(null, undistortedPoints));
        assertFalse(estimator.areValidPoints(distortedPoints, null));
    }
}
