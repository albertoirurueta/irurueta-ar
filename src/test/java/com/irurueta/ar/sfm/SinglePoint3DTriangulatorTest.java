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
package com.irurueta.ar.sfm;

import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point2D;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class SinglePoint3DTriangulatorTest {

    @Test
    public void testCreate() {
        // test create with type

        // Weighted inhomogeneous
        SinglePoint3DTriangulator triangulator =
                SinglePoint3DTriangulator.create(Point3DTriangulatorType.
                        WEIGHTED_INHOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertNull(triangulator.getListener());
        assertEquals(Point3DTriangulatorType.WEIGHTED_INHOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof WeightedInhomogeneousSinglePoint3DTriangulator);

        // Weighted homogeneous
        triangulator = SinglePoint3DTriangulator.create(
                Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertNull(triangulator.getListener());
        assertEquals(Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR,
                triangulator.getType());
        assertTrue(triangulator instanceof WeightedHomogeneousSinglePoint3DTriangulator);

        // LMSE inhomogeneous
        triangulator = SinglePoint3DTriangulator.create(
                Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertNull(triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR,
                triangulator.getType());
        assertTrue(triangulator instanceof LMSEInhomogeneousSinglePoint3DTriangulator);

        // LMSE homogeneous
        triangulator = SinglePoint3DTriangulator.create(
                Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertNull(triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR,
                triangulator.getType());
        assertTrue(triangulator instanceof LMSEHomogeneousSinglePoint3DTriangulator);

        // test with points and cameras
        final List<Point2D> points = new ArrayList<>();
        points.add(Point2D.create());
        points.add(Point2D.create());

        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        // Weighted inhomogeneous
        triangulator = SinglePoint3DTriangulator.create(points, cameras,
                Point3DTriangulatorType.WEIGHTED_INHOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertNull(triangulator.getListener());
        assertEquals(Point3DTriangulatorType.WEIGHTED_INHOMOGENEOUS_TRIANGULATOR,
                triangulator.getType());
        assertTrue(triangulator instanceof WeightedInhomogeneousSinglePoint3DTriangulator);

        // Weighted homogeneous
        triangulator = SinglePoint3DTriangulator.create(points, cameras,
                Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertNull(triangulator.getListener());
        assertEquals(Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR,
                triangulator.getType());
        assertTrue(triangulator instanceof WeightedHomogeneousSinglePoint3DTriangulator);

        // LMSE inhomogeneous
        triangulator = SinglePoint3DTriangulator.create(points, cameras,
                Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertNull(triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR,
                triangulator.getType());
        assertTrue(triangulator instanceof LMSEInhomogeneousSinglePoint3DTriangulator);

        // LMSE homogeneous
        triangulator = SinglePoint3DTriangulator.create(points, cameras,
                Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertNull(triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR,
                triangulator.getType());
        assertTrue(triangulator instanceof LMSEHomogeneousSinglePoint3DTriangulator);

        // test create with points, cameras and weights
        final double[] weights = new double[2];

        // Weighted inhomogeneous
        triangulator = SinglePoint3DTriangulator.create(points, cameras, weights,
                Point3DTriangulatorType.WEIGHTED_INHOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertNull(triangulator.getListener());
        assertEquals(Point3DTriangulatorType.WEIGHTED_INHOMOGENEOUS_TRIANGULATOR,
                triangulator.getType());
        assertTrue(triangulator instanceof WeightedInhomogeneousSinglePoint3DTriangulator);

        // Weighted homogeneous
        triangulator = SinglePoint3DTriangulator.create(points, cameras,
                weights, Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertNull(triangulator.getListener());
        assertEquals(Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR,
                triangulator.getType());
        assertTrue(triangulator instanceof WeightedHomogeneousSinglePoint3DTriangulator);

        // LMSE inhomogeneous
        triangulator = SinglePoint3DTriangulator.create(points, cameras,
                weights, Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertNull(triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR,
                triangulator.getType());
        assertTrue(triangulator instanceof LMSEInhomogeneousSinglePoint3DTriangulator);

        // LMSE homogeneous
        triangulator = SinglePoint3DTriangulator.create(points, cameras,
                weights, Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertNull(triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR,
                triangulator.getType());
        assertTrue(triangulator instanceof LMSEHomogeneousSinglePoint3DTriangulator);

        // test with listener
        final SinglePoint3DTriangulatorListener listener = new SinglePoint3DTriangulatorListener() {

            @Override
            public void onTriangulateStart(final SinglePoint3DTriangulator triangulator) {
            }

            @Override
            public void onTriangulateEnd(final SinglePoint3DTriangulator triangulator) {
            }
        };

        // Weighted inhomogeneous
        triangulator = SinglePoint3DTriangulator.create(listener,
                Point3DTriangulatorType.WEIGHTED_INHOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertSame(listener, triangulator.getListener());
        assertEquals(Point3DTriangulatorType.WEIGHTED_INHOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof WeightedInhomogeneousSinglePoint3DTriangulator);

        // Weighted homogeneous
        triangulator = SinglePoint3DTriangulator.create(listener,
                Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertSame(listener, triangulator.getListener());
        assertEquals(Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof WeightedHomogeneousSinglePoint3DTriangulator);

        // LMSE inhomogeneous
        triangulator = SinglePoint3DTriangulator.create(listener,
                Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertSame(listener, triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof LMSEInhomogeneousSinglePoint3DTriangulator);

        // LMSE homogeneous
        triangulator = SinglePoint3DTriangulator.create(listener,
                Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertSame(listener, triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof LMSEHomogeneousSinglePoint3DTriangulator);

        // test with points, cameras and listener

        // Weighted inhomogeneous
        triangulator = SinglePoint3DTriangulator.create(points, cameras, listener,
                Point3DTriangulatorType.WEIGHTED_INHOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertSame(listener, triangulator.getListener());
        assertEquals(Point3DTriangulatorType.WEIGHTED_INHOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof WeightedInhomogeneousSinglePoint3DTriangulator);

        // Weighted homogeneous
        triangulator = SinglePoint3DTriangulator.create(points, cameras, listener,
                Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertSame(listener, triangulator.getListener());
        assertEquals(Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof WeightedHomogeneousSinglePoint3DTriangulator);

        // LMSE inhomogeneous
        triangulator = SinglePoint3DTriangulator.create(points, cameras, listener,
                Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertSame(listener, triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof LMSEInhomogeneousSinglePoint3DTriangulator);

        // LMSE homogeneous
        triangulator = SinglePoint3DTriangulator.create(points, cameras, listener,
                Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertSame(listener, triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof LMSEHomogeneousSinglePoint3DTriangulator);

        // test with points, cameras, weights and listener

        // Weighted inhomogeneous
        triangulator = SinglePoint3DTriangulator.create(points, cameras, weights, listener,
                Point3DTriangulatorType.WEIGHTED_INHOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertSame(listener, triangulator.getListener());
        assertEquals(Point3DTriangulatorType.WEIGHTED_INHOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof WeightedInhomogeneousSinglePoint3DTriangulator);

        // Weighted homogeneous
        triangulator = SinglePoint3DTriangulator.create(points, cameras, weights, listener,
                Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertSame(listener, triangulator.getListener());
        assertEquals(Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof WeightedHomogeneousSinglePoint3DTriangulator);

        // LMSE inhomogeneous
        triangulator = SinglePoint3DTriangulator.create(points, cameras, weights, listener,
                Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertSame(listener, triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof LMSEInhomogeneousSinglePoint3DTriangulator);

        // LMSE homogeneous
        triangulator = SinglePoint3DTriangulator.create(points, cameras, weights, listener,
                Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertSame(listener, triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof LMSEHomogeneousSinglePoint3DTriangulator);

        // test without arguments
        triangulator = SinglePoint3DTriangulator.create();

        // check correctness
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertNull(triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof LMSEHomogeneousSinglePoint3DTriangulator);

        // test with points and cameras
        triangulator = SinglePoint3DTriangulator.create(points, cameras);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertNull(triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof LMSEHomogeneousSinglePoint3DTriangulator);

        // test with points, cameras and weights
        triangulator = SinglePoint3DTriangulator.create(points, cameras, weights);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertNull(triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof LMSEHomogeneousSinglePoint3DTriangulator);

        // test with listener
        triangulator = SinglePoint3DTriangulator.create(listener);

        // check correctness
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertSame(listener, triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof LMSEHomogeneousSinglePoint3DTriangulator);

        // test with points, cameras and listener
        triangulator = SinglePoint3DTriangulator.create(points, cameras, listener);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertSame(listener, triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof LMSEHomogeneousSinglePoint3DTriangulator);

        // test with points, cameras, weights and listener
        triangulator = SinglePoint3DTriangulator.create(points, cameras, weights, listener);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertSame(listener, triangulator.getListener());
        assertEquals(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertTrue(triangulator instanceof LMSEHomogeneousSinglePoint3DTriangulator);
    }
}
