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

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class WeightedHomogeneousSinglePoint3DTriangulatorTest implements SinglePoint3DTriangulatorListener {

    private static final int MIN_VIEWS = 2;
    private static final int MAX_VIEWS = 20;

    private static final double MIN_RANDOM_VALUE = 100.0;
    private static final double MAX_RANDOM_VALUE = 500.0;

    private static final double ABSOLUTE_ERROR = 5e-6;

    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 10.0;

    private static final double MIN_SKEWNESS = -0.001;
    private static final double MAX_SKEWNESS = 0.001;

    private static final double MIN_PRINCIPAL_POINT = 0.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;

    private static final double MIN_ANGLE_DEGREES = -10.0;
    private static final double MAX_ANGLE_DEGREES = 10.0;

    private static final double MIN_CAMERA_SEPARATION = 5.0;
    private static final double MAX_CAMERA_SEPARATION = 10.0;

    private static final double MIN_WEIGHT = 0.5;
    private static final double MAX_WEIGHT = 1.0;

    public static final int TIMES = 10;

    private int triangulateStart;
    private int triangulateEnd;

    @Test
    public void testConstructor() {
        // test constructor without arguments
        WeightedHomogeneousSinglePoint3DTriangulator triangulator =
                new WeightedHomogeneousSinglePoint3DTriangulator();

        // check correctness
        assertNull(triangulator.getWeights());
        assertEquals(WeightedHomogeneousSinglePoint3DTriangulator.DEFAULT_MAX_CORRESPONDENCES,
                triangulator.getMaxCorrespondences());
        assertEquals(WeightedHomogeneousSinglePoint3DTriangulator.DEFAULT_SORT_WEIGHTS,
                triangulator.isSortWeightsEnabled());
        assertEquals(Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR,
                triangulator.getType());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertNull(triangulator.getListener());

        // test constructor with points and cameras
        final List<Point2D> points = new ArrayList<>();
        points.add(Point2D.create());
        points.add(Point2D.create());

        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(points, cameras);

        // check correctness
        assertNull(triangulator.getWeights());
        assertEquals(WeightedHomogeneousSinglePoint3DTriangulator.DEFAULT_MAX_CORRESPONDENCES,
                triangulator.getMaxCorrespondences());
        assertEquals(WeightedHomogeneousSinglePoint3DTriangulator.DEFAULT_SORT_WEIGHTS,
                triangulator.isSortWeightsEnabled());
        assertEquals(Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertNull(triangulator.getListener());

        // force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();
        final List<PinholeCamera> emptyCameras = new ArrayList<>();

        triangulator = null;
        try {
            triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(emptyPoints, cameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(points, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(emptyPoints, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(triangulator);

        // test constructor with points, cameras and weights
        final double[] weights = new double[2];

        triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(points, cameras, weights);

        // check correctness
        assertSame(weights, triangulator.getWeights());
        assertEquals(WeightedHomogeneousSinglePoint3DTriangulator.DEFAULT_MAX_CORRESPONDENCES,
                triangulator.getMaxCorrespondences());
        assertEquals(WeightedHomogeneousSinglePoint3DTriangulator.DEFAULT_SORT_WEIGHTS,
                triangulator.isSortWeightsEnabled());
        assertEquals(Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertNull(triangulator.getListener());

        // force IllegalArgumentException
        final double[] emptyWeights = new double[0];
        triangulator = null;
        try {
            triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(emptyPoints, cameras, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(points, emptyCameras, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(emptyPoints, emptyCameras, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(points, cameras, emptyWeights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(triangulator);


        // test constructor with listener
        triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(this);

        // check correctness
        assertNull(triangulator.getWeights());
        assertEquals(WeightedHomogeneousSinglePoint3DTriangulator.DEFAULT_MAX_CORRESPONDENCES,
                triangulator.getMaxCorrespondences());
        assertEquals(WeightedHomogeneousSinglePoint3DTriangulator.DEFAULT_SORT_WEIGHTS,
                triangulator.isSortWeightsEnabled());
        assertEquals(Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertSame(this, triangulator.getListener());

        // test constructor with points, cameras and listener
        triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(points, cameras, this);

        // check correctness
        assertNull(triangulator.getWeights());
        assertEquals(WeightedHomogeneousSinglePoint3DTriangulator.DEFAULT_MAX_CORRESPONDENCES,
                triangulator.getMaxCorrespondences());
        assertEquals(WeightedHomogeneousSinglePoint3DTriangulator.DEFAULT_SORT_WEIGHTS,
                triangulator.isSortWeightsEnabled());
        assertEquals(Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertSame(this, triangulator.getListener());

        // force IllegalArgumentException
        triangulator = null;
        try {
            triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(emptyPoints, cameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(points, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(emptyPoints, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(triangulator);

        // test constructor with points, cameras, weights and listener
        triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(points, cameras, weights,
                this);

        // check correctness
        assertSame(triangulator.getWeights(), weights);
        assertEquals(WeightedHomogeneousSinglePoint3DTriangulator.DEFAULT_MAX_CORRESPONDENCES,
                triangulator.getMaxCorrespondences());
        assertEquals(WeightedHomogeneousSinglePoint3DTriangulator.DEFAULT_SORT_WEIGHTS,
                triangulator.isSortWeightsEnabled());
        assertEquals(Point3DTriangulatorType.WEIGHTED_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertSame(this, triangulator.getListener());

        // force IllegalArgumentException
        triangulator = null;
        try {
            triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(emptyPoints, cameras, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(points, emptyCameras, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(emptyPoints, emptyCameras, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new WeightedHomogeneousSinglePoint3DTriangulator(points, cameras, emptyWeights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(triangulator);
    }

    @Test
    public void testGetSetPointsAndCameras() throws LockedException {
        final WeightedHomogeneousSinglePoint3DTriangulator triangulator =
                new WeightedHomogeneousSinglePoint3DTriangulator();

        // check default values
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());

        // set new value
        final List<Point2D> points = new ArrayList<>();
        points.add(Point2D.create());
        points.add(Point2D.create());

        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        triangulator.setPointsAndCameras(points, cameras);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();
        final List<PinholeCamera> emptyCameras = new ArrayList<>();
        try {
            triangulator.setPointsAndCameras(emptyPoints, cameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator.setPointsAndCameras(points, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator.setPointsAndCameras(emptyPoints, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetPointsCamerasAndWeights() throws LockedException {
        final WeightedHomogeneousSinglePoint3DTriangulator triangulator =
                new WeightedHomogeneousSinglePoint3DTriangulator();

        // check default values
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getWeights());

        // set new value
        final List<Point2D> points = new ArrayList<>();
        points.add(Point2D.create());
        points.add(Point2D.create());

        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        final double[] weights = new double[2];

        triangulator.setPointsCamerasAndWeights(points, cameras, weights);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertSame(weights, triangulator.getWeights());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();
        final List<PinholeCamera> emptyCameras = new ArrayList<>();
        final double[] emptyWeights = new double[0];
        try {
            triangulator.setPointsCamerasAndWeights(emptyPoints, cameras, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator.setPointsCamerasAndWeights(points, emptyCameras, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator.setPointsCamerasAndWeights(emptyPoints, emptyCameras, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator.setPointsCamerasAndWeights(points, cameras, emptyWeights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final WeightedHomogeneousSinglePoint3DTriangulator triangulator =
                new WeightedHomogeneousSinglePoint3DTriangulator();

        // check default value
        assertNull(triangulator.getListener());

        // set new value
        triangulator.setListener(this);

        // check correctness
        assertSame(this, triangulator.getListener());
    }

    @Test
    public void testGetSetMaxCorrespondences() throws LockedException {
        final WeightedHomogeneousSinglePoint3DTriangulator triangulator =
                new WeightedHomogeneousSinglePoint3DTriangulator();

        // check default value
        assertEquals(WeightedHomogeneousSinglePoint3DTriangulator.DEFAULT_MAX_CORRESPONDENCES,
                triangulator.getMaxCorrespondences());

        // set new value
        triangulator.setMaxCorrespondences(10);

        // check correctness
        assertEquals(10, triangulator.getMaxCorrespondences());

        // Force IllegalArgumentException
        try {
            triangulator.setMaxCorrespondences(1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetSortWeightsEnabled() throws LockedException {
        final WeightedHomogeneousSinglePoint3DTriangulator triangulator =
                new WeightedHomogeneousSinglePoint3DTriangulator();

        // check default value
        assertEquals(WeightedHomogeneousSinglePoint3DTriangulator.DEFAULT_SORT_WEIGHTS,
                triangulator.isSortWeightsEnabled());

        // set new value
        triangulator.setSortWeightsEnabled(!WeightedHomogeneousSinglePoint3DTriangulator.DEFAULT_SORT_WEIGHTS);

        // check correctness
        assertEquals(!WeightedHomogeneousSinglePoint3DTriangulator.DEFAULT_SORT_WEIGHTS,
                triangulator.isSortWeightsEnabled());
    }

    @Test
    public void testTriangulate() throws LockedException, NotReadyException, Point3DTriangulationException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // obtain number of views
            final int numViews = randomizer.nextInt(MIN_VIEWS, MAX_VIEWS);

            // create a random 3D point
            final Point3D point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            final List<Point2D> points2D = new ArrayList<>();
            final List<PinholeCamera> cameras = new ArrayList<>();
            final Point3D previousCameraCenter = new InhomogeneousPoint3D();
            final double[] weights = new double[numViews];
            for (int i = 0; i < numViews; i++) {
                // create a random camera
                final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
                final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
                final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
                final double horizontalPrincipalPoint = randomizer.nextDouble(
                        MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
                final double verticalPrincipalPoint = randomizer.nextDouble(
                        MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

                final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES) * Math.PI / 180.0;

                final double cameraSeparationX = randomizer.nextDouble(
                        MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
                final double cameraSeparationY = randomizer.nextDouble(
                        MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
                final double cameraSeparationZ = randomizer.nextDouble(
                        MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

                weights[i] = randomizer.nextDouble(MIN_WEIGHT, MAX_WEIGHT);

                final PinholeCameraIntrinsicParameters intrinsic = new PinholeCameraIntrinsicParameters(
                        horizontalFocalLength, verticalFocalLength,
                        horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

                final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                final Point3D cameraCenter = new InhomogeneousPoint3D(
                        previousCameraCenter.getInhomX() + cameraSeparationX,
                        previousCameraCenter.getInhomY() + cameraSeparationY,
                        previousCameraCenter.getInhomZ() + cameraSeparationZ);

                final PinholeCamera camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

                // project 3D point using camera
                final Point2D point2D = camera.project(point3D);

                cameras.add(camera);
                points2D.add(point2D);
            }

            // create triangulator
            final WeightedHomogeneousSinglePoint3DTriangulator triangulator =
                    new WeightedHomogeneousSinglePoint3DTriangulator(points2D, cameras, weights, this);

            // check default values
            assertTrue(triangulator.isReady());
            assertFalse(triangulator.isLocked());
            assertEquals(0, triangulateStart);
            assertEquals(0, triangulateEnd);

            final Point3D triangulated = triangulator.triangulate();

            // check correctness
            assertTrue(triangulator.isReady());
            assertFalse(triangulator.isLocked());
            assertEquals(1, triangulateStart);
            assertEquals(1, triangulateEnd);
            reset();

            if (point3D.distanceTo(triangulated) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, point3D.distanceTo(triangulated), ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onTriangulateStart(final SinglePoint3DTriangulator triangulator) {
        triangulateStart++;
        checkLocked((WeightedHomogeneousSinglePoint3DTriangulator) triangulator);
    }

    @Override
    public void onTriangulateEnd(final SinglePoint3DTriangulator triangulator) {
        triangulateEnd++;
        checkLocked((WeightedHomogeneousSinglePoint3DTriangulator) triangulator);
    }

    private void reset() {
        triangulateStart = triangulateEnd = 0;
    }

    private void checkLocked(
            final WeightedHomogeneousSinglePoint3DTriangulator triangulator) {
        try {
            triangulator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            triangulator.setPointsAndCameras(null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            triangulator.setPointsCamerasAndWeights(null, null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            triangulator.setMaxCorrespondences(10);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            triangulator.setSortWeightsEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            triangulator.triangulate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        assertTrue(triangulator.isLocked());
    }
}
