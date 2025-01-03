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
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class LMSEHomogeneousSinglePoint3DTriangulatorTest implements SinglePoint3DTriangulatorListener {

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

    public static final int TIMES = 10;

    private int triangulateStart;
    private int triangulateEnd;

    @Test
    void testConstructor() {
        // test constructor without arguments
        var triangulator = new LMSEHomogeneousSinglePoint3DTriangulator();

        // check correctness
        assertEquals(LMSEHomogeneousSinglePoint3DTriangulator.DEFAULT_ALLOW_LMSE_SOLUTION,
                triangulator.isLMSESolutionAllowed());
        assertEquals(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertNull(triangulator.getListener());

        // test constructor with points and cameras
        final var points = new ArrayList<Point2D>();
        points.add(Point2D.create());
        points.add(Point2D.create());

        final var cameras = new ArrayList<PinholeCamera>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        triangulator = new LMSEHomogeneousSinglePoint3DTriangulator(points, cameras);

        // check correctness
        assertEquals(LMSEHomogeneousSinglePoint3DTriangulator.DEFAULT_ALLOW_LMSE_SOLUTION,
                triangulator.isLMSESolutionAllowed());
        assertEquals(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertNull(triangulator.getListener());

        // force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        final var emptyCameras = new ArrayList<PinholeCamera>();

        assertThrows(IllegalArgumentException.class,
                () -> new LMSEHomogeneousSinglePoint3DTriangulator(emptyPoints, cameras));
        assertThrows(IllegalArgumentException.class,
                () -> new LMSEHomogeneousSinglePoint3DTriangulator(points, emptyCameras));
        assertThrows(IllegalArgumentException.class,
                () -> new LMSEHomogeneousSinglePoint3DTriangulator(emptyPoints, emptyCameras));

        // test constructor with listener
        triangulator = new LMSEHomogeneousSinglePoint3DTriangulator(this);

        // check correctness
        assertEquals(LMSEHomogeneousSinglePoint3DTriangulator.DEFAULT_ALLOW_LMSE_SOLUTION,
                triangulator.isLMSESolutionAllowed());
        assertEquals(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertSame(this, triangulator.getListener());

        // test constructor with points, cameras and listener
        triangulator = new LMSEHomogeneousSinglePoint3DTriangulator(points, cameras, this);

        // check correctness
        assertEquals(LMSEHomogeneousSinglePoint3DTriangulator.DEFAULT_ALLOW_LMSE_SOLUTION,
                triangulator.isLMSESolutionAllowed());
        assertEquals(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR, triangulator.getType());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertSame(this, triangulator.getListener());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new LMSEHomogeneousSinglePoint3DTriangulator(emptyPoints, cameras));
        assertThrows(IllegalArgumentException.class,
                () -> new LMSEHomogeneousSinglePoint3DTriangulator(points, emptyCameras));
        assertThrows(IllegalArgumentException.class,
                () -> new LMSEHomogeneousSinglePoint3DTriangulator(emptyPoints, emptyCameras));
    }

    @Test
    void testIsSetLMSeSolutionAllowed() throws LockedException {
        final var triangulator = new LMSEHomogeneousSinglePoint3DTriangulator();

        // check default value
        assertEquals(LMSEHomogeneousSinglePoint3DTriangulator.DEFAULT_ALLOW_LMSE_SOLUTION,
                triangulator.isLMSESolutionAllowed());

        // set new value
        triangulator.setLMSESolutionAllowed(!LMSEHomogeneousSinglePoint3DTriangulator.DEFAULT_ALLOW_LMSE_SOLUTION);

        // check correctness
        assertEquals(!LMSEHomogeneousSinglePoint3DTriangulator.DEFAULT_ALLOW_LMSE_SOLUTION,
                triangulator.isLMSESolutionAllowed());
    }

    @Test
    void testGetSetPointsAndCameras() throws LockedException {
        final var triangulator = new LMSEHomogeneousSinglePoint3DTriangulator();

        // check default values
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());

        // set new value
        final var points = new ArrayList<Point2D>();
        points.add(Point2D.create());
        points.add(Point2D.create());

        final var cameras = new ArrayList<PinholeCamera>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        triangulator.setPointsAndCameras(points, cameras);

        // check correctness
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        final var emptyCameras = new ArrayList<PinholeCamera>();
        assertThrows(IllegalArgumentException.class, () -> triangulator.setPointsAndCameras(emptyPoints, cameras));
        assertThrows(IllegalArgumentException.class, () -> triangulator.setPointsAndCameras(points, emptyCameras));
        assertThrows(IllegalArgumentException.class, () -> triangulator.setPointsAndCameras(emptyPoints, emptyCameras));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var triangulator = new LMSEHomogeneousSinglePoint3DTriangulator();

        // check default value
        assertNull(triangulator.getListener());

        // set new value
        triangulator.setListener(this);

        // check correctness
        assertSame(this, triangulator.getListener());
    }

    @Test
    void testTriangulate() throws LockedException, NotReadyException, Point3DTriangulationException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // obtain number of views
            final var numViews = randomizer.nextInt(MIN_VIEWS, MAX_VIEWS);

            // create a random 3D point
            final var point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            final var points2D = new ArrayList<Point2D>();
            final var cameras = new ArrayList<PinholeCamera>();
            final var previousCameraCenter = new InhomogeneousPoint3D();
            for (var i = 0; i < numViews; i++) {
                // create a random camera
                final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
                final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
                final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
                final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
                final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var cameraSeparationX = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
                final var cameraSeparationY = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
                final var cameraSeparationZ = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

                final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                        horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                final var cameraCenter = new InhomogeneousPoint3D(
                        previousCameraCenter.getInhomX() + cameraSeparationX,
                        previousCameraCenter.getInhomY() + cameraSeparationY,
                        previousCameraCenter.getInhomZ() + cameraSeparationZ);

                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

                // project 3D point using camera
                final var point2D = camera.project(point3D);

                cameras.add(camera);
                points2D.add(point2D);
            }

            // create triangulator
            final var triangulator = new LMSEHomogeneousSinglePoint3DTriangulator(points2D, cameras, this);

            // check default values
            assertTrue(triangulator.isReady());
            assertFalse(triangulator.isLocked());
            assertEquals(0, triangulateStart);
            assertEquals(0, triangulateEnd);

            final var triangulated = triangulator.triangulate();

            // check correctness
            assertTrue(triangulator.isReady());
            assertFalse(triangulator.isLocked());
            assertEquals(1, triangulateStart);
            assertEquals(1, triangulateEnd);
            reset();

            if (Math.abs(point3D.distanceTo(triangulated)) > ABSOLUTE_ERROR) {
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
        checkLocked((LMSEHomogeneousSinglePoint3DTriangulator) triangulator);
    }

    @Override
    public void onTriangulateEnd(final SinglePoint3DTriangulator triangulator) {
        triangulateEnd++;
        checkLocked((LMSEHomogeneousSinglePoint3DTriangulator) triangulator);
    }

    private void reset() {
        triangulateStart = triangulateEnd = 0;
    }

    private void checkLocked(final LMSEHomogeneousSinglePoint3DTriangulator triangulator) {
        assertThrows(LockedException.class, () -> triangulator.setLMSESolutionAllowed(true));
        assertThrows(LockedException.class, () -> triangulator.setListener(this));
        assertThrows(LockedException.class, () -> triangulator.setPointsAndCameras(null, null));
        assertThrows(LockedException.class, triangulator::triangulate);
        assertTrue(triangulator.isLocked());
    }
}
