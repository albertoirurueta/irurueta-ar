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
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class PROMedSRobustSinglePoint3DTriangulatorTest implements RobustSinglePoint3DTriangulatorListener {

    private static final int MIN_VIEWS = 100;
    private static final int MAX_VIEWS = 500;

    private static final double MIN_RANDOM_VALUE = 100.0;
    private static final double MAX_RANDOM_VALUE = 500.0;

    private static final double ABSOLUTE_ERROR = 5e-4;

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

    private static final double PERCENTAGE_OUTLIERS = 20;

    private static final double STD_ERROR = 10.0;

    private static final double THRESHOLD = 1e-8;

    private static final int TIMES = 10;

    private int triangulateStart;
    private int triangulateEnd;
    private int triangulateNextIteration;
    private int triangulateProgressChange;

    @Test
    void testConstructor() {
        // test constructor without arguments
        var triangulator = new PROMedSRobustSinglePoint3DTriangulator();

        // check correctness
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_STOP_THRESHOLD, triangulator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, triangulator.getMethod());
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(),
                0.0);
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());

        // test constructor with listener
        triangulator = new PROMedSRobustSinglePoint3DTriangulator(this);

        // check correctness
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_STOP_THRESHOLD, triangulator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, triangulator.getMethod());
        assertSame(this, triangulator.getListener());
        assertTrue(triangulator.isListenerAvailable());
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(),
                0.0);
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());

        // test constructor with points and cameras
        final var points = new ArrayList<Point2D>();
        points.add(Point2D.create());
        points.add(Point2D.create());

        final var cameras = new ArrayList<PinholeCamera>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        triangulator = new PROMedSRobustSinglePoint3DTriangulator(points, cameras);

        // check correctness
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_STOP_THRESHOLD, triangulator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, triangulator.getMethod());
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(),
                0.0);
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        final var emptyCameras = new ArrayList<PinholeCamera>();

        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustSinglePoint3DTriangulator(emptyPoints, cameras));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustSinglePoint3DTriangulator(points, emptyCameras));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustSinglePoint3DTriangulator(emptyPoints, emptyCameras));

        // test constructor with points, cameras and listener
        triangulator = new PROMedSRobustSinglePoint3DTriangulator(points, cameras, this);

        // check correctness
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_STOP_THRESHOLD, triangulator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, triangulator.getMethod());
        assertSame(this, triangulator.getListener());
        assertTrue(triangulator.isListenerAvailable());
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(),
                0.0);
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustSinglePoint3DTriangulator(emptyPoints, cameras, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustSinglePoint3DTriangulator(points, emptyCameras, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustSinglePoint3DTriangulator(emptyPoints, emptyCameras, this));

        // test constructor with quality scores
        final var qualityScores = new double[2];
        triangulator = new PROMedSRobustSinglePoint3DTriangulator(qualityScores);

        // check correctness
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_STOP_THRESHOLD, triangulator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, triangulator.getMethod());
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(),
                0.0);
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertSame(qualityScores, triangulator.getQualityScores());
        assertFalse(triangulator.isReady());

        // Force IllegalArgumentException
        final var shortScores = new double[1];
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustSinglePoint3DTriangulator(shortScores));

        // test constructor with quality scores and listener
        triangulator = new PROMedSRobustSinglePoint3DTriangulator(qualityScores, this);

        // check correctness
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_STOP_THRESHOLD, triangulator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, triangulator.getMethod());
        assertSame(this, triangulator.getListener());
        assertTrue(triangulator.isListenerAvailable());
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(),
                0.0);
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertSame(qualityScores, triangulator.getQualityScores());
        assertFalse(triangulator.isReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustSinglePoint3DTriangulator(shortScores, this));

        // test constructor with points, cameras and quality scores
        triangulator = new PROMedSRobustSinglePoint3DTriangulator(points, cameras, qualityScores);

        // check correctness
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_STOP_THRESHOLD, triangulator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, triangulator.getMethod());
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(),
                0.0);
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertSame(qualityScores, triangulator.getQualityScores());
        assertTrue(triangulator.isReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustSinglePoint3DTriangulator(emptyPoints, cameras, qualityScores));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustSinglePoint3DTriangulator(points, emptyCameras, qualityScores));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustSinglePoint3DTriangulator(emptyPoints, emptyCameras, qualityScores));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustSinglePoint3DTriangulator(emptyPoints, emptyCameras, shortScores));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustSinglePoint3DTriangulator(points, cameras, shortScores));

        // test constructor with points, cameras, quality scores and listener
        triangulator = new PROMedSRobustSinglePoint3DTriangulator(points, cameras, qualityScores, this);

        // check correctness
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_STOP_THRESHOLD, triangulator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.PROMEDS, triangulator.getMethod());
        assertSame(this, triangulator.getListener());
        assertTrue(triangulator.isListenerAvailable());
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(),
                0.0);
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertSame(qualityScores, triangulator.getQualityScores());
        assertTrue(triangulator.isReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustSinglePoint3DTriangulator(emptyPoints,
                cameras, qualityScores, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustSinglePoint3DTriangulator(points,
                emptyCameras, qualityScores, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustSinglePoint3DTriangulator(emptyPoints,
                emptyCameras, qualityScores, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustSinglePoint3DTriangulator(emptyPoints,
                emptyCameras, shortScores, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustSinglePoint3DTriangulator(points, cameras,
                shortScores, this));
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var triangulator = new PROMedSRobustSinglePoint3DTriangulator();

        // check default value
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_STOP_THRESHOLD, triangulator.getStopThreshold(),
                0.0);

        // set new value
        triangulator.setStopThreshold(0.5);

        // check correctness
        assertEquals(0.5, triangulator.getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> triangulator.setStopThreshold(0.0));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var triangulator = new PROMedSRobustSinglePoint3DTriangulator();

        // check default value
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());

        // set new value
        triangulator.setListener(this);

        // check correctness
        assertSame(this, triangulator.getListener());
        assertTrue(triangulator.isListenerAvailable());
    }

    @Test
    void testIsSetUseHomogeneousSolution() throws LockedException {
        final var triangulator = new PROMedSRobustSinglePoint3DTriangulator();

        // check default value
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());

        // set new value
        triangulator.setUseHomogeneousSolution(
                !PROMedSRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION);

        // check correctness
        assertEquals(!PROSACRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var triangulator = new PROMedSRobustSinglePoint3DTriangulator();

        // check default value
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);

        // set new value
        triangulator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5, triangulator.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> triangulator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> triangulator.setProgressDelta(2.0f));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var triangulator = new PROMedSRobustSinglePoint3DTriangulator();

        // check default value
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(),
                0.0);

        // set new value
        triangulator.setConfidence(0.5);

        // check correctness
        assertEquals(0.5, triangulator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> triangulator.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> triangulator.setConfidence(2.0));
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var triangulator = new PROMedSRobustSinglePoint3DTriangulator();

        // check default value
        assertEquals(PROMedSRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());

        // set new value
        triangulator.setMaxIterations(1);

        // check correctness
        assertEquals(1, triangulator.getMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> triangulator.setMaxIterations(0));
    }

    @Test
    void testGetSetPointsAndCamerasAndIsReady() throws LockedException {
        final var triangulator = new PROMedSRobustSinglePoint3DTriangulator();

        // check default values
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isReady());

        // set new values
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
        assertFalse(triangulator.isReady());

        // if we set quality scores, instance becomes ready
        final var qualityScores = new double[2];
        triangulator.setQualityScores(qualityScores);

        assertTrue(triangulator.isReady());

        // Force IllegalArgumentException
        final var emptyPoints = new ArrayList<Point2D>();
        final var emptyCameras = new ArrayList<PinholeCamera>();
        assertThrows(IllegalArgumentException.class, () -> triangulator.setPointsAndCameras(emptyPoints, cameras));
        assertThrows(IllegalArgumentException.class, () -> triangulator.setPointsAndCameras(points, emptyCameras));
        assertThrows(IllegalArgumentException.class, () -> triangulator.setPointsAndCameras(emptyPoints, emptyCameras));
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var triangulator = new PROMedSRobustSinglePoint3DTriangulator();

        // check default value
        assertNull(triangulator.getQualityScores());

        // set new value
        final var qualityScores = new double[2];
        triangulator.setQualityScores(qualityScores);

        // check correctness
        assertSame(qualityScores, triangulator.getQualityScores());

        // Force IllegalArgumentException
        final var emptyScores = new double[0];
        assertThrows(IllegalArgumentException.class, () -> triangulator.setQualityScores(emptyScores));
    }

    @Test
    void testTriangulate() throws LockedException, NotReadyException, RobustEstimatorException {
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // obtain number of views
            final var numViews = randomizer.nextInt(MIN_VIEWS, MAX_VIEWS);

            // create a random 3D point
            final var point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            final var points2D = new ArrayList<Point2D>();
            final var cameras = new ArrayList<PinholeCamera>();
            final var qualityScores = new double[numViews];
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

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    final var errorX = errorRandomizer.nextDouble();
                    final var errorY = errorRandomizer.nextDouble();
                    final var error = Math.sqrt(errorX * errorX + errorY * errorY);
                    final var inhomX = point2D.getInhomX();
                    final var inhomY = point2D.getInhomY();

                    point2D.setInhomogeneousCoordinates(inhomX + errorX, inhomY + errorY);

                    qualityScores[i] = 1.0 / (1.0 + error);
                } else {
                    // inlier
                    qualityScores[i] = 1.0;
                }

                cameras.add(camera);
                points2D.add(point2D);
            }

            // create triangulator
            final var triangulator = new PROMedSRobustSinglePoint3DTriangulator(points2D, cameras, qualityScores,
                    this);
            triangulator.setStopThreshold(THRESHOLD);

            // check default values
            assertTrue(triangulator.isReady());
            assertFalse(triangulator.isLocked());
            assertEquals(0, triangulateStart);
            assertEquals(0, triangulateEnd);
            assertEquals(0, triangulateNextIteration);
            assertEquals(0, triangulateProgressChange);

            final var triangulated = triangulator.triangulate();

            // check correctness
            assertTrue(triangulator.isReady());
            assertFalse(triangulator.isLocked());
            assertEquals(1, triangulateStart);
            assertEquals(1, triangulateEnd);
            assertTrue(triangulateNextIteration > 0);
            assertTrue(triangulateProgressChange >= 0);
            reset();

            assertEquals(0.0, point3D.distanceTo(triangulated), ABSOLUTE_ERROR);
        }
    }

    private void reset() {
        triangulateStart = triangulateEnd = triangulateNextIteration = triangulateProgressChange = 0;
    }

    @Override
    public void onTriangulateStart(final RobustSinglePoint3DTriangulator triangulator) {
        triangulateStart++;
        checkLocked((PROMedSRobustSinglePoint3DTriangulator) triangulator);
    }

    @Override
    public void onTriangulateEnd(final RobustSinglePoint3DTriangulator triangulator) {
        triangulateEnd++;
        checkLocked((PROMedSRobustSinglePoint3DTriangulator) triangulator);
    }

    @Override
    public void onTriangulateNextIteration(final RobustSinglePoint3DTriangulator triangulator, final int iteration) {
        triangulateNextIteration++;
        checkLocked((PROMedSRobustSinglePoint3DTriangulator) triangulator);
    }

    @Override
    public void onTriangulateProgressChange(final RobustSinglePoint3DTriangulator triangulator, final float progress) {
        triangulateProgressChange++;
        checkLocked((PROMedSRobustSinglePoint3DTriangulator) triangulator);
    }

    private void checkLocked(final PROMedSRobustSinglePoint3DTriangulator triangulator) {
        assertThrows(LockedException.class, () -> triangulator.setStopThreshold(0.5));
        assertThrows(LockedException.class, () -> triangulator.setListener(this));
        assertThrows(LockedException.class, () -> triangulator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> triangulator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> triangulator.setMaxIterations(1));
        assertThrows(LockedException.class, () -> triangulator.setPointsAndCameras(null, null));
        assertThrows(LockedException.class, triangulator::triangulate);
        assertTrue(triangulator.isLocked());
    }
}
