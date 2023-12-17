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
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PROSACRobustSinglePoint3DTriangulatorTest implements
        RobustSinglePoint3DTriangulatorListener {

    private static final int MIN_VIEWS = 100;
    private static final int MAX_VIEWS = 500;

    private static final double MIN_RANDOM_VALUE = 100.0;
    private static final double MAX_RANDOM_VALUE = 500.0;

    private static final double ABSOLUTE_ERROR = 5e-5;

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

    private static final int TIMES = 100;

    private int triangulateStart;
    private int triangulateEnd;
    private int triangulateNextIteration;
    private int triangulateProgressChange;

    @Test
    public void testConstructor() {
        //test constructor without arguments
        PROSACRobustSinglePoint3DTriangulator triangulator = new PROSACRobustSinglePoint3DTriangulator();

        // check correctness
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD,
                triangulator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, triangulator.getMethod());
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());

        // test constructor with listener
        triangulator = new PROSACRobustSinglePoint3DTriangulator(this);

        // check correctness
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD,
                triangulator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, triangulator.getMethod());
        assertSame(this, triangulator.getListener());
        assertTrue(triangulator.isListenerAvailable());
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());

        // test constructor with points and cameras
        final List<Point2D> points = new ArrayList<>();
        points.add(Point2D.create());
        points.add(Point2D.create());

        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        triangulator = new PROSACRobustSinglePoint3DTriangulator(points, cameras);

        // check correctness
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD,
                triangulator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, triangulator.getMethod());
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());

        // Force IllegalArgumentException
        final List<Point2D> emptyPoints = new ArrayList<>();
        final List<PinholeCamera> emptyCameras = new ArrayList<>();

        triangulator = null;
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(emptyPoints, cameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(points, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(emptyPoints, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(triangulator);

        // test constructor with points, cameras and listener
        triangulator = new PROSACRobustSinglePoint3DTriangulator(points, cameras, this);

        // check correctness
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD,
                triangulator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, triangulator.getMethod());
        assertSame(this, triangulator.getListener());
        assertTrue(triangulator.isListenerAvailable());
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());

        // Force IllegalArgumentException
        triangulator = null;
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(emptyPoints, cameras, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(points, emptyCameras, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(emptyPoints, emptyCameras, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(triangulator);

        // test constructor with quality scores
        final double[] qualityScores = new double[2];
        triangulator = new PROSACRobustSinglePoint3DTriangulator(qualityScores);

        // check correctness
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD,
                triangulator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, triangulator.getMethod());
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertSame(qualityScores, triangulator.getQualityScores());
        assertFalse(triangulator.isReady());

        // Force IllegalArgumentException
        final double[] shortScores = new double[1];
        triangulator = null;
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(triangulator);

        // test constructor with quality scores and listener
        triangulator = new PROSACRobustSinglePoint3DTriangulator(qualityScores, this);

        // check correctness
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD,
                triangulator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, triangulator.getMethod());
        assertSame(this, triangulator.getListener());
        assertTrue(triangulator.isListenerAvailable());
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertSame(qualityScores, triangulator.getQualityScores());
        assertFalse(triangulator.isReady());

        // Force IllegalArgumentException
        triangulator = null;
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(shortScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(triangulator);

        // test constructor with points, cameras and quality scores
        triangulator = new PROSACRobustSinglePoint3DTriangulator(points, cameras, qualityScores);

        // check correctness
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD,
                triangulator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, triangulator.getMethod());
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertSame(qualityScores, triangulator.getQualityScores());
        assertTrue(triangulator.isReady());

        // Force IllegalArgumentException
        triangulator = null;
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    emptyPoints, cameras, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    points, emptyCameras, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    emptyPoints, emptyCameras, qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    emptyPoints, emptyCameras, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    points, cameras, shortScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(triangulator);

        // test constructor with points, cameras, quality scores and listener
        triangulator = new PROSACRobustSinglePoint3DTriangulator(points,
                cameras, qualityScores, this);

        // check correctness
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD,
                triangulator.getThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.PROSAC, triangulator.getMethod());
        assertSame(this, triangulator.getListener());
        assertTrue(triangulator.isListenerAvailable());
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertSame(qualityScores, triangulator.getQualityScores());
        assertTrue(triangulator.isReady());

        // Force IllegalArgumentException
        triangulator = null;
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    emptyPoints, cameras, qualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    points, emptyCameras, qualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    emptyPoints, emptyCameras, qualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    emptyPoints, emptyCameras, shortScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator = new PROSACRobustSinglePoint3DTriangulator(
                    points, cameras, shortScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(triangulator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final PROSACRobustSinglePoint3DTriangulator triangulator =
                new PROSACRobustSinglePoint3DTriangulator();

        // check default value
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD,
                triangulator.getThreshold(), 0.0);

        // set new value
        triangulator.setThreshold(0.5);

        // check correctness
        assertEquals(0.5, triangulator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        try {
            triangulator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final PROSACRobustSinglePoint3DTriangulator triangulator =
                new PROSACRobustSinglePoint3DTriangulator();

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
    public void testIsSetUseHomogeneousSolution() throws LockedException {
        final PROSACRobustSinglePoint3DTriangulator triangulator =
                new PROSACRobustSinglePoint3DTriangulator();

        // check default value
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());

        // set new value
        triangulator.setUseHomogeneousSolution(
                !PROSACRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION);

        // check correctness
        assertEquals(!PROSACRobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final PROSACRobustSinglePoint3DTriangulator triangulator =
                new PROSACRobustSinglePoint3DTriangulator();

        // check default value
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);

        // set new value
        triangulator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5, triangulator.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        try {
            triangulator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        final PROSACRobustSinglePoint3DTriangulator triangulator =
                new PROSACRobustSinglePoint3DTriangulator();

        // check default value
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);

        // set new value
        triangulator.setConfidence(0.5);

        // check correctness
        assertEquals(0.5, triangulator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        try {
            triangulator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triangulator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxIterations() throws LockedException {
        final PROSACRobustSinglePoint3DTriangulator triangulator =
                new PROSACRobustSinglePoint3DTriangulator();

        // check default value
        assertEquals(PROSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());

        // set new value
        triangulator.setMaxIterations(1);

        // check correctness
        assertEquals(1, triangulator.getMaxIterations());

        // Force IllegalArgumentException
        try {
            triangulator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetPointsAndCamerasAndIsReady() throws LockedException {
        final PROSACRobustSinglePoint3DTriangulator triangulator =
                new PROSACRobustSinglePoint3DTriangulator();

        // check default values
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isReady());

        // set new values
        final List<Point2D> points = new ArrayList<>();
        points.add(Point2D.create());
        points.add(Point2D.create());

        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        triangulator.setPointsAndCameras(points, cameras);

        // check correctness
        assertSame(triangulator.getPoints2D(), points);
        assertSame(triangulator.getCameras(), cameras);
        assertFalse(triangulator.isReady());

        // if we set quality scores, instance becomes ready
        final double[] qualityScores = new double[2];
        triangulator.setQualityScores(qualityScores);

        assertTrue(triangulator.isReady());

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
    public void testGetSetQualityScores() throws LockedException {
        final PROSACRobustSinglePoint3DTriangulator triangulator =
                new PROSACRobustSinglePoint3DTriangulator();

        // check default value
        assertNull(triangulator.getQualityScores());

        // set new value
        final double[] qualityScores = new double[2];
        triangulator.setQualityScores(qualityScores);

        // check correctness
        assertSame(qualityScores, triangulator.getQualityScores());

        // Force IllegalArgumentException
        final double[] emptyScores = new double[0];
        try {
            triangulator.setQualityScores(emptyScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testTriangulate() throws LockedException, NotReadyException,
            RobustEstimatorException {
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

            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final List<Point2D> points2D = new ArrayList<>();
            final List<PinholeCamera> cameras = new ArrayList<>();
            final double[] qualityScores = new double[numViews];
            final Point3D previousCameraCenter = new InhomogeneousPoint3D();
            for (int i = 0; i < numViews; i++) {
                // create a random camera
                final double horizontalFocalLength = randomizer.nextDouble(
                        MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
                final double verticalFocalLength = randomizer.nextDouble(
                        MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
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

                final PinholeCameraIntrinsicParameters intrinsic =
                        new PinholeCameraIntrinsicParameters(
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

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    final double errorX = errorRandomizer.nextDouble();
                    final double errorY = errorRandomizer.nextDouble();
                    final double error = Math.sqrt(errorX * errorX + errorY * errorY);
                    final double inhomX = point2D.getInhomX();
                    final double inhomY = point2D.getInhomY();

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
            final PROSACRobustSinglePoint3DTriangulator triangulator =
                    new PROSACRobustSinglePoint3DTriangulator(points2D, cameras, qualityScores, this);
            triangulator.setThreshold(THRESHOLD);

            // check default values
            assertTrue(triangulator.isReady());
            assertFalse(triangulator.isLocked());
            assertEquals(0, triangulateStart);
            assertEquals(0, triangulateEnd);
            assertEquals(0, triangulateNextIteration);
            assertEquals(0, triangulateProgressChange);

            final Point3D triangulated = triangulator.triangulate();

            // check correctness
            assertTrue(triangulator.isReady());
            assertFalse(triangulator.isLocked());
            assertEquals(1, triangulateStart);
            assertEquals(1, triangulateEnd);
            assertTrue(triangulateNextIteration > 0);
            assertTrue(triangulateProgressChange >= 0);
            reset();

            if (point3D.distanceTo(triangulated) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, point3D.distanceTo(triangulated), ABSOLUTE_ERROR);

            numValid++;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onTriangulateStart(
            final RobustSinglePoint3DTriangulator triangulator) {
        triangulateStart++;
        checkLocked((PROSACRobustSinglePoint3DTriangulator) triangulator);
    }

    @Override
    public void onTriangulateEnd(
            final RobustSinglePoint3DTriangulator triangulator) {
        triangulateEnd++;
        checkLocked((PROSACRobustSinglePoint3DTriangulator) triangulator);
    }

    @Override
    public void onTriangulateNextIteration(
            final RobustSinglePoint3DTriangulator triangulator, final int iteration) {
        triangulateNextIteration++;
        checkLocked((PROSACRobustSinglePoint3DTriangulator) triangulator);
    }

    @Override
    public void onTriangulateProgressChange(
            final RobustSinglePoint3DTriangulator triangulator, final float progress) {
        triangulateProgressChange++;
        checkLocked((PROSACRobustSinglePoint3DTriangulator) triangulator);
    }

    private void reset() {
        triangulateStart = triangulateEnd = triangulateNextIteration =
                triangulateProgressChange = 0;
    }

    private void checkLocked(final PROSACRobustSinglePoint3DTriangulator triangulator) {
        try {
            triangulator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            triangulator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            triangulator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            triangulator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            triangulator.setMaxIterations(1);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            triangulator.setPointsAndCameras(null, null);
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
