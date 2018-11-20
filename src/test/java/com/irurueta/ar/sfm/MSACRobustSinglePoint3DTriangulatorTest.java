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

import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class MSACRobustSinglePoint3DTriangulatorTest implements
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
    
    public MSACRobustSinglePoint3DTriangulatorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        MSACRobustSinglePoint3DTriangulator triangulator;
        
        //test constructor without arguments
        triangulator = new MSACRobustSinglePoint3DTriangulator();
        
        //check correctness
        assertEquals(triangulator.getThreshold(), 
                MSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(triangulator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(triangulator.isUseHomogeneousSolution(),
                MSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
        assertFalse(triangulator.isLocked());
        assertEquals(triangulator.getProgressDelta(), 
                MSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(triangulator.getConfidence(), 
                MSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(triangulator.getMaxIterations(), 
                MSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS);
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        
        //test constructor with listener
        triangulator = new MSACRobustSinglePoint3DTriangulator(this);
        
        //check correctness
        assertEquals(triangulator.getThreshold(), 
                MSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(triangulator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(triangulator.getListener(), this);
        assertTrue(triangulator.isListenerAvailable());
        assertEquals(triangulator.isUseHomogeneousSolution(),
                MSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
        assertFalse(triangulator.isLocked());
        assertEquals(triangulator.getProgressDelta(), 
                MSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(triangulator.getConfidence(), 
                MSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(triangulator.getMaxIterations(), 
                MSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS);
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());     
        
        //test constructor with points and cameras
        List<Point2D> points = new ArrayList<>();
        points.add(Point2D.create());
        points.add(Point2D.create());
        
        List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        
        triangulator = new MSACRobustSinglePoint3DTriangulator(points, 
                cameras);
        
        //check correctness
        assertEquals(triangulator.getThreshold(), 
                MSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(triangulator.getMethod(), RobustEstimatorMethod.MSAC);
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(triangulator.isUseHomogeneousSolution(),
                MSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
        assertFalse(triangulator.isLocked());
        assertEquals(triangulator.getProgressDelta(), 
                MSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(triangulator.getConfidence(), 
                MSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(triangulator.getMaxIterations(), 
                MSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS);
        assertSame(triangulator.getPoints2D(), points);
        assertSame(triangulator.getCameras(), cameras);
        assertNull(triangulator.getQualityScores());
        assertTrue(triangulator.isReady());        
        
        //Force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<>();
        List<PinholeCamera> emptyCameras = new ArrayList<>();
        
        triangulator = null;
        try {
            triangulator = new MSACRobustSinglePoint3DTriangulator(
                    emptyPoints, cameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            triangulator = new MSACRobustSinglePoint3DTriangulator(
                    points, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            triangulator = new MSACRobustSinglePoint3DTriangulator(
                    emptyPoints, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(triangulator);
        
        //test constructor with points, cameras and listener
        triangulator = new MSACRobustSinglePoint3DTriangulator(points, 
                cameras, this);
        
        //check correctness
        assertEquals(triangulator.getThreshold(), 
                MSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(triangulator.getMethod(), RobustEstimatorMethod.MSAC);
        assertSame(triangulator.getListener(), this);
        assertTrue(triangulator.isListenerAvailable());
        assertEquals(triangulator.isUseHomogeneousSolution(),
                MSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
        assertFalse(triangulator.isLocked());
        assertEquals(triangulator.getProgressDelta(), 
                MSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        assertEquals(triangulator.getConfidence(), 
                MSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(triangulator.getMaxIterations(), 
                MSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS);
        assertSame(triangulator.getPoints2D(), points);
        assertSame(triangulator.getCameras(), cameras);
        assertNull(triangulator.getQualityScores());
        assertTrue(triangulator.isReady());        
        
        //Force IllegalArgumentException
        triangulator = null;
        try {
            triangulator = new MSACRobustSinglePoint3DTriangulator(
                    emptyPoints, cameras, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            triangulator = new MSACRobustSinglePoint3DTriangulator(
                    points, emptyCameras, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            triangulator = new MSACRobustSinglePoint3DTriangulator(
                    emptyPoints, emptyCameras, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(triangulator);        
    }
    
    @Test
    public void testGetSetThreshold() throws LockedException {
        MSACRobustSinglePoint3DTriangulator triangulator = 
                new MSACRobustSinglePoint3DTriangulator();
        
        //check default value
        assertEquals(triangulator.getThreshold(),
                MSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD, 0.0);
        
        //set new value
        triangulator.setThreshold(0.5);
        
        //check correctness
        assertEquals(triangulator.getThreshold(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            triangulator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetListener() throws LockedException {
        MSACRobustSinglePoint3DTriangulator triangulator =
                new MSACRobustSinglePoint3DTriangulator();
        
        //check default value
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        
        //set new value
        triangulator.setListener(this);
        
        //check correctness
        assertSame(triangulator.getListener(), this);
        assertTrue(triangulator.isListenerAvailable());
    }
    
    @Test
    public void testIsSetUseHomogeneousSolution() throws LockedException {
        MSACRobustSinglePoint3DTriangulator triangulator =
                new MSACRobustSinglePoint3DTriangulator();
        
        //check default value
        assertEquals(triangulator.isUseHomogeneousSolution(),
                MSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
        
        //set new value
        triangulator.setUseHomogeneousSolution(
                !MSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
        
        //check correctness
        assertEquals(triangulator.isUseHomogeneousSolution(),
                !MSACRobustSinglePoint3DTriangulator.
                DEFAULT_USE_HOMOGENEOUS_SOLUTION);
    }
    
    @Test
    public void testGetSetProgressDelta() throws LockedException {
        MSACRobustSinglePoint3DTriangulator triangulator =
                new MSACRobustSinglePoint3DTriangulator();
        
        //check default value
        assertEquals(triangulator.getProgressDelta(),
                MSACRobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, 
                0.0);
        
        //set new value
        triangulator.setProgressDelta(0.5f);
        
        //check correctness
        assertEquals(triangulator.getProgressDelta(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            triangulator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            triangulator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetConfidence() throws LockedException {
        MSACRobustSinglePoint3DTriangulator triangulator =
                new MSACRobustSinglePoint3DTriangulator();

        //check default value
        assertEquals(triangulator.getConfidence(),
                MSACRobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, 0.0);
        
        //set new value
        triangulator.setConfidence(0.5);
        
        //check correctness
        assertEquals(triangulator.getConfidence(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            triangulator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            triangulator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetMaxIterations() throws LockedException {
        MSACRobustSinglePoint3DTriangulator triangulator =
                new MSACRobustSinglePoint3DTriangulator();
        
        //check default value
        assertEquals(triangulator.getMaxIterations(),
                MSACRobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS);
        
        //set new value
        triangulator.setMaxIterations(1);
        
        //check correctness
        assertEquals(triangulator.getMaxIterations(), 1);
        
        //Force IllegalArgumentException
        try {
            triangulator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetPointsAndCamerasAndIsReady() throws LockedException {
        MSACRobustSinglePoint3DTriangulator triangulator =
                new MSACRobustSinglePoint3DTriangulator();
        
        //check default values
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isReady());
        
        //set new values
        List<Point2D> points = new ArrayList<>();
        points.add(Point2D.create());
        points.add(Point2D.create());
        
        List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        
        triangulator.setPointsAndCameras(points, cameras);
        
        //check correctness
        assertSame(triangulator.getPoints2D(), points);
        assertSame(triangulator.getCameras(), cameras);
        assertTrue(triangulator.isReady());
        
        //Force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<>();
        List<PinholeCamera> emptyCameras = new ArrayList<>();
        
        try {
            triangulator.setPointsAndCameras(emptyPoints, cameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            triangulator.setPointsAndCameras(points, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            triangulator.setPointsAndCameras(emptyPoints, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetQualityScores() throws LockedException {
        MSACRobustSinglePoint3DTriangulator triangulator =
                new MSACRobustSinglePoint3DTriangulator();
        
        //check default value
        assertNull(triangulator.getQualityScores());
        
        //set new value
        double[] qualityScores = new double[2];
        triangulator.setQualityScores(qualityScores);
        
        //check correctness
        assertNull(triangulator.getQualityScores());
    }
    
    @Test
    public void testTriangulate() throws LockedException, NotReadyException, 
            RobustEstimatorException {
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //obtain number of views
            int numViews = randomizer.nextInt(MIN_VIEWS, MAX_VIEWS);

            //create a random 3D point
            Point3D point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            List<Point2D> points2D = new ArrayList<>();
            List<PinholeCamera> cameras = new ArrayList<>();
            Point3D previousCameraCenter = new InhomogeneousPoint3D();
            for (int i = 0; i < numViews; i++) {
                //create a random camera
                double horizontalFocalLength = randomizer.nextDouble(
                        MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
                double verticalFocalLength = randomizer.nextDouble(
                        MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
                double skewness = randomizer.nextDouble(MIN_SKEWNESS, 
                        MAX_SKEWNESS);
                double horizontalPrincipalPoint = randomizer.nextDouble(
                        MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
                double verticalPrincipalPoint = randomizer.nextDouble(
                        MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

                double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                        MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                        MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                        MAX_ANGLE_DEGREES) * Math.PI / 180.0;

                double cameraSeparationX = randomizer.nextDouble(
                        MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
                double cameraSeparationY = randomizer.nextDouble(
                        MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
                double cameraSeparationZ = randomizer.nextDouble(
                        MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

                PinholeCameraIntrinsicParameters intrinsic = 
                        new PinholeCameraIntrinsicParameters(
                        horizontalFocalLength, verticalFocalLength, 
                        horizontalPrincipalPoint, verticalPrincipalPoint, 
                        skewness);            

                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, 
                        betaEuler, gammaEuler);

                Point3D cameraCenter = new InhomogeneousPoint3D(
                        previousCameraCenter.getInhomX() + cameraSeparationX,
                        previousCameraCenter.getInhomY() + cameraSeparationY,
                        previousCameraCenter.getInhomZ() + cameraSeparationZ);

                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);

                //project 3D point using camera
                Point2D point2D = camera.project(point3D);
                
                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    double errorX = errorRandomizer.nextDouble();
                    double errorY = errorRandomizer.nextDouble();
                    double inhomX = point2D.getInhomX();
                    double inhomY = point2D.getInhomY();
                    
                    point2D.setInhomogeneousCoordinates(inhomX + errorX, 
                            inhomY + errorY);
                }

                cameras.add(camera);
                points2D.add(point2D);
            }

            //create triangulator
            MSACRobustSinglePoint3DTriangulator triangulator = 
                    new MSACRobustSinglePoint3DTriangulator(points2D, cameras, 
                    this);
            triangulator.setThreshold(THRESHOLD);
            
            //check default values
            assertTrue(triangulator.isReady());
            assertFalse(triangulator.isLocked());
            assertEquals(triangulateStart, 0);
            assertEquals(triangulateEnd, 0);
            assertEquals(triangulateNextIteration, 0);
            assertEquals(triangulateProgressChange, 0);
            
            Point3D triangulated = triangulator.triangulate();
            
            //check correctness
            assertTrue(triangulator.isReady());
            assertFalse(triangulator.isLocked());
            assertEquals(triangulateStart, 1);
            assertEquals(triangulateEnd, 1);
            assertTrue(triangulateNextIteration > 0);
            assertTrue(triangulateProgressChange >= 0);
            reset();
            
            assertEquals(point3D.distanceTo(triangulated), 0.0, ABSOLUTE_ERROR);
        }
    }

    @Override
    public void onTriangulateStart(
            RobustSinglePoint3DTriangulator triangulator) {
        triangulateStart++;
        checkLocked((MSACRobustSinglePoint3DTriangulator)triangulator);
    }

    @Override
    public void onTriangulateEnd(RobustSinglePoint3DTriangulator triangulator) {
        triangulateEnd++;
        checkLocked((MSACRobustSinglePoint3DTriangulator)triangulator);
    }

    @Override
    public void onTriangulateNextIteration(
            RobustSinglePoint3DTriangulator triangulator, int iteration) {
        triangulateNextIteration++;
        checkLocked((MSACRobustSinglePoint3DTriangulator)triangulator);
    }

    @Override
    public void onTriangulateProgressChange(
            RobustSinglePoint3DTriangulator triangulator, float progress) {
        triangulateProgressChange++;
        checkLocked((MSACRobustSinglePoint3DTriangulator)triangulator);
    }

    private void reset() {
        triangulateStart = triangulateEnd = triangulateNextIteration =
                triangulateProgressChange = 0;
    }

    private void checkLocked(MSACRobustSinglePoint3DTriangulator triangulator) {
        try {
            triangulator.setThreshold(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            triangulator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            triangulator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            triangulator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            triangulator.setMaxIterations(1);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            triangulator.setPointsAndCameras(null, null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            triangulator.triangulate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
        assertTrue(triangulator.isLocked());                
    }
}
