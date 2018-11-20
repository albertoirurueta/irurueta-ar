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
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class LMSEInhomogeneousSinglePoint3DTriangulatorTest implements
        SinglePoint3DTriangulatorListener {

    private static final int MIN_VIEWS = 2;
    private static final int MAX_VIEWS = 20;
    
    private static final double MIN_RANDOM_VALUE = 100.0;
    private static final double MAX_RANDOM_VALUE = 500.0;
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    
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
    
    private static final int TIMES = 100;
    
    private int triangulateStart;
    private int triangulateEnd;    
    
    public LMSEInhomogeneousSinglePoint3DTriangulatorTest() { }
    
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
        //test constructor without arguments
        LMSEInhomogeneousSinglePoint3DTriangulator triangulator =
                new LMSEInhomogeneousSinglePoint3DTriangulator();
        
        //check correctness
        assertEquals(triangulator.isLMSESolutionAllowed(),
                LMSEInhomogeneousSinglePoint3DTriangulator.
                DEFAULT_ALLOW_LMSE_SOLUTION);
        assertEquals(triangulator.getType(),
                Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR);
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertNull(triangulator.getListener());
        
        //test constructor with points and cameras
        List<Point2D> points = new ArrayList<>();
        points.add(Point2D.create());
        points.add(Point2D.create());
        
        List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        
        triangulator = new LMSEInhomogeneousSinglePoint3DTriangulator(points, 
                cameras);
        
        //check correctness
        assertEquals(triangulator.isLMSESolutionAllowed(),
                LMSEInhomogeneousSinglePoint3DTriangulator.
                DEFAULT_ALLOW_LMSE_SOLUTION);
        assertEquals(triangulator.getType(),
                Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR);
        assertSame(triangulator.getPoints2D(), points);
        assertSame(triangulator.getCameras(), cameras);
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertNull(triangulator.getListener());
        
        //force IllegalArgumentException
        List<Point2D> emptyPoints = new ArrayList<>();
        List<PinholeCamera> emptyCameras = new ArrayList<>();
        
        triangulator = null;
        try {
            triangulator = new LMSEInhomogeneousSinglePoint3DTriangulator(
                    emptyPoints, cameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            triangulator = new LMSEInhomogeneousSinglePoint3DTriangulator(
                    points, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            triangulator = new LMSEInhomogeneousSinglePoint3DTriangulator(
                    emptyPoints, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(triangulator);
        
        //test constructor with listener
        triangulator = new LMSEInhomogeneousSinglePoint3DTriangulator(this);
        
        //check correctness
        assertEquals(triangulator.isLMSESolutionAllowed(),
                LMSEInhomogeneousSinglePoint3DTriangulator.
                DEFAULT_ALLOW_LMSE_SOLUTION);
        assertEquals(triangulator.getType(),
                Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR);
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertFalse(triangulator.isLocked());
        assertFalse(triangulator.isReady());
        assertSame(triangulator.getListener(), this);
        
        //test constructor with points, cameras and listener
        triangulator = new LMSEInhomogeneousSinglePoint3DTriangulator(points, 
                cameras, this);
        
        //check correctness
        assertEquals(triangulator.isLMSESolutionAllowed(),
                LMSEInhomogeneousSinglePoint3DTriangulator.
                DEFAULT_ALLOW_LMSE_SOLUTION);
        assertEquals(triangulator.getType(),
                Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR);
        assertSame(triangulator.getPoints2D(), points);
        assertSame(triangulator.getCameras(), cameras);
        assertFalse(triangulator.isLocked());
        assertTrue(triangulator.isReady());
        assertSame(triangulator.getListener(), this);
        
        //force IllegalArgumentException        
        triangulator = null;
        try {
            triangulator = new LMSEInhomogeneousSinglePoint3DTriangulator(
                    emptyPoints, cameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            triangulator = new LMSEInhomogeneousSinglePoint3DTriangulator(
                    points, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            triangulator = new LMSEInhomogeneousSinglePoint3DTriangulator(
                    emptyPoints, emptyCameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(triangulator);        
    }
    
    @Test
    public void testIsSetLMSeSolutionAllowed() throws LockedException {
        LMSEInhomogeneousSinglePoint3DTriangulator triangulator =
                new LMSEInhomogeneousSinglePoint3DTriangulator();
        
        //check default value
        assertEquals(triangulator.isLMSESolutionAllowed(),
                LMSEInhomogeneousSinglePoint3DTriangulator.
                DEFAULT_ALLOW_LMSE_SOLUTION);
        
        //set new value
        triangulator.setLMSESolutionAllowed(
                !LMSEInhomogeneousSinglePoint3DTriangulator.
                DEFAULT_ALLOW_LMSE_SOLUTION);
        
        //check correctness
        assertEquals(triangulator.isLMSESolutionAllowed(),
                !LMSEInhomogeneousSinglePoint3DTriangulator.
                DEFAULT_ALLOW_LMSE_SOLUTION);        
    }
    
    @Test
    public void testGetSetPointsAndCameras() throws LockedException {
        LMSEInhomogeneousSinglePoint3DTriangulator triangulator =
                new LMSEInhomogeneousSinglePoint3DTriangulator();
        
        //check default values
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        
        //set new value
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
    public void testGetSetListener() throws LockedException {
        LMSEInhomogeneousSinglePoint3DTriangulator triangulator =
                new LMSEInhomogeneousSinglePoint3DTriangulator();
        
        //check default value
        assertNull(triangulator.getListener());
        
        //set new value
        triangulator.setListener(this);
        
        //check correctness
        assertSame(triangulator.getListener(), this);
    }
    
    @Test
    public void testTriangulate() throws LockedException, NotReadyException,
            Point3DTriangulationException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //obtain number of views
            int numViews = randomizer.nextInt(MIN_VIEWS, MAX_VIEWS);

            //create a random 3D point
            Point3D point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

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

                cameras.add(camera);
                points2D.add(point2D);
            }

            //create triangulator
            LMSEInhomogeneousSinglePoint3DTriangulator triangulator = 
                    new LMSEInhomogeneousSinglePoint3DTriangulator(points2D, 
                    cameras, this);

            //check default values
            assertTrue(triangulator.isReady());
            assertFalse(triangulator.isLocked());
            assertEquals(triangulateStart, 0);
            assertEquals(triangulateEnd, 0);

            Point3D triangulated = triangulator.triangulate();

            //check correctness
            assertTrue(triangulator.isReady());
            assertFalse(triangulator.isLocked());
            assertEquals(triangulateStart, 1);
            assertEquals(triangulateEnd, 1);
            reset();

            if (point3D.distanceTo(triangulated) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(point3D.distanceTo(triangulated), 0.0, ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onTriangulateStart(SinglePoint3DTriangulator triangulator) {
        triangulateStart++;
        checkLocked((LMSEInhomogeneousSinglePoint3DTriangulator)triangulator);
    }

    @Override
    public void onTriangulateEnd(SinglePoint3DTriangulator triangulator) {
        triangulateEnd++;
        checkLocked((LMSEInhomogeneousSinglePoint3DTriangulator)triangulator);
    }

    private void reset(){
        triangulateStart = triangulateEnd = 0;
    }

    private void checkLocked(
            LMSEInhomogeneousSinglePoint3DTriangulator triangulator) {
        try {
            triangulator.setLMSESolutionAllowed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            triangulator.setListener(this);
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
