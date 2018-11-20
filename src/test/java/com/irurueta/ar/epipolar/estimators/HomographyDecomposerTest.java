/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.ar.epipolar.estimators;

import com.irurueta.algebra.*;
import com.irurueta.ar.epipolar.EssentialMatrix;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.geometry.*;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.ProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class HomographyDecomposerTest implements HomographyDecomposerListener {
    
    private static final double MIN_FOCAL_LENGTH = 750.0;
    private static final double MAX_FOCAL_LENGTH = 1500.0;
    
    private static final double MIN_ANGLE_DEGREES = -30.0;
    private static final double MAX_ANGLE_DEGREES = -15.0;

    private static final double MIN_CAMERA_SEPARATION = 500.0;
    private static final double MAX_CAMERA_SEPARATION = 1000.0;
    
    private static final int MIN_NUM_POINTS = 25;
    private static final int MAX_NUM_POINTS = 50;
    
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    
    private static final double MIN_RANDOM_VALUE_PLANAR = -1500.0;
    private static final double MAX_RANDOM_VALUE_PLANAR = 1500.0;
    
    private static final int TIMES = 500;
    private static final int MAX_TRIES = 5000;
    
    private static final double HOMOGRAPHY_ESTIMATOR_THRESHOLD = 1e-8;
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;
    
    private int decomposeStart;
    private int decomposeEnd;
    
    public HomographyDecomposerTest() { }
    
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
        //empty constructor
        HomographyDecomposer decomposer = new HomographyDecomposer();
        
        //check default values
        assertNull(decomposer.getHomography());
        assertNull(decomposer.getLeftIntrinsics());
        assertNull(decomposer.getRightIntrinsics());
        assertNull(decomposer.getListener());
        assertFalse(decomposer.isLocked());
        assertFalse(decomposer.isReady());
        
        //test constructor with homography and intrinsics
        ProjectiveTransformation2D homography = 
                new ProjectiveTransformation2D();
        PinholeCameraIntrinsicParameters leftIntrinsics = 
                new PinholeCameraIntrinsicParameters();
        PinholeCameraIntrinsicParameters rightIntrinsics =
                new PinholeCameraIntrinsicParameters();
        decomposer = new HomographyDecomposer(homography, leftIntrinsics, 
                rightIntrinsics);
        
        //check default values
        assertSame(decomposer.getHomography(), homography);
        assertSame(decomposer.getLeftIntrinsics(), leftIntrinsics);
        assertSame(decomposer.getRightIntrinsics(), rightIntrinsics);
        assertNull(decomposer.getListener());
        assertFalse(decomposer.isLocked());
        assertTrue(decomposer.isReady());
        
        //test constructor with homography, intrinsics and listener
        decomposer = new HomographyDecomposer(homography, leftIntrinsics, 
                rightIntrinsics, this);   
        
        //check default values
        assertSame(decomposer.getHomography(), homography);
        assertSame(decomposer.getLeftIntrinsics(), leftIntrinsics);
        assertSame(decomposer.getRightIntrinsics(), rightIntrinsics);
        assertSame(decomposer.getListener(), this);
        assertFalse(decomposer.isLocked());
        assertTrue(decomposer.isReady());        
    }
    
    @Test
    public void testGetSetHomography() throws LockedException {
        HomographyDecomposer decomposer = new HomographyDecomposer();
        
        //check default value
        assertNull(decomposer.getHomography());
        
        //set new value
        ProjectiveTransformation2D homography = 
                new ProjectiveTransformation2D();
        decomposer.setHomography(homography);
        
        //check correctness
        assertSame(decomposer.getHomography(), homography);
    }
    
    @Test
    public void testGetSetLeftIntrinsics() throws LockedException {
        HomographyDecomposer decomposer = new HomographyDecomposer();
        
        //check default value
        assertNull(decomposer.getLeftIntrinsics());
        
        PinholeCameraIntrinsicParameters leftIntrinsics = 
                new PinholeCameraIntrinsicParameters();
        decomposer.setLeftIntrinsics(leftIntrinsics);
        
        //check correctness
        assertSame(decomposer.getLeftIntrinsics(), leftIntrinsics);
    }
    
    @Test
    public void testGetSetRightIntrinsics() throws LockedException {
        HomographyDecomposer decomposer = new HomographyDecomposer();
        
        //check default value
        assertNull(decomposer.getRightIntrinsics());
        
        PinholeCameraIntrinsicParameters rightIntrinsics =
                new PinholeCameraIntrinsicParameters();
        decomposer.setRightIntrinsics(rightIntrinsics);
        
        //check correctness
        assertSame(decomposer.getRightIntrinsics(), rightIntrinsics);
    }
    
    @Test
    public void testGetSetListener() {
        HomographyDecomposer decomposer = new HomographyDecomposer();
        
        //check default value
        assertNull(decomposer.getListener());
        
        //set new value
        decomposer.setListener(this);
        
        //check correctness
        assertSame(decomposer.getListener(), this);
    }
    
    @Test
    public void testDecompose() throws AlgebraException, GeometryException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            double focalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);            
            double skewness = 0.0;
            double principalPointX = 0.0;
            double principalPointY = 0.0;
            
            
            PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(focalLength1, 
                    focalLength1, principalPointX, principalPointY, skewness);
            PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(focalLength2,
                    focalLength2, principalPointX, principalPointY, skewness);
            
            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, 
                    MAX_CAMERA_SEPARATION);
        
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, 
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, 
                    betaEuler2, gammaEuler2);
        
            PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);
            
            //create 3D points laying in front of both cameras and laying in
            //a plane
        
            //1st find an approximate central point by intersecting the axis 
            //planes of both cameras
            Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());
        
            planesIntersectionMatrix.setElementAt(1, 0, 
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, 
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, 
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, 
                    horizontalPlane1.getD());
        
            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());
        
            planesIntersectionMatrix.setElementAt(3, 0, 
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, 
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, 
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, 
                    horizontalPlane2.getD());
        
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            Matrix v = decomposer.getV();
            HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));
            
            double[] principalAxis1 = camera1.getPrincipalAxisArray();
            double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double[] avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2), 
                    0.5);
            
            Plane plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();
        
            double planeA = plane.getA();
            double planeB = plane.getB();
            double planeC = plane.getC();
            double planeD = plane.getD();
            
            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS, 
                    MAX_NUM_POINTS);
        
            HomogeneousPoint3D point3D;
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1, front2;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {                
                    //get a random point belonging to the plane
                    //a*x + b*y + c*z + d*w = 0
                    //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    double homX, homY;
                    double homW = 1.0;
                    double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, 
                                MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) / 
                                planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, 
                                MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) / 
                                planeA;
                    }
                
                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                    assertTrue(plane.isLocus(point3D));
                
                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while(!front1 || !front2);
                
                //check that 3D point is in front of both cameras
                //noinspection all
                assertTrue(front1);
                //noinspection all
                assertTrue(front2);
            
                //project 3D point into both cameras
                projectedPoint1 = new HomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoint1.normalize();
                
                projectedPoints1.add(projectedPoint1);
            
                projectedPoint2 = new HomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoint2.normalize();
                
                projectedPoints2.add(projectedPoint2);
            }
            
            //estimate homography
            ProjectiveTransformation2DRobustEstimator homographyEstimator =
                    ProjectiveTransformation2DRobustEstimator.createFromPoints(
                    projectedPoints1, projectedPoints2, 
                    RobustEstimatorMethod.RANSAC);
            ((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                    homographyEstimator).setThreshold(
                    HOMOGRAPHY_ESTIMATOR_THRESHOLD);
            ((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                    homographyEstimator).setComputeAndKeepInliersEnabled(true);
            ((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                    homographyEstimator).setComputeAndKeepResidualsEnabled(true);            
            homographyEstimator.setResultRefined(false);
            ProjectiveTransformation2D homography;
            try {
                homography = homographyEstimator.estimate();
            } catch (RobustEstimatorException e) {
                continue;
            }
            
            homography.normalize();
            
            //check errors on estimated homography
            boolean failed = false;
            for (int i = 0; i < numPoints; i++) {
                projectedPoint1 = new InhomogeneousPoint2D(
                        projectedPoints1.get(i));
                projectedPoint2 = new InhomogeneousPoint2D(
                        projectedPoints2.get(i));
                
                InhomogeneousPoint2D projectedPoint2b = 
                        new InhomogeneousPoint2D();
                homography.transform(projectedPoint1, projectedPoint2b);
                
                if (!projectedPoint2.equals(projectedPoint2b, 
                        LARGE_ABSOLUTE_ERROR)) {
                    failed = true;
                    break;
                }
                assertTrue(projectedPoint2.equals(projectedPoint2b, 
                        LARGE_ABSOLUTE_ERROR));
            }
            
            if (failed) {
                continue;
            }
            
            HomographyDecomposer homographyDecomposer = 
                    new HomographyDecomposer(homography, intrinsic1, intrinsic2,
                    this);
            
            reset();
            assertFalse(decomposer.isLocked());
            assertTrue(decomposer.isReady());
            assertEquals(decomposeStart, 0);
            assertEquals(decomposeEnd, 0);
            
            List<HomographyDecomposition> result1 =
                    homographyDecomposer.decompose();
            List<HomographyDecomposition> result2 = 
                    new ArrayList<>();
            homographyDecomposer.decompose(result2);
            
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);
            fundamentalMatrix.normalize();
            
            //check that both results are equal
            assertEquals(result1.size(), result2.size());
            assertTrue(result1.size() == 2 || result1.size() == 4);

            int numValidDecompositions = 0;
            boolean atLeastOneValidFundamentalMatrix = false;
            for (int i = 0; i < result1.size(); i++) {
                //check that both result1 and result2 are equal
                assertEquals(result1.get(i).getPlaneDistance(),
                        result2.get(i).getPlaneDistance(), ABSOLUTE_ERROR);
                assertArrayEquals(result1.get(i).getPlaneNormal(),
                        result2.get(i).getPlaneNormal(), ABSOLUTE_ERROR);
                assertEquals(result1.get(i).getTransformation().getRotation(),
                        result2.get(i).getTransformation().getRotation());
                assertArrayEquals(result1.get(i).getTransformation().
                        getTranslation(),
                        result2.get(i).getTransformation().getTranslation(),
                        ABSOLUTE_ERROR);
                
                //check that each homography decomposition generates the 
                //original homography
                if (validDecomposition(homography, result1.get(i), intrinsic1,
                        intrinsic2)) {
                    numValidDecompositions++;
                }
                
                
                //check that at least one of the fundamental matrices generated
                //by one of the solutions is valid (up to scale)
                HomographyDecomposition decomposition = result1.get(i);
                Rotation3D rotation = decomposition.getTransformation().
                        getRotation();
                Point2D translation = new HomogeneousPoint2D(decomposition.
                        getTransformation().getTranslation());
                EssentialMatrix essential1 = new EssentialMatrix(rotation, 
                        translation); 
                FundamentalMatrix fundamentalMatrixB = essential1.
                        toFundamentalMatrix(intrinsic1, intrinsic2);
                fundamentalMatrixB.normalize();
                if (areEqualUpToScale(fundamentalMatrix, fundamentalMatrixB
                )) {
                    atLeastOneValidFundamentalMatrix = true;
                }
            }
            
            if (numValidDecompositions < result1.size()) {
                continue;
            }
            
            if(!atLeastOneValidFundamentalMatrix) {
                continue;
            }                        
            
            
            numValid++;
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }
    
    private static boolean validDecomposition(
            ProjectiveTransformation2D homography,
            HomographyDecomposition decomposition,
            PinholeCameraIntrinsicParameters intrinsic1,
            PinholeCameraIntrinsicParameters intrinsic2)
            throws WrongSizeException {
        //Homographies can be decomposed as H = d*R + t*n^T
        //H relates points in normalized coordinates and is related to G,
        //which relates non normalized points as follows:
        //H = K2^-1*G*K1 --> G = K2*H*K1^-1
        
        //normalize to increase accuracy
        homography.normalize();        
        Matrix g1 = homography.asMatrix();
        
        Matrix invK1 = intrinsic1.getInverseInternalMatrix();
        Matrix K2 = intrinsic2.getInternalMatrix();
        
        double d = decomposition.getPlaneDistance();
        Matrix R = decomposition.getTransformation().getRotation().
                asInhomogeneousMatrix();
        Matrix t = Matrix.newFromArray(decomposition.getTransformation().
                getTranslation(), true);
        Matrix transN = Matrix.newFromArray(decomposition.getPlaneNormal(), 
                false);
        
        Matrix tmp1 = R.multiplyByScalarAndReturnNew(d);
        Matrix tmp2 = t.multiplyAndReturnNew(transN);
        Matrix h2 = tmp1.addAndReturnNew(tmp2);
        
        Matrix g2 = K2.multiplyAndReturnNew(h2).multiplyAndReturnNew(invK1);
        
        
        return g1.equals(g2, HomographyDecomposerTest.ABSOLUTE_ERROR) ||
                g1.equals(g2.multiplyByScalarAndReturnNew(-1.0), HomographyDecomposerTest.ABSOLUTE_ERROR);
    }
    
    private static boolean areEqualUpToScale(
            FundamentalMatrix fundamentalMatrix1,
            FundamentalMatrix fundamentalMatrix2)
            throws NotAvailableException, NotReadyException {
        
        //normalize to increase accuracy
        fundamentalMatrix1.normalize();
        fundamentalMatrix2.normalize();
        
        Matrix f1 = fundamentalMatrix1.getInternalMatrix();
        Matrix f2a = fundamentalMatrix2.getInternalMatrix();
        Matrix f2b = f2a.multiplyByScalarAndReturnNew(-1.0);
        
        return f1.equals(f2a, HomographyDecomposerTest.ABSOLUTE_ERROR) || f1.equals(f2b,
                HomographyDecomposerTest.ABSOLUTE_ERROR);
    }
    
    private void reset() {
        decomposeStart = decomposeEnd = 0;
    }

    @Override
    public void onDecomposeStart(HomographyDecomposer decomposer) {
        decomposeStart++;
    }

    @Override
    public void onDecomposeEnd(HomographyDecomposer decomposer, 
            List<HomographyDecomposition> result) {
        decomposeEnd++;
    }
}
