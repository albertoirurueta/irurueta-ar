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
package com.irurueta.ar.sfm;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.ar.epipolar.Corrector;
import com.irurueta.ar.epipolar.CorrectorType;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidPairOfCamerasException;
import com.irurueta.ar.epipolar.estimators.FundamentalMatrixEstimatorException;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.PointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PlanarBestFundamentalMatrixEstimatorAndReconstructorTest implements
        PlanarBestFundamentalMatrixEstimatorAndReconstructorListener {
    
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
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;
    
    private int mEstimateStart;
    private int mEstimateEnd;
    
    public PlanarBestFundamentalMatrixEstimatorAndReconstructorTest() { }
    
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
        PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();
        
        //check default values
        assertNull(er.getLeftPoints());
        assertNull(er.getRightPoints());
        assertNull(er.getLeftIntrinsics());
        assertNull(er.getRightIntrinsics());
        assertNotNull(er.getHomographyEstimator());
        assertEquals(er.getEssentialCameraEstimatorCorrectorType(), 
                Corrector.DEFAULT_TYPE);
        assertEquals(er.getHomographyConfidence(),
                er.getHomographyEstimator().getConfidence(), 0.0);
        assertEquals(er.getHomographyMaxIterations(),
                er.getHomographyEstimator().getMaxIterations());
        assertEquals(er.isHomographyRefined(),
                er.getHomographyEstimator().isResultRefined());
        assertEquals(er.isHomographyCovarianceKept(),
                er.getHomographyEstimator().isCovarianceKept());
        assertNull(er.getHomographyCovariance());
        assertEquals(er.getHomographyMethod(),
                er.getHomographyEstimator().getMethod());
        assertNull(er.getQualityScores());
        assertNull(er.getListener());
        assertFalse(er.isLocked());
        assertFalse(er.isReady());
        assertNull(er.getFundamentalMatrix());
        assertNull(er.getTriangulatedPoints());
        assertNull(er.getValidTriangulatedPoints());
        assertNull(er.getEstimatedLeftCamera());
        assertNull(er.getEstimatedRightCamera());
        
        //constructor with points and intrinsics
        List<Point2D> leftPoints = new ArrayList<>();
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());
        
        List<Point2D> rightPoints = new ArrayList<>();
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());
        
        PinholeCameraIntrinsicParameters leftIntrinsics = 
                new PinholeCameraIntrinsicParameters();
        PinholeCameraIntrinsicParameters rightIntrinsics =
                new PinholeCameraIntrinsicParameters();
        
        er = new PlanarBestFundamentalMatrixEstimatorAndReconstructor(
                leftPoints, rightPoints, leftIntrinsics, rightIntrinsics);
        
        //check correctness
        assertSame(er.getLeftPoints(), leftPoints);
        assertSame(er.getRightPoints(), rightPoints);
        assertSame(er.getLeftIntrinsics(), leftIntrinsics);
        assertSame(er.getRightIntrinsics(), rightIntrinsics);
        assertNotNull(er.getHomographyEstimator());
        assertEquals(er.getEssentialCameraEstimatorCorrectorType(), 
                Corrector.DEFAULT_TYPE);
        assertEquals(er.getHomographyConfidence(),
                er.getHomographyEstimator().getConfidence(), 0.0);
        assertEquals(er.getHomographyMaxIterations(),
                er.getHomographyEstimator().getMaxIterations());
        assertEquals(er.isHomographyRefined(),
                er.getHomographyEstimator().isResultRefined());
        assertEquals(er.isHomographyCovarianceKept(),
                er.getHomographyEstimator().isCovarianceKept());
        assertNull(er.getHomographyCovariance());
        assertEquals(er.getHomographyMethod(),
                er.getHomographyEstimator().getMethod());
        assertNotNull(er.getQualityScores());
        assertNull(er.getListener());
        assertFalse(er.isLocked());
        assertTrue(er.isReady());
        assertNull(er.getFundamentalMatrix());
        assertNull(er.getTriangulatedPoints());
        assertNull(er.getValidTriangulatedPoints());
        assertNull(er.getEstimatedLeftCamera());
        assertNull(er.getEstimatedRightCamera());     
        
        //Force IllegalaRgumentException
        List<Point2D> wrong = new ArrayList<>();
        
        er = null;
        try {
            er = new PlanarBestFundamentalMatrixEstimatorAndReconstructor(wrong, 
                    rightPoints, leftIntrinsics, rightIntrinsics);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            er = new PlanarBestFundamentalMatrixEstimatorAndReconstructor(
                    leftPoints, wrong, leftIntrinsics, rightIntrinsics);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            er = new PlanarBestFundamentalMatrixEstimatorAndReconstructor(wrong,
                    wrong, leftIntrinsics, rightIntrinsics);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(er);
        
        
        //constructor with points, intrinsics and listener
        er = new PlanarBestFundamentalMatrixEstimatorAndReconstructor(
                leftPoints, rightPoints, leftIntrinsics, rightIntrinsics, this);
        
        //check correctness
        assertSame(er.getLeftPoints(), leftPoints);
        assertSame(er.getRightPoints(), rightPoints);
        assertSame(er.getLeftIntrinsics(), leftIntrinsics);
        assertSame(er.getRightIntrinsics(), rightIntrinsics);
        assertNotNull(er.getHomographyEstimator());
        assertEquals(er.getEssentialCameraEstimatorCorrectorType(), 
                Corrector.DEFAULT_TYPE);
        assertEquals(er.getHomographyConfidence(),
                er.getHomographyEstimator().getConfidence(), 0.0);
        assertEquals(er.getHomographyMaxIterations(),
                er.getHomographyEstimator().getMaxIterations());
        assertEquals(er.isHomographyRefined(),
                er.getHomographyEstimator().isResultRefined());
        assertEquals(er.isHomographyCovarianceKept(),
                er.getHomographyEstimator().isCovarianceKept());
        assertNull(er.getHomographyCovariance());
        assertEquals(er.getHomographyMethod(),
                er.getHomographyEstimator().getMethod());
        assertNotNull(er.getQualityScores());
        assertSame(er.getListener(), this);
        assertFalse(er.isLocked());
        assertTrue(er.isReady());
        assertNull(er.getFundamentalMatrix());
        assertNull(er.getTriangulatedPoints());
        assertNull(er.getValidTriangulatedPoints());
        assertNull(er.getEstimatedLeftCamera());
        assertNull(er.getEstimatedRightCamera());     
        
        //Force IllegalaRgumentException
        er = null;
        try {
            er = new PlanarBestFundamentalMatrixEstimatorAndReconstructor(wrong, 
                    rightPoints, leftIntrinsics, rightIntrinsics, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            er = new PlanarBestFundamentalMatrixEstimatorAndReconstructor(
                    leftPoints, wrong, leftIntrinsics, rightIntrinsics, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            er = new PlanarBestFundamentalMatrixEstimatorAndReconstructor(wrong,
                    wrong, leftIntrinsics, rightIntrinsics, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(er);        
    }
    
    @Test
    public void testGetSetLeftPoints() throws LockedException {
        PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        //check default value
        assertNull(er.getLeftPoints());
        
        //set new value
        List<Point2D> leftPoints = new ArrayList<>();
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());
        
        er.setLeftPoints(leftPoints);
        
        //check correctness
        assertSame(er.getLeftPoints(), leftPoints);
        
        //Force IllegalArgumentException
        List<Point2D> wrong = new ArrayList<>();
        try {
            er.setLeftPoints(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetRightPoints() throws LockedException {
        PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();
        
        //check default value
        assertNull(er.getRightPoints());
        
        //set new value
        List<Point2D> rightPoints = new ArrayList<>();
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());
        
        er.setRightPoints(rightPoints);
        
        //check correctness
        assertSame(er.getRightPoints(), rightPoints);
        
        //Force IllegalArgumentException
        List<Point2D> wrong = new ArrayList<>();
        try {
            er.setRightPoints(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testSetLeftAndRightPoints() throws LockedException {
        PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        //check default values
        assertNull(er.getLeftPoints());
        assertNull(er.getRightPoints());
        
        //set new value
        List<Point2D> leftPoints = new ArrayList<>();
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());
        
        List<Point2D> rightPoints = new ArrayList<>();
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());
        
        er.setLeftAndRightPoints(leftPoints, rightPoints);

        //check correctness
        assertSame(er.getLeftPoints(), leftPoints);
        assertSame(er.getRightPoints(), rightPoints);
        assertSame(er.getHomographyEstimator().getInputPoints(), leftPoints);
        assertSame(er.getHomographyEstimator().getOutputPoints(), rightPoints);
        
        //Force IllegalArgumentException
        List<Point2D> wrong = new ArrayList<>();
        try {
            er.setLeftAndRightPoints(wrong, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            er.setLeftAndRightPoints(leftPoints, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            er.setLeftAndRightPoints(wrong, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        rightPoints.add(Point2D.create());
        try {
            er.setLeftAndRightPoints(leftPoints, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetLeftIntrinsics() throws LockedException {
        PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        //initial value
        assertNull(er.getLeftIntrinsics());
        
        //set new value
        PinholeCameraIntrinsicParameters leftIntrinsics = 
                new PinholeCameraIntrinsicParameters();
        er.setLeftIntrinsics(leftIntrinsics);
        
        //check correctness
        assertSame(er.getLeftIntrinsics(), leftIntrinsics);
    }
    
    @Test
    public void testGetSetRightIntrinsics() throws LockedException {
        PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        //initial value
        assertNull(er.getRightIntrinsics());
        
        //set new value
        PinholeCameraIntrinsicParameters rightIntrinsics =
                new PinholeCameraIntrinsicParameters();
        er.setRightIntrinsics(rightIntrinsics);
        
        //check correctness
        assertSame(er.getRightIntrinsics(), rightIntrinsics);
    }
    
    @Test
    public void testGetsetHomographyEstimator() throws LockedException {
        PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        //initial value
        assertNotNull(er.getHomographyEstimator());
        
        //set new value
        PointCorrespondenceProjectiveTransformation2DRobustEstimator 
                homographyEstimator = 
                PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                        create();
        er.setHomographyEstimator(homographyEstimator);
        
        //check correctness
        assertSame(er.getHomographyEstimator(), homographyEstimator);
        
        //Force NullPointerException
        try {
            er.setHomographyEstimator(null);
            fail("NullPointerException expected but not thrown");
        } catch (NullPointerException ignore) { }
    }
    
    @Test
    public void testGetSetEssentialCameraEstimatorCorrectorType() 
            throws LockedException {
        PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        //check initial value
        assertEquals(er.getEssentialCameraEstimatorCorrectorType(), 
                Corrector.DEFAULT_TYPE);
        
        //set new value
        er.setEssentialCameraEstimatorCorrectorType(
                CorrectorType.GOLD_STANDARD);
        
        //check correctness
        assertEquals(er.getEssentialCameraEstimatorCorrectorType(),
                CorrectorType.GOLD_STANDARD);
    }
    
    @Test
    public void testGetSetHomographyConfidence() throws LockedException {
        PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();
        
        //check default value
        assertEquals(er.getHomographyConfidence(),
                er.getHomographyEstimator().getConfidence(), 0.0);
        
        //set new value
        er.setHomographyConfidence(0.5);
        
        //check correctness
        assertEquals(er.getHomographyConfidence(), 0.5, 0.0);
        assertEquals(er.getHomographyEstimator().getConfidence(), 0.5, 0.0);
    }
    
    @Test
    public void testGetSetHomographyMaxIterations() throws LockedException {
        PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();
        
        //check default value
        assertEquals(er.getHomographyMaxIterations(),
                er.getHomographyEstimator().getMaxIterations());
        
        //set new value
        er.setHomographyMaxIterations(10);
        
        //check correctness
        assertEquals(er.getHomographyMaxIterations(), 10);
        assertEquals(er.getHomographyEstimator().getMaxIterations(), 10);
    }
    
    @Test
    public void testIsSetHomographyRefined() throws LockedException {
        PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();
        
        //check default value
        boolean refined = er.isHomographyRefined();
        assertEquals(er.isHomographyRefined(), 
                er.getHomographyEstimator().isResultRefined());
        
        //set new value
        er.setHomographyRefined(!refined);
        
        //check correctness
        assertEquals(er.isHomographyRefined(), !refined);
        assertEquals(er.getHomographyEstimator().isResultRefined(), !refined);
    }
    
    @Test
    public void testIsSetHomographyCovarianceKept() throws LockedException {
        PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();
        
        //check default value
        boolean covarianceKept = er.isHomographyCovarianceKept();
        assertEquals(er.isHomographyCovarianceKept(),
                er.getHomographyEstimator().isCovarianceKept());
        
        //set new value
        er.setHomographyCovarianceKept(!covarianceKept);
        
        //check correctness
        assertEquals(er.isHomographyCovarianceKept(), !covarianceKept);
        assertEquals(er.getHomographyEstimator().isCovarianceKept(), 
                !covarianceKept);
    }
    
    @Test
    public void testGetSetQualityScores() throws LockedException {
        PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();
        
        //check default value
        assertNull(er.getQualityScores());
        
        //set new estimator
        PointCorrespondenceProjectiveTransformation2DRobustEstimator 
                homographyEstimator = 
                PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                        create(RobustEstimatorMethod.PROSAC);
        er.setHomographyEstimator(homographyEstimator);
        assertNull(er.getQualityScores());
        
        //set new value
        double[] qualityScores = new double[
                PlanarBestFundamentalMatrixEstimatorAndReconstructor.
                MINIMUM_SIZE];
        er.setQualityScores(qualityScores);
        
        //check correctness
        assertSame(er.getQualityScores(), qualityScores);
        
        //Force IllegalArgumentException
        double[] wrong = new double[1];
        try {
            er.setQualityScores(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetListener() {
        PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();
        
        //check default value
        assertNull(er.getListener());
        
        //set new value
        er.setListener(this);
        
        //check correctness
        assertSame(er.getListener(), this);
    }
    
    @Test
    public void testEstimateAndReconstruct() 
            throws InvalidPairOfCamerasException, AlgebraException, 
            CameraException, LockedException, NotReadyException, 
            NotAvailableException {
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
            
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);
            
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
                } while (!front1 || !front2);
                
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
            
            PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                    new PlanarBestFundamentalMatrixEstimatorAndReconstructor(
                            projectedPoints1, projectedPoints2, intrinsic1, 
                            intrinsic2, this);
            
            reset();
            assertEquals(mEstimateStart, 0);
            assertEquals(mEstimateEnd, 0);
            assertFalse(er.isLocked());
            assertTrue(er.isReady());
            
            try {
                er.estimateAndReconstruct();                            
            } catch (FundamentalMatrixEstimatorException e) {
                continue;
            }
            
            //check correctness
            assertEquals(mEstimateStart, 1);
            assertEquals(mEstimateEnd, 1);
            assertFalse(er.isLocked());
            
            //check correctness of homography
            boolean failed = false;
            InhomogeneousPoint2D point1, point2, point2b;
            for (int i = 0; i < numPoints; i++) {
                point1 = new InhomogeneousPoint2D(projectedPoints1.get(i));
                point2 = new InhomogeneousPoint2D(projectedPoints2.get(i));
                
                point2b = new InhomogeneousPoint2D();
                er.getHomography().transform(point1, point2b);
                
                if (!point2.equals(point2b, LARGE_ABSOLUTE_ERROR)) {
                    failed = true;
                    break;
                }
                assertTrue(point2.equals(point2b, LARGE_ABSOLUTE_ERROR));
            }
            
            if (failed) {
                continue;
            }
            
            //check that estimated fundamental matrix is equal up to scale to 
            //the original one
            if (!areEqualUpToScale(fundamentalMatrix, er.getFundamentalMatrix()
            )) {
                continue;
            }
            assertTrue(areEqualUpToScale(fundamentalMatrix, 
                    er.getFundamentalMatrix()));
            
            //check that triangulated points lie in front of both estimated 
            //cameras
            //NOTE: points and cameras are reconstructed up to scale respect to
            //original ones
            PinholeCamera camera1b = er.getEstimatedLeftCamera();
            PinholeCamera camera2b = er.getEstimatedRightCamera();
            
            List<Point3D> triangulatedPoints = er.getTriangulatedPoints();
            BitSet validTriangulatedPoints = er.getValidTriangulatedPoints();
            for (int i = 0; i < validTriangulatedPoints.length(); i++) {
                Point3D point = triangulatedPoints.get(i);
                
                assertTrue(validTriangulatedPoints.get(i));
                
                assertTrue(camera1b.isPointInFrontOfCamera(point));
                assertTrue(camera2b.isPointInFrontOfCamera(point));
            }
                        
            numValid++;
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(
            PlanarBestFundamentalMatrixEstimatorAndReconstructor 
                    estimatorAndReconstructor) {
        mEstimateStart++;
        checkLocked(estimatorAndReconstructor);
    }

    @Override
    public void onEstimateEnd(
            PlanarBestFundamentalMatrixEstimatorAndReconstructor 
                    estimatorAndReconstructor) {
        mEstimateEnd++;
        checkLocked(estimatorAndReconstructor);
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

        return f1.equals(f2a, ABSOLUTE_ERROR) || f1.equals(f2b, ABSOLUTE_ERROR);
    }

    private void reset() {
        mEstimateStart = mEstimateEnd = 0;
    }

    private void checkLocked(
            PlanarBestFundamentalMatrixEstimatorAndReconstructor 
                    estimatorAndReconstructor) {
        
        try {
            estimatorAndReconstructor.setLeftPoints(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimatorAndReconstructor.setRightPoints(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimatorAndReconstructor.setLeftAndRightPoints(null, null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimatorAndReconstructor.setLeftIntrinsics(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimatorAndReconstructor.setRightIntrinsics(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimatorAndReconstructor.setHomographyEstimator(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimatorAndReconstructor.setEssentialCameraEstimatorCorrectorType(
                    null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimatorAndReconstructor.setHomographyConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimatorAndReconstructor.setHomographyMaxIterations(10);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimatorAndReconstructor.setHomographyRefined(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimatorAndReconstructor.setHomographyCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimatorAndReconstructor.setQualityScores(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        assertTrue(estimatorAndReconstructor.isLocked());
    }
}
