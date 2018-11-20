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
package com.irurueta.ar.epipolar.refiners;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.ProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.refiners.Refiner;
import com.irurueta.geometry.refiners.RefinerListener;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.*;

import static org.junit.Assert.*;

public class HomogeneousRightEpipoleRefinerTest implements 
        RefinerListener<Point2D> {
    
    private static final double MIN_RANDOM_VALUE = -1500.0;
    private static final double MAX_RANDOM_VALUE = 1500.0;
    
    private static final double MIN_FOCAL_LENGTH = 750.0;
    private static final double MAX_FOCAL_LENGTH = 1500.0;
    
    private static final double MIN_ANGLE_DEGREES = -30.0;
    private static final double MAX_ANGLE_DEGREES = -15.0;
    
    private static final double MIN_CAMERA_SEPARATION = 500.0;
    private static final double MAX_CAMERA_SEPARATION = 1000.0;
    
    private static final int MIN_NUM_POINTS = 25;
    private static final int MAX_NUM_POINTS = 50;
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    
    private static final double ERROR_STD = 1e-3;
    
    private static final int MAX_TRIES = 5000;
    
    private static final int TIMES = 500;
    
    private int refineStart;
    private int refineEnd;
    
    public HomogeneousRightEpipoleRefinerTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructor() throws GeometryException, 
            AlgebraException, RobustEstimatorException {
        double refinementStandardDeviation = 1.0;
        PinholeCamera camera1 = new PinholeCamera();
        PinholeCamera camera2 = new PinholeCamera();
        FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
        List<Point2D> samples1 = new ArrayList<>();
        List<Point2D> samples2 = new ArrayList<>();
        Transformation2D homography = generateHomography(camera1, camera2, 
                fundamentalMatrix, samples1, samples2);
        
        final int numPoints = samples1.size();
        final BitSet inliers = new BitSet(numPoints);
        final double[] residuals = new double[numPoints];
        InliersData inliersData = new InliersData() {
            @Override
            public BitSet getInliers() {
                return inliers;
            }
            
            @Override
            public double[] getResiduals() {
                return residuals;
            }

            @Override
            public int getNumInliers() {
                return numPoints;
            }            
        };
        int numInliers = inliersData.getNumInliers();

        
        fundamentalMatrix.normalize();
        fundamentalMatrix.computeEpipoles();
        Point2D rightEpipole = fundamentalMatrix.getRightEpipole();
        
        HomogeneousRightEpipoleRefiner refiner = new HomogeneousRightEpipoleRefiner();
        
        //check default values
        assertEquals(refiner.getRefinementStandardDeviation(), 0.0, 0.0);
        assertNull(refiner.getHomography());
        assertNull(refiner.getSamples1());
        assertNull(refiner.getSamples2());
        assertFalse(refiner.isReady());
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(refiner.getNumInliers(), 0);
        assertEquals(refiner.getTotalSamples(), 0);
        assertNull(refiner.getInitialEstimation());
        assertFalse(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());
        
        //test non-empty constructor
        refiner = new HomogeneousRightEpipoleRefiner(rightEpipole, true, inliers, 
                residuals, numInliers, samples1, samples2, 
                refinementStandardDeviation, homography);
        
        //check default values
        assertEquals(refiner.getRefinementStandardDeviation(), 
                refinementStandardDeviation, 0.0);
        assertSame(refiner.getHomography(), homography);
        assertSame(refiner.getSamples1(), samples1);
        assertSame(refiner.getSamples2(), samples2);
        assertTrue(refiner.isReady());
        assertSame(refiner.getInliers(), inliers);
        assertSame(refiner.getResiduals(), residuals);
        assertEquals(refiner.getNumInliers(), numPoints);
        assertEquals(refiner.getTotalSamples(), numPoints);
        assertSame(refiner.getInitialEstimation(), rightEpipole);
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());
        
        //test non-empty constructor with InliersData
        refiner = new HomogeneousRightEpipoleRefiner(rightEpipole, true, inliersData, 
                samples1, samples2, refinementStandardDeviation, homography);
        
        //check default values
        assertEquals(refiner.getRefinementStandardDeviation(), 
                refinementStandardDeviation, 0.0);
        assertSame(refiner.getHomography(), homography);
        assertSame(refiner.getSamples1(), samples1);
        assertSame(refiner.getSamples2(), samples2);
        assertTrue(refiner.isReady());
        assertSame(refiner.getInliers(), inliers);
        assertSame(refiner.getResiduals(), residuals);
        assertEquals(refiner.getNumInliers(), numPoints);
        assertEquals(refiner.getTotalSamples(), numPoints);
        assertSame(refiner.getInitialEstimation(), rightEpipole);
        assertTrue(refiner.isCovarianceKept());
        assertFalse(refiner.isLocked());
        assertNull(refiner.getCovariance());
        assertNull(refiner.getListener());        
    }   
    
    @Test
    public void testGetSetListener() {
        HomogeneousRightEpipoleRefiner refiner = new HomogeneousRightEpipoleRefiner();
        
        //check default value
        assertNull(refiner.getListener());
        
        //set new value
        refiner.setListener(this);
        
        //check correctness
        assertSame(refiner.getListener(), this);
    }
    
    @Test
    public void testGetSetRefinementStandardDeviation() throws LockedException {
        HomogeneousRightEpipoleRefiner refiner = new HomogeneousRightEpipoleRefiner();
        
        //check default value
        assertEquals(refiner.getRefinementStandardDeviation(), 0.0, 0.0);
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double refinementStandardDeviation = randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        refiner.setRefinementStandardDeviation(refinementStandardDeviation);
        
        //check correctness
        assertEquals(refiner.getRefinementStandardDeviation(), 
                refinementStandardDeviation, 0.0);
    }
    
    @Test
    public void testGetSetHomography() throws LockedException {
        HomogeneousRightEpipoleRefiner refiner = new HomogeneousRightEpipoleRefiner();
        
        //check default value
        assertNull(refiner.getHomography());
        
        //set new value
        ProjectiveTransformation2D homography = 
                new ProjectiveTransformation2D();
        refiner.setHomography(homography);
        
        //check correctness
        assertSame(refiner.getHomography(), homography);
    }
    
    @Test
    public void testGetSetSamples1() throws LockedException {
        HomogeneousRightEpipoleRefiner refiner = new HomogeneousRightEpipoleRefiner();
        
        //check default value
        assertNull(refiner.getSamples1());
        
        //set new value
        List<Point2D> samples1 = new ArrayList<>();
        refiner.setSamples1(samples1);
        
        //check correctness
        assertSame(refiner.getSamples1(), samples1);
    }
    
    @Test
    public void testGetSetSamples2() throws LockedException {
        HomogeneousRightEpipoleRefiner refiner = new HomogeneousRightEpipoleRefiner();
        
        //check default value
        assertNull(refiner.getSamples2());
        
        //set new value
        List<Point2D> samples2 = new ArrayList<>();
        refiner.setSamples2(samples2);
        
        //check correctness
        assertSame(refiner.getSamples2(), samples2);
    }
    
    @Test
    public void testGetSetInliers() throws LockedException {
        HomogeneousRightEpipoleRefiner refiner = new HomogeneousRightEpipoleRefiner();
        
        //check default value
        assertNull(refiner.getInliers());
        
        //set new vlaue
        BitSet inliers = new BitSet(100);
        refiner.setInliers(inliers);
        
        //check correctness
        assertSame(refiner.getInliers(), inliers);
    }    
    
    @Test
    public void testGetSetResiduals() throws LockedException {
        HomogeneousRightEpipoleRefiner refiner = new HomogeneousRightEpipoleRefiner();
        
        //check default value
        assertNull(refiner.getResiduals());
        
        //set new value
        double[] residuals = new double[100];
        refiner.setResiduals(residuals);
        
        //check correctness
        assertSame(refiner.getResiduals(), residuals);
    }
        
    @Test
    public void testGetSetNumInliers() throws LockedException {
        HomogeneousRightEpipoleRefiner refiner = new HomogeneousRightEpipoleRefiner();
        
        //check default value
        assertEquals(refiner.getNumInliers(), 0);
        
        //set new value
        refiner.setNumInliers(10);
        
        //check correctness
        assertEquals(refiner.getNumInliers(), 10);
    }
    
    @Test
    public void testSetInliersData() throws LockedException {
        final int numPoints = MIN_NUM_POINTS;
        final BitSet inliers = new BitSet(numPoints);
        final double[] residuals = new double[numPoints];
        InliersData inliersData = new InliersData() {
            @Override
            public BitSet getInliers() {
                return inliers;
            }
            
            @Override
            public double[] getResiduals() {
                return residuals;
            }

            @Override
            public int getNumInliers() {
                return numPoints;
            }            
        };
        int numInliers = inliersData.getNumInliers();
        
        HomogeneousRightEpipoleRefiner refiner = new HomogeneousRightEpipoleRefiner();
        
        //check default values
        assertNull(refiner.getInliers());
        assertNull(refiner.getResiduals());
        assertEquals(refiner.getNumInliers(), 0);
        
        //set new value
        refiner.setInliersData(inliersData);
        
        //check correctness
        assertSame(refiner.getInliers(), inliers);
        assertSame(refiner.getResiduals(), residuals);
        assertEquals(refiner.getNumInliers(), numInliers);
    }
    
    @Test
    public void testGetSetInitialEstimation() throws LockedException {
        HomogeneousRightEpipoleRefiner refiner = new HomogeneousRightEpipoleRefiner();
        
        //check default value
        assertNull(refiner.getInitialEstimation());
        
        //set new value
        HomogeneousPoint2D initialEstimation = new HomogeneousPoint2D();
        refiner.setInitialEstimation(initialEstimation);
        
        //check correctness
        assertSame(refiner.getInitialEstimation(), initialEstimation);
    }
    
    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        HomogeneousRightEpipoleRefiner refiner = new HomogeneousRightEpipoleRefiner();
        
        //check default value
        assertFalse(refiner.isCovarianceKept());
        
        //set new value
        refiner.setCovarianceKept(true);
        
        //check correctness
        assertTrue(refiner.isCovarianceKept());
    }
    
    @Test
    public void testRefine() throws GeometryException,
            AlgebraException, RobustEstimatorException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            PinholeCamera camera1 = new PinholeCamera();
            PinholeCamera camera2 = new PinholeCamera();
            FundamentalMatrix groundTruthFundamentalMatrix = 
                    new FundamentalMatrix();
            List<Point2D> samples1 = new ArrayList<>();
            List<Point2D> samples2 = new ArrayList<>();
            Transformation2D homography = generateHomography(camera1, camera2, 
                    groundTruthFundamentalMatrix, samples1, samples2);
        
            final int numPoints = samples1.size();
            final BitSet inliers = new BitSet(numPoints);
            inliers.set(0, numPoints);
            final double[] residuals = new double[numPoints];
            Arrays.fill(residuals, 1.0);
        
            groundTruthFundamentalMatrix.normalize();
            groundTruthFundamentalMatrix.computeEpipoles();
            Point2D groundTruthRightEpipole = 
                    groundTruthFundamentalMatrix.getRightEpipole();
        
            //add noise to epipole
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            double noiseX = noiseRandomizer.nextDouble();
            double noiseY = noiseRandomizer.nextDouble();
            HomogeneousPoint2D noisyRightEpipole = new HomogeneousPoint2D(
                    groundTruthRightEpipole.getInhomX() + noiseX,
                    groundTruthRightEpipole.getInhomY() + noiseY, 1.0);
            noisyRightEpipole.normalize();
        
            HomogeneousRightEpipoleRefiner refiner = new HomogeneousRightEpipoleRefiner(
                    noisyRightEpipole, true, inliers, residuals, numPoints, 
                    samples1, samples2, ERROR_STD,
                    homography);
            refiner.setListener(this);
        
            reset();
            assertEquals(refineStart, 0);
            assertEquals(refineEnd, 0);
        
            HomogeneousPoint2D refinedEpipole1 = new HomogeneousPoint2D();
            refiner.refine(refinedEpipole1);
            Point2D refinedEpipole2 = refiner.refine();
                
            assertEquals(refineStart, 2);
            assertEquals(refineEnd, 2);
            
            refinedEpipole1.normalize();
            refinedEpipole2.normalize();            
        
            if (!refinedEpipole1.equals(groundTruthRightEpipole, 
                    ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(refinedEpipole1.equals(groundTruthRightEpipole, 
                    ABSOLUTE_ERROR));
            if (!refinedEpipole2.equals(groundTruthRightEpipole, 
                    ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(refinedEpipole2.equals(groundTruthRightEpipole, 
                    ABSOLUTE_ERROR));
        
            FundamentalMatrix refinedFundamentalMatrix1 = 
                    new FundamentalMatrix();
            HomogeneousRightEpipoleRefiner.computeFundamentalMatrix(homography, 
                    refinedEpipole1, refinedFundamentalMatrix1);
            refinedFundamentalMatrix1.normalize();
        
            FundamentalMatrix refinedFundamentalMatrix2 = 
                    new FundamentalMatrix();
            HomogeneousRightEpipoleRefiner.computeFundamentalMatrix(homography, 
                    new HomogeneousPoint2D(refinedEpipole2), 
                    refinedFundamentalMatrix2);
            refinedFundamentalMatrix2.normalize();
        
            //check correctness
            if (!groundTruthFundamentalMatrix.getInternalMatrix().equals(
                    refinedFundamentalMatrix1.getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !groundTruthFundamentalMatrix.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1.0).equals(
                    refinedFundamentalMatrix1.getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(groundTruthFundamentalMatrix.getInternalMatrix().equals(
                    refinedFundamentalMatrix1.getInternalMatrix(), ABSOLUTE_ERROR) ||
                    groundTruthFundamentalMatrix.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1.0).equals(
                    refinedFundamentalMatrix1.getInternalMatrix(), ABSOLUTE_ERROR));
            
            if (!groundTruthFundamentalMatrix.getInternalMatrix().equals(
                    refinedFundamentalMatrix2.getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !groundTruthFundamentalMatrix.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1.0).equals(
                    refinedFundamentalMatrix2.getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(groundTruthFundamentalMatrix.getInternalMatrix().equals(
                    refinedFundamentalMatrix2.getInternalMatrix(), ABSOLUTE_ERROR) ||
                    groundTruthFundamentalMatrix.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1.0).equals(
                    refinedFundamentalMatrix2.getInternalMatrix(), ABSOLUTE_ERROR));
            
            numValid++;
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }
    
    private Transformation2D generateHomography(PinholeCamera camera1, 
            PinholeCamera camera2, FundamentalMatrix fundamentalMatrix,
            List<Point2D> projectedPoints1, List<Point2D> projectedPoints2) 
            throws GeometryException, AlgebraException, 
            RobustEstimatorException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        double aspectRatio = 1.0;
        double skewness = 0.0;
        double principalPoint = 0.0;
        
        PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                principalPoint, principalPoint, skewness);
        intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);
        
        double alphaEuler1 = 0.0;
        double betaEuler1 = 0.0;
        double gammaEuler1 = 0.0;
        double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        double cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, 
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
        
        camera1.setIntrinsicAndExtrinsicParameters(intrinsic, rotation1, 
                center1);
        camera2.setIntrinsicAndExtrinsicParameters(intrinsic, rotation2, 
                center2);
        
        fundamentalMatrix.setFromPairOfCameras(camera1, camera2);
        
        //create 3D points laying in front of both cameras an in a plane
        Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
        Plane verticalPlane1 = camera1.getVerticalAxisPlane();
        Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
        Plane verticalPlane2 = camera2.getVerticalAxisPlane();
        Matrix planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS,
                Plane.PLANE_NUMBER_PARAMS);
        planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
        planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
        planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
        planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());
        
        planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
        planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
        planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
        planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());
        
        planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
        planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
        planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
        planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());
        
        planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
        planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
        planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
        planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());
        
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
        
        int numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
        
        HomogeneousPoint3D point3D;
        projectedPoints1.clear();
        projectedPoints2.clear();
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
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(planeA * homX + planeC * homZ + planeD * homW) / 
                            planeB;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(planeB * homY + planeC * homZ + planeD * homW) / 
                            planeA;
                }
                
                point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                assertTrue(plane.isLocus(point3D));
                
                front1 = camera1.isPointInFrontOfCamera(point3D);
                front2 = camera2.isPointInFrontOfCamera(point3D);
                if (numTry > MAX_TRIES) {
                    return null;
                }
                numTry++;
            } while(!front1 || !front2);
            
            //check that 3D point is in front of both cameras
            //noinspection all
            assertTrue(front1);
            //noinspection all
            assertTrue(front2);
            
            //project 3D point into both cameras
            projectedPoints1.add(camera1.project(point3D));
            projectedPoints2.add(camera2.project(point3D));
        }
        
        //estimate homography
        ProjectiveTransformation2DRobustEstimator homographyEstimator =
                ProjectiveTransformation2DRobustEstimator.createFromPoints(
                projectedPoints1, projectedPoints2, 
                RobustEstimatorMethod.LMedS);
        
        return homographyEstimator.estimate();
    }

    @Override
    public void onRefineStart(Refiner<Point2D> refiner, 
            Point2D initialEstimation) {
        refineStart++;
        checkLocked((HomogeneousRightEpipoleRefiner)refiner);
    }

    @Override
    public void onRefineEnd(Refiner<Point2D> refiner, Point2D initialEstimation,
            Point2D result, boolean errorDecreased) {
        refineEnd++;
        checkLocked((HomogeneousRightEpipoleRefiner)refiner);
    }

    private void reset() {
        refineStart = refineEnd = 0;
    }

    private void checkLocked(HomogeneousRightEpipoleRefiner refiner) {
        assertTrue(refiner.isLocked());
        try {
            refiner.setRefinementStandardDeviation(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            refiner.setHomography(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            refiner.refine();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception ignore) {
            fail("LockedException expected but not thrown");
        }
        try {
            refiner.refine(new HomogeneousPoint2D());
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception ignore) {
            fail("LockedException expected but not thrown");
        }
    }
}
