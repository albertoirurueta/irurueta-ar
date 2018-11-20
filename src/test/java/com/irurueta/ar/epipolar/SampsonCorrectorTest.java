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
package com.irurueta.ar.epipolar;

import com.irurueta.algebra.*;
import com.irurueta.algebra.NotAvailableException;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class SampsonCorrectorTest implements CorrectorListener {
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    
    private static final double MIN_PROJECTED_ERROR = -1.0;
    private static final double MAX_PROJECTED_ERROR = 1.0;
    
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;
    private static final double MIN_SKEWNESS = -1.0;
    private static final double MAX_SKEWNESS = 1.0;
    private static final double MIN_PRINCIPAL_POINT = 0.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;
    private static final double MIN_ANGLE_DEGREES = -30.0;
    private static final double MAX_ANGLE_DEGREES = 30.0;
    private static final double MIN_CAMERA_SEPARATION = 5.0;
    private static final double MAX_CAMERA_SEPARATION = 10.0;
    
    private static final int MIN_POINTS = 100;
    private static final int MAX_POINTS = 500;
    
    private static final int TIMES = 100;
    
    private int correctStart;
    private int correctEnd;
    private int correctProgressChange;    
    
    public SampsonCorrectorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructor() throws WrongSizeException, NotReadyException, 
            LockedException, DecomposerException, NotAvailableException, 
            InvalidFundamentalMatrixException, 
            com.irurueta.algebra.LockedException {
        //test constructor without arguments
        SampsonCorrector corrector = new SampsonCorrector();
        
        //check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(), 
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        
        //test constructor with fundamental matrix
        FundamentalMatrix emptyFundamentalMatrix = new FundamentalMatrix();
        corrector = new SampsonCorrector(emptyFundamentalMatrix);
        
        //check correctness
        assertSame(corrector.getFundamentalMatrix(), emptyFundamentalMatrix);
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        
        //test constructor with left and right points
        List<Point2D> leftPoints = new ArrayList<>();
        List<Point2D> rightPoints = new ArrayList<>();
        
        corrector = new SampsonCorrector(leftPoints, rightPoints);
        
        //check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertSame(corrector.getLeftPoints(), leftPoints);
        assertSame(corrector.getRightPoints(), rightPoints);
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        
        //Force IllegalArgumentException
        List<Point2D> badPoints = new ArrayList<>();
        badPoints.add(Point2D.create());
        
        corrector = null;
        try {
            corrector = new SampsonCorrector(badPoints, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            corrector = new SampsonCorrector(leftPoints, badPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(corrector);
        
        //test constructor with left and right points and fundamental matrix
        corrector = new SampsonCorrector(leftPoints, rightPoints, 
                emptyFundamentalMatrix);
        
        //check correctness
        assertSame(corrector.getFundamentalMatrix(), emptyFundamentalMatrix);
        assertSame(corrector.getLeftPoints(), leftPoints);
        assertSame(corrector.getRightPoints(), rightPoints);
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady()); //fundamental matrix not defined
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        
        //test constructor with left and right points and a valid fundamental
        //matrix
        FundamentalMatrix fundamentalMatrix;
        int rank;
        do {
            Matrix internalMatrix = Matrix.createWithUniformRandomValues(
                    FundamentalMatrix.FUNDAMENTAL_MATRIX_ROWS, 
                    FundamentalMatrix.FUNDAMENTAL_MATRIX_COLS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            
            //ensure that internal matrix has rank 2
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    internalMatrix);
            decomposer.decompose();
            
            rank = decomposer.getRank(); //if rank is less than 2 we need to
                                        //pick another random matrix
            
            Matrix U = decomposer.getU();
            Matrix W = decomposer.getW();
            Matrix V = decomposer.getV();
            Matrix transV = V.transposeAndReturnNew();
            
            //set last element to 0 to force rank 2
            W.setElementAt(2, 2, 0.0);
            
            internalMatrix = U.multiplyAndReturnNew(W.multiplyAndReturnNew(
                    transV));
            
            fundamentalMatrix = new FundamentalMatrix(internalMatrix);
        } while (rank < 2);
        
        corrector = new SampsonCorrector(leftPoints, rightPoints, 
                fundamentalMatrix);
        
        //check correctness
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
        assertSame(corrector.getLeftPoints(), leftPoints);
        assertSame(corrector.getRightPoints(), rightPoints);
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertNull(corrector.getListener());
        assertTrue(corrector.isReady()); //fundamental matrix not defined
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);        
        
        corrector = null;
        try {
            corrector = new SampsonCorrector(badPoints, rightPoints, 
                    fundamentalMatrix);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            corrector = new SampsonCorrector(leftPoints, badPoints, 
                    fundamentalMatrix);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(corrector);
        
        //test constructor with listener
        corrector = new SampsonCorrector(this);
        
        //check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertSame(corrector.getListener(), this);
        assertFalse(corrector.isReady());
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        
        //test constructor with fundamental matrix and listener
        corrector = new SampsonCorrector(fundamentalMatrix, this);
        
        //check correctness
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertSame(corrector.getListener(), this);
        assertFalse(corrector.isReady());
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        
        //test constructor with left and right points and listener
        corrector = new SampsonCorrector(leftPoints, rightPoints, this);
        
        //check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertSame(corrector.getLeftPoints(), leftPoints);
        assertSame(corrector.getRightPoints(), rightPoints);
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertSame(corrector.getListener(), this);
        assertFalse(corrector.isReady());
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        
        corrector = null;
        try {
            corrector = new SampsonCorrector(badPoints, rightPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            corrector = new SampsonCorrector(leftPoints, badPoints, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(corrector);
        
        //test constructor with left and right points, fundamental matrix
        //and listener
        corrector = new SampsonCorrector(leftPoints, rightPoints, 
                emptyFundamentalMatrix, this);
        
        //check correctness
        assertSame(corrector.getFundamentalMatrix(), emptyFundamentalMatrix);
        assertSame(corrector.getLeftPoints(), leftPoints);
        assertSame(corrector.getRightPoints(), rightPoints);
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertSame(corrector.getListener(), this);
        assertFalse(corrector.isReady()); //fundamental matrix not defined
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        
        //test constructor with left and right points, valid fundamental matrix
        //and listener
        corrector = new SampsonCorrector(leftPoints, rightPoints, 
                fundamentalMatrix, this);
        
        //check correctness
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
        assertSame(corrector.getLeftPoints(), leftPoints);
        assertSame(corrector.getRightPoints(), rightPoints);
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(corrector.getProgressDelta(),
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        assertSame(corrector.getListener(), this);
        assertTrue(corrector.isReady());
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        
        corrector = null;
        try {
            corrector = new SampsonCorrector(badPoints, rightPoints, 
                    fundamentalMatrix, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            corrector = new SampsonCorrector(leftPoints, badPoints, 
                    fundamentalMatrix, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(corrector);        
    }
    
    @Test
    public void testGetSetFundamentalMatrix() throws LockedException {
        SampsonCorrector corrector = new SampsonCorrector();
        
        //check default value
        assertNull(corrector.getFundamentalMatrix());
        
        //set new value
        FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
        corrector.setFundamentalMatrix(fundamentalMatrix);
        
        //check correctness
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
    }
    
    @Test
    public void testGetSetLeftAndRightPoints() throws LockedException {
        SampsonCorrector corrector = new SampsonCorrector();
        
        //check default values
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        
        //set new values
        List<Point2D> leftPoints = new ArrayList<>();
        List<Point2D> rightPoints = new ArrayList<>();
        
        corrector.setLeftAndRightPoints(leftPoints, rightPoints);
        
        //check correctness
        assertSame(corrector.getLeftPoints(), leftPoints);
        assertSame(corrector.getRightPoints(), rightPoints);
        
        //Force IllegalArgumentException
        List<Point2D> badPoints = new ArrayList<>();
        badPoints.add(Point2D.create());
        
        try {
            corrector.setLeftAndRightPoints(leftPoints, badPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            corrector.setLeftAndRightPoints(badPoints, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetPointsAndFundamentalMatrix() throws LockedException {
        SampsonCorrector corrector = new SampsonCorrector();
        
        //check default values
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getFundamentalMatrix());
        
        //set new values
        List<Point2D> leftPoints = new ArrayList<>();
        List<Point2D> rightPoints = new ArrayList<>();
        FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
        
        corrector.setPointsAndFundamentalMatrix(leftPoints, rightPoints, 
                fundamentalMatrix);
        
        //check correctness
        assertSame(corrector.getLeftPoints(), leftPoints);
        assertSame(corrector.getRightPoints(), rightPoints);
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
        
        //Force IllegalArgumentException
        List<Point2D> badPoints = new ArrayList<>();
        badPoints.add(Point2D.create());
        
        try {
            corrector.setPointsAndFundamentalMatrix(badPoints, rightPoints, 
                    fundamentalMatrix);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            corrector.setPointsAndFundamentalMatrix(leftPoints, badPoints, 
                    fundamentalMatrix);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetProgressDelta() throws LockedException {
        SampsonCorrector corrector = new SampsonCorrector();
        
        //check default values
        assertEquals(corrector.getProgressDelta(), 
                Corrector.DEFAULT_PROGRESS_DELTA, 0.0);
        
        //set new value
        corrector.setProgressDelta(0.5f);
        
        //check correctness
        assertEquals(corrector.getProgressDelta(), 0.5, 0.0);
    }
    
    @Test
    public void testGetSetListener() throws LockedException {
        SampsonCorrector corrector = new SampsonCorrector();
        
        //check default values
        assertNull(corrector.getListener());
        
        //set new value
        corrector.setListener(this);
        
        //check correctness
        assertSame(corrector.getListener(), this);
    }
    
    @Test
    public void testAreValidPoints() {
        List<Point2D> leftPoints = new ArrayList<>();
        List<Point2D> rightPoints = new ArrayList<>();
        
        assertFalse(SampsonCorrector.areValidPoints(null, null));
        assertFalse(SampsonCorrector.areValidPoints(leftPoints, null));
        assertFalse(SampsonCorrector.areValidPoints(null, rightPoints));
        assertTrue(SampsonCorrector.areValidPoints(leftPoints, rightPoints));
    }
    
    @Test
    public void testCorrect() throws InvalidPairOfCamerasException,
            com.irurueta.geometry.estimators.NotReadyException, LockedException {
        
        int improved = 0, total = 0;
        for (int t = 0; t < TIMES; t++) {
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            
            double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            
            double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            
            double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
            
            PinholeCameraIntrinsicParameters intrinsic1 = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1, 
                    verticalFocalLength1, horizontalPrincipalPoint1, 
                    verticalPrincipalPoint1, skewness1);
            PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                    verticalFocalLength2, horizontalPrincipalPoint2,
                    verticalPrincipalPoint2, skewness2);
            
            //camera centers
            Point3D cameraCenter1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            Point3D cameraCenter2 = new InhomogeneousPoint3D(
                    cameraCenter1.getInhomX() + cameraSeparation,
                    cameraCenter1.getInhomY() + cameraSeparation,
                    cameraCenter1.getInhomZ() + cameraSeparation);
            
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, 
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            
            //create random list of 3D points to project
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            List<Point3D> pointsToProject = new ArrayList<>(nPoints);
            for (int i = 0; i < nPoints; i++) {
                pointsToProject.add(new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }
            total += nPoints;
            
            //create two cameras
            PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, 
                    cameraCenter1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    cameraCenter2);
            
            //project 3D points with both cameras
            List<Point2D> leftPoints = camera1.project(pointsToProject);
            List<Point2D> rightPoints = camera2.project(pointsToProject);
            
            //add error to projected points
            List<Point2D> wrongLeftPoints = new ArrayList<>(nPoints);
            List<Point2D> wrongRightPoints = new ArrayList<>(nPoints);
            for (int i = 0; i < nPoints; i++) {
                double errorLeftX = randomizer.nextDouble(MIN_PROJECTED_ERROR, 
                        MAX_PROJECTED_ERROR);
                double errorLeftY = randomizer.nextDouble(MIN_PROJECTED_ERROR, 
                        MAX_PROJECTED_ERROR);
                double errorRightX = randomizer.nextDouble(MIN_PROJECTED_ERROR, 
                        MAX_PROJECTED_ERROR);
                double errorRightY = randomizer.nextDouble(MIN_PROJECTED_ERROR,
                        MAX_PROJECTED_ERROR);
                
                Point2D leftPoint = leftPoints.get(i);
                Point2D rightPoint = rightPoints.get(i);
                
                Point2D wrongLeftPoint = new HomogeneousPoint2D(
                        leftPoint.getInhomX() + errorLeftX, 
                        leftPoint.getInhomY() + errorLeftY, 1.0);
                Point2D wrongRightPoint = new HomogeneousPoint2D(
                        rightPoint.getInhomX() + errorRightX, 
                        rightPoint.getInhomY() + errorRightY, 1.0);
                
                wrongLeftPoints.add(wrongLeftPoint);
                wrongRightPoints.add(wrongRightPoint);
            }
            
            //create fundamental matrix for the same pair of cameras used to
            //project points
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, 
                    camera2);            
            
            //check that points without error belong to epipolar lines
            boolean validPoints = true;
            for (int i = 0; i < nPoints; i++) {
                Point2D leftPoint = leftPoints.get(i);
                Point2D rightPoint = rightPoints.get(i);
                
                Line2D rightEpipolarLine = 
                        fundamentalMatrix.getRightEpipolarLine(leftPoint);
                Line2D leftEpipolarLine =
                        fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                
                if (!rightEpipolarLine.isLocus(rightPoint, ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(rightEpipolarLine.isLocus(rightPoint, 
                        ABSOLUTE_ERROR));
                if (!leftEpipolarLine.isLocus(leftPoint, ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(leftEpipolarLine.isLocus(leftPoint, ABSOLUTE_ERROR));
            }     
            
            if(!validPoints) {
                continue;
            }
            
            //use corrector to fix points with error
            SampsonCorrector corrector = new SampsonCorrector(wrongLeftPoints,
                    wrongRightPoints, fundamentalMatrix, this);
            
            assertTrue(corrector.isReady());
            assertFalse(corrector.isLocked());
            assertEquals(correctStart, 0);
            assertEquals(correctEnd, 0);
            assertEquals(correctProgressChange, 0);
            
            corrector.correct();
            
            assertTrue(corrector.isReady());
            assertFalse(corrector.isLocked());
            assertEquals(correctStart, 1);
            assertEquals(correctEnd, 1);
            assertTrue(correctProgressChange > 0);
            reset();
            
            List<Point2D> correctedLeftPoints = 
                    corrector.getLeftCorrectedPoints();
            List<Point2D> correctedRightPoints =
                    corrector.getRightCorrectedPoints();
            
            //check correctness
            for (int i = 0; i < nPoints; i++) {
                Point2D correctedLeftPoint = correctedLeftPoints.get(i);
                Point2D correctedRightPoint = correctedRightPoints.get(i);
                
                Point2D wrongLeftPoint = wrongLeftPoints.get(i);
                Point2D wrongRightPoint = wrongRightPoints.get(i);
                
                Line2D rightEpipolarLine = 
                        fundamentalMatrix.getRightEpipolarLine(
                        correctedLeftPoint);
                Line2D leftEpipolarLine = fundamentalMatrix.getLeftEpipolarLine(
                        correctedRightPoint);
                
                double correctedDistanceLeft = leftEpipolarLine.signedDistance(
                        correctedLeftPoint);
                double correctedDistanceRight = 
                        rightEpipolarLine.signedDistance(correctedRightPoint);
                
                rightEpipolarLine = fundamentalMatrix.getRightEpipolarLine(
                        wrongLeftPoint);
                leftEpipolarLine = fundamentalMatrix.getLeftEpipolarLine(
                        wrongRightPoint);
                
                double wrongDistanceLeft = leftEpipolarLine.signedDistance(
                        wrongLeftPoint);
                double wrongDistanceRight = rightEpipolarLine.signedDistance(
                        wrongRightPoint);
                
                //check that corrector has indeed reduced the amount of
                //projection error
                if ((Math.abs(correctedDistanceLeft) <=
                        Math.abs(wrongDistanceLeft)) && 
                        (Math.abs(correctedDistanceRight) <=
                        Math.abs(wrongDistanceRight))) {
                    improved++;
                }
            }
        }
        
        assertTrue(improved > 3 * total / 4);
    }

    @Override
    public void onCorrectStart(Corrector corrector) {
        correctStart++;
        checkLocked((SampsonCorrector)corrector);
    }

    @Override
    public void onCorrectEnd(Corrector corrector) {
        correctEnd++;
        checkLocked((SampsonCorrector)corrector);
    }

    @Override
    public void onCorrectProgressChange(Corrector corrector, float progress) {
        correctProgressChange++;
        checkLocked((SampsonCorrector)corrector);
    }

    private void reset(){
        correctStart = correctEnd = correctProgressChange = 0;
    }

    private void checkLocked(SampsonCorrector corrector){
        try {
            corrector.setFundamentalMatrix(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            corrector.setLeftAndRightPoints(null, null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            corrector.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            corrector.setPointsAndFundamentalMatrix(null, null, null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            corrector.setProgressDelta(0.5f);
        } catch (LockedException ignore) { }
        try {
            corrector.correct();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception ignore) {
            fail("LockedException expected but not thrown");
        }
        assertTrue(corrector.isLocked());
    }
}
