/*
 * Copyright (C) 2016 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.ar.calibration.estimators;

import com.irurueta.ar.calibration.DualImageOfAbsoluteConic;
import com.irurueta.geometry.*;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidPairOfCamerasException;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class KruppaDualImageOfAbsoluteConicEstimatorTest implements
        KruppaDualImageOfAbsoluteConicEstimatorListener {
    
    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;
    
    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;
    
    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;
    
    private static final double MIN_ASPECT_RATIO = 0.5;
    private static final double MAX_ASPECT_RATIO = 2.0;
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 5.0;
    
    private static final int TIMES = 1000;
    
    private int estimateStart;
    private int estimateEnd;    
    
    public KruppaDualImageOfAbsoluteConicEstimatorTest() { }
    
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
        //test empty constructor
        KruppaDualImageOfAbsoluteConicEstimator estimator =
                new KruppaDualImageOfAbsoluteConicEstimator();
        
        //check default values
        assertEquals(estimator.getPrincipalPointX(), 0.0, 0.0);
        assertEquals(estimator.getPrincipalPointY(), 0.0, 0.0);
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertNull(estimator.getFundamentalMatrix());
        assertFalse(estimator.isReady());
        
        
        //test constructor with listener
        estimator = new KruppaDualImageOfAbsoluteConicEstimator(this);
        
        //check default values
        assertEquals(estimator.getPrincipalPointX(), 0.0, 0.0);
        assertEquals(estimator.getPrincipalPointY(), 0.0, 0.0);
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getFundamentalMatrix());
        assertFalse(estimator.isReady());
        
        
        //test constructor with fundamental matrix
        FundamentalMatrix f = new FundamentalMatrix();
        estimator = new KruppaDualImageOfAbsoluteConicEstimator(f);
        
        //check default values
        assertEquals(estimator.getPrincipalPointX(), 0.0, 0.0);
        assertEquals(estimator.getPrincipalPointY(), 0.0, 0.0);
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertSame(estimator.getFundamentalMatrix(), f);
        assertTrue(estimator.isReady());
        
        
        //test constructor with fundamental matrix and listener
        estimator = new KruppaDualImageOfAbsoluteConicEstimator(f, this);
        
        //check default values
        assertEquals(estimator.getPrincipalPointX(), 0.0, 0.0);
        assertEquals(estimator.getPrincipalPointY(), 0.0, 0.0);
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getFundamentalMatrix(), f);
        assertTrue(estimator.isReady());        
    }
    
    @Test
    public void testGetSetPrincipalPointX() throws LockedException {
        KruppaDualImageOfAbsoluteConicEstimator estimator = 
                new KruppaDualImageOfAbsoluteConicEstimator();
        
        //check default value
        assertEquals(estimator.getPrincipalPointX(), 0.0, 0.0);
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double principalPointX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        estimator.setPrincipalPointX(principalPointX);
        
        //check correctness
        assertEquals(estimator.getPrincipalPointX(), principalPointX, 0.0);
    }
    
    @Test
    public void testGetSetPrincipalPointY() throws LockedException {
        KruppaDualImageOfAbsoluteConicEstimator estimator = 
                new KruppaDualImageOfAbsoluteConicEstimator();
        
        //check default value
        assertEquals(estimator.getPrincipalPointY(), 0.0, 0.0);
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double principalPointY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        estimator.setPrincipalPointY(principalPointY);
        
        //check correctness
        assertEquals(estimator.getPrincipalPointY(), principalPointY, 0.0);        
    }
    
    @Test
    public void testIsSetFocalDistanceAspectRatioKnown() 
            throws LockedException {
        KruppaDualImageOfAbsoluteConicEstimator estimator = 
                new KruppaDualImageOfAbsoluteConicEstimator();
        
        //check default value
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        
        //st new value
        estimator.setFocalDistanceAspectRatioKnown(false);
        
        //check correctness
        assertFalse(estimator.isFocalDistanceAspectRatioKnown());
    }
    
    @Test
    public void testGetSetFocalDistanceAspectRatio() throws LockedException {
        KruppaDualImageOfAbsoluteConicEstimator estimator =
                new KruppaDualImageOfAbsoluteConicEstimator();
        
        //check default value
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double aspectRatio = randomizer.nextDouble(MIN_ASPECT_RATIO, 
                MAX_ASPECT_RATIO);
        
        estimator.setFocalDistanceAspectRatio(aspectRatio);
        
        //check correctness
        assertEquals(estimator.getFocalDistanceAspectRatio(), aspectRatio, 0.0);        
    }
    
    @Test
    public void testGetSetListener() throws LockedException {
        KruppaDualImageOfAbsoluteConicEstimator estimator =
                new KruppaDualImageOfAbsoluteConicEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        
        //set new value
        estimator.setListener(this);
        
        //check correctness
        assertSame(estimator.getListener(), this);
    }
    
    @Test
    public void testGetSetFundamentalMatrixAndIsReady() throws LockedException {
        KruppaDualImageOfAbsoluteConicEstimator estimator =
                new KruppaDualImageOfAbsoluteConicEstimator();
        
        //check default value
        assertNull(estimator.getFundamentalMatrix());
        assertFalse(estimator.isReady());
        
        //set new vlaue
        FundamentalMatrix f = new FundamentalMatrix();
        estimator.setFundamentalMatrix(f);
        
        //check correctness
        assertSame(estimator.getFundamentalMatrix(), f);
        assertTrue(estimator.isReady());
    }
    
    @Test
    public void testEstimateWithKnownAspectRatio() throws LockedException,
            NotReadyException, 
            InvalidPinholeCameraIntrinsicParametersException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            //create ground truth intrinsic parameters
            double horizontalFocalLength, verticalFocalLength, skewness,
                    horizontalPrincipalPoint, verticalPrincipalPoint;
            double aspectRatio = 1.0;
            horizontalFocalLength = 
                    randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            verticalFocalLength = aspectRatio * horizontalFocalLength;
            skewness = 0.0;
            horizontalPrincipalPoint = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            verticalPrincipalPoint = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);

            PinholeCameraIntrinsicParameters intrinsic = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength, 
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            double rollLeft, pitchLeft, yawLeft, xLeft, yLeft, zLeft;
            double rollRight, pitchRight, yawRight, xRight, yRight, zRight;
            Quaternion qLeft, qRight;
            InhomogeneousPoint3D leftCameraCenter, rightCameraCenter;
            PinholeCamera leftCamera, rightCamera;
            
            rollLeft = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            pitchLeft = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            yawLeft = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            qLeft = new Quaternion(rollLeft, pitchLeft, yawLeft);
            
            rollRight = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            pitchRight = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            yawRight = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            qRight = new Quaternion(rollRight, pitchRight, yawRight);
            
            xLeft = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            yLeft = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            zLeft = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            leftCameraCenter = new InhomogeneousPoint3D(xLeft, yLeft, zLeft);

            xRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            yRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            zRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            rightCameraCenter = new InhomogeneousPoint3D(xRight, yRight, 
                    zRight);
            
            leftCamera = new PinholeCamera(intrinsic, qLeft, leftCameraCenter);
            rightCamera = new PinholeCamera(intrinsic, qRight, 
                    rightCameraCenter);
            
            FundamentalMatrix fundamentalMatrix;
            try {
                fundamentalMatrix = new FundamentalMatrix(leftCamera, 
                        rightCamera);
            } catch (InvalidPairOfCamerasException e) {
                continue;
            }
        
            KruppaDualImageOfAbsoluteConicEstimator estimator = 
                    new KruppaDualImageOfAbsoluteConicEstimator(
                            fundamentalMatrix, this);
            estimator.setPrincipalPointX(horizontalPrincipalPoint);
            estimator.setPrincipalPointY(verticalPrincipalPoint);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(aspectRatio);
            
            //comprobamos
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertTrue(estimator.isReady());
        
            DualImageOfAbsoluteConic diac;
            DualImageOfAbsoluteConic diac2 = new DualImageOfAbsoluteConic(
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0);            
            try {
                diac = estimator.estimate();
                estimator.estimate(diac2);
            } catch (KruppaDualImageOfAbsoluteConicEstimatorException e) {
                continue;
            }
        
            //check correctness
            assertEquals(estimateStart, 2);
            assertEquals(estimateEnd, 2);
            diac.normalize();
            diac2.normalize();
        
            assertTrue(diac.asMatrix().equals(diac2.asMatrix(), 
                    ABSOLUTE_ERROR));
        
            PinholeCameraIntrinsicParameters intrinsic2 = 
                    diac.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters intrinsic3 =
                    diac2.getIntrinsicParameters();
        
            
            if (Math.abs(intrinsic2.getHorizontalFocalLength() -
                    horizontalFocalLength) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2.getHorizontalFocalLength(), 
                    horizontalFocalLength, ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalFocalLength(), 
                    verticalFocalLength, ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 
                    horizontalPrincipalPoint, ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), 
                    verticalPrincipalPoint, ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getSkewness(), 0.0, ABSOLUTE_ERROR);

            assertEquals(intrinsic3.getHorizontalFocalLength(), 
                    horizontalFocalLength, ABSOLUTE_ERROR);
            assertEquals(intrinsic3.getVerticalFocalLength(), 
                    verticalFocalLength, ABSOLUTE_ERROR);
            assertEquals(intrinsic3.getHorizontalPrincipalPoint(), 
                    horizontalPrincipalPoint, ABSOLUTE_ERROR);
            assertEquals(intrinsic3.getVerticalPrincipalPoint(), 
                    verticalPrincipalPoint, ABSOLUTE_ERROR);
            assertEquals(intrinsic3.getSkewness(), 0.0, ABSOLUTE_ERROR);
            
            numValid++;
        }
        
        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateWithUnknownAspectRatio() 
            throws LockedException, NotReadyException,
            InvalidPinholeCameraIntrinsicParametersException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            //create ground truth intrinsic parameters
            double horizontalFocalLength, verticalFocalLength, skewness,
                    horizontalPrincipalPoint, verticalPrincipalPoint;
            double aspectRatio = randomizer.nextDouble(MIN_ASPECT_RATIO, 
                    MAX_ASPECT_RATIO);
            horizontalFocalLength = 
                    randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            verticalFocalLength = aspectRatio * horizontalFocalLength;
            skewness = 0.0;
            horizontalPrincipalPoint = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            verticalPrincipalPoint = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);

            PinholeCameraIntrinsicParameters intrinsic = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength, 
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            double rollLeft, pitchLeft, yawLeft, xLeft, yLeft, zLeft;
            double rollRight, pitchRight, yawRight, xRight, yRight, zRight;
            Quaternion qLeft, qRight;
            InhomogeneousPoint3D leftCameraCenter, rightCameraCenter;
            PinholeCamera leftCamera, rightCamera;
            
            rollLeft = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            pitchLeft = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            yawLeft = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            qLeft = new Quaternion(rollLeft, pitchLeft, yawLeft);
            
            rollRight = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            pitchRight = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            yawRight = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            qRight = new Quaternion(rollRight, pitchRight, yawRight);
            
            xLeft = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            yLeft = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            zLeft = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            leftCameraCenter = new InhomogeneousPoint3D(xLeft, yLeft, zLeft);

            xRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            yRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            zRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            rightCameraCenter = new InhomogeneousPoint3D(xRight, yRight, 
                    zRight);
            
            leftCamera = new PinholeCamera(intrinsic, qLeft, leftCameraCenter);
            rightCamera = new PinholeCamera(intrinsic, qRight, 
                    rightCameraCenter);
            
            FundamentalMatrix fundamentalMatrix;
            try {
                fundamentalMatrix = new FundamentalMatrix(leftCamera, 
                        rightCamera);
            } catch (InvalidPairOfCamerasException e) {
                continue;
            }
        
            KruppaDualImageOfAbsoluteConicEstimator estimator = 
                    new KruppaDualImageOfAbsoluteConicEstimator(
                            fundamentalMatrix, this);
            estimator.setPrincipalPointX(horizontalPrincipalPoint);
            estimator.setPrincipalPointY(verticalPrincipalPoint);
            estimator.setFocalDistanceAspectRatioKnown(false);
            
            //comprobamos
            reset();
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertTrue(estimator.isReady());
        
            DualImageOfAbsoluteConic diac;
            DualImageOfAbsoluteConic diac2 = new DualImageOfAbsoluteConic(
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0);            
            try {
                diac = estimator.estimate();
                estimator.estimate(diac2);
            } catch (KruppaDualImageOfAbsoluteConicEstimatorException e) {
                continue;
            }
        
            //check correctness
            assertEquals(estimateStart, 2);
            assertEquals(estimateEnd, 2);
            diac.normalize();
            diac2.normalize();
        
            assertTrue(diac.asMatrix().equals(diac2.asMatrix(), 
                    ABSOLUTE_ERROR));
        
            PinholeCameraIntrinsicParameters intrinsic2 = 
                    diac.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters intrinsic3 =
                    diac2.getIntrinsicParameters();
        
            
            if (Math.abs(intrinsic2.getHorizontalFocalLength() -
                    horizontalFocalLength) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2.getHorizontalFocalLength(), 
                    horizontalFocalLength, LARGE_ABSOLUTE_ERROR);
            if (Math.abs(intrinsic2.getVerticalFocalLength() -
                    verticalFocalLength) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2.getVerticalFocalLength(), 
                    verticalFocalLength, LARGE_ABSOLUTE_ERROR);
            if (Math.abs(intrinsic2.getHorizontalPrincipalPoint() -
                    horizontalPrincipalPoint) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 
                    horizontalPrincipalPoint, ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), 
                    verticalPrincipalPoint, ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getSkewness(), 0.0, ABSOLUTE_ERROR);

            assertEquals(intrinsic3.getHorizontalFocalLength(), 
                    horizontalFocalLength, LARGE_ABSOLUTE_ERROR);
            assertEquals(intrinsic3.getVerticalFocalLength(), 
                    verticalFocalLength, LARGE_ABSOLUTE_ERROR);
            assertEquals(intrinsic3.getHorizontalPrincipalPoint(), 
                    horizontalPrincipalPoint, ABSOLUTE_ERROR);
            assertEquals(intrinsic3.getVerticalPrincipalPoint(), 
                    verticalPrincipalPoint, ABSOLUTE_ERROR);
            assertEquals(intrinsic3.getSkewness(), 0.0, ABSOLUTE_ERROR);
            
            numValid++;
        }
        
        assertTrue(numValid > 0);
    }
    
    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    @Override
    public void onEstimateStart(
            KruppaDualImageOfAbsoluteConicEstimator estimator) {
        estimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(
            KruppaDualImageOfAbsoluteConicEstimator estimator) {
        estimateEnd++;
        checkLocked(estimator);
    }
    
    private void checkLocked(
            KruppaDualImageOfAbsoluteConicEstimator estimator) {
        assertTrue(estimator.isLocked());
        
        try {
            estimator.setPrincipalPointX(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setPrincipalPointY(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setFocalDistanceAspectRatioKnown(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setFocalDistanceAspectRatio(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setFundamentalMatrix(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception ignore) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.estimate(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception ignore) {
            fail("LockedException expected but not thrown");
        }        
    }
}
