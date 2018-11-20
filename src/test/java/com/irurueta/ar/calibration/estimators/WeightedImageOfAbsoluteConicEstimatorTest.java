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
package com.irurueta.ar.calibration.estimators;

import com.irurueta.ar.calibration.ImageOfAbsoluteConic;
import com.irurueta.ar.calibration.Pattern2D;
import com.irurueta.ar.calibration.Pattern2DType;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.ProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class WeightedImageOfAbsoluteConicEstimatorTest implements
        ImageOfAbsoluteConicEstimatorListener {
    
    private static final double MIN_FOCAL_LENGTH = 3.0;
    private static final double MAX_FOCAL_LENGTH = 10.0;
    
    private static final double MIN_SKEWNESS = -0.01;
    private static final double MAX_SKEWNESS = 0.01;
    
    private static final double MIN_PRINCIPAL_POINT = -0.2;
    private static final double MAX_PRINCIPAL_POINT = 0.2;
    
    private static final double MIN_ANGLE_DEGREES = -10.0;
    private static final double MAX_ANGLE_DEGREES = 10.0;
    
    private static final int INHOM_3D_COORDS = 3;
    
    private static final double MIN_RANDOM_VALUE = -50.0;
    private static final double MAX_RANDOM_VALUE = -10.0;
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-3;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 5e-1;
    private static final double ULTRA_LARGE_ABSOLUTE_ERROR = 3.0;
    
    private static final int TIMES = 50;
    
    private int estimateStart;
    private int estimateEnd;
    private int estimationProgressChange;
    
    public WeightedImageOfAbsoluteConicEstimatorTest() { }
    
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
        WeightedImageOfAbsoluteConicEstimator estimator =
                new WeightedImageOfAbsoluteConicEstimator();
        
        //check default values
        assertEquals(estimator.isZeroSkewness(),
                WeightedImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertNull(estimator.getHomographies());
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(),
                ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR);
        assertNull(estimator.getWeights());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxHomographies(),
                WeightedImageOfAbsoluteConicEstimator.DEFAULT_MAX_HOMOGRAPHIES);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedImageOfAbsoluteConicEstimator.DEFAULT_SORT_WEIGHTS);
        
        //test constructor with listener
        estimator = new WeightedImageOfAbsoluteConicEstimator(this);
        
        //check default values
        assertEquals(estimator.isZeroSkewness(),
                WeightedImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);                
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getHomographies());
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(),
                ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR);
        assertNull(estimator.getWeights());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxHomographies(),
                WeightedImageOfAbsoluteConicEstimator.DEFAULT_MAX_HOMOGRAPHIES);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedImageOfAbsoluteConicEstimator.DEFAULT_SORT_WEIGHTS);

        //test constructor with homographies
        List<Transformation2D> homographies = new ArrayList<>();
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        double[] weights = new double[3];
        
        estimator = new WeightedImageOfAbsoluteConicEstimator(homographies, 
                weights);
        
        //check default values
        assertEquals(estimator.isZeroSkewness(),
                WeightedImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);                
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getType(),
                ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR);
        assertSame(estimator.getWeights(), weights);
        assertTrue(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxHomographies(),
                WeightedImageOfAbsoluteConicEstimator.DEFAULT_MAX_HOMOGRAPHIES);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedImageOfAbsoluteConicEstimator.DEFAULT_SORT_WEIGHTS);
        
        //Force IllegalArgumentException
        List<Transformation2D> emptyHomographies =
                new ArrayList<>();
        double[] shortWeights = new double[1];
        
        estimator = null;
        try {
            estimator = new WeightedImageOfAbsoluteConicEstimator(
                    null, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new WeightedImageOfAbsoluteConicEstimator(homographies, 
                    shortWeights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new WeightedImageOfAbsoluteConicEstimator(
                    emptyHomographies, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new WeightedImageOfAbsoluteConicEstimator(
                    homographies, shortWeights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //test constructor with homographies and listener
        estimator = new WeightedImageOfAbsoluteConicEstimator(homographies, 
                weights, this);
        
        //check default values
        assertEquals(estimator.isZeroSkewness(),
                WeightedImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);                
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getHomographies(), homographies);
        assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getType(),
                ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR);
        assertSame(estimator.getWeights(), weights);
        assertTrue(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxHomographies(),
                WeightedImageOfAbsoluteConicEstimator.DEFAULT_MAX_HOMOGRAPHIES);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedImageOfAbsoluteConicEstimator.DEFAULT_SORT_WEIGHTS);
        
        //Force IllegalArgumentException        
        estimator = null;
        try {
            estimator = new WeightedImageOfAbsoluteConicEstimator(
                    null, weights, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new WeightedImageOfAbsoluteConicEstimator(homographies, 
                    shortWeights, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new WeightedImageOfAbsoluteConicEstimator(
                    emptyHomographies, weights, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new WeightedImageOfAbsoluteConicEstimator(
                    homographies, shortWeights, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);        
    }
    
    @Test
    public void testIsSetZeroSkewness() throws LockedException {
        WeightedImageOfAbsoluteConicEstimator estimator =
                new WeightedImageOfAbsoluteConicEstimator();
        
        //check default value
        assertEquals(estimator.isZeroSkewness(),
                WeightedImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        
        //set new value
        estimator.setZeroSkewness(!WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_ZERO_SKEWNESS);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                !WeightedImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
    }
    
    @Test
    public void testIsSetPrincipalPointAtOrigin() throws LockedException {
        WeightedImageOfAbsoluteConicEstimator estimator =
                new WeightedImageOfAbsoluteConicEstimator();
        
        //check default value
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        
        //set new value
        estimator.setPrincipalPointAtOrigin(
                !WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        
        //check correctness
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                !WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
    }
    
    @Test
    public void testIsFocalDistanceAspectRatioKnown() throws LockedException {
        WeightedImageOfAbsoluteConicEstimator estimator =
                new WeightedImageOfAbsoluteConicEstimator();
        
        //check default value
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        
        //set new value
        estimator.setFocalDistanceAspectRatioKnown(
                !WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
    }
    
    @Test
    public void testGetSetFocalDistanceAspectRatio() throws LockedException {
        WeightedImageOfAbsoluteConicEstimator estimator =
                new WeightedImageOfAbsoluteConicEstimator();
        
        //check default value
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                WeightedImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);
        
        //set new value
        estimator.setFocalDistanceAspectRatio(0.5);
        
        //check correctness
        assertEquals(estimator.getFocalDistanceAspectRatio(), 0.5, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setFocalDistanceAspectRatio(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetListener() throws LockedException {
        WeightedImageOfAbsoluteConicEstimator estimator =
                new WeightedImageOfAbsoluteConicEstimator();
        
        //check default value
        assertNull(estimator.getListener());
        
        //set new value
        estimator.setListener(this);
        
        //check correctness
        assertSame(estimator.getListener(), this);
    }
    
    @Test
    public void testGetSetHomographiesAndWeightsAndIsReady() 
            throws LockedException {
        WeightedImageOfAbsoluteConicEstimator estimator =
                new WeightedImageOfAbsoluteConicEstimator();
        
        //check default value
        assertNull(estimator.getHomographies());
        assertNull(estimator.getWeights());
        assertFalse(estimator.isReady());
        
        //set new value
        List<Transformation2D> homographies = new ArrayList<>();
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        double[] weights = new double[3];
        
        estimator.setHomographiesAndWeights(homographies, weights);
        
        //check correctness
        assertSame(estimator.getHomographies(), homographies);
        assertSame(estimator.getWeights(), weights);
        assertTrue(estimator.isReady());
        
        //Force IllegalArgumentException
        List<Transformation2D> emptyHomographies =
                new ArrayList<>();
        double[] shortWeights = new double[1];
        
        try{
            estimator.setHomographiesAndWeights(null,
                    weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setHomographiesAndWeights(homographies, shortWeights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setHomographiesAndWeights(emptyHomographies, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setHomographiesAndWeights(homographies, shortWeights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        //setting only homographies always fails because weights must be 
        //provided too
        try {
            estimator.setHomographies(homographies);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetMaxHomographies() throws LockedException {
        WeightedImageOfAbsoluteConicEstimator estimator = 
                new WeightedImageOfAbsoluteConicEstimator();
        
        //check default value
        assertEquals(estimator.getMaxHomographies(),
                WeightedImageOfAbsoluteConicEstimator.DEFAULT_MAX_HOMOGRAPHIES);
        
        //set new value
        estimator.setMaxHomographies(
                estimator.getMinNumberOfRequiredHomographies());
        
        //check correctness
        assertEquals(estimator.getMaxHomographies(), 1);
        
        //Force IllegalArgumentException
        try {
            estimator.setMaxHomographies(
                    estimator.getMinNumberOfRequiredHomographies() - 1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testIsSetSortWeightsEnabled() throws LockedException {
        WeightedImageOfAbsoluteConicEstimator estimator =
                new WeightedImageOfAbsoluteConicEstimator();
        
        //check default value
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedImageOfAbsoluteConicEstimator.DEFAULT_SORT_WEIGHTS);
        
        //set new value
        estimator.setSortWeightsEnabled(
                !WeightedImageOfAbsoluteConicEstimator.DEFAULT_SORT_WEIGHTS);
        
        //check correctness
        assertEquals(estimator.isSortWeightsEnabled(),
                !WeightedImageOfAbsoluteConicEstimator.DEFAULT_SORT_WEIGHTS);
    }

    @Test
    @SuppressWarnings("all")
    public void testEstimateNoContraints() 
            throws InvalidPinholeCameraIntrinsicParametersException, 
            LockedException, NotReadyException, RobustEstimatorException,
            ImageOfAbsoluteConicEstimatorException {
        
        boolean succeededAtLeastOnce = false;
        int failed = 0, succeeded = 0, total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;     
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError, verticalFocalDistanceError,
                skewnessError, horizontalPrincipalPointError,
                verticalPrincipalPointError;        
        for (int j = 0; j < TIMES; j++) {
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);

            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);

            ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            //create pattern to estimate homography
            Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            List<Point2D> patternPoints = pattern.getIdealPoints();

            //assume that pattern points are located on a 3D plane 
            //(for instance Z = 0), but can be really any plane
            List<Point3D> points3D = new ArrayList<>();
            for (Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            //create 3 random cameras having random rotation and translation but
            //created intrinsic parameters in order to obtain 3 homographies to
            //estimate the IAC
            List<Transformation2D> homographies = 
                    new ArrayList<>();
            double[] weights = new double[50];
            randomizer.fill(weights, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            for (int i = 0; i < 50; i++) {
                //rotation
                double alphaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double betaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double gammaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);

                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, 
                        betaEuler, gammaEuler);

                //camera center
                double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE);
                InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);
                
                //create camera with intrinsic parameters, rotation and camera 
                //center
                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);
                camera.normalize();

                //project 3D pattern points in a plane
                List<Point2D> projectedPatternPoints = camera.project(points3D);

                ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                        createFromPoints(patternPoints, projectedPatternPoints, 
                        RobustEstimatorMethod.RANSAC);

                ProjectiveTransformation2D homography = 
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            //Estimate  IAC
            WeightedImageOfAbsoluteConicEstimator estimator = 
                    new WeightedImageOfAbsoluteConicEstimator(homographies, 
                    weights, this);
            estimator.setZeroSkewness(false);
            estimator.setPrincipalPointAtOrigin(false); 
            estimator.setFocalDistanceAspectRatioKnown(false);

            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 3);
            
            //check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            //estimate
            ImageOfAbsoluteConic iac2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            //check that estimated iac corresponds to the initial one (up to 
            //scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getB()), Math.abs(iac2.getB()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getD()), Math.abs(iac2.getD()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getE()), Math.abs(iac2.getE()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            
            try {
                PinholeCameraIntrinsicParameters intrinsic2 = 
                        iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(), 
                        intrinsic2.getHorizontalFocalLength(), 
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(), 
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(),
                        intrinsic2.getSkewness(), VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                        intrinsic2.getHorizontalPrincipalPoint(), 
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalPrincipalPoint(),
                        intrinsic2.getVerticalPrincipalPoint(), 
                        VERY_LARGE_ABSOLUTE_ERROR);
                
                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                        intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                        intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                        intrinsic2.getVerticalPrincipalPoint());
                
                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;
                
                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }
                
                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }
                
                succeededAtLeastOnce = true;
                succeeded++;
            } catch (InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }
        
        double failedRatio = (double)failed / (double)total;
        double succeededRatio = (double)succeeded / (double)total;
        
        avgHorizontalFocalDistanceError /= (double)succeeded;
        avgVerticalFocalDistanceError /= (double)succeeded;
        avgSkewnessError /= (double)succeeded;
        avgHorizontalPrincipalPointError /= (double)succeeded;
        avgVerticalPrincipalPointError /= (double)succeeded;
        
        //check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0, 
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, 
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, LARGE_ABSOLUTE_ERROR);
        
        
        String msg = "No contraints - failed: " + 
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 + 
                "% avg horizontal focal distance error: " + 
                avgHorizontalFocalDistanceError + 
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError + 
                " avg vertical principal point error: " + 
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;            
        Logger.getLogger(WeightedImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);
        
        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    @SuppressWarnings("all")
    public void testEstimateZeroSkewness() 
            throws InvalidPinholeCameraIntrinsicParametersException, 
            LockedException, NotReadyException, RobustEstimatorException, 
            ImageOfAbsoluteConicEstimatorException {
        
        boolean succeededAtLeastOnce = false;
        int failed = 0, succeeded = 0, total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;                
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError, verticalFocalDistanceError,
                skewnessError, horizontalPrincipalPointError,
                verticalPrincipalPointError;        
        for (int j = 0; j < TIMES; j++) {
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);

            double skewness = 0.0;

            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);

            ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            //create pattern to estimate homography
            Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            List<Point2D> patternPoints = pattern.getIdealPoints();

            //assume that pattern points are located on a 3D plane 
            //(for instance Z = 0), but can be really any plane
            List<Point3D> points3D = new ArrayList<>();
            for (Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            //create 3 random cameras having random rotation and translation but
            //created intrinsic parameters in order to obtain 3 homographies to
            //estimate the IAC
            List<Transformation2D> homographies = 
                    new ArrayList<>();
            double[] weights = new double[50];
            randomizer.fill(weights, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            for (int i = 0; i < 50; i++) {
                //rotation
                double alphaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double betaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double gammaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);

                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, 
                        betaEuler, gammaEuler);

                //camera center
                double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE);
                InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);
                
                //create camera with intrinsic parameters, rotation and camera 
                //center
                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);
                camera.normalize();

                //project 3D pattern points in a plane
                List<Point2D> projectedPatternPoints = camera.project(points3D);

                ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                        createFromPoints(patternPoints, projectedPatternPoints, 
                        RobustEstimatorMethod.RANSAC);

                ProjectiveTransformation2D homography = 
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            //Estimate  IAC
            WeightedImageOfAbsoluteConicEstimator estimator = 
                    new WeightedImageOfAbsoluteConicEstimator(homographies, 
                    weights, this);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(false);
            estimator.setFocalDistanceAspectRatioKnown(false);
            
            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 2);

            //check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            //estimate
            ImageOfAbsoluteConic iac2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            //check that estimated iac corresponds to the initial one (up to 
            //scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getB(), 0.0, 0.0);
            assertEquals(iac2.getB(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getD()), Math.abs(iac2.getD()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getE()), Math.abs(iac2.getE()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            
            try {
                PinholeCameraIntrinsicParameters intrinsic2 = 
                        iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(), 
                        intrinsic2.getHorizontalFocalLength(), 
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(), 
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                        intrinsic2.getHorizontalPrincipalPoint(), 
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalPrincipalPoint(),
                        intrinsic2.getVerticalPrincipalPoint(), 
                        VERY_LARGE_ABSOLUTE_ERROR);
                
                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                        intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                        intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                        intrinsic2.getVerticalPrincipalPoint());
                
                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;
                
                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }
                
                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }
                
                succeededAtLeastOnce = true;
                succeeded++;
            } catch (InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }
        
        double failedRatio = (double)failed / (double)total;
        double succeededRatio = (double)succeeded / (double)total;
        
        avgHorizontalFocalDistanceError /= (double)succeeded;
        avgVerticalFocalDistanceError /= (double)succeeded;
        avgSkewnessError /= (double)succeeded;
        avgHorizontalPrincipalPointError /= (double)succeeded;
        avgVerticalPrincipalPointError /= (double)succeeded;
        
        //check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0, 
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, 
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, LARGE_ABSOLUTE_ERROR);
        
        String msg = "Zero skewness - failed: " + 
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 + 
                "% avg horizontal focal distance error: " + 
                avgHorizontalFocalDistanceError + 
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError + 
                " avg vertical principal point error: " + 
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;                
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);
        
        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }
    
    @Test
    @SuppressWarnings("all")
    public void testEstimatePrincipalPointAtOrigin() 
            throws InvalidPinholeCameraIntrinsicParametersException, 
            LockedException, NotReadyException, RobustEstimatorException, 
            ImageOfAbsoluteConicEstimatorException {
        
        boolean succeededAtLeastOnce = false;
        int failed = 0, succeeded = 0, total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;                        
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError, verticalFocalDistanceError,
                skewnessError, horizontalPrincipalPointError,
                verticalPrincipalPointError;        
        for (int j = 0; j < TIMES; j++) {
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);

            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;

            PinholeCameraIntrinsicParameters intrinsic = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);

            ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            //create pattern to estimate homography
            Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            List<Point2D> patternPoints = pattern.getIdealPoints();

            //assume that pattern points are located on a 3D plane 
            //(for instance Z = 0), but can be really any plane
            List<Point3D> points3D = new ArrayList<>();
            for (Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            //create 3 random cameras having random rotation and translation but
            //created intrinsic parameters in order to obtain 3 homographies to
            //estimate the IAC
            List<Transformation2D> homographies = 
                    new ArrayList<>();
            double[] weights = new double[50];
            randomizer.fill(weights, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            for (int i = 0; i < 50; i++) {
                //rotation
                double alphaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double betaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double gammaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);

                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, 
                        betaEuler, gammaEuler);

                //camera center
                double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE);
                InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);
                
                //create camera with intrinsic parameters, rotation and camera 
                //center
                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);
                camera.normalize();

                //project 3D pattern points in a plane
                List<Point2D> projectedPatternPoints = camera.project(points3D);

                ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                        createFromPoints(patternPoints, projectedPatternPoints, 
                        RobustEstimatorMethod.RANSAC);

                ProjectiveTransformation2D homography = 
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            //Estimate  IAC
            WeightedImageOfAbsoluteConicEstimator estimator = 
                    new WeightedImageOfAbsoluteConicEstimator(homographies, 
                    weights, this);
            estimator.setZeroSkewness(false);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(false);
            
            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 2);

            //check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            //estimate
            ImageOfAbsoluteConic iac2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            //check that estimated iac corresponds to the initial one (up to 
            //scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getB()), Math.abs(iac2.getB()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getD(), 0.0, 0.0);
            assertEquals(iac2.getD(), 0.0, 0.0);
            assertEquals(iac.getE(), 0.0, 0.0);
            assertEquals(iac2.getE(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            
            try {
                PinholeCameraIntrinsicParameters intrinsic2 = 
                        iac2.getIntrinsicParameters();

                if (Math.abs(intrinsic.getHorizontalFocalLength() - intrinsic2.getHorizontalFocalLength()) >
                        VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalFocalLength(), 
                        intrinsic2.getHorizontalFocalLength(), 
                        VERY_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getVerticalFocalLength() - intrinsic2.getVerticalFocalLength()) >
                        VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(), 
                        VERY_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness()) > VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getSkewness(),
                        intrinsic2.getSkewness(), VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0, 
                        0.0);
                assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);
                
                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                        intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                        intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                        intrinsic2.getVerticalPrincipalPoint());
                
                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;
                
                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }
                
                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }
                
                succeededAtLeastOnce = true;
                succeeded++;
            } catch (InvalidPinholeCameraIntrinsicParametersException ignore) {
                failed++;
            }
            total++;
        }
        
        double failedRatio = (double)failed / (double)total;
        double succeededRatio = (double)succeeded / (double)total;
        
        avgHorizontalFocalDistanceError /= (double)succeeded;
        avgVerticalFocalDistanceError /= (double)succeeded;
        avgSkewnessError /= (double)succeeded;
        avgHorizontalPrincipalPointError /= (double)succeeded;
        avgVerticalPrincipalPointError /= (double)succeeded;
        
        //check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0, 
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, 
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, LARGE_ABSOLUTE_ERROR);
        
        String msg = "Principal point at origin - failed: " + 
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 + 
                "% avg horizontal focal distance error: " + 
                avgHorizontalFocalDistanceError + 
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError + 
                " avg vertical principal point error: " + 
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;          
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);
        
        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }

    @Test
    @SuppressWarnings("all")
    public void testEstimateZeroSkewnessPrincipalPointAtOrigin() 
            throws InvalidPinholeCameraIntrinsicParametersException, 
            LockedException, NotReadyException, RobustEstimatorException, 
            ImageOfAbsoluteConicEstimatorException {
        
        boolean succeededAtLeastOnce = false;
        int failed = 0, succeeded = 0, total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;        
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError, verticalFocalDistanceError,
                skewnessError, horizontalPrincipalPointError,
                verticalPrincipalPointError;                
        for (int j = 0; j < TIMES; j++) {
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);

            double skewness = 0.0;

            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;

            PinholeCameraIntrinsicParameters intrinsic = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);

            ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            //create pattern to estimate homography
            Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            List<Point2D> patternPoints = pattern.getIdealPoints();

            //assume that pattern points are located on a 3D plane 
            //(for instance Z = 0), but can be really any plane
            List<Point3D> points3D = new ArrayList<>();
            for (Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            //create 3 random cameras having random rotation and translation but
            //created intrinsic parameters in order to obtain 3 homographies to
            //estimate the IAC
            List<Transformation2D> homographies = 
                    new ArrayList<>();
            double[] weights = new double[50];
            randomizer.fill(weights, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            for (int i = 0; i < 50; i++) {
                //rotation
                double alphaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double betaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double gammaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);

                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, 
                        betaEuler, gammaEuler);

                //camera center
                double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE);
                InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);
                
                //create camera with intrinsic parameters, rotation and camera 
                //center
                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);
                camera.normalize();

                //project 3D pattern points in a plane
                List<Point2D> projectedPatternPoints = camera.project(points3D);

                ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                        createFromPoints(patternPoints, projectedPatternPoints, 
                        RobustEstimatorMethod.RANSAC);

                ProjectiveTransformation2D homography = 
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            //Estimate  IAC
            WeightedImageOfAbsoluteConicEstimator estimator = 
                    new WeightedImageOfAbsoluteConicEstimator(homographies, 
                    weights, this);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(false);
            
            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);

            //check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            //estimate
            ImageOfAbsoluteConic iac2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            //check that estimated iac corresponds to the initial one (up to 
            //scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getB(), 0.0, 0.0);
            assertEquals(iac2.getB(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getD(), 0.0, 0.0);
            assertEquals(iac2.getD(), 0.0, 0.0);
            assertEquals(iac.getE(), 0.0, 0.0);
            assertEquals(iac2.getE(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            
            try {
                PinholeCameraIntrinsicParameters intrinsic2 = 
                        iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(), 
                        intrinsic2.getHorizontalFocalLength(), 
                        ULTRA_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(), 
                        ULTRA_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0, 
                        0.0);
                assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);
                
                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                        intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                        intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                        intrinsic2.getVerticalPrincipalPoint());
                
                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;
                
                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }
                
                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }
                
                succeededAtLeastOnce = true;
                succeeded++;
            } catch (InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }
        
        double failedRatio = (double)failed / (double)total;
        double succeededRatio = (double)succeeded / (double)total;
        
        avgHorizontalFocalDistanceError /= (double)succeeded;
        avgVerticalFocalDistanceError /= (double)succeeded;
        avgSkewnessError /= (double)succeeded;
        avgHorizontalPrincipalPointError /= (double)succeeded;
        avgVerticalPrincipalPointError /= (double)succeeded;
        
        //check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0, 
                VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, 
                VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, 
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, LARGE_ABSOLUTE_ERROR);
        
        String msg = "No skewness and Principal point at origin - failed: " + 
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 + 
                "% avg horizontal focal distance error: " + 
                avgHorizontalFocalDistanceError + 
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError + 
                " avg vertical principal point error: " + 
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;            
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);
        
        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }    
    
    @Test
    @SuppressWarnings("all")
    public void testEstimateZeroSkewnessAspectRatioKnwon() 
            throws InvalidPinholeCameraIntrinsicParametersException, 
            LockedException, NotReadyException, RobustEstimatorException, 
            ImageOfAbsoluteConicEstimatorException {
        
        boolean succeededAtLeastOnce = false;
        int failed = 0, succeeded = 0, total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;                
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError, verticalFocalDistanceError,
                skewnessError, horizontalPrincipalPointError,
                verticalPrincipalPointError;        
        for (int j = 0; j < TIMES; j++) {
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            double skewness = 0.0;

            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic = 
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                    verticalPrincipalPoint, skewness);

            ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            //create pattern to estimate homography
            Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            List<Point2D> patternPoints = pattern.getIdealPoints();

            //assume that pattern points are located on a 3D plane 
            //(for instance Z = 0), but can be really any plane
            List<Point3D> points3D = new ArrayList<>();
            for (Point2D patternPoint : patternPoints) {
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            //create 3 random cameras having random rotation and translation but
            //created intrinsic parameters in order to obtain 3 homographies to
            //estimate the IAC
            List<Transformation2D> homographies = 
                    new ArrayList<>();
            double[] weights = new double[50];
            randomizer.fill(weights, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            for (int i = 0; i < 50; i++) {
                //rotation
                double alphaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double betaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double gammaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);

                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, 
                        betaEuler, gammaEuler);

                //camera center
                double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE);
                InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);
                
                //create camera with intrinsic parameters, rotation and camera 
                //center
                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);
                camera.normalize();

                //project 3D pattern points in a plane
                List<Point2D> projectedPatternPoints = camera.project(points3D);

                ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                        createFromPoints(patternPoints, projectedPatternPoints, 
                        RobustEstimatorMethod.RANSAC);

                ProjectiveTransformation2D homography = 
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            //Estimate  IAC
            WeightedImageOfAbsoluteConicEstimator estimator = 
                    new WeightedImageOfAbsoluteConicEstimator(homographies, 
                    weights, this);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(false);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(1.0);
            
            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 2);

            //check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            //estimate
            ImageOfAbsoluteConic iac2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            //check that estimated iac corresponds to the initial one (up to 
            //scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getB(), 0.0, 0.0);
            assertEquals(iac2.getB(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getD()), Math.abs(iac2.getD()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getE()), Math.abs(iac2.getE()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            
            try {
                PinholeCameraIntrinsicParameters intrinsic2 = 
                        iac2.getIntrinsicParameters();

                assertEquals(intrinsic.getHorizontalFocalLength(), 
                        intrinsic2.getHorizontalFocalLength(), 
                        2.0*VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(), 
                        2.0*VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                        intrinsic2.getHorizontalPrincipalPoint(), 
                        VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalPrincipalPoint(),
                        intrinsic2.getVerticalPrincipalPoint(), 
                        VERY_LARGE_ABSOLUTE_ERROR);
                
                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                        intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                        intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                        intrinsic2.getVerticalPrincipalPoint());
                
                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;
                
                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }
                
                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }
                
                succeededAtLeastOnce = true;
                succeeded++;
            } catch (InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }
        
        double failedRatio = (double)failed / (double)total;
        double succeededRatio = (double)succeeded / (double)total;
        
        avgHorizontalFocalDistanceError /= (double)succeeded;
        avgVerticalFocalDistanceError /= (double)succeeded;
        avgSkewnessError /= (double)succeeded;
        avgHorizontalPrincipalPointError /= (double)succeeded;
        avgVerticalPrincipalPointError /= (double)succeeded;
        
        //check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0, 
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, 
                LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, LARGE_ABSOLUTE_ERROR);
        
        String msg = "Zero skewness, aspect ratio known - failed: " + 
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 + 
                "% avg horizontal focal distance error: " + 
                avgHorizontalFocalDistanceError + 
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError + 
                " avg vertical principal point error: " + 
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;                
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);
        
        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }
    
    @Test
    @SuppressWarnings("all")
    public void testEstimateZeroSkewnessPrincipalPointAtOriginAspectRatioKnown() 
            throws InvalidPinholeCameraIntrinsicParametersException, 
            LockedException, NotReadyException, RobustEstimatorException, 
            ImageOfAbsoluteConicEstimatorException {
        
        boolean succeededAtLeastOnce = false;
        int failed = 0, succeeded = 0, total = 0;
        double avgHorizontalFocalDistanceError = 0.0;
        double avgVerticalFocalDistanceError = 0.0;
        double avgSkewnessError = 0.0;
        double avgHorizontalPrincipalPointError = 0.0;
        double avgVerticalPrincipalPointError = 0.0;        
        double minHorizontalFocalDistanceError = Double.MAX_VALUE;
        double minVerticalFocalDistanceError = Double.MAX_VALUE;
        double minSkewnessError = Double.MAX_VALUE;
        double minHorizontalPrincipalPointError = Double.MAX_VALUE;
        double minVerticalPrincipalPointError = Double.MAX_VALUE;
        double maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        double maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        double maxSkewnessError = -Double.MAX_VALUE;
        double maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        double maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError, verticalFocalDistanceError,
                skewnessError, horizontalPrincipalPointError,
                verticalPrincipalPointError;                
        for (int j = 0; j < TIMES; j++) {
            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            double skewness = 0.0;

            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;

            PinholeCameraIntrinsicParameters intrinsic = 
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, horizontalPrincipalPoint,
                    verticalPrincipalPoint, skewness);

            ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);

            //create pattern to estimate homography
            Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
            List<Point2D> patternPoints = pattern.getIdealPoints();

            //assume that pattern points are located on a 3D plane 
            //(for instance Z = 0), but can be really any plane
            List<Point3D> points3D = new ArrayList<>();
            for(Point2D patternPoint : patternPoints){
                points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(),
                        patternPoint.getInhomY(), 0.0, 1.0));
            }

            //create 3 random cameras having random rotation and translation but
            //created intrinsic parameters in order to obtain 3 homographies to
            //estimate the IAC
            List<Transformation2D> homographies = 
                    new ArrayList<>();
            double[] weights = new double[50];
            randomizer.fill(weights, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            for (int i = 0; i < 50; i++) {
                //rotation
                double alphaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double betaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);
                double gammaEuler = randomizer.nextDouble(
                        MIN_ANGLE_DEGREES * Math.PI / 180.0, 
                        MAX_ANGLE_DEGREES * Math.PI / 180.0);

                MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, 
                        betaEuler, gammaEuler);

                //camera center
                double[] cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE);
                InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                        cameraCenterArray);
                
                //create camera with intrinsic parameters, rotation and camera 
                //center
                PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                        cameraCenter);
                camera.normalize();

                //project 3D pattern points in a plane
                List<Point2D> projectedPatternPoints = camera.project(points3D);

                ProjectiveTransformation2DRobustEstimator homographyEstimator =
                        ProjectiveTransformation2DRobustEstimator.
                        createFromPoints(patternPoints, projectedPatternPoints, 
                        RobustEstimatorMethod.RANSAC);

                ProjectiveTransformation2D homography = 
                        homographyEstimator.estimate();
                homographies.add(homography);
            }

            //Estimate  IAC
            WeightedImageOfAbsoluteConicEstimator estimator = 
                    new WeightedImageOfAbsoluteConicEstimator(homographies, 
                    weights, this);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(1.0);
            
            assertEquals(estimator.getMinNumberOfRequiredHomographies(), 1);

            //check initial state
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);
            assertEquals(estimationProgressChange, 0);

            //estimate
            ImageOfAbsoluteConic iac2 = estimator.estimate();
            
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertTrue(estimationProgressChange >= 0);
            reset();

            //check that estimated iac corresponds to the initial one (up to 
            //scale)

            iac.normalize();
            iac2.normalize();

            assertEquals(Math.abs(iac.getA()), Math.abs(iac2.getA()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getB(), 0.0, 0.0);
            assertEquals(iac2.getB(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getC()), Math.abs(iac2.getC()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(iac.getD(), 0.0, 0.0);
            assertEquals(iac2.getD(), 0.0, 0.0);
            assertEquals(iac.getE(), 0.0, 0.0);
            assertEquals(iac2.getE(), 0.0, 0.0);
            assertEquals(Math.abs(iac.getF()), Math.abs(iac2.getF()), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            
            try {
                PinholeCameraIntrinsicParameters intrinsic2 = 
                        iac2.getIntrinsicParameters();

                if (Math.abs(intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength()) > ULTRA_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(intrinsic.getHorizontalFocalLength(), 
                        intrinsic2.getHorizontalFocalLength(), 
                        ULTRA_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getVerticalFocalLength(),
                        intrinsic2.getVerticalFocalLength(), 
                        ULTRA_LARGE_ABSOLUTE_ERROR);
                assertEquals(intrinsic.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic2.getSkewness(), 0.0, 0.0);
                assertEquals(intrinsic.getHorizontalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getHorizontalPrincipalPoint(), 0.0, 
                        0.0);
                assertEquals(intrinsic.getVerticalPrincipalPoint(), 0.0, 0.0);
                assertEquals(intrinsic2.getVerticalPrincipalPoint(), 0.0, 0.0);
                
                horizontalFocalDistanceError = Math.abs(
                        intrinsic.getHorizontalFocalLength() -
                        intrinsic2.getHorizontalFocalLength());
                verticalFocalDistanceError = Math.abs(
                        intrinsic.getVerticalFocalLength() -
                        intrinsic2.getVerticalFocalLength());
                skewnessError = Math.abs(intrinsic.getSkewness() -
                        intrinsic2.getSkewness());
                horizontalPrincipalPointError = Math.abs(
                        intrinsic.getHorizontalPrincipalPoint() -
                        intrinsic2.getHorizontalPrincipalPoint());
                verticalPrincipalPointError = Math.abs(
                        intrinsic.getVerticalPrincipalPoint() -
                        intrinsic2.getVerticalPrincipalPoint());
                
                avgHorizontalFocalDistanceError += horizontalFocalDistanceError;
                avgVerticalFocalDistanceError += verticalFocalDistanceError;
                avgSkewnessError += skewnessError;
                avgHorizontalPrincipalPointError += horizontalPrincipalPointError;
                avgVerticalPrincipalPointError += verticalPrincipalPointError;
                
                if (horizontalFocalDistanceError < minHorizontalFocalDistanceError) {
                    minHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError < minVerticalFocalDistanceError) {
                    minVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError < minSkewnessError) {
                    minSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError < minHorizontalPrincipalPointError) {
                    minHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError < minVerticalPrincipalPointError) {
                    minVerticalPrincipalPointError = verticalPrincipalPointError;
                }
                
                if (horizontalFocalDistanceError > maxHorizontalFocalDistanceError) {
                    maxHorizontalFocalDistanceError = horizontalFocalDistanceError;
                }
                if (verticalFocalDistanceError > maxVerticalFocalDistanceError) {
                    maxVerticalFocalDistanceError = verticalFocalDistanceError;
                }
                if (skewnessError > maxSkewnessError) {
                    maxSkewnessError = skewnessError;
                }
                if (horizontalPrincipalPointError > maxHorizontalPrincipalPointError) {
                    maxHorizontalPrincipalPointError = horizontalPrincipalPointError;
                }
                if (verticalPrincipalPointError > maxVerticalPrincipalPointError) {
                    maxVerticalPrincipalPointError = verticalPrincipalPointError;
                }
                
                succeededAtLeastOnce = true;
                succeeded++;
            } catch (InvalidPinholeCameraIntrinsicParametersException e) {
                failed++;
            }
            total++;
        }
        
        double failedRatio = (double)failed / (double)total;
        double succeededRatio = (double)succeeded / (double)total;
        
        avgHorizontalFocalDistanceError /= (double)succeeded;
        avgVerticalFocalDistanceError /= (double)succeeded;
        avgSkewnessError /= (double)succeeded;
        avgHorizontalPrincipalPointError /= (double)succeeded;
        avgVerticalPrincipalPointError /= (double)succeeded;
        
        //check that average error of intrinsic parameters is small enough
        assertEquals(avgHorizontalFocalDistanceError, 0.0, 
                VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalFocalDistanceError, 0.0, 
                VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgSkewnessError, 0.0, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgHorizontalPrincipalPointError, 0.0, 
                VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(avgVerticalPrincipalPointError, 0.0, 
                VERY_LARGE_ABSOLUTE_ERROR);
        
        String msg = "No skewness, Principal point at origin, aspect ratio known - failed: " + 
                failedRatio * 100.0 + "% succeeded: " + succeededRatio * 100.0 + 
                "% avg horizontal focal distance error: " + 
                avgHorizontalFocalDistanceError + 
                " avg vertical focal distance error: " +
                avgVerticalFocalDistanceError + " avg skewness error: " +
                avgSkewnessError + " avg horizontal principal point error: " +
                avgHorizontalPrincipalPointError + 
                " avg vertical principal point error: " + 
                avgVerticalPrincipalPointError +
                " min horizontal focal distance error: " +
                minHorizontalFocalDistanceError +
                " min vertical focal distance error: " +
                minVerticalFocalDistanceError + " min skewness error: " +
                minSkewnessError + " min horizontal principal point error: " +
                minHorizontalPrincipalPointError +
                " min vertical principal point error: " +
                minVerticalPrincipalPointError +
                " max horizontal focal distance error: " +
                maxHorizontalFocalDistanceError +
                " max vertical focal distance error: " +
                maxVerticalFocalDistanceError + " max skewness error: " +
                maxSkewnessError + " max horizontal principal point error: " +
                maxHorizontalPrincipalPointError +
                " max vertical principal point error: " +
                maxVerticalPrincipalPointError;            
        Logger.getLogger(LMSEImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);
        
        assertTrue(failedRatio < 0.75);
        assertTrue(succeededRatio >= 0.25);
        assertTrue(succeededAtLeastOnce);
    }    

    @Test
    public void testEstimateRealData() throws CoincidentPointsException, 
            InvalidPinholeCameraIntrinsicParametersException, LockedException,
            NotReadyException, ImageOfAbsoluteConicEstimatorException {
        //For a QR pattern, assuming zero skewness, equal focal lengths and
        //principal point at origin on a nexus 5 device
        Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);        
        List<Point2D> patternPoints = pattern.getIdealPoints();
        
        /*
        Sampled data (before and after centering coordinates and setting correct y axis 
        direction)
        Point[0] = 774.5, 1084.5 | 6.5, -60.5
        Point[1] = 791.5, 840.0 | 23.5, 184.0
        Point[2] = 1037.5, 854.0 | 269.5, 170.0
        Point[3] = 999.0, 1074.0 | 231.0, -50.0        
        */        
        List<Point2D> sampledPoints1 = new ArrayList<>();
        sampledPoints1.add(new InhomogeneousPoint2D(6.5, -60.5));
        sampledPoints1.add(new InhomogeneousPoint2D(23.5, 184.0));
        sampledPoints1.add(new InhomogeneousPoint2D(269.5, 170.0));
        sampledPoints1.add(new InhomogeneousPoint2D(231.0, -50.0));
        
        /*
        Sampled data (before and after centering coordinates and setting correct y axis 
        direction)
        Point[0] = 382.5, 701.5 | -385.5, 322.5
        Point[1] = 351.0, 473.5 | -417.0, 550.5
        Point[2] = 585.0, 451.5 | -183.0, 572.5
        Point[3] = 592.0, 653.5 | -176.0, 370.5
        */
        List<Point2D> sampledPoints2 = new ArrayList<>();
        sampledPoints2.add(new InhomogeneousPoint2D(-385.5, 322.5));
        sampledPoints2.add(new InhomogeneousPoint2D(-417.0, 550.5));
        sampledPoints2.add(new InhomogeneousPoint2D(-183.0, 572.5));
        sampledPoints2.add(new InhomogeneousPoint2D(-176.0, 370.5));

        /*
        Sampled data (before and after centering coordinates and setting correct y axis 
        direction)
        Point[0] = 988.0, 486.5 | 220.0, 537.5
        Point[1] = 1028.5, 278.5 | 260.5, 745.5
        Point[2] = 1241.0, 316.5 | 473.0, 707.5
        Point[3] = 1185.5, 498.5 | 417.5, 525.5
        */
        List<Point2D> sampledPoints3 = new ArrayList<>();
        sampledPoints3.add(new InhomogeneousPoint2D(220.0, 537.5));
        sampledPoints3.add(new InhomogeneousPoint2D(260.5, 745.5));
        sampledPoints3.add(new InhomogeneousPoint2D(473.0, 707.5));
        sampledPoints3.add(new InhomogeneousPoint2D(417.5, 525.5));
        
        /*
        Sampled data (before and after centering coordinates and setting correct y axis 
        direction)
        Point[0] = 576.0, 1404.4166 | -192.0, -380.4166259765625
        Point[1] = 544.5, 1151.5 | -223.5, -127.5
        Point[2] = 792.0, 1117.5 | 24.0, -93.5
        Point[3] = 798.5, 1347.0 | 30.5, -323.0
        */
        List<Point2D> sampledPoints4 = new ArrayList<>();
        sampledPoints4.add(new InhomogeneousPoint2D(-192.0, -380.4166259765625));
        sampledPoints4.add(new InhomogeneousPoint2D(-223.5, -127.5));
        sampledPoints4.add(new InhomogeneousPoint2D(24.0, -93.5));
        sampledPoints4.add(new InhomogeneousPoint2D(30.5, -323.0));

        /*
        Sampled data (before and after centering coordinates and setting correct y axis 
        direction)
        Point[0] = 913.5, 1596.0 | 145.5, -572.0
        Point[1] = 939.5, 1360.7 | 171.5, -336.699951171875
        Point[2] = 1170.5, 1391.0 | 402.5, -367.0
        Point[3] = 1126.5, 1600.5 | 358.5, -576.5 
        */   
        List<Point2D> sampledPoints5 = new ArrayList<>();
        sampledPoints5.add(new InhomogeneousPoint2D(145.5, -572.0));
        sampledPoints5.add(new InhomogeneousPoint2D(171.5, -336.699951171875));
        sampledPoints5.add(new InhomogeneousPoint2D(402.5, -367.0));
        sampledPoints5.add(new InhomogeneousPoint2D(358.5, -576.5));
        
        //obtain homographies
        ProjectiveTransformation2D homography1 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1), 
                patternPoints.get(2), patternPoints.get(3), 
                sampledPoints1.get(0), sampledPoints1.get(1), 
                sampledPoints1.get(2), sampledPoints1.get(3));

        ProjectiveTransformation2D homography2 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1), 
                patternPoints.get(2), patternPoints.get(3), 
                sampledPoints2.get(0), sampledPoints2.get(1), 
                sampledPoints2.get(2), sampledPoints2.get(3));

        ProjectiveTransformation2D homography3 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1), 
                patternPoints.get(2), patternPoints.get(3), 
                sampledPoints3.get(0), sampledPoints3.get(1), 
                sampledPoints3.get(2), sampledPoints3.get(3));
        
        ProjectiveTransformation2D homography4 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1), 
                patternPoints.get(2), patternPoints.get(3), 
                sampledPoints4.get(0), sampledPoints4.get(1), 
                sampledPoints4.get(2), sampledPoints4.get(3));
        
        ProjectiveTransformation2D homography5 = new ProjectiveTransformation2D(
                patternPoints.get(0), patternPoints.get(1), 
                patternPoints.get(2), patternPoints.get(3), 
                sampledPoints5.get(0), sampledPoints5.get(1), 
                sampledPoints5.get(2), sampledPoints5.get(3));
        
        
        List<Transformation2D> homographies = new ArrayList<>();
        homographies.add(homography1);
        homographies.add(homography2);
        homographies.add(homography3);
        homographies.add(homography4);
        homographies.add(homography5);
        
        double[] weights = new double[]{ 0.001, 1.0, 0.1, 0.003, 0.002};
        
        //estimate IAC
        WeightedImageOfAbsoluteConicEstimator estimator = 
                new WeightedImageOfAbsoluteConicEstimator(homographies, 
                weights);
        
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        
        ImageOfAbsoluteConic iac = estimator.estimate();
        
        assertNotNull(iac);
        
        //obtain intrinsic parameters
        PinholeCameraIntrinsicParameters intrinsic = 
                iac.getIntrinsicParameters();
        
        assertNotNull(intrinsic);
        
        double horizontalFocalLength = intrinsic.getHorizontalFocalLength();
        double verticalFocalLength = intrinsic.getVerticalFocalLength();
        double skewness = intrinsic.getSkewness();
        double horizontalPrincipalPoint = intrinsic.getHorizontalPrincipalPoint();
        double verticalPrincipalPoint = intrinsic.getVerticalPrincipalPoint();
        assertTrue(horizontalFocalLength > 0);
        assertTrue(verticalFocalLength > 0);
        assertEquals(horizontalFocalLength, verticalFocalLength, 
                ABSOLUTE_ERROR);
        assertEquals(skewness, 0.0, 0.0);
        assertEquals(horizontalPrincipalPoint, 0.0, 0.0);
        assertEquals(verticalPrincipalPoint, 0.0, 0.0);   
        
        String msg = "Real data focal length: " + horizontalFocalLength;
        Logger.getLogger(WeightedImageOfAbsoluteConicEstimatorTest.class.getName()).
                log(Level.INFO, msg);                
    }

    @Override
    public void onEstimateStart(ImageOfAbsoluteConicEstimator estimator) {
        estimateStart++;
        testLocked((WeightedImageOfAbsoluteConicEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(ImageOfAbsoluteConicEstimator estimator) {
        estimateEnd++;
        testLocked((WeightedImageOfAbsoluteConicEstimator)estimator);
    }

    @Override
    public void onEstimationProgressChange(
            ImageOfAbsoluteConicEstimator estimator, float progress) {
        estimationProgressChange++;
        testLocked((WeightedImageOfAbsoluteConicEstimator)estimator);
    }

    private void reset() {
        estimateStart = estimateEnd = estimationProgressChange = 0;
    }

    private void testLocked(WeightedImageOfAbsoluteConicEstimator estimator) {
        try {
            estimator.setZeroSkewness(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setPrincipalPointAtOrigin(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setHomographies(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator.setHomographiesAndWeights(null, null);
        } catch (LockedException ignore) { }
        try {
            estimator.setMaxHomographies(1);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setSortWeightsEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
    }
}
