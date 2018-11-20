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

import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.geometry.Transformation2D;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class ImageOfAbsoluteConicEstimatorTest {
    
    public ImageOfAbsoluteConicEstimatorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testCreate() {

        //test without parameters
        ImageOfAbsoluteConicEstimator estimator =
                ImageOfAbsoluteConicEstimator.create();
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertNull(estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), 
                ImageOfAbsoluteConicEstimator.DEFAULT_ESTIMATOR_TYPE);
        assertTrue(estimator instanceof LMSEImageOfAbsoluteConicEstimator);
        
        //test with listener
        ImageOfAbsoluteConicEstimatorListener listener =
                new ImageOfAbsoluteConicEstimatorListener() {

            @Override
            public void onEstimateStart(
                    ImageOfAbsoluteConicEstimator estimator) { }

            @Override
            public void onEstimateEnd(
                    ImageOfAbsoluteConicEstimator estimator) { }

            @Override
            public void onEstimationProgressChange(
                    ImageOfAbsoluteConicEstimator estimator, float progress) { }
        };
        estimator = ImageOfAbsoluteConicEstimator.create(listener);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), listener);
        assertNull(estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ESTIMATOR_TYPE);
        assertTrue(estimator instanceof LMSEImageOfAbsoluteConicEstimator);
        
        //test with homographies
        List<Transformation2D> homographies = new ArrayList<>();
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        homographies.add(new ProjectiveTransformation2D());
        
        estimator = ImageOfAbsoluteConicEstimator.create(homographies);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertSame(estimator.getHomographies(), homographies);
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getType(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ESTIMATOR_TYPE);
        assertTrue(estimator instanceof LMSEImageOfAbsoluteConicEstimator);
        
        //Force IllegalArgumentException
        List<Transformation2D> emptyHomographies = 
                new ArrayList<>();
        estimator = null;
        try {
            estimator = ImageOfAbsoluteConicEstimator.create(emptyHomographies);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
        
        //test with homographies and listener
        estimator = ImageOfAbsoluteConicEstimator.create(homographies, 
                listener);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), listener);
        assertSame(estimator.getHomographies(), homographies);
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getType(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ESTIMATOR_TYPE);
        assertTrue(estimator instanceof LMSEImageOfAbsoluteConicEstimator);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = ImageOfAbsoluteConicEstimator.create(emptyHomographies, 
                    listener);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);        
        
        //test with LMSE type
        estimator = ImageOfAbsoluteConicEstimator.create(
                ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertNull(estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), 
                ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);
        assertTrue(estimator instanceof LMSEImageOfAbsoluteConicEstimator);
        
        //test with Weighted type
        estimator = ImageOfAbsoluteConicEstimator.create(
                ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertNull(estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), 
                ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR);
        assertTrue(estimator instanceof WeightedImageOfAbsoluteConicEstimator);
        
        //test with LMSE type and listener
        estimator = ImageOfAbsoluteConicEstimator.create(listener,
                ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), listener);
        assertNull(estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), 
                ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);
        assertTrue(estimator instanceof LMSEImageOfAbsoluteConicEstimator);
        
        //test with weighted type and listener
        estimator = ImageOfAbsoluteConicEstimator.create(listener,
                ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), listener);
        assertNull(estimator.getHomographies());
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), 
                ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR);
        assertTrue(estimator instanceof WeightedImageOfAbsoluteConicEstimator);
        
        //test with LMSE type and homographies
        estimator = ImageOfAbsoluteConicEstimator.create(homographies, 
                ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertSame(estimator.getHomographies(), homographies);
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getType(), 
                ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);
        assertTrue(estimator instanceof LMSEImageOfAbsoluteConicEstimator);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = ImageOfAbsoluteConicEstimator.create(emptyHomographies,
                    ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);

        //test with weighted type and homographies
        estimator = ImageOfAbsoluteConicEstimator.create(homographies, 
                ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertSame(estimator.getHomographies(), homographies);
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getType(), 
                ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR);
        assertTrue(estimator instanceof WeightedImageOfAbsoluteConicEstimator);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = ImageOfAbsoluteConicEstimator.create(emptyHomographies,
                    ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);

        //test with LMSE type, homographies and listener
        estimator = ImageOfAbsoluteConicEstimator.create(homographies, 
                listener, ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), listener);
        assertSame(estimator.getHomographies(), homographies);
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getType(), 
                ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR);
        assertTrue(estimator instanceof LMSEImageOfAbsoluteConicEstimator);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = ImageOfAbsoluteConicEstimator.create(emptyHomographies, 
                    listener, ImageOfAbsoluteConicEstimatorType.
                    LMSE_IAC_ESTIMATOR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);        

        //test with weighted type, homographies and listener
        estimator = ImageOfAbsoluteConicEstimator.create(homographies, 
                listener, ImageOfAbsoluteConicEstimatorType.
                WEIGHTED_IAC_ESTIMATOR);
        
        //check correctness
        assertEquals(estimator.isZeroSkewness(),
                ImageOfAbsoluteConicEstimator.DEFAULT_ZERO_SKEWNESS);
        assertEquals(estimator.isPrincipalPointAtOrigin(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_PRINCIPAL_POINT_AT_ORIGIN);
        assertEquals(estimator.isFocalDistanceAspectRatioKnown(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN);
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                ImageOfAbsoluteConicEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);        
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), listener);
        assertSame(estimator.getHomographies(), homographies);
        assertTrue(estimator.getMinNumberOfRequiredHomographies() > 0);
        assertTrue(estimator.isReady());
        assertEquals(estimator.getType(), 
                ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR);
        assertTrue(estimator instanceof WeightedImageOfAbsoluteConicEstimator);
        
        //Force IllegalArgumentException
        estimator = null;
        try {
            estimator = ImageOfAbsoluteConicEstimator.create(emptyHomographies, 
                    listener, ImageOfAbsoluteConicEstimatorType.
                    WEIGHTED_IAC_ESTIMATOR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);                
    }
}
