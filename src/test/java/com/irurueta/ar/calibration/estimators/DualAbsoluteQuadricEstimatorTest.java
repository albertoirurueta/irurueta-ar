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

import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class DualAbsoluteQuadricEstimatorTest implements
        DualAbsoluteQuadricEstimatorListener {
    
    private static final double MIN_ASPECT_RATIO = 0.5;
    private static final double MAX_ASPECT_RATIO = 2.0;
    
    public DualAbsoluteQuadricEstimatorTest() { }
    
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
        DualAbsoluteQuadricEstimator estimator =
                DualAbsoluteQuadricEstimator.create();
        
        assertTrue(estimator instanceof LMSEDualAbsoluteQuadricEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);        
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertNull(estimator.getCameras()); 
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), DualAbsoluteQuadricEstimatorType.
                LMSE_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR);
        
        
        estimator = DualAbsoluteQuadricEstimator.create(
                DualAbsoluteQuadricEstimatorType.
                        LMSE_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR);
        
        assertTrue(estimator instanceof LMSEDualAbsoluteQuadricEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced()); 
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);        
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());        
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), DualAbsoluteQuadricEstimatorType.
                LMSE_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR);
        
        
        estimator = DualAbsoluteQuadricEstimator.create(
                DualAbsoluteQuadricEstimatorType.
                        WEIGHTED_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR);
        
        assertTrue(estimator instanceof WeightedDualAbsoluteQuadricEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);        
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());                
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), DualAbsoluteQuadricEstimatorType.
                WEIGHTED_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR);   
        
        
        //test create with listener
        estimator = DualAbsoluteQuadricEstimator.create(this);
        
        assertTrue(estimator instanceof LMSEDualAbsoluteQuadricEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);        
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());                        
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), DualAbsoluteQuadricEstimatorType.
                LMSE_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR);
        
        estimator = DualAbsoluteQuadricEstimator.create(this,
                DualAbsoluteQuadricEstimatorType.
                        LMSE_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR);
        
        assertTrue(estimator instanceof LMSEDualAbsoluteQuadricEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);        
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), DualAbsoluteQuadricEstimatorType.
                LMSE_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR);
        
        estimator = DualAbsoluteQuadricEstimator.create(this,
                DualAbsoluteQuadricEstimatorType.
                        WEIGHTED_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR);
        
        assertTrue(estimator instanceof WeightedDualAbsoluteQuadricEstimator);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);        
        assertFalse(estimator.isLocked());
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), DualAbsoluteQuadricEstimatorType.
                WEIGHTED_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR);
    }
    
    @Test
    public void testIsSetZeroSkewness() throws LockedException {
        DualAbsoluteQuadricEstimator estimator = DualAbsoluteQuadricEstimator.
                create();
        
        //check default value
        assertTrue(estimator.isZeroSkewness());
        
        //set new value
        estimator.setZeroSkewness(false);
        
        //check correctness
        assertFalse(estimator.isZeroSkewness());
    }
    
    @Test
    public void testIsSetPrincipalPointAtOrigin() throws LockedException {
        DualAbsoluteQuadricEstimator estimator = DualAbsoluteQuadricEstimator.
                create();
        
        //check default value
        assertTrue(estimator.isPrincipalPointAtOrigin());
        
        //set new value
        estimator.setPrincipalPointAtOrigin(false);
        
        //check correctness
        assertFalse(estimator.isPrincipalPointAtOrigin());
    }
    
    @Test
    public void testIsSetFocalDistanceAspectRatioKnown() throws LockedException {
        DualAbsoluteQuadricEstimator estimator = DualAbsoluteQuadricEstimator.
                create();
        
        //check default value
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        
        //set new value
        estimator.setFocalDistanceAspectRatioKnown(false);
        
        //check correctness
        assertFalse(estimator.isFocalDistanceAspectRatioKnown());
    }
    
    @Test
    public void testGetSetFocalDistanceAspectRatio() throws LockedException {
        DualAbsoluteQuadricEstimator estimator = DualAbsoluteQuadricEstimator.
                create();
        
        //check default value
        assertEquals(estimator.getFocalDistanceAspectRatio(), 
                DualAbsoluteQuadricEstimator.
                        DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double aspectRatio = randomizer.nextDouble(MIN_ASPECT_RATIO, 
                MAX_ASPECT_RATIO);
        estimator.setFocalDistanceAspectRatio(aspectRatio);
        
        //check correctness
        assertEquals(estimator.getFocalDistanceAspectRatio(), aspectRatio, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setFocalDistanceAspectRatio(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testIsSetSingularityEnforced() throws LockedException {
        DualAbsoluteQuadricEstimator estimator = DualAbsoluteQuadricEstimator.
                create();
        
        //check default value
        assertTrue(estimator.isSingularityEnforced());
        
        //set new value
        estimator.setSingularityEnforced(false);
        
        //check correctness
        assertFalse(estimator.isSingularityEnforced());
    }
    
    @Test
    public void testIsSetEnforcedSingularityValidated() throws LockedException {
        DualAbsoluteQuadricEstimator estimator = DualAbsoluteQuadricEstimator.
                create();
        
        //check default value
        assertTrue(estimator.isEnforcedSingularityValidated());
        
        //set new value
        estimator.setEnforcedSingularityValidated(false);
        
        //check correctness
        assertFalse(estimator.isEnforcedSingularityValidated());
    }
    
    @Test
    public void testGetSetDeterminantThreshold() throws LockedException {
        DualAbsoluteQuadricEstimator estimator = DualAbsoluteQuadricEstimator.
                create();
        
        //check default value
        assertEquals(estimator.getDeterminantThreshold(), 
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, 
                0.0);
        
        //set new value
        estimator.setDeterminantThreshold(1e-3);
        
        //check correctness
        assertEquals(estimator.getDeterminantThreshold(), 1e-3, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setDeterminantThreshold(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetListener() {
        DualAbsoluteQuadricEstimator estimator = DualAbsoluteQuadricEstimator.
                create();
        
        //check default value
        assertNull(estimator.getListener());
        
        //set new value
        estimator.setListener(this);
        
        //check correctness
        assertSame(estimator.getListener(), this);
    }
    
    @Test
    public void testGetSetCameras() throws LockedException {
        DualAbsoluteQuadricEstimator estimator = DualAbsoluteQuadricEstimator.
                create();
        
        //check default value
        assertNull(estimator.getCameras());
        
        List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        
        estimator.setCameras(cameras);
        
        //check correctness
        assertSame(estimator.getCameras(), cameras);
        
        //Force IllegalArgumentException
        cameras.clear();
        
        try {
            estimator.setCameras(cameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetMinNumberOfRequiredCameras() throws LockedException {
        DualAbsoluteQuadricEstimator estimator = DualAbsoluteQuadricEstimator.
                create();
        
        //check default value
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        
        //disable principal point at origin
        estimator.setPrincipalPointAtOrigin(false);
        
        //check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), -1);
        
        
        //disable zero skewness
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setZeroSkewness(false);
        
        //check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 4);
        
        
        //disable focal distance aspect ratio known
        estimator.setZeroSkewness(true);
        estimator.setFocalDistanceAspectRatioKnown(false);
        
        //check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 3);
        
        
        //disable zero skewness and singularity enforcement
        estimator.setZeroSkewness(false);
        estimator.setSingularityEnforced(false);
        
        //check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 5);
        
        
        //disable focal distance aspect ratio known and singularity enforcement
        estimator.setZeroSkewness(true);
        
        //check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 3);
    }
    
    @Test
    public void testAreValidConstraints() throws LockedException {
        DualAbsoluteQuadricEstimator estimator = DualAbsoluteQuadricEstimator.
                create();
        
        //check default value
        assertTrue(estimator.areValidConstraints());
        
        //disable principal point at origin
        estimator.setPrincipalPointAtOrigin(false);
        
        //check correctness
        assertFalse(estimator.areValidConstraints());
        
        
        //disable zero skewness
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setZeroSkewness(false);
        
        //check correctness
        assertTrue(estimator.areValidConstraints());
        
        
        //disable focal distance aspect ratio known
        estimator.setZeroSkewness(true);
        estimator.setFocalDistanceAspectRatioKnown(false);
        
        //check correctness
        assertFalse(estimator.areValidConstraints());
        
        //disable singularity enforcement
        estimator.setSingularityEnforced(false);
        
        //check correctness
        assertTrue(estimator.areValidConstraints());
    }
    
    @Test
    public void testIsReady() throws LockedException {        
        DualAbsoluteQuadricEstimator estimator = DualAbsoluteQuadricEstimator.
                create();
        
        //check default value
        assertNull(estimator.getCameras());
        assertTrue(estimator.areValidConstraints());        
        assertFalse(estimator.isReady());
        
        List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        
        estimator.setCameras(cameras);
        
        //check correctness
        assertTrue(estimator.isReady());

        
        //disable principal point at origin
        estimator.setPrincipalPointAtOrigin(false);
        
        //check correctness
        assertFalse(estimator.areValidConstraints());
        assertFalse(estimator.isReady());

        //enable principal point at origin
        estimator.setPrincipalPointAtOrigin(true);
        
        //check correctness
        assertTrue(estimator.isReady());
        
        //clear cameras
        cameras.clear();

        //check correctness
        assertFalse(estimator.isReady());
    }

    @Override
    public void onEstimateStart(DualAbsoluteQuadricEstimator estimator) { }

    @Override
    public void onEstimateEnd(DualAbsoluteQuadricEstimator estimator) { }

    @Override
    public void onEstimationProgressChange(
            DualAbsoluteQuadricEstimator estimator, float progress) { }
}
