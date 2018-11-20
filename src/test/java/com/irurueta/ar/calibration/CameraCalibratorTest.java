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
package com.irurueta.ar.calibration;

import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class CameraCalibratorTest {
    
    public CameraCalibratorTest() { }
    
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
        //test create with method
        
        //Error Optimization
        CameraCalibrator calibrator = CameraCalibrator.create(
                CameraCalibratorMethod.ERROR_OPTIMIZATION);
        
        //check default values
        assertEquals(calibrator.getMethod(),
                CameraCalibratorMethod.ERROR_OPTIMIZATION);
        assertNull(calibrator.getPattern());
        assertNull(calibrator.getSamples());
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(calibrator.getEstimateRadialDistortion(),
                CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION);
        assertEquals(calibrator.getHomographyMethod(), 
                CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD);
        assertEquals(calibrator.getImageOfAbsoluteConicMethod(),
                CameraCalibrator.DEFAULT_IAC_METHOD);
        assertEquals(calibrator.isZeroSkewness(),
                calibrator.getIACEstimator().isZeroSkewness());
        assertEquals(calibrator.isPrincipalPointAtOrigin(),
                calibrator.getIACEstimator().isPrincipalPointAtOrigin());
        assertEquals(calibrator.isFocalDistanceAspectRatioKnown(),
                calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getFocalDistanceAspectRatio(),
                calibrator.getIACEstimator().getFocalDistanceAspectRatio(), 
                0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(calibrator.getProgressDelta(), 
                CameraCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertFalse(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimatorConfidence(),
                calibrator.getHomographyEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimatorMaxIterations(),
                calibrator.getHomographyEstimator().getMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimatorConfidence(),
                calibrator.getIACEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimatorMaxIterations(),
                calibrator.getIACEstimator().getMaxIterations());
        assertNull(calibrator.getListener());

        //Alternating Calibrator
        calibrator = CameraCalibrator.create(
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
        
        //check default values
        assertEquals(calibrator.getMethod(),
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
        assertNull(calibrator.getPattern());
        assertNull(calibrator.getSamples());
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(calibrator.getEstimateRadialDistortion(),
                CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION);
        assertEquals(calibrator.getHomographyMethod(), 
                CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD);
        assertEquals(calibrator.getImageOfAbsoluteConicMethod(),
                CameraCalibrator.DEFAULT_IAC_METHOD);
        assertEquals(calibrator.isZeroSkewness(),
                calibrator.getIACEstimator().isZeroSkewness());
        assertEquals(calibrator.isPrincipalPointAtOrigin(),
                calibrator.getIACEstimator().isPrincipalPointAtOrigin());
        assertEquals(calibrator.isFocalDistanceAspectRatioKnown(),
                calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getFocalDistanceAspectRatio(),
                calibrator.getIACEstimator().getFocalDistanceAspectRatio(), 
                0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(calibrator.getProgressDelta(), 
                CameraCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertFalse(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimatorConfidence(),
                calibrator.getHomographyEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimatorMaxIterations(),
                calibrator.getHomographyEstimator().getMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimatorConfidence(),
                calibrator.getIACEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimatorMaxIterations(),
                calibrator.getIACEstimator().getMaxIterations());
        assertNull(calibrator.getListener());
        
        
        //test create with pattern, samples and method
        Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
        List<CameraCalibratorSample> samples =
                new ArrayList<>();
        samples.add(new CameraCalibratorSample());
        
        //Error Optimization
        calibrator = CameraCalibrator.create(pattern, samples,
                CameraCalibratorMethod.ERROR_OPTIMIZATION);
        
        //check default values
        assertEquals(calibrator.getMethod(),
                CameraCalibratorMethod.ERROR_OPTIMIZATION);
        assertSame(calibrator.getPattern(), pattern);
        assertSame(calibrator.getSamples(), samples);
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(calibrator.getEstimateRadialDistortion(),
                CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION);
        assertEquals(calibrator.getHomographyMethod(), 
                CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD);
        assertEquals(calibrator.getImageOfAbsoluteConicMethod(),
                CameraCalibrator.DEFAULT_IAC_METHOD);
        assertEquals(calibrator.isZeroSkewness(),
                calibrator.getIACEstimator().isZeroSkewness());
        assertEquals(calibrator.isPrincipalPointAtOrigin(),
                calibrator.getIACEstimator().isPrincipalPointAtOrigin());
        assertEquals(calibrator.isFocalDistanceAspectRatioKnown(),
                calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getFocalDistanceAspectRatio(),
                calibrator.getIACEstimator().getFocalDistanceAspectRatio(), 
                0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(calibrator.getProgressDelta(), 
                CameraCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertTrue(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimatorConfidence(),
                calibrator.getHomographyEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimatorMaxIterations(),
                calibrator.getHomographyEstimator().getMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimatorConfidence(),
                calibrator.getIACEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimatorMaxIterations(),
                calibrator.getIACEstimator().getMaxIterations());
        assertNull(calibrator.getListener());
        
        //Force IllegalArgumentException
        List<CameraCalibratorSample> emptySamples =
                new ArrayList<>();
        calibrator = null;
        try {
            calibrator = CameraCalibrator.create(pattern, emptySamples, 
                    CameraCalibratorMethod.ERROR_OPTIMIZATION);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(calibrator);

        //Alternating Calibrator
        calibrator = CameraCalibrator.create(pattern, samples,
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
        
        //check default values
        assertEquals(calibrator.getMethod(),
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
        assertSame(calibrator.getPattern(), pattern);
        assertSame(calibrator.getSamples(), samples);
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(calibrator.getEstimateRadialDistortion(),
                CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION);
        assertEquals(calibrator.getHomographyMethod(), 
                CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD);
        assertEquals(calibrator.getImageOfAbsoluteConicMethod(),
                CameraCalibrator.DEFAULT_IAC_METHOD);
        assertEquals(calibrator.isZeroSkewness(),
                calibrator.getIACEstimator().isZeroSkewness());
        assertEquals(calibrator.isPrincipalPointAtOrigin(),
                calibrator.getIACEstimator().isPrincipalPointAtOrigin());
        assertEquals(calibrator.isFocalDistanceAspectRatioKnown(),
                calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getFocalDistanceAspectRatio(),
                calibrator.getIACEstimator().getFocalDistanceAspectRatio(), 
                0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(calibrator.getProgressDelta(), 
                CameraCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertTrue(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimatorConfidence(),
                calibrator.getHomographyEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimatorMaxIterations(),
                calibrator.getHomographyEstimator().getMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimatorConfidence(),
                calibrator.getIACEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimatorMaxIterations(),
                calibrator.getIACEstimator().getMaxIterations());
        assertNull(calibrator.getListener());
        
        //Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = CameraCalibrator.create(pattern, emptySamples, 
                    CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(calibrator);
        
        
        //test create with pattern, samples, quality scores and method
        double[] samplesQualityScores = new double[1];
        
        //Error Optimization
        calibrator = CameraCalibrator.create(pattern, samples,
                samplesQualityScores, 
                CameraCalibratorMethod.ERROR_OPTIMIZATION);
        
        //check default values
        assertEquals(calibrator.getMethod(),
                CameraCalibratorMethod.ERROR_OPTIMIZATION);
        assertSame(calibrator.getPattern(), pattern);
        assertSame(calibrator.getSamples(), samples);
        assertSame(calibrator.getSamplesQualityScores(), samplesQualityScores);
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(calibrator.getEstimateRadialDistortion(),
                CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION);
        assertEquals(calibrator.getHomographyMethod(), 
                CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD);
        assertEquals(calibrator.getImageOfAbsoluteConicMethod(),
                CameraCalibrator.DEFAULT_IAC_METHOD);
        assertEquals(calibrator.isZeroSkewness(),
                calibrator.getIACEstimator().isZeroSkewness());
        assertEquals(calibrator.isPrincipalPointAtOrigin(),
                calibrator.getIACEstimator().isPrincipalPointAtOrigin());
        assertEquals(calibrator.isFocalDistanceAspectRatioKnown(),
                calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getFocalDistanceAspectRatio(),
                calibrator.getIACEstimator().getFocalDistanceAspectRatio(), 
                0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(calibrator.getProgressDelta(), 
                CameraCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertTrue(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimatorConfidence(),
                calibrator.getHomographyEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimatorMaxIterations(),
                calibrator.getHomographyEstimator().getMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimatorConfidence(),
                calibrator.getIACEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimatorMaxIterations(),
                calibrator.getIACEstimator().getMaxIterations());
        assertNull(calibrator.getListener());
        
        //Force IllegalArgumentException
        double[] shortSamplesQualityScores = new double[0];
        calibrator = null;
        try {
            calibrator = CameraCalibrator.create(pattern, emptySamples, 
                    samplesQualityScores, 
                    CameraCalibratorMethod.ERROR_OPTIMIZATION);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            calibrator = CameraCalibrator.create(pattern, samples, 
                    shortSamplesQualityScores, 
                    CameraCalibratorMethod.ERROR_OPTIMIZATION);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(calibrator);

        //Alternating Calibrator
        calibrator = CameraCalibrator.create(pattern, samples,
                samplesQualityScores, 
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
        
        //check default values
        assertEquals(calibrator.getMethod(),
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
        assertSame(calibrator.getPattern(), pattern);
        assertSame(calibrator.getSamples(), samples);
        assertSame(calibrator.getSamplesQualityScores(), samplesQualityScores);
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(calibrator.getEstimateRadialDistortion(),
                CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION);
        assertEquals(calibrator.getHomographyMethod(), 
                CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD);
        assertEquals(calibrator.getImageOfAbsoluteConicMethod(),
                CameraCalibrator.DEFAULT_IAC_METHOD);
        assertEquals(calibrator.isZeroSkewness(),
                calibrator.getIACEstimator().isZeroSkewness());
        assertEquals(calibrator.isPrincipalPointAtOrigin(),
                calibrator.getIACEstimator().isPrincipalPointAtOrigin());
        assertEquals(calibrator.isFocalDistanceAspectRatioKnown(),
                calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getFocalDistanceAspectRatio(),
                calibrator.getIACEstimator().getFocalDistanceAspectRatio(), 
                0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(calibrator.getProgressDelta(), 
                CameraCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertTrue(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimatorConfidence(),
                calibrator.getHomographyEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimatorMaxIterations(),
                calibrator.getHomographyEstimator().getMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimatorConfidence(),
                calibrator.getIACEstimator().getConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimatorMaxIterations(),
                calibrator.getIACEstimator().getMaxIterations());
        assertNull(calibrator.getListener());
        
        calibrator = null;
        try {
            calibrator = CameraCalibrator.create(pattern, emptySamples, 
                    samplesQualityScores, 
                    CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            calibrator = CameraCalibrator.create(pattern, samples, 
                    shortSamplesQualityScores, 
                    CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(calibrator);
        
    }
}
