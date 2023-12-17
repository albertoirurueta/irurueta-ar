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

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class CameraCalibratorTest {

    @Test
    public void testCreate() {
        // test create with method

        // Error Optimization
        CameraCalibrator calibrator = CameraCalibrator.create(CameraCalibratorMethod.ERROR_OPTIMIZATION);

        // check default values
        assertEquals(CameraCalibratorMethod.ERROR_OPTIMIZATION, calibrator.getMethod());
        assertNull(calibrator.getPattern());
        assertNull(calibrator.getSamples());
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION,
                calibrator.getEstimateRadialDistortion());
        assertEquals(CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD, calibrator.getHomographyMethod());
        assertEquals(CameraCalibrator.DEFAULT_IAC_METHOD, calibrator.getImageOfAbsoluteConicMethod());
        assertEquals(calibrator.getIACEstimator().isZeroSkewness(), calibrator.isZeroSkewness());
        assertEquals(calibrator.getIACEstimator().isPrincipalPointAtOrigin(),
                calibrator.isPrincipalPointAtOrigin());
        assertEquals(calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown(),
                calibrator.isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getIACEstimator().getFocalDistanceAspectRatio(),
                calibrator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(CameraCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertFalse(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimator().getConfidence(),
                calibrator.getHomographyEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimator().getMaxIterations(),
                calibrator.getHomographyEstimatorMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimator().getConfidence(),
                calibrator.getIACEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimator().getMaxIterations(),
                calibrator.getIACEstimatorMaxIterations());
        assertNull(calibrator.getListener());

        // Alternating Calibrator
        calibrator = CameraCalibrator.create(CameraCalibratorMethod.ALTERNATING_CALIBRATOR);

        // check default values
        assertEquals(CameraCalibratorMethod.ALTERNATING_CALIBRATOR,
                calibrator.getMethod());
        assertNull(calibrator.getPattern());
        assertNull(calibrator.getSamples());
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION,
                calibrator.getEstimateRadialDistortion());
        assertEquals(CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD, calibrator.getHomographyMethod());
        assertEquals(CameraCalibrator.DEFAULT_IAC_METHOD, calibrator.getImageOfAbsoluteConicMethod());
        assertEquals(calibrator.getIACEstimator().isZeroSkewness(), calibrator.isZeroSkewness());
        assertEquals(calibrator.getIACEstimator().isPrincipalPointAtOrigin(),
                calibrator.isPrincipalPointAtOrigin());
        assertEquals(calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown(),
                calibrator.isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getIACEstimator().getFocalDistanceAspectRatio(),
                calibrator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(CameraCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 0.0);
        assertFalse(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimator().getConfidence(),
                calibrator.getHomographyEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimator().getMaxIterations(),
                calibrator.getHomographyEstimatorMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimator().getConfidence(),
                calibrator.getIACEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimator().getMaxIterations(),
                calibrator.getIACEstimatorMaxIterations());
        assertNull(calibrator.getListener());

        // test create with pattern, samples and method
        final Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
        final List<CameraCalibratorSample> samples = new ArrayList<>();
        samples.add(new CameraCalibratorSample());

        // Error Optimization
        calibrator = CameraCalibrator.create(pattern, samples, CameraCalibratorMethod.ERROR_OPTIMIZATION);

        // check default values
        assertEquals(CameraCalibratorMethod.ERROR_OPTIMIZATION,
                calibrator.getMethod());
        assertSame(pattern, calibrator.getPattern());
        assertSame(samples, calibrator.getSamples());
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION,
                calibrator.getEstimateRadialDistortion());
        assertEquals(CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD,
                calibrator.getHomographyMethod());
        assertEquals(CameraCalibrator.DEFAULT_IAC_METHOD,
                calibrator.getImageOfAbsoluteConicMethod());
        assertEquals(calibrator.getIACEstimator().isZeroSkewness(),
                calibrator.isZeroSkewness());
        assertEquals(calibrator.getIACEstimator().isPrincipalPointAtOrigin(),
                calibrator.isPrincipalPointAtOrigin());
        assertEquals(calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown(),
                calibrator.isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getIACEstimator().getFocalDistanceAspectRatio(),
                calibrator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(CameraCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 0.0);
        assertTrue(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimator().getConfidence(),
                calibrator.getHomographyEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimator().getMaxIterations(),
                calibrator.getHomographyEstimatorMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimator().getConfidence(),
                calibrator.getIACEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimator().getMaxIterations(),
                calibrator.getIACEstimatorMaxIterations());
        assertNull(calibrator.getListener());

        // Force IllegalArgumentException
        final List<CameraCalibratorSample> emptySamples = new ArrayList<>();
        calibrator = null;
        try {
            calibrator = CameraCalibrator.create(pattern, emptySamples,
                    CameraCalibratorMethod.ERROR_OPTIMIZATION);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);

        // Alternating Calibrator
        calibrator = CameraCalibrator.create(pattern, samples,
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);

        // check default values
        assertEquals(CameraCalibratorMethod.ALTERNATING_CALIBRATOR, calibrator.getMethod());
        assertSame(pattern, calibrator.getPattern());
        assertSame(samples, calibrator.getSamples());
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION,
                calibrator.getEstimateRadialDistortion());
        assertEquals(CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD,
                calibrator.getHomographyMethod());
        assertEquals(CameraCalibrator.DEFAULT_IAC_METHOD,
                calibrator.getImageOfAbsoluteConicMethod());
        assertEquals(calibrator.getIACEstimator().isZeroSkewness(),
                calibrator.isZeroSkewness());
        assertEquals(calibrator.getIACEstimator().isPrincipalPointAtOrigin(),
                calibrator.isPrincipalPointAtOrigin());
        assertEquals(calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown(),
                calibrator.isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getIACEstimator().getFocalDistanceAspectRatio(),
                calibrator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(CameraCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertTrue(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimator().getConfidence(),
                calibrator.getHomographyEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimator().getMaxIterations(),
                calibrator.getHomographyEstimatorMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimator().getConfidence(),
                calibrator.getIACEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimator().getMaxIterations(),
                calibrator.getIACEstimatorMaxIterations());
        assertNull(calibrator.getListener());

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = CameraCalibrator.create(pattern, emptySamples,
                    CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);

        // test create with pattern, samples, quality scores and method
        final double[] samplesQualityScores = new double[1];

        // Error Optimization
        calibrator = CameraCalibrator.create(pattern, samples, samplesQualityScores,
                CameraCalibratorMethod.ERROR_OPTIMIZATION);

        // check default values
        assertEquals(CameraCalibratorMethod.ERROR_OPTIMIZATION, calibrator.getMethod());
        assertSame(pattern, calibrator.getPattern());
        assertSame(samples, calibrator.getSamples());
        assertSame(samplesQualityScores, calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION,
                calibrator.getEstimateRadialDistortion());
        assertEquals(CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD, calibrator.getHomographyMethod());
        assertEquals(CameraCalibrator.DEFAULT_IAC_METHOD, calibrator.getImageOfAbsoluteConicMethod());
        assertEquals(calibrator.getIACEstimator().isZeroSkewness(), calibrator.isZeroSkewness());
        assertEquals(calibrator.getIACEstimator().isPrincipalPointAtOrigin(),
                calibrator.isPrincipalPointAtOrigin());
        assertEquals(calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown(),
                calibrator.isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getIACEstimator().getFocalDistanceAspectRatio(),
                calibrator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(CameraCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 0.0);
        assertTrue(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimator().getConfidence(),
                calibrator.getHomographyEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimator().getMaxIterations(),
                calibrator.getHomographyEstimatorMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimator().getConfidence(),
                calibrator.getIACEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimator().getMaxIterations(),
                calibrator.getIACEstimatorMaxIterations());
        assertNull(calibrator.getListener());

        // Force IllegalArgumentException
        final double[] shortSamplesQualityScores = new double[0];
        calibrator = null;
        try {
            calibrator = CameraCalibrator.create(pattern, emptySamples, samplesQualityScores,
                    CameraCalibratorMethod.ERROR_OPTIMIZATION);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = CameraCalibrator.create(pattern, samples, shortSamplesQualityScores,
                    CameraCalibratorMethod.ERROR_OPTIMIZATION);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);

        // Alternating Calibrator
        calibrator = CameraCalibrator.create(pattern, samples, samplesQualityScores,
                CameraCalibratorMethod.ALTERNATING_CALIBRATOR);

        // check default values
        assertEquals(CameraCalibratorMethod.ALTERNATING_CALIBRATOR, calibrator.getMethod());
        assertSame(pattern, calibrator.getPattern());
        assertSame(samples, calibrator.getSamples());
        assertSame(samplesQualityScores, calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION,
                calibrator.getEstimateRadialDistortion());
        assertEquals(CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD, calibrator.getHomographyMethod());
        assertEquals(CameraCalibrator.DEFAULT_IAC_METHOD, calibrator.getImageOfAbsoluteConicMethod());
        assertEquals(calibrator.isZeroSkewness(), calibrator.getIACEstimator().isZeroSkewness());
        assertEquals(calibrator.getIACEstimator().isPrincipalPointAtOrigin(),
                calibrator.isPrincipalPointAtOrigin());
        assertEquals(calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown(),
                calibrator.isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getIACEstimator().getFocalDistanceAspectRatio(),
                calibrator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(CameraCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertTrue(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimator().getConfidence(),
                calibrator.getHomographyEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimator().getMaxIterations(),
                calibrator.getHomographyEstimatorMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimator().getConfidence(),
                calibrator.getIACEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimator().getMaxIterations(),
                calibrator.getIACEstimatorMaxIterations());
        assertNull(calibrator.getListener());

        calibrator = null;
        try {
            calibrator = CameraCalibrator.create(pattern, emptySamples, samplesQualityScores,
                    CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = CameraCalibrator.create(pattern, samples, shortSamplesQualityScores,
                    CameraCalibratorMethod.ALTERNATING_CALIBRATOR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testCreate2() {
        // test create with method

        // Error Optimization
        CameraCalibrator calibrator = CameraCalibrator.create();

        // check default values
        assertEquals(CameraCalibratorMethod.ERROR_OPTIMIZATION, calibrator.getMethod());
        assertNull(calibrator.getPattern());
        assertNull(calibrator.getSamples());
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION,
                calibrator.getEstimateRadialDistortion());
        assertEquals(CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD, calibrator.getHomographyMethod());
        assertEquals(CameraCalibrator.DEFAULT_IAC_METHOD, calibrator.getImageOfAbsoluteConicMethod());
        assertEquals(calibrator.getIACEstimator().isZeroSkewness(), calibrator.isZeroSkewness());
        assertEquals(calibrator.getIACEstimator().isPrincipalPointAtOrigin(),
                calibrator.isPrincipalPointAtOrigin());
        assertEquals(calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown(),
                calibrator.isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getIACEstimator().getFocalDistanceAspectRatio(),
                calibrator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(CameraCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 0.0);
        assertFalse(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimator().getConfidence(),
                calibrator.getHomographyEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimator().getMaxIterations(),
                calibrator.getHomographyEstimatorMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimator().getConfidence(),
                calibrator.getIACEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimator().getMaxIterations(),
                calibrator.getIACEstimatorMaxIterations());
        assertNull(calibrator.getListener());

        // test create with pattern, samples and method
        final Pattern2D pattern = Pattern2D.create(Pattern2DType.CIRCLES);
        final List<CameraCalibratorSample> samples = new ArrayList<>();
        samples.add(new CameraCalibratorSample());

        // Error Optimization
        calibrator = CameraCalibrator.create(pattern, samples);

        // check default values
        assertEquals(CameraCalibratorMethod.ERROR_OPTIMIZATION, calibrator.getMethod());
        assertSame(pattern, calibrator.getPattern());
        assertSame(samples, calibrator.getSamples());
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION,
                calibrator.getEstimateRadialDistortion());
        assertEquals(CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD, calibrator.getHomographyMethod());
        assertEquals(CameraCalibrator.DEFAULT_IAC_METHOD, calibrator.getImageOfAbsoluteConicMethod());
        assertEquals(calibrator.getIACEstimator().isZeroSkewness(), calibrator.isZeroSkewness());
        assertEquals(calibrator.getIACEstimator().isPrincipalPointAtOrigin(),
                calibrator.isPrincipalPointAtOrigin());
        assertEquals(calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown(),
                calibrator.isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getIACEstimator().getFocalDistanceAspectRatio(),
                calibrator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(CameraCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 0.0);
        assertTrue(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimator().getConfidence(),
                calibrator.getHomographyEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimator().getMaxIterations(),
                calibrator.getHomographyEstimatorMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimator().getConfidence(),
                calibrator.getIACEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimator().getMaxIterations(),
                calibrator.getIACEstimatorMaxIterations());
        assertNull(calibrator.getListener());

        // Force IllegalArgumentException
        final List<CameraCalibratorSample> emptySamples = new ArrayList<>();
        calibrator = null;
        try {
            calibrator = CameraCalibrator.create(pattern, emptySamples,
                    CameraCalibratorMethod.ERROR_OPTIMIZATION);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);

        // test create with pattern, samples, quality scores and method
        final double[] samplesQualityScores = new double[1];

        // Error Optimization
        calibrator = CameraCalibrator.create(pattern, samples, samplesQualityScores);

        // check default values
        assertEquals(CameraCalibratorMethod.ERROR_OPTIMIZATION, calibrator.getMethod());
        assertSame(pattern, calibrator.getPattern());
        assertSame(samples, calibrator.getSamples());
        assertSame(samplesQualityScores, calibrator.getSamplesQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION,
                calibrator.getEstimateRadialDistortion());
        assertEquals(CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD, calibrator.getHomographyMethod());
        assertEquals(CameraCalibrator.DEFAULT_IAC_METHOD, calibrator.getImageOfAbsoluteConicMethod());
        assertEquals(calibrator.getIACEstimator().isZeroSkewness(),
                calibrator.isZeroSkewness());
        assertEquals(calibrator.getIACEstimator().isPrincipalPointAtOrigin(),
                calibrator.isPrincipalPointAtOrigin());
        assertEquals(calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown(),
                calibrator.isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getIACEstimator().getFocalDistanceAspectRatio(),
                calibrator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(CameraCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 0.0);
        assertTrue(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimator().getConfidence(),
                calibrator.getHomographyEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getHomographyEstimator().getMaxIterations(),
                calibrator.getHomographyEstimatorMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimator().getConfidence(),
                calibrator.getIACEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimator().getMaxIterations(),
                calibrator.getIACEstimatorMaxIterations());
        assertNull(calibrator.getListener());

        // Force IllegalArgumentException
        final double[] shortSamplesQualityScores = new double[0];
        calibrator = null;
        try {
            calibrator = CameraCalibrator.create(pattern, emptySamples, samplesQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = CameraCalibrator.create(pattern, samples, shortSamplesQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }
}
