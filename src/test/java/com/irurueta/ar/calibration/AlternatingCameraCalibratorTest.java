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

import com.irurueta.ar.calibration.estimators.*;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.jupiter.api.Assertions.*;

class AlternatingCameraCalibratorTest implements CameraCalibratorListener {

    private static final double MIN_FOCAL_LENGTH = 3.0;
    private static final double MAX_FOCAL_LENGTH = 10.0;

    private static final double MIN_ANGLE_DEGREES = -10.0;
    private static final double MAX_ANGLE_DEGREES = 10.0;

    private static final int INHOM_3D_COORDS = 3;

    private static final double MIN_RANDOM_VALUE = -50.0;
    private static final double MAX_RANDOM_VALUE = -10.0;

    private static final double ABSOLUTE_ERROR = 5e-6;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 5e-1;
    private static final double ULTRA_LARGE_ABSOLUTE_ERROR = 3.0;

    private static final int MIN_NUM_SAMPLES = 100;
    private static final int MAX_NUM_SAMPLES = 500;

    private static final int TIMES = 10;

    private static final double STD_ERROR = 10.0;
    private static final int PERCENTAGE_OUTLIERS = 20;

    private static final double MIN_DISTORTION_PARAM_VALUE = -1e-3;
    private static final double MAX_DISTORTION_PARAM_VALUE = 1e-3;

    private static final double REAL_IAC_THRESHOLD = 1e-8;

    private int calibrateStart;
    private int calibrateEnd;
    private int calibrateProgressChange;
    private int intrinsicParametersEstimationStarts;
    private int intrinsicParametersEstimationEnds;
    private int radialDistortionEstimationStarts;
    private int radialDistortionEstimationEnds;

    @Test
    void testConstructor() {
        // test constructor without arguments
        var calibrator = new AlternatingCameraCalibrator();

        // check default values
        assertEquals(AlternatingCameraCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertEquals(AlternatingCameraCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, calibrator.getConvergenceThreshold(),
                0.0);
        assertEquals(AlternatingCameraCalibrator.DEFAULT_RADIAL_DISTORTION_METHOD, calibrator.getDistortionMethod());
        assertEquals(AlternatingCameraCalibrator.DEFAULT_RADIAL_DISTORTION_METHOD,
                calibrator.getDistortionEstimator().getMethod());
        assertTrue(calibrator.getDistortionEstimatorThreshold() > 0);
        assertEquals(calibrator.getDistortionEstimator().getConfidence(), calibrator.getDistortionEstimatorConfidence(),
                0.0);
        assertEquals(calibrator.getDistortionEstimator().getMaxIterations(),
                calibrator.getDistortionEstimatorMaxIterations());
        assertEquals(CameraCalibratorMethod.ALTERNATING_CALIBRATOR, calibrator.getMethod());
        assertNull(calibrator.getPattern());
        assertNull(calibrator.getSamples());
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getHomographyQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION, calibrator.getEstimateRadialDistortion());
        assertEquals(CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD, calibrator.getHomographyMethod());
        assertEquals(CameraCalibrator.DEFAULT_IAC_METHOD, calibrator.getImageOfAbsoluteConicMethod());
        assertEquals(calibrator.getIACEstimator().isZeroSkewness(), calibrator.isZeroSkewness());
        assertEquals(calibrator.getIACEstimator().isPrincipalPointAtOrigin(), calibrator.isPrincipalPointAtOrigin());
        assertEquals(calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown(),
                calibrator.isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getIACEstimator().getFocalDistanceAspectRatio(),
                calibrator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(CameraCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 0.0);
        assertFalse(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimator().getConfidence(), calibrator.getHomographyEstimatorConfidence(),
                0.0);
        assertEquals(calibrator.getHomographyEstimator().getMaxIterations(),
                calibrator.getHomographyEstimatorMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimator().getConfidence(), calibrator.getIACEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimator().getMaxIterations(), calibrator.getIACEstimatorMaxIterations());
        assertNull(calibrator.getListener());

        // test constructor with pattern and samples
        final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);
        final var samples = new ArrayList<CameraCalibratorSample>();
        samples.add(new CameraCalibratorSample());

        assertEquals(1, calibrator.getIACEstimator().getMinNumberOfRequiredHomographies());

        calibrator = new AlternatingCameraCalibrator(pattern, samples);

        // check default values
        assertEquals(AlternatingCameraCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertEquals(AlternatingCameraCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, calibrator.getConvergenceThreshold(),
                0.0);
        assertEquals(AlternatingCameraCalibrator.DEFAULT_RADIAL_DISTORTION_METHOD, calibrator.getDistortionMethod());
        assertEquals(AlternatingCameraCalibrator.DEFAULT_RADIAL_DISTORTION_METHOD,
                calibrator.getDistortionEstimator().getMethod());
        assertTrue(calibrator.getDistortionEstimatorThreshold() > 0);
        assertEquals(calibrator.getDistortionEstimator().getConfidence(), calibrator.getDistortionEstimatorConfidence(),
                0.0);
        assertEquals(calibrator.getDistortionEstimator().getMaxIterations(),
                calibrator.getDistortionEstimatorMaxIterations());
        assertEquals(CameraCalibratorMethod.ALTERNATING_CALIBRATOR, calibrator.getMethod());
        assertSame(pattern, calibrator.getPattern());
        assertSame(samples, calibrator.getSamples());
        assertNull(calibrator.getSamplesQualityScores());
        assertNull(calibrator.getHomographyQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION, calibrator.getEstimateRadialDistortion());
        assertEquals(CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD, calibrator.getHomographyMethod());
        assertEquals(CameraCalibrator.DEFAULT_IAC_METHOD, calibrator.getImageOfAbsoluteConicMethod());
        assertEquals(calibrator.getIACEstimator().isZeroSkewness(), calibrator.isZeroSkewness());
        assertEquals(calibrator.getIACEstimator().isPrincipalPointAtOrigin(), calibrator.isPrincipalPointAtOrigin());
        assertEquals(calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown(),
                calibrator.isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getIACEstimator().getFocalDistanceAspectRatio(),
                calibrator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(CameraCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 0.0);
        assertTrue(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimator().getConfidence(), calibrator.getHomographyEstimatorConfidence(),
                0.0);
        assertEquals(calibrator.getHomographyEstimator().getMaxIterations(),
                calibrator.getHomographyEstimatorMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimator().getConfidence(), calibrator.getIACEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimator().getMaxIterations(), calibrator.getIACEstimatorMaxIterations());
        assertNull(calibrator.getListener());

        // Force IllegalArgumentException
        final var emptySamples = new ArrayList<CameraCalibratorSample>();
        assertThrows(IllegalArgumentException.class, () -> new AlternatingCameraCalibrator(pattern, emptySamples));

        // test constructor with pattern, samples and quality scores
        final var samplesQualityScores = new double[1];
        calibrator = new AlternatingCameraCalibrator(pattern, samples, samplesQualityScores);

        // check default values
        assertEquals(AlternatingCameraCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertEquals(AlternatingCameraCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, calibrator.getConvergenceThreshold(),
                0.0);
        assertEquals(AlternatingCameraCalibrator.DEFAULT_RADIAL_DISTORTION_METHOD, calibrator.getDistortionMethod());
        assertEquals(AlternatingCameraCalibrator.DEFAULT_RADIAL_DISTORTION_METHOD,
                calibrator.getDistortionEstimator().getMethod());
        assertTrue(calibrator.getDistortionEstimatorThreshold() > 0);
        assertEquals(calibrator.getDistortionEstimator().getConfidence(), calibrator.getDistortionEstimatorConfidence(),
                0.0);
        assertEquals(calibrator.getDistortionEstimator().getMaxIterations(),
                calibrator.getDistortionEstimatorMaxIterations());
        assertEquals(CameraCalibratorMethod.ALTERNATING_CALIBRATOR, calibrator.getMethod());
        assertSame(pattern, calibrator.getPattern());
        assertSame(samples, calibrator.getSamples());
        assertSame(samplesQualityScores, calibrator.getSamplesQualityScores());
        assertNull(calibrator.getHomographyQualityScores());
        assertNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNull(calibrator.getEstimatedIntrinsicParameters());
        assertNull(calibrator.getDistortion());
        assertEquals(CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION, calibrator.getEstimateRadialDistortion());
        assertEquals(CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD, calibrator.getHomographyMethod());
        assertEquals(CameraCalibrator.DEFAULT_IAC_METHOD, calibrator.getImageOfAbsoluteConicMethod());
        assertEquals(calibrator.getIACEstimator().isZeroSkewness(), calibrator.isZeroSkewness());
        assertEquals(calibrator.getIACEstimator().isPrincipalPointAtOrigin(), calibrator.isPrincipalPointAtOrigin());
        assertEquals(calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown(),
                calibrator.isFocalDistanceAspectRatioKnown());
        assertEquals(calibrator.getIACEstimator().getFocalDistanceAspectRatio(),
                calibrator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(calibrator.isLocked());
        assertEquals(CameraCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 0.0);
        assertTrue(calibrator.isReady());
        assertTrue(calibrator.getHomographyEstimatorThreshold() > 0);
        assertEquals(calibrator.getHomographyEstimator().getConfidence(), calibrator.getHomographyEstimatorConfidence(),
                0.0);
        assertEquals(calibrator.getHomographyEstimator().getMaxIterations(),
                calibrator.getHomographyEstimatorMaxIterations());
        assertTrue(calibrator.getIACEstimatorThreshold() > 0);
        assertEquals(calibrator.getIACEstimator().getConfidence(), calibrator.getIACEstimatorConfidence(), 0.0);
        assertEquals(calibrator.getIACEstimator().getMaxIterations(), calibrator.getIACEstimatorMaxIterations());
        assertNull(calibrator.getListener());

        // Force IllegalArgumentException
        final var shortSamplesQualityScores = new double[0];
        assertThrows(IllegalArgumentException.class,
                () -> new AlternatingCameraCalibrator(pattern, emptySamples, samplesQualityScores));
        assertThrows(IllegalArgumentException.class,
                () -> new AlternatingCameraCalibrator(pattern, samples, shortSamplesQualityScores));
    }

    @Test
    void testGetSetPattern() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertNull(calibrator.getPattern());

        // set new value
        final var pattern = Pattern2D.create(Pattern2DType.QR);
        calibrator.setPattern(pattern);

        // check correctness
        assertSame(pattern, calibrator.getPattern());
    }

    @Test
    void testGetSetSamples() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertNull(calibrator.getSamples());

        // set new value
        final var samples = new ArrayList<CameraCalibratorSample>();
        samples.add(new CameraCalibratorSample());

        calibrator.setSamples(samples);

        // check correctness
        assertSame(samples, calibrator.getSamples());

        // Force IllegalArgumentException
        final var emptySamples = new ArrayList<CameraCalibratorSample>();
        assertThrows(IllegalArgumentException.class, () -> calibrator.setSamples(emptySamples));
    }

    @Test
    void testGetSetSamplesQualityScores() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertNull(calibrator.getSamplesQualityScores());

        // set new value
        final var sampleQualityScores = new double[1];
        calibrator.setSamplesQualityScores(sampleQualityScores);

        // check correctness
        assertSame(sampleQualityScores, calibrator.getSamplesQualityScores());

        // Force IllegalArgumentException
        final var emptyQualityScores = new double[0];
        assertThrows(IllegalArgumentException.class, () -> calibrator.setSamplesQualityScores(emptyQualityScores));
    }

    @Test
    void testGetSetEstimateRadialDistortion() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertEquals(CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION, calibrator.getEstimateRadialDistortion());

        // set new value
        calibrator.setEstimateRadialDistortion(!CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION);

        // check correctness
        assertEquals(!CameraCalibrator.DEFAULT_ESTIMATE_RADIAL_DISTORTION, calibrator.getEstimateRadialDistortion());
    }

    @Test
    void testGetSetHomographyMethod() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertEquals(CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD, calibrator.getHomographyMethod());
        assertEquals(CameraCalibrator.DEFAULT_HOMOGRAPHY_METHOD, calibrator.getHomographyEstimator().getMethod());

        final var threshold = calibrator.getHomographyEstimatorThreshold();
        final var confidence = calibrator.getHomographyEstimatorConfidence();
        final var maxIterations = calibrator.getHomographyEstimatorMaxIterations();

        // set new value
        calibrator.setHomographyMethod(RobustEstimatorMethod.RANSAC);

        // check correctness
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getHomographyMethod());
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getHomographyEstimator().getMethod());

        assertEquals(threshold, calibrator.getHomographyEstimatorThreshold(), 0.0);
        assertEquals(confidence, calibrator.getHomographyEstimatorConfidence(), 0.0);
        assertEquals(maxIterations, calibrator.getHomographyEstimatorMaxIterations());
    }

    @Test
    void testGetSetImageOfAbsoluteConicMethod() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertEquals(CameraCalibrator.DEFAULT_IAC_METHOD, calibrator.getImageOfAbsoluteConicMethod());
        assertEquals(CameraCalibrator.DEFAULT_IAC_METHOD, calibrator.getIACEstimator().getMethod());

        final var threshold = calibrator.getIACEstimatorThreshold();
        final var confidence = calibrator.getIACEstimatorConfidence();
        final var maxIterations = calibrator.getIACEstimatorMaxIterations();
        final var zeroSkewness = calibrator.isZeroSkewness();
        final var principalPointAtOrigin = calibrator.isPrincipalPointAtOrigin();
        final var focalDistanceAspectRatioKnown = calibrator.isFocalDistanceAspectRatioKnown();
        final var focalDistanceAspectRatio = calibrator.getFocalDistanceAspectRatio();

        // set new value
        calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.RANSAC);

        // check correctness
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getImageOfAbsoluteConicMethod());
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getIACEstimator().getMethod());

        assertEquals(threshold, calibrator.getIACEstimatorThreshold(), 0.0);
        assertEquals(confidence, calibrator.getIACEstimatorConfidence(), 0.0);
        assertEquals(maxIterations, calibrator.getIACEstimatorMaxIterations());
        assertEquals(zeroSkewness, calibrator.isZeroSkewness());
        assertEquals(principalPointAtOrigin, calibrator.isPrincipalPointAtOrigin());
        assertEquals(focalDistanceAspectRatioKnown, calibrator.isFocalDistanceAspectRatioKnown());
        assertEquals(focalDistanceAspectRatio, calibrator.getFocalDistanceAspectRatio(), 0.0);
    }

    @Test
    void testIsSetZeroSkewness() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        final var zeroSkewness = calibrator.isZeroSkewness();
        assertEquals(zeroSkewness, calibrator.getIACEstimator().isZeroSkewness());

        // set new value
        calibrator.setZeroSkewness(!zeroSkewness);

        // check correctness
        assertEquals(!zeroSkewness, calibrator.isZeroSkewness());
        assertEquals(!zeroSkewness, calibrator.getIACEstimator().isZeroSkewness());
    }

    @Test
    void testIsSetPrincipalPointAtOrigin() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        final var principalPointAtOrigin = calibrator.isPrincipalPointAtOrigin();
        assertEquals(principalPointAtOrigin, calibrator.getIACEstimator().isPrincipalPointAtOrigin());

        // set new value
        calibrator.setPrincipalPointAtOrigin(!principalPointAtOrigin);

        // check correctness
        assertEquals(!principalPointAtOrigin, calibrator.isPrincipalPointAtOrigin());
        assertEquals(!principalPointAtOrigin, calibrator.getIACEstimator().isPrincipalPointAtOrigin());
    }

    @Test
    void testIsSetFocalDistanceAspectRatioKnown() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        final var focalDistanceAspectRatioKnown = calibrator.isFocalDistanceAspectRatioKnown();
        assertEquals(focalDistanceAspectRatioKnown, calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown());

        // set new value
        calibrator.setFocalDistanceAspectRatioKnown(!focalDistanceAspectRatioKnown);

        // check correctness
        assertEquals(!focalDistanceAspectRatioKnown, calibrator.isFocalDistanceAspectRatioKnown());
        assertEquals(!focalDistanceAspectRatioKnown, calibrator.getIACEstimator().isFocalDistanceAspectRatioKnown());
    }

    @Test
    void testGetSetFocalDistanceAspectRatio() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        final var focalDistanceAspectRatio = calibrator.getFocalDistanceAspectRatio();
        assertEquals(calibrator.getIACEstimator().getFocalDistanceAspectRatio(), focalDistanceAspectRatio, 0.0);

        // set new value
        calibrator.setFocalDistanceAspectRatio(0.5);

        // check correctness
        assertEquals(0.5, calibrator.getFocalDistanceAspectRatio(), 0.0);
        assertEquals(0.5, calibrator.getIACEstimator().getFocalDistanceAspectRatio(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setFocalDistanceAspectRatio(0.0));
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertEquals(CameraCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 0.0);

        // set new value
        calibrator.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5f, calibrator.getProgressDelta(), 0.0);
    }

    @Test
    void testGetSetHomographyEstimatorThreshold() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // set RANSAC homography method
        calibrator.setHomographyMethod(RobustEstimatorMethod.RANSAC);

        // check default value
        final var threshold = calibrator.getHomographyEstimatorThreshold();
        assertEquals(threshold, ((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) calibrator
                .getHomographyEstimator()).getThreshold(), 0.0);

        // set new value
        calibrator.setHomographyEstimatorThreshold(1.0);

        // check correctness
        assertEquals(1.0, calibrator.getHomographyEstimatorThreshold(), 0.0);
        assertEquals(1.0, ((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) calibrator
                .getHomographyEstimator()).getThreshold(), 0.0);

        // set LMedS homography method
        calibrator.setHomographyMethod(RobustEstimatorMethod.LMEDS);

        // check default value
        assertEquals(1.0, calibrator.getHomographyEstimatorThreshold(), 0.0);
        assertEquals(1.0, ((LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                calibrator.getHomographyEstimator()).getStopThreshold(), 0.0);

        // set new value
        calibrator.setHomographyEstimatorThreshold(2.0);

        // check correctness
        assertEquals(2.0, calibrator.getHomographyEstimatorThreshold(), 0.0);
        assertEquals(2.0, ((LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator) calibrator
                .getHomographyEstimator()).getStopThreshold(), 0.0);

        // set MSAC homography method
        calibrator.setHomographyMethod(RobustEstimatorMethod.MSAC);

        // check default value
        assertEquals(2.0, calibrator.getHomographyEstimatorThreshold(), 0.0);
        assertEquals(2.0, ((MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) calibrator
                .getHomographyEstimator()).getThreshold(), 0.0);

        // set new value
        calibrator.setHomographyEstimatorThreshold(3.0);

        // check correctness
        assertEquals(3.0, calibrator.getHomographyEstimatorThreshold(), 0.0);
        assertEquals(3.0, ((MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) calibrator
                .getHomographyEstimator()).getThreshold(), 0.0);

        // set PROSAC homography method
        calibrator.setHomographyMethod(RobustEstimatorMethod.PROSAC);

        // check default value
        assertEquals(3.0, calibrator.getHomographyEstimatorThreshold(), 0.0);
        assertEquals(3.0, ((PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) calibrator
                .getHomographyEstimator()).getThreshold(), 0.0);

        // set new value
        calibrator.setHomographyEstimatorThreshold(4.0);

        // check correctness
        assertEquals(4.0, calibrator.getHomographyEstimatorThreshold(), 0.0);
        assertEquals(4.0, ((PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) calibrator
                .getHomographyEstimator()).getThreshold(), 0.0);

        // set PROMedS homography method
        calibrator.setHomographyMethod(RobustEstimatorMethod.PROMEDS);

        // check default value
        assertEquals(4.0, calibrator.getHomographyEstimatorThreshold(), 0.0);
        assertEquals(4.0, ((PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator) calibrator
                .getHomographyEstimator()).getStopThreshold(), 0.0);

        // set new value
        calibrator.setHomographyEstimatorThreshold(5.0);

        // check correctness
        assertEquals(5.0, calibrator.getHomographyEstimatorThreshold(), 0.0);
        assertEquals(5.0, ((PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator) calibrator
                .getHomographyEstimator()).getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setHomographyEstimatorThreshold(0.0));
    }

    @Test
    void testGetSetHomographyEstimatorConfidence() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertEquals(calibrator.getHomographyEstimatorConfidence(), calibrator.getHomographyEstimator().getConfidence(),
                0.0);

        // set new value
        calibrator.setHomographyEstimatorConfidence(0.5);

        // check correctness
        assertEquals(0.5, calibrator.getHomographyEstimatorConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setHomographyEstimatorConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setHomographyEstimatorConfidence(2.0));
    }

    @Test
    void testGetSetHomographyEstimatorMaxIterations() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertEquals(calibrator.getHomographyEstimator().getMaxIterations(),
                calibrator.getHomographyEstimatorMaxIterations());

        // set new value
        calibrator.setHomographyEstimatorMaxIterations(10);

        // check correctness
        assertEquals(10, calibrator.getHomographyEstimatorMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setHomographyEstimatorMaxIterations(0));
    }

    @Test
    void testGetSetIACEstimatorThreshold() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // set RANSAC homography method
        calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.RANSAC);

        // check default value
        final var threshold = calibrator.getIACEstimatorThreshold();
        assertEquals(threshold, ((RANSACImageOfAbsoluteConicRobustEstimator) calibrator.getIACEstimator())
                .getThreshold(), 0.0);

        // set new value
        calibrator.setIACEstimatorThreshold(1.0);

        // check correctness
        assertEquals(1.0, calibrator.getIACEstimatorThreshold(), 0.0);
        assertEquals(1.0, ((RANSACImageOfAbsoluteConicRobustEstimator) calibrator.getIACEstimator())
                        .getThreshold(), 0.0);

        // set LMedS homography method
        calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.LMEDS);

        // check default value
        assertEquals(1.0, calibrator.getIACEstimatorThreshold(), 0.0);
        assertEquals(1.0, ((LMedSImageOfAbsoluteConicRobustEstimator) calibrator.getIACEstimator())
                .getStopThreshold(), 0.0);

        // set new value
        calibrator.setIACEstimatorThreshold(2.0);

        // check correctness
        assertEquals(2.0, calibrator.getIACEstimatorThreshold(), 0.0);
        assertEquals(2.0, ((LMedSImageOfAbsoluteConicRobustEstimator) calibrator.getIACEstimator())
                .getStopThreshold(), 0.0);

        // set MSAC homography method
        calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.MSAC);

        // check default value
        assertEquals(2.0, calibrator.getIACEstimatorThreshold(), 0.0);
        assertEquals(2.0, ((MSACImageOfAbsoluteConicRobustEstimator) calibrator.getIACEstimator())
                        .getThreshold(), 0.0);

        // set new value
        calibrator.setIACEstimatorThreshold(3.0);

        // check correctness
        assertEquals(3.0, calibrator.getIACEstimatorThreshold(), 0.0);
        assertEquals(3.0, ((MSACImageOfAbsoluteConicRobustEstimator) calibrator.getIACEstimator())
                        .getThreshold(), 0.0);

        // set PROSAC homography method
        calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.PROSAC);

        // check default value
        assertEquals(3.0, calibrator.getIACEstimatorThreshold(), 0.0);
        assertEquals(3.0, ((PROSACImageOfAbsoluteConicRobustEstimator) calibrator.getIACEstimator())
                        .getThreshold(), 0.0);

        // set new value
        calibrator.setIACEstimatorThreshold(4.0);

        // check correctness
        assertEquals(4.0, calibrator.getIACEstimatorThreshold(), 0.0);
        assertEquals(4.0, ((PROSACImageOfAbsoluteConicRobustEstimator) calibrator.getIACEstimator())
                        .getThreshold(), 0.0);

        // set PROMedS homography method
        calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.PROMEDS);

        // check default value
        assertEquals(4.0, calibrator.getIACEstimatorThreshold(), 0.0);
        assertEquals(4.0, ((PROMedSImageOfAbsoluteConicRobustEstimator) calibrator.getIACEstimator())
                .getStopThreshold(), 0.0);

        // set new value
        calibrator.setIACEstimatorThreshold(5.0);

        // check correctness
        assertEquals(5.0, calibrator.getIACEstimatorThreshold(), 0.0);
        assertEquals(5.0, ((PROMedSImageOfAbsoluteConicRobustEstimator) calibrator.getIACEstimator())
                .getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setIACEstimatorThreshold(0.0));
    }

    @Test
    void testGetSetIACEstimatorConfidence() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertEquals(calibrator.getIACEstimatorConfidence(), calibrator.getIACEstimator().getConfidence(), 0.0);

        // set new value
        calibrator.setIACEstimatorConfidence(0.5);

        // check correctness
        assertEquals(0.5, calibrator.getIACEstimatorConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setIACEstimatorConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setIACEstimatorConfidence(2.0));
    }

    @Test
    void testGetSetIACEstimatorMaxIterations() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertEquals(calibrator.getIACEstimator().getMaxIterations(), calibrator.getIACEstimatorMaxIterations());

        // set new value
        calibrator.setIACEstimatorMaxIterations(10);

        // check correctness
        assertEquals(10, calibrator.getIACEstimatorMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setIACEstimatorMaxIterations(0));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check correctness
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertEquals(AlternatingCameraCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());

        // set new value
        calibrator.setMaxIterations(5);

        // check correctness
        assertEquals(5, calibrator.getMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setMaxIterations(0));
    }

    @Test
    void testGetSetConvergenceThreshold() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertEquals(AlternatingCameraCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, calibrator.getConvergenceThreshold(),
                0.0);

        // set new value
        calibrator.setConvergenceThreshold(1.0);

        // check correctness
        assertEquals(1.0, calibrator.getConvergenceThreshold(), 0.0);
    }

    @Test
    void testGetSetDistortionMethod() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertEquals(AlternatingCameraCalibrator.DEFAULT_RADIAL_DISTORTION_METHOD, calibrator.getDistortionMethod());
        assertEquals(AlternatingCameraCalibrator.DEFAULT_RADIAL_DISTORTION_METHOD,
                calibrator.getDistortionEstimator().getMethod());

        final var threshold = calibrator.getDistortionEstimatorThreshold();
        final var confidence = calibrator.getDistortionEstimatorConfidence();
        final var maxIterations = calibrator.getDistortionEstimatorMaxIterations();

        // set new value
        calibrator.setDistortionMethod(RobustEstimatorMethod.LMEDS);

        // check correctness
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getDistortionMethod());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getDistortionEstimator().getMethod());

        assertEquals(threshold, calibrator.getDistortionEstimatorThreshold(), 0.0);
        assertEquals(confidence, calibrator.getDistortionEstimatorConfidence(), 0.0);
        assertEquals(maxIterations, calibrator.getDistortionEstimatorMaxIterations());
    }

    @Test
    void testGetSetDistortionEstimatorThreshold() throws LockedException {
        final var calibrator = new AlternatingCameraCalibrator();

        // set RANSAC method
        calibrator.setDistortionMethod(RobustEstimatorMethod.RANSAC);

        // check default value
        final var threshold = calibrator.getDistortionEstimatorThreshold();
        assertEquals(threshold, ((RANSACRadialDistortionRobustEstimator) calibrator.getDistortionEstimator())
                .getThreshold(), 0.0);

        // set new value
        calibrator.setDistortionEstimatorThreshold(1.0);

        // check correctness
        assertEquals(1.0, calibrator.getDistortionEstimatorThreshold(), 0.0);
        assertEquals(1.0, ((RANSACRadialDistortionRobustEstimator) calibrator.getDistortionEstimator())
                .getThreshold(), 0.0);

        // set LMedS method
        calibrator.setDistortionMethod(RobustEstimatorMethod.LMEDS);

        // check default value
        assertEquals(1.0, calibrator.getDistortionEstimatorThreshold(), 0.0);
        assertEquals(1.0, ((LMedSRadialDistortionRobustEstimator) calibrator.getDistortionEstimator())
                .getStopThreshold(), 0.0);

        // set new value
        calibrator.setDistortionEstimatorThreshold(2.0);

        // check correctness
        assertEquals(2.0, calibrator.getDistortionEstimatorThreshold(), 0.0);
        assertEquals(2.0, ((LMedSRadialDistortionRobustEstimator) calibrator.getDistortionEstimator())
                .getStopThreshold(), 0.0);

        // set MSAC method
        calibrator.setDistortionMethod(RobustEstimatorMethod.MSAC);

        // check default value
        assertEquals(2.0, calibrator.getDistortionEstimatorThreshold(), 0.0);
        assertEquals(2.0, ((MSACRadialDistortionRobustEstimator) calibrator.getDistortionEstimator())
                .getThreshold(), 0.0);

        // set new value
        calibrator.setDistortionEstimatorThreshold(3.0);

        // check correctness
        assertEquals(3.0, calibrator.getDistortionEstimatorThreshold(), 0.0);
        assertEquals(3.0, ((MSACRadialDistortionRobustEstimator) calibrator.getDistortionEstimator())
                .getThreshold(), 0.0);

        // set PROSAC method
        calibrator.setDistortionMethod(RobustEstimatorMethod.PROSAC);

        // check default value
        assertEquals(3.0, calibrator.getDistortionEstimatorThreshold(), 0.0);
        assertEquals(3.0, ((PROSACRadialDistortionRobustEstimator) calibrator.getDistortionEstimator())
                .getThreshold(), 0.0);

        // set new value
        calibrator.setDistortionEstimatorThreshold(4.0);

        // check correctness
        assertEquals(4.0, calibrator.getDistortionEstimatorThreshold(), 0.0);
        assertEquals(4.0, ((PROSACRadialDistortionRobustEstimator) calibrator.getDistortionEstimator())
                .getThreshold(), 0.0);

        // set PROMedS method
        calibrator.setDistortionMethod(RobustEstimatorMethod.PROMEDS);

        // check default value
        assertEquals(4.0, calibrator.getDistortionEstimatorThreshold(), 0.0);
        assertEquals(4.0, ((PROMedSRadialDistortionRobustEstimator) calibrator.getDistortionEstimator())
                .getStopThreshold(), 0.0);

        // set new value
        calibrator.setDistortionEstimatorThreshold(5.0);

        // check correctness
        assertEquals(5.0, calibrator.getDistortionEstimatorThreshold(), 0.0);
        assertEquals(5.0, ((PROMedSRadialDistortionRobustEstimator) calibrator.getDistortionEstimator())
                .getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setDistortionEstimatorThreshold(0.0));
    }

    @Test
    void testGetSetDistortionEstimatorConfidence() throws LockedException {

        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertEquals(calibrator.getDistortionEstimatorConfidence(), calibrator.getDistortionEstimator().getConfidence(),
                0.0);

        // set new value
        calibrator.setDistortionEstimatorConfidence(0.5);

        // check correctness
        assertEquals(0.5, calibrator.getDistortionEstimatorConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setDistortionEstimatorConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setDistortionEstimatorConfidence(2.0));
    }

    @Test
    void testGetSetDistortionEstimatorMaxIterations() throws LockedException {

        final var calibrator = new AlternatingCameraCalibrator();

        // check default value
        assertEquals(calibrator.getDistortionEstimator().getMaxIterations(),
                calibrator.getDistortionEstimatorMaxIterations());

        // set new value
        calibrator.setDistortionEstimatorMaxIterations(10);

        // check correctness
        assertEquals(10, calibrator.getDistortionEstimatorMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setDistortionEstimatorMaxIterations(0));
    }

    @Test
    void testCalibrateCirclesPatternNoDistortion() throws LockedException, CalibrationException, NotReadyException {

        final var startTime = System.currentTimeMillis();

        final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);

        final var patternPoints = pattern.getIdealPoints();

        // assume that pattern points are located on a 3D plane
        // (for instance Z = 0), but can be really any plane
        final var points3D = new ArrayList<Point3D>();
        for (final var patternPoint : patternPoints) {
            points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0, 1.0));
        }

        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create samples (each sample has one pinhole camera associated and
            // its corresponding sampled markers)
            final var numSamples = randomizer.nextInt(MIN_NUM_SAMPLES, MAX_NUM_SAMPLES);
            final var samples = new ArrayList<CameraCalibratorSample>();
            for (var i = 0; i < numSamples; i++) {
                // create random camera
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points
                final var projectedPatternPoints = camera.project(points3D);

                // add random noise to projected points with a certain outlier
                // proportion

                samples.add(new CameraCalibratorSample(projectedPatternPoints));
            }

            // create calibrator
            final var calibrator = new AlternatingCameraCalibrator(pattern, samples);
            calibrator.setListener(this);
            calibrator.setEstimateRadialDistortion(false);
            calibrator.setHomographyMethod(RobustEstimatorMethod.LMEDS);
            calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.PROMEDS);

            // check correctness
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isLocked());

            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateProgressChange);
            assertEquals(0, intrinsicParametersEstimationStarts);
            assertEquals(0, intrinsicParametersEstimationEnds);
            assertEquals(0, radialDistortionEstimationStarts);
            assertEquals(0, radialDistortionEstimationEnds);

            // calibrate
            calibrator.calibrate();

            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateProgressChange >= 0);
            assertTrue(intrinsicParametersEstimationStarts > 0);
            assertTrue(intrinsicParametersEstimationEnds > 0);
            assertEquals(0, radialDistortionEstimationStarts);
            assertEquals(0, radialDistortionEstimationEnds);
            reset();

            // check correctness
            assertFalse(calibrator.isLocked());

            // check intrinsic parameters
            assertNotNull(calibrator.getHomographyQualityScores());
            assertNotNull(calibrator.getEstimatedImageOfAbsoluteConic());
            assertNotNull(calibrator.getEstimatedIntrinsicParameters());
            final var intrinsic2 = calibrator.getEstimatedIntrinsicParameters();

            assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic.getSkewness(), 0.0);
            assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
            assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic2.getHorizontalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic2.getVerticalPrincipalPoint(), 0.0);

            horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                    - intrinsic2.getHorizontalFocalLength());
            verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                    - intrinsic2.getVerticalFocalLength());
            skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
            horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                    - intrinsic2.getHorizontalPrincipalPoint());
            verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                    - intrinsic2.getVerticalPrincipalPoint());

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

            // check radial distortion
            assertNull(calibrator.getDistortion());

            // check camera poses were not estimated (because distortion was not
            // estimated)
            var estimatedCams = 0;
            for (var i = 0; i < numSamples; i++) {
                if (samples.get(i).getHomography() != null) {
                    assertNull(samples.get(i).getCamera());
                    assertNull(samples.get(i).getCameraCenter());
                    assertNull(samples.get(i).getRotation());
                    if (samples.get(i).getCamera() != null) {
                        estimatedCams++;
                    }
                }
            }

            assertEquals(0, estimatedCams);
        }

        final var endTime = System.currentTimeMillis();

        avgHorizontalFocalDistanceError /= TIMES;
        avgVerticalFocalDistanceError /= TIMES;
        avgSkewnessError /= TIMES;
        avgHorizontalPrincipalPointError /= TIMES;
        avgVerticalPrincipalPointError /= TIMES;
        final var avgTimeSeconds = (double) (endTime - startTime) / (double) TIMES / (double) 1000;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgHorizontalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);

        final var msg = "Circles pattern - No distortion avg horizontal focal distance error: "
                + avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " avg min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError
                + " time: " + avgTimeSeconds + " seconds";
        Logger.getLogger(AlternatingCameraCalibratorTest.class.getName()).log(Level.INFO, msg);
    }

    @Test
    void testCalibrateQRPatternNoDistortion() throws LockedException, CalibrationException, NotReadyException {

        final var startTime = System.currentTimeMillis();

        final var pattern = Pattern2D.create(Pattern2DType.QR);

        final var patternPoints = pattern.getIdealPoints();

        // assume that pattern points are located on a 3D plane
        // (for instance Z = 0), but can be really any plane
        final var points3D = new ArrayList<Point3D>();
        for (final var patternPoint : patternPoints) {
            points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0, 1.0));
        }

        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        var numValid = 0;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create samples (each sample has one pinhole camera associated and
            // its corresponding sampled markers)
            final var numSamples = randomizer.nextInt(MIN_NUM_SAMPLES, MAX_NUM_SAMPLES);
            final var samples = new ArrayList<CameraCalibratorSample>();
            for (var i = 0; i < numSamples; i++) {
                // create random camera
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points
                final var projectedPatternPoints = camera.project(points3D);

                // add random noise to projected points with a certain outlier
                // proportion

                samples.add(new CameraCalibratorSample(projectedPatternPoints));
            }

            // create calibrator
            final var calibrator = new AlternatingCameraCalibrator(pattern, samples);
            calibrator.setListener(this);
            calibrator.setEstimateRadialDistortion(false);
            calibrator.setHomographyMethod(RobustEstimatorMethod.LMEDS);
            calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.PROMEDS);

            // check correctness
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isLocked());

            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateProgressChange);
            assertEquals(0, intrinsicParametersEstimationStarts);
            assertEquals(0, intrinsicParametersEstimationEnds);
            assertEquals(0, radialDistortionEstimationStarts);
            assertEquals(0, radialDistortionEstimationEnds);

            // calibrate
            calibrator.calibrate();

            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateProgressChange >= 0);
            assertTrue(intrinsicParametersEstimationStarts > 0);
            assertTrue(intrinsicParametersEstimationEnds > 0);
            assertEquals(0, radialDistortionEstimationStarts);
            assertEquals(0, radialDistortionEstimationEnds);
            reset();

            // check correctness
            assertFalse(calibrator.isLocked());

            // check intrinsic parameters
            assertNotNull(calibrator.getHomographyQualityScores());
            assertNotNull(calibrator.getEstimatedImageOfAbsoluteConic());
            assertNotNull(calibrator.getEstimatedIntrinsicParameters());
            final var intrinsic2 = calibrator.getEstimatedIntrinsicParameters();

            if (Math.abs(intrinsic.getHorizontalFocalLength() - intrinsic2.getHorizontalFocalLength())
                    > 3.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                    3.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
            if (Math.abs(intrinsic.getVerticalFocalLength() - intrinsic2.getVerticalFocalLength())
                    > 3.0 * ULTRA_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                    3.0 * ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic.getSkewness(), 0.0);
            assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
            assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic2.getHorizontalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic2.getVerticalPrincipalPoint(), 0.0);

            horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                    - intrinsic2.getHorizontalFocalLength());
            verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                    - intrinsic2.getVerticalFocalLength());
            skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
            horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                    - intrinsic2.getHorizontalPrincipalPoint());
            verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                    - intrinsic2.getVerticalPrincipalPoint());

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

            // check radial distortion
            assertNull(calibrator.getDistortion());

            // check camera poses were not estimated (because distortion was not
            // estimated)
            var estimatedCams = 0;
            for (var i = 0; i < numSamples; i++) {
                if (samples.get(i).getHomography() != null) {
                    assertNull(samples.get(i).getCamera());
                    assertNull(samples.get(i).getCameraCenter());
                    assertNull(samples.get(i).getRotation());
                    if (samples.get(i).getCamera() != null) {
                        estimatedCams++;
                    }
                }
            }

            assertEquals(0, estimatedCams);

            numValid++;
        }

        assertTrue(numValid > 0);

        final var endTime = System.currentTimeMillis();

        avgHorizontalFocalDistanceError /= TIMES;
        avgVerticalFocalDistanceError /= TIMES;
        avgSkewnessError /= TIMES;
        avgHorizontalPrincipalPointError /= TIMES;
        avgVerticalPrincipalPointError /= TIMES;
        final var avgTimeSeconds = (double) (endTime - startTime) / (double) TIMES / (double) 1000;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgHorizontalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalFocalDistanceError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgSkewnessError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);

        final var msg = "QR pattern - No distortion avg horizontal focal distance error: "
                + avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " avg min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError
                + " time: " + avgTimeSeconds + " seconds";
        Logger.getLogger(AlternatingCameraCalibratorTest.class.getName()).log(Level.INFO, msg);
    }

    @Test
    void testCalibrateCirclesPatternDistortion() throws LockedException, NotReadyException, NotSupportedException,
            DistortionException {

        final var startTime = System.currentTimeMillis();

        final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);

        final var patternPoints = pattern.getIdealPoints();

        // assume that pattern points are located on a 3D plane
        // (for instance Z = 0), but can be really any plane
        final var points3D = new ArrayList<Point3D>();
        for (final var patternPoint : patternPoints) {
            points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0, 1.0));
        }

        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var avgK1Error = 0.0;
        var avgK2Error = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var minK1Error = Double.MAX_VALUE;
        var minK2Error = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        var maxK1Error = -Double.MAX_VALUE;
        var maxK2Error = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        double k1Error;
        double k2Error;
        var numValid = 0;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create distortion
            final var k1 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, MAX_DISTORTION_PARAM_VALUE);
            var k2 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, MAX_DISTORTION_PARAM_VALUE);
            // square k2 so that we ensure it is smaller than k1 (typically k1
            // is the dominant term)
            final var signk2 = Math.signum(k2);
            k2 = signk2 * k2 * k2;

            final var distortion = new RadialDistortion(k1, k2);
            distortion.setIntrinsic(intrinsic);

            // create samples (each sample has one pinhole camera associated and
            // its corresponding sampled markers)
            final var numSamples = randomizer.nextInt(MIN_NUM_SAMPLES, MAX_NUM_SAMPLES);
            final var samples = new ArrayList<CameraCalibratorSample>();
            for (var i = 0; i < numSamples; i++) {
                // create random camera
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points
                final var projectedPatternPoints = camera.project(points3D);

                // add random noise to projected points with a certain outlier
                // proportion

                samples.add(new CameraCalibratorSample(projectedPatternPoints));
            }

            // distort projected pattern points using created distortion
            // (we modify the samples)
            // To avoid optimization exceptions while computing distortions,
            // we need first to normalize samples and then denormalize them
            CameraCalibratorSample sample;
            for (int i = 0; i < numSamples; i++) {
                sample = samples.get(i);

                sample.setSampledMarkers(distortion.distort(sample.getSampledMarkers()));
            }

            // create calibrator
            final var calibrator = new AlternatingCameraCalibrator(pattern, samples);
            calibrator.setListener(this);
            calibrator.setEstimateRadialDistortion(true);
            calibrator.setHomographyMethod(RobustEstimatorMethod.LMEDS);
            calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.PROSAC);
            calibrator.setDistortionMethod(RobustEstimatorMethod.PROSAC);
            calibrator.setIACEstimatorThreshold(1e-6);

            // check correctness
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isLocked());

            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateProgressChange);
            assertEquals(0, intrinsicParametersEstimationStarts);
            assertEquals(0, intrinsicParametersEstimationEnds);
            assertEquals(0, radialDistortionEstimationStarts);
            assertEquals(0, radialDistortionEstimationEnds);

            // calibrate
            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateProgressChange >= 0);
            assertTrue(intrinsicParametersEstimationStarts > 0);
            assertTrue(intrinsicParametersEstimationEnds > 0);
            assertTrue(radialDistortionEstimationStarts > 0);
            assertTrue(radialDistortionEstimationEnds > 0);
            reset();

            // check correctness
            assertFalse(calibrator.isLocked());

            // check intrinsic parameters
            assertNotNull(calibrator.getHomographyQualityScores());
            assertNotNull(calibrator.getEstimatedImageOfAbsoluteConic());
            assertNotNull(calibrator.getEstimatedIntrinsicParameters());
            final var intrinsic2 = calibrator.getEstimatedIntrinsicParameters();

            assertEquals(0.0, intrinsic.getSkewness(), 0.0);
            assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
            assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic2.getHorizontalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic2.getVerticalPrincipalPoint(), 0.0);

            horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                    - intrinsic2.getHorizontalFocalLength());
            verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                    - intrinsic2.getVerticalFocalLength());
            skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
            horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                    - intrinsic2.getHorizontalPrincipalPoint());
            verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                    - intrinsic2.getVerticalPrincipalPoint());

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

            // check radial distortion
            final var distortion2 = calibrator.getDistortion();

            k1Error = Math.abs(distortion.getK1() - distortion2.getK1());
            k2Error = Math.abs(distortion.getK2() - distortion2.getK2());

            avgK1Error += k1Error;
            avgK2Error += k2Error;

            if (k1Error < minK1Error) {
                minK1Error = k1Error;
            }
            if (k2Error < minK2Error) {
                minK2Error = k2Error;
            }
            if (k1Error > maxK1Error) {
                maxK1Error = k1Error;
            }
            if (k2Error > maxK2Error) {
                maxK2Error = k2Error;
            }

            var estimatedCams = 0;
            for (var i = 0; i < numSamples; i++) {
                if (samples.get(i).getHomography() != null) {
                    assertNotNull(samples.get(i).getCamera());
                    assertNotNull(samples.get(i).getCameraCenter());
                    assertNotNull(samples.get(i).getRotation());
                    if (samples.get(i).getCamera() != null) {
                        estimatedCams++;
                    }
                }
            }

            assertTrue(estimatedCams > 0);
            numValid++;
        }

        assertTrue(numValid > 0);

        final var endTime = System.currentTimeMillis();

        avgHorizontalFocalDistanceError /= TIMES;
        avgVerticalFocalDistanceError /= TIMES;
        avgSkewnessError /= TIMES;
        avgHorizontalPrincipalPointError /= TIMES;
        avgVerticalPrincipalPointError /= TIMES;

        avgK1Error /= TIMES;
        avgK2Error /= TIMES;

        final var avgTimeSeconds = (double) (endTime - startTime) / (double) TIMES / (double) 1000;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgSkewnessError, ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, ABSOLUTE_ERROR);

        // check that average distortion parameters error is small enough

        final var msg = "Circles pattern - With distortion avg horizontal focal distance error: "
                + avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " avg distortion K1 error: " + avgK1Error
                + " avg distortion K2 error: " + avgK2Error
                + " min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " min distortion K1 error: " + minK1Error
                + " min distortion K2 error: " + minK2Error
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError
                + " max distortion K1 error: " + maxK1Error
                + " max distortion K2 error: " + maxK2Error
                + " time: " + avgTimeSeconds + " seconds";
        Logger.getLogger(AlternatingCameraCalibratorTest.class.getName()).log(Level.INFO, msg);
    }

    @Test
    void testCalibrateQRPatternDistortion() throws LockedException, CalibrationException, NotReadyException,
            NotSupportedException, DistortionException {

        final var startTime = System.currentTimeMillis();

        final var pattern = Pattern2D.create(Pattern2DType.QR);

        final var patternPoints = pattern.getIdealPoints();

        // assume that pattern points are located on a 3D plane
        // (for instance Z = 0), but can be really any plane
        final var points3D = new ArrayList<Point3D>();
        for (final var patternPoint : patternPoints) {
            points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0, 1.0));
        }

        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var avgK1Error = 0.0;
        var avgK2Error = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var minK1Error = Double.MAX_VALUE;
        var minK2Error = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        var maxK1Error = -Double.MAX_VALUE;
        var maxK2Error = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        double k1Error;
        double k2Error;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create distortion
            final var k1 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, MAX_DISTORTION_PARAM_VALUE);
            var k2 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, MAX_DISTORTION_PARAM_VALUE);
            // square k2 so that we ensure it is smaller than k1 (typically k1
            // is the dominant term)
            final var signk2 = Math.signum(k2);
            k2 = signk2 * k2 * k2;

            final var distortion = new RadialDistortion(k1, k2);
            distortion.setIntrinsic(intrinsic);

            // create samples (each sample has one pinhole camera associated and
            // its corresponding sampled markers)
            final var numSamples = randomizer.nextInt(MIN_NUM_SAMPLES, MAX_NUM_SAMPLES);
            final var samples = new ArrayList<CameraCalibratorSample>();
            for (var i = 0; i < numSamples; i++) {
                // create random camera
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points
                final var projectedPatternPoints = camera.project(points3D);

                // add random noise to projected points with a certain outlier
                // proportion

                samples.add(new CameraCalibratorSample(projectedPatternPoints));
            }

            // distort projected pattern points using created distortion
            // (we modify the samples)
            // To avoid optimization exceptions while computing distortions,
            // we need first to normalize samples and then denormalize them
            CameraCalibratorSample sample;
            for (var i = 0; i < numSamples; i++) {
                sample = samples.get(i);

                sample.setSampledMarkers(distortion.distort(sample.getSampledMarkers()));
            }

            // create calibrator
            final var calibrator = new AlternatingCameraCalibrator(pattern, samples);
            calibrator.setListener(this);
            calibrator.setEstimateRadialDistortion(true);
            calibrator.setHomographyMethod(RobustEstimatorMethod.LMEDS);
            calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.PROSAC);
            calibrator.setDistortionMethod(RobustEstimatorMethod.PROSAC);
            calibrator.setIACEstimatorThreshold(1e-6);

            // check correctness
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isLocked());

            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateProgressChange);
            assertEquals(0, intrinsicParametersEstimationStarts);
            assertEquals(0, intrinsicParametersEstimationEnds);
            assertEquals(0, radialDistortionEstimationStarts);
            assertEquals(0, radialDistortionEstimationEnds);

            // calibrate
            calibrator.calibrate();

            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateProgressChange >= 0);
            assertTrue(intrinsicParametersEstimationStarts > 0);
            assertTrue(intrinsicParametersEstimationEnds > 0);
            assertTrue(radialDistortionEstimationStarts > 0);
            assertTrue(radialDistortionEstimationEnds > 0);
            reset();

            // check correctness
            assertFalse(calibrator.isLocked());

            // check intrinsic parameters
            assertNotNull(calibrator.getHomographyQualityScores());
            assertNotNull(calibrator.getEstimatedImageOfAbsoluteConic());
            assertNotNull(calibrator.getEstimatedIntrinsicParameters());
            final var intrinsic2 = calibrator.getEstimatedIntrinsicParameters();

            assertEquals(0.0, intrinsic.getSkewness(), 0.0);
            assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
            assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic2.getHorizontalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic2.getVerticalPrincipalPoint(), 0.0);

            horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                    - intrinsic2.getHorizontalFocalLength());
            verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                    - intrinsic2.getVerticalFocalLength());
            skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
            horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                    - intrinsic2.getHorizontalPrincipalPoint());
            verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                    - intrinsic2.getVerticalPrincipalPoint());

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

            // check radial distortion
            final var distortion2 = calibrator.getDistortion();

            k1Error = Math.abs(distortion.getK1() - distortion2.getK1());
            k2Error = Math.abs(distortion.getK2() - distortion2.getK2());

            avgK1Error += k1Error;
            avgK2Error += k2Error;

            if (k1Error < minK1Error) {
                minK1Error = k1Error;
            }
            if (k2Error < minK2Error) {
                minK2Error = k2Error;
            }
            if (k1Error > maxK1Error) {
                maxK1Error = k1Error;
            }
            if (k2Error > maxK2Error) {
                maxK2Error = k2Error;
            }

            // check camera poses were not estimated (because distortion was not
            // estimated)
            var estimatedCams = 0;
            for (var i = 0; i < numSamples; i++) {
                if (samples.get(i).getHomography() != null) {
                    assertNotNull(samples.get(i).getCamera());
                    assertNotNull(samples.get(i).getCameraCenter());
                    assertNotNull(samples.get(i).getRotation());
                    if (samples.get(i).getCamera() != null) {
                        estimatedCams++;
                    }
                }
            }

            assertTrue(estimatedCams > 0);
        }

        final var endTime = System.currentTimeMillis();

        avgHorizontalFocalDistanceError /= TIMES;
        avgVerticalFocalDistanceError /= TIMES;
        avgSkewnessError /= TIMES;
        avgHorizontalPrincipalPointError /= TIMES;
        avgVerticalPrincipalPointError /= TIMES;

        avgK1Error /= TIMES;
        avgK2Error /= TIMES;

        final var avgTimeSeconds = (double) (endTime - startTime) / (double) TIMES / (double) 1000;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgSkewnessError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);

        // check that average distortion parameters error is small enough

        final var msg = "QR pattern - With distortion avg horizontal focal distance error: "
                + avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " avg distortion K1 error: " + avgK1Error
                + " avg distortion K2 error: " + avgK2Error
                + " min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " min distortion K1 error: " + minK1Error
                + " min distortion K2 error: " + minK2Error
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError
                + " max distortion K1 error: " + maxK1Error
                + " max distortion K2 error: " + maxK2Error
                + " time: " + avgTimeSeconds + " seconds";
        Logger.getLogger(AlternatingCameraCalibratorTest.class.getName()).log(Level.INFO, msg);
    }

    @Test
    void testCalibrateCirclesPatternDistortionAndOutliers() throws LockedException, NotReadyException,
            NotSupportedException, DistortionException {

        final var startTime = System.currentTimeMillis();

        final var pattern = Pattern2D.create(Pattern2DType.CIRCLES);

        final var patternPoints = pattern.getIdealPoints();

        // assume that pattern points are located on a 3D plane
        // (for instance Z = 0), but can be really any plane
        final var points3D = new ArrayList<Point3D>();
        for (Point2D patternPoint : patternPoints) {
            points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0, 1.0));
        }

        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var avgK1Error = 0.0;
        var avgK2Error = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var minK1Error = Double.MAX_VALUE;
        var minK2Error = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        var maxK1Error = -Double.MAX_VALUE;
        var maxK2Error = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        double k1Error;
        double k2Error;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create distortion
            final var k1 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, MAX_DISTORTION_PARAM_VALUE);
            var k2 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, MAX_DISTORTION_PARAM_VALUE);
            // square k2 so that we ensure it is smaller than k1 (typically k1
            // is the dominant term)
            final var signk2 = Math.signum(k2);
            k2 = signk2 * k2 * k2;

            final var distortion = new RadialDistortion(k1, k2);
            distortion.setIntrinsic(intrinsic);

            // create samples (each sample has one pinhole camera associated and
            // its corresponding sampled markers)
            final var numSamples = randomizer.nextInt(MIN_NUM_SAMPLES, MAX_NUM_SAMPLES);
            final var samples = new ArrayList<CameraCalibratorSample>();
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            for (var i = 0; i < numSamples; i++) {
                // create random camera
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points
                final var projectedPatternPoints = camera.project(points3D);

                // add outliers
                final var projectedPatternPointsWithError = new ArrayList<Point2D>();
                for (final var p : projectedPatternPoints) {
                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        final var errorX = errorRandomizer.nextDouble();
                        final var errorY = errorRandomizer.nextDouble();
                        projectedPatternPointsWithError.add(new HomogeneousPoint2D(p.getInhomX() + errorX,
                                        p.getInhomY() + errorY, 1.0));
                    } else {
                        // inlier
                        projectedPatternPointsWithError.add(p);
                    }
                }

                samples.add(new CameraCalibratorSample(projectedPatternPointsWithError));
            }

            // distort projected pattern points using created distortion
            // (we modify the samples)
            // To avoid optimization exceptions while computing distortions,
            // we need first to normalize samples and then denormalize them
            CameraCalibratorSample sample;
            for (var i = 0; i < numSamples; i++) {
                sample = samples.get(i);

                sample.setSampledMarkers(distortion.distort(sample.getSampledMarkers()));
            }

            // create calibrator
            final var calibrator = new AlternatingCameraCalibrator(pattern, samples);
            calibrator.setListener(this);
            calibrator.setEstimateRadialDistortion(true);
            calibrator.setHomographyMethod(RobustEstimatorMethod.LMEDS);
            calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.PROSAC);
            calibrator.setDistortionMethod(RobustEstimatorMethod.PROSAC);
            calibrator.setIACEstimatorThreshold(1e-6);

            // check correctness
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isLocked());

            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateProgressChange);
            assertEquals(0, intrinsicParametersEstimationStarts);
            assertEquals(0, intrinsicParametersEstimationEnds);
            assertEquals(0, radialDistortionEstimationStarts);
            assertEquals(0, radialDistortionEstimationEnds);

            // calibrate
            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                reset();
                continue;
            }

            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateProgressChange >= 0);
            assertTrue(intrinsicParametersEstimationStarts > 0);
            assertTrue(intrinsicParametersEstimationEnds > 0);
            assertTrue(radialDistortionEstimationStarts > 0);
            assertTrue(radialDistortionEstimationEnds > 0);
            reset();

            // check correctness
            assertFalse(calibrator.isLocked());

            // check intrinsic parameters
            assertNotNull(calibrator.getHomographyQualityScores());
            assertNotNull(calibrator.getEstimatedImageOfAbsoluteConic());
            assertNotNull(calibrator.getEstimatedIntrinsicParameters());
            final var intrinsic2 = calibrator.getEstimatedIntrinsicParameters();

            assertEquals(0.0, intrinsic.getSkewness(), 0.0);
            assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
            assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic2.getHorizontalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic2.getVerticalPrincipalPoint(), 0.0);

            horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                    - intrinsic2.getHorizontalFocalLength());
            verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                    - intrinsic2.getVerticalFocalLength());
            skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
            horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                    - intrinsic2.getHorizontalPrincipalPoint());
            verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                    - intrinsic2.getVerticalPrincipalPoint());

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

            // check radial distortion
            final var distortion2 = calibrator.getDistortion();

            k1Error = Math.abs(distortion.getK1() - distortion2.getK1());
            k2Error = Math.abs(distortion.getK2() - distortion2.getK2());

            avgK1Error += k1Error;
            avgK2Error += k2Error;

            if (k1Error < minK1Error) {
                minK1Error = k1Error;
            }
            if (k2Error < minK2Error) {
                minK2Error = k2Error;
            }
            if (k1Error > maxK1Error) {
                maxK1Error = k1Error;
            }
            if (k2Error > maxK2Error) {
                maxK2Error = k2Error;
            }

            // check camera poses were not estimated (because distortion was not
            // estimated)
            int estimatedCams = 0;
            for (var i = 0; i < numSamples; i++) {
                if (samples.get(i).getHomography() != null) {
                    assertNotNull(samples.get(i).getCamera());
                    assertNotNull(samples.get(i).getCameraCenter());
                    assertNotNull(samples.get(i).getRotation());
                    if (samples.get(i).getCamera() != null) {
                        estimatedCams++;
                    }
                }
            }

            assertTrue(estimatedCams > 0);
        }

        final var endTime = System.currentTimeMillis();

        avgHorizontalFocalDistanceError /= TIMES;
        avgVerticalFocalDistanceError /= TIMES;
        avgSkewnessError /= TIMES;
        avgHorizontalPrincipalPointError /= TIMES;
        avgVerticalPrincipalPointError /= TIMES;

        avgK1Error /= TIMES;
        avgK2Error /= TIMES;

        final var avgTimeSeconds = (double) (endTime - startTime) / (double) TIMES / (double) 1000;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgSkewnessError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);

        // check that average distortion parameters error is small enough

        final var msg = "Circles pattern - With distortion and outliers avg horizontal focal distance error: " +
                avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " avg distortion K1 error: " + avgK1Error
                + " avg distortion K2 error: " + avgK2Error
                + " min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " min distortion K1 error: " + minK1Error
                + " min distortion K2 error: " + minK2Error
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError
                + " max distortion K1 error: " + maxK1Error
                + " max distortion K2 error: " + maxK2Error
                + " time: " + avgTimeSeconds + " seconds";
        Logger.getLogger(AlternatingCameraCalibratorTest.class.getName()).log(Level.INFO, msg);
    }

    @Test
    void testCalibrateQRPatternDistortionAndOutliers() throws LockedException, CalibrationException, NotReadyException,
            NotSupportedException, DistortionException {

        final var startTime = System.currentTimeMillis();

        final var pattern = Pattern2D.create(Pattern2DType.QR);

        final var patternPoints = pattern.getIdealPoints();

        // assume that pattern points are located on a 3D plane
        // (for instance Z = 0), but can be really any plane
        final var points3D = new ArrayList<Point3D>();
        for (final var patternPoint : patternPoints) {
            points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0, 1.0));
        }

        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var avgK1Error = 0.0;
        var avgK2Error = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var minK1Error = Double.MAX_VALUE;
        var minK2Error = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        var maxK1Error = -Double.MAX_VALUE;
        var maxK2Error = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        double k1Error;
        double k2Error;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create distortion
            final var k1 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, MAX_DISTORTION_PARAM_VALUE);
            var k2 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, MAX_DISTORTION_PARAM_VALUE);
            // square k2 so that we ensure it is smaller than k1 (typically k1
            // is the dominant term)
            k2 = k2 * k2;

            final var distortion = new RadialDistortion(k1, k2);
            distortion.setIntrinsic(intrinsic);

            // create samples (each sample has one pinhole camera associated and
            // its corresponding sampled markers)
            final var numSamples = randomizer.nextInt(MIN_NUM_SAMPLES, MAX_NUM_SAMPLES);
            final var samples = new ArrayList<CameraCalibratorSample>();
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            for (var i = 0; i < numSamples; i++) {
                // create random camera
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // project 3D pattern points
                final var projectedPatternPoints = camera.project(points3D);

                // add outliers
                final var projectedPatternPointsWithError = new ArrayList<Point2D>();
                for (final var p : projectedPatternPoints) {
                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        final var errorX = errorRandomizer.nextDouble();
                        final var errorY = errorRandomizer.nextDouble();
                        projectedPatternPointsWithError.add(new HomogeneousPoint2D(p.getInhomX() + errorX,
                                p.getInhomY() + errorY, 1.0));
                    } else {
                        // inlier
                        projectedPatternPointsWithError.add(p);
                    }
                }

                samples.add(new CameraCalibratorSample(projectedPatternPointsWithError));
            }

            // distort projected pattern points using created distortion
            // (we modify the samples)
            // To avoid optimization exceptions while computing distortions,
            // we need first to normalize samples and then denormalize them
            CameraCalibratorSample sample;
            for (var i = 0; i < numSamples; i++) {
                sample = samples.get(i);

                sample.setSampledMarkers(distortion.distort(sample.getSampledMarkers()));
            }

            // create calibrator
            final var calibrator = new AlternatingCameraCalibrator(pattern, samples);
            calibrator.setListener(this);
            calibrator.setEstimateRadialDistortion(true);
            calibrator.setHomographyMethod(RobustEstimatorMethod.LMEDS);
            calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.PROSAC);
            calibrator.setDistortionMethod(RobustEstimatorMethod.PROSAC);
            calibrator.setIACEstimatorThreshold(1e-6);

            // check correctness
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isLocked());

            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateProgressChange);
            assertEquals(0, intrinsicParametersEstimationStarts);
            assertEquals(0, intrinsicParametersEstimationEnds);
            assertEquals(0, radialDistortionEstimationStarts);
            assertEquals(0, radialDistortionEstimationEnds);

            // calibrate
            calibrator.calibrate();

            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateProgressChange >= 0);
            assertTrue(intrinsicParametersEstimationStarts > 0);
            assertTrue(intrinsicParametersEstimationEnds > 0);
            assertTrue(radialDistortionEstimationStarts > 0);
            assertTrue(radialDistortionEstimationEnds > 0);
            reset();

            // check correctness
            assertFalse(calibrator.isLocked());

            // check intrinsic parameters
            assertNotNull(calibrator.getHomographyQualityScores());
            assertNotNull(calibrator.getEstimatedImageOfAbsoluteConic());
            assertNotNull(calibrator.getEstimatedIntrinsicParameters());
            final var intrinsic2 = calibrator.getEstimatedIntrinsicParameters();

            assertEquals(0.0, intrinsic.getSkewness(), 0.0);
            assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
            assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic2.getHorizontalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic2.getVerticalPrincipalPoint(), 0.0);

            horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                    - intrinsic2.getHorizontalFocalLength());
            verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                    - intrinsic2.getVerticalFocalLength());
            skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
            horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                    - intrinsic2.getHorizontalPrincipalPoint());
            verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                    - intrinsic2.getVerticalPrincipalPoint());

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

            // check radial distortion
            final var distortion2 = calibrator.getDistortion();

            k1Error = Math.abs(distortion.getK1() - distortion2.getK1());
            k2Error = Math.abs(distortion.getK2() - distortion2.getK2());

            avgK1Error += k1Error;
            avgK2Error += k2Error;

            if (k1Error < minK1Error) {
                minK1Error = k1Error;
            }
            if (k2Error < minK2Error) {
                minK2Error = k2Error;
            }
            if (k1Error > maxK1Error) {
                maxK1Error = k1Error;
            }
            if (k2Error > maxK2Error) {
                maxK2Error = k2Error;
            }

            // check camera poses were not estimated (because distortion was not
            // estimated)
            var estimatedCams = 0;
            for (var i = 0; i < numSamples; i++) {
                if (samples.get(i).getHomography() != null) {
                    assertNotNull(samples.get(i).getCamera());
                    assertNotNull(samples.get(i).getCameraCenter());
                    assertNotNull(samples.get(i).getRotation());
                    if (samples.get(i).getCamera() != null) {
                        estimatedCams++;
                    }
                }
            }

            assertTrue(estimatedCams > 0);
        }

        final var endTime = System.currentTimeMillis();

        avgHorizontalFocalDistanceError /= TIMES;
        avgVerticalFocalDistanceError /= TIMES;
        avgSkewnessError /= TIMES;
        avgHorizontalPrincipalPointError /= TIMES;
        avgVerticalPrincipalPointError /= TIMES;

        avgK1Error /= TIMES;
        avgK2Error /= TIMES;

        final var avgTimeSeconds = (double) (endTime - startTime) / (double) TIMES / (double) 1000;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgSkewnessError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);

        // check that average distortion parameters error is small enough

        final var msg = "QR pattern - With distortion and outliers avg horizontal focal distance error: " +
                avgHorizontalFocalDistanceError
                + " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " avg distortion K1 error: " + avgK1Error
                + " avg distortion K2 error: " + avgK2Error
                + " min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " min distortion K1 error: " + minK1Error
                + " min distortion K2 error: " + minK2Error
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError
                + " max distortion K1 error: " + maxK1Error
                + " max distortion K2 error: " + maxK2Error
                + " time: " + avgTimeSeconds + " seconds";
        Logger.getLogger(AlternatingCameraCalibratorTest.class.getName()).log(Level.INFO, msg);
    }

    @Test
    void testCalibrateMixedPatternDistortionAndOutliers() throws LockedException, NotReadyException,
            NotSupportedException, DistortionException {

        final var startTime = System.currentTimeMillis();

        var avgHorizontalFocalDistanceError = 0.0;
        var avgVerticalFocalDistanceError = 0.0;
        var avgSkewnessError = 0.0;
        var avgHorizontalPrincipalPointError = 0.0;
        var avgVerticalPrincipalPointError = 0.0;
        var avgK1Error = 0.0;
        var avgK2Error = 0.0;
        var minHorizontalFocalDistanceError = Double.MAX_VALUE;
        var minVerticalFocalDistanceError = Double.MAX_VALUE;
        var minSkewnessError = Double.MAX_VALUE;
        var minHorizontalPrincipalPointError = Double.MAX_VALUE;
        var minVerticalPrincipalPointError = Double.MAX_VALUE;
        var minK1Error = Double.MAX_VALUE;
        var minK2Error = Double.MAX_VALUE;
        var maxHorizontalFocalDistanceError = -Double.MAX_VALUE;
        var maxVerticalFocalDistanceError = -Double.MAX_VALUE;
        var maxSkewnessError = -Double.MAX_VALUE;
        var maxHorizontalPrincipalPointError = -Double.MAX_VALUE;
        var maxVerticalPrincipalPointError = -Double.MAX_VALUE;
        var maxK1Error = -Double.MAX_VALUE;
        var maxK2Error = -Double.MAX_VALUE;
        double horizontalFocalDistanceError;
        double verticalFocalDistanceError;
        double skewnessError;
        double horizontalPrincipalPointError;
        double verticalPrincipalPointError;
        double k1Error;
        double k2Error;
        var numValid = 0;
        for (var j = 0; j < TIMES; j++) {
            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = 0.0;
            final var verticalPrincipalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            // create distortion
            final var k1 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, MAX_DISTORTION_PARAM_VALUE);
            var k2 = randomizer.nextDouble(MIN_DISTORTION_PARAM_VALUE, MAX_DISTORTION_PARAM_VALUE);
            // square k2 so that we ensure it is smaller than k1 (typically k1
            // is the dominant term)
            final var signk2 = Math.signum(k2);
            k2 = signk2 * k2 * k2;

            final var distortion = new RadialDistortion(k1, k2);
            distortion.setIntrinsic(intrinsic);

            // create samples (each sample has one pinhole camera associated and
            // its corresponding sampled markers)
            final var numSamples = randomizer.nextInt(MIN_NUM_SAMPLES, MAX_NUM_SAMPLES);
            final var samples = new ArrayList<CameraCalibratorSample>();
            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            for (var i = 0; i < numSamples; i++) {
                // create random camera
                // rotation
                final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

                // camera center
                final var cameraCenterArray = new double[INHOM_3D_COORDS];
                randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

                // create camera with intrinsic parameters, rotation and camera
                // center
                final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);
                camera.normalize();

                // chose random pattern
                final var patternPos = randomizer.nextInt(0, Pattern2DType.values().length);
                final var samplePattern = Pattern2D.create(Pattern2DType.values()[patternPos]);

                // build 3D points corresponding to sample pattern (assuming they
                // are in plane Z = 0)
                final var samplePatternPoints = samplePattern.getIdealPoints();
                final var points3D = new ArrayList<Point3D>();
                for (final var patternPoint : samplePatternPoints) {
                    points3D.add(new HomogeneousPoint3D(patternPoint.getInhomX(), patternPoint.getInhomY(), 0.0,
                            1.0));
                }

                // project 3D pattern points
                final var projectedPatternPoints = camera.project(points3D);

                // add outliers
                final var projectedPatternPointsWithError = new ArrayList<Point2D>();
                for (final var p : projectedPatternPoints) {
                    if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                        // outlier
                        final var errorX = errorRandomizer.nextDouble();
                        final var errorY = errorRandomizer.nextDouble();
                        projectedPatternPointsWithError.add(new HomogeneousPoint2D(p.getInhomX() + errorX,
                                p.getInhomY() + errorY, 1.0));
                    } else {
                        // inlier
                        projectedPatternPointsWithError.add(p);
                    }
                }

                // add random noise to projected points with a certain outlier
                // proportion and a random pattern
                samples.add(new CameraCalibratorSample(samplePattern, projectedPatternPointsWithError));
            }

            // distort projected pattern points using created distortion
            // (we modify the samples)
            // To avoid optimization exceptions while computing distortions,
            // we need first to normalize samples and then denormalize them
            CameraCalibratorSample sample;
            for (var i = 0; i < numSamples; i++) {
                sample = samples.get(i);

                sample.setSampledMarkers(distortion.distort(sample.getSampledMarkers()));
            }

            final var pattern = Pattern2D.create(Pattern2DType.QR);

            // create calibrator
            final var calibrator = new AlternatingCameraCalibrator(pattern, samples);
            calibrator.setListener(this);
            calibrator.setEstimateRadialDistortion(true);
            calibrator.setHomographyMethod(RobustEstimatorMethod.LMEDS);
            calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.PROSAC);
            calibrator.setDistortionMethod(RobustEstimatorMethod.PROSAC);
            calibrator.setIACEstimatorThreshold(1e-6);

            // check correctness
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isLocked());

            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateProgressChange);
            assertEquals(0, intrinsicParametersEstimationStarts);
            assertEquals(0, intrinsicParametersEstimationEnds);
            assertEquals(0, radialDistortionEstimationStarts);
            assertEquals(0, radialDistortionEstimationEnds);

            // calibrate
            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                reset();
                continue;
            }

            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateProgressChange >= 0);
            assertTrue(intrinsicParametersEstimationStarts > 0);
            assertTrue(intrinsicParametersEstimationEnds > 0);
            assertTrue(radialDistortionEstimationStarts > 0);
            assertTrue(radialDistortionEstimationEnds > 0);
            reset();

            // check correctness
            assertFalse(calibrator.isLocked());

            // check intrinsic parameters
            assertNotNull(calibrator.getHomographyQualityScores());
            assertNotNull(calibrator.getEstimatedImageOfAbsoluteConic());
            assertNotNull(calibrator.getEstimatedIntrinsicParameters());
            final var intrinsic2 = calibrator.getEstimatedIntrinsicParameters();

            assertEquals(0.0, intrinsic.getSkewness(), 0.0);
            assertEquals(0.0, intrinsic2.getSkewness(), 0.0);
            assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic2.getHorizontalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), 0.0);
            assertEquals(0.0, intrinsic2.getVerticalPrincipalPoint(), 0.0);

            horizontalFocalDistanceError = Math.abs(intrinsic.getHorizontalFocalLength()
                    - intrinsic2.getHorizontalFocalLength());
            verticalFocalDistanceError = Math.abs(intrinsic.getVerticalFocalLength()
                    - intrinsic2.getVerticalFocalLength());
            skewnessError = Math.abs(intrinsic.getSkewness() - intrinsic2.getSkewness());
            horizontalPrincipalPointError = Math.abs(intrinsic.getHorizontalPrincipalPoint()
                    - intrinsic2.getHorizontalPrincipalPoint());
            verticalPrincipalPointError = Math.abs(intrinsic.getVerticalPrincipalPoint()
                    - intrinsic2.getVerticalPrincipalPoint());

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

            // check radial distortion
            final var distortion2 = calibrator.getDistortion();

            k1Error = Math.abs(distortion.getK1() - distortion2.getK1());
            k2Error = Math.abs(distortion.getK2() - distortion2.getK2());

            avgK1Error += k1Error;
            avgK2Error += k2Error;

            if (k1Error < minK1Error) {
                minK1Error = k1Error;
            }
            if (k2Error < minK2Error) {
                minK2Error = k2Error;
            }
            if (k1Error > maxK1Error) {
                maxK1Error = k1Error;
            }
            if (k2Error > maxK2Error) {
                maxK2Error = k2Error;
            }

            // check camera poses were not estimated (because distortion was not
            // estimated)
            var estimatedCams = 0;
            for (var i = 0; i < numSamples; i++) {
                if (samples.get(i).getHomography() != null) {
                    assertNotNull(samples.get(i).getCamera());
                    assertNotNull(samples.get(i).getCameraCenter());
                    assertNotNull(samples.get(i).getRotation());
                    if (samples.get(i).getCamera() != null) {
                        estimatedCams++;
                    }
                }
            }

            numValid++;

            assertTrue(estimatedCams > 0);
        }

        assertTrue(numValid > 0);

        final var endTime = System.currentTimeMillis();

        avgHorizontalFocalDistanceError /= TIMES;
        avgVerticalFocalDistanceError /= TIMES;
        avgSkewnessError /= TIMES;
        avgHorizontalPrincipalPointError /= TIMES;
        avgVerticalPrincipalPointError /= TIMES;

        avgK1Error /= TIMES;
        avgK2Error /= TIMES;

        final var avgTimeSeconds = (double) (endTime - startTime) / (double) TIMES / (double) 1000;

        // check that average error of intrinsic parameters is small enough
        assertEquals(0.0, avgSkewnessError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgHorizontalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(0.0, avgVerticalPrincipalPointError, VERY_LARGE_ABSOLUTE_ERROR);

        // check that average distortion parameters error is small enough
        final var msg = "Mixed pattern - With distortion and outliers avg horizontal focal distance error: " +
                avgHorizontalFocalDistanceError +
                " avg vertical focal distance error: " + avgVerticalFocalDistanceError
                + " avg skewness error: " + avgSkewnessError
                + " avg horizontal principal point error: " + avgHorizontalPrincipalPointError
                + " avg vertical principal point error: " + avgVerticalPrincipalPointError
                + " avg distortion K1 error: " + avgK1Error
                + " avg distortion K2 error: " + avgK2Error
                + " min horizontal focal distance error: " + minHorizontalFocalDistanceError
                + " min vertical focal distance error: " + minVerticalFocalDistanceError
                + " min skewness error: " + minSkewnessError
                + " min horizontal principal point error: " + minHorizontalPrincipalPointError
                + " min vertical principal point error: " + minVerticalPrincipalPointError
                + " min distortion K1 error: " + minK1Error
                + " min distortion K2 error: " + minK2Error
                + " max horizontal focal distance error: " + maxHorizontalFocalDistanceError
                + " max vertical focal distance error: " + maxVerticalFocalDistanceError
                + " max skewness error: " + maxSkewnessError
                + " max horizontal principal point error: " + maxHorizontalPrincipalPointError
                + " max vertical principal point error: " + maxVerticalPrincipalPointError
                + " max distortion K1 error: " + maxK1Error
                + " max distortion K2 error: " + maxK2Error
                + " time: " + avgTimeSeconds + " seconds";
        Logger.getLogger(AlternatingCameraCalibratorTest.class.getName()).log(Level.INFO, msg);
    }

    @Test
    void testCalibrateRealData() throws LockedException, CalibrationException, NotReadyException {
        // For a QR pattern, assuming zero skewness, equal focal lengths and
        // principal point at origin on a nexus 5 device
        final var pattern = Pattern2D.create(Pattern2DType.QR);
        
        /*
        Sampled data (before and after centering coordinates and setting correct y-axis
        direction)
        Point[0] = 774.5, 1084.5 | 6.5, -60.5
        Point[1] = 791.5, 840.0 | 23.5, 184.0
        Point[2] = 1037.5, 854.0 | 269.5, 170.0
        Point[3] = 999.0, 1074.0 | 231.0, -50.0        
        */
        final var sampledPoints1 = new ArrayList<Point2D>();
        sampledPoints1.add(new InhomogeneousPoint2D(6.5, -60.5));
        sampledPoints1.add(new InhomogeneousPoint2D(23.5, 184.0));
        sampledPoints1.add(new InhomogeneousPoint2D(269.5, 170.0));
        sampledPoints1.add(new InhomogeneousPoint2D(231.0, -50.0));
        
        /*
        Sampled data (before and after centering coordinates and setting correct y-axis
        direction)
        Point[0] = 382.5, 701.5 | -385.5, 322.5
        Point[1] = 351.0, 473.5 | -417.0, 550.5
        Point[2] = 585.0, 451.5 | -183.0, 572.5
        Point[3] = 592.0, 653.5 | -176.0, 370.5
        */
        final var sampledPoints2 = new ArrayList<Point2D>();
        sampledPoints2.add(new InhomogeneousPoint2D(-385.5, 322.5));
        sampledPoints2.add(new InhomogeneousPoint2D(-417.0, 550.5));
        sampledPoints2.add(new InhomogeneousPoint2D(-183.0, 572.5));
        sampledPoints2.add(new InhomogeneousPoint2D(-176.0, 370.5));

        /*
        Sampled data (before and after centering coordinates and setting correct y-axis
        direction)
        Point[0] = 988.0, 486.5 | 220.0, 537.5
        Point[1] = 1028.5, 278.5 | 260.5, 745.5
        Point[2] = 1241.0, 316.5 | 473.0, 707.5
        Point[3] = 1185.5, 498.5 | 417.5, 525.5
        */
        final var sampledPoints3 = new ArrayList<Point2D>();
        sampledPoints3.add(new InhomogeneousPoint2D(220.0, 537.5));
        sampledPoints3.add(new InhomogeneousPoint2D(260.5, 745.5));
        sampledPoints3.add(new InhomogeneousPoint2D(473.0, 707.5));
        sampledPoints3.add(new InhomogeneousPoint2D(417.5, 525.5));
        
        /*
        Sampled data (before and after centering coordinates and setting correct y-axis
        direction)
        Point[0] = 576.0, 1404.4166 | -192.0, -380.4166259765625
        Point[1] = 544.5, 1151.5 | -223.5, -127.5
        Point[2] = 792.0, 1117.5 | 24.0, -93.5
        Point[3] = 798.5, 1347.0 | 30.5, -323.0
        */
        final var sampledPoints4 = new ArrayList<Point2D>();
        sampledPoints4.add(new InhomogeneousPoint2D(-192.0, -380.4166259765625));
        sampledPoints4.add(new InhomogeneousPoint2D(-223.5, -127.5));
        sampledPoints4.add(new InhomogeneousPoint2D(24.0, -93.5));
        sampledPoints4.add(new InhomogeneousPoint2D(30.5, -323.0));

        /*
        Sampled data (before and after centering coordinates and setting correct y-axis
        direction)
        Point[0] = 913.5, 1596.0 | 145.5, -572.0
        Point[1] = 939.5, 1360.7 | 171.5, -336.699951171875
        Point[2] = 1170.5, 1391.0 | 402.5, -367.0
        Point[3] = 1126.5, 1600.5 | 358.5, -576.5 
        */
        final var sampledPoints5 = new ArrayList<Point2D>();
        sampledPoints5.add(new InhomogeneousPoint2D(145.5, -572.0));
        sampledPoints5.add(new InhomogeneousPoint2D(171.5, -336.699951171875));
        sampledPoints5.add(new InhomogeneousPoint2D(402.5, -367.0));
        sampledPoints5.add(new InhomogeneousPoint2D(358.5, -576.5));

        // create samples for each set of sampled QR points
        final var samples = new ArrayList<CameraCalibratorSample>();
        final var sample1 = new CameraCalibratorSample(pattern, sampledPoints1);
        final var sample2 = new CameraCalibratorSample(pattern, sampledPoints2);
        final var sample3 = new CameraCalibratorSample(pattern, sampledPoints3);
        final var sample4 = new CameraCalibratorSample(pattern, sampledPoints4);
        final var sample5 = new CameraCalibratorSample(pattern, sampledPoints5);

        samples.add(sample1);
        samples.add(sample2);
        samples.add(sample3);
        samples.add(sample4);
        samples.add(sample5);

        // create calibrator
        final var calibrator = new AlternatingCameraCalibrator(pattern, samples);
        calibrator.setListener(this);
        calibrator.setEstimateRadialDistortion(true);
        calibrator.setHomographyMethod(RobustEstimatorMethod.LMEDS);
        calibrator.setImageOfAbsoluteConicMethod(RobustEstimatorMethod.PROSAC);
        calibrator.setIACEstimatorThreshold(REAL_IAC_THRESHOLD);
        calibrator.setDistortionMethod(RobustEstimatorMethod.PROSAC);

        // check correctness
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isLocked());

        assertEquals(0, calibrateStart);
        assertEquals(0, calibrateEnd);
        assertEquals(0, calibrateProgressChange);
        assertEquals(0, intrinsicParametersEstimationStarts);
        assertEquals(0, intrinsicParametersEstimationEnds);
        assertEquals(0, radialDistortionEstimationStarts);
        assertEquals(0, radialDistortionEstimationEnds);

        // calibrate
        calibrator.calibrate();

        assertEquals(1, calibrateStart);
        assertEquals(1, calibrateEnd);
        assertTrue(calibrateProgressChange >= 0);
        assertTrue(intrinsicParametersEstimationStarts > 0);
        assertTrue(intrinsicParametersEstimationEnds > 0);
        assertTrue(radialDistortionEstimationStarts > 0);
        assertTrue(radialDistortionEstimationEnds > 0);
        reset();

        // check correctness
        assertFalse(calibrator.isLocked());

        // check intrinsic parameters
        assertNotNull(calibrator.getHomographyQualityScores());
        assertNotNull(calibrator.getEstimatedImageOfAbsoluteConic());
        assertNotNull(calibrator.getEstimatedIntrinsicParameters());
        final var intrinsic = calibrator.getEstimatedIntrinsicParameters();

        final var horizontalFocalLength = intrinsic.getHorizontalFocalLength();
        final var verticalFocalLength = intrinsic.getVerticalFocalLength();
        final var skewness = intrinsic.getSkewness();
        final var horizontalPrincipalPoint = intrinsic.getHorizontalPrincipalPoint();
        final var verticalPrincipalPoint = intrinsic.getVerticalPrincipalPoint();
        assertTrue(horizontalFocalLength > 0);
        assertTrue(verticalFocalLength > 0);
        assertEquals(horizontalFocalLength, verticalFocalLength, ABSOLUTE_ERROR);
        assertEquals(0.0, skewness, 0.0);
        assertEquals(0.0, horizontalPrincipalPoint, 0.0);
        assertEquals(0.0, verticalPrincipalPoint, 0.0);

        // check distortion
        final var distortion = calibrator.getDistortion();

        final var k1 = distortion.getK1();
        final var k2 = distortion.getK2();

        final var msg = "Real data - focal length: " + horizontalFocalLength + "Distortion K1: " + k1 + " K2: " + k2;
        Logger.getLogger(AlternatingCameraCalibratorTest.class.getName()).log(Level.INFO, msg);
    }

    @Override
    public void onCalibrateStart(final CameraCalibrator calibrator) {
        calibrateStart++;
        checkLocked((AlternatingCameraCalibrator) calibrator);
    }

    @Override
    public void onCalibrateEnd(final CameraCalibrator calibrator) {
        calibrateEnd++;
        checkLocked((AlternatingCameraCalibrator) calibrator);
    }

    @Override
    public void onCalibrateProgressChange(final CameraCalibrator calibrator, final float progress) {
        calibrateProgressChange++;
        checkLocked((AlternatingCameraCalibrator) calibrator);
    }

    @Override
    public void onIntrinsicParametersEstimationStarts(final CameraCalibrator calibrator) {
        intrinsicParametersEstimationStarts++;
        checkLocked((AlternatingCameraCalibrator) calibrator);
    }

    @Override
    public void onIntrinsicParametersEstimationEnds(
            final CameraCalibrator calibrator, final PinholeCameraIntrinsicParameters intrinsicParameters) {
        intrinsicParametersEstimationEnds++;
        checkLocked((AlternatingCameraCalibrator) calibrator);
    }

    @Override
    public void onRadialDistortionEstimationStarts(final CameraCalibrator calibrator) {
        radialDistortionEstimationStarts++;
        checkLocked((AlternatingCameraCalibrator) calibrator);
    }

    @Override
    public void onRadialDistortionEstimationEnds(final CameraCalibrator calibrator, final RadialDistortion distortion) {
        radialDistortionEstimationEnds++;
        checkLocked((AlternatingCameraCalibrator) calibrator);
    }

    private void reset() {
        calibrateStart = calibrateEnd = calibrateProgressChange = intrinsicParametersEstimationStarts =
                intrinsicParametersEstimationEnds = radialDistortionEstimationStarts = radialDistortionEstimationEnds =
                        0;
    }

    private void checkLocked(final AlternatingCameraCalibrator calibrator) {
        assertThrows(LockedException.class, () -> calibrator.setMaxIterations(1));
        assertThrows(LockedException.class, () -> calibrator.setConvergenceThreshold(0.5));
        assertThrows(LockedException.class, () -> calibrator.setDistortionMethod(RobustEstimatorMethod.LMEDS));
        assertThrows(LockedException.class, () -> calibrator.setDistortionEstimatorThreshold(1.0));
        assertThrows(LockedException.class, () -> calibrator.setDistortionEstimatorConfidence(0.5));
        assertThrows(LockedException.class, () -> calibrator.setDistortionEstimatorMaxIterations(10));
        assertThrows(LockedException.class, () -> calibrator.setPattern(null));
        assertThrows(LockedException.class, () -> calibrator.setSamples(null));
        assertThrows(LockedException.class, () -> calibrator.setSamplesQualityScores(null));
        assertThrows(LockedException.class, () -> calibrator.setEstimateRadialDistortion(true));
        assertThrows(LockedException.class, () -> calibrator.setHomographyMethod(RobustEstimatorMethod.LMEDS));
        assertThrows(LockedException.class, () -> calibrator.setImageOfAbsoluteConicMethod(
                RobustEstimatorMethod.PROSAC));
        assertThrows(LockedException.class, () -> calibrator.setZeroSkewness(true));
        assertThrows(LockedException.class, () -> calibrator.setPrincipalPointAtOrigin(true));
        assertThrows(LockedException.class, () -> calibrator.setFocalDistanceAspectRatioKnown(true));
        assertThrows(LockedException.class, () -> calibrator.setFocalDistanceAspectRatio(0.5));
        assertThrows(LockedException.class, () -> calibrator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> calibrator.setHomographyEstimatorThreshold(0.5));
        assertThrows(LockedException.class, () -> calibrator.setHomographyEstimatorConfidence(0.5));
        assertThrows(LockedException.class, () -> calibrator.setHomographyEstimatorMaxIterations(10));
        assertThrows(LockedException.class, () -> calibrator.setIACEstimatorThreshold(0.5));
        assertThrows(LockedException.class, () -> calibrator.setIACEstimatorConfidence(0.5));
        assertThrows(LockedException.class, () -> calibrator.setIACEstimatorMaxIterations(10));
        assertThrows(LockedException.class, () -> calibrator.setListener(this));
        assertThrows(LockedException.class, calibrator::calibrate);
        assertTrue(calibrator.isLocked());
    }
}
