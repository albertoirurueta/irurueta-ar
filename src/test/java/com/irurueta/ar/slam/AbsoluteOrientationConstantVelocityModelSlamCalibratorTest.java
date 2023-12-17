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
package com.irurueta.ar.slam;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.numerical.signal.processing.MeasurementNoiseCovarianceEstimator;
import com.irurueta.numerical.signal.processing.SignalProcessingException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.InvalidCovarianceMatrixException;
import com.irurueta.statistics.MultivariateNormalDist;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class AbsoluteOrientationConstantVelocityModelSlamCalibratorTest implements
        BaseSlamCalibrator.BaseSlamCalibratorListener<
                AbsoluteOrientationConstantVelocityModelSlamCalibrationData> {

    private static final int TIMES = 50;
    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-2;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 0.5;

    private static final float MIN_OFFSET = -10.0f;
    private static final float MAX_OFFSET = 10.0f;

    private static final float NOISE_DEVIATION = 1e-5f;

    private static final int N_SAMPLES = 10000;

    // conversion from milliseconds to nanoseconds
    public static final int MILLIS_TO_NANOS = 1000000;

    // time between samples expressed in nanoseconds (a typical sensor in Android
    // delivers a sample every 20ms)
    public static final int DELTA_NANOS = 20000000; //0.02 seconds
    public static final double DELTA_SECONDS = 0.02;

    private int fullSampleReceived;
    private int fullSampleProcessed;
    private int calibratorFinished;

    @Test
    public void testConstructor() throws WrongSizeException {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // check initial values
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH,
                calibrator.getSampleLength());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamEstimator.STATE_LENGTH,
                calibrator.getEstimatorStateLength());
        assertFalse(calibrator.isConverged());
        assertFalse(calibrator.isFailed());
        assertFalse(calibrator.isFinished());
        assertEquals(0, calibrator.getSampleCount());
        assertEquals(SlamCalibrator.DEFAULT_MIN_NUM_SAMPLES, calibrator.getMinNumSamples());
        assertEquals(SlamCalibrator.DEFAULT_MAX_NUM_SAMPLES, calibrator.getMaxNumSamples());
        assertEquals(SlamCalibrator.DEFAULT_CONVERGENCE_THRESHOLD,
                calibrator.getConvergenceThreshold(), 0.0);
        assertEquals(SlamCalibrator.DEFAULT_ENABLE_SAMPLE_ACCUMULATION, calibrator.isAccumulationEnabled());
        assertEquals(-1, calibrator.getAccelerometerTimestampNanos());
        assertEquals(-1, calibrator.getGyroscopeTimestampNanos());
        assertEquals(0, calibrator.getAccumulatedAccelerometerSamples());
        assertEquals(0, calibrator.getAccumulatedGyroscopeSamples());
        assertFalse(calibrator.isAccelerometerSampleReceived());
        assertFalse(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleZ(), 0.0);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, calibrator.getAccumulatedAccelerationSample(), 0.0);
        final double[] accumAcceleration = new double[3];
        calibrator.getAccumulatedAccelerationSample(accumAcceleration);
        assertArrayEquals(accumAcceleration, new double[]{0.0, 0.0, 0.0}, 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, calibrator.getAccumulatedAngularSpeedSample(), 0.0);
        final double[] accumAngularSpeed = new double[3];
        calibrator.getAccumulatedAngularSpeedSample(accumAngularSpeed);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, accumAngularSpeed, 0.0);
        assertNull(calibrator.getListener());
        assertArrayEquals(new double[AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH],
                calibrator.getControlMean(), 0.0);
        final double[] controlMean = new double[
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH];
        calibrator.getControlMean(controlMean);
        assertArrayEquals(new double[AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH],
                controlMean, 0.0);
        assertEquals(new Matrix(
                        AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH,
                        AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH),
                calibrator.getControlCovariance());
        final Matrix cov = new Matrix(1, 1);
        calibrator.getControlCovariance(cov);
        assertEquals(new Matrix(
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH), cov);
    }

    @Test
    public void testIsConverged() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertFalse(calibrator.isConverged());

        // set new value
        calibrator.mConverged = true;

        // check correctness
        assertTrue(calibrator.isConverged());
    }

    @Test
    public void testIsFailed() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertFalse(calibrator.isFailed());

        // set new value
        calibrator.mFailed = true;

        // check correctness
        assertTrue(calibrator.isFailed());
    }

    @Test
    public void testIsFinished() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertFalse(calibrator.isFinished());

        // set new value
        calibrator.mFinished = true;

        // check correctness
        assertTrue(calibrator.isFinished());
    }

    @Test
    public void testGetSampleCount() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertEquals(0, calibrator.getSampleCount());

        // set new value
        calibrator.mSampleCount = 5;

        // check correctness
        assertEquals(5, calibrator.getSampleCount());
    }

    @Test
    public void testGetSetMinNumSamples() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertEquals(SlamCalibrator.DEFAULT_MIN_NUM_SAMPLES, calibrator.getMinNumSamples());

        // set new value
        calibrator.setMinNumSamples(50);

        // check correctness
        assertEquals(50, calibrator.getMinNumSamples());

        // Force IllegalArgumentException
        try {
            calibrator.setMinNumSamples(-1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxNumSamples() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertEquals(SlamCalibrator.DEFAULT_MAX_NUM_SAMPLES, calibrator.getMaxNumSamples());

        // set new value
        calibrator.setMaxNumSamples(1000);

        // check correctness
        assertEquals(1000, calibrator.getMaxNumSamples());

        // Force IllegalArgumentException
        try {
            calibrator.setMaxNumSamples(-1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetConvergenceThreshold() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertEquals(SlamCalibrator.DEFAULT_CONVERGENCE_THRESHOLD,
                calibrator.getConvergenceThreshold(), 0.0);

        // set new value
        calibrator.setConvergenceThreshold(1.0);

        // check correctness
        assertEquals(1.0, calibrator.getConvergenceThreshold(), 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.setConvergenceThreshold(-0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testReset() throws WrongSizeException {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        calibrator.reset();

        // check correctness
        assertFalse(calibrator.isConverged());
        assertFalse(calibrator.isFailed());
        assertFalse(calibrator.isFinished());
        assertEquals(0, calibrator.getSampleCount());
        assertEquals(SlamCalibrator.DEFAULT_MIN_NUM_SAMPLES, calibrator.getMinNumSamples());
        assertEquals(SlamCalibrator.DEFAULT_MAX_NUM_SAMPLES, calibrator.getMaxNumSamples());
        assertEquals(SlamCalibrator.DEFAULT_CONVERGENCE_THRESHOLD,
                calibrator.getConvergenceThreshold(), 0.0);
        assertEquals(SlamCalibrator.DEFAULT_ENABLE_SAMPLE_ACCUMULATION,
                calibrator.isAccumulationEnabled());
        assertEquals(-1, calibrator.getAccelerometerTimestampNanos());
        assertEquals(-1, calibrator.getGyroscopeTimestampNanos());
        assertEquals(0, calibrator.getAccumulatedAccelerometerSamples());
        assertEquals(0, calibrator.getAccumulatedGyroscopeSamples());
        assertFalse(calibrator.isAccelerometerSampleReceived());
        assertFalse(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleZ(), 0.0);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, calibrator.getAccumulatedAccelerationSample(), 0.0);
        final double[] accumAcceleration = new double[3];
        calibrator.getAccumulatedAccelerationSample(accumAcceleration);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, accumAcceleration, 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, calibrator.getAccumulatedAngularSpeedSample(), 0.0);
        final double[] accumAngularSpeed = new double[3];
        calibrator.getAccumulatedAngularSpeedSample(accumAngularSpeed);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, accumAngularSpeed, 0.0);
        assertNull(calibrator.getListener());
        assertArrayEquals(new double[AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH],
                calibrator.getControlMean(), 0.0);
        final double[] controlMean = new double[
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH];
        calibrator.getControlMean(controlMean);
        assertArrayEquals(new double[AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH],
                controlMean, 0.0);
        assertEquals(new Matrix(
                        AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH,
                        AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH),
                calibrator.getControlCovariance());
        final Matrix cov = new Matrix(1, 1);
        calibrator.getControlCovariance(cov);
        assertEquals(new Matrix(
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH), cov);
    }

    @Test
    public void testIsSetAccumulationEnabled() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertTrue(calibrator.isAccumulationEnabled());

        // set new value
        calibrator.setAccumulationEnabled(false);

        // check correctness
        assertFalse(calibrator.isAccumulationEnabled());
    }

    @Test
    public void testGetAccelerometerTimestampNanos() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertEquals(-1, calibrator.getAccelerometerTimestampNanos());

        // set new value
        calibrator.mAccelerometerTimestampNanos = 1000;

        // check correctness
        assertEquals(1000, calibrator.getAccelerometerTimestampNanos());
    }

    @Test
    public void testGetGyroscopeTimestampNanos() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertEquals(-1, calibrator.getGyroscopeTimestampNanos());

        // set new value
        calibrator.mGyroscopeTimestampNanos = 2000;

        // check correctness
        assertEquals(2000, calibrator.getGyroscopeTimestampNanos());
    }

    @Test
    public void testGetAccumulatedAccelerometerSamplesAndIsAccelerometerSampleReceived() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertEquals(0, calibrator.getAccumulatedAccelerometerSamples());
        assertFalse(calibrator.isAccelerometerSampleReceived());

        // set new value
        calibrator.mAccumulatedAccelerometerSamples = 50;

        // check correctness
        assertEquals(50, calibrator.getAccumulatedAccelerometerSamples());
        assertTrue(calibrator.isAccelerometerSampleReceived());
    }

    @Test
    public void testGetAccumulatedGyroscopeSamples() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertEquals(0, calibrator.getAccumulatedGyroscopeSamples());
        assertFalse(calibrator.isGyroscopeSampleReceived());

        // set new value
        calibrator.mAccumulatedGyroscopeSamples = 500;

        // check correctness
        assertEquals(500, calibrator.getAccumulatedGyroscopeSamples());
        assertTrue(calibrator.isGyroscopeSampleReceived());
    }

    @Test
    public void testIsFullSampleAvailable() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial values
        assertEquals(0, calibrator.getAccumulatedAccelerometerSamples());
        assertEquals(0, calibrator.getAccumulatedGyroscopeSamples());
        assertEquals(0, calibrator.getAccumulatedOrientationSamples());
        assertFalse(calibrator.isAccelerometerSampleReceived());
        assertFalse(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isOrientationSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());

        // set accelerometer sample
        calibrator.mAccumulatedAccelerometerSamples = 1;

        // check correctness
        assertEquals(1, calibrator.getAccumulatedAccelerometerSamples());
        assertEquals(0, calibrator.getAccumulatedGyroscopeSamples());
        assertEquals(0, calibrator.getAccumulatedOrientationSamples());
        assertTrue(calibrator.isAccelerometerSampleReceived());
        assertFalse(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isOrientationSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());

        // set gyroscope sample
        calibrator.mAccumulatedGyroscopeSamples = 1;

        // check correctness
        assertEquals(1, calibrator.getAccumulatedAccelerometerSamples());
        assertEquals(1, calibrator.getAccumulatedGyroscopeSamples());
        assertEquals(0, calibrator.getAccumulatedOrientationSamples());
        assertTrue(calibrator.isAccelerometerSampleReceived());
        assertTrue(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isOrientationSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());

        // set orientation sample
        calibrator.mAccumulatedOrientationSamples = 1;

        // check correctness
        assertEquals(1, calibrator.getAccumulatedAccelerometerSamples());
        assertEquals(1, calibrator.getAccumulatedGyroscopeSamples());
        assertEquals(1, calibrator.getAccumulatedOrientationSamples());
        assertTrue(calibrator.isAccelerometerSampleReceived());
        assertTrue(calibrator.isGyroscopeSampleReceived());
        assertTrue(calibrator.isOrientationSampleReceived());
        assertTrue(calibrator.isFullSampleAvailable());
    }

    @Test
    public void testGetAccumulatedAccelerationSampleX() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleX(), 0.0);

        // set new value
        calibrator.mAccumulatedAccelerationSampleX = 1.0;

        // check correctness
        assertEquals(1.0, calibrator.getAccumulatedAccelerationSampleX(), 0.0);
    }

    @Test
    public void testGetAccumulatedAccelerationSampleY() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleY(), 0.0);

        // set new value
        calibrator.mAccumulatedAccelerationSampleY = 2.0;

        // check correctness
        assertEquals(2.0, calibrator.getAccumulatedAccelerationSampleY(), 0.0);
    }

    @Test
    public void testGetAccumulatedAccelerationSampleZ() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleZ(), 0.0);

        // set new value
        calibrator.mAccumulatedAccelerationSampleZ = 3.0;

        // check correctness
        assertEquals(3.0, calibrator.getAccumulatedAccelerationSampleZ(), 0.0);
    }

    @Test
    public void testGetAccumulatedAccelerationSample() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, calibrator.getAccumulatedAccelerationSample(), 0.0);

        // set new value
        calibrator.mAccumulatedAccelerationSampleX = 1.0;
        calibrator.mAccumulatedAccelerationSampleY = 2.0;
        calibrator.mAccumulatedAccelerationSampleZ = 3.0;

        // check correctness
        assertArrayEquals(new double[]{1.0, 2.0, 3.0}, calibrator.getAccumulatedAccelerationSample(), 0.0);

        final double[] sample = new double[3];
        calibrator.getAccumulatedAccelerationSample(sample);

        // check correctness
        assertArrayEquals(new double[]{1.0, 2.0, 3.0}, sample, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.getAccumulatedAccelerationSample(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetAccumulatedAngularSpeedSampleX() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleX(), 0.0);

        // set new value
        calibrator.mAccumulatedAngularSpeedSampleX = 1.0;

        // check correctness
        assertEquals(1.0, calibrator.getAccumulatedAngularSpeedSampleX(), 0.0);
    }

    @Test
    public void testGetAccumulatedAngularSpeedSampleY() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleY(), 0.0);

        // set new value
        calibrator.mAccumulatedAngularSpeedSampleY = 2.0;

        // check correctness
        assertEquals(2.0, calibrator.getAccumulatedAngularSpeedSampleY(), 0.0);
    }

    @Test
    public void testGetAccumulatedAngularSpeedSampleZ() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0);

        // set new value
        calibrator.mAccumulatedAngularSpeedSampleZ = 3.0;

        // check correctness
        assertEquals(3.0, calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0);
    }

    @Test
    public void testGetAccumulatedAngularSpeedSample() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, calibrator.getAccumulatedAngularSpeedSample(), 0.0);

        // set new value
        calibrator.mAccumulatedAngularSpeedSampleX = 1.0;
        calibrator.mAccumulatedAngularSpeedSampleY = 2.0;
        calibrator.mAccumulatedAngularSpeedSampleZ = 3.0;

        // check correctness
        assertArrayEquals(new double[]{1.0, 2.0, 3.0}, calibrator.getAccumulatedAngularSpeedSample(), 0.0);

        final double[] sample = new double[3];
        calibrator.getAccumulatedAngularSpeedSample(sample);

        // check correctness
        assertArrayEquals(new double[]{1.0, 2.0, 3.0}, sample, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.getAccumulatedAngularSpeedSample(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetMostRecentTimestampNanos() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // check correctness
        assertEquals(-1, calibrator.getMostRecentTimestampNanos());

        // set new value
        calibrator.mAccelerometerTimestampNanos = 1000;

        // check correctness
        assertEquals(1000, calibrator.getMostRecentTimestampNanos());

        // set new value
        calibrator.mGyroscopeTimestampNanos = 2000;

        // check correctness
        assertEquals(2000, calibrator.getMostRecentTimestampNanos());
    }

    @Test
    public void testGetSetListener() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        // initial value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check correctness
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testGetControlMean() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        assertArrayEquals(new double[AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH],
                calibrator.getControlMean(), 0.0);
        assertArrayEquals(calibrator.mEstimator.getSampleAverage(), calibrator.getControlMean(), 0.0);

        final double[] controlMean = new double[
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH];
        calibrator.getControlMean(controlMean);
        assertArrayEquals(controlMean, calibrator.getControlMean(), 0.0);

        // Force IllegalArgumentException
        final double[] wrong = new double[1];
        try {
            calibrator.getControlMean(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetControlCovariance() throws WrongSizeException {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        assertEquals(new Matrix(AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH,
                        AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH),
                calibrator.getControlCovariance());

        final Matrix cov = new Matrix(1, 1);
        calibrator.getControlCovariance(cov);

        assertEquals(cov, calibrator.getControlCovariance());
    }

    @Test
    public void testGetControlDistribution()
            throws InvalidCovarianceMatrixException {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        final double[] controlMean = calibrator.getControlMean();
        final Matrix cov = calibrator.getControlCovariance();

        final MultivariateNormalDist dist = calibrator.getControlDistribution();

        // check correctness
        assertArrayEquals(controlMean, dist.getMean(), ABSOLUTE_ERROR);
        assertTrue(cov.equals(dist.getCovariance(), ABSOLUTE_ERROR));

        final MultivariateNormalDist dist2 = new MultivariateNormalDist();
        calibrator.getControlDistribution(dist2);

        // check correctness
        assertArrayEquals(controlMean, dist2.getMean(), ABSOLUTE_ERROR);
        assertTrue(cov.equals(dist2.getCovariance(), ABSOLUTE_ERROR));
    }

    @Test
    public void testGetCalibrationData() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        final double[] controlMean = calibrator.getControlMean();
        final Matrix cov = calibrator.getControlCovariance();

        final AbsoluteOrientationConstantVelocityModelSlamCalibrationData data1 =
                calibrator.getCalibrationData();
        final AbsoluteOrientationConstantVelocityModelSlamCalibrationData data2 =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrationData();
        calibrator.getCalibrationData(data2);

        // check correctness
        assertSame(controlMean, data1.getControlMean());
        assertSame(cov, data1.getControlCovariance());

        assertSame(controlMean, data2.getControlMean());
        assertSame(cov, data2.getControlCovariance());
    }

    @Test
    public void testPropagateWithControlJacobian() throws WrongSizeException, InvalidCovarianceMatrixException {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();

        final UniformRandomizer offsetRandomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, NOISE_DEVIATION);

        final float accelerationOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final float accelerationOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final float accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);

        final float angularOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final float angularOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final float angularOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);

        long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;

        float accelerationNoiseX;
        float accelerationNoiseY;
        float accelerationNoiseZ;
        float angularNoiseX;
        float angularNoiseY;
        float angularNoiseZ;

        double accelerationX;
        double accelerationY;
        double accelerationZ;
        double angularX;
        double angularY;
        double angularZ;
        final Quaternion orientation = new Quaternion();

        calibrator.reset();

        for (int i = 0; i < N_SAMPLES; i++) {
            accelerationNoiseX = noiseRandomizer.nextFloat();
            accelerationNoiseY = noiseRandomizer.nextFloat();
            accelerationNoiseZ = noiseRandomizer.nextFloat();

            angularNoiseX = noiseRandomizer.nextFloat();
            angularNoiseY = noiseRandomizer.nextFloat();
            angularNoiseZ = noiseRandomizer.nextFloat();

            accelerationX = accelerationOffsetX + accelerationNoiseX;
            accelerationY = accelerationOffsetY + accelerationNoiseY;
            accelerationZ = accelerationOffsetZ + accelerationNoiseZ;

            angularX = angularOffsetX + angularNoiseX;
            angularY = angularOffsetY + angularNoiseY;
            angularZ = angularOffsetZ + angularNoiseZ;

            calibrator.updateAccelerometerSample(timestamp,
                    (float) accelerationX, (float) accelerationY, (float) accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float) angularX, (float) angularY, (float) angularZ);
            calibrator.updateOrientationSample(timestamp, orientation);

            if (calibrator.isFinished()) {
                break;
            }

            timestamp += DELTA_NANOS;
        }

        assertTrue(calibrator.isConverged());
        assertTrue(calibrator.isFinished());
        assertFalse(calibrator.isFailed());

        final Matrix cov = calibrator.getControlCovariance();

        final Matrix jacobian = Matrix.identity(calibrator.getEstimatorStateLength(),
                calibrator.getSampleLength());
        jacobian.multiplyByScalar(2.0);
        final MultivariateNormalDist dist = calibrator.propagateWithControlJacobian(jacobian);
        final MultivariateNormalDist dist2 = new MultivariateNormalDist();
        calibrator.propagateWithControlJacobian(jacobian, dist2);

        // check correctness
        final Matrix propagatedCov = jacobian.multiplyAndReturnNew(cov).
                multiplyAndReturnNew(jacobian.transposeAndReturnNew());

        assertTrue(dist.getCovariance().equals(propagatedCov, ABSOLUTE_ERROR));
        assertTrue(dist2.getCovariance().equals(propagatedCov, ABSOLUTE_ERROR));
    }

    @Test
    public void testUpdateAccelerometerSampleWithAccumulationDisabled() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();
        calibrator.setAccumulationEnabled(false);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        long timestamp = System.currentTimeMillis();
        final float accelerationX = randomizer.nextFloat();
        final float accelerationY = randomizer.nextFloat();
        final float accelerationZ = randomizer.nextFloat();

        // check initial values
        assertEquals(-1, calibrator.getAccelerometerTimestampNanos());
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleZ(), 0.0);
        assertEquals(0, calibrator.getAccumulatedAccelerometerSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        calibrator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);

        // check correctness
        assertEquals(timestamp, calibrator.getAccelerometerTimestampNanos());
        assertEquals(accelerationX, calibrator.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(accelerationY, calibrator.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(accelerationZ, calibrator.getAccumulatedAccelerationSampleZ(), 0.0);
        assertEquals(1, calibrator.getAccumulatedAccelerometerSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        // test again but using an array
        final float[] acceleration = new float[3];
        randomizer.fill(acceleration);
        timestamp += 1000;

        calibrator.updateAccelerometerSample(timestamp, acceleration);

        // check correctness
        assertEquals(timestamp, calibrator.getAccelerometerTimestampNanos());
        assertEquals(acceleration[0], calibrator.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(acceleration[1], calibrator.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(acceleration[2], calibrator.getAccumulatedAccelerationSampleZ(), 0.0);
        assertEquals(2, calibrator.getAccumulatedAccelerometerSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        // Force IllegalArgumentException
        final float[] wrong = new float[4];
        try {
            calibrator.updateAccelerometerSample(timestamp, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testUpdateAccelerometerSampleWithAccumulationEnabled() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();
        calibrator.setAccumulationEnabled(true);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(-1, calibrator.getAccelerometerTimestampNanos());
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleZ(), 0.0);
        assertEquals(0, calibrator.getAccumulatedAccelerometerSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        // update with several samples
        long timestamp = System.currentTimeMillis();
        float accelerationX;
        float accelerationY;
        float accelerationZ;
        double avgAccelerationX = 0.0;
        double avgAccelerationY = 0.0;
        double avgAccelerationZ = 0.0;
        for (int i = 0; i < TIMES; i++) {
            timestamp += 1000;
            accelerationX = randomizer.nextFloat();
            accelerationY = randomizer.nextFloat();
            accelerationZ = randomizer.nextFloat();

            avgAccelerationX += accelerationX / TIMES;
            avgAccelerationY += accelerationY / TIMES;
            avgAccelerationZ += accelerationZ / TIMES;

            calibrator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
        }

        // check correctness
        assertEquals(timestamp, calibrator.getAccelerometerTimestampNanos());
        assertEquals(avgAccelerationX, calibrator.getAccumulatedAccelerationSampleX(), ABSOLUTE_ERROR);
        assertEquals(avgAccelerationY, calibrator.getAccumulatedAccelerationSampleY(), ABSOLUTE_ERROR);
        assertEquals(avgAccelerationZ, calibrator.getAccumulatedAccelerationSampleZ(), ABSOLUTE_ERROR);
        assertEquals(TIMES, calibrator.getAccumulatedAccelerometerSamples());
        assertFalse(calibrator.isFullSampleAvailable());
    }

    @Test
    public void testUpdateGyroscopeSampleWithAccumulationDisabled() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();
        calibrator.setAccumulationEnabled(false);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        long timestamp = System.currentTimeMillis();
        final float angularSpeedX = randomizer.nextFloat();
        final float angularSpeedY = randomizer.nextFloat();
        final float angularSpeedZ = randomizer.nextFloat();

        // check initial values
        assertEquals(-1, calibrator.getGyroscopeTimestampNanos());
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertEquals(0, calibrator.getAccumulatedGyroscopeSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        calibrator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

        // check correctness
        assertEquals(timestamp, calibrator.getGyroscopeTimestampNanos());
        assertEquals(angularSpeedX, calibrator.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(angularSpeedY, calibrator.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(angularSpeedZ, calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertEquals(1, calibrator.getAccumulatedGyroscopeSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        // test again but using an array
        final float[] angularSpeed = new float[3];
        randomizer.fill(angularSpeed);
        timestamp += 100;

        calibrator.updateGyroscopeSample(timestamp, angularSpeed);

        // check correctness
        assertEquals(timestamp, calibrator.getGyroscopeTimestampNanos());
        assertEquals(angularSpeed[0], calibrator.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(angularSpeed[1], calibrator.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(angularSpeed[2], calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertEquals(2, calibrator.getAccumulatedGyroscopeSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        // Force IllegalArgumentException
        final float[] wrong = new float[4];
        try {
            calibrator.updateGyroscopeSample(timestamp, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testUpdateGyroscopeSampleWithAccumulationEnabled() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();
        calibrator.setAccumulationEnabled(true);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(-1, calibrator.getGyroscopeTimestampNanos());
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertEquals(0, calibrator.getAccumulatedGyroscopeSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        // update with several samples
        long timestamp = System.currentTimeMillis();
        float angularSpeedX;
        float angularSpeedY;
        float angularSpeedZ;
        double avgAngularSpeedX = 0.0;
        double avgAngularSpeedY = 0.0;
        double avgAngularSpeedZ = 0.0;
        for (int i = 0; i < TIMES; i++) {
            timestamp += 1000;
            angularSpeedX = randomizer.nextFloat();
            angularSpeedY = randomizer.nextFloat();
            angularSpeedZ = randomizer.nextFloat();

            avgAngularSpeedX += angularSpeedX / TIMES;
            avgAngularSpeedY += angularSpeedY / TIMES;
            avgAngularSpeedZ += angularSpeedZ / TIMES;

            calibrator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
        }

        // check correctness
        assertEquals(timestamp, calibrator.getGyroscopeTimestampNanos());
        assertEquals(avgAngularSpeedX, calibrator.getAccumulatedAngularSpeedSampleX(), ABSOLUTE_ERROR);
        assertEquals(avgAngularSpeedY, calibrator.getAccumulatedAngularSpeedSampleY(), ABSOLUTE_ERROR);
        assertEquals(avgAngularSpeedZ, calibrator.getAccumulatedAngularSpeedSampleZ(), ABSOLUTE_ERROR);
        assertEquals(TIMES, calibrator.getAccumulatedGyroscopeSamples());
        assertFalse(calibrator.isFullSampleAvailable());
    }

    @Test
    public void testUpdateOrientationSampleWithAccumulationDisabled() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();
        calibrator.setAccumulationEnabled(false);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final long timestamp = System.currentTimeMillis();
        final double orientationA = randomizer.nextDouble();
        final double orientationB = randomizer.nextDouble();
        final double orientationC = randomizer.nextDouble();
        final double orientationD = randomizer.nextDouble();
        final Quaternion orientation = new Quaternion(orientationA, orientationB, orientationC, orientationD);

        // check initial values
        assertEquals(-1, calibrator.getOrientationTimestampNanos());
        Quaternion accumulatedOrientation = (Quaternion) calibrator.getAccumulatedOrientation();
        assertEquals(1.0, accumulatedOrientation.getA(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getB(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getC(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getD(), 0.0);
        assertEquals(0, calibrator.getAccumulatedOrientationSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        final Quaternion accumulatedOrientation2 = new Quaternion();
        calibrator.getAccumulatedOrientation(accumulatedOrientation2);
        assertEquals(accumulatedOrientation, accumulatedOrientation2);

        calibrator.updateOrientationSample(timestamp, orientation);

        // check correctness
        assertEquals(timestamp, calibrator.getOrientationTimestampNanos());
        accumulatedOrientation = (Quaternion) calibrator.getAccumulatedOrientation();
        assertEquals(orientationA, accumulatedOrientation.getA(), 0.0);
        assertEquals(orientationB, accumulatedOrientation.getB(), 0.0);
        assertEquals(orientationC, accumulatedOrientation.getC(), 0.0);
        assertEquals(orientationD, accumulatedOrientation.getD(), 0.0);

        calibrator.getAccumulatedOrientation(accumulatedOrientation2);
        assertEquals(accumulatedOrientation, accumulatedOrientation2);
    }

    @Test
    public void testUpdateOrientationSampleWithAccumulationEnabled() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();
        calibrator.setAccumulationEnabled(true);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(-1, calibrator.getOrientationTimestampNanos());
        Quaternion accumulatedOrientation = (Quaternion) calibrator.getAccumulatedOrientation();
        assertEquals(1.0, accumulatedOrientation.getA(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getB(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getC(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getD(), 0.0);
        assertEquals(0, calibrator.getAccumulatedOrientationSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        final Quaternion accumulatedOrientation2 = new Quaternion();
        calibrator.getAccumulatedOrientation(accumulatedOrientation2);
        assertEquals(accumulatedOrientation, accumulatedOrientation2);

        // update with several samples
        long timestamp = System.currentTimeMillis();
        double orientationA;
        double orientationB;
        double orientationC;
        double orientationD;
        double avgOrientationA = 0.0;
        double avgOrientationB = 0.0;
        double avgOrientationC = 0.0;
        double avgOrientationD = 0.0;
        final Quaternion orientation = new Quaternion();
        double norm;
        for (int i = 0; i < TIMES; i++) {
            timestamp += 1000;
            orientationA = randomizer.nextDouble();
            orientationB = randomizer.nextDouble();
            orientationC = randomizer.nextDouble();
            orientationD = randomizer.nextDouble();
            norm = Math.sqrt(orientationA * orientationA +
                    orientationB * orientationB +
                    orientationC * orientationC +
                    orientationD * orientationD);
            orientationA /= norm;
            orientationB /= norm;
            orientationC /= norm;
            orientationD /= norm;

            avgOrientationA += orientationA / TIMES;
            avgOrientationB += orientationB / TIMES;
            avgOrientationC += orientationC / TIMES;
            avgOrientationD += orientationD / TIMES;

            orientation.setA(orientationA);
            orientation.setB(orientationB);
            orientation.setC(orientationC);
            orientation.setD(orientationD);

            calibrator.updateOrientationSample(timestamp, orientation);
        }

        // check correctness
        assertEquals(calibrator.getOrientationTimestampNanos(), timestamp);
        accumulatedOrientation = (Quaternion) calibrator.getAccumulatedOrientation();
        assertEquals(avgOrientationA, accumulatedOrientation.getA(), ABSOLUTE_ERROR);
        assertEquals(avgOrientationB, accumulatedOrientation.getB(), ABSOLUTE_ERROR);
        assertEquals(avgOrientationC, accumulatedOrientation.getC(), ABSOLUTE_ERROR);
        assertEquals(avgOrientationD, accumulatedOrientation.getD(), ABSOLUTE_ERROR);

        calibrator.getAccumulatedOrientation(accumulatedOrientation2);
        assertEquals(accumulatedOrientation, accumulatedOrientation2);
    }

    @Test
    public void testCalibrationWithOffset() throws SignalProcessingException {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();
        calibrator.setListener(this);

        // setup so that calibrator doesn't reach convergence
        calibrator.setMaxNumSamples(N_SAMPLES);
        calibrator.setConvergenceThreshold(0.0);

        final UniformRandomizer offsetRandomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, NOISE_DEVIATION);

        final float accelerationOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final float accelerationOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final float accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);

        final float angularOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final float angularOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final float angularOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);

        long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;

        float accelerationNoiseX;
        float accelerationNoiseY;
        float accelerationNoiseZ;
        float angularNoiseX;
        float angularNoiseY;
        float angularNoiseZ;

        double accelerationX = 0.0;
        double accelerationY = 0.0;
        double accelerationZ = 0.0;
        double angularX = 0.0;
        double angularY = 0.0;
        double angularZ = 0.0;
        double deltaAngularX;
        double deltaAngularY;
        double deltaAngularZ;
        double lastAngularX = 0.0;
        double lastAngularY = 0.0;
        double lastAngularZ = 0.0;
        final Quaternion orientation = new Quaternion();

        calibrator.reset();
        reset();

        final MeasurementNoiseCovarianceEstimator estimator =
                new MeasurementNoiseCovarianceEstimator(10);
        final double[] sample = new double[10];

        for (int i = 0; i < N_SAMPLES; i++) {
            accelerationNoiseX = noiseRandomizer.nextFloat();
            accelerationNoiseY = noiseRandomizer.nextFloat();
            accelerationNoiseZ = noiseRandomizer.nextFloat();

            angularNoiseX = noiseRandomizer.nextFloat();
            angularNoiseY = noiseRandomizer.nextFloat();
            angularNoiseZ = noiseRandomizer.nextFloat();

            accelerationX += accelerationOffsetX + accelerationNoiseX;
            accelerationY += accelerationOffsetY + accelerationNoiseY;
            accelerationZ += accelerationOffsetZ + accelerationNoiseZ;

            angularX += angularOffsetX + angularNoiseX;
            angularY += angularOffsetY + angularNoiseY;
            angularZ += angularOffsetZ + angularNoiseZ;

            calibrator.updateAccelerometerSample(timestamp, (float) accelerationX, (float) accelerationY,
                    (float) accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float) angularX, (float) angularY, (float) angularZ);
            calibrator.updateOrientationSample(timestamp, orientation);

            timestamp += DELTA_NANOS;

            if (i != 0) {
                deltaAngularX = angularX - lastAngularX;
                deltaAngularY = angularY - lastAngularY;
                deltaAngularZ = angularZ - lastAngularZ;

                sample[0] = orientation.getA();
                sample[1] = orientation.getB();
                sample[2] = orientation.getC();
                sample[3] = orientation.getD();
                sample[4] = accelerationX * DELTA_SECONDS;
                sample[5] = accelerationY * DELTA_SECONDS;
                sample[6] = accelerationZ * DELTA_SECONDS;
                sample[7] = deltaAngularX;
                sample[8] = deltaAngularY;
                sample[9] = deltaAngularZ;
                estimator.update(sample);
            }

            lastAngularX = angularX;
            lastAngularY = angularY;
            lastAngularZ = angularZ;
        }

        final double[] mean = calibrator.getControlMean();
        final double[] mean2 = estimator.getSampleAverage();

        assertArrayEquals(mean, mean2, LARGE_ABSOLUTE_ERROR);
        assertEquals(1.0, mean[0], ABSOLUTE_ERROR);
        assertEquals(0.0, mean[1], ABSOLUTE_ERROR);
        assertEquals(0.0, mean[2], ABSOLUTE_ERROR);
        assertEquals(0.0, mean[3], ABSOLUTE_ERROR);
        assertEquals(accelerationOffsetX / DELTA_SECONDS * 2.0, mean[4],
                VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(accelerationOffsetY / DELTA_SECONDS * 2.0, mean[5],
                VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(accelerationOffsetZ / DELTA_SECONDS * 2.0, mean[6],
                VERY_LARGE_ABSOLUTE_ERROR);
        assertEquals(angularOffsetX, mean[7], LARGE_ABSOLUTE_ERROR);
        assertEquals(angularOffsetY, mean[8], LARGE_ABSOLUTE_ERROR);
        assertEquals(angularOffsetZ, mean[9], LARGE_ABSOLUTE_ERROR);

        assertFalse(calibrator.isFailed());
        assertFalse(calibrator.isFinished());
        assertFalse(calibrator.isConverged());
        assertEquals(N_SAMPLES, fullSampleReceived);
        assertEquals(N_SAMPLES, fullSampleProcessed);
        assertEquals(0, calibratorFinished);

        // add one last sample
        calibrator.updateAccelerometerSample(timestamp, (float) accelerationX,
                (float) accelerationY, (float) accelerationZ);
        calibrator.updateGyroscopeSample(timestamp, (float) angularX, (float) angularY, (float) angularZ);
        calibrator.updateOrientationSample(timestamp, orientation);

        // check
        assertFalse(calibrator.isFailed());
        assertTrue(calibrator.isFinished());
        assertFalse(calibrator.isConverged());
        assertEquals(N_SAMPLES + 1, fullSampleReceived);
        assertEquals(N_SAMPLES + 1, fullSampleProcessed);
        assertEquals(1, calibratorFinished);
    }

    @Test
    public void testCalibrationWithoutOffset() throws SignalProcessingException {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();
        calibrator.setListener(this);

        // setup so that calibrator doesn't reach convergence
        calibrator.setMaxNumSamples(N_SAMPLES);
        calibrator.setConvergenceThreshold(0.0);

        final UniformRandomizer offsetRandomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, NOISE_DEVIATION);

        final float accelerationOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final float accelerationOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final float accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);

        final float angularOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final float angularOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final float angularOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);

        long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;

        float accelerationNoiseX;
        float accelerationNoiseY;
        float accelerationNoiseZ;
        float angularNoiseX;
        float angularNoiseY;
        float angularNoiseZ;

        double accelerationX;
        double accelerationY;
        double accelerationZ;
        double angularX;
        double angularY;
        double angularZ;
        double deltaAngularX;
        double deltaAngularY;
        double deltaAngularZ;
        double lastAngularX = 0.0;
        double lastAngularY = 0.0;
        double lastAngularZ = 0.0;
        final Quaternion orientation = new Quaternion();

        calibrator.reset();
        reset();

        final MeasurementNoiseCovarianceEstimator estimator =
                new MeasurementNoiseCovarianceEstimator(10);
        final double[] sample = new double[10];

        for (int i = 0; i < N_SAMPLES; i++) {
            accelerationNoiseX = noiseRandomizer.nextFloat();
            accelerationNoiseY = noiseRandomizer.nextFloat();
            accelerationNoiseZ = noiseRandomizer.nextFloat();

            angularNoiseX = noiseRandomizer.nextFloat();
            angularNoiseY = noiseRandomizer.nextFloat();
            angularNoiseZ = noiseRandomizer.nextFloat();

            accelerationX = accelerationOffsetX + accelerationNoiseX;
            accelerationY = accelerationOffsetY + accelerationNoiseY;
            accelerationZ = accelerationOffsetZ + accelerationNoiseZ;

            angularX = angularOffsetX + angularNoiseX;
            angularY = angularOffsetY + angularNoiseY;
            angularZ = angularOffsetZ + angularNoiseZ;

            calibrator.updateAccelerometerSample(timestamp, (float) accelerationX,
                    (float) accelerationY, (float) accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float) angularX, (float) angularY, (float) angularZ);
            calibrator.updateOrientationSample(timestamp, orientation);

            timestamp += DELTA_NANOS;

            if (i != 0) {
                deltaAngularX = angularX - lastAngularX;
                deltaAngularY = angularY - lastAngularY;
                deltaAngularZ = angularZ - lastAngularZ;

                sample[0] = orientation.getA();
                sample[1] = orientation.getB();
                sample[2] = orientation.getC();
                sample[3] = orientation.getD();
                sample[4] = accelerationX * DELTA_SECONDS;
                sample[5] = accelerationY * DELTA_SECONDS;
                sample[6] = accelerationZ * DELTA_SECONDS;
                sample[7] = deltaAngularX;
                sample[8] = deltaAngularY;
                sample[9] = deltaAngularZ;
                estimator.update(sample);
            }

            lastAngularX = angularX;
            lastAngularY = angularY;
            lastAngularZ = angularZ;
        }

        final double[] mean = calibrator.getControlMean();
        final double[] mean2 = estimator.getSampleAverage();

        final Matrix cov = calibrator.getControlCovariance();
        final Matrix cov2 = estimator.getMeasurementNoiseCov();

        assertArrayEquals(mean, mean2, LARGE_ABSOLUTE_ERROR);

        assertTrue(cov.equals(cov2, ABSOLUTE_ERROR));

        assertFalse(calibrator.isFailed());
        assertFalse(calibrator.isFinished());
        assertFalse(calibrator.isConverged());
        assertEquals(N_SAMPLES, fullSampleReceived);
        assertEquals(N_SAMPLES, fullSampleProcessed);
        assertEquals(0, calibratorFinished);

        // add one last sample
        calibrator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
        calibrator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
        calibrator.updateOrientationSample(timestamp, orientation);

        // check
        assertFalse(calibrator.isFailed());
        assertTrue(calibrator.isFinished());
        assertFalse(calibrator.isConverged());
        assertEquals(N_SAMPLES + 1, fullSampleReceived);
        assertEquals(N_SAMPLES + 1, fullSampleProcessed);
        assertEquals(1, calibratorFinished);
    }

    @Test
    public void testCalibrationConvergence() {
        final AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrator();
        calibrator.setListener(this);

        final UniformRandomizer offsetRandomizer = new UniformRandomizer(new Random());
        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, NOISE_DEVIATION);

        final float accelerationOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final float accelerationOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final float accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);

        final float angularOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final float angularOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final float angularOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);

        long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;

        float accelerationNoiseX;
        float accelerationNoiseY;
        float accelerationNoiseZ;
        float angularNoiseX;
        float angularNoiseY;
        float angularNoiseZ;

        double accelerationX;
        double accelerationY;
        double accelerationZ;
        double angularX;
        double angularY;
        double angularZ;
        final Quaternion orientation = new Quaternion();

        calibrator.reset();
        reset();

        for (int i = 0; i < N_SAMPLES; i++) {
            accelerationNoiseX = noiseRandomizer.nextFloat();
            accelerationNoiseY = noiseRandomizer.nextFloat();
            accelerationNoiseZ = noiseRandomizer.nextFloat();

            angularNoiseX = noiseRandomizer.nextFloat();
            angularNoiseY = noiseRandomizer.nextFloat();
            angularNoiseZ = noiseRandomizer.nextFloat();

            accelerationX = accelerationOffsetX + accelerationNoiseX;
            accelerationY = accelerationOffsetY + accelerationNoiseY;
            accelerationZ = accelerationOffsetZ + accelerationNoiseZ;

            angularX = angularOffsetX + angularNoiseX;
            angularY = angularOffsetY + angularNoiseY;
            angularZ = angularOffsetZ + angularNoiseZ;

            calibrator.updateAccelerometerSample(timestamp, (float) accelerationX,
                    (float) accelerationY, (float) accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float) angularX, (float) angularY, (float) angularZ);
            calibrator.updateOrientationSample(timestamp, orientation);

            if (calibrator.isFinished()) break;

            timestamp += DELTA_NANOS;
        }

        assertTrue(calibrator.isConverged());
        assertTrue(calibrator.isFinished());
        assertFalse(calibrator.isFailed());

        assertTrue(fullSampleReceived < N_SAMPLES);
        assertTrue(fullSampleProcessed < N_SAMPLES);
        assertEquals(fullSampleReceived, fullSampleProcessed);
        assertEquals(1, calibratorFinished);
    }

    @Override
    public void onFullSampleReceived(
            final BaseSlamCalibrator<AbsoluteOrientationConstantVelocityModelSlamCalibrationData> calibrator) {
        fullSampleReceived++;
    }

    @Override
    public void onFullSampleProcessed(
            final BaseSlamCalibrator<AbsoluteOrientationConstantVelocityModelSlamCalibrationData> calibrator) {
        fullSampleProcessed++;
    }

    @Override
    public void onCalibratorFinished(
            final BaseSlamCalibrator<AbsoluteOrientationConstantVelocityModelSlamCalibrationData> calibrator,
            final boolean converged, final boolean failed) {
        calibratorFinished++;
    }

    private void reset() {
        fullSampleReceived = fullSampleProcessed = calibratorFinished = 0;
    }
}
