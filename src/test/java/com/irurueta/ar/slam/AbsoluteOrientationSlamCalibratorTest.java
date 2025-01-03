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
import com.irurueta.ar.slam.BaseSlamCalibrator.BaseSlamCalibratorListener;
import com.irurueta.geometry.Quaternion;
import com.irurueta.numerical.signal.processing.MeasurementNoiseCovarianceEstimator;
import com.irurueta.numerical.signal.processing.SignalProcessingException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.InvalidCovarianceMatrixException;
import com.irurueta.statistics.MultivariateNormalDist;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class AbsoluteOrientationSlamCalibratorTest implements
        BaseSlamCalibratorListener<AbsoluteOrientationSlamCalibrationData> {

    private static final int TIMES = 50;
    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final float MIN_OFFSET = -10.0f;
    private static final float MAX_OFFSET = 10.0f;

    private static final float NOISE_DEVIATION = 1e-5f;

    private static final int N_SAMPLES = 10000;

    // conversion from milliseconds to nanoseconds
    private static final int MILLIS_TO_NANOS = 1000000;

    // time between samples expressed in nanoseconds (a typical sensor in Android
    // delivers a sample every 20ms)
    private static final int DELTA_NANOS = 20000000; // 0.02 seconds

    private int fullSampleReceived;
    private int fullSampleProcessed;
    private int calibratorFinished;

    @Test
    void testConstructor() throws WrongSizeException {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // check initial values
        assertEquals(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH, calibrator.getSampleLength());
        assertEquals(AbsoluteOrientationSlamEstimator.STATE_LENGTH, calibrator.getEstimatorStateLength());
        assertFalse(calibrator.isConverged());
        assertFalse(calibrator.isFailed());
        assertFalse(calibrator.isFinished());
        assertEquals(0, calibrator.getSampleCount());
        assertEquals(SlamCalibrator.DEFAULT_MIN_NUM_SAMPLES, calibrator.getMinNumSamples());
        assertEquals(SlamCalibrator.DEFAULT_MAX_NUM_SAMPLES, calibrator.getMaxNumSamples());
        assertEquals(SlamCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, calibrator.getConvergenceThreshold(), 0.0);
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
        final var accumAcceleration = new double[3];
        calibrator.getAccumulatedAccelerationSample(accumAcceleration);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, accumAcceleration, 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, calibrator.getAccumulatedAngularSpeedSample(), 0.0);
        final var accumAngularSpeed = new double[3];
        calibrator.getAccumulatedAngularSpeedSample(accumAngularSpeed);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, accumAngularSpeed, 0.0);
        assertNull(calibrator.getListener());
        assertArrayEquals(new double[AbsoluteOrientationSlamEstimator.CONTROL_LENGTH], calibrator.getControlMean(),
                0.0);
        final var controlMean = new double[AbsoluteOrientationSlamEstimator.CONTROL_LENGTH];
        calibrator.getControlMean(controlMean);
        assertArrayEquals(new double[AbsoluteOrientationSlamEstimator.CONTROL_LENGTH], controlMean, 0.0);
        assertEquals(new Matrix(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH,
                        AbsoluteOrientationSlamEstimator.CONTROL_LENGTH), calibrator.getControlCovariance());
        final var cov = new Matrix(1, 1);
        calibrator.getControlCovariance(cov);
        assertEquals(new Matrix(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH), cov);
    }

    @Test
    void testIsConverged() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertFalse(calibrator.isConverged());

        // set new value
        calibrator.converged = true;

        // check correctness
        assertTrue(calibrator.isConverged());
    }

    @Test
    void testIsFailed() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertFalse(calibrator.isFailed());

        // set new value
        calibrator.failed = true;

        // check correctness
        assertTrue(calibrator.isFailed());
    }

    @Test
    void testIsFinished() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertFalse(calibrator.isFinished());

        // set new value
        calibrator.finished = true;

        // check correctness
        assertTrue(calibrator.isFinished());
    }

    @Test
    void testGetSampleCount() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertEquals(0, calibrator.getSampleCount());

        // set new value
        calibrator.sampleCount = 5;

        // check correctness
        assertEquals(5, calibrator.getSampleCount());
    }

    @Test
    void testGetSetMinNumSamples() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertEquals(SlamCalibrator.DEFAULT_MIN_NUM_SAMPLES, calibrator.getMinNumSamples());

        // set new value
        calibrator.setMinNumSamples(50);

        // check correctness
        assertEquals(50, calibrator.getMinNumSamples());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setMinNumSamples(-1));
    }

    @Test
    void testGetSetMaxNumSamples() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertEquals(SlamCalibrator.DEFAULT_MAX_NUM_SAMPLES, calibrator.getMaxNumSamples());

        // set new value
        calibrator.setMaxNumSamples(1000);

        // check correctness
        assertEquals(1000, calibrator.getMaxNumSamples());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setMaxNumSamples(-1));
    }

    @Test
    void testGetSetConvergenceThreshold() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertEquals(SlamCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, calibrator.getConvergenceThreshold(), 0.0);

        // set new value
        calibrator.setConvergenceThreshold(1.0);

        // check correctness
        assertEquals(1.0, calibrator.getConvergenceThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setConvergenceThreshold(-0.1));
    }

    @Test
    void testReset() throws WrongSizeException {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        calibrator.reset();

        // check correctness
        assertFalse(calibrator.isConverged());
        assertFalse(calibrator.isFailed());
        assertFalse(calibrator.isFinished());
        assertEquals(0, calibrator.getSampleCount());
        assertEquals(SlamCalibrator.DEFAULT_MIN_NUM_SAMPLES, calibrator.getMinNumSamples());
        assertEquals(SlamCalibrator.DEFAULT_MAX_NUM_SAMPLES, calibrator.getMaxNumSamples());
        assertEquals(SlamCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, calibrator.getConvergenceThreshold(), 0.0);
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
        final var accumAcceleration = new double[3];
        calibrator.getAccumulatedAccelerationSample(accumAcceleration);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, accumAcceleration, 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, calibrator.getAccumulatedAngularSpeedSample(), 0.0);
        final var accumAngularSpeed = new double[3];
        calibrator.getAccumulatedAngularSpeedSample(accumAngularSpeed);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, accumAngularSpeed, 0.0);
        assertNull(calibrator.getListener());
        assertArrayEquals(new double[AbsoluteOrientationSlamEstimator.CONTROL_LENGTH], calibrator.getControlMean(),
                0.0);
        final var controlMean = new double[AbsoluteOrientationSlamEstimator.CONTROL_LENGTH];
        calibrator.getControlMean(controlMean);
        assertArrayEquals(new double[AbsoluteOrientationSlamEstimator.CONTROL_LENGTH], controlMean, 0.0);
        assertEquals(new Matrix(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH), calibrator.getControlCovariance());
        final var cov = new Matrix(1, 1);
        calibrator.getControlCovariance(cov);
        assertEquals(new Matrix(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH), cov);
    }

    @Test
    void testIsSetAccumulationEnabled() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertTrue(calibrator.isAccumulationEnabled());

        // set new value
        calibrator.setAccumulationEnabled(false);

        // check correctness
        assertFalse(calibrator.isAccumulationEnabled());
    }

    @Test
    void testGetAccelerometerTimestampNanos() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertEquals(-1, calibrator.getAccelerometerTimestampNanos());

        // set new value
        calibrator.accelerometerTimestampNanos = 1000;

        // check correctness
        assertEquals(1000, calibrator.getAccelerometerTimestampNanos());
    }

    @Test
    void testGetGyroscopeTimestampNanos() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertEquals(-1, calibrator.getGyroscopeTimestampNanos());

        // set new value
        calibrator.gyroscopeTimestampNanos = 2000;

        // check correctness
        assertEquals(2000, calibrator.getGyroscopeTimestampNanos());
    }

    @Test
    void testGetAccumulatedAccelerometerSamplesAndIsAccelerometerSampleReceived() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertEquals(0, calibrator.getAccumulatedAccelerometerSamples());
        assertFalse(calibrator.isAccelerometerSampleReceived());

        // set new value
        calibrator.accumulatedAccelerometerSamples = 50;

        // check correctness
        assertEquals(50, calibrator.getAccumulatedAccelerometerSamples());
        assertTrue(calibrator.isAccelerometerSampleReceived());
    }

    @Test
    void testGetAccumulatedGyroscopeSamples() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertEquals(0, calibrator.getAccumulatedGyroscopeSamples());
        assertFalse(calibrator.isGyroscopeSampleReceived());

        // set new value
        calibrator.accumulatedGyroscopeSamples = 500;

        // check correctness
        assertEquals(500, calibrator.getAccumulatedGyroscopeSamples());
        assertTrue(calibrator.isGyroscopeSampleReceived());
    }

    @Test
    void testIsFullSampleAvailable() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial values
        assertEquals(0, calibrator.getAccumulatedAccelerometerSamples());
        assertEquals(0, calibrator.getAccumulatedGyroscopeSamples());
        assertEquals(0, calibrator.getAccumulatedOrientationSamples());
        assertFalse(calibrator.isAccelerometerSampleReceived());
        assertFalse(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isOrientationSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());

        // set accelerometer sample
        calibrator.accumulatedAccelerometerSamples = 1;

        // check correctness
        assertEquals(1, calibrator.getAccumulatedAccelerometerSamples());
        assertEquals(0, calibrator.getAccumulatedGyroscopeSamples());
        assertEquals(0, calibrator.getAccumulatedOrientationSamples());
        assertTrue(calibrator.isAccelerometerSampleReceived());
        assertFalse(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isOrientationSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());

        // set gyroscope sample
        calibrator.accumulatedGyroscopeSamples = 1;

        // check correctness
        assertEquals(1, calibrator.getAccumulatedAccelerometerSamples());
        assertEquals(1, calibrator.getAccumulatedGyroscopeSamples());
        assertEquals(0, calibrator.getAccumulatedOrientationSamples());
        assertTrue(calibrator.isAccelerometerSampleReceived());
        assertTrue(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isOrientationSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());

        // set orientation sample
        calibrator.accumulatedOrientationSamples = 1;

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
    void testGetAccumulatedAccelerationSampleX() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleX(), 0.0);

        // set new value
        calibrator.accumulatedAccelerationSampleX = 1.0;

        // check correctness
        assertEquals(1.0, calibrator.getAccumulatedAccelerationSampleX(), 0.0);
    }

    @Test
    void testGetAccumulatedAccelerationSampleY() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleY(), 0.0);

        // set new value
        calibrator.accumulatedAccelerationSampleY = 2.0;

        // check correctness
        assertEquals(2.0, calibrator.getAccumulatedAccelerationSampleY(), 0.0);
    }

    @Test
    void testGetAccumulatedAccelerationSampleZ() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleZ(), 0.0);

        // set new value
        calibrator.accumulatedAccelerationSampleZ = 3.0;

        // check correctness
        assertEquals(3.0, calibrator.getAccumulatedAccelerationSampleZ(), 0.0);
    }

    @Test
    void testGetAccumulatedAccelerationSample() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, calibrator.getAccumulatedAccelerationSample(), 0.0);

        // set new value
        calibrator.accumulatedAccelerationSampleX = 1.0;
        calibrator.accumulatedAccelerationSampleY = 2.0;
        calibrator.accumulatedAccelerationSampleZ = 3.0;

        // check correctness
        assertArrayEquals(new double[]{1.0, 2.0, 3.0}, calibrator.getAccumulatedAccelerationSample(), 0.0);

        final var sample = new double[3];
        calibrator.getAccumulatedAccelerationSample(sample);

        // check correctness
        assertArrayEquals(new double[]{1.0, 2.0, 3.0}, sample, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getAccumulatedAccelerationSample(new double[1]));
    }

    @Test
    void testGetAccumulatedAngularSpeedSampleX() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleX(), 0.0);

        // set new value
        calibrator.accumulatedAngularSpeedSampleX = 1.0;

        // check correctness
        assertEquals(1.0, calibrator.getAccumulatedAngularSpeedSampleX(), 0.0);
    }

    @Test
    void testGetAccumulatedAngularSpeedSampleY() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleY(), 0.0);

        // set new value
        calibrator.accumulatedAngularSpeedSampleY = 2.0;

        // check correctness
        assertEquals(2.0, calibrator.getAccumulatedAngularSpeedSampleY(), 0.0);
    }

    @Test
    void testGetAccumulatedAngularSpeedSampleZ() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0);

        // set new value
        calibrator.accumulatedAngularSpeedSampleZ = 3.0;

        // check correctness
        assertEquals(3.0, calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0);
    }

    @Test
    void testGetAccumulatedAngularSpeedSample() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, calibrator.getAccumulatedAngularSpeedSample(), 0.0);

        // set new value
        calibrator.accumulatedAngularSpeedSampleX = 1.0;
        calibrator.accumulatedAngularSpeedSampleY = 2.0;
        calibrator.accumulatedAngularSpeedSampleZ = 3.0;

        // check correctness
        assertArrayEquals(new double[]{1.0, 2.0, 3.0}, calibrator.getAccumulatedAngularSpeedSample(), 0.0);

        final var sample = new double[3];
        calibrator.getAccumulatedAngularSpeedSample(sample);

        // check correctness
        assertArrayEquals(new double[]{1.0, 2.0, 3.0}, sample, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getAccumulatedAngularSpeedSample(new double[1]));
    }

    @Test
    void testGetMostRecentTimestampNanos() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // check correctness
        assertEquals(-1, calibrator.getMostRecentTimestampNanos());

        // set new value
        calibrator.accelerometerTimestampNanos = 1000;

        // check correctness
        assertEquals(1000, calibrator.getMostRecentTimestampNanos());

        // set new value
        calibrator.gyroscopeTimestampNanos = 2000;

        // check correctness
        assertEquals(2000, calibrator.getMostRecentTimestampNanos());
    }

    @Test
    void testGetSetListener() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        // initial value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check correctness
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testGetControlMean() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        assertArrayEquals(new double[AbsoluteOrientationSlamEstimator.CONTROL_LENGTH], calibrator.getControlMean(),
                0.0);
        assertArrayEquals(calibrator.estimator.getSampleAverage(), calibrator.getControlMean(), 0.0);

        final var controlMean = new double[AbsoluteOrientationSlamEstimator.CONTROL_LENGTH];
        calibrator.getControlMean(controlMean);
        assertArrayEquals(controlMean, calibrator.getControlMean(), 0.0);

        // Force IllegalArgumentException
        final var wrong = new double[1];
        assertThrows(IllegalArgumentException.class, () -> calibrator.getControlMean(wrong));
    }

    @Test
    void testGetControlCovariance() throws WrongSizeException {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        assertEquals(new Matrix(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH,
                        AbsoluteOrientationSlamEstimator.CONTROL_LENGTH), calibrator.getControlCovariance());

        final var cov = new Matrix(1, 1);
        calibrator.getControlCovariance(cov);

        assertEquals(cov, calibrator.getControlCovariance());
    }

    @Test
    void testGetControlDistribution() throws InvalidCovarianceMatrixException {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        final var controlMean = calibrator.getControlMean();
        final var cov = calibrator.getControlCovariance();

        final var dist = calibrator.getControlDistribution();

        // check correctness
        assertArrayEquals(controlMean, dist.getMean(), ABSOLUTE_ERROR);
        assertTrue(cov.equals(dist.getCovariance(), ABSOLUTE_ERROR));

        final var dist2 = new MultivariateNormalDist();
        calibrator.getControlDistribution(dist2);

        // check correctness
        assertArrayEquals(controlMean, dist2.getMean(), ABSOLUTE_ERROR);
        assertTrue(cov.equals(dist2.getCovariance(), ABSOLUTE_ERROR));
    }

    @Test
    void testGetCalibrationData() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        final var controlMean = calibrator.getControlMean();
        final var cov = calibrator.getControlCovariance();

        final var data1 = calibrator.getCalibrationData();
        final var data2 = new AbsoluteOrientationSlamCalibrationData();
        calibrator.getCalibrationData(data2);

        // check correctness
        assertSame(controlMean, data1.getControlMean());
        assertSame(cov, data1.getControlCovariance());

        assertSame(controlMean, data2.getControlMean());
        assertSame(cov, data2.getControlCovariance());
    }

    @Test
    void testPropagateWithControlJacobian() throws WrongSizeException, InvalidCovarianceMatrixException {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();

        final var offsetRandomizer = new UniformRandomizer();
        final var noiseRandomizer = new GaussianRandomizer(0.0, NOISE_DEVIATION);

        final var accelerationOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final var accelerationOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final var accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);

        final var angularOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final var angularOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final var angularOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);

        var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;

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
        final var orientation = new Quaternion();

        calibrator.reset();

        for (var i = 0; i < N_SAMPLES; i++) {
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

            calibrator.updateAccelerometerSample(timestamp, (float) accelerationX, (float) accelerationY,
                    (float) accelerationZ);
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

        final var cov = calibrator.getControlCovariance();

        final var jacobian = Matrix.identity(calibrator.getEstimatorStateLength(), calibrator.getSampleLength());
        jacobian.multiplyByScalar(2.0);
        final var dist = calibrator.propagateWithControlJacobian(jacobian);
        final var dist2 = new MultivariateNormalDist();
        calibrator.propagateWithControlJacobian(jacobian, dist2);

        // check correctness
        final var propagatedCov = jacobian.multiplyAndReturnNew(cov).multiplyAndReturnNew(
                jacobian.transposeAndReturnNew());

        assertTrue(dist.getCovariance().equals(propagatedCov, ABSOLUTE_ERROR));
        assertTrue(dist2.getCovariance().equals(propagatedCov, ABSOLUTE_ERROR));
    }

    @Test
    void testUpdateAccelerometerSampleWithAccumulationDisabled() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();
        calibrator.setAccumulationEnabled(false);

        final var randomizer = new UniformRandomizer();

        var timestamp = System.currentTimeMillis();
        final var accelerationX = randomizer.nextFloat();
        final var accelerationY = randomizer.nextFloat();
        final var accelerationZ = randomizer.nextFloat();

        // check initial values
        assertEquals(-1, calibrator.getAccelerometerTimestampNanos());
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleZ(), 0.0);
        assertEquals(0, calibrator.getAccumulatedAccelerometerSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        calibrator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);

        // check correctness
        assertEquals(calibrator.getAccelerometerTimestampNanos(), timestamp);
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(), accelerationX, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(), accelerationY, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(), accelerationZ, 0.0);
        assertEquals(1, calibrator.getAccumulatedAccelerometerSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        // test again but using an array
        final var acceleration = new float[3];
        randomizer.fill(acceleration);
        timestamp += 1000;

        calibrator.updateAccelerometerSample(timestamp, acceleration);

        // check correctness
        assertEquals(calibrator.getAccelerometerTimestampNanos(), timestamp);
        assertEquals(acceleration[0], calibrator.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(acceleration[1], calibrator.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(acceleration[2], calibrator.getAccumulatedAccelerationSampleZ(), 0.0);
        assertEquals(2, calibrator.getAccumulatedAccelerometerSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        // Force IllegalArgumentException
        final var wrong = new float[4];
        final var finalTimestamp = timestamp;
        assertThrows(IllegalArgumentException.class, () -> calibrator.updateAccelerometerSample(finalTimestamp, wrong));
    }

    @Test
    void testUpdateAccelerometerSampleWithAccumulationEnabled() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();
        calibrator.setAccumulationEnabled(true);

        final var randomizer = new UniformRandomizer();

        // check initial values
        assertEquals(-1, calibrator.getAccelerometerTimestampNanos());
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAccelerationSampleZ(), 0.0);
        assertEquals(0, calibrator.getAccumulatedAccelerometerSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        // update with several samples
        var timestamp = System.currentTimeMillis();
        float accelerationX;
        float accelerationY;
        float accelerationZ;
        var avgAccelerationX = 0.0;
        var avgAccelerationY = 0.0;
        var avgAccelerationZ = 0.0;
        for (var i = 0; i < TIMES; i++) {
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
        assertEquals(calibrator.getAccelerometerTimestampNanos(), timestamp);
        assertEquals(avgAccelerationX, calibrator.getAccumulatedAccelerationSampleX(), ABSOLUTE_ERROR);
        assertEquals(avgAccelerationY, calibrator.getAccumulatedAccelerationSampleY(), ABSOLUTE_ERROR);
        assertEquals(avgAccelerationZ, calibrator.getAccumulatedAccelerationSampleZ(), ABSOLUTE_ERROR);
        assertEquals(TIMES, calibrator.getAccumulatedAccelerometerSamples());
        assertFalse(calibrator.isFullSampleAvailable());
    }

    @Test
    void testUpdateGyroscopeSampleWithAccumulationDisabled() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();
        calibrator.setAccumulationEnabled(false);

        final var randomizer = new UniformRandomizer();

        var timestamp = System.currentTimeMillis();
        final var angularSpeedX = randomizer.nextFloat();
        final var angularSpeedY = randomizer.nextFloat();
        final var angularSpeedZ = randomizer.nextFloat();

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
        final var angularSpeed = new float[3];
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
        final var wrong = new float[4];
        final var finalTimestamp = timestamp;
        assertThrows(IllegalArgumentException.class, () -> calibrator.updateGyroscopeSample(finalTimestamp, wrong));
    }

    @Test
    void testUpdateGyroscopeSampleWithAccumulationEnabled() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();
        calibrator.setAccumulationEnabled(true);

        final var randomizer = new UniformRandomizer();

        // check initial values
        assertEquals(-1, calibrator.getGyroscopeTimestampNanos());
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(0.0, calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertEquals(0, calibrator.getAccumulatedGyroscopeSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        // update with several samples
        var timestamp = System.currentTimeMillis();
        float angularSpeedX;
        float angularSpeedY;
        float angularSpeedZ;
        var avgAngularSpeedX = 0.0;
        var avgAngularSpeedY = 0.0;
        var avgAngularSpeedZ = 0.0;
        for (var i = 0; i < TIMES; i++) {
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
    void testUpdateOrientationSampleWithAccumulationDisabled() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();
        calibrator.setAccumulationEnabled(false);

        final var randomizer = new UniformRandomizer();

        final var timestamp = System.currentTimeMillis();
        final var orientationA = randomizer.nextDouble();
        final var orientationB = randomizer.nextDouble();
        final var orientationC = randomizer.nextDouble();
        final var orientationD = randomizer.nextDouble();
        final var orientation = new Quaternion(orientationA, orientationB, orientationC, orientationD);

        // check initial values
        assertEquals(-1, calibrator.getOrientationTimestampNanos());
        var accumulatedOrientation = (Quaternion) calibrator.getAccumulatedOrientation();
        assertEquals(1.0, accumulatedOrientation.getA(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getB(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getC(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getD(), 0.0);
        assertEquals(0, calibrator.getAccumulatedOrientationSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        final var accumulatedOrientation2 = new Quaternion();
        calibrator.getAccumulatedOrientation(accumulatedOrientation2);
        assertEquals(accumulatedOrientation, accumulatedOrientation2);

        calibrator.updateOrientationSample(timestamp, orientation);

        // check correctness
        assertEquals(calibrator.getOrientationTimestampNanos(), timestamp);
        accumulatedOrientation = (Quaternion) calibrator.getAccumulatedOrientation();
        assertEquals(orientationA, accumulatedOrientation.getA(), 0.0);
        assertEquals(orientationB, accumulatedOrientation.getB(), 0.0);
        assertEquals(orientationC, accumulatedOrientation.getC(), 0.0);
        assertEquals(orientationD, accumulatedOrientation.getD(), 0.0);

        calibrator.getAccumulatedOrientation(accumulatedOrientation2);
        assertEquals(accumulatedOrientation, accumulatedOrientation2);
    }

    @Test
    void testUpdateOrientationSampleWithAccumulationEnabled() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();
        calibrator.setAccumulationEnabled(true);

        final var randomizer = new UniformRandomizer();

        // check initial values
        assertEquals(-1, calibrator.getOrientationTimestampNanos());
        var accumulatedOrientation = (Quaternion) calibrator.getAccumulatedOrientation();
        assertEquals(1.0, accumulatedOrientation.getA(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getB(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getC(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getD(), 0.0);
        assertEquals(0, calibrator.getAccumulatedOrientationSamples());
        assertFalse(calibrator.isFullSampleAvailable());

        final var accumulatedOrientation2 = new Quaternion();
        calibrator.getAccumulatedOrientation(accumulatedOrientation2);
        assertEquals(accumulatedOrientation, accumulatedOrientation2);

        // update with several samples
        var timestamp = System.currentTimeMillis();
        double orientationA;
        double orientationB;
        double orientationC;
        double orientationD;
        var avgOrientationA = 0.0;
        var avgOrientationB = 0.0;
        var avgOrientationC = 0.0;
        var avgOrientationD = 0.0;
        final var orientation = new Quaternion();
        double norm;
        for (var i = 0; i < TIMES; i++) {
            timestamp += 1000;
            orientationA = randomizer.nextDouble();
            orientationB = randomizer.nextDouble();
            orientationC = randomizer.nextDouble();
            orientationD = randomizer.nextDouble();
            norm = Math.sqrt(orientationA * orientationA + orientationB * orientationB + orientationC * orientationC
                    + orientationD * orientationD);
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
        assertEquals(timestamp, calibrator.getOrientationTimestampNanos());
        accumulatedOrientation = (Quaternion) calibrator.getAccumulatedOrientation();
        assertEquals(avgOrientationA, accumulatedOrientation.getA(), ABSOLUTE_ERROR);
        assertEquals(avgOrientationB, accumulatedOrientation.getB(), ABSOLUTE_ERROR);
        assertEquals(avgOrientationC, accumulatedOrientation.getC(), ABSOLUTE_ERROR);
        assertEquals(avgOrientationD, accumulatedOrientation.getD(), ABSOLUTE_ERROR);

        calibrator.getAccumulatedOrientation(accumulatedOrientation2);
        assertEquals(accumulatedOrientation, accumulatedOrientation2);
    }

    @Test
    void testCalibrationWithOffset() throws SignalProcessingException {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();
        calibrator.setListener(this);

        // setup so that calibrator doesn't reach convergence
        calibrator.setMaxNumSamples(N_SAMPLES);
        calibrator.setConvergenceThreshold(0.0);

        final var offsetRandomizer = new UniformRandomizer();
        final var noiseRandomizer = new GaussianRandomizer(0.0, NOISE_DEVIATION);

        final var accelerationOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final var accelerationOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final var accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);

        final var angularOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final var angularOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final var angularOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);

        var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;

        float accelerationNoiseX;
        float accelerationNoiseY;
        float accelerationNoiseZ;
        float angularNoiseX;
        float angularNoiseY;
        float angularNoiseZ;

        var accelerationX = 0.0;
        var accelerationY = 0.0;
        var accelerationZ = 0.0;
        var angularX = 0.0;
        var angularY = 0.0;
        var angularZ = 0.0;
        double deltaAccelerationX;
        double deltaAccelerationY;
        double deltaAccelerationZ;
        double deltaAngularX;
        double deltaAngularY;
        double deltaAngularZ;
        var lastAccelerationX = 0.0;
        var lastAccelerationY = 0.0;
        var lastAccelerationZ = 0.0;
        var lastAngularX = 0.0;
        var lastAngularY = 0.0;
        var lastAngularZ = 0.0;
        final var orientation = new Quaternion();

        calibrator.reset();
        reset();

        final var estimator = new MeasurementNoiseCovarianceEstimator(13);
        final var sample = new double[13];

        for (var i = 0; i < N_SAMPLES; i++) {
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
                deltaAccelerationX = accelerationX - lastAccelerationX;
                deltaAccelerationY = accelerationY - lastAccelerationY;
                deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                deltaAngularX = angularX - lastAngularX;
                deltaAngularY = angularY - lastAngularY;
                deltaAngularZ = angularZ - lastAngularZ;

                sample[0] = orientation.getA();
                sample[1] = orientation.getB();
                sample[2] = orientation.getC();
                sample[3] = orientation.getD();
                sample[4] = sample[5] = sample[6] = 0.0;
                sample[7] = deltaAccelerationX;
                sample[8] = deltaAccelerationY;
                sample[9] = deltaAccelerationZ;
                sample[10] = deltaAngularX;
                sample[11] = deltaAngularY;
                sample[12] = deltaAngularZ;
                estimator.update(sample);
            }

            lastAccelerationX = accelerationX;
            lastAccelerationY = accelerationY;
            lastAccelerationZ = accelerationZ;

            lastAngularX = angularX;
            lastAngularY = angularY;
            lastAngularZ = angularZ;
        }

        final var mean = calibrator.getControlMean();
        final var mean2 = estimator.getSampleAverage();

        assertArrayEquals(mean, mean2, LARGE_ABSOLUTE_ERROR);
        assertEquals(1.0, mean[0], ABSOLUTE_ERROR);
        assertEquals(0.0, mean[1], ABSOLUTE_ERROR);
        assertEquals(0.0, mean[2], ABSOLUTE_ERROR);
        assertEquals(0.0, mean[3], ABSOLUTE_ERROR);
        assertEquals(0.0, mean[4], ABSOLUTE_ERROR);
        assertEquals(0.0, mean[5], ABSOLUTE_ERROR);
        assertEquals(0.0, mean[6], ABSOLUTE_ERROR);
        assertEquals(accelerationOffsetX, mean[7], LARGE_ABSOLUTE_ERROR);
        assertEquals(accelerationOffsetY, mean[8], LARGE_ABSOLUTE_ERROR);
        assertEquals(accelerationOffsetZ, mean[9], LARGE_ABSOLUTE_ERROR);
        assertEquals(angularOffsetX, mean[10], LARGE_ABSOLUTE_ERROR);
        assertEquals(angularOffsetY, mean[11], LARGE_ABSOLUTE_ERROR);
        assertEquals(angularOffsetZ, mean[12], LARGE_ABSOLUTE_ERROR);

        assertFalse(calibrator.isFailed());
        assertFalse(calibrator.isFinished());
        assertFalse(calibrator.isConverged());
        assertEquals(N_SAMPLES, fullSampleReceived);
        assertEquals(N_SAMPLES, fullSampleProcessed);
        assertEquals(0, calibratorFinished);

        // add one last sample
        calibrator.updateAccelerometerSample(timestamp, (float) accelerationX, (float) accelerationY,
                (float) accelerationZ);
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
    void testCalibrationWithoutOffset() throws SignalProcessingException {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();
        calibrator.setListener(this);

        // setup so that calibrator doesn't reach convergence
        calibrator.setMaxNumSamples(N_SAMPLES);
        calibrator.setConvergenceThreshold(0.0);

        final var offsetRandomizer = new UniformRandomizer();
        final var noiseRandomizer = new GaussianRandomizer(0.0, NOISE_DEVIATION);

        final var accelerationOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final var accelerationOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final var accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);

        final var angularOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final var angularOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final var angularOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);

        var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;

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
        double deltaAccelerationX;
        double deltaAccelerationY;
        double deltaAccelerationZ;
        double deltaAngularX;
        double deltaAngularY;
        double deltaAngularZ;
        var lastAccelerationX = 0.0;
        var lastAccelerationY = 0.0;
        var lastAccelerationZ = 0.0;
        var lastAngularX = 0.0;
        var lastAngularY = 0.0;
        var lastAngularZ = 0.0;
        final var orientation = new Quaternion();

        calibrator.reset();
        reset();

        final var estimator = new MeasurementNoiseCovarianceEstimator(13);
        final var sample = new double[13];

        for (var i = 0; i < N_SAMPLES; i++) {
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

            calibrator.updateAccelerometerSample(timestamp, (float) accelerationX, (float) accelerationY,
                    (float) accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float) angularX, (float) angularY, (float) angularZ);
            calibrator.updateOrientationSample(timestamp, orientation);

            timestamp += DELTA_NANOS;

            if (i != 0) {
                deltaAccelerationX = accelerationX - lastAccelerationX;
                deltaAccelerationY = accelerationY - lastAccelerationY;
                deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                deltaAngularX = angularX - lastAngularX;
                deltaAngularY = angularY - lastAngularY;
                deltaAngularZ = angularZ - lastAngularZ;

                sample[0] = orientation.getA();
                sample[1] = orientation.getB();
                sample[2] = orientation.getC();
                sample[3] = orientation.getD();
                sample[4] = sample[5] = sample[6] = 0.0;
                sample[7] = deltaAccelerationX;
                sample[8] = deltaAccelerationY;
                sample[9] = deltaAccelerationZ;
                sample[10] = deltaAngularX;
                sample[11] = deltaAngularY;
                sample[12] = deltaAngularZ;
                estimator.update(sample);
            }

            lastAccelerationX = accelerationX;
            lastAccelerationY = accelerationY;
            lastAccelerationZ = accelerationZ;

            lastAngularX = angularX;
            lastAngularY = angularY;
            lastAngularZ = angularZ;
        }

        final var mean = calibrator.getControlMean();
        final var mean2 = estimator.getSampleAverage();

        final var cov = calibrator.getControlCovariance();
        final var cov2 = estimator.getMeasurementNoiseCov();

        assertArrayEquals(mean, mean2, LARGE_ABSOLUTE_ERROR);
        assertEquals(1.0, mean[0], ABSOLUTE_ERROR);
        assertEquals(0.0, mean[1], ABSOLUTE_ERROR);
        assertEquals(0.0, mean[2], ABSOLUTE_ERROR);
        assertEquals(0.0, mean[3], ABSOLUTE_ERROR);
        assertEquals(0.0, mean[4], ABSOLUTE_ERROR);
        assertEquals(0.0, mean[5], ABSOLUTE_ERROR);
        assertEquals(0.0, mean[6], ABSOLUTE_ERROR);

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
    void testCalibrationConvergence() {
        final var calibrator = new AbsoluteOrientationSlamCalibrator();
        calibrator.setListener(this);

        final var offsetRandomizer = new UniformRandomizer();
        final var noiseRandomizer = new GaussianRandomizer(0.0, NOISE_DEVIATION);

        final var accelerationOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final var accelerationOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final var accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);

        final var angularOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final var angularOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);
        final var angularOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, MAX_OFFSET);

        var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;

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
        final var orientation = new Quaternion();

        calibrator.reset();
        reset();

        for (var i = 0; i < N_SAMPLES; i++) {
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

            calibrator.updateAccelerometerSample(timestamp, (float) accelerationX, (float) accelerationY,
                    (float) accelerationZ);
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

        assertTrue(fullSampleReceived < N_SAMPLES);
        assertTrue(fullSampleProcessed < N_SAMPLES);
        assertEquals(fullSampleReceived, fullSampleProcessed);
        assertEquals(1, calibratorFinished);
    }

    @Override
    public void onFullSampleReceived(final BaseSlamCalibrator<AbsoluteOrientationSlamCalibrationData> calibrator) {
        fullSampleReceived++;
    }

    @Override
    public void onFullSampleProcessed(final BaseSlamCalibrator<AbsoluteOrientationSlamCalibrationData> calibrator) {
        fullSampleProcessed++;
    }

    @Override
    public void onCalibratorFinished(
            final BaseSlamCalibrator<AbsoluteOrientationSlamCalibrationData> calibrator, final boolean converged,
            final boolean failed) {
        calibratorFinished++;
    }

    private void reset() {
        fullSampleReceived = fullSampleProcessed = calibratorFinished = 0;
    }
}
