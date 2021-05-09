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
import com.irurueta.numerical.signal.processing.MeasurementNoiseCovarianceEstimator;
import com.irurueta.numerical.signal.processing.SignalProcessingException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.InvalidCovarianceMatrixException;
import com.irurueta.statistics.MultivariateNormalDist;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class SlamCalibratorTest implements BaseSlamCalibratorListener<SlamCalibrationData> {

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
    private static final int DELTA_NANOS = 20000000; //0.02 seconds

    private int fullSampleReceived;
    private int fullSampleProcessed;
    private int calibratorFinished;

    @Test
    public void testConstructor() throws WrongSizeException {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // check initial values
        assertEquals(calibrator.getSampleLength(),
                SlamEstimator.CONTROL_LENGTH);
        assertEquals(calibrator.getEstimatorStateLength(),
                SlamEstimator.STATE_LENGTH);
        assertFalse(calibrator.isConverged());
        assertFalse(calibrator.isFailed());
        assertFalse(calibrator.isFinished());
        assertEquals(calibrator.getSampleCount(), 0);
        assertEquals(calibrator.getMinNumSamples(),
                SlamCalibrator.DEFAULT_MIN_NUM_SAMPLES);
        assertEquals(calibrator.getMaxNumSamples(),
                SlamCalibrator.DEFAULT_MAX_NUM_SAMPLES);
        assertEquals(calibrator.getConvergenceThreshold(),
                SlamCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, 0.0);
        assertEquals(calibrator.isAccumulationEnabled(),
                SlamCalibrator.DEFAULT_ENABLE_SAMPLE_ACCUMULATION);
        assertEquals(calibrator.getAccelerometerTimestampNanos(), -1);
        assertEquals(calibrator.getGyroscopeTimestampNanos(), -1);
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 0);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(calibrator.isAccelerometerSampleReceived());
        assertFalse(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);
        assertArrayEquals(calibrator.getAccumulatedAccelerationSample(),
                new double[]{0.0, 0.0, 0.0}, 0.0);
        final double[] accumAcceleration = new double[3];
        calibrator.getAccumulatedAccelerationSample(accumAcceleration);
        assertArrayEquals(accumAcceleration, new double[]{0.0, 0.0, 0.0}, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);
        assertArrayEquals(calibrator.getAccumulatedAngularSpeedSample(),
                new double[]{0.0, 0.0, 0.0}, 0.0);
        final double[] accumAngularSpeed = new double[3];
        calibrator.getAccumulatedAngularSpeedSample(accumAngularSpeed);
        assertArrayEquals(accumAngularSpeed, new double[]{0.0, 0.0, 0.0}, 0.0);
        assertNull(calibrator.getListener());
        assertArrayEquals(calibrator.getControlMean(),
                new double[SlamEstimator.CONTROL_LENGTH], 0.0);
        final double[] controlMean = new double[SlamEstimator.CONTROL_LENGTH];
        calibrator.getControlMean(controlMean);
        assertArrayEquals(controlMean, new double[SlamEstimator.CONTROL_LENGTH], 0.0);
        assertEquals(calibrator.getControlCovariance(),
                new Matrix(SlamEstimator.CONTROL_LENGTH,
                        SlamEstimator.CONTROL_LENGTH));
        final Matrix cov = new Matrix(1, 1);
        calibrator.getControlCovariance(cov);
        assertEquals(cov, new Matrix(SlamEstimator.CONTROL_LENGTH,
                SlamEstimator.CONTROL_LENGTH));
    }

    @Test
    public void testIsConverged() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertFalse(calibrator.isConverged());

        // set new value
        calibrator.mConverged = true;

        // check correctness
        assertTrue(calibrator.isConverged());
    }

    @Test
    public void testIsFailed() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertFalse(calibrator.isFailed());

        // set new value
        calibrator.mFailed = true;

        // check correctness
        assertTrue(calibrator.isFailed());
    }

    @Test
    public void testIsFinished() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertFalse(calibrator.isFinished());

        // set new value
        calibrator.mFinished = true;

        // check correctness
        assertTrue(calibrator.isFinished());
    }

    @Test
    public void testGetSampleCount() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertEquals(calibrator.getSampleCount(), 0);

        // set new value
        calibrator.mSampleCount = 5;

        // check correctness
        assertEquals(calibrator.getSampleCount(), 5);
    }

    @Test
    public void testGetSetMinNumSamples() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertEquals(calibrator.getMinNumSamples(),
                SlamCalibrator.DEFAULT_MIN_NUM_SAMPLES);

        // set new value
        calibrator.setMinNumSamples(50);

        // check correctness
        assertEquals(calibrator.getMinNumSamples(), 50);

        // Force IllegalArgumentException
        try {
            calibrator.setMinNumSamples(-1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxNumSamples() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertEquals(calibrator.getMaxNumSamples(),
                SlamCalibrator.DEFAULT_MAX_NUM_SAMPLES);

        // set new value
        calibrator.setMaxNumSamples(1000);

        // check correctness
        assertEquals(calibrator.getMaxNumSamples(), 1000);

        // Force IllegalArgumentException
        try {
            calibrator.setMaxNumSamples(-1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetConvergenceThreshold() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertEquals(calibrator.getConvergenceThreshold(),
                SlamCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, 0.0);

        // set new value
        calibrator.setConvergenceThreshold(1.0);

        // check correctness
        assertEquals(calibrator.getConvergenceThreshold(), 1.0, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.setConvergenceThreshold(-0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testReset() throws WrongSizeException {
        final SlamCalibrator calibrator = new SlamCalibrator();

        calibrator.reset();

        // check correctness
        assertFalse(calibrator.isConverged());
        assertFalse(calibrator.isFailed());
        assertFalse(calibrator.isFinished());
        assertEquals(calibrator.getSampleCount(), 0);
        assertEquals(calibrator.getMinNumSamples(),
                SlamCalibrator.DEFAULT_MIN_NUM_SAMPLES);
        assertEquals(calibrator.getMaxNumSamples(),
                SlamCalibrator.DEFAULT_MAX_NUM_SAMPLES);
        assertEquals(calibrator.getConvergenceThreshold(),
                SlamCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, 0.0);
        assertEquals(calibrator.isAccumulationEnabled(),
                SlamCalibrator.DEFAULT_ENABLE_SAMPLE_ACCUMULATION);
        assertEquals(calibrator.getAccelerometerTimestampNanos(), -1);
        assertEquals(calibrator.getGyroscopeTimestampNanos(), -1);
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 0);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(calibrator.isAccelerometerSampleReceived());
        assertFalse(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);
        assertArrayEquals(calibrator.getAccumulatedAccelerationSample(),
                new double[]{0.0, 0.0, 0.0}, 0.0);
        final double[] accumAcceleration = new double[3];
        calibrator.getAccumulatedAccelerationSample(accumAcceleration);
        assertArrayEquals(accumAcceleration, new double[]{0.0, 0.0, 0.0}, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);
        assertArrayEquals(calibrator.getAccumulatedAngularSpeedSample(),
                new double[]{0.0, 0.0, 0.0}, 0.0);
        final double[] accumAngularSpeed = new double[3];
        calibrator.getAccumulatedAngularSpeedSample(accumAngularSpeed);
        assertArrayEquals(accumAngularSpeed, new double[]{0.0, 0.0, 0.0}, 0.0);
        assertNull(calibrator.getListener());
        assertArrayEquals(calibrator.getControlMean(),
                new double[SlamEstimator.CONTROL_LENGTH], 0.0);
        final double[] controlMean = new double[SlamEstimator.CONTROL_LENGTH];
        calibrator.getControlMean(controlMean);
        assertArrayEquals(controlMean, new double[SlamEstimator.CONTROL_LENGTH], 0.0);
        assertEquals(calibrator.getControlCovariance(),
                new Matrix(SlamEstimator.CONTROL_LENGTH,
                        SlamEstimator.CONTROL_LENGTH));
        final Matrix cov = new Matrix(1, 1);
        calibrator.getControlCovariance(cov);
        assertEquals(cov, new Matrix(SlamEstimator.CONTROL_LENGTH,
                SlamEstimator.CONTROL_LENGTH));
    }

    @Test
    public void testIsSetAccumulationEnabled() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertTrue(calibrator.isAccumulationEnabled());

        // set new value
        calibrator.setAccumulationEnabled(false);

        // check correctness
        assertFalse(calibrator.isAccumulationEnabled());
    }

    @Test
    public void testGetAccelerometerTimestampNanos() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertEquals(calibrator.getAccelerometerTimestampNanos(), -1);

        // set new value
        calibrator.mAccelerometerTimestampNanos = 1000;

        // check correctness
        assertEquals(calibrator.getAccelerometerTimestampNanos(), 1000);
    }

    @Test
    public void testGetGyroscopeTimestampNanos() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertEquals(calibrator.getGyroscopeTimestampNanos(), -1);

        // set new value
        calibrator.mGyroscopeTimestampNanos = 2000;

        // check correctness
        assertEquals(calibrator.getGyroscopeTimestampNanos(), 2000);
    }

    @Test
    public void testGetAccumulatedAccelerometerSamplesAndIsAccelerometerSampleReceived() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 0);
        assertFalse(calibrator.isAccelerometerSampleReceived());

        // set new value
        calibrator.mAccumulatedAccelerometerSamples = 50;

        // check correctness
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 50);
        assertTrue(calibrator.isAccelerometerSampleReceived());
    }

    @Test
    public void testGetAccumulatedGyroscopeSamples() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(calibrator.isGyroscopeSampleReceived());

        // set new value
        calibrator.mAccumulatedGyroscopeSamples = 500;

        // check correctness
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 500);
        assertTrue(calibrator.isGyroscopeSampleReceived());
    }

    @Test
    public void testIsFullSampleAvailable() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial values
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 0);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(calibrator.isAccelerometerSampleReceived());
        assertFalse(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());

        // set accelerometer sample
        calibrator.mAccumulatedAccelerometerSamples = 1;

        // check correctness
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 1);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 0);
        assertTrue(calibrator.isAccelerometerSampleReceived());
        assertFalse(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());

        // set gyroscope sample
        calibrator.mAccumulatedGyroscopeSamples = 1;

        // check correctness
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 1);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 1);
        assertTrue(calibrator.isAccelerometerSampleReceived());
        assertTrue(calibrator.isGyroscopeSampleReceived());
        assertTrue(calibrator.isFullSampleAvailable());
    }

    @Test
    public void testGetAccumulatedAccelerationSampleX() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);

        // set new value
        calibrator.mAccumulatedAccelerationSampleX = 1.0;

        // check correctness
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(), 1.0, 0.0);
    }

    @Test
    public void testGetAccumulatedAccelerationSampleY() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);

        // set new value
        calibrator.mAccumulatedAccelerationSampleY = 2.0;

        // check correctness
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(), 2.0, 0.0);
    }

    @Test
    public void testGetAccumulatedAccelerationSampleZ() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);

        // set new value
        calibrator.mAccumulatedAccelerationSampleZ = 3.0;

        // check correctness
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(), 3.0, 0.0);
    }

    @Test
    public void testGetAccumulatedAccelerationSample() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertArrayEquals(calibrator.getAccumulatedAccelerationSample(),
                new double[]{0.0, 0.0, 0.0}, 0.0);

        // set new value
        calibrator.mAccumulatedAccelerationSampleX = 1.0;
        calibrator.mAccumulatedAccelerationSampleY = 2.0;
        calibrator.mAccumulatedAccelerationSampleZ = 3.0;

        // check correctness
        assertArrayEquals(calibrator.getAccumulatedAccelerationSample(),
                new double[]{1.0, 2.0, 3.0}, 0.0);

        final double[] sample = new double[3];
        calibrator.getAccumulatedAccelerationSample(sample);

        // check correctness
        assertArrayEquals(sample, new double[]{1.0, 2.0, 3.0}, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.getAccumulatedAccelerationSample(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetAccumulatedAngularSpeedSampleX() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);

        // set new value
        calibrator.mAccumulatedAngularSpeedSampleX = 1.0;

        // check correctness
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(), 1.0, 0.0);
    }

    @Test
    public void testGetAccumulatedAngularSpeedSampleY() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);

        // set new value
        calibrator.mAccumulatedAngularSpeedSampleY = 2.0;

        // check correctness
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(), 2.0, 0.0);
    }

    @Test
    public void testGetAccumulatedAngularSpeedSampleZ() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);

        // set new value
        calibrator.mAccumulatedAngularSpeedSampleZ = 3.0;

        // check correctness
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(), 3.0, 0.0);
    }

    @Test
    public void testGetAccumulatedAngularSpeedSample() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertArrayEquals(calibrator.getAccumulatedAngularSpeedSample(),
                new double[]{0.0, 0.0, 0.0}, 0.0);

        // set new value
        calibrator.mAccumulatedAngularSpeedSampleX = 1.0;
        calibrator.mAccumulatedAngularSpeedSampleY = 2.0;
        calibrator.mAccumulatedAngularSpeedSampleZ = 3.0;

        // check correctness
        assertArrayEquals(calibrator.getAccumulatedAngularSpeedSample(),
                new double[]{1.0, 2.0, 3.0}, 0.0);

        final double[] sample = new double[3];
        calibrator.getAccumulatedAngularSpeedSample(sample);

        // check correctness
        assertArrayEquals(sample, new double[]{1.0, 2.0, 3.0}, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.getAccumulatedAngularSpeedSample(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetMostRecentTimestampNanos() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // check correctness
        assertEquals(calibrator.getMostRecentTimestampNanos(), -1);

        // set new value
        calibrator.mAccelerometerTimestampNanos = 1000;

        // check correctness
        assertEquals(calibrator.getMostRecentTimestampNanos(), 1000);

        // set new value
        calibrator.mGyroscopeTimestampNanos = 2000;

        // check correctness
        assertEquals(calibrator.getMostRecentTimestampNanos(), 2000);
    }

    @Test
    public void testGetSetListener() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        // initial value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check correctness
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testGetControlMean() {
        final SlamCalibrator calibrator = new SlamCalibrator();

        assertArrayEquals(calibrator.getControlMean(),
                new double[SlamEstimator.CONTROL_LENGTH], 0.0);
        assertArrayEquals(calibrator.getControlMean(),
                calibrator.mEstimator.getSampleAverage(), 0.0);

        final double[] controlMean = new double[SlamEstimator.CONTROL_LENGTH];
        calibrator.getControlMean(controlMean);
        assertArrayEquals(calibrator.getControlMean(), controlMean, 0.0);

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
        final SlamCalibrator calibrator = new SlamCalibrator();

        assertEquals(calibrator.getControlCovariance(),
                new Matrix(SlamEstimator.CONTROL_LENGTH,
                        SlamEstimator.CONTROL_LENGTH));

        final Matrix cov = new Matrix(1, 1);
        calibrator.getControlCovariance(cov);

        assertEquals(calibrator.getControlCovariance(), cov);
    }

    @Test
    public void testGetControlDistribution()
            throws InvalidCovarianceMatrixException {
        final SlamCalibrator calibrator = new SlamCalibrator();

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
        final SlamCalibrator calibrator = new SlamCalibrator();

        final double[] controlMean = calibrator.getControlMean();
        final Matrix cov = calibrator.getControlCovariance();

        final SlamCalibrationData data1 = calibrator.getCalibrationData();
        final SlamCalibrationData data2 = new SlamCalibrationData();
        calibrator.getCalibrationData(data2);

        // check correctness
        assertSame(data1.getControlMean(), controlMean);
        assertSame(data1.getControlCovariance(), cov);

        assertSame(data2.getControlMean(), controlMean);
        assertSame(data2.getControlCovariance(), cov);
    }

    @Test
    public void testPropagateWithControlJacobian() throws WrongSizeException,
            InvalidCovarianceMatrixException {
        final SlamCalibrator calibrator = new SlamCalibrator();

        final UniformRandomizer offsetRandomizer = new UniformRandomizer(
                new Random());
        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, NOISE_DEVIATION);

        final float accelerationOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        final float accelerationOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        final float accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);

        final float angularOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        final float angularOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        final float angularOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);

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
                    (float) accelerationX, (float) accelerationY,
                    (float) accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float) angularX,
                    (float) angularY, (float) angularZ);

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
        final MultivariateNormalDist dist = calibrator.propagateWithControlJacobian(
                jacobian);
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
        final SlamCalibrator calibrator = new SlamCalibrator();
        calibrator.setAccumulationEnabled(false);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        long timestamp = System.currentTimeMillis();
        final float accelerationX = randomizer.nextFloat();
        final float accelerationY = randomizer.nextFloat();
        final float accelerationZ = randomizer.nextFloat();

        // check initial values
        assertEquals(calibrator.getAccelerometerTimestampNanos(), -1);
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 0);
        assertFalse(calibrator.isFullSampleAvailable());

        calibrator.updateAccelerometerSample(timestamp, accelerationX,
                accelerationY, accelerationZ);

        // check correctness
        assertEquals(calibrator.getAccelerometerTimestampNanos(), timestamp);
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(),
                accelerationX, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(),
                accelerationY, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(),
                accelerationZ, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 1);
        assertFalse(calibrator.isFullSampleAvailable());

        // test again but using an array
        final float[] acceleration = new float[3];
        randomizer.fill(acceleration);
        timestamp += 1000;

        calibrator.updateAccelerometerSample(timestamp, acceleration);

        // check correctness
        assertEquals(calibrator.getAccelerometerTimestampNanos(), timestamp);
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(),
                acceleration[0], 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(),
                acceleration[1], 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(),
                acceleration[2], 0.0);
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 2);
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
        final SlamCalibrator calibrator = new SlamCalibrator();
        calibrator.setAccumulationEnabled(true);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(calibrator.getAccelerometerTimestampNanos(), -1);
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 0);
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

            calibrator.updateAccelerometerSample(timestamp, accelerationX,
                    accelerationY, accelerationZ);
        }

        // check correctness
        assertEquals(calibrator.getAccelerometerTimestampNanos(), timestamp);
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(),
                avgAccelerationX, ABSOLUTE_ERROR);
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(),
                avgAccelerationY, ABSOLUTE_ERROR);
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(),
                avgAccelerationZ, ABSOLUTE_ERROR);
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), TIMES);
        assertFalse(calibrator.isFullSampleAvailable());
    }

    @Test
    public void testUpdateGyroscopeSampleWithAccumulationDisabled() {
        final SlamCalibrator calibrator = new SlamCalibrator();
        calibrator.setAccumulationEnabled(false);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        long timestamp = System.currentTimeMillis();
        final float angularSpeedX = randomizer.nextFloat();
        final float angularSpeedY = randomizer.nextFloat();
        final float angularSpeedZ = randomizer.nextFloat();

        // check initial values
        assertEquals(calibrator.getGyroscopeTimestampNanos(), -1);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(calibrator.isFullSampleAvailable());

        calibrator.updateGyroscopeSample(timestamp, angularSpeedX,
                angularSpeedY, angularSpeedZ);

        // check correctness
        assertEquals(calibrator.getGyroscopeTimestampNanos(), timestamp);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(),
                angularSpeedX, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(),
                angularSpeedY, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(),
                angularSpeedZ, 0.0);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 1);
        assertFalse(calibrator.isFullSampleAvailable());

        // test again but using an array
        final float[] angularSpeed = new float[3];
        randomizer.fill(angularSpeed);
        timestamp += 100;

        calibrator.updateGyroscopeSample(timestamp, angularSpeed);

        // check correctness
        assertEquals(calibrator.getGyroscopeTimestampNanos(), timestamp);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(),
                angularSpeed[0], 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(),
                angularSpeed[1], 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(),
                angularSpeed[2], 0.0);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 2);
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
        final SlamCalibrator calibrator = new SlamCalibrator();
        calibrator.setAccumulationEnabled(true);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(calibrator.getGyroscopeTimestampNanos(), -1);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 0);
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

            calibrator.updateGyroscopeSample(timestamp, angularSpeedX,
                    angularSpeedY, angularSpeedZ);
        }

        // check correctness
        assertEquals(calibrator.getGyroscopeTimestampNanos(), timestamp);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(),
                avgAngularSpeedX, ABSOLUTE_ERROR);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(),
                avgAngularSpeedY, ABSOLUTE_ERROR);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(),
                avgAngularSpeedZ, ABSOLUTE_ERROR);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), TIMES);
        assertFalse(calibrator.isFullSampleAvailable());
    }

    @Test
    public void testCalibrationWithOffset() throws SignalProcessingException {
        final SlamCalibrator calibrator = new SlamCalibrator();
        calibrator.setListener(this);

        // setup so that calibrator doesn't reach convergence
        calibrator.setMaxNumSamples(N_SAMPLES);
        calibrator.setConvergenceThreshold(0.0);

        final UniformRandomizer offsetRandomizer = new UniformRandomizer(
                new Random());
        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, NOISE_DEVIATION);

        final float accelerationOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        final float accelerationOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        final float accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);

        final float angularOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        final float angularOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        final float angularOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);

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
        double deltaAccelerationX;
        double deltaAccelerationY;
        double deltaAccelerationZ;
        double deltaAngularX;
        double deltaAngularY;
        double deltaAngularZ;
        double lastAccelerationX = 0.0;
        double lastAccelerationY = 0.0;
        double lastAccelerationZ = 0.0;
        double lastAngularX = 0.0;
        double lastAngularY = 0.0;
        double lastAngularZ = 0.0;

        calibrator.reset();
        reset();

        final MeasurementNoiseCovarianceEstimator estimator =
                new MeasurementNoiseCovarianceEstimator(9);
        final double[] sample = new double[9];

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

            calibrator.updateAccelerometerSample(timestamp, (float) accelerationX,
                    (float) accelerationY, (float) accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float) angularX, (float) angularY,
                    (float) angularZ);

            timestamp += DELTA_NANOS;

            if (i != 0) {
                deltaAccelerationX = accelerationX - lastAccelerationX;
                deltaAccelerationY = accelerationY - lastAccelerationY;
                deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                deltaAngularX = angularX - lastAngularX;
                deltaAngularY = angularY - lastAngularY;
                deltaAngularZ = angularZ - lastAngularZ;

                sample[0] = sample[1] = sample[2] = 0.0;
                sample[3] = deltaAccelerationX;
                sample[4] = deltaAccelerationY;
                sample[5] = deltaAccelerationZ;
                sample[6] = deltaAngularX;
                sample[7] = deltaAngularY;
                sample[8] = deltaAngularZ;
                estimator.update(sample);
            }

            lastAccelerationX = accelerationX;
            lastAccelerationY = accelerationY;
            lastAccelerationZ = accelerationZ;

            lastAngularX = angularX;
            lastAngularY = angularY;
            lastAngularZ = angularZ;
        }

        final double[] mean = calibrator.getControlMean();
        final double[] mean2 = estimator.getSampleAverage();

        assertArrayEquals(mean, mean2, LARGE_ABSOLUTE_ERROR);
        assertEquals(mean[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(mean[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(mean[2], 0.0, ABSOLUTE_ERROR);
        assertEquals(mean[3], accelerationOffsetX, LARGE_ABSOLUTE_ERROR);
        assertEquals(mean[4], accelerationOffsetY, LARGE_ABSOLUTE_ERROR);
        assertEquals(mean[5], accelerationOffsetZ, LARGE_ABSOLUTE_ERROR);
        assertEquals(mean[6], angularOffsetX, LARGE_ABSOLUTE_ERROR);
        assertEquals(mean[7], angularOffsetY, LARGE_ABSOLUTE_ERROR);
        assertEquals(mean[8], angularOffsetZ, LARGE_ABSOLUTE_ERROR);

        assertFalse(calibrator.isFailed());
        assertFalse(calibrator.isFinished());
        assertFalse(calibrator.isConverged());
        assertEquals(fullSampleReceived, N_SAMPLES);
        assertEquals(fullSampleProcessed, N_SAMPLES);
        assertEquals(calibratorFinished, 0);

        // add one last sample
        calibrator.updateAccelerometerSample(timestamp, (float) accelerationX,
                (float) accelerationY, (float) accelerationZ);
        calibrator.updateGyroscopeSample(timestamp, (float) angularX, (float) angularY,
                (float) angularZ);

        // check
        assertFalse(calibrator.isFailed());
        assertTrue(calibrator.isFinished());
        assertFalse(calibrator.isConverged());
        assertEquals(fullSampleReceived, N_SAMPLES + 1);
        assertEquals(fullSampleProcessed, N_SAMPLES + 1);
        assertEquals(calibratorFinished, 1);
    }

    @Test
    public void testCalibrationWithoutOffset() throws SignalProcessingException, InvalidCovarianceMatrixException {
        final SlamCalibrator calibrator = new SlamCalibrator();
        calibrator.setListener(this);

        // setup so that calibrator doesn't reach convergence
        calibrator.setMaxNumSamples(N_SAMPLES);
        calibrator.setConvergenceThreshold(0.0);

        final UniformRandomizer offsetRandomizer = new UniformRandomizer(
                new Random());
        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, NOISE_DEVIATION);

        final float accelerationOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        final float accelerationOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        final float accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);

        final float angularOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        final float angularOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        final float angularOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);

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
        double deltaAccelerationX;
        double deltaAccelerationY;
        double deltaAccelerationZ;
        double deltaAngularX;
        double deltaAngularY;
        double deltaAngularZ;
        double lastAccelerationX = 0.0;
        double lastAccelerationY = 0.0;
        double lastAccelerationZ = 0.0;
        double lastAngularX = 0.0;
        double lastAngularY = 0.0;
        double lastAngularZ = 0.0;

        calibrator.reset();
        reset();

        final MeasurementNoiseCovarianceEstimator estimator =
                new MeasurementNoiseCovarianceEstimator(9);
        final double[] sample = new double[9];

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
            calibrator.updateGyroscopeSample(timestamp, (float) angularX, (float) angularY,
                    (float) angularZ);

            timestamp += DELTA_NANOS;

            if (i != 0) {
                deltaAccelerationX = accelerationX - lastAccelerationX;
                deltaAccelerationY = accelerationY - lastAccelerationY;
                deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                deltaAngularX = angularX - lastAngularX;
                deltaAngularY = angularY - lastAngularY;
                deltaAngularZ = angularZ - lastAngularZ;

                sample[0] = sample[1] = sample[2] = 0.0;
                sample[3] = deltaAccelerationX;
                sample[4] = deltaAccelerationY;
                sample[5] = deltaAccelerationZ;
                sample[6] = deltaAngularX;
                sample[7] = deltaAngularY;
                sample[8] = deltaAngularZ;
                estimator.update(sample);
            }

            lastAccelerationX = accelerationX;
            lastAccelerationY = accelerationY;
            lastAccelerationZ = accelerationZ;

            lastAngularX = angularX;
            lastAngularY = angularY;
            lastAngularZ = angularZ;
        }

        final double[] mean = calibrator.getControlMean();
        final double[] mean2 = estimator.getSampleAverage();

        final Matrix cov = calibrator.getControlCovariance();
        final Matrix cov2 = estimator.getMeasurementNoiseCov();

        assertArrayEquals(mean, mean2, LARGE_ABSOLUTE_ERROR);
        assertEquals(mean[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(mean[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(mean[2], 0.0, ABSOLUTE_ERROR);

        assertTrue(cov.equals(cov2, ABSOLUTE_ERROR));

        assertFalse(calibrator.isFailed());
        assertFalse(calibrator.isFinished());
        assertFalse(calibrator.isConverged());
        assertEquals(fullSampleReceived, N_SAMPLES);
        assertEquals(fullSampleProcessed, N_SAMPLES);
        assertEquals(calibratorFinished, 0);

        // add one last sample
        calibrator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
        calibrator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

        // check
        assertFalse(calibrator.isFailed());
        assertTrue(calibrator.isFinished());
        assertFalse(calibrator.isConverged());
        assertEquals(fullSampleReceived, N_SAMPLES + 1);
        assertEquals(fullSampleProcessed, N_SAMPLES + 1);
        assertEquals(calibratorFinished, 1);

        final MultivariateNormalDist dist = calibrator.getControlDistribution();
        assertArrayEquals(dist.getMean(), calibrator.getControlMean(), 0.0);
        assertEquals(dist.getCovariance(), cov);
    }

    @Test
    public void testCalibrationConvergence() {
        final SlamCalibrator calibrator = new SlamCalibrator();
        calibrator.setListener(this);

        final UniformRandomizer offsetRandomizer = new UniformRandomizer(
                new Random());
        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, NOISE_DEVIATION);

        final float accelerationOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        final float accelerationOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        final float accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);

        final float angularOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        final float angularOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        final float angularOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);

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
            calibrator.updateGyroscopeSample(timestamp, (float) angularX, (float) angularY,
                    (float) angularZ);

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
        assertEquals(calibratorFinished, 1);

        calibrator.updateSample();
    }

    @Override
    public void onFullSampleReceived(
            final BaseSlamCalibrator<SlamCalibrationData> calibrator) {
        fullSampleReceived++;
    }

    @Override
    public void onFullSampleProcessed(
            final BaseSlamCalibrator<SlamCalibrationData> calibrator) {
        fullSampleProcessed++;
    }

    @Override
    public void onCalibratorFinished(
            final BaseSlamCalibrator<SlamCalibrationData> calibrator,
            final boolean converged, final boolean failed) {
        calibratorFinished++;
    }

    private void reset() {
        fullSampleReceived = fullSampleProcessed = calibratorFinished = 0;
    }
}
