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

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.numerical.signal.processing.MeasurementNoiseCovarianceEstimator;
import com.irurueta.numerical.signal.processing.SignalProcessingException;
import com.irurueta.statistics.InvalidCovarianceMatrixException;
import com.irurueta.statistics.MultivariateNormalDist;

import java.util.Arrays;

/**
 * Base class for estimating mean and covariance of noise in control values
 * when the system state is held constant (only noise is provided as control
 * input).
 *
 * @param <D> type of calibration data.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class BaseSlamCalibrator<D extends BaseCalibrationData> {

    /**
     * Minimum allowed sample length.
     */
    public static final int MIN_SAMPLE_LENGTH = 1;

    /**
     * Default minimum number of samples to take into account.
     */
    public static final int DEFAULT_MIN_NUM_SAMPLES = 20;

    /**
     * Default maximum number of samples to take into account.
     */
    public static final int DEFAULT_MAX_NUM_SAMPLES = 100;

    /**
     * Value to consider that mean and covariance have converged.
     */
    public static final double DEFAULT_CONVERGENCE_THRESHOLD = 1e-5;

    /**
     * Indicates whether sample accumulation must be enabled or not.
     */
    protected static final boolean DEFAULT_ENABLE_SAMPLE_ACCUMULATION = true;

    /**
     * Number of components in 3D.
     */
    protected static final int N_COMPONENTS_3D = 3;

    /**
     * Conversion of nanoseconds to milliseconds.
     */
    protected static final double NANOS_TO_SECONDS = 1e-9;

    /**
     * Sample length of control values used during prediction stage in SLAM
     * estimator.
     */
    private final int sampleLength;

    /**
     * Array containing a control sample used during SLAM prediction stage.
     */
    protected double[] sample;

    /**
     * Mean and covariance estimator.
     */
    protected MeasurementNoiseCovarianceEstimator estimator;

    /**
     * Contains previous mean value.
     */
    protected double[] previousMean;

    /**
     * Contains mean value of covariance.
     */
    protected Matrix previousCovariance;

    /**
     * Indicates whether this calibrator converged.
     */
    protected boolean converged;

    /**
     * Indicates whether this calibrator failed.
     */
    protected boolean failed;

    /**
     * Indicates whether calibrator has finished taking samples.
     */
    protected boolean finished;

    /**
     * Number of obtained samples.
     */
    protected int sampleCount;

    /**
     * Array to store the difference between average values to determine whether
     * the result has converged or not.
     */
    protected double[] meanDiff;

    /**
     * Matrix to store the difference between covariance matrices to determine
     * whether the result has converged or not.
     */
    protected Matrix covDiff;

    /**
     * Minimum number of samples to take into account.
     */
    protected int minNumSamples = DEFAULT_MIN_NUM_SAMPLES;

    /**
     * Maximum number of samples to take into account.
     */
    protected int maxNumSamples = DEFAULT_MAX_NUM_SAMPLES;

    /**
     * Threshold to consider whether calibration has converged or not.
     */
    protected double convergenceThreshold = DEFAULT_CONVERGENCE_THRESHOLD;

    /**
     * Indicates whether accumulation of samples is enabled or not.
     */
    protected boolean accumulationEnabled = DEFAULT_ENABLE_SAMPLE_ACCUMULATION;

    /**
     * Timestamp expressed in nanoseconds since the epoch time of the last
     * accelerometer sample.
     */
    protected long accelerometerTimestampNanos = -1;

    /**
     * Timestamp expressed in nanoseconds since the epoch time of the last
     * gyroscope sample.
     */
    protected long gyroscopeTimestampNanos = -1;

    /**
     * Number of accelerometer samples accumulated since last full sample.
     */
    protected int accumulatedAccelerometerSamples = 0;

    /**
     * Number of gyroscope samples accumulated since last full sample.
     */
    protected int accumulatedGyroscopeSamples = 0;

    /**
     * Average of acceleration along x-axis accumulated since last full sample.
     * Expressed in meters per squared second (m/s^2).
     */
    protected double accumulatedAccelerationSampleX;

    /**
     * Average of acceleration along y-axis accumulated since last full sample.
     * Expressed in meters per squared second (m/s^2).
     */
    protected double accumulatedAccelerationSampleY;

    /**
     * Average of acceleration along z-axis accumulated since last full sample.
     * Expressed in meters per squared second (m/s^2).
     */
    protected double accumulatedAccelerationSampleZ;

    /**
     * Average of angular speed along x-axis accumulated since last full sample.
     * Expressed in meters per squared second (m/s^2).
     */
    protected double accumulatedAngularSpeedSampleX;

    /**
     * Average of angular speed along y-axis accumulated since last full sample.
     * Expressed in meters per squared second (m/s^2).
     */
    protected double accumulatedAngularSpeedSampleY;

    /**
     * Average of angular speed along z-axis accumulated since last full sample.
     * Expressed in meters per squared second (m/s^2).
     */
    protected double accumulatedAngularSpeedSampleZ;

    /**
     * Listener in charge of handling events raised by instances of this class.
     */
    protected BaseSlamCalibratorListener<D> listener;

    /**
     * Constructor.
     *
     * @param sampleLength sample length of control values used during
     *                     prediction stage in SLAM estimator.
     * @throws IllegalArgumentException if sample length is less than 1.
     */
    protected BaseSlamCalibrator(final int sampleLength) {
        if (sampleLength < MIN_SAMPLE_LENGTH) {
            throw new IllegalArgumentException("length must be greater than 0");
        }

        this.sampleLength = sampleLength;
        sample = new double[sampleLength];
        previousMean = new double[sampleLength];
        meanDiff = new double[sampleLength];
        try {
            previousCovariance = new Matrix(sampleLength, sampleLength);
            covDiff = new Matrix(sampleLength, sampleLength);
            estimator = new MeasurementNoiseCovarianceEstimator(sampleLength);
        } catch (final Exception ignore) {
            // never thrown
        }
    }

    /**
     * Gets sample length of control values used during prediction stage in SLAM
     * estimator.
     *
     * @return sample length of control values used during prediction stage in
     * SLAM estimator.
     */
    public int getSampleLength() {
        return sampleLength;
    }

    /**
     * Indicates whether calibrator converged or not.
     *
     * @return true if calibrator converged, false otherwise.
     */
    public boolean isConverged() {
        return converged;
    }

    /**
     * Indicates whether this calibrator failed or not.
     *
     * @return true if calibrator failed, false otherwise.
     */
    public boolean isFailed() {
        return failed;
    }

    /**
     * Indicates whether calibrator has finished taking samples or not.
     *
     * @return true if calibrator has finished taking samples, false otherwise.
     */
    public boolean isFinished() {
        return finished;
    }

    /**
     * Gets number of obtained samples.
     *
     * @return number of obtained samples.
     */
    public int getSampleCount() {
        return sampleCount;
    }

    /**
     * Obtains the minimum number of samples to use before taking convergence
     * into account.
     *
     * @return minimum number of samples to use before taking convergence into
     * account.
     */
    public int getMinNumSamples() {
        return minNumSamples;
    }

    /**
     * Specifies the minimum number of samples to take before taking convergence
     * into account.
     *
     * @param minNumSamples minimum number of samples to take before taking
     *                      convergence into account.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setMinNumSamples(final int minNumSamples) {
        if (minNumSamples < 0) {
            throw new IllegalArgumentException("minNumSamples must be positive");
        }
        this.minNumSamples = minNumSamples;
    }

    /**
     * Gets maximum number of samples to take into account.
     *
     * @return maximum number of samples to take into account.
     */
    public int getMaxNumSamples() {
        return maxNumSamples;
    }

    /**
     * Specifies the maximum number of samples to take.
     *
     * @param maxNumSamples maximum number of samples to take.
     * @throws IllegalArgumentException if provided value is negative or zero.
     */
    public void setMaxNumSamples(final int maxNumSamples) {
        if (maxNumSamples <= 0) {
            throw new IllegalArgumentException("maxNumSamples must be positive");
        }
        this.maxNumSamples = maxNumSamples;
    }

    /**
     * Gets threshold to consider that calibration has converged.
     *
     * @return threshold to consider that calibration has converged.
     */
    public double getConvergenceThreshold() {
        return convergenceThreshold;
    }

    /**
     * Specifies threshold to determine that calibration has converged.
     *
     * @param convergenceThreshold threshold to determine that calibration has
     *                             converged.
     * @throws IllegalArgumentException if threshold is negative.
     */
    public void setConvergenceThreshold(final double convergenceThreshold) {
        if (convergenceThreshold < 0.0) {
            throw new IllegalArgumentException("convergenceThreshold must be positive");
        }
        this.convergenceThreshold = convergenceThreshold;
    }

    /**
     * Resets calibrator.
     */
    public void reset() {
        converged = failed = false;
        sampleCount = 0;
        finished = false;
        Arrays.fill(sample, 0.0);
        Arrays.fill(previousMean, 0.0);
        previousCovariance.initialize(0.0);
        try {
            estimator = new MeasurementNoiseCovarianceEstimator(sample.length);
        } catch (final SignalProcessingException e) {
            // never thrown
        }
        sampleCount = 0;
        Arrays.fill(meanDiff, 0.0);
        covDiff.initialize(0.0);
        accelerometerTimestampNanos = gyroscopeTimestampNanos = -1;
        accumulatedAccelerometerSamples = accumulatedGyroscopeSamples = 0;
        accumulatedAccelerationSampleX = accumulatedAccelerationSampleY = accumulatedAccelerationSampleZ = 0.0;
        accumulatedAngularSpeedSampleX = accumulatedAngularSpeedSampleY = accumulatedAngularSpeedSampleZ = 0.0;
    }

    /**
     * Indicates whether accumulation of samples is enabled or not.
     *
     * @return true if accumulation of samples is enabled, false otherwise.
     */
    public boolean isAccumulationEnabled() {
        return accumulationEnabled;
    }

    /**
     * Specifies whether accumulation of samples is enabled or not.
     *
     * @param accumulationEnabled true if accumulation of samples is enabled,
     *                            false otherwise.
     */
    public void setAccumulationEnabled(final boolean accumulationEnabled) {
        this.accumulationEnabled = accumulationEnabled;
    }

    /**
     * Gets timestamp expressed in nanoseconds since the epoch time of the last
     * accelerometer sample, or -1 if no sample has been set yet.
     *
     * @return timestamp expressed in nanoseconds since the epoch time of the
     * last accelerometer sample, or -1.
     */
    public long getAccelerometerTimestampNanos() {
        return accelerometerTimestampNanos;
    }

    /**
     * Gets timestamp expressed in nanoseconds since the epoch time of the last
     * gyroscope sample, or -1 if no sample has been set yet.
     *
     * @return timestamp expressed in nanoseconds since the epoch time of the
     * last gyroscope sample, or -1.
     */
    public long getGyroscopeTimestampNanos() {
        return gyroscopeTimestampNanos;
    }

    /**
     * Gets number of accelerometer samples accumulated since last full sample.
     *
     * @return number of accelerometer samples accumulated since last full
     * sample.
     */
    public int getAccumulatedAccelerometerSamples() {
        return accumulatedAccelerometerSamples;
    }

    /**
     * Gets number of gyroscope samples accumulated since last full sample.
     *
     * @return number of gyroscope samples accumulated since last full sample.
     */
    public int getAccumulatedGyroscopeSamples() {
        return accumulatedGyroscopeSamples;
    }

    /**
     * Indicates whether the accelerometer sample has been received since the
     * last full sample (accelerometer + gyroscope).
     *
     * @return true if accelerometer sample has been received, false otherwise.
     */
    public boolean isAccelerometerSampleReceived() {
        return accumulatedAccelerometerSamples > 0;
    }

    /**
     * Indicates whether the gyroscope sample has been received since the last
     * full sample (accelerometer + gyroscope).
     *
     * @return true if gyroscope sample has been received, false otherwise.
     */
    public boolean isGyroscopeSampleReceived() {
        return accumulatedGyroscopeSamples > 0;
    }

    /**
     * Indicates whether a full sample (accelerometer + gyroscope) has been
     * received or not.
     *
     * @return true if full sample has been received, false otherwise.
     */
    public boolean isFullSampleAvailable() {
        return isAccelerometerSampleReceived() && isGyroscopeSampleReceived();
    }

    /**
     * Gets average acceleration along x-axis accumulated since last full
     * sample. Expressed in meters per squared second (m/s^2).
     *
     * @return average acceleration along x-axis accumulated since last full
     * sample.
     */
    public double getAccumulatedAccelerationSampleX() {
        return accumulatedAccelerationSampleX;
    }

    /**
     * Gets average acceleration along y-axis accumulated since last full
     * sample. Expressed in meters per squared second (m/s^2).
     *
     * @return average acceleration along y-axis accumulated since last full
     * sample.
     */
    public double getAccumulatedAccelerationSampleY() {
        return accumulatedAccelerationSampleY;
    }

    /**
     * Gets average acceleration along z-axis accumulated since last full
     * sample. Expressed in meters per squared second (m/s^2).
     *
     * @return average acceleration along z-axis accumulated since last full
     * sample.
     */
    public double getAccumulatedAccelerationSampleZ() {
        return accumulatedAccelerationSampleZ;
    }

    /**
     * Gets average acceleration along x,y,z axes accumulated since last full
     * sample. Expressed in meters per squared second (m/s^2).
     *
     * @return average acceleration along x,y,z axes expressed in meters per
     * squared second (m/s^2).
     */
    public double[] getAccumulatedAccelerationSample() {
        return new double[]{
                accumulatedAccelerationSampleX,
                accumulatedAccelerationSampleY,
                accumulatedAccelerationSampleZ
        };
    }

    /**
     * Gets average acceleration along x,yz axes accumulated since last full
     * sample. Expressed in meters per squared second (m/s^2).
     *
     * @param result array where average acceleration along x,y,z axes will be
     *               stored.
     * @throws IllegalArgumentException if provided array does not have length
     *                                  3.
     */
    public void getAccumulatedAccelerationSample(final double[] result) {
        if (result.length != N_COMPONENTS_3D) {
            throw new IllegalArgumentException("result must have length 3");
        }
        result[0] = accumulatedAccelerationSampleX;
        result[1] = accumulatedAccelerationSampleY;
        result[2] = accumulatedAccelerationSampleZ;
    }

    /**
     * Gets average angular speed along x-axis accumulated since last full
     * sample. Expressed in radians per second (rad/s).
     *
     * @return average angular speed along x-axis expressed in radians per
     * second (rad/s).
     */
    public double getAccumulatedAngularSpeedSampleX() {
        return accumulatedAngularSpeedSampleX;
    }

    /**
     * Gets average angular speed along y-axis accumulated since last full
     * sample. Expressed in radians per second (rad/s).
     *
     * @return average angular speed along y-axis expressed in radians per
     * second (rad/s).
     */
    public double getAccumulatedAngularSpeedSampleY() {
        return accumulatedAngularSpeedSampleY;
    }

    /**
     * Gets average angular speed along z-axis accumulated since last full
     * sample. Expressed in radians per second (rad/s).
     *
     * @return average angular speed along z-axis expressed in radians per
     * second (rad/s).
     */
    public double getAccumulatedAngularSpeedSampleZ() {
        return accumulatedAngularSpeedSampleZ;
    }

    /**
     * Gets average angular speed along x,y,z axes accumulated since last full
     * sample. Expressed in radians per second (rad/s).
     *
     * @return average angular speed along x,y,z axes expressed in radians per
     * second.
     */
    public double[] getAccumulatedAngularSpeedSample() {
        return new double[]{
                accumulatedAngularSpeedSampleX,
                accumulatedAngularSpeedSampleY,
                accumulatedAngularSpeedSampleZ
        };
    }

    /**
     * Gets average angular speed along x,y,z axes accumulated since last full
     * sample. Expressed in radians per second (rad/s).
     *
     * @param result array where average angular speed along x,y,z axes will be
     *               stored.
     * @throws IllegalArgumentException if provided array does not have length
     *                                  3.
     */
    public void getAccumulatedAngularSpeedSample(final double[] result) {
        if (result.length != N_COMPONENTS_3D) {
            throw new IllegalArgumentException("result must have length 3");
        }
        result[0] = accumulatedAngularSpeedSampleX;
        result[1] = accumulatedAngularSpeedSampleY;
        result[2] = accumulatedAngularSpeedSampleZ;
    }

    /**
     * Provides a new accelerometer sample.
     * If accumulation is enabled, samples are averaged until a full sample is
     * received.
     * When a full sample (accelerometer + gyroscope) is received, internal
     * state gets also updated.
     *
     * @param timestamp     timestamp of accelerometer sample since epoch time and
     * @param accelerationX linear acceleration along x-axis expressed in meters
     *                      per squared second (m/s^2).
     * @param accelerationY linear acceleration along y-axis expressed in meters
     *                      per squared second (m/s^2).
     * @param accelerationZ linear acceleration along z-axis expressed in meters
     *                      per squared second (m/s^2).
     */
    public void updateAccelerometerSample(
            final long timestamp, final float accelerationX, final float accelerationY, final float accelerationZ) {
        if (!isFullSampleAvailable()) {
            accelerometerTimestampNanos = timestamp;
            if (isAccumulationEnabled() && isAccelerometerSampleReceived()) {
                // accumulation enabled
                final var nextSamples = accumulatedAccelerometerSamples + 1;
                accumulatedAccelerationSampleX = (accumulatedAccelerationSampleX * accumulatedAccelerometerSamples
                        + accelerationX) / nextSamples;
                accumulatedAccelerationSampleY = (accumulatedAccelerationSampleY * accumulatedAccelerometerSamples
                        + accelerationY) / nextSamples;
                accumulatedAccelerationSampleZ = (accumulatedAccelerationSampleZ * accumulatedAccelerometerSamples
                        + accelerationZ) / nextSamples;
                accumulatedAccelerometerSamples = nextSamples;
            } else {
                // accumulation disabled
                accumulatedAccelerationSampleX = accelerationX;
                accumulatedAccelerationSampleY = accelerationY;
                accumulatedAccelerationSampleZ = accelerationZ;
                accumulatedAccelerometerSamples++;
            }
            notifyFullSampleAndResetSampleReceive();
        }
    }

    /**
     * Provides a new accelerometer sample.
     * If accumulation is enabled, samples are averaged until a full sample is
     * received.
     * When a full sample (accelerometer + gyroscope) is received, internal
     * state gets also updated.
     *
     * @param timestamp timestamp of accelerometer sample since epoch time and
     *                  expressed in nanoseconds.
     * @param data      array containing x,y,z components of linear acceleration
     *                  expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided array does not have length
     *                                  3.
     */
    public void updateAccelerometerSample(final long timestamp, final float[] data) {
        if (data.length != N_COMPONENTS_3D) {
            throw new IllegalArgumentException("acceleration must have length 3");
        }
        updateAccelerometerSample(timestamp, data[0], data[1], data[2]);
    }

    /**
     * Provides a new gyroscope sample.
     * If accumulation is enabled, samples are averaged until a full sample is
     * received.
     * When a full sample (accelerometer + gyroscope) is received, internal
     * state gets also updated.
     *
     * @param timestamp     timestamp of accelerometer sample since epoch time and
     *                      expressed in nanoseconds.
     * @param angularSpeedX angular speed of rotation along x-axis expressed in
     *                      radians per second (rad/s).
     * @param angularSpeedY angular speed of rotation along y-axis expressed in
     *                      radians per second (rad/s).
     * @param angularSpeedZ angular speed of rotation along z-axis expressed in
     *                      radians per second (rad/s).
     */
    public void updateGyroscopeSample(
            final long timestamp, final float angularSpeedX, final float angularSpeedY, final float angularSpeedZ) {
        if (!isFullSampleAvailable()) {
            gyroscopeTimestampNanos = timestamp;
            if (isAccumulationEnabled() && isGyroscopeSampleReceived()) {
                // accumulation enabled
                final var nextSamples = accumulatedGyroscopeSamples + 1;
                accumulatedAngularSpeedSampleX = (accumulatedAngularSpeedSampleX * accumulatedGyroscopeSamples
                        + angularSpeedX) / nextSamples;
                accumulatedAngularSpeedSampleY = (accumulatedAngularSpeedSampleY * accumulatedGyroscopeSamples
                        + angularSpeedY) / nextSamples;
                accumulatedAngularSpeedSampleZ = (accumulatedAngularSpeedSampleZ * accumulatedGyroscopeSamples
                        + angularSpeedZ) / nextSamples;
                accumulatedGyroscopeSamples = nextSamples;
            } else {
                // accumulation disabled
                accumulatedAngularSpeedSampleX = angularSpeedX;
                accumulatedAngularSpeedSampleY = angularSpeedY;
                accumulatedAngularSpeedSampleZ = angularSpeedZ;
                accumulatedGyroscopeSamples++;
            }
            notifyFullSampleAndResetSampleReceive();
        }
    }

    /**
     * Provides a new gyroscope sample.
     * If accumulation is enabled, samples are averaged until a full sample is
     * received.
     * When a full sample (accelerometer + gyroscope) is received, internal
     * state gets also updated.
     *
     * @param timestamp timestamp of gyroscope sample since epoch time and
     *                  expressed in nanoseconds.
     * @param data      angular speed of rotation along x,y,z axes expressed in
     *                  radians per second (rad/s).
     * @throws IllegalArgumentException if provided array does not have length
     *                                  3.
     */
    public void updateGyroscopeSample(final long timestamp, final float[] data) {
        if (data.length != N_COMPONENTS_3D) {
            throw new IllegalArgumentException("angular speed must have length 3");
        }
        updateGyroscopeSample(timestamp, data[0], data[1], data[2]);
    }

    /**
     * Gets most recent timestamp of received partial samples (accelerometer or
     * gyroscope).
     *
     * @return most recent timestamp of received partial sample.
     */
    public long getMostRecentTimestampNanos() {
        return Math.max(accelerometerTimestampNanos, gyroscopeTimestampNanos);
    }

    /**
     * Gets listener in charge of handling events raised by instances of this
     * class.
     *
     * @return listener in charge of handling events raised by instances of this
     * class.
     */
    public BaseSlamCalibratorListener<D> getListener() {
        return listener;
    }

    /**
     * Sets listener in charge of handling events raised by instances of this
     * class.
     *
     * @param listener listener in charge of handling events raised by instances
     *                 of this class.
     */
    public void setListener(final BaseSlamCalibratorListener<D> listener) {
        this.listener = listener;
    }

    /**
     * Obtains mean values of control signal used for SLAM estimation during
     * prediction stage of Kalman filter in order to correct possible biases
     * and offsets.
     *
     * @return mean values of control signal.
     */
    public double[] getControlMean() {
        return estimator.getSampleAverage();
    }

    /**
     * Obtains mean values of control signal used for SLAM estimation during
     * prediction stage of Kalman filter in order to correct possible biases
     * and offsets.
     *
     * @param result array where mean values of control signal will be stored.
     *               Array must have the same length as the control signal.
     * @throws IllegalArgumentException if provided length is invalid.
     */
    public void getControlMean(final double[] result) {
        final var src = getControlMean();
        if (result.length != src.length) {
            throw new IllegalArgumentException("wrong length");
        }

        System.arraycopy(src, 0, result, 0, src.length);
    }

    /**
     * Gets covariance of control signal used for SLAM estimation during
     * prediction stage of Kalman filter in order to correct possible biases
     * and offsets.
     *
     * @return covariance matrix of control signal.
     */
    public Matrix getControlCovariance() {
        return estimator.getMeasurementNoiseCov();
    }

    /**
     * Gets covariance of control signal used for SLAM estimation during
     * prediction stage of Kalman filter in order to correct possible biases
     * and offsets.
     *
     * @param result matrix where covariance will be stored.
     */
    public void getControlCovariance(final Matrix result) {
        final var src = getControlCovariance();
        src.copyTo(result);
    }

    /**
     * Gets a multivariate normal distribution containing control signal mean
     * and covariance used for SLAM estimation during prediction stage of Kalman
     * filter in order to correct possible biases and offsets.
     *
     * @return a multivariate normal distribution.
     * @throws InvalidCovarianceMatrixException if estimated covariance is not
     *                                          valid.
     */
    public MultivariateNormalDist getControlDistribution() throws InvalidCovarianceMatrixException {
        final var cov = getControlCovariance();
        try {
            cov.symmetrize();
        } catch (final WrongSizeException ignore) {
            // never thrown
        }
        return new MultivariateNormalDist(getControlMean(), cov, false);
    }

    /**
     * Gets a multivariate normal distribution containing control signal mean
     * and covariance used for SLAM estimation during prediction stage of Kalman
     * filter in order to correct possible biases and offsets.
     *
     * @param dist a multivariate normal distribution.
     * @throws InvalidCovarianceMatrixException if estimated covariance is not
     *                                          valid.
     */
    public void getControlDistribution(final MultivariateNormalDist dist) throws InvalidCovarianceMatrixException {
        final var cov = getControlCovariance();
        try {
            cov.symmetrize();
        } catch (final WrongSizeException ignore) {
            // never thrown
        }
        dist.setMeanAndCovariance(getControlMean(), cov, false);
    }

    /**
     * Gets a new instance containing calibration data estimated by this
     * calibrator.
     *
     * @return a new calibration data instance.
     */
    public abstract D getCalibrationData();

    /**
     * Gets calibration data estimated by this calibrator.
     *
     * @param result instance where calibration data will be stored.
     */
    public void getCalibrationData(final D result) {
        result.setControlMeanAndCovariance(getControlMean(), getControlCovariance());
    }

    /**
     * Propagates calibrated control signal covariance using current control
     * jacobian matrix.
     * The propagated distribution can be used during prediction stage in Kalman
     * filtering.
     *
     * @param controlJacobian current control jacobian matrix.
     * @return propagated distribution.
     * @throws InvalidCovarianceMatrixException if estimated covariance is not
     *                                          valid.
     */
    public MultivariateNormalDist propagateWithControlJacobian(final Matrix controlJacobian)
            throws InvalidCovarianceMatrixException {
        return getCalibrationData().propagateWithControlJacobian(controlJacobian);
    }

    /**
     * Propagates calibrated control signal covariance using current control
     * jacobian matrix.
     * The propagated distribution can be used during prediction stage in Kalman
     * filtering.
     *
     * @param controlJacobian current control jacobian matrix.
     * @param result          instance where propagated distribution is stored.
     * @throws InvalidCovarianceMatrixException if estimated covariance is not
     *                                          valid.
     */
    public void propagateWithControlJacobian(final Matrix controlJacobian, final MultivariateNormalDist result)
            throws InvalidCovarianceMatrixException {
        getCalibrationData().propagateWithControlJacobian(controlJacobian, result);
    }

    /**
     * Notifies that a full sample has been received and resets flags indicating
     * whether partial samples have been received.
     */
    protected void notifyFullSampleAndResetSampleReceive() {
        if (isFullSampleAvailable()) {
            processFullSample();
            accumulatedAccelerometerSamples = accumulatedGyroscopeSamples = 0;
        }
    }

    /**
     * Obtains the number of state parameters in associated SLAM estimator.
     *
     * @return number of state parameters.
     */
    protected abstract int getEstimatorStateLength();

    /**
     * Method to be implemented in subclasses to process a full sample.
     */
    protected abstract void processFullSample();

    /**
     * Updates internal mean and covariance values and checks whether
     * convergence has been reached and calibrator has finished or failed.
     */
    protected void updateSample() {
        if (finished) {
            return;
        }

        try {
            estimator.update(sample);
        } catch (final SignalProcessingException e) {
            failed = finished = true;

            if (listener != null) {
                listener.onCalibratorFinished(this, converged, true);
            }
            return;
        }

        sampleCount++;

        final var mean = estimator.getSampleAverage();
        final var cov = estimator.getMeasurementNoiseCov();

        // check if minimum number of samples has been reached
        if (sampleCount >= maxNumSamples) {
            finished = true;

            if (listener != null) {
                listener.onCalibratorFinished(this, converged, failed);
            }
            return;
        }

        // check if estimator has converged
        if (sampleCount >= minNumSamples) {
            ArrayUtils.subtract(mean, previousMean, meanDiff);
            try {
                cov.subtract(previousCovariance, covDiff);
            } catch (final WrongSizeException ignore) {
                // never thrown
            }
            final var meanDiffNorm = com.irurueta.algebra.Utils.normF(meanDiff);
            final var covDiffNorm = com.irurueta.algebra.Utils.normF(covDiff);
            if (meanDiffNorm <= convergenceThreshold && covDiffNorm <= convergenceThreshold) {
                converged = finished = true;

                if (listener != null) {
                    listener.onCalibratorFinished(this, true, failed);
                }
                return;
            }
        }

        // copy current value for next iteration
        System.arraycopy(mean, 0, previousMean, 0, mean.length);
        cov.copyTo(previousCovariance);

        finished = false;
    }

    /**
     * Listener for implementations of this class.
     */
    public interface BaseSlamCalibratorListener<D extends BaseCalibrationData> {
        /**
         * Called when a full sample (accelerometer + gyroscope, etc.) has been
         * received.
         *
         * @param calibrator SLAM calibrator.
         */
        void onFullSampleReceived(final BaseSlamCalibrator<D> calibrator);

        /**
         * Called when a full sample (accelerometer + gyroscope, etc.) has been
         * received and has already been processed.
         *
         * @param calibrator SLAM calibrator.
         */
        void onFullSampleProcessed(final BaseSlamCalibrator<D> calibrator);

        /**
         * Called when calibration finishes.
         *
         * @param calibrator SLAM calibrator.
         * @param converged  true if calibration converged, false otherwise.
         * @param failed     true if calibration failed, false otherwise.
         */
        void onCalibratorFinished(
                final BaseSlamCalibrator<D> calibrator, final boolean converged, final boolean failed);
    }
}
