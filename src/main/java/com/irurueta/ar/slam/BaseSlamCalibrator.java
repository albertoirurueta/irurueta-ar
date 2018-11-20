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

import java.io.Serializable;
import java.util.Arrays;

/**
 * Base class for estimating mean and covariance of noise in control values
 * when the system state is held constant (only noise is provided as control 
 * input).
 * @param <D> type of calibration data.
 */
@SuppressWarnings("WeakerAccess")
public abstract class BaseSlamCalibrator<D extends BaseCalibrationData> 
        implements Serializable {
            
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
    private int mSampleLength;
    
    /**
     * Array containing a control sample used during SLAM prediction stage.
     */
    protected double[] mSample;
    
    /**
     * Mean and covariance estimator.
     */
    protected MeasurementNoiseCovarianceEstimator mEstimator;
    
    /**
     * Contains previous mean value.
     */
    protected double[] mPreviousMean;
    
    /**
     * Contains mean value of covariance.
     */
    protected Matrix mPreviousCovariance;
    
    /**
     * Indicates whether this calibrator converged.
     */
    protected boolean mConverged;
    
    /**
     * Indicates whether this calibrator failed.
     */
    protected boolean mFailed;
    
    /**
     * Indicates whether calibrator has finished taking samples.
     */
    protected boolean mFinished;
    
    /**
     * Number of obtained samples.
     */
    protected int mSampleCount;
    
    /**
     * Array to store the difference between average values to determine whether
     * the result has converged or not.
     */
    protected double[] mMeanDiff;
    
    /**
     * Matrix to store the difference between covariance matrices to determine
     * whether the result has converged or not.
     */
    protected Matrix mCovDiff;
    
    /**
     * Minimum number of samples to take into account.
     */
    protected int mMinNumSamples = DEFAULT_MIN_NUM_SAMPLES;
    
    /**
     * Maximum number of samples to take into account.
     */
    protected int mMaxNumSamples = DEFAULT_MAX_NUM_SAMPLES;
    
    /**
     * Threshold to consider whether calibration has converged or not.
     */
    protected double mConvergenceThreshold = DEFAULT_CONVERGENCE_THRESHOLD;
    
    /**
     * Indicates whether accumulation of samples is enabled or not.
     */
    protected boolean mAccumulationEnabled = DEFAULT_ENABLE_SAMPLE_ACCUMULATION;
    
    /**
     * Timestamp expressed in nanoseconds since the epoch time of the last
     * accelerometer sample.
     */
    protected long mAccelerometerTimestampNanos = -1;
    
    /**
     * Timestamp expressed in nanoseconds since the epoch time of the last
     * gyroscope sample.
     */
    protected long mGyroscopeTimestampNanos = -1;
    
    /**
     * Number of accelerometer samples accumulated since last full sample.
     */
    protected int mAccumulatedAccelerometerSamples = 0;
    
    /**
     * Number of gyroscope samples accumulated since last full sample.
     */
    protected int mAccumulatedGyroscopeSamples = 0;
    
    /**
     * Average of acceleration along x-axis accumulated since last full sample.
     * Expressed in meters per squared second (m/s^2).
     */
    protected double mAccumulatedAccelerationSampleX;
    
    /**
     * Average of acceleration along y-axis accumulated since last full sample.
     * Expressed in meters per squared second (m/s^2).
     */
    protected double mAccumulatedAccelerationSampleY;
    
    /**
     * Average of acceleration along z-axis accumulated since last full sample.
     * Expressed in meters per squared second (m/s^2).
     */
    protected double mAccumulatedAccelerationSampleZ;
    
    /**
     * Average of angular speed along x-axis accumulated since last full sample.
     * Expressed in meters per squared second (m/s^2).
     */
    protected double mAccumulatedAngularSpeedSampleX;
    
    /**
     * Average of angular speed along y-axis accumulated since last full sample.
     * Expressed in meters per squared second (m/s^2).
     */
    protected double mAccumulatedAngularSpeedSampleY;
    
    /**
     * Average of angular speed along z-axis accumulated since last full sample.
     * Expressed in meters per squared second (m/s^2).
     */
    protected double mAccumulatedAngularSpeedSampleZ;
    
    /**
     * Listener in charge of handling events raised by instances of this class.
     */
    protected BaseSlamCalibratorListener mListener;
    
    /**
     * Constructor.
     * @param sampleLength sample length of control values used during 
     * prediction stage in SLAM estimator.
     * @throws IllegalArgumentException if sample length is less than 1.
     */
    public BaseSlamCalibrator(int sampleLength) 
            throws IllegalArgumentException {
        if(sampleLength < MIN_SAMPLE_LENGTH) {
            throw new IllegalArgumentException("length must be greater than 0");
        }
        
        mSampleLength = sampleLength;
        mSample = new double[sampleLength];
        mPreviousMean = new double[sampleLength];
        mMeanDiff = new double[sampleLength];
        try {
            mPreviousCovariance = new Matrix(sampleLength, sampleLength);
            mCovDiff = new Matrix(sampleLength, sampleLength);
            mEstimator = new MeasurementNoiseCovarianceEstimator(
                    sampleLength);            
        } catch (Exception ignore) {
            /* never thrown */
        }
    }
    
    /**
     * Gets sample length of control values used during prediction stage in SLAM 
     * estimator.
     * @return sample length of control values used during prediction stage in
     * SLAM estimator.
     */
    public int getSampleLength() {
        return mSampleLength;
    }        
    
    /**
     * Indicates whether calibrator converged or not.
     * @return true if calibrator converged, false otherwise.
     */
    public boolean isConverged() {
        return mConverged;
    }
    
    /**
     * Indicates whether this calibrator failed or not.
     * @return true if calibrator failed, false otherwise.
     */
    public boolean isFailed() {
        return mFailed;
    }
    
    /**
     * Indicates whether calibrator has finished taking samples or not.
     * @return true if calibrator has finished taing samples, false otherwise.
     */
    public boolean isFinished() {
        return mFinished;
    }
    
    /**
     * Gets number of obtained samples.
     * @return number of obtained samples.
     */
    public int getSampleCount() {
        return mSampleCount;
    }
    
    /**
     * Obtains the minimum number of samples to use before taking convergence 
     * into account.
     * @return minimum number of samples to use before taking convergence into 
     * account.
     */
    public int getMinNumSamples() {
        return mMinNumSamples;
    }
    
    /**
     * Specifies the minimum number of samples to take before taking convergence
     * into account.
     * @param minNumSamples minimum number of samples to take before taking
     * convergence into account.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setMinNumSamples(int minNumSamples) 
            throws IllegalArgumentException {
        if (minNumSamples < 0) {
            throw new IllegalArgumentException(
                    "minNumSamples must be positive");
        }
        mMinNumSamples = minNumSamples;
    }
    
    /**
     * Gets maximum number of samples to take into account.
     * @return maximum number of samples to take into account.
     */
    public int getMaxNumSamples() {
        return mMaxNumSamples;
    }
    
    /**
     * Specifies the maximum number of samples to take.
     * @param maxNumSamples maximum number of samples to take.
     * @throws IllegalArgumentException if provided value is negative or zero.
     */
    public void setMaxNumSamples(int maxNumSamples) 
            throws IllegalArgumentException {
        if (maxNumSamples <= 0) {
            throw new IllegalArgumentException(
                    "maxNumSamples must be positive");
        }
        mMaxNumSamples = maxNumSamples;
    }
    
    /**
     * Gets threshold to consider that calibration has converged.
     * @return threshold to consider that calibration has converged.
     */
    public double getConvergenceThreshold() {
        return mConvergenceThreshold;
    }
    
    /**
     * Specifies threshold to determine that calibration has converged.
     * @param convergenceThreshold threshold to determine that calibration has 
     * converged.
     * @throws IllegalArgumentException if threshold is negative.
     */
    public void setConvergenceThreshold(double convergenceThreshold)
            throws IllegalArgumentException {
        if (convergenceThreshold < 0.0) {
            throw new IllegalArgumentException(
                    "convergenceThreshold must be positive");
        }
        mConvergenceThreshold = convergenceThreshold;
    }
    
    /**
     * Resets calibrator.
     */
    public void reset() {
        mConverged = mFailed = false;
        mSampleCount = 0;
        mFinished = false;
        Arrays.fill(mSample, 0.0);
        Arrays.fill(mPreviousMean, 0.0);
        mPreviousCovariance.initialize(0.0);
        try {
            mEstimator = new MeasurementNoiseCovarianceEstimator(
                    mSample.length);
        }catch(SignalProcessingException e) { /* never thrown */ }        
        mConverged = mFailed = mFinished = false;
        mSampleCount = 0;
        Arrays.fill(mMeanDiff, 0.0);
        mCovDiff.initialize(0.0);
        mAccelerometerTimestampNanos = mGyroscopeTimestampNanos = -1;
        mAccumulatedAccelerometerSamples = mAccumulatedGyroscopeSamples = 0;
        mAccumulatedAccelerationSampleX = mAccumulatedAccelerationSampleY =
                mAccumulatedAccelerationSampleZ = 0.0;
        mAccumulatedAngularSpeedSampleX = mAccumulatedAngularSpeedSampleY =
                mAccumulatedAngularSpeedSampleZ = 0.0;
    }
    
    /**
     * Indicates whether accumulation of samples is enabled or not.
     * @return true if accumulation of samples is enabled, false otherwise.
     */
    public boolean isAccumulationEnabled() {
        return mAccumulationEnabled;
    }
    
    /**
     * Specifies whether accumulation of samples is enabled or not.
     * @param accumulationEnabled true if accumulation of samples is enabled,
     * false otherwise.
     */
    public void setAccumulationEnabled(boolean accumulationEnabled) {
        mAccumulationEnabled = accumulationEnabled;
    }
    
    /**
     * Gets timestamp expressed in nanoseconds since the epoch time of the last
     * accelerometer sample, or -1 if no sample has been set yet.
     * @return timestamp expressed in nanoseconds since the epoch time of the
     * last accelerometer sample, or -1.
     */
    public long getAccelerometerTimestampNanos() {
        return mAccelerometerTimestampNanos;
    }
    
    /**
     * Gets timestamp expressed in nanoseconds since the epoch time of the last
     * gyroscope sample, or -1 if no sample has been set yet.
     * @return timestamp expressed in nanoseconds since the epoch time of the 
     * last gyroscope sample, or -1.
     */
    public long getGyroscopeTimestampNanos() {
        return mGyroscopeTimestampNanos;
    }
    
    /**
     * Gets number of accelerometer samples accumulated since last full sample.
     * @return number of accelerometer samples accumulated since last full 
     * sample.
     */
    public int getAccumulatedAccelerometerSamples() {
        return mAccumulatedAccelerometerSamples;
    }
    
    /**
     * Gets number of gyroscope samples accumulated since last full sample.
     * @return number of gyroscope samples accumulated since last full sample.
     */
    public int getAccumulatedGyroscopeSamples() {
        return mAccumulatedGyroscopeSamples;
    }
    
    /**
     * Indicates whether the accelerometer sample has been received since the
     * last full sample (accelerometer + gyroscope).
     * @return true if accelerometer sample has been received, false otherwise.
     */
    public boolean isAccelerometerSampleReceived() {
        return mAccumulatedAccelerometerSamples > 0;
    }
    
    /**
     * Indicates whether the gyroscope sample has been received since the last
     * full sample (accelerometer + gyroscope).
     * @return true if gyroscope sample has been received, false otherwise.
     */
    public boolean isGyroscopeSampleReceived() {
        return mAccumulatedGyroscopeSamples > 0;
    }
    
    /**
     * Indicates whether a full sample (accelerometer + gyroscope) has been
     * received or not.
     * @return true if full sample has been received, false otherwise.
     */
    public boolean isFullSampleAvailable() {
        return isAccelerometerSampleReceived() && isGyroscopeSampleReceived();
    }
    
    /**
     * Gets average acceleration along x-axis accumulated since last full
     * sample. Expressed in meters per squared second (m/s^2).
     * @return average acceleration along x-axis accumulated since last full
     * sample.
     */
    public double getAccumulatedAccelerationSampleX() {
        return mAccumulatedAccelerationSampleX;
    }
    
    /**
     * Gets average acceleration along y-axis accumulated since last full
     * sample. Expressed in meters per squared second (m/s^2).
     * @return average acceleration along y-axis accumulated since last full 
     * sample.
     */
    public double getAccumulatedAccelerationSampleY() {
        return mAccumulatedAccelerationSampleY;
    }
    
    /**
     * Gets acerage acceleration along z-axis accumulated since last full 
     * sample. Expressed in meters per squared second (m/s^2).
     * @return average acceleration along z-axis accumulated since last full
     * sample.
     */
    public double getAccumulatedAccelerationSampleZ() {
        return mAccumulatedAccelerationSampleZ;
    }
    
    /**
     * Gets average acceleration along x,y,z axes accumulated since last full
     * sample. Expressed in meters per squared second (m/s^2).
     * @return average acceleration along x,y,z axes expressed in meters per
     * squared second (m/s^2).
     */
    public double[] getAccumulatedAccelerationSample() {
        return new double[]{
            mAccumulatedAccelerationSampleX,
            mAccumulatedAccelerationSampleY,
            mAccumulatedAccelerationSampleZ
        };
    }
    
    /**
     * Gets average acceleration along x,yz axes accumulated since last full
     * sample. Expressed in meters per squared second (m/s^2).
     * @param result array where average acceleration along x,y,z axes will be
     * stored.
     * @throws IllegalArgumentException if provided array does not have length 
     * 3.
     */
    public void getAccumulatedAccelerationSample(double[] result)
            throws IllegalArgumentException {
        if(result.length != N_COMPONENTS_3D) {
            throw new IllegalArgumentException("result must have length 3");
        }
        result[0] = mAccumulatedAccelerationSampleX;
        result[1] = mAccumulatedAccelerationSampleY;
        result[2] = mAccumulatedAccelerationSampleZ;
    }
    
    /**
     * Gets average angular speed along x-axis accumulated since last full 
     * sample. Expressed in radians per second (rad/s).
     * @return average angular speed along x-axis expressed in radians per 
     * second (rad/s).
     */
    public double getAccumulatedAngularSpeedSampleX() {
        return mAccumulatedAngularSpeedSampleX;
    }
    
    /**
     * Gets average angular speed along y-axis accumulated since last full
     * sample. Expressed in radians per second (rad/s).
     * @return average angular speed along y-axis expressed in radians per
     * second (rad/s).
     */
    public double getAccumulatedAngularSpeedSampleY() {
        return mAccumulatedAngularSpeedSampleY;
    }
    
    /**
     * Gets average angular speed along z-axis accumulated since last full
     * sample. Expressed in radians per second (rad/s).
     * @return average angular speed along z-axis expressed in radians per
     * second (rad/s).
     */
    public double getAccumulatedAngularSpeedSampleZ() {
        return mAccumulatedAngularSpeedSampleZ;
    }
    
    /**
     * Gets average angular speed along x,y,z axes accumulated since last full
     * sample. Expressed in radians per second (rad/s).
     * @return average angular speed along x,y,z axes expressed in radians per
     * second.
     */
    public double[] getAccumulatedAngularSpeedSample() {
        return new double[]{
            mAccumulatedAngularSpeedSampleX,
            mAccumulatedAngularSpeedSampleY,
            mAccumulatedAngularSpeedSampleZ
        };
    }
    
    /**
     * Gets average angular speed along x,y,z axes accumulated since last full
     * sample. Expressed in radians per second (rad/s).
     * @param result array where average angular speed along x,y,z axes will be
     * stored.
     * @throws IllegalArgumentException if provided array does not have length 
     * 3.
     */
    public void getAccumulatedAngularSpeedSample(double[] result)
            throws IllegalArgumentException {
        if (result.length != N_COMPONENTS_3D) {
            throw new IllegalArgumentException("result must have length 3");
        }
        result[0] = mAccumulatedAngularSpeedSampleX;
        result[1] = mAccumulatedAngularSpeedSampleY;
        result[2] = mAccumulatedAngularSpeedSampleZ;
    }
    
    /**
     * Provides a new accelerometer sample.
     * If accumulation is enabled, samples are averaged until a full sample is
     * received.
     * When a full sample (accelerometer + gyroscope) is received, internal
     * state gets also updated.
     * @param timestamp timestamp of accelerometer sample since epoch time and
     * @param accelerationX linear acceleration along x-axis expressed in meters 
     * per squared second (m/s^2).
     * @param accelerationY linear acceleration along y-axis expressed in meters
     * per squared second (m/s^2).
     * @param accelerationZ linear acceleration along z-axis expressed in meters
     * per squared second (m/s^2).
     */
    public void updateAccelerometerSample(long timestamp, float accelerationX,
            float accelerationY, float accelerationZ) {
        if(!isFullSampleAvailable()) {
            mAccelerometerTimestampNanos = timestamp;
            if (isAccumulationEnabled() && isAccelerometerSampleReceived()) {
                //accumulation enabled
                int nextSamples = mAccumulatedAccelerometerSamples + 1;
                mAccumulatedAccelerationSampleX = 
                        (mAccumulatedAccelerationSampleX * mAccumulatedAccelerometerSamples + 
                        accelerationX) / nextSamples;
                mAccumulatedAccelerationSampleY =
                        (mAccumulatedAccelerationSampleY * mAccumulatedAccelerometerSamples +
                        accelerationY) / nextSamples;
                mAccumulatedAccelerationSampleZ =
                        (mAccumulatedAccelerationSampleZ * mAccumulatedAccelerometerSamples +
                        accelerationZ) / nextSamples;
                mAccumulatedAccelerometerSamples = nextSamples;                
            } else {
                //accumulation disabled
               mAccumulatedAccelerationSampleX = accelerationX;
               mAccumulatedAccelerationSampleY = accelerationY;
               mAccumulatedAccelerationSampleZ = accelerationZ;
               mAccumulatedAccelerometerSamples++;  
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
     * @param timestamp timestamp of accelerometer sample since epoch time and
     * expressed in nanoseconds.
     * @param data array containing x,y,z components of linear acceleration 
     * expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided array does not have length 
     * 3.
     */
    public void updateAccelerometerSample(long timestamp, float[] data) 
            throws IllegalArgumentException {
        if(data.length != N_COMPONENTS_3D) {
            throw new IllegalArgumentException(
                    "acceleration must have length 3");
        }
        updateAccelerometerSample(timestamp, data[0], data[1],
                data[2]);
    }
    
    /**
     * Provides a new gyroscope sample.
     * If accumulation is enabled, samples are averaged until a full sample is
     * received.
     * When a full sample (accelerometer + gyroscope) is received, internal 
     * state gets also updated.
     * @param timestamp timestamp of accelerometer sample since epoch time and
     * expressed in nanoseconds.
     * @param angularSpeedX angular speed of rotation along x-axis expressed in
     * radians per second (rad/s).
     * @param angularSpeedY angular speed of rotation along y-axis expressed in
     * radians per second (rad/s).
     * @param angularSpeedZ angular speed of rotation along z-axis expressed in
     * radians per second (rad/s).
     */
    public void updateGyroscopeSample(long timestamp, float angularSpeedX,
            float angularSpeedY, float angularSpeedZ) {
        if(!isFullSampleAvailable()) {
            mGyroscopeTimestampNanos = timestamp;
            if(isAccumulationEnabled() && isGyroscopeSampleReceived()) {
                //accumulation enabled
                int nextSamples = mAccumulatedGyroscopeSamples + 1;
                mAccumulatedAngularSpeedSampleX = 
                        (mAccumulatedAngularSpeedSampleX * mAccumulatedGyroscopeSamples +
                        angularSpeedX) / nextSamples;
                mAccumulatedAngularSpeedSampleY = 
                        (mAccumulatedAngularSpeedSampleY * mAccumulatedGyroscopeSamples +
                        angularSpeedY) / nextSamples;
                mAccumulatedAngularSpeedSampleZ = 
                        (mAccumulatedAngularSpeedSampleZ * mAccumulatedGyroscopeSamples +
                        angularSpeedZ) / nextSamples;
                mAccumulatedGyroscopeSamples = nextSamples;
            } else {
                //accumulation disabled
                mAccumulatedAngularSpeedSampleX = angularSpeedX;
                mAccumulatedAngularSpeedSampleY = angularSpeedY;
                mAccumulatedAngularSpeedSampleZ = angularSpeedZ;
                mAccumulatedGyroscopeSamples++;
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
     * @param timestamp timestamp of gyroscope sample since epoch time and
     * expressed in nanoseconds.
     * @param data angular speed of rotation along x,y,z axes expressed in 
     * radians per second (rad/s).
     * @throws IllegalArgumentException if provided array does not have length 
     * 3.
     */
    public void updateGyroscopeSample(long timestamp, float[] data) 
            throws IllegalArgumentException {
        if(data.length != N_COMPONENTS_3D) {
            throw new IllegalArgumentException(
                    "angular speed must have length 3");
        }
        updateGyroscopeSample(timestamp, data[0], data[1], data[2]);
    }
    
    /**
     * Gets most recent timestamp of received partial samples (accelerometer or
     * gyroscope).
     * @return most recent timestamp of received partial sample.
     */
    public long getMostRecentTimestampNanos() {
        return mAccelerometerTimestampNanos > mGyroscopeTimestampNanos ?
                    mAccelerometerTimestampNanos : mGyroscopeTimestampNanos;
    }
    
    /**
     * Gets listener in charge of handling events raised by instances of this 
     * class.
     * @return listener in charge of handling events raised by instances of this 
     * class.
     */
    public BaseSlamCalibratorListener getListener() {
        return mListener;
    }
    
    /**
     * Sets listener in charge of handling events raised by instances of this
     * class.
     * @param listener listener in charge of handling events raised by instances
     * of this class.
     */
    public void setListener(BaseSlamCalibratorListener listener) {
        mListener = listener;
    }
    
    /**
     * Obtains mean values of control signal used for SLAM estimation during
     * prediction stage of Kalman filter in order to corect possible biases
     * and offsets.
     * @return mean values of control signal.
     */
    public double[] getControlMean() {
        return mEstimator.getSampleAverage();
    }
    
    /**
     * Obtains mean values of control signal used for SLAM estimation during
     * prediction stage of Kalman filter in order to correct possible biases
     * and offsets.
     * @param result array where mean values of control signal will be stored.
     * Array must have the same length as the control signal.
     * @throws IllegalArgumentException if provided length is invalid.
     */
    public void getControlMean(double[] result) throws IllegalArgumentException{
        double[] src = getControlMean();
        if(result.length != src.length) {
            throw new IllegalArgumentException("wrong legnth");
        }
        
        System.arraycopy(src, 0, result, 0, src.length);
    }
    
    /**
     * Gets covariance of control signal used for SLAM estimation during
     * prediction stage of Kalman filter in order to correct possible biases
     * and offsets.
     * @return covariance matrix of control signal.
     */
    public Matrix getControlCovariance() {
        return mEstimator.getMeasurementNoiseCov();
    }
    
    /**
     * Gets covariance of control signal used for SLAM estimation during
     * prediction stage of Kalman filter in order to correct possible biases
     * and offsets.
     * @param result matrix where covariance will be stored.
     */
    public void getControlCovariance(Matrix result) {
        Matrix src = getControlCovariance();
        src.copyTo(result);
    }
    
    /**
     * Gets a multivariate normal distribution containing control signal mean
     * and covariance used for SLAM estimation during prediction stage of Kalman
     * filter in order to correct possible biases and offsets.
     * @return a multivariate normal distribution.
     * @throws InvalidCovarianceMatrixException if estimated covariance is not
     * valid.
     */
    public MultivariateNormalDist getControlDistribution() 
            throws InvalidCovarianceMatrixException {
        Matrix cov = getControlCovariance();
        try {
            cov.symmetrize();
        } catch (WrongSizeException ignore) { /* never thrown */ }
        return new MultivariateNormalDist(getControlMean(), cov, false);
    }
    
    /**
     * Gets a multivariate normal distribution containing control signal mean
     * and covariance used for SLAM estimation during prediction stage of Kalman
     * filter in order to correct possible biases and offsets.
     * @param dist a multivariate normal distribution.
     * @throws InvalidCovarianceMatrixException if estimated covariance is not
     * valid.
     */
    public void getControlDistribution(MultivariateNormalDist dist) 
            throws InvalidCovarianceMatrixException {
        Matrix cov = getControlCovariance();
        try {
            cov.symmetrize();
        } catch (WrongSizeException ignore) { /* never thrown */ }        
        dist.setMeanAndCovariance(getControlMean(), cov, false);
    }
    
    /**
     * Gets a new instance containing calibration data estimated by this
     * calibrator.
     * @return a new calibration data instance.
     */
    public abstract D getCalibrationData();
    
    /**
     * Gets calibration data estimated by this calibrator.
     * @param result instance where calibration data will be stored.
     */
    public void getCalibrationData(D result) {
        result.setControlMeanAndCovariance(getControlMean(), 
                getControlCovariance());
    }
    
    /**
     * Propagates calibrated control signal covariance using current control
     * jacobian matrix. 
     * The propagated distribution can be used during prediction stage in Kalman
     * filtering.
     * @param controlJacobian current control jacobian matrix.
     * @return propagated distribution.
     * @throws InvalidCovarianceMatrixException if estimated covariance is not
     * valid.
     */
    public MultivariateNormalDist propagateWithControlJacobian(
            Matrix controlJacobian) throws InvalidCovarianceMatrixException, 
            IllegalArgumentException {
        return getCalibrationData().propagateWithControlJacobian(
                controlJacobian);
    }
    
    /**
     * Propagates calibrated control signal covariance using current control
     * jacobian matrix.
     * The propagated distribution can be used during prediction stage in Kalman
     * filtering.
     * @param controlJacobian current control jacobian matrix.
     * @param result instance where propagated distribution is stored.
     * @throws InvalidCovarianceMatrixException if estimated covariance is not
     * valid.
     */
    public void propagateWithControlJacobian(final Matrix controlJacobian, 
            MultivariateNormalDist result) 
            throws InvalidCovarianceMatrixException, IllegalArgumentException {
        getCalibrationData().propagateWithControlJacobian(controlJacobian, 
                result);
    }    
    
    /**
     * Notifies that a full sample has been received and resets flags indicating
     * whether partial samples have been received.
     */
    protected void notifyFullSampleAndResetSampleReceive() {
        if(isFullSampleAvailable()) {
            processFullSample();     
            mAccumulatedAccelerometerSamples = mAccumulatedGyroscopeSamples = 0;
        }
    }
    
    /**
     * Obtains the number of state parameters in associated SLAM estimator.
     * @return number of state parameters.
     */
    protected abstract int getEstimatorStateLength();
    
    /**
     * Method to be implemented in subclasses to process a full sample.
     */
    protected abstract void processFullSample();
    
    /**
     * Updates internal mean and covariance values and checkes whether 
     * convergence has been reached and calibrator has finished or failed.
     */
    protected void updateSample() {
        if(mFinished) return;
        
        try {
            mEstimator.update(mSample);
        } catch (SignalProcessingException e) {
            mFailed = mFinished = true;
            
            if(mListener != null) {
                mListener.onCalibratorFinished(this, mConverged, true);
            }
            return;
        }
        
        mSampleCount++;
        
        double[] mean = mEstimator.getSampleAverage();
        Matrix cov = mEstimator.getMeasurementNoiseCov();
        
        //check if minimum number of samples has been reached
        if(mSampleCount >= mMaxNumSamples) {
            mFinished = true;
            
            if(mListener != null) {
                mListener.onCalibratorFinished(this, mConverged, mFailed);
            }            
            return;
        }
        
        //check if estimator has converged
        if(mSampleCount >= mMinNumSamples) {
            ArrayUtils.subtract(mean, mPreviousMean, mMeanDiff);
            try {
                cov.subtract(mPreviousCovariance, mCovDiff);
            } catch (WrongSizeException ignore) { /* never thrown */ }
            double meanDiffNorm = com.irurueta.algebra.Utils.normF(mMeanDiff);
            double covDiffNorm = com.irurueta.algebra.Utils.normF(mCovDiff);
            if(meanDiffNorm <= mConvergenceThreshold && 
                    covDiffNorm <= mConvergenceThreshold) {
                mConverged = mFinished = true;
                
                if(mListener != null) {
                    mListener.onCalibratorFinished(this, true, mFailed);
                }
                return;
            }
        }
        
        //copy current value for next iteration
        System.arraycopy(mean, 0, mPreviousMean, 0, mean.length);
        cov.copyTo(mPreviousCovariance);
               
        mFinished = false;
    }
    
    /**
     * Listener for implementations of this class.
     */
    public interface BaseSlamCalibratorListener {
        /**
         * Called when a full sample (accelerometer + gyroscope, etc) has been
         * received.
         * @param calibrator SLAM calibrator.
         */
        void onFullSampleReceived(BaseSlamCalibrator calibrator);
        
        /**
         * Called when a full sample (accelerometer + gyroscope, etc) has been
         * received and has already been processed.
         * @param calibrator SLAM calibrator.
         */
        void onFullSampleProcessed(BaseSlamCalibrator calibrator);
        
        /**
         * Called when calibration finishes.
         * @param calibrator SLAM calibrator.
         * @param converged true if calibration converged, false otherwise.
         * @param failed true if calibration failed, false otherwise.
         */
        void onCalibratorFinished(BaseSlamCalibrator calibrator, 
                boolean converged, boolean failed);
    }    
}
