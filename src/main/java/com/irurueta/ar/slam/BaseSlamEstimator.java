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
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.statistics.MultivariateNormalDist;

import java.io.Serializable;

/**
 * Base class to estimate position, velocity, acceleration and orientation of
 * a device using sensor data such as accelerometers and gyroscopes.
 * Implementations of this class are designed taking into account sensors 
 * available on Android devices.
 * @param <D> calibrator type associated to implementations of SLAM calibration 
 * data.
 */
@SuppressWarnings("WeakerAccess")
public abstract class BaseSlamEstimator<D extends BaseCalibrationData> 
        implements Serializable {
    
    /**
     * Number of components in 3D.
     */
    protected static final int N_COMPONENTS_3D = 3;
    
    /**
     * Conversion of nanoseconds to milliseconds.
     */
    protected static final double NANOS_TO_SECONDS = 1e-9;
    
    /**
     * Indicates whether sample accumulation must be enabled or not.
     */
    protected static final boolean DEFAULT_ENABLE_SAMPLE_ACCUMULATION = true;
    
    /**
     * Current position of the device along x-axis expressed in meters (m).
     */
    protected double mStatePositionX;
    
    /**
     * Current position of the device along y-axis expressed in meters (m).
     */
    protected double mStatePositionY;
    
    /**
     * Current position of the device along z-axis expressed in meters (m).
     */
    protected double mStatePositionZ;
    
    /**
     * Current linear velocity of the device along x-axis expressed in meters
     * per second (m/s).
     */
    protected double mStateVelocityX;
    
    /**
     * Current linear velocity of the device along y-axis expressed in meters
     * per second (m/s).
     */
    protected double mStateVelocityY;
    
    /**
     * Current linear velocity of the device along z-axis expressed in meters
     * per second (m/s).
     */
    protected double mStateVelocityZ;
    
    /**
     * Current linear acceleration of the device along x-axis expressed in 
     * meters per squared second (m/s^2).
     */
    protected double mStateAccelerationX;
    
    /**
     * Current linear acceleration of the device along y-axis expressed in 
     * meters per squared second (m/s^2).
     */
    protected double mStateAccelerationY;
    
    /**
     * Current linear acceleration of the device along z-axis expressed in 
     * meters per squared second (m/s^2).
     */
    protected double mStateAccelerationZ;
    
    /**
     * A value of quaternion containing current device orientation.
     */
    protected double mStateQuaternionA;
    
    /**
     * B value of quaternion containing current device orientation.
     */
    protected double mStateQuaternionB;
    
    /**
     * C value of quaternion containing current device orientation.
     */
    protected double mStateQuaternionC;
    
    /**
     * D value of quaternion containing current device orientation.
     */
    protected double mStateQuaternionD;
    
    /**
     * Angular speed of rotation of the device along x-axis expressed in radians 
     * per second (rad/s).
     */
    protected double mStateAngularSpeedX;
    
    /**
     * Angular speed of rotation of the device along y-axis expressed in radians
     * per second (rad/s).
     */
    protected double mStateAngularSpeedY;
    
    /**
     * Angular speed of rotation of the device along z-axis expressed in radians
     * per second (rad/s).
     */
    protected double mStateAngularSpeedZ;
    
    /**
     * Indicates whether an error occurred during the estimation.
     * If an error occurs the estimator should be restarted since state values
     * might become unreliable.
     */
    protected boolean mError;
    
    /**
     * Indicates whether accumulation of samples is enabled or not.
     */
    protected boolean mAccumulationEnabled = 
            DEFAULT_ENABLE_SAMPLE_ACCUMULATION;
    
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
     * Expressed in radians per second (rad/s).
     */
    protected double mAccumulatedAngularSpeedSampleX;
    
    /**
     * Average of angular speed along y-axis accumulated since last full sample.
     * Expressed in radians per seoncd (rad/s).
     */
    protected double mAccumulatedAngularSpeedSampleY;
    
    /**
     * Acerage of angular speed along z-axis accumulated since last full sample.
     * Expressed in radians per second (red/s).
     */
    protected double mAccumulatedAngularSpeedSampleZ;
    
    /**
     * Listener in charge of handling events raised by instances of this class.
     */
    protected transient BaseSlamEstimatorListener mListener;
    
    /**
     * Calibration data. When provided, its mean and covariance are used
     * to correct control samples and adjust process covariance matrix during
     * Kalman filtering in prediction stage.
     */
    protected D mCalibrationData;
    
    /**
     * Multivariate distribution to be reused during propagation of calibrated
     * covariance.
     */
    protected MultivariateNormalDist mNormalDist;
    
    /**
     * Constructor.
     */
    public BaseSlamEstimator() {
        reset();
    }

    /**
     * Resets position and timestamp to zero while keeping other state parameters.
     */
    public final void resetPosition() {
        reset(0.0, 0.0, 0.0, mStateVelocityX, mStateVelocityY, mStateVelocityZ,
                mStateAccelerationX, mStateAccelerationY, mStateAccelerationZ,
                mStateQuaternionA, mStateQuaternionB, mStateQuaternionC, mStateQuaternionD,
                mStateAngularSpeedX, mStateAngularSpeedY, mStateAngularSpeedZ);
    }

    /**
     * Resets linear velocity and timestamp to zero while keeping other state parameters.
     */
    public final void resetVelocity() {
        reset(mStatePositionX, mStatePositionY, mStatePositionZ, 0.0, 0.0, 0.0,
                mStateAccelerationX, mStateAccelerationY, mStateAccelerationZ,
                mStateQuaternionA, mStateQuaternionB, mStateQuaternionC, mStateQuaternionD,
                mStateAngularSpeedX, mStateAngularSpeedY, mStateAngularSpeedZ);
    }

    /**
     * Resets position, linear velocity and timestamp to zero while keeping other state parameters.
     */
    public final void resetPositionAndVelocity() {
        reset(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                mStateAccelerationX, mStateAccelerationY, mStateAccelerationZ,
                mStateQuaternionA, mStateQuaternionB, mStateQuaternionC, mStateQuaternionD,
                mStateAngularSpeedX, mStateAngularSpeedY, mStateAngularSpeedZ);
    }

    /**
     * Resets acceleration and timestamp to zero while keeping other state parameters.
     */
    public final void resetAcceleration() {
        reset(mStatePositionX, mStatePositionY, mStatePositionZ,
                mStateVelocityX, mStateVelocityY, mStateVelocityZ, 0.0, 0.0, 0.0,
                mStateQuaternionA, mStateQuaternionB, mStateQuaternionC, mStateQuaternionD,
                mStateAngularSpeedX, mStateAngularSpeedY, mStateAngularSpeedZ);
    }

    /**
     * Resets orientation and timestamp to zero while keeping other state parameters.
     */
    public final void resetOrientation() {
        reset(mStatePositionX, mStatePositionY, mStatePositionZ,
                mStateVelocityX, mStateVelocityY, mStateVelocityZ,
                mStateAccelerationX, mStateAccelerationY, mStateAccelerationZ,
                1.0, 0.0, 0.0, 0.0, mStateAngularSpeedX, mStateAngularSpeedY, mStateAngularSpeedZ);
    }

    /**
     * Resets angular speed and timestamp to zero while keeping other state parameters.
     */
    public final void resetAngularSpeed() {
        reset(mStatePositionX, mStatePositionY, mStatePositionZ,
                mStateVelocityX, mStateVelocityY, mStateVelocityZ,
                mStateAccelerationX, mStateAccelerationY, mStateAccelerationZ,
                mStateQuaternionA, mStateQuaternionB, mStateQuaternionC, mStateQuaternionD,
                0.0, 0.0, 0.0);
    }

    /**
     * Resets position, linear velocity, linear acceleration, orientation and
     * angular speed of the device to zero.
     */
    public final void reset() {
        //NOTE: initial orientation is expressed as quaternion 
        //(1.0, 0.0, 0.0, 0.0) which is equivalent to no rotation.
        reset(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0);
    }

    /**
     * Obtains current x-position of the device expressed in meters (m).
     * @return x-position of the device expressed in meters (m).
     */
    public double getStatePositionX() {
        return mStatePositionX;
    }
    
    /**
     * Obtains current y-position of the device expressed in meters (m).
     * @return y-position of the device expressed in meters (m).
     */
    public double getStatePositionY() {
        return mStatePositionY;
    }
    
    /**
     * Obtains current z-position of the device expressed in meters (m).
     * @return z-position of the device expressed in meters (m).
     */
    public double getStatePositionZ() {
        return mStatePositionZ;
    }
    
    /**
     * Gets x,y,z coordinates of the device position expressed in meters (m).
     * @return position of the device.
     */
    public double[] getStatePosition() {
        return new double[]{mStatePositionX, mStatePositionY, mStatePositionZ};
    }
    
    /**
     * Gets x,y,z coordinates of the device position expressed in meters (m) and
     * stores the result into provided array.
     * @param result array where position coordinates will be stored.
     * @throws IllegalArgumentException if the array does not have length 3.
     */
    public void getStatePosition(double[] result) 
            throws IllegalArgumentException {
        if (result.length != N_COMPONENTS_3D) {
            throw new IllegalArgumentException("result must have length 3");
        }
        result[0] = mStatePositionX;
        result[1] = mStatePositionY;
        result[2] = mStatePositionZ;        
    }
    
    /**
     * Gets current linear velocity of the device along x-axis expressed in
     * meters per second (m/s).
     * @return current velocity along x-axis expressed in meters per second 
     * (m/s).
     */
    public double getStateVelocityX() {
        return mStateVelocityX;
    }
    
    /**
     * Gets current linear velocity of the device along y-axis expressed in 
     * meters per second (m/s).
     * @return current velocity along y-axis expressed in meters per second 
     * (m/s).
     */
    public double getStateVelocityY() {
        return mStateVelocityY;
    }
    
    /**
     * Gets current linear velocity of the device along z-axis expressed in
     * meters per second (m/s).
     * @return current velocity along z-axis expressed in meters per second
     * (m/s).
     */
    public double getStateVelocityZ() {
        return mStateVelocityZ;
    }
    
    /**
     * Gets x,y,z coordinates of current linear velocity of the device expressed
     * in meters per second (m/s).
     * @return current linear velocity of the device.
     */
    public double[] getStateVelocity() {
        return new double[]{mStateVelocityX, mStateVelocityY, mStateVelocityZ};
    }
    
    /**
     * Gets x,y,z coordinates of current linear velocity of the device expressed
     * in meters per second (m/s).
     * @param result array where linear velocity of the device will be stored.
     * @throws IllegalArgumentException if result array does not have length 3.
     */
    public void getStateVelocity(double[] result) 
            throws IllegalArgumentException {
        if (result.length != N_COMPONENTS_3D) {
            throw new IllegalArgumentException("result must have length 3");
        }
        result[0] = mStateVelocityX;
        result[1] = mStateVelocityY;
        result[2] = mStateVelocityZ;        
    }
    
    /**
     * Gets current linear acceleration of the device along x-axis expressed in
     * meters per squared second (m/s^2).
     * @return linear acceleration of the device along x-axis.
     */
    public double getStateAccelerationX() {
        return mStateAccelerationX;
    }
    
    /**
     * Gets current linear acceleration of the device along y-axis expressed in
     * meters per squared second (m/s^2).
     * @return linear acceleration of the device along y-axis.
     */
    public double getStateAccelerationY() {
        return mStateAccelerationY;
    }
    
    /**
     * Gets current linear acceleration of the device along z-axis expressed in
     * meters per squared second (m/s^2).
     * @return linear acceleration of the device along z-axis.
     */
    public double getStateAccelerationZ() {
        return mStateAccelerationZ;
    }
    
    /**
     * Gets current x,y,z linear acceleration coordinates of the device 
     * expressed in meters per squared second (m/s^2).
     * @return current linear acceleration of the device.
     */
    public double[] getStateAcceleration() {
        return new double[]{mStateAccelerationX, mStateAccelerationY, 
            mStateAccelerationZ};
    }
    
    /**
     * Gets current x,y,z linear acceleration coordinates of the device 
     * expressed in meters per squared second (m/s^2).
     * @param result array where resulting linear acceleration coordinates will 
     * be stored.
     * @throws IllegalArgumentException if array does not have length 3.
     */
    public void getStateAcceleration(double[] result) 
            throws IllegalArgumentException {
        if (result.length != N_COMPONENTS_3D) {
            throw new IllegalArgumentException("result must have length 3");
        }
        result[0] = mStateAccelerationX;
        result[1] = mStateAccelerationY;
        result[2] = mStateAccelerationZ;        
    }
    
    /**
     * Gets A value of quaternion containing current device orientation.
     * @return A value of quaternion containing current device orientation.
     */
    public double getStateQuaternionA() {
        return mStateQuaternionA;
    }
    
    /**
     * Gets B value of quaternion containing current device orientation.
     * @return B value of quaternion containing current device orientation.
     */
    public double getStateQuaternionB() {
        return mStateQuaternionB;
    }
    
    /**
     * Gets C value of quaternion containing current device orientation.
     * @return C value of quaternion containing current device orientation.
     */
    public double getStateQuaternionC() {
        return mStateQuaternionC;
    }
    
    /**
     * Gets D value of quaternion containing current device orientation.
     * @return D value of quaternion containing current device orientation.
     */
    public double getStateQuaternionD() {
        return mStateQuaternionD;
    }
    
    /**
     * Gets A, B, C, D values of quaternion containing current device 
     * orientation.
     * @return A, B, C, D values of quaternion containing current device 
     * orientation.
     */
    public double[] getStateQuaternionArray() {
        return new double[]{ mStateQuaternionA, mStateQuaternionB, 
            mStateQuaternionC, mStateQuaternionD };
    }
    
    /**
     * Gets A, B, C, D values of quaternion containing current device 
     * orientation.
     * @param result array where A, B, C, D values of quaternion will be stored.
     * Must have length 4.
     * @throws IllegalArgumentException if provided array does not have length 
     * 4.
     */
    public void getStateQuaternionArray(double[] result) 
            throws IllegalArgumentException {
        if (result.length != Quaternion.N_PARAMS) {
            throw new IllegalArgumentException("result must have length 4");
        }
        result[0] = mStateQuaternionA;
        result[1] = mStateQuaternionB;
        result[2] = mStateQuaternionC;
        result[3] = mStateQuaternionD;        
    }
    
    /**
     * Gets quaternion containing current device orientation.
     * @return quaternion containing current device orientation.
     */
    public Quaternion getStateQuaternion() {
        return new Quaternion(mStateQuaternionA, mStateQuaternionB, 
                mStateQuaternionC, mStateQuaternionD);
    }
    
    /**
     * Gets quaternion containing current device orientation.
     * @param result instance where quaternion data will be stored.
     */
    public void getStateQuaternion(Quaternion result) {
        result.setA(mStateQuaternionA);
        result.setB(mStateQuaternionB);
        result.setC(mStateQuaternionC);
        result.setD(mStateQuaternionD);        
    }
    
    /**
     * Gets angular speed along x-axis expressed in radians per second (rad/s).
     * @return angular speed along x-axis.
     */
    public double getStateAngularSpeedX() {
        return mStateAngularSpeedX;
    }
    
    /**
     * Gets angular speed along y-axis expressed in radians per second (rad/s).
     * @return angular speed along y-axis.
     */
    public double getStateAngularSpeedY() {
        return mStateAngularSpeedY;
    }
    
    /**
     * Gets angular speed along z-axis expressed in radians per second (rad/s).
     * @return angular speed along z-axis.
     */
    public double getStateAngularSpeedZ() {
        return mStateAngularSpeedZ;
    }
    
    /**
     * Gets angular speed of the device along x,y,z axes expressed in radians
     * per second (rad/s).
     * @return device's angular speed.
     */
    public double[] getStateAngularSpeed() {
        return new double[]{ mStateAngularSpeedX, mStateAngularSpeedY, 
                mStateAngularSpeedZ };
    }
    
    /**
     * Gets angular speed of the device along x,y,z axes expressed in radians
     * per second (rad/s) and stores the result into provided array.
     * @param result array where angular speed will be stored. Must have length 
     * 3.
     * @throws IllegalArgumentException if provided array does not have length 
     * 3.
     */
    public void getStateAngularSpeed(double[] result) 
            throws IllegalArgumentException {
        if(result.length != N_COMPONENTS_3D) {
            throw new IllegalArgumentException("result must have length 3");
        }
        result[0] = mStateAngularSpeedX;
        result[1] = mStateAngularSpeedY;
        result[2] = mStateAngularSpeedZ;
    }

    /**
     * Gets covariance matrix of state variables (position, velocity, acceleration, orientation and angular speed).
     * Actual meaning of elements in returned matrix will depend on actual implementation of the estimator.
     * @return covariance matrix of state variables.
     */
    public abstract Matrix getStateCovariance();

    /**
     * Indicates whether an error occurred during the estimation.
     * If an error occurs the estimator should be restarted since state values
     * might become unreliable.
     * @return true if an error occurred since last start time, false otherwise.
     */
    public boolean hasError() {
        return mError;
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
     * full sample (acceleromter + gyroscope).
     * @return  true if gyroscope sample has been received, false otherwise.
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
     * Gets average acceleration along z-axis accumulated since last full 
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
     * Gets average acceleration along x,y,z axes accumulated since last full
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
     * second.
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
        return new double[] {
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
        if(result.length != N_COMPONENTS_3D) {
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
     * expressed in nanoseconds.
     * @param accelerationX linear acceleration along x-axis expressed in meters
     * per squared second (m/s^2).
     * @param accelerationY linear acceleration along y-axis expressed in meters
     * per squared second (m/s^2).
     * @param accelerationZ linear acceleration along z-axis expressed in meters
     * per squared second (m/s^2).
     */
    public void updateAccelerometerSample(long timestamp, float accelerationX,
            float accelerationY, float accelerationZ) {
        if (!isFullSampleAvailable()) {
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
     * @param timestamp timestamp of gyroscope sample since epoch time and
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
                mAccumulatedAngularSpeedSampleX = (mAccumulatedAngularSpeedSampleX * mAccumulatedGyroscopeSamples +
                        angularSpeedX) / nextSamples;
                mAccumulatedAngularSpeedSampleY = (mAccumulatedAngularSpeedSampleY * mAccumulatedGyroscopeSamples +
                        angularSpeedY) / nextSamples;
                mAccumulatedAngularSpeedSampleZ = (mAccumulatedAngularSpeedSampleZ * mAccumulatedGyroscopeSamples +
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
     * Corrects system state with provided position measure having an accuracy
     * determined by provided covariance matrix.
     * @param positionX new position along x axis expressed in meters (m).
     * @param positionY new position along y axis expressed in meters (m).
     * @param positionZ new position along z axis expressed in meters (m).
     * @param positionCovariance new position covariance matrix determining
     * new position accuracy or null if last available covariance does not need
     * to be updated.
     * @throws IllegalArgumentException if provided covariance matrix is not 
     * 3x3.
     */
    public void correctWithPositionMeasure(double positionX, double positionY, 
            double positionZ, Matrix positionCovariance) 
            throws IllegalArgumentException {        
        setPositionCovarianceMatrix(positionCovariance);
        correctWithPositionMeasure(positionX, positionY, positionZ);
    }
    
    /**
     * Corrects system state with provided position measure having an accuracy
     * determined by provided covariance matrix.
     * @param position x,y,z coordinates of position expressed in meters (m). 
     * Must have length 3.
     * @param positionCovariance new position covariance matrix determining new
     * position accuracy or null if last available covariance does not need to
     * be udpated.
     * @throws IllegalArgumentException if provided covariance matrix is not 
     * 3x3 or if provided position array does not have length 3.
     */
    public void correctWithPositionMeasure(double[] position, 
            Matrix positionCovariance) throws IllegalArgumentException {
        if(position.length != N_COMPONENTS_3D) {
            throw new IllegalArgumentException("position must have length 3");
        }
        correctWithPositionMeasure(position[0], position[1], position[2], 
                positionCovariance);
    }
    
    /**
     * Corrects system state with provided position measure having an accuracy
     * determined by provided covariance matrix.
     * @param position position expressed in meters (m).
     * @param positionCovariance new position covariance matrix determining new
     * position accuracy or null if last available covariance does not need to
     * be updated.
     * @throws IllegalArgumentException if provided covariance matrix is not 
     * 3x3.
     */
    public void correctWithPositionMeasure(Point3D position, 
            Matrix positionCovariance) throws IllegalArgumentException {
        correctWithPositionMeasure(position.getInhomX(), position.getInhomY(),
                position.getInhomZ(), positionCovariance);
    }
    
    /**
     * Updates covariance matrix of position measures.
     * If null is provided, covariance matrix is not updated.
     * @param positionCovariance new position covariance determining position
     * accuracy or null if last available covariance does not need to be 
     * updated.
     * @throws IllegalArgumentException if provided covariance matrix is not 
     * 3x3.
     */
    public abstract void setPositionCovarianceMatrix(Matrix positionCovariance)
            throws IllegalArgumentException;    
        
    /**
     * Gets current covariance matrix of position measures determining current
     * accuracy of provided position measures.
     * @return covariance matrix of position measures.
     */
    public abstract Matrix getPositionCovarianceMatrix();
    
    /**
     * Corrects system state with provided position measure using current
     * position accuracy.
     * @param positionX new position along x axis expressed in meters (m).
     * @param positionY new position along y axis expressed in meters (m).
     * @param positionZ new position along z axis expressed in meters (m).
     */
    public abstract void correctWithPositionMeasure(double positionX, 
            double positionY, double positionZ);
    
    /**
     * Corrects system state with provided position measure using current 
     * position accuracy.
     * @param position x,y,z coordinates of position expressed in meters (m).
     * Must have length 3.
     * @throws IllegalArgumentException if provided array does not have length 
     * 3.
     */
    public void correctWithPositionMeasure(double[] position) 
            throws IllegalArgumentException {
        correctWithPositionMeasure(position, null);
    }
    
    /**
     * Corrects system state with provided position measure using current
     * position accuracy.
     * @param position position expressed in meters (m).
     */
    public void correctWithPositionMeasure(Point3D position) {
        correctWithPositionMeasure(position, null);
    }   
    
    /**
     * Gets listener in charge of handling events raised by instances of this 
     * class.
     * @return listener in charge of handling events raised by instances of this
     * class.
     */
    public BaseSlamEstimatorListener getListener() {
        return mListener;
    }
    
    /**
     * Sets listener in charge of handling events raised by instances of this
     * class.
     * @param listener listener in charge of handling events raised by instances
     * of this class.
     */
    public void setListener(BaseSlamEstimatorListener listener) {
        mListener = listener;
    }    
        
    /**
     * Gets calibration data. When provided, its mean and covariance are used
     * to correct control samples and adjust process covariance matrix during
     * Kalman filtering in prediction stage.
     * @return calibration data.
     */
    public D getCalibrationData() {
        return mCalibrationData;
    }
    
    /**
     * Sets calibration data. When provided, its mean and covariance are used
     * to correct control samples and adjust process covariance matrix during
     * Kalman filtering in prediction stage.
     * @param calibrationData calibration data.
     */
    public void setCalibrationData(D calibrationData) {
        mCalibrationData = calibrationData;
    }

    /**
     * Resets position, linear velocity, linear acceleration, orientation and
     * angular speed to provided values.
     * @param statePositionX position along x-axis expressed in meters (m).
     * @param statePositionY position along y-axis expressed in meters (m).
     * @param statePositionZ position along z-axis expressed in meters (m).
     * @param stateVelocityX linear velocity along x-axis expressed in meters
     * per second (m/s).
     * @param stateVelocityY linear velocity along y-axis expressed in meters
     * per second (m/s).
     * @param stateVelocityZ linear velocity along z-axis expressed in meters
     * per second (m/s).
     * @param stateAccelerationX linear acceleration along x-axis expressed in
     * meters per squared second (m/s^2).
     * @param stateAccelerationY linear acceleration along y-axis expressed in
     * meters per squared second (m/s^2).
     * @param stateAccelerationZ linear acceleration along z-axis expressed in
     * meters per squared second (m/s^2).
     * @param stateQuaternionA A value of orientation quaternion.
     * @param stateQuaternionB B value of orientation quaternion.
     * @param stateQuaternionC C value of orientation quaternion.
     * @param stateQuaternionD D value of orientation quaternion.
     * @param stateAngularSpeedX angular speed along x-axis expressed in radians
     * per second (rad/s).
     * @param stateAngularSpeedY angular speed along y-axis expressed in radians
     * per second (rad/s).
     * @param stateAngularSpeedZ angular speed along z-axis expressed in radians
     * per second (rad/s).
     */
    protected void reset(double statePositionX, double statePositionY,
                         double statePositionZ, double stateVelocityX, double stateVelocityY,
                         double stateVelocityZ, double stateAccelerationX,
                         double stateAccelerationY, double stateAccelerationZ,
                         double stateQuaternionA, double stateQuaternionB,
                         double stateQuaternionC, double stateQuaternionD,
                         double stateAngularSpeedX, double stateAngularSpeedY,
                         double stateAngularSpeedZ) {
        mStatePositionX = statePositionX;
        mStatePositionY = statePositionY;
        mStatePositionZ = statePositionZ;
        mStateVelocityX = stateVelocityX;
        mStateVelocityY = stateVelocityY;
        mStateVelocityZ = stateVelocityZ;
        mStateAccelerationX = stateAccelerationX;
        mStateAccelerationY = stateAccelerationY;
        mStateAccelerationZ = stateAccelerationZ;
        mStateQuaternionA = stateQuaternionA;
        mStateQuaternionB = stateQuaternionB;
        mStateQuaternionC = stateQuaternionC;
        mStateQuaternionD = stateQuaternionD;
        mStateAngularSpeedX = stateAngularSpeedX;
        mStateAngularSpeedY = stateAngularSpeedY;
        mStateAngularSpeedZ = stateAngularSpeedZ;
        mAccelerometerTimestampNanos = mGyroscopeTimestampNanos = -1;
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
     * Method to be implemented in subclasses to process a full sample.
     */
    protected abstract void processFullSample();
    
    /**
     * Listener for implementations of this class..
     */
    public interface BaseSlamEstimatorListener {
        /**
         * Called when a full sample (accelerometer + gyroscope, etc) has been
         * received and is about to be processed to update internal state.
         * @param estimator SLAM estimator.
         */
        void onFullSampleReceived(BaseSlamEstimator estimator);
        
        /**
         * Callen when a full sample (accelerometer + gyroscope, etc) has been
         * received and has already been processed, and hence internal state has
         * also been updated.
         * @param estimator SLAM estimator.
         */
        void onFullSampleProcessed(BaseSlamEstimator estimator);
        
        /**
         * Called when internal state is about to be corrected by using an 
         * external measure.
         * @param estimator SLAM estimator.
         */
        void onCorrectWithPositionMeasure(BaseSlamEstimator estimator);        
        
        /**
         * Called after internal state has been corrected using an external 
         * measure.
         * @param estimator SLAM estimator.
         */
        void onCorrectedWithPositionMeasure(BaseSlamEstimator estimator);
    }
}
