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
 *
 * @param <D> calibrator type associated to implementations of SLAM calibration
 *            data.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class BaseSlamEstimator<D extends BaseCalibrationData> implements Serializable {

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
    protected double statePositionX;

    /**
     * Current position of the device along y-axis expressed in meters (m).
     */
    protected double statePositionY;

    /**
     * Current position of the device along z-axis expressed in meters (m).
     */
    protected double statePositionZ;

    /**
     * Current linear velocity of the device along x-axis expressed in meters
     * per second (m/s).
     */
    protected double stateVelocityX;

    /**
     * Current linear velocity of the device along y-axis expressed in meters
     * per second (m/s).
     */
    protected double stateVelocityY;

    /**
     * Current linear velocity of the device along z-axis expressed in meters
     * per second (m/s).
     */
    protected double stateVelocityZ;

    /**
     * Current linear acceleration of the device along x-axis expressed in
     * meters per squared second (m/s^2).
     */
    protected double stateAccelerationX;

    /**
     * Current linear acceleration of the device along y-axis expressed in
     * meters per squared second (m/s^2).
     */
    protected double stateAccelerationY;

    /**
     * Current linear acceleration of the device along z-axis expressed in
     * meters per squared second (m/s^2).
     */
    protected double stateAccelerationZ;

    /**
     * A value of quaternion containing current device orientation.
     */
    protected double stateQuaternionA;

    /**
     * B value of quaternion containing current device orientation.
     */
    protected double stateQuaternionB;

    /**
     * C value of quaternion containing current device orientation.
     */
    protected double stateQuaternionC;

    /**
     * D value of quaternion containing current device orientation.
     */
    protected double stateQuaternionD;

    /**
     * Angular speed of rotation of the device along x-axis expressed in radians
     * per second (rad/s).
     */
    protected double stateAngularSpeedX;

    /**
     * Angular speed of rotation of the device along y-axis expressed in radians
     * per second (rad/s).
     */
    protected double stateAngularSpeedY;

    /**
     * Angular speed of rotation of the device along z-axis expressed in radians
     * per second (rad/s).
     */
    protected double stateAngularSpeedZ;

    /**
     * Indicates whether an error occurred during the estimation.
     * If an error occurs the estimator should be restarted since state values
     * might become unreliable.
     */
    protected boolean error;

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
     * Expressed in radians per second (rad/s).
     */
    protected double accumulatedAngularSpeedSampleX;

    /**
     * Average of angular speed along y-axis accumulated since last full sample.
     * Expressed in radians per second (rad/s).
     */
    protected double accumulatedAngularSpeedSampleY;

    /**
     * Average of angular speed along z-axis accumulated since last full sample.
     * Expressed in radians per second (red/s).
     */
    protected double accumulatedAngularSpeedSampleZ;

    /**
     * Listener in charge of handling events raised by instances of this class.
     */
    protected transient BaseSlamEstimatorListener<D> listener;

    /**
     * Calibration data. When provided, its mean and covariance are used
     * to correct control samples and adjust process covariance matrix during
     * Kalman filtering in prediction stage.
     */
    protected D calibrationData;

    /**
     * Multivariate distribution to be reused during propagation of calibrated
     * covariance.
     */
    protected transient MultivariateNormalDist normalDist;

    /**
     * Constructor.
     */
    protected BaseSlamEstimator() {
        reset();
    }

    /**
     * Resets position and timestamp to zero while keeping other state parameters.
     */
    public final void resetPosition() {
        reset(0.0, 0.0, 0.0, stateVelocityX, stateVelocityY,
                stateVelocityZ, stateAccelerationX, stateAccelerationY, stateAccelerationZ,
                stateQuaternionA, stateQuaternionB, stateQuaternionC, stateQuaternionD,
                stateAngularSpeedX, stateAngularSpeedY, stateAngularSpeedZ);
    }

    /**
     * Resets linear velocity and timestamp to zero while keeping other state parameters.
     */
    public final void resetVelocity() {
        reset(statePositionX, statePositionY, statePositionZ, 0.0, 0.0,
                0.0, stateAccelerationX, stateAccelerationY, stateAccelerationZ,
                stateQuaternionA, stateQuaternionB, stateQuaternionC, stateQuaternionD,
                stateAngularSpeedX, stateAngularSpeedY, stateAngularSpeedZ);
    }

    /**
     * Resets position, linear velocity and timestamp to zero while keeping other state parameters.
     */
    public final void resetPositionAndVelocity() {
        reset(0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, stateAccelerationX, stateAccelerationY, stateAccelerationZ,
                stateQuaternionA, stateQuaternionB, stateQuaternionC, stateQuaternionD,
                stateAngularSpeedX, stateAngularSpeedY, stateAngularSpeedZ);
    }

    /**
     * Resets acceleration and timestamp to zero while keeping other state parameters.
     */
    public final void resetAcceleration() {
        reset(statePositionX, statePositionY, statePositionZ,
                stateVelocityX, stateVelocityY, stateVelocityZ, 0.0,
                0.0, 0.0, stateQuaternionA, stateQuaternionB,
                stateQuaternionC, stateQuaternionD, stateAngularSpeedX, stateAngularSpeedY,
                stateAngularSpeedZ);
    }

    /**
     * Resets orientation and timestamp to zero while keeping other state parameters.
     */
    public final void resetOrientation() {
        reset(statePositionX, statePositionY, statePositionZ,
                stateVelocityX, stateVelocityY, stateVelocityZ,
                stateAccelerationX, stateAccelerationY, stateAccelerationZ,
                1.0, 0.0, 0.0, 0.0,
                stateAngularSpeedX, stateAngularSpeedY, stateAngularSpeedZ);
    }

    /**
     * Resets angular speed and timestamp to zero while keeping other state parameters.
     */
    public final void resetAngularSpeed() {
        reset(statePositionX, statePositionY, statePositionZ,
                stateVelocityX, stateVelocityY, stateVelocityZ,
                stateAccelerationX, stateAccelerationY, stateAccelerationZ,
                stateQuaternionA, stateQuaternionB, stateQuaternionC, stateQuaternionD,
                0.0, 0.0, 0.0);
    }

    /**
     * Resets position, linear velocity, linear acceleration, orientation and
     * angular speed of the device to zero.
     */
    public final void reset() {
        // NOTE: initial orientation is expressed as quaternion
        // (1.0, 0.0, 0.0, 0.0) which is equivalent to no rotation.
        reset(0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0);
    }

    /**
     * Obtains current x-position of the device expressed in meters (m).
     *
     * @return x-position of the device expressed in meters (m).
     */
    public double getStatePositionX() {
        return statePositionX;
    }

    /**
     * Obtains current y-position of the device expressed in meters (m).
     *
     * @return y-position of the device expressed in meters (m).
     */
    public double getStatePositionY() {
        return statePositionY;
    }

    /**
     * Obtains current z-position of the device expressed in meters (m).
     *
     * @return z-position of the device expressed in meters (m).
     */
    public double getStatePositionZ() {
        return statePositionZ;
    }

    /**
     * Gets x,y,z coordinates of the device position expressed in meters (m).
     *
     * @return position of the device.
     */
    public double[] getStatePosition() {
        return new double[]{statePositionX, statePositionY, statePositionZ};
    }

    /**
     * Gets x,y,z coordinates of the device position expressed in meters (m) and
     * stores the result into provided array.
     *
     * @param result array where position coordinates will be stored.
     * @throws IllegalArgumentException if the array does not have length 3.
     */
    public void getStatePosition(final double[] result) {
        if (result.length != N_COMPONENTS_3D) {
            // result must have length 3
            throw new IllegalArgumentException();
        }
        result[0] = statePositionX;
        result[1] = statePositionY;
        result[2] = statePositionZ;
    }

    /**
     * Gets current linear velocity of the device along x-axis expressed in
     * meters per second (m/s).
     *
     * @return current velocity along x-axis expressed in meters per second
     * (m/s).
     */
    public double getStateVelocityX() {
        return stateVelocityX;
    }

    /**
     * Gets current linear velocity of the device along y-axis expressed in
     * meters per second (m/s).
     *
     * @return current velocity along y-axis expressed in meters per second
     * (m/s).
     */
    public double getStateVelocityY() {
        return stateVelocityY;
    }

    /**
     * Gets current linear velocity of the device along z-axis expressed in
     * meters per second (m/s).
     *
     * @return current velocity along z-axis expressed in meters per second
     * (m/s).
     */
    public double getStateVelocityZ() {
        return stateVelocityZ;
    }

    /**
     * Gets x,y,z coordinates of current linear velocity of the device expressed
     * in meters per second (m/s).
     *
     * @return current linear velocity of the device.
     */
    public double[] getStateVelocity() {
        return new double[]{stateVelocityX, stateVelocityY, stateVelocityZ};
    }

    /**
     * Gets x,y,z coordinates of current linear velocity of the device expressed
     * in meters per second (m/s).
     *
     * @param result array where linear velocity of the device will be stored.
     * @throws IllegalArgumentException if result array does not have length 3.
     */
    public void getStateVelocity(final double[] result) {
        if (result.length != N_COMPONENTS_3D) {
            // result must have length 3
            throw new IllegalArgumentException();
        }
        result[0] = stateVelocityX;
        result[1] = stateVelocityY;
        result[2] = stateVelocityZ;
    }

    /**
     * Gets current linear acceleration of the device along x-axis expressed in
     * meters per squared second (m/s^2).
     *
     * @return linear acceleration of the device along x-axis.
     */
    public double getStateAccelerationX() {
        return stateAccelerationX;
    }

    /**
     * Gets current linear acceleration of the device along y-axis expressed in
     * meters per squared second (m/s^2).
     *
     * @return linear acceleration of the device along y-axis.
     */
    public double getStateAccelerationY() {
        return stateAccelerationY;
    }

    /**
     * Gets current linear acceleration of the device along z-axis expressed in
     * meters per squared second (m/s^2).
     *
     * @return linear acceleration of the device along z-axis.
     */
    public double getStateAccelerationZ() {
        return stateAccelerationZ;
    }

    /**
     * Gets current x,y,z linear acceleration coordinates of the device
     * expressed in meters per squared second (m/s^2).
     *
     * @return current linear acceleration of the device.
     */
    public double[] getStateAcceleration() {
        return new double[]{stateAccelerationX, stateAccelerationY, stateAccelerationZ};
    }

    /**
     * Gets current x,y,z linear acceleration coordinates of the device
     * expressed in meters per squared second (m/s^2).
     *
     * @param result array where resulting linear acceleration coordinates will
     *               be stored.
     * @throws IllegalArgumentException if array does not have length 3.
     */
    public void getStateAcceleration(final double[] result) {
        if (result.length != N_COMPONENTS_3D) {
            // result must have length 3
            throw new IllegalArgumentException();
        }
        result[0] = stateAccelerationX;
        result[1] = stateAccelerationY;
        result[2] = stateAccelerationZ;
    }

    /**
     * Gets A value of quaternion containing current device orientation.
     *
     * @return A value of quaternion containing current device orientation.
     */
    public double getStateQuaternionA() {
        return stateQuaternionA;
    }

    /**
     * Gets B value of quaternion containing current device orientation.
     *
     * @return B value of quaternion containing current device orientation.
     */
    public double getStateQuaternionB() {
        return stateQuaternionB;
    }

    /**
     * Gets C value of quaternion containing current device orientation.
     *
     * @return C value of quaternion containing current device orientation.
     */
    public double getStateQuaternionC() {
        return stateQuaternionC;
    }

    /**
     * Gets D value of quaternion containing current device orientation.
     *
     * @return D value of quaternion containing current device orientation.
     */
    public double getStateQuaternionD() {
        return stateQuaternionD;
    }

    /**
     * Gets A, B, C, D values of quaternion containing current device
     * orientation.
     *
     * @return A, B, C, D values of quaternion containing current device
     * orientation.
     */
    public double[] getStateQuaternionArray() {
        return new double[]{stateQuaternionA, stateQuaternionB, stateQuaternionC, stateQuaternionD};
    }

    /**
     * Gets A, B, C, D values of quaternion containing current device
     * orientation.
     *
     * @param result array where A, B, C, D values of quaternion will be stored.
     *               Must have length 4.
     * @throws IllegalArgumentException if provided array does not have length
     *                                  4.
     */
    public void getStateQuaternionArray(final double[] result) {
        if (result.length != Quaternion.N_PARAMS) {
            throw new IllegalArgumentException("result must have length 4");
        }
        result[0] = stateQuaternionA;
        result[1] = stateQuaternionB;
        result[2] = stateQuaternionC;
        result[3] = stateQuaternionD;
    }

    /**
     * Gets quaternion containing current device orientation.
     *
     * @return quaternion containing current device orientation.
     */
    public Quaternion getStateQuaternion() {
        return new Quaternion(stateQuaternionA, stateQuaternionB, stateQuaternionC, stateQuaternionD);
    }

    /**
     * Gets quaternion containing current device orientation.
     *
     * @param result instance where quaternion data will be stored.
     */
    public void getStateQuaternion(final Quaternion result) {
        result.setA(stateQuaternionA);
        result.setB(stateQuaternionB);
        result.setC(stateQuaternionC);
        result.setD(stateQuaternionD);
    }

    /**
     * Gets angular speed along x-axis expressed in radians per second (rad/s).
     *
     * @return angular speed along x-axis.
     */
    public double getStateAngularSpeedX() {
        return stateAngularSpeedX;
    }

    /**
     * Gets angular speed along y-axis expressed in radians per second (rad/s).
     *
     * @return angular speed along y-axis.
     */
    public double getStateAngularSpeedY() {
        return stateAngularSpeedY;
    }

    /**
     * Gets angular speed along z-axis expressed in radians per second (rad/s).
     *
     * @return angular speed along z-axis.
     */
    public double getStateAngularSpeedZ() {
        return stateAngularSpeedZ;
    }

    /**
     * Gets angular speed of the device along x,y,z axes expressed in radians
     * per second (rad/s).
     *
     * @return device's angular speed.
     */
    public double[] getStateAngularSpeed() {
        return new double[]{stateAngularSpeedX, stateAngularSpeedY, stateAngularSpeedZ};
    }

    /**
     * Gets angular speed of the device along x,y,z axes expressed in radians
     * per second (rad/s) and stores the result into provided array.
     *
     * @param result array where angular speed will be stored. Must have length
     *               3.
     * @throws IllegalArgumentException if provided array does not have length
     *                                  3.
     */
    public void getStateAngularSpeed(final double[] result) {
        if (result.length != N_COMPONENTS_3D) {
            // result must have length 3
            throw new IllegalArgumentException();
        }
        result[0] = stateAngularSpeedX;
        result[1] = stateAngularSpeedY;
        result[2] = stateAngularSpeedZ;
    }

    /**
     * Gets covariance matrix of state variables (position, velocity, acceleration, orientation and
     * angular speed).
     * Actual meaning of elements in returned matrix will depend on actual implementation of the estimator.
     *
     * @return covariance matrix of state variables.
     */
    public abstract Matrix getStateCovariance();

    /**
     * Indicates whether an error occurred during the estimation.
     * If an error occurs the estimator should be restarted since state values
     * might become unreliable.
     *
     * @return true if an error occurred since last start time, false otherwise.
     */
    public boolean hasError() {
        return error;
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
     * Gets average acceleration along x,y,z axes accumulated since last full
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
     * second.
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
     *                      expressed in nanoseconds.
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
     * @param timestamp     timestamp of gyroscope sample since epoch time and
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
     * Corrects system state with provided position measure having an accuracy
     * determined by provided covariance matrix.
     *
     * @param positionX          new position along x-axis expressed in meters (m).
     * @param positionY          new position along y-axis expressed in meters (m).
     * @param positionZ          new position along z-axis expressed in meters (m).
     * @param positionCovariance new position covariance matrix determining
     *                           new position accuracy or null if last available covariance does not need
     *                           to be updated.
     * @throws IllegalArgumentException if provided covariance matrix is not
     *                                  3x3.
     */
    public void correctWithPositionMeasure(
            final double positionX, final double positionY, final double positionZ, final Matrix positionCovariance) {
        setPositionCovarianceMatrix(positionCovariance);
        correctWithPositionMeasure(positionX, positionY, positionZ);
    }

    /**
     * Corrects system state with provided position measure having an accuracy
     * determined by provided covariance matrix.
     *
     * @param position           x,y,z coordinates of position expressed in meters (m).
     *                           Must have length 3.
     * @param positionCovariance new position covariance matrix determining new
     *                           position accuracy or null if last available covariance does not need to
     *                           be updated.
     * @throws IllegalArgumentException if provided covariance matrix is not
     *                                  3x3 or if provided position array does not have length 3.
     */
    public void correctWithPositionMeasure(final double[] position, final Matrix positionCovariance) {
        if (position.length != N_COMPONENTS_3D) {
            throw new IllegalArgumentException("position must have length 3");
        }
        correctWithPositionMeasure(position[0], position[1], position[2], positionCovariance);
    }

    /**
     * Corrects system state with provided position measure having an accuracy
     * determined by provided covariance matrix.
     *
     * @param position           position expressed in meters (m).
     * @param positionCovariance new position covariance matrix determining new
     *                           position accuracy or null if last available covariance does not need to
     *                           be updated.
     * @throws IllegalArgumentException if provided covariance matrix is not
     *                                  3x3.
     */
    public void correctWithPositionMeasure(final Point3D position, final Matrix positionCovariance) {
        correctWithPositionMeasure(position.getInhomX(), position.getInhomY(), position.getInhomZ(),
                positionCovariance);
    }

    /**
     * Updates covariance matrix of position measures.
     * If null is provided, covariance matrix is not updated.
     *
     * @param positionCovariance new position covariance determining position
     *                           accuracy or null if last available covariance does not need to be
     *                           updated.
     * @throws IllegalArgumentException if provided covariance matrix is not
     *                                  3x3.
     */
    public abstract void setPositionCovarianceMatrix(final Matrix positionCovariance);

    /**
     * Gets current covariance matrix of position measures determining current
     * accuracy of provided position measures.
     *
     * @return covariance matrix of position measures.
     */
    public abstract Matrix getPositionCovarianceMatrix();

    /**
     * Corrects system state with provided position measure using current
     * position accuracy.
     *
     * @param positionX new position along x-axis expressed in meters (m).
     * @param positionY new position along y-axis expressed in meters (m).
     * @param positionZ new position along z-axis expressed in meters (m).
     */
    public abstract void correctWithPositionMeasure(
            final double positionX, final double positionY, final double positionZ);

    /**
     * Corrects system state with provided position measure using current
     * position accuracy.
     *
     * @param position x,y,z coordinates of position expressed in meters (m).
     *                 Must have length 3.
     * @throws IllegalArgumentException if provided array does not have length
     *                                  3.
     */
    public void correctWithPositionMeasure(final double[] position) {
        correctWithPositionMeasure(position, null);
    }

    /**
     * Corrects system state with provided position measure using current
     * position accuracy.
     *
     * @param position position expressed in meters (m).
     */
    public void correctWithPositionMeasure(final Point3D position) {
        correctWithPositionMeasure(position, null);
    }

    /**
     * Gets listener in charge of handling events raised by instances of this
     * class.
     *
     * @return listener in charge of handling events raised by instances of this
     * class.
     */
    public BaseSlamEstimatorListener<D> getListener() {
        return listener;
    }

    /**
     * Sets listener in charge of handling events raised by instances of this
     * class.
     *
     * @param listener listener in charge of handling events raised by instances
     *                 of this class.
     */
    public void setListener(final BaseSlamEstimatorListener<D> listener) {
        this.listener = listener;
    }

    /**
     * Gets calibration data. When provided, its mean and covariance are used
     * to correct control samples and adjust process covariance matrix during
     * Kalman filtering in prediction stage.
     *
     * @return calibration data.
     */
    public D getCalibrationData() {
        return calibrationData;
    }

    /**
     * Sets calibration data. When provided, its mean and covariance are used
     * to correct control samples and adjust process covariance matrix during
     * Kalman filtering in prediction stage.
     *
     * @param calibrationData calibration data.
     */
    public void setCalibrationData(final D calibrationData) {
        this.calibrationData = calibrationData;
    }

    /**
     * Resets position, linear velocity, linear acceleration, orientation and
     * angular speed to provided values.
     *
     * @param statePositionX     position along x-axis expressed in meters (m).
     * @param statePositionY     position along y-axis expressed in meters (m).
     * @param statePositionZ     position along z-axis expressed in meters (m).
     * @param stateVelocityX     linear velocity along x-axis expressed in meters
     *                           per second (m/s).
     * @param stateVelocityY     linear velocity along y-axis expressed in meters
     *                           per second (m/s).
     * @param stateVelocityZ     linear velocity along z-axis expressed in meters
     *                           per second (m/s).
     * @param stateAccelerationX linear acceleration along x-axis expressed in
     *                           meters per squared second (m/s^2).
     * @param stateAccelerationY linear acceleration along y-axis expressed in
     *                           meters per squared second (m/s^2).
     * @param stateAccelerationZ linear acceleration along z-axis expressed in
     *                           meters per squared second (m/s^2).
     * @param stateQuaternionA   A value of orientation quaternion.
     * @param stateQuaternionB   B value of orientation quaternion.
     * @param stateQuaternionC   C value of orientation quaternion.
     * @param stateQuaternionD   D value of orientation quaternion.
     * @param stateAngularSpeedX angular speed along x-axis expressed in radians
     *                           per second (rad/s).
     * @param stateAngularSpeedY angular speed along y-axis expressed in radians
     *                           per second (rad/s).
     * @param stateAngularSpeedZ angular speed along z-axis expressed in radians
     *                           per second (rad/s).
     */
    protected void reset(
            final double statePositionX, final double statePositionY, final double statePositionZ,
            final double stateVelocityX, final double stateVelocityY, final double stateVelocityZ,
            final double stateAccelerationX, final double stateAccelerationY, final double stateAccelerationZ,
            final double stateQuaternionA, final double stateQuaternionB,
            final double stateQuaternionC, final double stateQuaternionD,
            final double stateAngularSpeedX, final double stateAngularSpeedY, final double stateAngularSpeedZ) {
        this.statePositionX = statePositionX;
        this.statePositionY = statePositionY;
        this.statePositionZ = statePositionZ;
        this.stateVelocityX = stateVelocityX;
        this.stateVelocityY = stateVelocityY;
        this.stateVelocityZ = stateVelocityZ;
        this.stateAccelerationX = stateAccelerationX;
        this.stateAccelerationY = stateAccelerationY;
        this.stateAccelerationZ = stateAccelerationZ;
        this.stateQuaternionA = stateQuaternionA;
        this.stateQuaternionB = stateQuaternionB;
        this.stateQuaternionC = stateQuaternionC;
        this.stateQuaternionD = stateQuaternionD;
        this.stateAngularSpeedX = stateAngularSpeedX;
        this.stateAngularSpeedY = stateAngularSpeedY;
        this.stateAngularSpeedZ = stateAngularSpeedZ;
        accelerometerTimestampNanos = gyroscopeTimestampNanos = -1;
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
     * Method to be implemented in subclasses to process a full sample.
     */
    protected abstract void processFullSample();

    /**
     * Listener for implementations of this class.
     *
     * @param <D> calibrator type associated to implementations of SLAM calibration
     *            data.
     */
    public interface BaseSlamEstimatorListener<D extends BaseCalibrationData> {
        /**
         * Called when a full sample (accelerometer + gyroscope, etc.) has been
         * received and is about to be processed to update internal state.
         *
         * @param estimator SLAM estimator.
         */
        void onFullSampleReceived(final BaseSlamEstimator<D> estimator);

        /**
         * Called when a full sample (accelerometer + gyroscope, etc.) has been
         * received and has already been processed, and hence internal state has
         * also been updated.
         *
         * @param estimator SLAM estimator.
         */
        void onFullSampleProcessed(final BaseSlamEstimator<D> estimator);

        /**
         * Called when internal state is about to be corrected by using an
         * external measure.
         *
         * @param estimator SLAM estimator.
         */
        void onCorrectWithPositionMeasure(final BaseSlamEstimator<D> estimator);

        /**
         * Called after internal state has been corrected using an external
         * measure.
         *
         * @param estimator SLAM estimator.
         */
        void onCorrectedWithPositionMeasure(final BaseSlamEstimator<D> estimator);
    }
}
