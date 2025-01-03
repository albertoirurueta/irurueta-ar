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

import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;

import java.io.Serializable;

/**
 * Base class to estimate position, velocity, acceleration and orientation of
 * a device using sensor data such as accelerometers, gyroscopes and magnetic
 * fields (to obtain absolute orientation).
 * Implementations of this class are designed taking into account sensors
 * available on Android devices.
 * This subclass of BaseSlamEstimator takes into account magnetic field measures
 * to compute device orientation respect Earth's frame rather than relative to
 * the start position and orientation of the device.
 *
 * @param <D> calibrator type associated to implementations of SLAM calibration
 *            data.
 */
public abstract class AbsoluteOrientationBaseSlamEstimator<D extends BaseCalibrationData> extends
        BaseSlamEstimator<D> implements Serializable {

    /**
     * Timestamp expressed in nanoseconds since the epoch time of the last
     * sample containing absolute orientation.
     */
    protected long orientationTimestampNanos = -1;

    /**
     * Number of orientation samples accumulated since last full sample.
     */
    protected int accumulatedOrientationSamples = 0;

    /**
     * Average orientation accumulated since last full sample.
     */
    protected Quaternion accumulatedOrientation = new Quaternion();

    /**
     * Temporary quaternion. For memory reuse.
     */
    private Quaternion tempQ;

    /**
     * Constructor.
     */
    protected AbsoluteOrientationBaseSlamEstimator() {
        super();
    }

    /**
     * Gets timestamp expressed in nanoseconds since the epoch time of the last
     * orientation sample, or -1 if no sample has been set yet.
     *
     * @return timestamp expressed in nanoseconds since the epoch time of the
     * last orientation sample, or -1.
     */
    public long getOrientationTimestampNanos() {
        return orientationTimestampNanos;
    }

    /**
     * Gets average orientation accumulated since last full sample.
     *
     * @return orientation accumulated since last full sample.
     */
    public Rotation3D getAccumulatedOrientation() {
        return accumulatedOrientation.toQuaternion();
    }

    /**
     * Gets average orientation accumulated since last full sample.
     *
     * @param result instance where orientation accumulated since last full
     *               sample will be stored.
     */
    public void getAccumulatedOrientation(final Rotation3D result) {
        result.fromRotation(accumulatedOrientation);
    }

    /**
     * Gets number of orientation samples accumulated since last full sample.
     *
     * @return number of orientation samples accumulated since last full sample.
     */
    public int getAccumulatedOrientationSamples() {
        return accumulatedOrientationSamples;
    }

    /**
     * Indicates whether the orientation sample has been received since last
     * full sample (accelerometer + gyroscope + orientation).
     *
     * @return true if orientation sample has been received, false otherwise.
     */
    public boolean isOrientationSampleReceived() {
        return accumulatedOrientationSamples > 0;
    }

    /**
     * Indicates whether a full sample (accelerometer + gyroscope +
     * magnetic field) has been received or not.
     *
     * @return true if full sample has been received, false otherwise.
     */
    @Override
    public boolean isFullSampleAvailable() {
        return super.isFullSampleAvailable() && isOrientationSampleReceived();
    }

    /**
     * Provides a new orientation sample.
     * If accumulation is enabled, samples are averaged until a full sample is
     * received.
     * When a full sample (accelerometer + gyroscope + orientation) is
     * received, internal state gets also updated.
     *
     * @param timestamp   timestamp of accelerometer sample since epoch time and
     *                    expressed in nanoseconds.
     * @param orientation new orientation.
     */
    @SuppressWarnings("DuplicatedCode")
    public void updateOrientationSample(final long timestamp, final Rotation3D orientation) {
        if (!isFullSampleAvailable()) {
            orientationTimestampNanos = timestamp;
            if (isAccumulationEnabled() && isOrientationSampleReceived()) {
                // accumulation enabled
                final var nextSamples = accumulatedOrientationSamples + 1;

                var accumA = accumulatedOrientation.getA();
                var accumB = accumulatedOrientation.getB();
                var accumC = accumulatedOrientation.getC();
                var accumD = accumulatedOrientation.getD();

                if (tempQ == null) {
                    tempQ = new Quaternion();
                }
                tempQ.fromRotation(orientation);
                tempQ.normalize();
                final var a = tempQ.getA();
                final var b = tempQ.getB();
                final var c = tempQ.getC();
                final var d = tempQ.getD();

                accumA = (accumA * accumulatedOrientationSamples + a) / nextSamples;
                accumB = (accumB * accumulatedOrientationSamples + b) / nextSamples;
                accumC = (accumC * accumulatedOrientationSamples + c) / nextSamples;
                accumD = (accumD * accumulatedOrientationSamples + d) / nextSamples;

                accumulatedOrientation.setA(accumA);
                accumulatedOrientation.setB(accumB);
                accumulatedOrientation.setC(accumC);
                accumulatedOrientation.setD(accumD);
                accumulatedOrientationSamples = nextSamples;
            } else {
                // accumulation disabled
                accumulatedOrientation.fromRotation(orientation);
                accumulatedOrientationSamples++;
            }
            notifyFullSampleAndResetSampleReceive();
        }
    }

    /**
     * Gets most recent timestamp of received partial samples (accelerometer,
     * gyroscope or magnetic field).
     *
     * @return most recent timestamp of received partial sample.
     */
    @Override
    public long getMostRecentTimestampNanos() {
        final var mostRecent = super.getMostRecentTimestampNanos();
        return Math.max(mostRecent, orientationTimestampNanos);
    }

    /**
     * Notifies that a full sample has been received and resets flags indicating
     * whether partial samples have been received.
     */
    @Override
    protected void notifyFullSampleAndResetSampleReceive() {
        if (isFullSampleAvailable()) {
            processFullSample();
            accumulatedAccelerometerSamples = accumulatedGyroscopeSamples = accumulatedOrientationSamples = 0;
        }
    }
}
