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

/**
 * Base class for estimating mean and covariance of noise in control values
 * when the system state is held constant (only noise is provided as control
 * input).
 * This subclass of BaseSlamCalibrator is used for slam estimator taking into
 * account absolute orientation.
 *
 * @param <D> type of calibration data.
 */
public abstract class AbsoluteOrientationBaseSlamCalibrator<D extends BaseCalibrationData> extends
        BaseSlamCalibrator<D> {

    /**
     * Timestamp expressed in nanoseconds since the epoch time of the last
     * sample containing absolute orientation.
     */
    protected long mOrientationTimestampNanos = -1;

    /**
     * Number of orientation samples accumulated since last full sample.
     */
    protected int mAccumulatedOrientationSamples = 0;

    /**
     * Average orientation accumulated since last full sample.
     */
    protected final Quaternion mAccumulatedOrientation;

    /**
     * Temporary quaternion. For memory reuse.
     */
    private Quaternion mTempQ;

    /**
     * Constructor.
     *
     * @param sampleLength sample length of control values used during
     *                     prediction stage in SLAM estimator.
     * @throws IllegalArgumentException if sample length is less than 1.
     */
    protected AbsoluteOrientationBaseSlamCalibrator(final int sampleLength) {
        super(sampleLength);
        mAccumulatedOrientation = new Quaternion();
    }

    /**
     * Gets timestamp expressed in nanoseconds since the epoch time of the last
     * orientation sample, or -1 if no sample has been set yet.
     *
     * @return timestamp expressed in nanoseconds since the epoch time of the
     * last orientation sample, or -1.
     */
    public long getOrientationTimestampNanos() {
        return mOrientationTimestampNanos;
    }

    /**
     * Gets average orientation accumulated since last full sample.
     *
     * @return orientation accumulated since last full sample.
     */
    public Rotation3D getAccumulatedOrientation() {
        return mAccumulatedOrientation.toQuaternion();
    }

    /**
     * Gets average orientation accumulated since last full sample.
     *
     * @param result instance where orientation accumulated since last full
     *               sample will be stored.
     */
    public void getAccumulatedOrientation(final Rotation3D result) {
        result.fromRotation(mAccumulatedOrientation);
    }

    /**
     * Gets number of orientation samples accumulated since last full sample.
     *
     * @return number of orientation samples accumulated since last full sample.
     */
    public int getAccumulatedOrientationSamples() {
        return mAccumulatedOrientationSamples;
    }

    /**
     * Indicates whether the orientation sample has been received since last
     * full sample (accelerometer + gyroscope + orientation).
     *
     * @return true if orientation sample has been received, false otherwise.
     */
    public boolean isOrientationSampleReceived() {
        return mAccumulatedOrientationSamples > 0;
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
    public void updateOrientationSample(final long timestamp,
                                        final Rotation3D orientation) {
        if (!isFullSampleAvailable()) {
            mOrientationTimestampNanos = timestamp;
            if (isAccumulationEnabled() && isOrientationSampleReceived()) {
                // accumulation enabled
                final int nextSamples = mAccumulatedOrientationSamples + 1;

                double accumA = mAccumulatedOrientation.getA();
                double accumB = mAccumulatedOrientation.getB();
                double accumC = mAccumulatedOrientation.getC();
                double accumD = mAccumulatedOrientation.getD();

                if (mTempQ == null) {
                    mTempQ = new Quaternion();
                }
                mTempQ.fromRotation(orientation);
                mTempQ.normalize();
                final double a = mTempQ.getA();
                final double b = mTempQ.getB();
                final double c = mTempQ.getC();
                final double d = mTempQ.getD();

                accumA = (accumA * mAccumulatedOrientationSamples + a) /
                        nextSamples;
                accumB = (accumB * mAccumulatedOrientationSamples + b) /
                        nextSamples;
                accumC = (accumC * mAccumulatedOrientationSamples + c) /
                        nextSamples;
                accumD = (accumD * mAccumulatedOrientationSamples + d) /
                        nextSamples;

                mAccumulatedOrientation.setA(accumA);
                mAccumulatedOrientation.setB(accumB);
                mAccumulatedOrientation.setC(accumC);
                mAccumulatedOrientation.setD(accumD);
                mAccumulatedOrientationSamples = nextSamples;
            } else {
                // accumulation disabled
                mAccumulatedOrientation.fromRotation(orientation);
                mAccumulatedOrientationSamples++;
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
        final long mostRecent = super.getMostRecentTimestampNanos();
        return Math.max(mostRecent, mOrientationTimestampNanos);
    }

    /**
     * Notifies that a full sample has been received and resets flags indicating
     * whether partial samples have been received.
     */
    @Override
    protected void notifyFullSampleAndResetSampleReceive() {
        if (isFullSampleAvailable()) {
            processFullSample();
            mAccumulatedAccelerometerSamples = mAccumulatedGyroscopeSamples =
                    mAccumulatedOrientationSamples = 0;
        }
    }
}
