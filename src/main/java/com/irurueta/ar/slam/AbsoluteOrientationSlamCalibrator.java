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

/**
 * Processes data to estimate calibration for absolute orientation SLAM
 * estimator.
 * This class must be used while gathering data for a system being kept constant
 * (under no motion).
 */
public class AbsoluteOrientationSlamCalibrator extends
        AbsoluteOrientationBaseSlamCalibrator<AbsoluteOrientationSlamCalibrationData> {

    /**
     * Last sample of linear acceleration along x-axis.
     */
    private double lastAccelerationX;

    /**
     * Last sample of linear acceleration along y-axis.
     */
    private double lastAccelerationY;

    /**
     * Last sample of linear acceleration along z-axis.
     */
    private double lastAccelerationZ;

    /**
     * Last sample of angular speed along x-axis.
     */
    private double lastAngularSpeedX;

    /**
     * Last sample of angular speed along y-axis.
     */
    private double lastAngularSpeedY;

    /**
     * Last sample of angular speed along z-axis.
     */
    private double lastAngularSpeedZ;

    /**
     * Last timestamp of a full sample expressed in nanoseconds since the epoch
     * time.
     */
    private long lastTimestampNanos = -1;

    /**
     * Last sample of absolute orientation.
     */
    private Quaternion lastOrientation;

    /**
     * Variation of orientation respect to last sample.
     */
    private final Quaternion deltaOrientation;

    /**
     * Constructor.
     */
    public AbsoluteOrientationSlamCalibrator() {
        super(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH);
        lastOrientation = new Quaternion();
        deltaOrientation = new Quaternion();
    }

    /**
     * Resets calibrator.
     */
    @Override
    public void reset() {
        super.reset();
        lastOrientation = new Quaternion();
        lastAccelerationX = lastAccelerationY = lastAccelerationZ =
                lastAngularSpeedX = lastAngularSpeedY = lastAngularSpeedZ = 0.0;
        lastTimestampNanos = -1;
    }

    /**
     * Obtains the number of state parameters in associated SLAM estimator.
     *
     * @return number of state parameters.
     */
    @Override
    protected int getEstimatorStateLength() {
        return AbsoluteOrientationSlamEstimator.STATE_LENGTH;
    }

    /**
     * Gets a new instance containing calibration data estimated by this
     * calibrator.
     *
     * @return a new calibration data instance.
     */
    @Override
    public AbsoluteOrientationSlamCalibrationData getCalibrationData() {
        final var result = new AbsoluteOrientationSlamCalibrationData();
        getCalibrationData(result);
        return result;
    }

    /**
     * Processes a full sample of accelerometer and gyroscope data to compute
     * statistics such as mean and covariance of variations.
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    protected void processFullSample() {
        if (listener != null) {
            listener.onFullSampleReceived(this);
        }

        final var timestamp = getMostRecentTimestampNanos();
        if (lastTimestampNanos < 0) {
            // first time receiving control data we cannot determine its
            // variation
            lastOrientation.fromQuaternion(accumulatedOrientation);

            lastAccelerationX = accumulatedAccelerationSampleX;
            lastAccelerationY = accumulatedAccelerationSampleY;
            lastAccelerationZ = accumulatedAccelerationSampleZ;

            lastAngularSpeedX = accumulatedAngularSpeedSampleX;
            lastAngularSpeedY = accumulatedAngularSpeedSampleY;
            lastAngularSpeedZ = accumulatedAngularSpeedSampleZ;

            lastTimestampNanos = timestamp;

            if (listener != null) {
                listener.onFullSampleProcessed(this);
            }

            return;
        }

        accumulatedOrientation.normalize();

        lastOrientation.inverse(deltaOrientation);
        deltaOrientation.combine(accumulatedOrientation);
        deltaOrientation.normalize();

        final var deltaAccelerationX = accumulatedAccelerationSampleX - lastAccelerationX;
        final var deltaAccelerationY = accumulatedAccelerationSampleY - lastAccelerationY;
        final var deltaAccelerationZ = accumulatedAccelerationSampleZ - lastAccelerationZ;
        final var deltaAngularSpeedX = accumulatedAngularSpeedSampleX - lastAngularSpeedX;
        final var deltaAngularSpeedY = accumulatedAngularSpeedSampleY - lastAngularSpeedY;
        final var deltaAngularSpeedZ = accumulatedAngularSpeedSampleZ - lastAngularSpeedZ;

        sample[0] = deltaOrientation.getA();
        sample[1] = deltaOrientation.getB();
        sample[2] = deltaOrientation.getC();
        sample[3] = deltaOrientation.getD();
        sample[4] = sample[5] = sample[6] = 0.0;

        sample[7] = deltaAccelerationX;
        sample[8] = deltaAccelerationY;
        sample[9] = deltaAccelerationZ;

        sample[10] = deltaAngularSpeedX;
        sample[11] = deltaAngularSpeedY;
        sample[12] = deltaAngularSpeedZ;
        updateSample();

        lastOrientation.combine(deltaOrientation);
        lastAccelerationX = accumulatedAccelerationSampleX;
        lastAccelerationY = accumulatedAccelerationSampleY;
        lastAccelerationZ = accumulatedAccelerationSampleZ;

        lastAngularSpeedX = accumulatedAngularSpeedSampleX;
        lastAngularSpeedY = accumulatedAngularSpeedSampleY;
        lastAngularSpeedZ = accumulatedAngularSpeedSampleZ;

        lastTimestampNanos = timestamp;

        if (listener != null) {
            listener.onFullSampleProcessed(this);
        }
    }
}
