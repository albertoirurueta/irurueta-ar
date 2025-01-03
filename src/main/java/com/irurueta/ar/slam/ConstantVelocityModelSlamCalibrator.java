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

/**
 * Processes data to estimate calibration for constant velocity model SLAM
 * estimator.
 * This class must be used while gathering data for a system being kept constant
 * (no motion).
 */
public class ConstantVelocityModelSlamCalibrator extends BaseSlamCalibrator<ConstantVelocityModelSlamCalibrationData> {

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
     * Constructor.
     */
    public ConstantVelocityModelSlamCalibrator() {
        super(ConstantVelocityModelSlamEstimator.CONTROL_LENGTH);
    }

    /**
     * Resets calibrator.
     */
    @Override
    public void reset() {
        super.reset();
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
        return ConstantVelocityModelSlamEstimator.STATE_LENGTH;
    }

    /**
     * Gets a new instance containing calibration data estimated by this
     * calibrator.
     *
     * @return a new calibration data instance.
     */
    @Override
    public ConstantVelocityModelSlamCalibrationData getCalibrationData() {
        final var result = new ConstantVelocityModelSlamCalibrationData();
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
            lastAngularSpeedX = accumulatedAngularSpeedSampleX;
            lastAngularSpeedY = accumulatedAngularSpeedSampleY;
            lastAngularSpeedZ = accumulatedAngularSpeedSampleZ;

            lastTimestampNanos = timestamp;

            if (listener != null) {
                listener.onFullSampleProcessed(this);
            }

            return;
        }

        final var deltaAngularSpeedX = accumulatedAngularSpeedSampleX - lastAngularSpeedX;
        final var deltaAngularSpeedY = accumulatedAngularSpeedSampleY - lastAngularSpeedY;
        final var deltaAngularSpeedZ = accumulatedAngularSpeedSampleZ - lastAngularSpeedZ;
        final var deltaTimestamp = (timestamp - lastTimestampNanos) * NANOS_TO_SECONDS;

        sample[0] = accumulatedAccelerationSampleX * deltaTimestamp;
        sample[1] = accumulatedAccelerationSampleY * deltaTimestamp;
        sample[2] = accumulatedAccelerationSampleZ * deltaTimestamp;
        sample[3] = deltaAngularSpeedX;
        sample[4] = deltaAngularSpeedY;
        sample[5] = deltaAngularSpeedZ;
        updateSample();

        lastAngularSpeedX = accumulatedAngularSpeedSampleX;
        lastAngularSpeedY = accumulatedAngularSpeedSampleY;
        lastAngularSpeedZ = accumulatedAngularSpeedSampleZ;

        lastTimestampNanos = timestamp;

        if (listener != null) {
            listener.onFullSampleProcessed(this);
        }
    }
}
