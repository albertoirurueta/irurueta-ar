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
 * Processes data to estimate calibration for SLAM estimator.
 * This class must be used while gathering data for a system being kept constant
 * (no motion).
 */
public class SlamCalibrator extends BaseSlamCalibrator<SlamCalibrationData> {

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
     * Constructor.
     */
    public SlamCalibrator() {
        super(SlamEstimator.CONTROL_LENGTH);
    }

    /**
     * Resets calibrator.
     */
    @Override
    public void reset() {
        super.reset();
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
        return SlamEstimator.STATE_LENGTH;
    }

    /**
     * Gets a new instance containing calibration data estimated by this
     * calibrator.
     *
     * @return a new calibration data instance.
     */
    @Override
    public SlamCalibrationData getCalibrationData() {
        final var result = new SlamCalibrationData();
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

        final var deltaAccelerationX = accumulatedAccelerationSampleX - lastAccelerationX;
        final var deltaAccelerationY = accumulatedAccelerationSampleY - lastAccelerationY;
        final var deltaAccelerationZ = accumulatedAccelerationSampleZ - lastAccelerationZ;
        final var deltaAngularSpeedX = accumulatedAngularSpeedSampleX - lastAngularSpeedX;
        final var deltaAngularSpeedY = accumulatedAngularSpeedSampleY - lastAngularSpeedY;
        final var deltaAngularSpeedZ = accumulatedAngularSpeedSampleZ - lastAngularSpeedZ;

        sample[0] = sample[1] = sample[2] = 0.0;
        sample[3] = deltaAccelerationX;
        sample[4] = deltaAccelerationY;
        sample[5] = deltaAccelerationZ;
        sample[6] = deltaAngularSpeedX;
        sample[7] = deltaAngularSpeedY;
        sample[8] = deltaAngularSpeedZ;
        updateSample();

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
