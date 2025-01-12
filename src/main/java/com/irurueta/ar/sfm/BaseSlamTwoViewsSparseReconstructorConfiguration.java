/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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

package com.irurueta.ar.sfm;

import com.irurueta.ar.slam.BaseCalibrationData;

import java.io.Serializable;

/**
 * Contains base configuration for a two view sparse re-constructor using SLAM
 * (Simultaneous Location And Mapping) to determine the scale of the scene
 * (i.e. the baseline or separation between cameras) by fusing both camera data
 * and data from sensors like an accelerometer or gyroscope.
 *
 * @param <C> type defining calibration data.
 * @param <T> an actual implementation of a configuration class.
 */
public abstract class BaseSlamTwoViewsSparseReconstructorConfiguration<
        C extends BaseCalibrationData,
        T extends BaseSlamTwoViewsSparseReconstructorConfiguration<C, T>> extends
        BaseTwoViewsSparseReconstructorConfiguration<T> implements Serializable {

    /**
     * Indicates that by default new available SLAM state is notified each time that a whole set of IMU
     * (Inertial Measurement Unit) data is received (accelerometer, gyroscope and orientation). SLAM state
     * contains position, velocity, linear acceleration, orientation and angular speed.
     */
    public static final boolean DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE = true;

    /**
     * Indicates that by default any new camera that can be estimated by means of SLAM using IMU data,
     * will be notified each time that accelerometer, gyroscope and orientation data is received.
     */
    public static final boolean DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA = true;

    /**
     * Calibration data for accelerometer and gyroscope.
     * This data is usually captured and estimated in an offline step previous
     * to the actual scene reconstruction.
     * Calibration data is usually obtained by keeping the system in a constant
     * state of motion (e.g. acceleration and rotation).
     * If this is null, no calibration data will be used.
     */
    private C calibrationData;

    /**
     * Indicates whether new available SLAM state is notified each time that a whole set of IMU (Inertial
     * Measurement Unit) data is received.
     */
    private boolean notifyAvailableSlamData = DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE;

    /**
     * Indicates whether any new camera that can be estimated by means of SLAM using IMU data, will be
     * notified each time that accelerometer, gyroscope and orientation data is received.
     */
    private boolean notifyEstimatedSlamCamera = DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA;

    /**
     * Constructor.
     */
    protected BaseSlamTwoViewsSparseReconstructorConfiguration() {
    }

    /**
     * Gets calibration data for accelerometer and gyroscope.
     * This data is usually captured and estimated in an offline step previous
     * to the actual scene reconstruction.
     * Calibration data is usually obtained by keeping the system in a constant
     * state of motion (e.g. acceleration and rotation).
     * If this is null, no calibration data will be used.
     *
     * @return calibration data or null.
     */
    public C getCalibrationData() {
        return calibrationData;
    }

    /**
     * Specifies calibration data for accelerometer and gyroscope.
     * This data is usually captured and estimated in an offline step previous
     * to the actual scene reconstruction.
     * Calibration data is usually obtained by keeping the system in a constant
     * state of motion (e.g. acceleration and rotation).
     * If set to null, no calibration data will be used.
     *
     * @param calibrationData calibration data or null.
     * @return this instance so that method can be easily chained.
     */
    public T setCalibrationData(final C calibrationData) {
        this.calibrationData = calibrationData;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether new available SLAM state is notified each time that a whole set of IMU (Inertial
     * Measurement Unit) data is received. IMU data contains accelerometer, gyroscope and orientation
     * samples.
     *
     * @return true if new available SLAM state is notified each time that a whole set of IMU data is
     * received.
     */
    public boolean isNotifyAvailableSlamDataEnabled() {
        return notifyAvailableSlamData;
    }

    /**
     * Specifies whether new available SLAM state is notified each time that a whole set of IMU (Inertial
     * Measurement Unit) data is received. IMU data contains accelerometer, gyroscope and orientation
     * samples.
     *
     * @param notifyAvailableSlamData true is new available SLAM state is notified each time that a whole
     *                                set of IMU data is received, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setNotifyAvailableSlamDataEnabled(final boolean notifyAvailableSlamData) {
        this.notifyAvailableSlamData = notifyAvailableSlamData;

        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether any new camera that can be estimated by means of SLAM using IMU data, will be
     * notified each time that accelerometer, gyroscope and orientation data is received.
     *
     * @return true if any newly estimated camera is notified, false otherwise.
     */
    public boolean isNotifyEstimatedSlamCameraEnabled() {
        return notifyEstimatedSlamCamera;
    }

    /**
     * Specifies whether any new camera that can be estimated by means of SLAM using IMU data, will be
     * notified each time that accelerometer, gyroscope and orientation data is received.
     *
     * @param notifyEstimatedSlamCamera true if any newly estimated camera is notified, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setNotifyEstimatedSlamCameraEnabled(final boolean notifyEstimatedSlamCamera) {
        this.notifyEstimatedSlamCamera = notifyEstimatedSlamCamera;

        //noinspection unchecked
        return (T) this;
    }
}
