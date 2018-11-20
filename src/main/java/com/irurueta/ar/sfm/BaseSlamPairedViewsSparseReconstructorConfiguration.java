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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.ar.slam.BaseCalibrationData;
import com.irurueta.geometry.Point3D;

import java.io.Serializable;

/**
 * Contains base configuration for a paired view sparse reconstructor using SLAM (Simultaneous
 * Location And Mapping) to determine the scale of the scene (i.e. the baseline or separation
 * between cameras) by fusing both camera data and data from sensors like an accelerometer or
 * gyroscope.
 * @param <C> type defining calibration data.
 * @param <T> an actual implementation of a configuration class.
 */
public class BaseSlamPairedViewsSparseReconstructorConfiguration<C extends BaseCalibrationData,
        T extends BaseSlamPairedViewsSparseReconstructorConfiguration> extends
        BasePairedViewsSparseReconstructorConfiguration<T> implements Serializable {

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
     * Default variance for coordinates of estimated camera positions.
     */
    private static final double DEFAULT_CAMERA_POSITION_VARIANCE = 1e-6;

    /**
     * Calibration data for accelerometer and gyroscope.
     * This data is usually captured and estimated in an offline step previous to the
     * actual scene reconstruction.
     * Calibration data is usually obtained by keeping the system in a constant state of
     * motion (e.g. acceleration and rotation).
     * If this is null, no claibration data will be used.
     */
    private C mCalibrationData;

    /**
     * Matrix containing covariance of measured camera positions.
     * This should usually be an "almost" diagonal matrix, where diagonal elements are
     * close to the position estimation error variance.
     * Values of this matrix are device specific and depends on factors such as resolution
     * of images, pictures quality, gyroscope and accelerometer accuracy.
     * This matrix must be a 3x3 symmetric positive definite matrix.
     */
    private Matrix mCameraPositionCovariance;

    /**
     * Indicates whether new available SLAM state is notified each time that a whole set of IMU
     * (Inertial Measurement Unit) data is received.
     */
    private boolean mNotifyAvailableSlamData = DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE;

    /**
     * Indicates whether any new camera that can be estimated by means of SLAM using IMU data, will be
     * notified each time that accelerometer, gyroscope and orientation data is received.
     */
    private boolean mNotifyEstimatedSlamCamera = DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA;

    /**
     * Constructor.
     */
    public BaseSlamPairedViewsSparseReconstructorConfiguration() {
        //initialize default covariance
        try {
            mCameraPositionCovariance = Matrix.identity(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
            mCameraPositionCovariance.multiplyByScalar(DEFAULT_CAMERA_POSITION_VARIANCE);
        } catch (AlgebraException ignore) { }
    }

    /**
     * Gets calibration data for accelerometer and gyroscope.
     * This data is usually captured and estimated in an offline step previous to the
     * actual scene reconstruction.
     * Calibration data is usually obtained by keeping the system in a constant state
     * of motion (e.g. acceleration and rotation).
     * If this is null, no calibration data will be used.
     * @return calibration data or null.
     */
    public C getCalibrationData() {
        return mCalibrationData;
    }

    /**
     * Specifies calibration data for accelerometer and gyroscope.
     * This data is usually captured and estimated in an offline step previous to the
     * actual scene reconstruction.
     * Calibration data is usually obtained by keeping the system in a constant state
     * of motion (e.g. acceleration and rotation).
     * If set to null, no calibration data will be used.
     * @param calibrationData calibration data or null.
     * @return this instance so that method can be easily chained.
     */
    public T setCalibrationData(C calibrationData) {
        mCalibrationData = calibrationData;

        //noinspection all
        return (T)this;
    }

    /**
     * Gets matrix containing covariance of measured camera positions.
     * This should usually be an "almost" diagonal matrix, where diagonal elements are
     * close to the position estimation error variance.
     * Values of this matrix are device specific and depends on factors such as
     * resolution of images, pictures quality, gyroscope and accelerometer accuracy.
     * This matrix must be a 3x3 symmetric positive definite matrix.
     * @return covariance of measured camera positions.
     */
    public Matrix getCameraPositionCovariance() {
        return mCameraPositionCovariance;
    }

    /**
     * Sets matrix containing covariance of measured camera positions.
     * This should usually be an "almost" diagonal matrix, where diagonal elements are
     * close to the position estimation error variance.
     * Values of this matrix are device specific and depends on factors such as
     * resolution of images, pictures quality, gyroscope and accelerometer accuracy.
     * This matrix must be 3x3 symmetric positive definite matrix.
     * @param cameraPositionCovariance covariance of measured camera positions.
     * @return this instance so that method can be easily chained.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public T setCameraPositionCovariance(Matrix cameraPositionCovariance)
            throws IllegalArgumentException {
        if (cameraPositionCovariance.getRows() != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH ||
                cameraPositionCovariance.getColumns() != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            throw new IllegalArgumentException();
        }

        mCameraPositionCovariance = cameraPositionCovariance;

        //noinspection all
        return (T)this;
    }

    /**
     * Sets independent variance of coordinates of measured camera positions.
     * When using this method, camera position covariance matrix is set as a diagonal
     * matrix whose diagonal elements are equal to provided value.
     * @param variance variance of coordinates of measured camera positions.
     * @return this instance so that method can be easily chained.
     */
    public T setCameraPositionVariance(double variance) {
        try {
            mCameraPositionCovariance = Matrix.identity(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
            mCameraPositionCovariance.multiplyByScalar(variance);
        } catch (AlgebraException ignore) { }

        //noinspection all
        return (T)this;
    }

    /**
     * Indicates whether new available SLAM state is notified each time that a whole set
     * of IMU (Inertial Measurement Unit) data is received. IMU data contains
     * accelerometer, gyroscope and orientation samples.
     * @return true if new available SLAM state is notified each time that a whole set
     * of IMU data is received.
     */
    public boolean isNotifyAvailableSlamDataEnabled() {
        return mNotifyAvailableSlamData;
    }

    /**
     * Specifies whether new available SLAM state is notified each time that a whole set
     * of IMU (Inertial Measurement Unit) data is received. IMU data contains accelerometer,
     * gyroscope and orientation samples.
     * @param notifyAvailableSlamData true if new available SLAM state is notified each
     *                                time that a whole set of IMU data is received, false
     *                                otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setNotifyAvailableSlamDataEnabled(boolean notifyAvailableSlamData) {
        mNotifyAvailableSlamData = notifyAvailableSlamData;

        //noinspection all
        return (T)this;
    }

    /**
     * Indicates whether any new camera that can be estimated by means of SLAM using IMU
     * data, will be notified each time that accelerometer, gyroscope and orientation
     * data is received.
     * @return true if any newly estimated camera is notified, false otherwise.
     */
    public boolean isNotifyEstimatedSlamCameraEnabled() {
        return mNotifyEstimatedSlamCamera;
    }

    /**
     * Specifies whether any new camera that can be estimated by means of SLAM using IMU
     * data, will be notified each time that accelerometer, gyroscope and orientation
     * data is received.
     * @param notifyEstimatedSlamCamera true if any newly estimated camera is notified,
     *                                  false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setNotifyEstimatedSlamCameraEnabled(boolean notifyEstimatedSlamCamera) {
        mNotifyEstimatedSlamCamera = notifyEstimatedSlamCamera;

        //noinspection all
        return (T)this;
    }
}
