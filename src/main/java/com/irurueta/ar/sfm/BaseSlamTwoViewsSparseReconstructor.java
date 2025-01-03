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
import com.irurueta.ar.slam.BaseSlamEstimator;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MetricTransformation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;

import java.util.ArrayList;

/**
 * Base class in charge of estimating cameras and 3D reconstructed points from
 * sparse image point correspondences in two views and also in charge of
 * estimating overall scene scale by means of SLAM (Simultaneous Location And
 * Mapping) using data obtained from sensors like accelerometers or gyroscopes.
 *
 * @param <D> type of calibration data.
 * @param <C> type of configuration.
 * @param <R> type of re-constructor.
 * @param <L> type of listener.
 * @param <S> type of SLAM estimator.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class BaseSlamTwoViewsSparseReconstructor<
        D extends BaseCalibrationData,
        C extends BaseSlamTwoViewsSparseReconstructorConfiguration<D, C>,
        R extends BaseSlamTwoViewsSparseReconstructor<D, C, R, L, S>,
        L extends BaseSlamTwoViewsSparseReconstructorListener<R>,
        S extends BaseSlamEstimator<D>> extends BaseTwoViewsSparseReconstructor<C, R, L> {

    /**
     * Slam estimator to estimate position, speed, orientation using
     * accelerometer and gyroscope data.
     */
    protected S slamEstimator;

    /**
     * Position estimated by means of SLAM. It is reused each time it is notified.
     */
    private final InhomogeneousPoint3D slamPosition = new InhomogeneousPoint3D();

    /**
     * Camera estimated by means of SLAM. It is reused each time it is notified.
     */
    private final PinholeCamera slamCamera = new PinholeCamera();

    /**
     * Camera rotation estimated by means of SLAM. It is reused each time it is notified.
     */
    private final Quaternion slamRotation = new Quaternion();

    /**
     * Constructor.
     *
     * @param configuration configuration for this re-constructor.
     * @param listener      listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     *                              provided.
     */
    protected BaseSlamTwoViewsSparseReconstructor(final C configuration, final L listener) {
        super(configuration, listener);
    }

    /**
     * Provides a new accelerometer sample to update SLAM estimation.
     * This method must be called whenever the accelerometer sensor receives new
     * data.
     * If re-constructor is not running, calling this method has no effect.
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
        if (slamEstimator != null) {
            slamEstimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
        }
    }

    /**
     * Provides a new accelerometer sample to update SLAM estimation.
     * This method must be called whenever the accelerometer sensor receives new
     * data.
     * If re-constructor is not running, calling this method has no effect.
     *
     * @param timestamp timestamp of accelerometer sample since epoch time and
     *                  expressed in nanoseconds.
     * @param data      array containing x,y,z components of linear acceleration
     *                  expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided array does not have length
     *                                  3.
     */
    public void updateAccelerometerSample(final long timestamp, final float[] data) {
        if (slamEstimator != null) {
            slamEstimator.updateAccelerometerSample(timestamp, data);
        }
    }

    /**
     * Provides a new gyroscope sample to update SLAM estimation.
     * If re-constructor is not running, calling this method has no effect.
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
        if (slamEstimator != null) {
            slamEstimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
        }
    }

    /**
     * Provides a new gyroscope sample to update SLAM estimation.
     * If re-constructor is not running, calling this method has no effect.
     *
     * @param timestamp timestamp of gyroscope sample since epoch time and
     *                  expressed in nanoseconds.
     * @param data      angular speed of rotation along x,y,z axes expressed in
     *                  radians per second (rad/s).
     * @throws IllegalArgumentException if provided array does not have length
     *                                  3.
     */
    public void updateGyroscopeSample(final long timestamp, final float[] data) {
        if (slamEstimator != null) {
            slamEstimator.updateGyroscopeSample(timestamp, data);
        }
    }

    /**
     * Set ups calibration data on SLAM estimator if available.
     */
    protected void setUpCalibrationData() {
        D calibrationData = configuration.getCalibrationData();
        if (calibrationData != null) {
            slamEstimator.setCalibrationData(calibrationData);
        }
    }

    /**
     * Configures listener of SLAM estimator
     */
    protected void setUpSlamEstimatorListener() {
        slamEstimator.setListener(new BaseSlamEstimator.BaseSlamEstimatorListener<>() {
            @Override
            public void onFullSampleReceived(final BaseSlamEstimator<D> estimator) {
                // not used
            }

            @Override
            public void onFullSampleProcessed(final BaseSlamEstimator<D> estimator) {
                notifySlamStateAndCamera();
            }

            @Override
            public void onCorrectWithPositionMeasure(final BaseSlamEstimator<D> estimator) {
                // not used
            }

            @Override
            public void onCorrectedWithPositionMeasure(final BaseSlamEstimator<D> estimator) {
                notifySlamStateAndCamera();
            }

            private void notifySlamStateAndCamera() {
                notifySlamStateIfNeeded();
                notifySlamCameraIfNeeded();
            }
        });
    }

    /**
     * Update scene scale using SLAM data.
     *
     * @return true if scale was successfully updated, false otherwise.
     */
    protected boolean updateScale() {
        // obtain baseline (camera separation from slam estimator data
        final var posX = slamEstimator.getStatePositionX();
        final var posY = slamEstimator.getStatePositionY();
        final var posZ = slamEstimator.getStatePositionZ();

        // to estimate baseline, we assume that first camera is placed at
        // world origin
        final var baseline = Math.sqrt(posX * posX + posY * posY + posZ * posZ);

        try {
            final var camera1 = estimatedCamera1.getCamera();
            final var camera2 = estimatedCamera2.getCamera();

            camera1.decompose();
            camera2.decompose();

            final var center1 = camera1.getCameraCenter();
            final var center2 = camera2.getCameraCenter();

            final var estimatedBaseline = center1.distanceTo(center2);

            final var scale = baseline / estimatedBaseline;

            final var scaleTransformation = new MetricTransformation3D(scale);

            // update scale of cameras
            scaleTransformation.transform(camera1);
            scaleTransformation.transform(camera2);

            estimatedCamera1.setCamera(camera1);
            estimatedCamera2.setCamera(camera2);

            // update scale of reconstructed points
            final var numPoints = reconstructedPoints.size();
            final var reconstructedPoints3D = new ArrayList<Point3D>();
            for (final var reconstructedPoint : reconstructedPoints) {
                reconstructedPoints3D.add(reconstructedPoint.getPoint());
            }

            scaleTransformation.transformAndOverwritePoints(reconstructedPoints3D);

            // set scaled points into result
            for (var i = 0; i < numPoints; i++) {
                reconstructedPoints.get(i).setPoint(reconstructedPoints3D.get(i));
            }

            return true;
        } catch (final Exception e) {
            failed = true;
            //noinspection unchecked
            listener.onFail((R) this);

            return false;
        }
    }

    /**
     * Notifies SLAM state if notification is enabled at configuration time.
     */
    private void notifySlamStateIfNeeded() {
        if (!configuration.isNotifyAvailableSlamDataEnabled()) {
            return;
        }

        final var positionX = slamEstimator.getStatePositionX();
        final var positionY = slamEstimator.getStatePositionY();
        final var positionZ = slamEstimator.getStatePositionZ();

        final var velocityX = slamEstimator.getStateVelocityX();
        final var velocityY = slamEstimator.getStateVelocityY();
        final var velocityZ = slamEstimator.getStateVelocityZ();

        final var accelerationX = slamEstimator.getStateAccelerationX();
        final var accelerationY = slamEstimator.getStateAccelerationY();
        final var accelerationZ = slamEstimator.getStateAccelerationZ();

        final var quaternionA = slamEstimator.getStateQuaternionA();
        final var quaternionB = slamEstimator.getStateQuaternionB();
        final var quaternionC = slamEstimator.getStateQuaternionC();
        final var quaternionD = slamEstimator.getStateQuaternionD();

        final var angularSpeedX = slamEstimator.getStateAngularSpeedX();
        final var angularSpeedY = slamEstimator.getStateAngularSpeedY();
        final var angularSpeedZ = slamEstimator.getStateAngularSpeedZ();

        //noinspection unchecked
        listener.onSlamDataAvailable((R) this, positionX, positionY, positionZ,
                velocityX, velocityY, velocityZ, accelerationX, accelerationY, accelerationZ,
                quaternionA, quaternionB, quaternionC, quaternionD, angularSpeedX, angularSpeedY, angularSpeedZ,
                slamEstimator.getStateCovariance());
    }

    /**
     * Notifies estimated camera by means of SLAM if notification is enabled at
     * configuration time and intrinsics are already available.
     */
    private void notifySlamCameraIfNeeded() {
        if (!configuration.isNotifyEstimatedSlamCameraEnabled()) {
            return;
        }

        PinholeCameraIntrinsicParameters intrinsicParameters = null;
        if (configuration.getInitialIntrinsic1() != null) {
            intrinsicParameters = configuration.getInitialIntrinsic1();
        } else if (configuration.getInitialIntrinsic2() != null) {
            intrinsicParameters = configuration.getInitialIntrinsic2();
        }

        if (intrinsicParameters == null) {
            return;
        }

        final var positionX = slamEstimator.getStatePositionX();
        final var positionY = slamEstimator.getStatePositionY();
        final var positionZ = slamEstimator.getStatePositionZ();
        slamPosition.setInhomogeneousCoordinates(positionX, positionY, positionZ);

        final var quaternionA = slamEstimator.getStateQuaternionA();
        final var quaternionB = slamEstimator.getStateQuaternionB();
        final var quaternionC = slamEstimator.getStateQuaternionC();
        final var quaternionD = slamEstimator.getStateQuaternionD();
        slamRotation.setA(quaternionA);
        slamRotation.setB(quaternionB);
        slamRotation.setC(quaternionC);
        slamRotation.setD(quaternionD);

        slamCamera.setIntrinsicAndExtrinsicParameters(intrinsicParameters, slamRotation, slamPosition);

        //noinspection unchecked
        listener.onSlamCameraEstimated((R) this, slamCamera);
    }
}
