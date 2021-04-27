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
import java.util.List;

/**
 * Base class in charge of estimating cameras and 3D reconstructed points from
 * sparse image point correspondences in two views and also in charge of
 * estimating overall scene scale by means of SLAM (Simultaneous Location And
 * Mapping) using data obtained from sensors like accelerometers or gyroscopes.
 *
 * @param <D> type of calibration data.
 * @param <C> type of configuration.
 * @param <R> type of reconstructor.
 * @param <L> type of listener.
 * @param <S> type of SLAM estimator.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class BaseSlamTwoViewsSparseReconstructor<
        D extends BaseCalibrationData,
        C extends BaseSlamTwoViewsSparseReconstructorConfiguration<D, C>,
        R extends BaseSlamTwoViewsSparseReconstructor<D, C, R, L, S>,
        L extends BaseSlamTwoViewsSparseReconstructorListener<R>,
        S extends BaseSlamEstimator<D>> extends
        BaseTwoViewsSparseReconstructor<C, R, L> {

    /**
     * Slam estimator to estimate position, speed, orientation using
     * accelerometer and gyroscope data.
     */
    protected S mSlamEstimator;

    /**
     * Position estimated by means of SLAM. It is reused each time it is notified.
     */
    private final InhomogeneousPoint3D mSlamPosition = new InhomogeneousPoint3D();

    /**
     * Camera estimated by means of SLAM. It is reused each time it is notified.
     */
    private final PinholeCamera mSlamCamera = new PinholeCamera();

    /**
     * Camera rotation estimated by means of SLAM. It is reused each time it is notified.
     */
    private final Quaternion mSlamRotation = new Quaternion();

    /**
     * Constructor.
     *
     * @param configuration configuration for this reconstructor.
     * @param listener      listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     *                              provided.
     */
    protected BaseSlamTwoViewsSparseReconstructor(
            final C configuration, final L listener) {
        super(configuration, listener);
    }

    /**
     * Provides a new accelerometer sample to update SLAM estimation.
     * This method must be called whenever the accelerometer sensor receives new
     * data.
     * If reconstructor is not running, calling this method has no effect.
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
            final long timestamp, final float accelerationX,
            final float accelerationY, final float accelerationZ) {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateAccelerometerSample(timestamp, accelerationX,
                    accelerationY, accelerationZ);
        }
    }

    /**
     * Provides a new accelerometer sample to update SLAM estimation.
     * This method must be called whenever the accelerometer sensor receives new
     * data.
     * If reconstructor is not running, calling this method has no effect.
     *
     * @param timestamp timestamp of accelerometer sample since epoch time and
     *                  expressed in nanoseconds.
     * @param data      array containing x,y,z components of linear acceleration
     *                  expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided array does not have length
     *                                  3.
     */
    public void updateAccelerometerSample(
            final long timestamp, final float[] data) {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateAccelerometerSample(timestamp, data);
        }
    }

    /**
     * Provides a new gyroscope sample to update SLAM estimation.
     * If reconstructor is not running, calling this method has no effect.
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
            final long timestamp, final float angularSpeedX,
            final float angularSpeedY, final float angularSpeedZ) {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateGyroscopeSample(timestamp, angularSpeedX,
                    angularSpeedY, angularSpeedZ);
        }
    }

    /**
     * Provies a new gyroscope sample to update SLAM estimation.
     * If reconstructor is not running, calling this method has no effect.
     *
     * @param timestamp timestamp of gyroscope sample since epoch time and
     *                  expressed in nanoseconds.
     * @param data      angular speed of rotation along x,y,z axes expressed in
     *                  radians per second (rad/s).
     * @throws IllegalArgumentException if provided array does not have length
     *                                  3.
     */
    public void updateGyroscopeSample(final long timestamp, final float[] data) {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateGyroscopeSample(timestamp, data);
        }
    }

    /**
     * Set ups calibration data on SLAM estimator if available.
     */
    protected void setUpCalibrationData() {
        D calibrationData = mConfiguration.getCalibrationData();
        if (calibrationData != null) {
            mSlamEstimator.setCalibrationData(calibrationData);
        }
    }

    /**
     * Configures listener of SLAM estimator
     */
    protected void setUpSlamEstimatorListener() {
        mSlamEstimator.setListener(new BaseSlamEstimator.BaseSlamEstimatorListener<D>() {
            @Override
            public void onFullSampleReceived(final BaseSlamEstimator<D> estimator) {
                // not used
            }

            @Override
            public void onFullSampleProcessed(final BaseSlamEstimator<D> estimator) {
                notifySlamStateIfNeeded();
                notifySlamCameraIfNeeded();
            }

            @Override
            public void onCorrectWithPositionMeasure(final BaseSlamEstimator<D> estimator) {
                // not used
            }

            @Override
            public void onCorrectedWithPositionMeasure(final BaseSlamEstimator<D> estimator) {
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
        final double posX = mSlamEstimator.getStatePositionX();
        final double posY = mSlamEstimator.getStatePositionY();
        final double posZ = mSlamEstimator.getStatePositionZ();

        // to estimate baseline, we assume that first camera is placed at
        // world origin
        final double baseline = Math.sqrt(posX * posX + posY * posY + posZ * posZ);

        try {
            final PinholeCamera camera1 = mEstimatedCamera1.getCamera();
            final PinholeCamera camera2 = mEstimatedCamera2.getCamera();

            camera1.decompose();
            camera2.decompose();

            final Point3D center1 = camera1.getCameraCenter();
            final Point3D center2 = camera2.getCameraCenter();

            final double estimatedBaseline = center1.distanceTo(center2);

            final double scale = baseline / estimatedBaseline;

            final MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);

            // update scale of cameras
            scaleTransformation.transform(camera1);
            scaleTransformation.transform(camera2);

            mEstimatedCamera1.setCamera(camera1);
            mEstimatedCamera2.setCamera(camera2);

            // update scale of reconstructed points
            final int numPoints = mReconstructedPoints.size();
            final List<Point3D> reconstructedPoints3D = new ArrayList<>();
            for (final ReconstructedPoint3D reconstructedPoint : mReconstructedPoints) {
                reconstructedPoints3D.add(reconstructedPoint.
                        getPoint());
            }

            scaleTransformation.transformAndOverwritePoints(
                    reconstructedPoints3D);

            // set scaled points into result
            for (int i = 0; i < numPoints; i++) {
                mReconstructedPoints.get(i).setPoint(
                        reconstructedPoints3D.get(i));
            }

            return true;
        } catch (final Exception e) {
            mFailed = true;
            //noinspection unchecked
            mListener.onFail((R) this);

            return false;
        }
    }

    /**
     * Notifies SLAM state if notification is enabled at configuration time.
     */
    private void notifySlamStateIfNeeded() {
        if (!mConfiguration.isNotifyAvailableSlamDataEnabled()) {
            return;
        }

        final double positionX = mSlamEstimator.getStatePositionX();
        final double positionY = mSlamEstimator.getStatePositionY();
        final double positionZ = mSlamEstimator.getStatePositionZ();

        final double velocityX = mSlamEstimator.getStateVelocityX();
        final double velocityY = mSlamEstimator.getStateVelocityY();
        final double velocityZ = mSlamEstimator.getStateVelocityZ();

        final double accelerationX = mSlamEstimator.getStateAccelerationX();
        final double accelerationY = mSlamEstimator.getStateAccelerationY();
        final double accelerationZ = mSlamEstimator.getStateAccelerationZ();

        final double quaternionA = mSlamEstimator.getStateQuaternionA();
        final double quaternionB = mSlamEstimator.getStateQuaternionB();
        final double quaternionC = mSlamEstimator.getStateQuaternionC();
        final double quaternionD = mSlamEstimator.getStateQuaternionD();

        final double angularSpeedX = mSlamEstimator.getStateAngularSpeedX();
        final double angularSpeedY = mSlamEstimator.getStateAngularSpeedY();
        final double angularSpeedZ = mSlamEstimator.getStateAngularSpeedZ();

        //noinspection unchecked
        mListener.onSlamDataAvailable((R) this, positionX, positionY, positionZ,
                velocityX, velocityY, velocityZ,
                accelerationX, accelerationY, accelerationZ,
                quaternionA, quaternionB, quaternionC, quaternionD,
                angularSpeedX, angularSpeedY, angularSpeedZ, mSlamEstimator.getStateCovariance());
    }

    /**
     * Notifies estimated camera by means of SLAM if notification is enabled at
     * configuration time and intrinsics are already available.
     */
    private void notifySlamCameraIfNeeded() {
        if (!mConfiguration.isNotifyEstimatedSlamCameraEnabled()) {
            return;
        }

        PinholeCameraIntrinsicParameters intrinsicParameters = null;
        if (mConfiguration.getInitialIntrinsic1() != null) {
            intrinsicParameters = mConfiguration.getInitialIntrinsic1();
        } else if (mConfiguration.getInitialIntrinsic2() != null) {
            intrinsicParameters = mConfiguration.getInitialIntrinsic2();
        }

        if (intrinsicParameters == null) {
            return;
        }

        final double positionX = mSlamEstimator.getStatePositionX();
        final double positionY = mSlamEstimator.getStatePositionY();
        final double positionZ = mSlamEstimator.getStatePositionZ();
        mSlamPosition.setInhomogeneousCoordinates(positionX, positionY, positionZ);

        final double quaternionA = mSlamEstimator.getStateQuaternionA();
        final double quaternionB = mSlamEstimator.getStateQuaternionB();
        final double quaternionC = mSlamEstimator.getStateQuaternionC();
        final double quaternionD = mSlamEstimator.getStateQuaternionD();
        mSlamRotation.setA(quaternionA);
        mSlamRotation.setB(quaternionB);
        mSlamRotation.setC(quaternionC);
        mSlamRotation.setD(quaternionD);

        mSlamCamera.setIntrinsicAndExtrinsicParameters(intrinsicParameters, mSlamRotation,
                mSlamPosition);

        //noinspection unchecked
        mListener.onSlamCameraEstimated((R) this, mSlamCamera);
    }
}
