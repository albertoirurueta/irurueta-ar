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
import com.irurueta.geometry.GeometryException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MetricTransformation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;

import java.util.ArrayList;

/**
 * Base class in charge of estimating cameras and 3D reconstructed points from sparse
 * image point correspondences in multiple views and also in charge of estimating overall
 * scene scale by means of SLAM (Simultaneous Location And Mapping) using data obtained
 * from sensors like accelerometers or gyroscopes.
 *
 * @param <D> type defining calibration data.
 * @param <C> type of configuration.
 * @param <R> type of re-constructor.
 * @param <L> type of listener.
 * @param <S> type of SLAM estimator.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class BaseSlamSparseReconstructor<
        D extends BaseCalibrationData,
        C extends BaseSlamSparseReconstructorConfiguration<D, C>,
        R extends BaseSlamSparseReconstructor<D, C, R, L, S>,
        L extends BaseSlamSparseReconstructorListener<R>,
        S extends BaseSlamEstimator<D>> extends BaseSparseReconstructor<C, R, L> {

    /**
     * Slam estimator to estimate position, speed, orientation using
     * accelerometer and gyroscope data.
     */
    protected S slamEstimator;

    /**
     * Position estimated by means of SLAM. It is reused each time it is notified.
     */
    protected final InhomogeneousPoint3D slamPosition = new InhomogeneousPoint3D();

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
    protected BaseSlamSparseReconstructor(final C configuration, final L listener) {
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
    public void updateAccelerometerSample(final long timestamp, final float accelerationX, final float accelerationY,
                                          final float accelerationZ) {
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
    public void updateGyroscopeSample(final long timestamp, final float angularSpeedX,
                                      final float angularSpeedY, final float angularSpeedZ) {
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
     * Configures calibration data on SLAM estimator if available.
     */
    protected void setUpCalibrationData() {
        final var calibrationData = configuration.getCalibrationData();
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
     * @param isInitialPairOfViews true if initial pair of views is being processed, false otherwise.
     * @return true if scale was successfully updated, false otherwise.
     */
    protected boolean updateScale(final boolean isInitialPairOfViews) {
        try {
            var metricCamera1 = previousMetricEstimatedCamera.getCamera();
            var metricCamera2 = currentMetricEstimatedCamera.getCamera();

            double slamPosX;
            double slamPosY;
            double slamPosZ;
            double scale;
            if (isInitialPairOfViews) {
                // obtain baseline (camera separation from slam estimator data
                slamPosX = slamEstimator.getStatePositionX();
                slamPosY = slamEstimator.getStatePositionY();
                slamPosZ = slamEstimator.getStatePositionZ();

                slamPosition.setInhomogeneousCoordinates(slamPosX, slamPosY, slamPosZ);

                if (!metricCamera1.isCameraCenterAvailable()) {
                    metricCamera1.decompose(false, true);
                }
                if (!metricCamera2.isCameraCenterAvailable()) {
                    metricCamera2.decompose(false, true);
                }

                final var center1 = metricCamera1.getCameraCenter();
                final var center2 = metricCamera2.getCameraCenter();

                final var baseline = center1.distanceTo(slamPosition);
                final var estimatedBaseline = center1.distanceTo(center2);

                scale = currentScale = baseline / estimatedBaseline;
            } else {
                scale = currentScale;
            }

            final var scaleTransformation = new MetricTransformation3D(scale);

            // update scale of cameras
            final var euclideanCamera1 = scaleTransformation.transformAndReturnNew(metricCamera1);
            final var euclideanCamera2 = scaleTransformation.transformAndReturnNew(metricCamera2);

            if (!euclideanCamera2.isCameraCenterAvailable()) {
                euclideanCamera2.decompose(false, true);
            }
            slamEstimator.correctWithPositionMeasure(euclideanCamera2.getCameraCenter(),
                    configuration.getCameraPositionCovariance());

            if (!isInitialPairOfViews) {

                slamPosX = slamEstimator.getStatePositionX();
                slamPosY = slamEstimator.getStatePositionY();
                slamPosZ = slamEstimator.getStatePositionZ();
                slamPosition.setInhomogeneousCoordinates(slamPosX, slamPosY, slamPosZ);

                // adjust scale of current camera
                final var euclideanCenter2 = euclideanCamera2.getCameraCenter();

                final var euclideanPosX = euclideanCenter2.getInhomX();
                final var euclideanPosY = euclideanCenter2.getInhomY();
                final var euclideanPosZ = euclideanCenter2.getInhomZ();

                final var scaleVariationX = euclideanPosX / slamPosX;
                final var scaleVariationY = euclideanPosY / slamPosY;
                final var scaleVariationZ = euclideanPosZ / slamPosZ;

                final var scaleVariation = (scaleVariationX + scaleVariationY + scaleVariationZ) / 3.0;
                scale *= scaleVariation;
                currentScale = scale;
                scaleTransformation.setScale(currentScale);

                // update camera
                scaleTransformation.transform(metricCamera2, euclideanCamera2);
            }
            final var sqrScale = scale * scale;

            previousEuclideanEstimatedCamera = new EstimatedCamera();
            previousEuclideanEstimatedCamera.setCamera(euclideanCamera1);
            previousEuclideanEstimatedCamera.setViewId(previousMetricEstimatedCamera.getViewId());
            previousEuclideanEstimatedCamera.setQualityScore(previousMetricEstimatedCamera.getQualityScore());
            if (previousMetricEstimatedCamera.getCovariance() != null) {
                previousEuclideanEstimatedCamera.setCovariance(
                        previousMetricEstimatedCamera.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
            }

            currentEuclideanEstimatedCamera = new EstimatedCamera();
            currentEuclideanEstimatedCamera.setCamera(euclideanCamera2);
            currentEuclideanEstimatedCamera.setViewId(currentMetricEstimatedCamera.getViewId());
            currentEuclideanEstimatedCamera.setQualityScore(currentMetricEstimatedCamera.getQualityScore());
            if (currentMetricEstimatedCamera.getCovariance() != null) {
                currentEuclideanEstimatedCamera.setCovariance(
                        currentMetricEstimatedCamera.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
            }

            // update scale of reconstructed points
            final var numPoints = activeMetricReconstructedPoints.size();
            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            for (final var reconstructedPoint : activeMetricReconstructedPoints) {
                metricReconstructedPoints3D.add(reconstructedPoint.getPoint());
            }

            final var euclideanReconstructedPoints3D = scaleTransformation.transformPointsAndReturnNew(
                    metricReconstructedPoints3D);

            // set scaled points into result
            activeEuclideanReconstructedPoints = new ArrayList<>();
            ReconstructedPoint3D euclideanPoint;
            ReconstructedPoint3D metricPoint;
            for (var i = 0; i < numPoints; i++) {
                metricPoint = activeMetricReconstructedPoints.get(i);

                euclideanPoint = new ReconstructedPoint3D();
                euclideanPoint.setId(metricPoint.getId());
                euclideanPoint.setPoint(euclideanReconstructedPoints3D.get(i));
                euclideanPoint.setInlier(metricPoint.isInlier());
                euclideanPoint.setQualityScore(metricPoint.getQualityScore());
                if (metricPoint.getCovariance() != null) {
                    euclideanPoint.setCovariance(metricPoint.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
                }
                euclideanPoint.setColorData(metricPoint.getColorData());

                activeEuclideanReconstructedPoints.add(euclideanPoint);
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
                velocityX, velocityY, velocityZ,
                accelerationX, accelerationY, accelerationZ,
                quaternionA, quaternionB, quaternionC, quaternionD,
                angularSpeedX, angularSpeedY, angularSpeedZ, slamEstimator.getStateCovariance());
    }

    /**
     * Notifies estimated camera by means of SLAM if notification is enabled at
     * configuration time and intrinsics are already available.
     */
    private void notifySlamCameraIfNeeded() {
        if (!configuration.isNotifyEstimatedSlamCameraEnabled()) {
            return;
        }

        // try with current camera
        var camera = currentEuclideanEstimatedCamera != null ? currentEuclideanEstimatedCamera.getCamera() : null;
        if (camera == null) {
            // if not available try with previous camera
            camera = previousEuclideanEstimatedCamera != null ? previousEuclideanEstimatedCamera.getCamera() : null;
        }

        try {
            PinholeCameraIntrinsicParameters intrinsicParameters = null;
            if (camera != null) {
                if (!camera.areIntrinsicParametersAvailable()) {
                    // decompose camera to obtain intrinsic parameters
                    camera.decompose();
                }

                intrinsicParameters = camera.getIntrinsicParameters();
            } else if (configuration.getInitialIntrinsic1() != null) {
                intrinsicParameters = configuration.getInitialIntrinsic1();
            } else if (configuration.getInitialIntrinsic2() != null) {
                intrinsicParameters = configuration.getInitialIntrinsic2();
            } else if (configuration.getAdditionalCamerasIntrinsics() != null) {
                intrinsicParameters = configuration.getAdditionalCamerasIntrinsics();
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

        } catch (final GeometryException ignore) {
            // do nothing
        }
    }
}
