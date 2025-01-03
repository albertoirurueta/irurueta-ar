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
import com.irurueta.ar.slam.BaseCalibrationData;
import com.irurueta.ar.slam.BaseSlamEstimator;
import com.irurueta.geometry.GeometryException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MetricTransformation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;

import java.util.ArrayList;

@SuppressWarnings("DuplicatedCode")
public abstract class BaseSlamPairedViewsSparseReconstructor<
        D extends BaseCalibrationData,
        C extends BaseSlamPairedViewsSparseReconstructorConfiguration<D, C>,
        R extends BaseSlamPairedViewsSparseReconstructor<D, C, R, L, S>,
        L extends BaseSlamPairedViewsSparseReconstructorListener<R>,
        S extends BaseSlamEstimator<D>> extends BasePairedViewsSparseReconstructor<C, R, L> {

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
     * Inverse Euclidean camera rotation. This is reused for memory efficiency.
     */
    private Rotation3D invEuclideanCameraRotation;

    /**
     * Camera estimated by means of SLAM. It is reused each time it is notified.
     */
    private final PinholeCamera slamCamera = new PinholeCamera();

    /**
     * Camera rotation estimated by means of SLAM. It is reused each time it is notified.
     */
    private final Quaternion slamRotation = new Quaternion();

    /**
     * Last SLAM timestamp.
     */
    private long lastTimestamp = -1;

    /**
     * Last view pair SLAM timestamp.
     */
    private long lastViewPairTimestamp = -1;

    /**
     * Constructor.
     *
     * @param configuration configuration for this re-constructor.
     * @param listener      listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     *                              provided.
     */
    protected BaseSlamPairedViewsSparseReconstructor(final C configuration, final L listener) {
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
    public void updateAccelerometerSample(final long timestamp, final float accelerationX,
                                          final float accelerationY, final float accelerationZ) {
        if (lastViewPairTimestamp < 0) {
            lastViewPairTimestamp = timestamp;
        }

        if (slamEstimator != null) {
            slamEstimator.updateAccelerometerSample(timestamp - lastViewPairTimestamp,
                    accelerationX, accelerationY, accelerationZ);
        }
        lastTimestamp = timestamp;
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
        if (lastViewPairTimestamp < 0) {
            lastViewPairTimestamp = timestamp;
        }

        if (slamEstimator != null) {
            slamEstimator.updateAccelerometerSample(timestamp - lastViewPairTimestamp, data);
        }
        lastTimestamp = timestamp;
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
        if (lastViewPairTimestamp < 0) {
            lastViewPairTimestamp = timestamp;
        }

        if (slamEstimator != null) {
            slamEstimator.updateGyroscopeSample(timestamp - lastViewPairTimestamp,
                    angularSpeedX, angularSpeedY, angularSpeedZ);
        }
        lastTimestamp = timestamp;
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
        if (lastViewPairTimestamp < 0) {
            lastViewPairTimestamp = timestamp;
        }

        if (slamEstimator != null) {
            slamEstimator.updateGyroscopeSample(timestamp - lastViewPairTimestamp, data);
        }
        lastTimestamp = timestamp;
    }

    /**
     * Resets this instance so that a reconstruction can be started from the beginning without
     * cancelling current one.
     */
    @Override
    public void reset() {
        super.reset();
        lastTimestamp = lastViewPairTimestamp = -1;
    }

    /**
     * Indicates whether implementations of a re-constructor uses absolute orientation or
     * not.
     *
     * @return true if absolute orientation is used, false, otherwise.
     */
    @Override
    protected boolean hasAbsoluteOrientation() {
        return false;
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
     * Transforms metric cameras on current pair of views so that they are referred to
     * last kept location and rotation and upgrades cameras from metric stratum to
     * Euclidean stratum.
     *
     * @param isInitialPairOfViews   true if initial pair of views is being processed, false otherwise.
     * @param hasAbsoluteOrientation true if absolute orientation is required, false otherwise.
     * @return true if cameras were successfully transformed.
     */
    @Override
    protected boolean transformPairOfCamerasAndPoints(final boolean isInitialPairOfViews,
                                                      final boolean hasAbsoluteOrientation) {
        final var previousMetricCamera = previousMetricEstimatedCamera.getCamera();
        final var currentMetricCamera = currentMetricEstimatedCamera.getCamera();
        if (previousMetricCamera == null || currentMetricCamera == null) {
            return false;
        }

        currentScale = estimateCurrentScale();
        final var sqrScale = currentScale * currentScale;

        referenceEuclideanTransformation = new MetricTransformation3D(currentScale);
        if (hasAbsoluteOrientation) {
            invEuclideanCameraRotation = lastEuclideanCameraRotation.inverseRotationAndReturnNew();
            if (isInitialPairOfViews) {
                referenceEuclideanTransformation.setRotation(invEuclideanCameraRotation);
            }
        }

        if (!isInitialPairOfViews) {
            // additional pairs also need to translate and rotate
            if (invEuclideanCameraRotation == null) {
                invEuclideanCameraRotation = lastEuclideanCameraRotation.inverseRotationAndReturnNew();
            } else {
                lastEuclideanCameraRotation.inverseRotation(invEuclideanCameraRotation);
            }
            referenceEuclideanTransformation.setRotation(invEuclideanCameraRotation);
            referenceEuclideanTransformation.setTranslation(lastEuclideanCameraCenter);
        }

        try {
            // transform cameras
            final var previousEuclideanCamera = referenceEuclideanTransformation.transformAndReturnNew(
                    previousMetricCamera);
            final var currentEuclideanCamera = referenceEuclideanTransformation.transformAndReturnNew(
                    currentMetricCamera);

            previousEuclideanEstimatedCamera = new EstimatedCamera();
            previousEuclideanEstimatedCamera.setCamera(previousEuclideanCamera);
            previousEuclideanEstimatedCamera.setViewId(previousMetricEstimatedCamera.getViewId());
            previousEuclideanEstimatedCamera.setQualityScore(previousMetricEstimatedCamera.getQualityScore());
            if (previousMetricEstimatedCamera.getCovariance() != null) {
                previousEuclideanEstimatedCamera.setCovariance(previousMetricEstimatedCamera.getCovariance()
                        .multiplyByScalarAndReturnNew(sqrScale));
            }

            currentEuclideanEstimatedCamera = new EstimatedCamera();
            currentEuclideanEstimatedCamera.setCamera(currentEuclideanCamera);
            currentEuclideanEstimatedCamera.setViewId(currentMetricEstimatedCamera.getViewId());
            currentEuclideanEstimatedCamera.setQualityScore(currentMetricEstimatedCamera.getQualityScore());
            if (currentMetricEstimatedCamera.getCovariance() != null) {
                currentEuclideanEstimatedCamera.setCovariance(currentMetricEstimatedCamera.getCovariance()
                        .multiplyByScalarAndReturnNew(sqrScale));
            }

            // transform points
            euclideanReconstructedPoints = new ArrayList<>();
            ReconstructedPoint3D euclideanReconstructedPoint;
            for (final ReconstructedPoint3D metricReconstructedPoint : metricReconstructedPoints) {
                final var metricPoint = metricReconstructedPoint.getPoint();
                final var euclideanPoint = referenceEuclideanTransformation.transformAndReturnNew(metricPoint);
                euclideanReconstructedPoint = new ReconstructedPoint3D();
                euclideanReconstructedPoint.setPoint(euclideanPoint);
                euclideanReconstructedPoint.setInlier(metricReconstructedPoint.isInlier());
                euclideanReconstructedPoint.setId(metricReconstructedPoint.getId());
                euclideanReconstructedPoint.setColorData(metricReconstructedPoint.getColorData());
                if (metricReconstructedPoint.getCovariance() != null) {
                    euclideanReconstructedPoint.setCovariance(metricReconstructedPoint.getCovariance()
                            .multiplyByScalarAndReturnNew(sqrScale));
                }
                euclideanReconstructedPoint.setQualityScore(metricReconstructedPoint.getQualityScore());
                euclideanReconstructedPoints.add(euclideanReconstructedPoint);
            }

        } catch (final AlgebraException e) {
            return false;
        }

        return super.transformPairOfCamerasAndPoints(isInitialPairOfViews, hasAbsoluteOrientation);
    }

    /**
     * Estimates current scale using SLAM data.
     *
     * @return estimated scale.
     */
    private double estimateCurrentScale() {
        try {
            final var metricCamera1 = previousMetricEstimatedCamera.getCamera();
            final var metricCamera2 = currentMetricEstimatedCamera.getCamera();

            if (!metricCamera1.isCameraCenterAvailable()) {
                metricCamera1.decompose(false, true);
            }
            if (!metricCamera2.isCameraCenterAvailable()) {
                metricCamera2.decompose(false, true);
            }

            final var metricCenter1 = metricCamera1.getCameraCenter();
            final var metricCenter2 = metricCamera2.getCameraCenter();

            // obtain baseline (camera separation from slam estimator data)
            final var slamPosX = slamEstimator.getStatePositionX();
            final var slamPosY = slamEstimator.getStatePositionY();
            final var slamPosZ = slamEstimator.getStatePositionZ();

            slamPosition.setInhomogeneousCoordinates(slamPosX, slamPosY, slamPosZ);

            // we assume that Euclidean center of 1st camera is at origin because by the time
            // this method is called, metric cameras have not yet been orientation and position
            // transformed
            final var euclideanBaseline = Math.sqrt(slamPosX * slamPosX + slamPosY * slamPosY + slamPosZ * slamPosZ);
            final var metricBaseline = metricCenter1.distanceTo(metricCenter2);

            // because when we call reset in SLAM estimator, timestamp is lost, we keep
            // track of last timestamp to be subtracted on subsequent update calls
            lastViewPairTimestamp = lastTimestamp;

            // reset linear velocity and orientation and keep other slam state parameters
            slamEstimator.resetPositionAndVelocity();

            return euclideanBaseline / metricBaseline;
        } catch (final Exception e) {
            failed = true;
            //noinspection unchecked
            listener.onFail((R) this);
            return DEFAULT_SCALE;
        }
    }

    /**
     * Notifies SLAM state if notification is enabled at configuration time.
     */
    private void notifySlamStateIfNeeded() {
        if (!configuration.isNotifyAvailableSlamDataEnabled()) {
            return;
        }

        final var lastPosX = lastEuclideanCameraCenter != null ? lastEuclideanCameraCenter.getInhomX() : 0.0;
        final var lastPosY = lastEuclideanCameraCenter != null ? lastEuclideanCameraCenter.getInhomY() : 0.0;
        final var lastPosZ = lastEuclideanCameraCenter != null ? lastEuclideanCameraCenter.getInhomZ() : 0.0;

        final var positionX = lastPosX + slamEstimator.getStatePositionX();
        final var positionY = lastPosY + slamEstimator.getStatePositionY();
        final var positionZ = lastPosZ + slamEstimator.getStatePositionZ();

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
        listener.onSlamDataAvailable((R) this, positionX, positionY, positionZ, velocityX, velocityY, velocityZ,
                accelerationX, accelerationY, accelerationZ, quaternionA, quaternionB, quaternionC, quaternionD,
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
            } else if (configuration.areIntrinsicParametersKnown()) {
                //noinspection unchecked
                intrinsicParameters = listener.onIntrinsicParametersRequested((R) this, currentViewId);
            }

            if (intrinsicParameters == null) {
                return;
            }

            final var lastPosX = lastEuclideanCameraCenter != null ? lastEuclideanCameraCenter.getInhomX() : 0.0;
            final var lastPosY = lastEuclideanCameraCenter != null ? lastEuclideanCameraCenter.getInhomY() : 0.0;
            final var lastPosZ = lastEuclideanCameraCenter != null ? lastEuclideanCameraCenter.getInhomZ() : 0.0;

            final var positionX = lastPosX + slamEstimator.getStatePositionX();
            final var positionY = lastPosY + slamEstimator.getStatePositionY();
            final var positionZ = lastPosZ + slamEstimator.getStatePositionZ();
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
