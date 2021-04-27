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
import com.irurueta.geometry.Point3D;
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
    protected S mSlamEstimator;

    /**
     * Position estimated by means of SLAM. It is reused each time it is notified.
     */
    private final InhomogeneousPoint3D mSlamPosition = new InhomogeneousPoint3D();

    /**
     * Inverse euclidean camera rotation. This is reused for memory efficiency.
     */
    private Rotation3D mInvEuclideanCameraRotation;

    /**
     * Camera estimated by means of SLAM. It is reused each time it is notified.
     */
    private final PinholeCamera mSlamCamera = new PinholeCamera();

    /**
     * Camera rotation estimated by means of SLAM. It is reused each time it is notified.
     */
    private final Quaternion mSlamRotation = new Quaternion();

    /**
     * Last SLAM timestamp.
     */
    private long mLastTimestamp = -1;

    /**
     * Last view pair SLAM timestamp.
     */
    private long mLastViewPairTimestamp = -1;

    /**
     * Constructor.
     *
     * @param configuration configuration for this reconstructor.
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
    public void updateAccelerometerSample(final long timestamp, final float accelerationX,
                                          final float accelerationY, final float accelerationZ) {
        if (mLastViewPairTimestamp < 0) {
            mLastViewPairTimestamp = timestamp;
        }

        if (mSlamEstimator != null) {
            mSlamEstimator.updateAccelerometerSample(timestamp - mLastViewPairTimestamp,
                    accelerationX, accelerationY, accelerationZ);
        }
        mLastTimestamp = timestamp;
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
    public void updateAccelerometerSample(final long timestamp, final float[] data) {
        if (mLastViewPairTimestamp < 0) {
            mLastViewPairTimestamp = timestamp;
        }

        if (mSlamEstimator != null) {
            mSlamEstimator.updateAccelerometerSample(timestamp - mLastViewPairTimestamp,
                    data);
        }
        mLastTimestamp = timestamp;
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
    public void updateGyroscopeSample(final long timestamp, final float angularSpeedX,
                                      final float angularSpeedY, final float angularSpeedZ) {
        if (mLastViewPairTimestamp < 0) {
            mLastViewPairTimestamp = timestamp;
        }

        if (mSlamEstimator != null) {
            mSlamEstimator.updateGyroscopeSample(timestamp - mLastViewPairTimestamp,
                    angularSpeedX, angularSpeedY, angularSpeedZ);
        }
        mLastTimestamp = timestamp;
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
        if (mLastViewPairTimestamp < 0) {
            mLastViewPairTimestamp = timestamp;
        }

        if (mSlamEstimator != null) {
            mSlamEstimator.updateGyroscopeSample(timestamp - mLastViewPairTimestamp,
                    data);
        }
        mLastTimestamp = timestamp;
    }

    /**
     * Resets this instance so that a reconstruction can be started from the beginning without cancelling current one.
     */
    @Override
    public void reset() {
        super.reset();
        mLastTimestamp = mLastViewPairTimestamp = -1;
    }

    /**
     * Indicates whether implementations of a reconstructor uses absolute orientation or
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
        final D calibrationData = mConfiguration.getCalibrationData();
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
     * euclidean stratum.
     *
     * @param isInitialPairOfViews   true if initial pair of views is being processed, false otherwise.
     * @param hasAbsoluteOrientation true if absolute orientation is required, false otherwise.
     * @return true if cameras were successfully transformed.
     */
    @Override
    protected boolean transformPairOfCamerasAndPoints(final boolean isInitialPairOfViews,
                                                      final boolean hasAbsoluteOrientation) {
        final PinholeCamera previousMetricCamera = mPreviousMetricEstimatedCamera.getCamera();
        final PinholeCamera currentMetricCamera = mCurrentMetricEstimatedCamera.getCamera();
        if (previousMetricCamera == null || currentMetricCamera == null) {
            return false;
        }

        mCurrentScale = estimateCurrentScale();
        final double sqrScale = mCurrentScale * mCurrentScale;

        mReferenceEuclideanTransformation = new MetricTransformation3D(mCurrentScale);
        if (hasAbsoluteOrientation) {
            mInvEuclideanCameraRotation = mLastEuclideanCameraRotation.inverseRotationAndReturnNew();
            if (isInitialPairOfViews) {
                mReferenceEuclideanTransformation.setRotation(mInvEuclideanCameraRotation);
            }
        }

        if (!isInitialPairOfViews) {
            // additional pairs also need to translate and rotate
            if (mInvEuclideanCameraRotation == null) {
                mInvEuclideanCameraRotation = mLastEuclideanCameraRotation.inverseRotationAndReturnNew();
            } else {
                mLastEuclideanCameraRotation.inverseRotation(mInvEuclideanCameraRotation);
            }
            mReferenceEuclideanTransformation.setRotation(mInvEuclideanCameraRotation);
            mReferenceEuclideanTransformation.setTranslation(mLastEuclideanCameraCenter);
        }

        try {
            // transform cameras
            final PinholeCamera previousEuclideanCamera = mReferenceEuclideanTransformation.
                    transformAndReturnNew(previousMetricCamera);
            final PinholeCamera currentEuclideanCamera = mReferenceEuclideanTransformation.
                    transformAndReturnNew(currentMetricCamera);

            mPreviousEuclideanEstimatedCamera = new EstimatedCamera();
            mPreviousEuclideanEstimatedCamera.setCamera(previousEuclideanCamera);
            mPreviousEuclideanEstimatedCamera.setViewId(
                    mPreviousMetricEstimatedCamera.getViewId());
            mPreviousEuclideanEstimatedCamera.setQualityScore(
                    mPreviousMetricEstimatedCamera.getQualityScore());
            if (mPreviousMetricEstimatedCamera.getCovariance() != null) {
                mPreviousEuclideanEstimatedCamera.setCovariance(
                        mPreviousMetricEstimatedCamera.getCovariance().
                                multiplyByScalarAndReturnNew(sqrScale));
            }

            mCurrentEuclideanEstimatedCamera = new EstimatedCamera();
            mCurrentEuclideanEstimatedCamera.setCamera(currentEuclideanCamera);
            mCurrentEuclideanEstimatedCamera.setViewId(
                    mCurrentMetricEstimatedCamera.getViewId());
            mCurrentEuclideanEstimatedCamera.setQualityScore(
                    mCurrentMetricEstimatedCamera.getQualityScore());
            if (mCurrentMetricEstimatedCamera.getCovariance() != null) {
                mCurrentEuclideanEstimatedCamera.setCovariance(
                        mCurrentMetricEstimatedCamera.getCovariance().
                                multiplyByScalarAndReturnNew(sqrScale));
            }

            // transform points
            mEuclideanReconstructedPoints = new ArrayList<>();
            ReconstructedPoint3D euclideanReconstructedPoint;
            Point3D metricPoint;
            Point3D euclideanPoint;
            for (final ReconstructedPoint3D metricReconstructedPoint : mMetricReconstructedPoints) {
                metricPoint = metricReconstructedPoint.getPoint();
                euclideanPoint = mReferenceEuclideanTransformation.transformAndReturnNew(
                        metricPoint);
                euclideanReconstructedPoint = new ReconstructedPoint3D();
                euclideanReconstructedPoint.setPoint(euclideanPoint);
                euclideanReconstructedPoint.setInlier(metricReconstructedPoint.isInlier());
                euclideanReconstructedPoint.setId(metricReconstructedPoint.getId());
                euclideanReconstructedPoint.setColorData(
                        metricReconstructedPoint.getColorData());
                if (metricReconstructedPoint.getCovariance() != null) {
                    euclideanReconstructedPoint.setCovariance(
                            metricReconstructedPoint.getCovariance().
                                    multiplyByScalarAndReturnNew(sqrScale));
                }
                euclideanReconstructedPoint.setQualityScore(
                        metricReconstructedPoint.getQualityScore());
                mEuclideanReconstructedPoints.add(euclideanReconstructedPoint);
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
            final PinholeCamera metricCamera1 = mPreviousMetricEstimatedCamera.getCamera();
            final PinholeCamera metricCamera2 = mCurrentMetricEstimatedCamera.getCamera();

            if (!metricCamera1.isCameraCenterAvailable()) {
                metricCamera1.decompose(false, true);
            }
            if (!metricCamera2.isCameraCenterAvailable()) {
                metricCamera2.decompose(false, true);
            }

            final Point3D metricCenter1 = metricCamera1.getCameraCenter();
            final Point3D metricCenter2 = metricCamera2.getCameraCenter();

            // obtain baseline (camera separation from slam estimator data)
            final double slamPosX = mSlamEstimator.getStatePositionX();
            final double slamPosY = mSlamEstimator.getStatePositionY();
            final double slamPosZ = mSlamEstimator.getStatePositionZ();

            mSlamPosition.setInhomogeneousCoordinates(slamPosX, slamPosY, slamPosZ);

            // we assume that euclidean center of 1st camera is at origin because by the time
            // this method is called, metric cameras have not yet been orientation and position
            // transformed
            final double euclideanBaseline = Math.sqrt(
                    slamPosX * slamPosX +
                            slamPosY * slamPosY +
                            slamPosZ * slamPosZ);
            final double metricBaseline = metricCenter1.distanceTo(metricCenter2);

            // because when we call reset in SLAM estimator, timestamp is lost, we keep
            // track of last timestamp to be subtracted on subsequent update calls
            mLastViewPairTimestamp = mLastTimestamp;

            // reset linear velocity and orientation and keep other slam state parameters
            mSlamEstimator.resetPositionAndVelocity();

            return euclideanBaseline / metricBaseline;
        } catch (final Exception e) {
            mFailed = true;
            //noinspection unchecked
            mListener.onFail((R) this);
            return DEFAULT_SCALE;
        }
    }

    /**
     * Notifies SLAM state if notification is enabled at configuration time.
     */
    private void notifySlamStateIfNeeded() {
        if (!mConfiguration.isNotifyAvailableSlamDataEnabled()) {
            return;
        }

        final double lastPosX = mLastEuclideanCameraCenter != null ?
                mLastEuclideanCameraCenter.getInhomX() : 0.0;
        final double lastPosY = mLastEuclideanCameraCenter != null ?
                mLastEuclideanCameraCenter.getInhomY() : 0.0;
        final double lastPosZ = mLastEuclideanCameraCenter != null ?
                mLastEuclideanCameraCenter.getInhomZ() : 0.0;

        final double positionX = lastPosX + mSlamEstimator.getStatePositionX();
        final double positionY = lastPosY + mSlamEstimator.getStatePositionY();
        final double positionZ = lastPosZ + mSlamEstimator.getStatePositionZ();

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

        // try with current camera
        PinholeCamera camera = mCurrentEuclideanEstimatedCamera != null ?
                mCurrentEuclideanEstimatedCamera.getCamera() : null;
        if (camera == null) {
            // if not available try with previous camera
            camera = mPreviousEuclideanEstimatedCamera != null ?
                    mPreviousEuclideanEstimatedCamera.getCamera() : null;
        }

        try {
            PinholeCameraIntrinsicParameters intrinsicParameters = null;
            if (camera != null) {
                if (!camera.areIntrinsicParametersAvailable()) {
                    // decompose camera to obtain intrinsic parameters
                    camera.decompose();
                }

                intrinsicParameters = camera.getIntrinsicParameters();
            } else if (mConfiguration.areIntrinsicParametersKnown()) {
                //noinspection unchecked
                intrinsicParameters = mListener.onIntrinsicParametersRequested((R) this, mCurrentViewId);
            }

            if (intrinsicParameters == null) {
                return;
            }

            final double lastPosX = mLastEuclideanCameraCenter != null ?
                    mLastEuclideanCameraCenter.getInhomX() : 0.0;
            final double lastPosY = mLastEuclideanCameraCenter != null ?
                    mLastEuclideanCameraCenter.getInhomY() : 0.0;
            final double lastPosZ = mLastEuclideanCameraCenter != null ?
                    mLastEuclideanCameraCenter.getInhomZ() : 0.0;

            final double positionX = lastPosX + mSlamEstimator.getStatePositionX();
            final double positionY = lastPosY + mSlamEstimator.getStatePositionY();
            final double positionZ = lastPosZ + mSlamEstimator.getStatePositionZ();
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

        } catch (final GeometryException ignore) {
            // do nothing
        }
    }
}
