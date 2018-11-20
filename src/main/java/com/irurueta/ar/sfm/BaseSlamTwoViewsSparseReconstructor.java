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
import com.irurueta.geometry.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Base class in charge of estimating cameras and 3D reconstructed points from
 * sparse image point correspondences in two views and also in charge of 
 * estimating overall scene scale by means of SLAM (Simultaneous Location And
 * Mapping) using data obtained from sensors like accelerometers or gyroscopes.
 * @param <C> type of configuration.
 * @param <R> type of reconstructor.
 * @param <L> type of listener.
 * @param <S> type of SLAM estimator.
 */
public abstract class BaseSlamTwoViewsSparseReconstructor<
        C extends BaseSlamTwoViewsSparseReconstructorConfiguration,
        R extends BaseSlamTwoViewsSparseReconstructor,
        L extends BaseSlamTwoViewsSparseReconstructorListener<R>,
        S extends BaseSlamEstimator> extends 
        BaseTwoViewsSparseReconstructor<C, R, L> {
    
    /**
     * Slam estimator to estimate position, speed, orientation using 
     * accelerometer and gyroscope data.
     */
    protected S mSlamEstimator;

    /**
     * Position estimated by means of SLAM. It is reused each time it is notified.
     */
    private InhomogeneousPoint3D mSlamPosition = new InhomogeneousPoint3D();

    /**
     * Camera estimated by means of SLAM. It is reused each time it is notified.
     */
    private PinholeCamera mSlamCamera = new PinholeCamera();

    /**
     * Camera rotation estimated by means of SLAM. It is reused each time it is notified.
     */
    private Quaternion mSlamRotation = new Quaternion();

    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not 
     * provided.
     */
    public BaseSlamTwoViewsSparseReconstructor(C configuration, 
            L listener) throws NullPointerException {
        super(configuration, listener);
    }
    
    /**
     * Provides a new accelerometer sample to update SLAM estimation.
     * This method must be called whenever the accelerometer sensor receives new 
     * data.
     * If reconstructor is not running, calling this method has no effect.
     * @param timestamp timestamp of accelerometer sample since epoch time and 
     * expressed in nanoseconds.
     * @param accelerationX linear acceleration along x-axis expressed in meters
     * per squared second (m/s^2).
     * @param accelerationY linear acceleration along y-axis expressed in meters
     * per squared second (m/s^2).
     * @param accelerationZ linear acceleration along z-axis expressed in meters
     * per squared second (m/s^2).
     */
    public void updateAccelerometerSample(long timestamp, float accelerationX,
            float accelerationY, float accelerationZ) {
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
     * @param timestamp timestamp of accelerometer sample since epoch time and
     * expressed in nanoseconds.
     * @param data array containing x,y,z components of linear acceleration
     * expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided array does not have length 
     * 3.
     */
    public void updateAccelerometerSample(long timestamp, float[] data)
            throws IllegalArgumentException {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateAccelerometerSample(timestamp, data);
        }
    }
    
    /**
     * Provides a new gyroscope sample to update SLAM estimation.
     * If reconstructor is not running, calling this method has no effect.
     * @param timestamp timestamp of gyroscope sample since epoch time and
     * expressed in nanoseconds.
     * @param angularSpeedX angular speed of rotation along x-axis expressed in
     * radians per second (rad/s).
     * @param angularSpeedY angular speed of rotation along y-axis expressed in
     * radians per second (rad/s).
     * @param angularSpeedZ angular speed of rotation along z-axis expressed in
     * radians per second (rad/s).
     */
    public void updateGyroscopeSample(long timestamp, float angularSpeedX,
            float angularSpeedY, float angularSpeedZ) {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateGyroscopeSample(timestamp, angularSpeedX, 
                    angularSpeedY, angularSpeedZ);
        }
    }
    
    /**
     * Provies a new gyroscope sample to update SLAM estimation.
     * If reconstructor is not running, calling this method has no effect.
     * @param timestamp timestamp of gyroscope sample since epoch time and 
     * expressed in nanoseconds.
     * @param data angular speed of rotation along x,y,z axes expressed in 
     * radians per second (rad/s).
     * @throws IllegalArgumentException if provided array does not have length 
     * 3.
     */
    public void updateGyroscopeSample(long timestamp, float[] data)
            throws IllegalArgumentException {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateGyroscopeSample(timestamp, data);
        }        
    }
    
    /**
     * Set ups calibration data on SLAM estimator if available.
     */
    protected void setUpCalibrationData() {
        BaseCalibrationData calibrationData = 
                mConfiguration.getCalibrationData();
        if (calibrationData != null) {
            //noinspection unchecked
            mSlamEstimator.setCalibrationData(calibrationData);
        }
    }

    /**
     * Configures listener of SLAM estimator
     */
    protected void setUpSlamEstimatorListener() {
        mSlamEstimator.setListener(new BaseSlamEstimator.BaseSlamEstimatorListener() {
            @Override
            public void onFullSampleReceived(BaseSlamEstimator estimator) { /* not used */ }

            @Override
            public void onFullSampleProcessed(BaseSlamEstimator estimator) {
                notifySlamStateIfNeeded();
                notifySlamCameraIfNeeded();
            }

            @Override
            public void onCorrectWithPositionMeasure(BaseSlamEstimator estimator) { /* not used */ }

            @Override
            public void onCorrectedWithPositionMeasure(BaseSlamEstimator estimator) {
                notifySlamStateIfNeeded();
                notifySlamCameraIfNeeded();
            }
        });
    }
    
    /**
     * Update scene scale using SLAM data.
     * @return true if scale was successfully updated, false otherwise.
     */
    protected boolean updateScale() {
        //obtain baseline (camera separation from slam estimator data
        double posX = mSlamEstimator.getStatePositionX();
        double posY = mSlamEstimator.getStatePositionY();
        double posZ = mSlamEstimator.getStatePositionZ();
            
        //to estimate baseline, we assume that first camera is placed at
        //world origin
        double baseline = Math.sqrt(posX*posX + posY*posY + posZ*posZ);
            
        try {
            PinholeCamera camera1 = mEstimatedCamera1.getCamera();
            PinholeCamera camera2 = mEstimatedCamera2.getCamera();
            
            camera1.decompose();
            camera2.decompose();

            Point3D center1 = camera1.getCameraCenter();
            Point3D center2 = camera2.getCameraCenter();
            
            double estimatedBaseline = center1.distanceTo(center2);
            
            double scale = baseline / estimatedBaseline;
            
            MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);
            
            //update scale of cameras
            scaleTransformation.transform(camera1);
            scaleTransformation.transform(camera2);
            
            mEstimatedCamera1.setCamera(camera1);
            mEstimatedCamera2.setCamera(camera2);
            
            //update scale of reconstructed points
            int numPoints = mReconstructedPoints.size();
            List<Point3D> reconstructedPoints3D = new ArrayList<>();
            for (ReconstructedPoint3D reconstructedPoint : mReconstructedPoints) {
                reconstructedPoints3D.add(reconstructedPoint.
                        getPoint());
            }
            
            scaleTransformation.transformAndOverwritePoints(
                    reconstructedPoints3D);
            
            //set scaled points into result
            for (int i = 0; i < numPoints; i++) {
                mReconstructedPoints.get(i).setPoint(
                        reconstructedPoints3D.get(i));
            }

            return true;
        } catch (Exception e) {
            mFailed = true;
            //noinspection unchecked
            mListener.onFail((R)this);

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

        double positionX = mSlamEstimator.getStatePositionX();
        double positionY = mSlamEstimator.getStatePositionY();
        double positionZ = mSlamEstimator.getStatePositionZ();

        double velocityX = mSlamEstimator.getStateVelocityX();
        double velocityY = mSlamEstimator.getStateVelocityY();
        double velocityZ = mSlamEstimator.getStateVelocityZ();

        double accelerationX = mSlamEstimator.getStateAccelerationX();
        double accelerationY = mSlamEstimator.getStateAccelerationY();
        double accelerationZ = mSlamEstimator.getStateAccelerationZ();

        double quaternionA = mSlamEstimator.getStateQuaternionA();
        double quaternionB = mSlamEstimator.getStateQuaternionB();
        double quaternionC = mSlamEstimator.getStateQuaternionC();
        double quaternionD = mSlamEstimator.getStateQuaternionD();

        double angularSpeedX = mSlamEstimator.getStateAngularSpeedX();
        double angularSpeedY = mSlamEstimator.getStateAngularSpeedY();
        double angularSpeedZ = mSlamEstimator.getStateAngularSpeedZ();

        //noinspection all
        mListener.onSlamDataAvailable((R)this, positionX, positionY, positionZ,
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

        double positionX = mSlamEstimator.getStatePositionX();
        double positionY = mSlamEstimator.getStatePositionY();
        double positionZ = mSlamEstimator.getStatePositionZ();
        mSlamPosition.setInhomogeneousCoordinates(positionX, positionY, positionZ);

        double quaternionA = mSlamEstimator.getStateQuaternionA();
        double quaternionB = mSlamEstimator.getStateQuaternionB();
        double quaternionC = mSlamEstimator.getStateQuaternionC();
        double quaternionD = mSlamEstimator.getStateQuaternionD();
        mSlamRotation.setA(quaternionA);
        mSlamRotation.setB(quaternionB);
        mSlamRotation.setC(quaternionC);
        mSlamRotation.setD(quaternionD);

        mSlamCamera.setIntrinsicAndExtrinsicParameters(intrinsicParameters, mSlamRotation,
                mSlamPosition);

        //noinspection all
        mListener.onSlamCameraEstimated((R)this, mSlamCamera);
    }
}
