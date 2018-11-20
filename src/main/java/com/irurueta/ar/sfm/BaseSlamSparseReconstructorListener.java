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

import com.irurueta.algebra.Matrix;
import com.irurueta.ar.slam.AbsoluteOrientationConstantVelocityModelSlamEstimator;
import com.irurueta.ar.slam.AbsoluteOrientationSlamEstimator;
import com.irurueta.ar.slam.ConstantVelocityModelSlamEstimator;
import com.irurueta.ar.slam.SlamEstimator;
import com.irurueta.geometry.PinholeCamera;

/**
 * Listener to retrieve and store required data to compute a 3D reconstruction from sparse
 * image point correspondences in multiple views.
 * Implementations of this interface also notify when cameras are estimated due to
 * received IMU (Inertial Measurement Unit) data, which contains accelerometer and gyroscope
 * data and uses a SLAM estimator.
 * @param <R> type of reconstructor.
 */
public interface BaseSlamSparseReconstructorListener<R extends BaseSlamSparseReconstructor> extends
        BaseSparseReconstructorListener<R> {

    /**
     * Called whenever slam data notification is enabled and each time all required samples (accelerometer,
     * gyroscope or orientation) are received in order to update SLAM system state to notify
     * new position, velocity, acceleration, orientation and angular speed.
     * @param reconstructor reconstructor raising this event.
     * @param positionX x position coordinate expressed in meters (m).
     * @param positionY y position coordinate expressed in meters (m).
     * @param positionZ z position coordinate expressed in meters (m).
     * @param velocityX x velocity coordinate expressed in meters per second (m/s).
     * @param velocityY y velocity coordinate expressed in meters per second (m/s).
     * @param velocityZ z velocity coordinate expressed in meters per second (m/s).
     * @param accelerationX x linear acceleration coordinate expressed in meters per squared second (m/s^2).
     * @param accelerationY y linear acceleration coordinate expressed in meters per squared second (m/s^2).
     * @param accelerationZ z linear acceleration coordinate expressed in meters per squared second (m/s^2).
     * @param quaternionA a component of quaternion expressing current orientation.
     * @param quaternionB b component of quaternion expressing current orientation.
     * @param quaternionC c component of quaternion expressing current orientation.
     * @param quaternionD d component of quaternion expressing current orientation.
     * @param angularSpeedX x coordinate of angular speed expressed in radians per second (rad/s).
     * @param angularSpeedY y coordinate of angular speed expressed in radians per second (rad/s).
     * @param angularSpeedZ z coordinate of angular speed expressed in radians per second (rad/s).
     * @param covariance contains covariance matrix of estimated SLAM state. Matrix meaning will change depending
     *                       on slam implementation. See: {@link SlamEstimator#getStateCovariance()},
     *                       {@link ConstantVelocityModelSlamEstimator#getStateCovariance()},
     *                       {@link AbsoluteOrientationSlamEstimator#getStateCovariance()} or
     *                       {@link AbsoluteOrientationConstantVelocityModelSlamEstimator#getStateCovariance()}.
     */
    void onSlamDataAvailable(R reconstructor, double positionX, double positionY, double positionZ,
                             double velocityX, double velocityY, double velocityZ,
                             double accelerationX, double accelerationY, double accelerationZ,
                             double quaternionA, double quaternionB, double quaternionC, double quaternionD,
                             double angularSpeedX, double angularSpeedY, double angularSpeedZ, Matrix covariance);

    /**
     * Called whenever estimated SLAM camera notification is enabled.
     * This method is called each time all required samples (accelerometer, gyroscope or orientation) are
     * received in order to update SLAM system state to notify a new camera containing
     * current intrinsic parameters, position and orientation.
     * @param reconstructor reconstructor raising this event.
     * @param camera current camera estimated using IMU data. This instance and its associated instances (camera center,
     *               rotation and intrinsic parameters) will be reused between consecutive calls to this method.
     */
    void onSlamCameraEstimated(R reconstructor, PinholeCamera camera);
}
