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

import com.irurueta.ar.slam.AbsoluteOrientationBaseSlamEstimator;
import com.irurueta.ar.slam.BaseCalibrationData;
import com.irurueta.geometry.Rotation3D;

/**
 * Base class in charge of estimating pairs of cameras and 3D reconstructed points from sparse
 * image point correspondences in multiple view pairs and also in charge of estimating overall
 * scene scale and absolute orientation by means of SLAM (Simultaneous Location And Mapping)
 * using data obtained from sensors like accelerometers or gyroscopes.
 * NOTE: absolute orientation slam estimators are not very accurate during estimation of
 * the orientation state, for that reason we take into account the initial orientation.
 *
 * @param <D> calibrator type associated to implementations of SLAM calibration
 *            data.
 * @param <C> type of configuration.
 * @param <R> type of re-constructor.
 * @param <L> type of listener.
 * @param <S> type of SLAM estimator.
 */
public abstract class BaseAbsoluteOrientationSlamPairedViewsSparseReconstructor<
        D extends BaseCalibrationData,
        C extends BaseSlamPairedViewsSparseReconstructorConfiguration<D, C>,
        R extends BaseSlamPairedViewsSparseReconstructor<D, C, R, L, S>,
        L extends BaseSlamPairedViewsSparseReconstructorListener<R>,
        S extends AbsoluteOrientationBaseSlamEstimator<D>> extends
        BaseSlamPairedViewsSparseReconstructor<D, C, R, L, S> {

    /**
     * First sample of orientation received.
     */
    private Rotation3D mFirstOrientation;

    /**
     * Constructor.
     *
     * @param configuration configuration for this re-constructor.
     * @param listener      listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     *                              provided.
     */
    protected BaseAbsoluteOrientationSlamPairedViewsSparseReconstructor(
            final C configuration, final L listener) {
        super(configuration, listener);
    }

    /**
     * Provides a new orientation sample to update SLAM estimator.
     * If re-constructor is not running, calling this method has no effect.
     *
     * @param timestamp   timestamp of accelerometer sample since epoch time and
     *                    expressed in nanoseconds.
     * @param orientation new orientation.
     */
    public void updateOrientationSample(final long timestamp,
                                        final Rotation3D orientation) {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateOrientationSample(timestamp, orientation);
        }
        if (mFirstOrientation == null) {
            // make a copy of orientation
            mFirstOrientation = orientation.toQuaternion();
        }
    }

    /**
     * Indicates whether implementations of a re-constructor uses absolute orientation or
     * not.
     *
     * @return true if absolute orientation is used, false, otherwise.
     */
    @Override
    protected boolean hasAbsoluteOrientation() {
        return true;
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
    protected boolean transformPairOfCamerasAndPoints(
            final boolean isInitialPairOfViews,
            final boolean hasAbsoluteOrientation) {

        // R1' = R1*Rdiff
        // Rdiff = R1^T*R1'

        // where R1' is the desired orientation (obtained by sampling a
        // sensor)
        // and R1 is always the identity for the 1st camera.
        // Hence R1' = Rdiff

        // t1' is the desired translation which is zero for the 1st
        // camera.

        // We want: P1' = K*[R1' t1'] = K*[R1' 0]
        // And we have P1 = K[I 0]

        // We need a transformation T so that:
        // P1' = P1*T^-1 = K[I 0][R1' 0]
        //                       [0   1]

        // Hence: T^-1 = [R1' 0]
        //               [0   1]

        // or T = [R1'^T 0]
        //        [0     1]

        // because we are also applying a transformation of scale s,
        // the combination of both transformations is
        // T = [s*R1'^T 0]
        //     [0       1]

        if (isInitialPairOfViews && hasAbsoluteOrientation) {
            // The first pair of views does not require setting translation and rotation
            mLastEuclideanCameraRotation = mFirstOrientation;
        }

        return super.transformPairOfCamerasAndPoints(isInitialPairOfViews, hasAbsoluteOrientation);
    }
}
