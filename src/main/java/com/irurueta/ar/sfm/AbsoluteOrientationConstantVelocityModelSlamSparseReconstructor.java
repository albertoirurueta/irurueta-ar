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

import com.irurueta.ar.slam.AbsoluteOrientationConstantVelocityModelSlamCalibrationData;
import com.irurueta.ar.slam.AbsoluteOrientationConstantVelocityModelSlamEstimator;

/**
 * Estimates cameras and 3D reconstructed points from sparse image point
 * correspondences in multiple views and using SLAM (with accelerometer and gyroscope
 * data) with absolute orientation for overall scale and orientation estimation.
 * NOTE: This implementation does not seem to be very reliable because the SLAM
 * estimator is not accurate at all on position or orientation estimator.
 * Use {@link AbsoluteOrientationSlamSparseReconstructor} if absolute orientation is needed,
 * or {@link SlamSparseReconstructor} otherwise to get better accuracy.
 */
public class AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor extends
        BaseAbsoluteOrientationSlamSparseReconstructor<
                AbsoluteOrientationConstantVelocityModelSlamCalibrationData,
                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorConfiguration,
                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor,
                AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorListener,
                AbsoluteOrientationConstantVelocityModelSlamEstimator> {

    /**
     * Constructor.
     *
     * @param configuration configuration for this re-constructor.
     * @param listener      listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     *                              provided.
     */
    public AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor(
            final AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorConfiguration configuration,
            final AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorListener listener) {
        super(configuration, listener);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener is not provided.
     */
    public AbsoluteOrientationConstantVelocityModelSlamSparseReconstructor(
            final AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorListener listener) {
        this(new AbsoluteOrientationConstantVelocityModelSlamSparseReconstructorConfiguration(), listener);
    }

    /**
     * Process one view of all the available data during the reconstruction.
     * This method can be called multiple times instead of {@link #start()} to build the reconstruction
     * step by step, one view at a time.
     *
     * @return true if more views can be processed, false when reconstruction has finished.
     */
    @Override
    public boolean processOneView() {
        if (!running) {
            slamEstimator = new AbsoluteOrientationConstantVelocityModelSlamEstimator();
            setUpSlamEstimatorListener();
            setUpCalibrationData();
        }

        return super.processOneView();
    }

    /**
     * Called when processing one frame is successfully finished. This can be done to estimate scale on those
     * implementations where scale can be measured or is already known.
     *
     * @param isInitialPairOfViews true if initial pair of views is being processed, false otherwise.
     * @return true if post-processing succeeded, false otherwise.
     */
    @Override
    protected boolean postProcessOne(final boolean isInitialPairOfViews) {
        return updateScaleAndOrientation(isInitialPairOfViews);
    }
}
