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

import com.irurueta.ar.slam.AbsoluteOrientationSlamEstimator;

/**
 * Estimates cameras and 3D reconstructed points from sparse image point 
 * correspondences in two views and using SLAM (with accelerometer and gyroscope
 * data) with absolute orientation for overall scale and orientation estimation.
 */
public class AbsoluteOrientationSlamTwoViewsSparseReconstructor extends 
        BaseAbsoluteOrientationSlamTwoViewsSparseReconstructor<
        AbsoluteOrientationSlamTwoViewsSparseReconstructorConfiguration,
        AbsoluteOrientationSlamTwoViewsSparseReconstructor,
        AbsoluteOrientationSlamTwoViewsSparseReconstructorListener,
        AbsoluteOrientationSlamEstimator> {
    
    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not 
     * provided.
     */
    public AbsoluteOrientationSlamTwoViewsSparseReconstructor(
            AbsoluteOrientationSlamTwoViewsSparseReconstructorConfiguration configuration,
            AbsoluteOrientationSlamTwoViewsSparseReconstructorListener listener)
            throws NullPointerException {
        super(configuration, listener);
    }
    
    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener is not provided.
     */
    public AbsoluteOrientationSlamTwoViewsSparseReconstructor(
            AbsoluteOrientationSlamTwoViewsSparseReconstructorListener listener)
            throws NullPointerException {
        this(new AbsoluteOrientationSlamTwoViewsSparseReconstructorConfiguration(), 
                listener);
    }

    /**
     * Process one view of all the available data during the reconstruction.
     * This method can be called multiple times instead of {@link #start()} to build the reconstruction step by step,
     * one view at a time.
     * @return true if more views can be processed, false when reconstruction has finished.
     */
    @Override
    public boolean processOneView() {
        if (!mRunning) {
            mSlamEstimator = new AbsoluteOrientationSlamEstimator();
            setUpSlamEstimatorListener();
            setUpCalibrationData();
        }

        return super.processOneView();
    }

    /**
     * Called when processing one frame is successfully finished. This can be done to estimate scale on those
     * implementations where scale can be measured or is already known.
     * @return true if post processing succeeded, false otherwise.
     */
    @Override
    protected boolean postProcessOne() {
        return updateScaleAndOrientation();
    }
}
