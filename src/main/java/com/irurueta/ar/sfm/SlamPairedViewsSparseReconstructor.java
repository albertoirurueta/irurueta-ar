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

import com.irurueta.ar.slam.SlamCalibrationData;
import com.irurueta.ar.slam.SlamEstimator;

/**
 * Estimates pairs of cameras and 3D reconstructed points from sparse image point correspondences
 * in multiple view pairs and using SLAM (with accelerometer and gyroscope data) for overall
 * scale estimation.
 */
public class SlamPairedViewsSparseReconstructor extends BaseSlamPairedViewsSparseReconstructor<
        SlamCalibrationData, SlamPairedViewsSparseReconstructorConfiguration, SlamPairedViewsSparseReconstructor,
        SlamPairedViewsSparseReconstructorListener, SlamEstimator> {

    /**
     * Constructor.
     *
     * @param configuration configuration for this re-constructor.
     * @param listener      listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     *                              provided.
     */
    public SlamPairedViewsSparseReconstructor(
            final SlamPairedViewsSparseReconstructorConfiguration configuration,
            final SlamPairedViewsSparseReconstructorListener listener) {
        super(configuration, listener);
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     *                              provided.
     */
    public SlamPairedViewsSparseReconstructor(final SlamPairedViewsSparseReconstructorListener listener) {
        super(new SlamPairedViewsSparseReconstructorConfiguration(), listener);
    }

    /**
     * Process one view of all the available data during the reconstruction.
     * This method can be called multiple times instead of {@link #start()} to build the reconstruction
     * step by step, one view at a time.
     *
     * @return true if more views can be processed, false when reconstruction has finished.
     */
    @Override
    public boolean processOneViewPair() {
        if (!running) {
            slamEstimator = new SlamEstimator();
            setUpSlamEstimatorListener();
            setUpCalibrationData();
        }

        return super.processOneViewPair();
    }
}
