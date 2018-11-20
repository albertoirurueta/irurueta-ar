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

/**
 * Class in charge of estimating cameras and 3D reconstruction points from
 * sparse image point correspondences.
 */
public class SparseReconstructor extends
        BaseSparseReconstructor<SparseReconstructorConfiguration, SparseReconstructor,
        SparseReconstructorListener> {

    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     * provided.
     */
    public SparseReconstructor(SparseReconstructorConfiguration configuration,
            SparseReconstructorListener listener) throws NullPointerException {
        super(configuration, listener);
    }

    /**
     * Constructor with default configuration.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener is not provided.
     */
    public SparseReconstructor(SparseReconstructorListener listener)
            throws NullPointerException {
        this(new SparseReconstructorConfiguration(), listener);
    }

    /**
     * Called when processing one frame is successfully finished. This can be done to estimate scale on those
     * implementations where scale can be measured or is already known.
     * @param isInitialPairOfViews true if initial pair of views is being processed, false otherwise.
     * @return true if post processing succeeded, false otherwise.
     */
    @Override
    protected boolean postProcessOne(boolean isInitialPairOfViews) {
        //no need for post processing when computing metric reconstruction
        mPreviousEuclideanEstimatedCamera = mPreviousMetricEstimatedCamera;
        mCurrentEuclideanEstimatedCamera = mCurrentMetricEstimatedCamera;
        mActiveEuclideanReconstructedPoints = mActiveMetricReconstructedPoints;
        mCurrentScale = DEFAULT_SCALE;
        return true;
    }
}
