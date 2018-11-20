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
 * Listener to retrieve and and store required data to compute a 3D reconstruction
 * from sparse image point correspondences.
 */
public interface PairedViewsSparseReconstructorListener extends
        BasePairedViewsSparseReconstructorListener<PairedViewsSparseReconstructor>{

    /**
     * Requests baseline for provided pair of views so that actual scale can be estimated
     * to obtain cameras an reconstructed points in an euclidean space (up to certain rotation and
     * translation).
     * @param reconstructor reconstructor raising this event.
     * @param viewId1 id of previous view (i.e. 1st view).
     * @param viewId2 id of current view (i.e. 2nd view).
     * @param metricCamera1 estimated first metric camera.
     * @param metricCamera2 estimated second metric camera
     * @return baseline separating cameras.
     */
    double onBaselineRequested(PairedViewsSparseReconstructor reconstructor, int viewId1, int viewId2,
                       EstimatedCamera metricCamera1, EstimatedCamera metricCamera2);
}
