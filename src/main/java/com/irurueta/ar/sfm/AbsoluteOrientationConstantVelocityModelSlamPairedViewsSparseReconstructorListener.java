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
 * Listener to retrieve and store required data to compute a 3D reconstruction from
 * sparse image point correspondences in multiple view pairs and using SLAM with
 * absolute orientation and constant velocity model for scale and orientation estimation.
 */
public interface AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorListener
        extends BaseSlamPairedViewsSparseReconstructorListener<
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructor> {
}
