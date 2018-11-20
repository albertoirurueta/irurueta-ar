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

import java.io.Serializable;

/**
 * Contains configuration for a paired view sparse reconstructor using SLAM (Simultaneous
 * LocationAnd Mapping) to determine the scale of the scene (i.e. the baseline or separation
 * between cameras) by fusing both camera data and data from sensors like an accelerometer
 * or gyroscope.
 * This configuration assumes a constant velocity model and that an absolute orientation
 * will be considered on each view.
 */
public class AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration
        extends BaseSlamPairedViewsSparseReconstructorConfiguration<
        AbsoluteOrientationConstantVelocityModelSlamCalibrationData,
        AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration> implements
        Serializable {

    /**
     * Creates an instance of a multiple views sparse reconstructor configuration with
     * constant velocity model in slam estimation and absolute orientation.
     * @return configuration instance.
     */
    public static AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration make() {
        return new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();
    }
}
