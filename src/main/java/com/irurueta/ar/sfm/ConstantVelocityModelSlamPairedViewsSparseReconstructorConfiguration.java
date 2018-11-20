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

import com.irurueta.ar.slam.ConstantVelocityModelSlamCalibrationData;

import java.io.Serializable;

/**
 * Contains configuration for a paired view sparse reconstructor using SLAM (Simultaneous
 * Location And Mapping) to determine the scale of the scene (i.e. the baseline or separation
 * between cameras) by fusing both camera data and data from sensors like and accelerometer or
 * gyroscope.
 * This configuration assumes a constant velocity model and that an orientation relative to
 * the first view will be estimated.
 */
public class ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration extends
        BaseSlamPairedViewsSparseReconstructorConfiguration<ConstantVelocityModelSlamCalibrationData,
                ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration> implements Serializable {

    /**
     * Creates an instance of a paired view sparse reconstructor configuration with constant
     * velocity model in slam estimation.
     * @return configuration instance.
     */
    public static ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration make() {
        return new ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();
    }
}
