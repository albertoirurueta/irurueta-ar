/*
 * Copyright (C) 2016 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.ar.slam;

import java.io.Serializable;

/**
 * Contains control calibration data for an absolute orientation SLAM estimator
 * during Kalman filtering prediction stage.
 */
public class AbsoluteOrientationSlamCalibrationData extends BaseCalibrationData implements Serializable {

    /**
     * Constructor.
     */
    public AbsoluteOrientationSlamCalibrationData() {
        super(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH, AbsoluteOrientationSlamEstimator.STATE_LENGTH);
    }
}
