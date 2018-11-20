/*
 * Copyright (C) 2015 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.ar.calibration;

/**
 * Contains camera calibrator methods.
 */
public enum CameraCalibratorMethod {
    /**
     * Alternates the estimation of the intrinsic camera parameters with
     * radial distortion estimation until the solution converges.
     * This is usually slower than error optimization method but typically is
     * more numerically stable.
     */
    ALTERNATING_CALIBRATOR,
    
    /**
     * Optimizes radial distortion and intrinsic parameters after an initial
     * guess is found until the solution converges.
     * This typically converges faster than the alternating method.
     */
    ERROR_OPTIMIZATION,
}
