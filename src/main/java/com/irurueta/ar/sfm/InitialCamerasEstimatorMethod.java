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
package com.irurueta.ar.sfm;

/**
 * Indicates method used for initial cameras estimation.
 */
public enum InitialCamerasEstimatorMethod {
    /**
     * Uses a two step method based on the estimation of the Dual Absolute 
     * Quadric (DAQ) to estimate camera intrinsic parameters, and then those
     * intrinsic parameters are used to estimate initial cameras using the
     * essential matrix.
     * This is the most accurate method, solving cameras and reconstructed 
     * points up to scale with the smallest error.
     */
    DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX,
    
    /**
     * Uses a method based on the estimation of the Dual Absolute Quadric (DAQ).
     * This method always assumes that principal point is zero (located at the
     * center of the image).
     * This method usually gets good accuracy on camera intrinsics estimation,
     * but extrinsic parameters and reconstructed points usually have poor
     * accuracy.
     */
    DUAL_ABSOLUTE_QUADRIC,
    
    /**
     * Uses a method based on the estimation of the Dual Image of Absolute Conic
     * (DIAC) by solving the Kruppa equations.
     * This method requires that principal point is not close to zero (typically
     * coordinates of the center of the image are provided assuming that origin
     * is at bottom-left corner, (unless aspect ratio is negative and origin is 
     * considered to be at top-left corner).
     * This method also requires that cameras pose has enough rotation (i.e.
     * are not too parallel).
     * This method might fail or produce poor results if this requirements are
     * not met.
     */
    DUAL_IMAGE_OF_ABSOLUTE_CONIC,
    
    /**
     * This method uses the essential matrix to determine initial cameras.
     * This can only be used when cameras have been previously calibrated by
     * some offline method and their intrinsic parameters are known.
     */
    ESSENTIAL_MATRIX,
}
