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
package com.irurueta.ar.epipolar.estimators;

/**
 * Indicates method of non-robust fundamental matrix estimator.
 */
public enum FundamentalMatrixEstimatorMethod {
    /**
     * Fundamental matrix estimation method based on 7 matched 2D points.
     */
    SEVEN_POINTS_ALGORITHM,
    
    /**
     * Fundamental matrix estimation method based on 8 matched 2D points.
     */
    EIGHT_POINTS_ALGORITHM,
    
    /**
     * Algorhtm used for Affine geometry. This method should only be used
     * on specific cases where cameras ore geometry are known to be affine.
     */
    AFFINE_ALGORITHM
}
