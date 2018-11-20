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
package com.irurueta.ar.sfm;

/**
 * Type of 3D point triangulator.
 * Homogeneous methods are suitable for any case
 * Inhomogeneous ones are suitable only for cases where finite points and
 * cameras are being used. If points or cameras are located very far or at
 * infinity, triangulation will fail when using inhomogeneous methods.
 */
public enum Point3DTriangulatorType {
    /**
     * Triangulator using homogeneous method and an LMSE (Least Mean Square 
     * Error) solution.
     */
    LMSE_HOMOGENEOUS_TRIANGULATOR,
    
    /**
     * Triangulator using inhomogeneous method and an LMSE (Least Mean Square
     * Error) solution.
     */
    LMSE_INHOMOGENEOUS_TRIANGULATOR,
    
    /**
     * Triangulator using homogeneous method and a weighted solution.
     */
    WEIGHTED_HOMOGENEOUS_TRIANGULATOR,
    
    /**
     * Triangulator using inhomogeneous method and a weighted solution.
     */
    WEIGHTED_INHOMOGENEOUS_TRIANGULATOR
}
