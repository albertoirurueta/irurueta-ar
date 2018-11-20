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

import java.io.Serializable;

/**
 * Contains configuration for a two view sparse reconstructor assuming that the
 * baseline (separation between cameras) is known.
 */
public class KnownBaselineTwoViewsSparseReconstructorConfiguration extends 
        BaseTwoViewsSparseReconstructorConfiguration<
        KnownBaselineTwoViewsSparseReconstructorConfiguration> implements Serializable {
    
    /**
     * Default camera baseline (expressed in a unit of distance such as meters).
     * Methods such as DIAC or Essential assume that the camera baseline is 1.0,
     * which yields a reconstruction up to scale, unless the real baseline is
     * provided.
     */
    public static final double DEFAULT_BASELINE = 1.0;
    
    /**
     * Camera baseline (expressed in a unit of distance such as meters).
     * Contains the real separation between camera centers so that the real
     * scale of cameras and reconstructed points can be retrieved.
     */
    private double mBaseline = DEFAULT_BASELINE;
    
    /**
     * Constructor.
     */
    public KnownBaselineTwoViewsSparseReconstructorConfiguration() { }
    
    /**
     * Creates an instance of a two views sparse reconstructor configuration 
     * with known camera baseline.
     * @return configuration instance.
     */
    public static KnownBaselineTwoViewsSparseReconstructorConfiguration make() {
        return new KnownBaselineTwoViewsSparseReconstructorConfiguration();
    }
    
    /**
     * Gets camera baseline (expressed in a unit of distance such as meters).
     * Contains the real separation between camera centers so that the real
     * scale of cameras and reconstructed points can be retrieved.
     * @return camera baseline.
     */
    public double getBaseline() {
        return mBaseline;
    }
    
    /**
     * Sets camera baseline (expressed in a unit of distance such as meters).
     * Contains the real separation between camera centers so that the real
     * scale of cameras and reconstructed points can be retrieved.
     * @param baseline camera baseline.
     * @return this instance so that method can be easily chained.
     */
    public KnownBaselineTwoViewsSparseReconstructorConfiguration setBaseline(
            double baseline) {
        mBaseline = baseline;
        return this;
    }
}
