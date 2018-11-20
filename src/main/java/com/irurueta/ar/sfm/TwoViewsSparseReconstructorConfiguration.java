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
 * Contains configuration for a two view sparse reconstructor.
 */
public class TwoViewsSparseReconstructorConfiguration extends 
        BaseTwoViewsSparseReconstructorConfiguration<
        TwoViewsSparseReconstructorConfiguration> implements Serializable {
    

    /**
     * Constructor.
     */
    public TwoViewsSparseReconstructorConfiguration() { }
    
    /**
     * Creates an instance of a two views sparse reconstructor configuration.
     * @return configuration instance.
     */
    public static TwoViewsSparseReconstructorConfiguration make() {
        return new TwoViewsSparseReconstructorConfiguration();
    }   
}
