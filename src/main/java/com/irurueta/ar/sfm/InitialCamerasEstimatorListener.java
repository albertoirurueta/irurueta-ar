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

import com.irurueta.geometry.PinholeCamera;

/**
 * Listener in charge of attending events for an InitialCamerasEstimator,
 * such as when estimation starts or finishes.
 */
public interface InitialCamerasEstimatorListener {
    
    /**
     * Called when estimation starts.
     * @param estimator estimator raising the event.
     */
    void onStart(InitialCamerasEstimator estimator);
    
    /**
     * Called when estimation successfully finishes.
     * @param estimator estimator raising the event.
     * @param estimatedLeftCamera estimated camera on left view.
     * @param estimatedRightCamera estimated camera on right view.
     */
    void onFinish(InitialCamerasEstimator estimator, 
            PinholeCamera estimatedLeftCamera, 
            PinholeCamera estimatedRightCamera);
    
    /**
     * Called when estimation fails.
     * @param estimator estimator raising the event.
     * @param e reason of estimation failure.
     */
    void onFail(InitialCamerasEstimator estimator, 
            InitialCamerasEstimationFailedException e);
}
