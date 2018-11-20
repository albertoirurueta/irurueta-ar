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
 * Listener to be notified of events such as when triangulation starts, ends or
 * when progress changes.
 */
public interface RobustSinglePoint3DTriangulatorListener {
    
    /**
     * Called when triangulation starts.
     * @param triangulator reference to robust triangulator.
     */
    void onTriangulateStart(
            RobustSinglePoint3DTriangulator triangulator);
    
    /**
     * Called when triangulation ends.
     * @param triangulator reference to robust triangulator.
     */
    void onTriangulateEnd(RobustSinglePoint3DTriangulator triangulator);
    
    /**
     * Called when robust triangulator iterates to refine a possible solution.
     * @param triangulator reference to robust triangulator.
     * @param iteration current iteration.
     */
    void onTriangulateNextIteration(
            RobustSinglePoint3DTriangulator triangulator, int iteration);
    
    /**
     * Called when estimation progress changes significantly.
     * @param triangulator reference to robust triangulator.
     * @param progress progress of estimation expressed as a value between 0.0
     * and 1.0.
     */
    void onTriangulateProgressChange(
            RobustSinglePoint3DTriangulator triangulator, float progress);
}
