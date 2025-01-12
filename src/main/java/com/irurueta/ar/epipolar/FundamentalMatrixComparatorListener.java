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
package com.irurueta.ar.epipolar;

/**
 * Handles events produced by a FundamentalMatrixComparator.
 */
public interface FundamentalMatrixComparatorListener {

    /**
     * Called when comparison starts.
     *
     * @param comparator instance that raised the event.
     */
    void onCompareStart(final FundamentalMatrixComparator comparator);

    /**
     * Called when comparison finishes.
     *
     * @param comparator instance that raised the event.
     */
    void onCompareEnd(final FundamentalMatrixComparator comparator);

    /**
     * Called when progress of comparison significantly changes.
     *
     * @param comparator instance that raised the event.
     * @param progress   progress of comparison expressed as a value between 0.0
     *                   and 1.0.
     */
    void onCompareProgressChange(final FundamentalMatrixComparator comparator, final float progress);
}
