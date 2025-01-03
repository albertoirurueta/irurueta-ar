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

import java.util.List;

/**
 * Listener to retrieve and store required data to compute a 3D reconstruction
 * from sparse image point correspondences in two views.
 *
 * @param <R> type of re-constructor.
 */
public interface BaseTwoViewsSparseReconstructorListener<R extends BaseTwoViewsSparseReconstructor<?, ?, ?>> {
    /**
     * Called to determine whether there are more views available to attempt to
     * use for the reconstruction.
     *
     * @param reconstructor re-constructor raising this event.
     * @return true if there are more views available, false otherwise.
     */
    boolean hasMoreViewsAvailable(final R reconstructor);

    /**
     * Called when samples containing points of interest for current view must
     * be retrieved.
     *
     * @param reconstructor re-constructor raising this event.
     * @param viewId        id of view where points will be used.
     * @param samples       samples containing points of interest for current view to
     *                      test.
     */
    void onRequestSamplesForCurrentView(final R reconstructor, final int viewId, final List<Sample2D> samples);

    /**
     * Called when requested samples have been accepted.
     * This method can be used to determine whether samples can be stored or
     * not.
     *
     * @param reconstructor re-constructor raising this event.
     * @param viewId        id of view whose samples have been accepted.
     * @param samples       accepted samples.
     */
    void onSamplesAccepted(final R reconstructor, final int viewId, final List<Sample2D> samples);

    /**
     * Called when requested samples have been rejected.
     * This method can be used to remove provided samples.
     *
     * @param reconstructor re-constructor raising this event.
     * @param viewId        id of view whose samples have been rejected.
     * @param samples       rejected samples.
     */
    void onSamplesRejected(final R reconstructor, final int viewId, final List<Sample2D> samples);

    /**
     * Finds matches for provided samples.
     *
     * @param reconstructor re-constructor raising this event.
     * @param samples1      samples on first view.
     * @param samples2      samples on second view.
     * @param viewId1       id of first view.
     * @param viewId2       id of second view.
     * @param matches       instance where matches must be stored.
     */
    void onRequestMatches(final R reconstructor, final List<Sample2D> samples1, final List<Sample2D> samples2,
                          final int viewId1, final int viewId2, final List<MatchedSamples> matches);

    /**
     * Called when a fundamental matrix relating two views has been estimated.
     * This event can be used to store estimated fundamental matrix relating
     * two views.
     *
     * @param reconstructor              re-constructor raising this event.
     * @param estimatedFundamentalMatrix estimated fundamental matrix.
     */
    void onFundamentalMatrixEstimated(final R reconstructor,
                                      final EstimatedFundamentalMatrix estimatedFundamentalMatrix);

    /**
     * Notifies when cameras for provided matched pair of views have been
     * estimated. This event can be used to store points associated to such
     * view.
     *
     * @param reconstructor re-constructor raising this event.
     * @param viewId1       id of first view.
     * @param viewId2       id of second view.
     * @param camera1       estimated camera for first view.
     * @param camera2       estimated camera for second view.
     */
    void onCamerasEstimated(final R reconstructor, final int viewId1, final int viewId2, final EstimatedCamera camera1,
                            final EstimatedCamera camera2);

    /**
     * Called when reconstructed points have been estimated from a series of 2D
     * matches. This event can be used to store reconstructed points and their
     * associated data.
     *
     * @param reconstructor re-constructor raising this event.
     * @param matches       2D matches associated to estimated reconstructed points.
     * @param points        reconstructed 3D points.
     */
    void onReconstructedPointsEstimated(final R reconstructor, final List<MatchedSamples> matches,
                                        final List<ReconstructedPoint3D> points);

    /**
     * Called when reconstruction starts.
     *
     * @param reconstructor re-constructor raising this event.
     */
    void onStart(final R reconstructor);

    /**
     * Called when reconstruction stops.
     *
     * @param reconstructor re-constructor raising this event.
     */
    void onFinish(final R reconstructor);

    /**
     * Called when reconstruction is cancelled before it has finished.
     *
     * @param reconstructor re-constructor raising this event.
     */
    void onCancel(final R reconstructor);

    /**
     * Called when reconstruction fails.
     *
     * @param reconstructor re-constructor raising this event.
     */
    void onFail(final R reconstructor);
}
