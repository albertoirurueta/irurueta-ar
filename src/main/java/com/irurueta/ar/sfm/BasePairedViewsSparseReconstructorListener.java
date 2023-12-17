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

import com.irurueta.geometry.PinholeCameraIntrinsicParameters;

import java.util.List;

/**
 * Listener to retrieve and store required data to compute a 3D reconstruction from
 * sparse image point correspondences in multiple views.
 *
 * @param <R> type of re-constructor.
 */
public interface BasePairedViewsSparseReconstructorListener<
        R extends BasePairedViewsSparseReconstructor<?, ?, ?>> {

    /**
     * Called to determine whether there are more views available to attempt to use for
     * the reconstruction.
     *
     * @param reconstructor reconstructor raising this event.
     * @return true if there are more views available, false otherwise.
     */
    boolean hasMoreViewsAvailable(final R reconstructor);

    /**
     * Called when samples containing points of interest for current view must be
     * retrieved.
     *
     * @param reconstructor re-constructor raising this event.
     * @param viewId1       id of 1st view where points will be used in current view pair.
     * @param viewId2       id of 2nd view where points will be used in current view pair.
     * @param samples1      samples containing points of interest for 1st view to test in
     *                      current pair.
     * @param samples2      samples containing points of interest for 2nd view to test in
     *                      current pair.
     */
    void onRequestSamplesForCurrentViewPair(final R reconstructor, final int viewId1, final int viewId2,
                                            final List<Sample2D> samples1, final List<Sample2D> samples2);

    /**
     * Called when requested samples have been accepted.
     * This method can be used to determine whether samples can be stored or
     * not.
     *
     * @param reconstructor re-constructor raising this event.
     * @param viewId1       id of 1st view whose samples have been accepted in current view pair.
     * @param viewId2       id of 2nd view whose samples have been accepted in current view pair.
     * @param samples1      accepted samples on 1st view in current view pair.
     * @param samples2      accepted samples on 2nd view in current view pair.
     */
    void onSamplesAccepted(final R reconstructor, final int viewId1, final int viewId2,
                           final List<Sample2D> samples1, final List<Sample2D> samples2);

    /**
     * Called when requested samples have been rejected.
     * This method can be used to remove provided samples.
     *
     * @param reconstructor re-constructor raising this event.
     * @param viewId1       id of 1st view whose samples have been rejected in current view pair.
     * @param viewId2       id of 2nd view whose samples have been rejected in current view pair.
     * @param samples1      rejected samples on 1st view in current view pair.
     * @param samples2      rejected samples on 2nd view in current view pair.
     */
    void onSamplesRejected(final R reconstructor, final int viewId1, final int viewId2,
                           final List<Sample2D> samples1, final List<Sample2D> samples2);

    /**
     * Finds matches for provided samples.
     *
     * @param reconstructor re-constructor raising this event.
     * @param viewId1       id of 1st view where points will be used in current view pair.
     * @param viewId2       id of 2nd view where points will be used in current view pair.
     * @param samples1      samples containing points of interest for 1st view to test in
     *                      current pair.
     * @param samples2      samples containing points of interest for 2nd view to test in
     *                      current pair.
     * @param matches       instance where matches must be stored.
     */
    void onRequestMatches(final R reconstructor, final int viewId1, final int viewId2,
                          final List<Sample2D> samples1, final List<Sample2D> samples2,
                          final List<MatchedSamples> matches);

    /**
     * Called when a fundamental matrix relating a pair of views has been estimated.
     * This event can be used to store estimated fundamental matrix relating two views.
     *
     * @param reconstructor              re-constructor raising this event.
     * @param viewId1                    id of 1st view where points will be used in current view pair.
     * @param viewId2                    id of 2nd view where points will be used in current view pair.
     * @param estimatedFundamentalMatrix estimated fundamental matrix.
     */
    void onFundamentalMatrixEstimated(final R reconstructor, final int viewId1, final int viewId2,
                                      final EstimatedFundamentalMatrix estimatedFundamentalMatrix);

    /**
     * Called when cameras for provided matched pair of views have been estimated in an
     * Euclidean stratum (up to certain translation and rotation).
     * Implementations using SLAM techniques by mixing additional sensor data (i.e.
     * gyroscope and accelerometer) to estimate scale of each view pair, might also have
     * some inaccuracies in estimated scale.
     *
     * @param reconstructor re-constructor raising this event.
     * @param viewId1       id of previous view (i.e. 1st view).
     * @param viewId2       id of current view (i.e. 2nd view).
     * @param scale         estimated scale. When using SLAM this is estimated up to a certain
     *                      accuracy.
     * @param camera1       estimated Euclidean camera for previous view (i.e. 1st view).
     * @param camera2       estimated Euclidean camera for current view (i.e. 2nd view).
     */
    void onEuclideanCameraPairEstimated(final R reconstructor, final int viewId1, final int viewId2,
                                        final double scale, final EstimatedCamera camera1,
                                        final EstimatedCamera camera2);

    /**
     * Called when reconstructed points have been estimated from a series of 2D matches in a
     * pair of views in an Euclidean stratum (up to certain translation and rotation).
     *
     * @param reconstructor re-constructor raising this event.
     * @param viewId1       id of previous view (i.e. 1st view).
     * @param viewId2       id of current view (i.e. 2nd view).
     * @param scale         estimated scale. When using SLAM this is estimated up to a certain
     *                      accuracy.
     * @param points        reconstructed 3D points in Euclidean space.
     */
    void onEuclideanReconstructedPointsEstimated(final R reconstructor,
                                                 final int viewId1, final int viewId2, final double scale,
                                                 final List<ReconstructedPoint3D> points);

    /**
     * Called when intrinsic parameters are requested for a given view.
     * This is required if at configuration it was indicated that intrinsic parameters are
     * known, so that essential matrix method can be used for scene reconstruction.
     * If intrinsic parameters are unknown, DIAC or DAQ method will be attempted if possible.
     *
     * @param reconstructor re-constructor raising this event.
     * @param viewId        id of view whose parameters are requested.
     * @return intrinsic parameters if known, null otherwise.
     */
    PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
            final R reconstructor, final int viewId);

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
