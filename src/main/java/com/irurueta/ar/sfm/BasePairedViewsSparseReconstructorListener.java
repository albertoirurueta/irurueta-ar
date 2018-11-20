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
 * @param <R> type of reconstructor.
 */
public interface BasePairedViewsSparseReconstructorListener<
        R extends BasePairedViewsSparseReconstructor> {

    /**
     * Called to determine whether there are more views available to attempt to use for
     * the reconstruction.
     * @param reconstructor reconstructor raising this event.
     * @return true if there are more views available, false otherwise.
     */
    boolean hasMoreViewsAvailable(R reconstructor);

    /**
     * Called when samples containing points of interest for current view must be
     * retrieved.
     * @param reconstructor reconstructor raising this event.
     * @param viewId1 id of 1st view where points will be used in current view pair.
     * @param viewId2 id of 2nd view where points will be used in current view pair.
     * @param samples1 samples containing points of interest for 1st view to test in
     *                 current pair.
     * @param samples2 samples containing points of interest for 2nd view to test in
     *                 current pair.
     */
    void onRequestSamplesForCurrentViewPair(R reconstructor, int viewId1, int viewId2,
                                            List<Sample2D> samples1, List<Sample2D> samples2);

    /**
     * Called when requested samples have been accepted.
     * This method can be used to determine whether samples can be stored or
     * not.
     * @param reconstructor reconstructor raising this event.
     * @param viewId1 id of 1st view whose samples have been accepted in current view pair.
     * @param viewId2 id of 2nd view whose samples have been accepted in current view pair.
     * @param samples1 accepted samples on 1st view in current view pair.
     * @param samples2 accepted samples on 2nd view in current view pair.
     */
    void onSamplesAccepted(R reconstructor, int viewId1, int viewId2,
                           List<Sample2D> samples1, List<Sample2D> samples2);

    /**
     * Called when requested samples have been rejected.
     * This method can be used to remove provided samples.
     * @param reconstructor reconstructor raising this event.
     * @param viewId1 id of 1st view whose samples have been rejected in current view pair.
     * @param viewId2 id of 2nd view whose samples have been rejected in current view pair.
     * @param samples1 rejected samples on 1st view in current view pair.
     * @param samples2 rejected samples on 2nd view in current view pair.
     */
    void onSamplesRejected(R reconstructor, int viewId1, int viewId2,
                           List<Sample2D> samples1, List<Sample2D> samples2);

    /**
     * Finds matches for provided samples.
     * @param reconstructor reconstructor raising this event.
     * @param viewId1 id of 1st view where points will be used in current view pair.
     * @param viewId2 id of 2nd view where points will be used in current view pair.
     * @param samples1 samples containing points of interest for 1st view to test in
     *                 current pair.
     * @param samples2 samples containing points of interest for 2nd view to test in
     *                 current pair.
     * @param matches instance where matches must be stored.
     */
    void onRequestMatches(R reconstructor, int viewId1, int viewId2,
                          List<Sample2D> samples1, List<Sample2D> samples2,
                          List<MatchedSamples> matches);

    /**
     * Called when a fundamental matrix relating a pair of views has been estimated.
     * This event can be used to store estimated fundamental matrix relating two views.
     * @param reconstructor reconstructor raising this event.
     * @param viewId1 id of 1st view where points will be used in current view pair.
     * @param viewId2 id of 2nd view where points will be used in current view pair.
     * @param estimatedFundamentalMatrix estimated fundamental matrix.
     */
    void onFundamentalMatrixEstimated(R reconstructor, int viewId1, int viewId2,
                                      EstimatedFundamentalMatrix estimatedFundamentalMatrix);

    /**
     * Called when cameras for provided matched pair of views have been estimated in an
     * euclidean stratum (up to certain translation and rotation).
     * Implementations using SLAM techniques by mixing additional sensor data (i.e.
     * gyroscope and accelerometer) to estimate scale of each view pair, might also have
     * some inaccuracies in estimated scale.
     * @param reconstructor reconstructor raising this event.
     * @param viewId1 id of previous view (i.e. 1st view).
     * @param viewId2 id of current view (i.e. 2nd view).
     * @param scale estimated scale. When using SLAM this is estimated up to a certain
     *                 accuracy.
     * @param camera1 estimated euclidean camera for previous view (i.e. 1st view).
     * @param camera2 estimated euclidean camera for current view (i.e. 2nd view).
     */
    void onEuclideanCameraPairEstimated(R reconstructor, int viewId1, int viewId2,
                                        double scale, EstimatedCamera camera1,
                                        EstimatedCamera camera2);

    /**
     * Called when reconstructed points have been estimated from a series of 2D matches in a
     * pair of views in an euclidean stratum (up to certain translation and rotation).
     * @param reconstructor reconstructor raising this event.
     * @param viewId1 id of previous view (i.e. 1st view).
     * @param viewId2 id of current view (i.e. 2nd view).
     * @param scale estimated scale. When using SLAM this is estimated up to a certain
     *                 accuracy.
     * @param points reconstructed 3D points in euclidean space.
     */
    void onEuclideanReconstructedPointsEstimated(R reconstructor,
                                                 int viewId1, int viewId2, double scale,
                                                 List<ReconstructedPoint3D> points);

    /**
     * Called when intrinsic parameters are requested for a given view.
     * This is required if at configuration it was indicated that intrinsic parameters are
     * known, so that essential matrix method can be used for scene reconstruction.
     * If intrinsic parameters are unknown, DIAC or DAQ method will be attempted if possible.
     * @param reconstructor reconstructor raising this event.
     * @param viewId id of view whose parameters are requested.
     * @return intrinsic parameters if known, false otherwise.
     */
    PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(R reconstructor, int viewId);

    /**
     * Called when reconstruction starts.
     * @param reconstructor reconstructor raising this event.
     */
    void onStart(R reconstructor);

    /**
     * Called when reconstruction stops.
     * @param reconstructor reconstructor raising this event.
     */
    void onFinish(R reconstructor);

    /**
     * Called when reconstruction is cancelled before it has finished.
     * @param reconstructor reconstructor raising this event.
     */
    void onCancel(R reconstructor);

    /**
     * Called when reconstruction fails.
     * @param reconstructor reconstructor raising this event.
     */
    void onFail(R reconstructor);
}
