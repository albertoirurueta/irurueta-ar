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

import com.irurueta.algebra.Matrix;
import com.irurueta.ar.calibration.estimators.KruppaDualImageOfAbsoluteConicEstimator;
import com.irurueta.ar.calibration.estimators.LMSEImageOfAbsoluteConicEstimator;
import com.irurueta.ar.epipolar.Corrector;
import com.irurueta.ar.epipolar.EpipolarException;
import com.irurueta.ar.epipolar.EssentialMatrix;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.estimators.EightPointsFundamentalMatrixEstimator;
import com.irurueta.ar.epipolar.estimators.FundamentalMatrixEstimatorMethod;
import com.irurueta.ar.epipolar.estimators.FundamentalMatrixRobustEstimator;
import com.irurueta.ar.epipolar.estimators.LMedSFundamentalMatrixRobustEstimator;
import com.irurueta.ar.epipolar.estimators.MSACFundamentalMatrixRobustEstimator;
import com.irurueta.ar.epipolar.estimators.PROMedSFundamentalMatrixRobustEstimator;
import com.irurueta.ar.epipolar.estimators.PROSACFundamentalMatrixRobustEstimator;
import com.irurueta.ar.epipolar.estimators.RANSACFundamentalMatrixRobustEstimator;
import com.irurueta.ar.epipolar.estimators.SevenPointsFundamentalMatrixEstimator;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Transformation2D;
import com.irurueta.geometry.estimators.*;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * Base class in charge of estimating cameras and 3D reconstructed points from sparse
 * image point correspondences for multiple views.
 *
 * @param <C> type of configuration.
 * @param <R> type of re-constructor.
 * @param <L> type of listener.
 */
public abstract class BaseSparseReconstructor<C extends BaseSparseReconstructorConfiguration<C>,
        R extends BaseSparseReconstructor<C, R, L>, L extends BaseSparseReconstructorListener<R>> {

    /**
     * Minimum required number of views.
     */
    public static final int MIN_NUMBER_OF_VIEWS = 2;

    /**
     * Default scale.
     */
    protected static final double DEFAULT_SCALE = 1.0;

    /**
     * Current estimated camera in a metric stratum (i.e. up to scale).
     */
    protected EstimatedCamera currentMetricEstimatedCamera;

    /**
     * Previous estimated camera in a metric stratum (i.e. up to scale).
     */
    protected EstimatedCamera previousMetricEstimatedCamera;

    /**
     * Reconstructed 3D points which still remain active to match next view in a metric stratum (i.e. up
     * to scale).
     */
    protected List<ReconstructedPoint3D> activeMetricReconstructedPoints;

    /**
     * Current estimated scale. This will typically converge to a constant value as more views are
     * processed.
     * The smaller the variance of estimated scale, the more accurate the scale will be.
     */
    protected double currentScale = DEFAULT_SCALE;

    /**
     * Current estimated camera in euclidean stratum (i.e. with actual scale).
     */
    protected EstimatedCamera currentEuclideanEstimatedCamera;

    /**
     * Previous estimated camera in Euclidean stratum (i.e. with actual scale).
     */
    protected EstimatedCamera previousEuclideanEstimatedCamera;

    /**
     * Reconstructed 3D points which still remain active to match next view in Euclidean stratum (i.e.
     * with actual scale).
     */
    protected List<ReconstructedPoint3D> activeEuclideanReconstructedPoints;

    /**
     * Configuration for this re-constructor.
     */
    protected C configuration;

    /**
     * Listener in charge of handling events such as when reconstruction starts, ends,
     * when certain data is needed or when estimation of data has been computed.
     */
    protected L listener;

    /**
     * Indicates whether reconstruction has failed or not.
     */
    protected volatile boolean failed;

    /**
     * Indicates whether reconstruction is running or not.
     */
    protected volatile boolean running;

    /**
     * Current estimated fundamental matrix.
     */
    private EstimatedFundamentalMatrix currentEstimatedFundamentalMatrix;

    /**
     * Indicates whether reconstruction has been cancelled or not.
     */
    private volatile boolean cancelled;

    /**
     * Counter of number of processed views.
     */
    private int viewCount;

    /**
     * Indicates whether reconstruction has finished or not.
     */
    private boolean finished = false;

    /**
     * All samples (tracked and non-tracked) on previous view.
     */
    private List<Sample2D> allPreviousViewSamples;

    /**
     * Tracked samples on previous view.
     */
    private List<Sample2D> previousViewTrackedSamples;

    /**
     * Tracked samples on last processed view (i.e. current view).
     */
    private List<Sample2D> currentViewTrackedSamples;

    /**
     * New samples on las processed view (i.e. current view).
     */
    private List<Sample2D> currentViewNewlySpawnedSamples;

    /**
     * Active matches between current and previous views.
     */
    private final List<MatchedSamples> matches = new ArrayList<>();

    /**
     * ID of previous view.
     */
    private int previousViewId;

    /**
     * ID of current view.
     */
    private int currentViewId;

    /**
     * Constructor.
     *
     * @param configuration configuration for this re-constructor.
     * @param listener      listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not provided.
     */
    protected BaseSparseReconstructor(final C configuration, final L listener) {
        if (configuration == null || listener == null) {
            throw new NullPointerException();
        }
        this.configuration = configuration;
        this.listener = listener;
    }

    /**
     * Gets configuration for this re-constructor.
     *
     * @return configuration for this re-constructor.
     */
    public C getConfiguration() {
        return configuration;
    }

    /**
     * Gets listener in charge of handling events such as when reconstruction starts,
     * ends, when certain data is needed or when estimation of data has been computed.
     *
     * @return listener in charge of handling events.
     */
    public L getListener() {
        return listener;
    }

    /**
     * Indicates whether reconstruction is running or not.
     *
     * @return true if reconstruction is running, false if reconstruction has stopped
     * for any reason.
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Indicates whether reconstruction has been cancelled or not.
     *
     * @return true if reconstruction has been cancelled, false otherwise.
     */
    public boolean isCancelled() {
        return cancelled;
    }

    /**
     * Indicates whether reconstruction has failed or not.
     *
     * @return true if reconstruction has failed, false otherwise.
     */
    public boolean hasFailed() {
        return failed;
    }

    /**
     * Indicates whether the reconstruction has finished.
     *
     * @return true if reconstruction has finished, false otherwise.
     */
    public boolean isFinished() {
        return finished;
    }

    /**
     * Gets counter of number of processed views.
     *
     * @return counter of number of processed views.
     */
    public int getViewCount() {
        return viewCount;
    }

    /**
     * Gets estimated fundamental matrix for current view.
     * This fundamental matrix relates current view with the previously processed one.
     *
     * @return current estimated fundamental matrix.
     */
    public EstimatedFundamentalMatrix getCurrentEstimatedFundamentalMatrix() {
        return currentEstimatedFundamentalMatrix;
    }

    /**
     * Gets estimated metric camera for current view (i.e. up to scale).
     *
     * @return current estimated metric camera.
     */
    public EstimatedCamera getCurrentMetricEstimatedCamera() {
        return currentMetricEstimatedCamera;
    }

    /**
     * Gets estimated camera for previous view (i.e. up to scale).
     *
     * @return previous estimated metric camera.
     */
    public EstimatedCamera getPreviousMetricEstimatedCamera() {
        return previousMetricEstimatedCamera;
    }

    /**
     * Gets estimated euclidean camera for current view (i.e. with actual scale).
     *
     * @return current estimated euclidean camera.
     */
    public EstimatedCamera getCurrentEuclideanEstimatedCamera() {
        return currentEuclideanEstimatedCamera;
    }

    /**
     * Gets estimated Euclidean camera for previous view (i.e. with actual scale).
     *
     * @return previous estimated euclidean camera.
     */
    public EstimatedCamera getPreviousEuclideanEstimatedCamera() {
        return previousEuclideanEstimatedCamera;
    }

    /**
     * Gets metric reconstructed 3D points (i.e. up to scale) which still remain active to match next view.
     *
     * @return active metric reconstructed 3D points.
     */
    public List<ReconstructedPoint3D> getActiveMetricReconstructedPoints() {
        return activeMetricReconstructedPoints;
    }

    /**
     * Gets Euclidean reconstructed 3D points (i.e. with actual scale) which still remain active to match
     * next view.
     *
     * @return active euclidean reconstructed 3D points.
     */
    public List<ReconstructedPoint3D> getActiveEuclideanReconstructedPoints() {
        return activeEuclideanReconstructedPoints;
    }

    /**
     * Gets current estimated scale. This will typically converge to a constant value as more views are
     * processed.
     * The smaller the variance of estimated scale, the more accurate the scale will be.
     *
     * @return current estimated scale.
     */
    public double getCurrentScale() {
        return currentScale;
    }

    /**
     * Gets tracked samples on previous view.
     *
     * @return tracked samples on previous view.
     */
    public List<Sample2D> getPreviousViewTrackedSamples() {
        return previousViewTrackedSamples;
    }

    /**
     * Gets tracked samples (from previous view) on current view.
     *
     * @return tracked samples on current view
     */
    public List<Sample2D> getCurrentViewTrackedSamples() {
        return currentViewTrackedSamples;
    }

    /**
     * Gets new samples (not tracked) on current view.
     *
     * @return new samples on current view.
     */
    public List<Sample2D> getCurrentViewNewlySpawnedSamples() {
        return currentViewNewlySpawnedSamples;
    }

    /**
     * Process one view of all the available data during the reconstruction.
     * This method can be called multiple times instead of {@link #start()} to build the
     * reconstruction step by step, one view at a time.
     * This method is useful when data is gathered on real time from a camera and the
     * number of views is unknown.
     *
     * @return true if more views can be processed, false when reconstruction has finished.
     */
    public boolean processOneView() {
        if (viewCount == 0) {
            if (running) {
                // already started
                return true;
            }

            reset();
            running = true;

            //noinspection unchecked
            listener.onStart((R) this);
        }

        //noinspection unchecked
        if (!listener.hasMoreViewsAvailable((R) this)) {
            //noinspection unchecked
            listener.onFinish((R) this);
            running = false;
            finished = true;
            return false;
        }

        previousViewTrackedSamples = new ArrayList<>();
        currentViewTrackedSamples = new ArrayList<>();
        currentViewNewlySpawnedSamples = new ArrayList<>();
        //noinspection unchecked
        listener.onRequestSamples((R) this, previousViewId, viewCount, previousViewTrackedSamples,
                currentViewTrackedSamples, currentViewNewlySpawnedSamples);

        boolean processed;
        if (isFirstView()) {
            currentEstimatedFundamentalMatrix = null;
            // for first view we simply keep samples (if enough are provided)
            processed = processFirstView();
        } else {

            if (isSecondView()) {
                // for second view, check that we have enough samples
                processed = processSecondView();
            } else {
                processed = processAdditionalView();
            }
        }

        if (processed) {
            viewCount++;
        }

        if (cancelled) {
            //noinspection unchecked
            listener.onCancel((R) this);
        }

        return !finished;
    }

    /**
     * Indicates whether current view is the first view.
     *
     * @return true if current view is the first view, false otherwise.
     */
    public boolean isFirstView() {
        return viewCount == 0 && (previousViewTrackedSamples == null || previousViewTrackedSamples.isEmpty());
    }

    /**
     * Indicates whether current view is the second view.
     *
     * @return true if current view is the second view, false otherwise.
     */
    public boolean isSecondView() {
        return !isFirstView() && currentEstimatedFundamentalMatrix == null;
    }

    /**
     * Indicates whether current view is an additional view.
     *
     * @return true if current view is an additional view, false otherwise.
     */
    public boolean isAdditionalView() {
        return !isFirstView() && !isSecondView();
    }

    /**
     * Starts reconstruction of all available data to reconstruct the whole scene.
     * If reconstruction has already started and is running, calling this method has
     * no effect.
     * This method is useful when all data is available before starting the reconstruction.
     */
    public void start() {
        while (processOneView()) {
            if (cancelled) {
                break;
            }
        }
    }

    /**
     * Cancels reconstruction.
     * If reconstruction has already been cancelled, calling this method has no effect.
     */
    public void cancel() {
        if (cancelled) {
            // already cancelled
            return;
        }

        cancelled = true;
    }

    /**
     * Resets this instance so that a reconstruction can be started from the beginning without cancelling
     * current one.
     */
    public void reset() {
        if (previousViewTrackedSamples != null) {
            previousViewTrackedSamples.clear();
        }
        if (currentViewTrackedSamples != null) {
            currentViewTrackedSamples.clear();
        }
        if (currentViewNewlySpawnedSamples != null) {
            currentViewNewlySpawnedSamples.clear();
        }
        matches.clear();

        cancelled = failed = false;
        viewCount = 0;
        running = false;

        currentEstimatedFundamentalMatrix = null;
        currentMetricEstimatedCamera = previousMetricEstimatedCamera = null;
        activeMetricReconstructedPoints = null;
        currentScale = DEFAULT_SCALE;
        currentEuclideanEstimatedCamera = previousEuclideanEstimatedCamera = null;
        activeEuclideanReconstructedPoints = null;

        previousViewId = 0;
        currentViewId = 0;

        finished = false;
    }

    /**
     * Called when processing one frame is successfully finished. This can be done to estimate scale on
     * those implementations where scale can be measured or is already known.
     *
     * @param isInitialPairOfViews true if initial pair of views is being processed, false otherwise.
     * @return true if post-processing succeeded, false otherwise.
     */
    protected abstract boolean postProcessOne(final boolean isInitialPairOfViews);

    /**
     * Processes data for first view.
     *
     * @return true if view was successfully processed, false otherwise.
     */
    private boolean processFirstView() {
        if (hasEnoughSamplesForFundamentalMatrixEstimation(currentViewTrackedSamples)) {
            //noinspection unchecked
            listener.onSamplesAccepted((R) this, viewCount, previousViewTrackedSamples, currentViewTrackedSamples);
            if (allPreviousViewSamples == null) {
                allPreviousViewSamples = new ArrayList<>();
            } else {
                allPreviousViewSamples.clear();
            }

            allPreviousViewSamples.addAll(currentViewTrackedSamples);
            allPreviousViewSamples.addAll(currentViewNewlySpawnedSamples);

            previousViewTrackedSamples = currentViewTrackedSamples;
            previousViewId = viewCount;
            return true;
        } else {
            //noinspection unchecked
            listener.onSamplesRejected((R) this, viewCount, previousViewTrackedSamples, currentViewTrackedSamples);
            return false;
        }
    }

    /**
     * Processes data for second view.
     *
     * @return true if view was successfully processed, false otherwise.
     */
    private boolean processSecondView() {
        if (hasEnoughSamplesForFundamentalMatrixEstimation(currentViewTrackedSamples)) {

            // find matches
            matches.clear();

            // matching is up to listener implementation
            //noinspection unchecked
            listener.onRequestMatches((R) this, allPreviousViewSamples, previousViewTrackedSamples,
                    currentViewTrackedSamples, previousViewId, viewCount, matches);

            if (hasEnoughMatchesForFundamentalMatrixEstimation(matches)) {
                // if enough matches are retrieved, attempt to compute
                // fundamental matrix
                if ((configuration.isGeneralSceneAllowed()
                        && estimateFundamentalMatrix(matches, previousViewId, viewCount, true))
                        || (configuration.isPlanarSceneAllowed()
                        && estimatePlanarFundamentalMatrix(matches, previousViewId, viewCount, true))) {
                    // fundamental matrix could be estimated
                    //noinspection unchecked
                    listener.onSamplesAccepted((R) this, viewCount, previousViewTrackedSamples,
                            currentViewTrackedSamples);

                    allPreviousViewSamples.clear();
                    allPreviousViewSamples.addAll(currentViewTrackedSamples);
                    allPreviousViewSamples.addAll(currentViewNewlySpawnedSamples);

                    previousViewTrackedSamples = currentViewTrackedSamples;
                    previousViewId = currentViewId;
                    currentViewId = viewCount;

                    //noinspection unchecked
                    listener.onFundamentalMatrixEstimated((R) this, currentEstimatedFundamentalMatrix);

                    if (estimateInitialCamerasAndPoints()) {
                        // cameras and points have been estimated
                        //noinspection unchecked
                        listener.onMetricCameraEstimated((R) this, previousViewId, currentViewId,
                                previousMetricEstimatedCamera, currentMetricEstimatedCamera);
                        //noinspection unchecked
                        listener.onMetricReconstructedPointsEstimated((R) this, matches,
                                activeMetricReconstructedPoints);

                        if (!postProcessOne(true)) {
                            // something failed
                            failed = true;
                            //noinspection unchecked
                            listener.onFail((R) this);
                            return false;
                        } else {
                            // post-processing succeeded
                            //noinspection unchecked
                            listener.onEuclideanCameraEstimated((R) this, previousViewId, currentViewId, currentScale,
                                    previousEuclideanEstimatedCamera, currentEuclideanEstimatedCamera);
                            //noinspection unchecked
                            listener.onEuclideanReconstructedPointsEstimated((R) this, currentScale,
                                    activeEuclideanReconstructedPoints);
                            return true;
                        }
                    } else {
                        // initial cameras failed
                        failed = true;
                        //noinspection unchecked
                        listener.onFail((R) this);
                        return false;
                    }
                } else {
                    // estimation of fundamental matrix failed
                    //noinspection unchecked
                    listener.onSamplesRejected((R) this, viewCount, previousViewTrackedSamples,
                            currentViewTrackedSamples);
                    return false;
                }
            }
        }

        //noinspection unchecked
        listener.onSamplesRejected((R) this, viewCount, previousViewTrackedSamples, currentViewTrackedSamples);
        return false;
    }

    /**
     * Processes data for one additional view.
     *
     * @return true if view was successfully processed, false otherwise.
     */
    private boolean processAdditionalView() {
        // find matches
        matches.clear();

        //noinspection unchecked
        listener.onRequestMatches((R) this, allPreviousViewSamples, previousViewTrackedSamples,
                currentViewTrackedSamples, currentViewId, viewCount, matches);

        final var points3D = new ArrayList<Point3D>();
        final var points2D = new ArrayList<Point2D>();
        final var qualityScores = setUpCameraEstimatorMatches(points3D, points2D);
        var samplesRejected = false;

        if (hasEnoughSamplesForCameraEstimation(points3D, points2D) && hasEnoughMatchesForCameraEstimation(matches)) {
            // enough matches available.
            PinholeCamera currentCamera = null;
            Matrix currentCameraCovariance = null;
            if (configuration.getUseEPnPForAdditionalCamerasEstimation()) {
                // use EPnP for additional cameras' estimation.
                // EPnP requires knowledge of camera intrinsics

                PinholeCameraIntrinsicParameters intrinsicParameters = null;
                if ((configuration.getUseDAQForAdditionalCamerasIntrinsics()
                        || configuration.getUseDIACForAdditionalCamerasIntrinsics())
                        && hasEnoughMatchesForFundamentalMatrixEstimation(matches)) {

                    // compute fundamental matrix to estimate intrinsics
                    if ((configuration.isGeneralSceneAllowed()
                            && estimateFundamentalMatrix(matches, currentViewId, viewCount, false))
                            || (configuration.isPlanarSceneAllowed()
                            && estimatePlanarFundamentalMatrix(matches, currentViewId, viewCount,
                            false))) {
                        // fundamental matrix could be estimated
                        //noinspection unchecked
                        listener.onFundamentalMatrixEstimated((R) this, currentEstimatedFundamentalMatrix);

                        // use fundamental matrix to estimate intrinsics using DIAC or DAQ
                        if (configuration.getUseDIACForAdditionalCamerasIntrinsics()) {
                            intrinsicParameters = estimateIntrinsicsDIAC();
                        } else if (configuration.getUseDAQForAdditionalCamerasIntrinsics()) {
                            intrinsicParameters = estimateIntrinsicsDAQ();
                        }

                    } else {
                        // fundamental matrix estimation failed

                        //noinspection unchecked
                        listener.onSamplesRejected((R) this, viewCount, previousViewTrackedSamples,
                                currentViewTrackedSamples);
                        return false;
                    }

                } else if (configuration.getAdditionalCamerasIntrinsics() != null) {
                    // use configuration provided intrinsics
                    intrinsicParameters = configuration.getAdditionalCamerasIntrinsics();

                    if (intrinsicParameters == null) {
                        // something failed or bad configuration
                        failed = true;
                        //noinspection unchecked
                        listener.onFail((R) this);
                        return false;
                    }
                }

                try {
                    if (intrinsicParameters != null) {
                        // use EPnP for additional cameras estimation
                        final var cameraEstimator = EPnPPointCorrespondencePinholeCameraRobustEstimator.create(
                                intrinsicParameters, points3D, points2D, qualityScores,
                                configuration.getAdditionalCamerasRobustEstimationMethod());
                        cameraEstimator.setPlanarConfigurationAllowed(
                                configuration.getAdditionalCamerasAllowPlanarConfiguration());
                        cameraEstimator.setNullspaceDimension2Allowed(
                                configuration.getAdditionalCamerasAllowNullspaceDimension2());
                        cameraEstimator.setNullspaceDimension3Allowed(
                                configuration.getAdditionalCamerasAllowNullspaceDimension3());
                        cameraEstimator.setPlanarThreshold(configuration.getAdditionalCamerasPlanarThreshold());
                        cameraEstimator.setResultRefined(configuration.areAdditionalCamerasRefined());
                        cameraEstimator.setCovarianceKept(configuration.isAdditionalCamerasCovarianceKept());
                        cameraEstimator.setFastRefinementUsed(configuration.getAdditionalCamerasUseFastRefinement());
                        cameraEstimator.setConfidence(configuration.getAdditionalCamerasConfidence());
                        cameraEstimator.setMaxIterations(configuration.getAdditionalCamerasMaxIterations());

                        switch (configuration.getAdditionalCamerasRobustEstimationMethod()) {
                            case LMEDS:
                                ((LMedSEPnPPointCorrespondencePinholeCameraRobustEstimator) cameraEstimator)
                                        .setStopThreshold(configuration.getAdditionalCamerasThreshold());
                                break;
                            case MSAC:
                                ((MSACEPnPPointCorrespondencePinholeCameraRobustEstimator) cameraEstimator)
                                        .setThreshold(configuration.getAdditionalCamerasThreshold());
                                break;
                            case PROMEDS:
                                ((PROMedSEPnPPointCorrespondencePinholeCameraRobustEstimator) cameraEstimator)
                                        .setStopThreshold(configuration.getAdditionalCamerasThreshold());
                                break;
                            case PROSAC:
                                var prosacCameraEstimator =
                                        (PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator) cameraEstimator;
                                prosacCameraEstimator.setThreshold(configuration.getAdditionalCamerasThreshold());
                                prosacCameraEstimator.setComputeAndKeepInliersEnabled(
                                        configuration.getAdditionalCamerasComputeAndKeepInliers());
                                prosacCameraEstimator.setComputeAndKeepResidualsEnabled(
                                        configuration.getAdditionalCamerasComputeAndKeepResiduals());
                                break;
                            case RANSAC:
                                var ransacCameraEstimator =
                                        (RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator) cameraEstimator;
                                ransacCameraEstimator.setThreshold(configuration.getAdditionalCamerasThreshold());
                                ransacCameraEstimator.setComputeAndKeepInliersEnabled(
                                        configuration.getAdditionalCamerasComputeAndKeepInliers());
                                ransacCameraEstimator.setComputeAndKeepResidualsEnabled(
                                        configuration.getAdditionalCamerasComputeAndKeepResiduals());
                                break;
                            default:
                                break;
                        }

                        cameraEstimator.setSuggestSkewnessValueEnabled(
                                configuration.isAdditionalCamerasSuggestSkewnessValueEnabled());
                        cameraEstimator.setSuggestedSkewnessValue(
                                configuration.getAdditionalCamerasSuggestedSkewnessValue());

                        cameraEstimator.setSuggestHorizontalFocalLengthEnabled(
                                configuration.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled());
                        cameraEstimator.setSuggestedHorizontalFocalLengthValue(
                                configuration.getAdditionalCamerasSuggestedHorizontalFocalLengthValue());

                        cameraEstimator.setSuggestVerticalFocalLengthEnabled(
                                configuration.isAdditionalCamerasSuggestVerticalFocalLengthEnabled());
                        cameraEstimator.setSuggestedVerticalFocalLengthValue(
                                configuration.getAdditionalCamerasSuggestedVerticalFocalLengthValue());

                        cameraEstimator.setSuggestAspectRatioEnabled(
                                configuration.isAdditionalCamerasSuggestAspectRatioEnabled());
                        cameraEstimator.setSuggestedAspectRatioValue(
                                configuration.getAdditionalCamerasSuggestedAspectRatioValue());

                        cameraEstimator.setSuggestPrincipalPointEnabled(
                                configuration.isAdditionalCamerasSuggestPrincipalPointEnabled());
                        cameraEstimator.setSuggestedPrincipalPointValue(
                                configuration.getAdditionalCamerasSuggestedPrincipalPointValue());

                        currentCamera = cameraEstimator.estimate();
                        currentCameraCovariance = cameraEstimator.getCovariance();

                        //noinspection unchecked
                        listener.onSamplesAccepted((R) this, viewCount, previousViewTrackedSamples,
                                currentViewTrackedSamples);

                        allPreviousViewSamples.clear();
                        allPreviousViewSamples.addAll(currentViewTrackedSamples);
                        allPreviousViewSamples.addAll(currentViewNewlySpawnedSamples);

                        previousViewTrackedSamples = currentViewTrackedSamples;
                        previousViewId = currentViewId;
                        currentViewId = viewCount;
                    }

                } catch (final Exception e) {
                    // camera estimation failed
                    samplesRejected = true;
                }

            } else if (configuration.getUseUPnPForAdditionalCamerasEstimation()) {

                try {
                    // use UPnP for additional cameras estimation
                    final var cameraEstimator = UPnPPointCorrespondencePinholeCameraRobustEstimator.create(points3D,
                            points2D, qualityScores, configuration.getAdditionalCamerasRobustEstimationMethod());
                    cameraEstimator.setPlanarConfigurationAllowed(
                            configuration.getAdditionalCamerasAllowPlanarConfiguration());
                    cameraEstimator.setNullspaceDimension2Allowed(
                            configuration.getAdditionalCamerasAllowNullspaceDimension2());
                    cameraEstimator.setPlanarThreshold(configuration.getAdditionalCamerasPlanarThreshold());
                    cameraEstimator.setResultRefined(configuration.areAdditionalCamerasRefined());
                    cameraEstimator.setCovarianceKept(configuration.isAdditionalCamerasCovarianceKept());
                    cameraEstimator.setFastRefinementUsed(configuration.getAdditionalCamerasUseFastRefinement());
                    cameraEstimator.setConfidence(configuration.getAdditionalCamerasConfidence());
                    cameraEstimator.setMaxIterations(configuration.getAdditionalCamerasMaxIterations());

                    switch (configuration.getAdditionalCamerasRobustEstimationMethod()) {
                        case LMEDS:
                            ((LMedSUPnPPointCorrespondencePinholeCameraRobustEstimator) cameraEstimator)
                                    .setStopThreshold(configuration.getAdditionalCamerasThreshold());
                            break;
                        case MSAC:
                            ((MSACUPnPPointCorrespondencePinholeCameraRobustEstimator) cameraEstimator)
                                    .setThreshold(configuration.getAdditionalCamerasThreshold());
                            break;
                        case PROMEDS:
                            ((PROMedSUPnPPointCorrespondencePinholeCameraRobustEstimator) cameraEstimator)
                                    .setStopThreshold(configuration.getAdditionalCamerasThreshold());
                            break;
                        case PROSAC:
                            var prosacCameraEstimator =
                                    (PROSACUPnPPointCorrespondencePinholeCameraRobustEstimator) cameraEstimator;
                            prosacCameraEstimator.setThreshold(configuration.getAdditionalCamerasThreshold());
                            prosacCameraEstimator.setComputeAndKeepInliersEnabled(
                                    configuration.getAdditionalCamerasComputeAndKeepInliers());
                            prosacCameraEstimator.setComputeAndKeepResidualsEnabled(
                                    configuration.getAdditionalCamerasComputeAndKeepResiduals());
                            break;
                        case RANSAC:
                            var ransacCameraEstimator =
                                    (RANSACUPnPPointCorrespondencePinholeCameraRobustEstimator) cameraEstimator;
                            ransacCameraEstimator.setThreshold(configuration.getAdditionalCamerasThreshold());
                            ransacCameraEstimator.setComputeAndKeepInliersEnabled(
                                    configuration.getAdditionalCamerasComputeAndKeepInliers());
                            ransacCameraEstimator.setComputeAndKeepResidualsEnabled(
                                    configuration.getAdditionalCamerasComputeAndKeepResiduals());
                            break;
                        default:
                            break;
                    }

                    cameraEstimator.setSkewness(configuration.getAdditionalCamerasSkewness());
                    cameraEstimator.setHorizontalPrincipalPoint(
                            configuration.getAdditionalCamerasHorizontalPrincipalPoint());
                    cameraEstimator.setVerticalPrincipalPoint(
                            configuration.getAdditionalCamerasVerticalPrincipalPoint());

                    cameraEstimator.setSuggestSkewnessValueEnabled(
                            configuration.isAdditionalCamerasSuggestSkewnessValueEnabled());
                    cameraEstimator.setSuggestedSkewnessValue(
                            configuration.getAdditionalCamerasSuggestedSkewnessValue());

                    cameraEstimator.setSuggestHorizontalFocalLengthEnabled(
                            configuration.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled());
                    cameraEstimator.setSuggestedHorizontalFocalLengthValue(
                            configuration.getAdditionalCamerasSuggestedHorizontalFocalLengthValue());

                    cameraEstimator.setSuggestVerticalFocalLengthEnabled(
                            configuration.isAdditionalCamerasSuggestVerticalFocalLengthEnabled());
                    cameraEstimator.setSuggestedVerticalFocalLengthValue(
                            configuration.getAdditionalCamerasSuggestedVerticalFocalLengthValue());

                    cameraEstimator.setSuggestAspectRatioEnabled(
                            configuration.isAdditionalCamerasSuggestAspectRatioEnabled());
                    cameraEstimator.setSuggestedAspectRatioValue(
                            configuration.getAdditionalCamerasSuggestedAspectRatioValue());

                    cameraEstimator.setSuggestPrincipalPointEnabled(
                            configuration.isAdditionalCamerasSuggestPrincipalPointEnabled());
                    cameraEstimator.setSuggestedPrincipalPointValue(
                            configuration.getAdditionalCamerasSuggestedPrincipalPointValue());

                    currentCamera = cameraEstimator.estimate();
                    currentCameraCovariance = cameraEstimator.getCovariance();

                    //noinspection unchecked
                    listener.onSamplesAccepted((R) this, viewCount, previousViewTrackedSamples,
                            currentViewTrackedSamples);

                    allPreviousViewSamples.clear();
                    allPreviousViewSamples.addAll(currentViewTrackedSamples);
                    allPreviousViewSamples.addAll(currentViewNewlySpawnedSamples);

                    previousViewTrackedSamples = currentViewTrackedSamples;
                    previousViewId = currentViewId;
                    currentViewId = viewCount;

                } catch (final Exception e) {
                    // camera estimation failed
                    samplesRejected = true;
                }

            } else {

                try {
                    // use DLT for additional cameras estimation
                    final var cameraEstimator = DLTPointCorrespondencePinholeCameraRobustEstimator.create(points3D,
                            points2D, qualityScores, configuration.getAdditionalCamerasRobustEstimationMethod());
                    cameraEstimator.setResultRefined(configuration.areAdditionalCamerasRefined());
                    cameraEstimator.setCovarianceKept(configuration.isAdditionalCamerasCovarianceKept());
                    cameraEstimator.setFastRefinementUsed(configuration.getAdditionalCamerasUseFastRefinement());
                    cameraEstimator.setConfidence(configuration.getAdditionalCamerasConfidence());
                    cameraEstimator.setMaxIterations(configuration.getAdditionalCamerasMaxIterations());

                    switch (configuration.getAdditionalCamerasRobustEstimationMethod()) {
                        case LMEDS:
                            ((LMedSDLTPointCorrespondencePinholeCameraRobustEstimator) cameraEstimator)
                                    .setStopThreshold(configuration.getAdditionalCamerasThreshold());
                            break;
                        case MSAC:
                            ((MSACDLTPointCorrespondencePinholeCameraRobustEstimator) cameraEstimator)
                                    .setThreshold(configuration.getAdditionalCamerasThreshold());
                            break;
                        case PROMEDS:
                            ((PROMedSDLTPointCorrespondencePinholeCameraRobustEstimator) cameraEstimator)
                                    .setStopThreshold(configuration.getAdditionalCamerasThreshold());
                            break;
                        case PROSAC:
                            var prosacCameraEstimator =
                                    (PROSACDLTPointCorrespondencePinholeCameraRobustEstimator) cameraEstimator;
                            prosacCameraEstimator.setThreshold(configuration.getAdditionalCamerasThreshold());
                            prosacCameraEstimator.setComputeAndKeepInliersEnabled(
                                    configuration.getAdditionalCamerasComputeAndKeepInliers());
                            prosacCameraEstimator.setComputeAndKeepResidualsEnabled(
                                    configuration.getAdditionalCamerasComputeAndKeepResiduals());
                            break;
                        case RANSAC:
                            var ransacCameraEstimator =
                                    (RANSACDLTPointCorrespondencePinholeCameraRobustEstimator) cameraEstimator;
                            ransacCameraEstimator.setThreshold(configuration.getAdditionalCamerasThreshold());
                            ransacCameraEstimator.setComputeAndKeepInliersEnabled(
                                    configuration.getAdditionalCamerasComputeAndKeepInliers());
                            ransacCameraEstimator.setComputeAndKeepResidualsEnabled(
                                    configuration.getAdditionalCamerasComputeAndKeepResiduals());
                            break;
                        default:
                            break;
                    }

                    cameraEstimator.setSuggestSkewnessValueEnabled(
                            configuration.isAdditionalCamerasSuggestSkewnessValueEnabled());
                    cameraEstimator.setSuggestedSkewnessValue(
                            configuration.getAdditionalCamerasSuggestedSkewnessValue());

                    cameraEstimator.setSuggestHorizontalFocalLengthEnabled(
                            configuration.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled());
                    cameraEstimator.setSuggestedHorizontalFocalLengthValue(
                            configuration.getAdditionalCamerasSuggestedHorizontalFocalLengthValue());

                    cameraEstimator.setSuggestVerticalFocalLengthEnabled(
                            configuration.isAdditionalCamerasSuggestVerticalFocalLengthEnabled());
                    cameraEstimator.setSuggestedVerticalFocalLengthValue(
                            configuration.getAdditionalCamerasSuggestedVerticalFocalLengthValue());

                    cameraEstimator.setSuggestAspectRatioEnabled(
                            configuration.isAdditionalCamerasSuggestAspectRatioEnabled());
                    cameraEstimator.setSuggestedAspectRatioValue(
                            configuration.getAdditionalCamerasSuggestedAspectRatioValue());

                    cameraEstimator.setSuggestPrincipalPointEnabled(
                            configuration.isAdditionalCamerasSuggestPrincipalPointEnabled());
                    cameraEstimator.setSuggestedPrincipalPointValue(
                            configuration.getAdditionalCamerasSuggestedPrincipalPointValue());

                    currentCamera = cameraEstimator.estimate();
                    currentCameraCovariance = cameraEstimator.getCovariance();

                    //noinspection unchecked
                    listener.onSamplesAccepted((R) this, viewCount, previousViewTrackedSamples,
                            currentViewTrackedSamples);

                    allPreviousViewSamples.clear();
                    allPreviousViewSamples.addAll(currentViewTrackedSamples);
                    allPreviousViewSamples.addAll(currentViewNewlySpawnedSamples);

                    previousViewTrackedSamples = currentViewTrackedSamples;
                    previousViewId = currentViewId;
                    currentViewId = viewCount;

                } catch (final Exception e) {
                    // camera estimation failed
                    samplesRejected = true;
                }
            }

            if (!samplesRejected) {
                previousMetricEstimatedCamera = currentMetricEstimatedCamera;

                currentMetricEstimatedCamera = new EstimatedCamera();
                currentMetricEstimatedCamera.setCamera(currentCamera);
                currentMetricEstimatedCamera.setViewId(currentViewId);
                currentMetricEstimatedCamera.setCovariance(currentCameraCovariance);

                // notify camera estimation
                //noinspection unchecked
                listener.onMetricCameraEstimated((R) this, previousViewId, currentViewId, previousMetricEstimatedCamera,
                        currentMetricEstimatedCamera);

                // reconstruct all matches and refine existing reconstructed points
                reconstructAndRefineMatches();

                // notify reconstruction update
                //noinspection unchecked
                listener.onMetricReconstructedPointsEstimated((R) this, matches, activeMetricReconstructedPoints);

                if (!postProcessOne(false)) {
                    // something failed
                    failed = true;
                    //noinspection unchecked
                    listener.onFail((R) this);
                    return false;
                } else {
                    // post-processing succeeded
                    //noinspection unchecked
                    listener.onEuclideanCameraEstimated((R) this, previousViewId, currentViewId, currentScale,
                            previousEuclideanEstimatedCamera, currentEuclideanEstimatedCamera);
                    //noinspection unchecked
                    listener.onEuclideanReconstructedPointsEstimated((R) this, currentScale,
                            activeEuclideanReconstructedPoints);
                    return true;
                }
            }
        }

        //noinspection unchecked
        listener.onSamplesRejected((R) this, viewCount, previousViewTrackedSamples, currentViewTrackedSamples);
        return false;
    }

    /**
     * Reconstructs new 3D points or refines existing ones taking into account existing matches and estimated cameras
     */
    private void reconstructAndRefineMatches() {
        if (matches.isEmpty()) {
            return;
        }

        try {
            RobustSinglePoint3DTriangulator robustTriangulator = null;
            SinglePoint3DTriangulator triangulator = null;
            var qualityScoresRequired = false;
            if (configuration.getAdditionalCamerasRobustEstimationMethod() != null) {
                robustTriangulator = RobustSinglePoint3DTriangulator.create(
                        configuration.getAdditionalCamerasRobustEstimationMethod());
                robustTriangulator.setConfidence(configuration.getPointTriangulatorConfidence());
                robustTriangulator.setMaxIterations(configuration.getPointTriangulatorMaxIterations());

                var threshold = configuration.getPointTriangulatorThreshold();
                switch (configuration.getAdditionalCamerasRobustEstimationMethod()) {
                    case LMEDS:
                        ((LMedSRobustSinglePoint3DTriangulator) robustTriangulator).setStopThreshold(threshold);
                        break;
                    case MSAC:
                        ((MSACRobustSinglePoint3DTriangulator) robustTriangulator).setThreshold(threshold);
                        break;
                    case PROMEDS:
                        ((PROMedSRobustSinglePoint3DTriangulator) robustTriangulator).setStopThreshold(threshold);
                        qualityScoresRequired = true;
                        break;
                    case PROSAC:
                        ((PROSACRobustSinglePoint3DTriangulator) robustTriangulator).setThreshold(threshold);
                        qualityScoresRequired = true;
                        break;
                    case RANSAC:
                        ((RANSACRobustSinglePoint3DTriangulator) robustTriangulator).setThreshold(threshold);
                        break;
                    default:
                        break;
                }

            } else {
                if (configuration.isHomogeneousPointTriangulatorUsed()) {
                    triangulator = SinglePoint3DTriangulator.create(
                            Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR);
                } else {
                    triangulator = SinglePoint3DTriangulator.create(
                            Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR);
                }
            }

            activeMetricReconstructedPoints = new ArrayList<>();
            ReconstructedPoint3D reconstructedPoint;
            var matchPos = 0;
            for (final var match : matches) {
                final var samples = match.getSamples();
                final var estimatedCameras = match.getCameras();

                // estimated cameras does not yet contain last estimated camera
                if (samples.length != estimatedCameras.length + 1) {
                    continue;
                }

                final var points = new ArrayList<Point2D>();
                final var cameras = new ArrayList<PinholeCamera>();
                final var validSamples = new BitSet(samples.length);
                PinholeCamera camera = null;
                Point2D point2D;
                var numValid = 0;
                final var samplesLength = samples.length;
                final var samplesLengthMinusOne = samplesLength - 1;
                boolean isLast;
                for (var i = 0; i < samples.length; i++) {
                    isLast = (i == samplesLengthMinusOne);
                    point2D = samples[i].getPoint();

                    if (!isLast) {
                        camera = estimatedCameras[i].getCamera();
                    }

                    if (point2D == null || (camera == null && !isLast)) {
                        validSamples.clear(i);
                    } else {
                        validSamples.set(i);

                        points.add(point2D);
                        if (!isLast) {
                            cameras.add(camera);
                        }

                        numValid++;
                    }
                }

                // also add current camera which is not yet available on estimated cameras array
                cameras.add(currentMetricEstimatedCamera.getCamera());

                if (points.size() < SinglePoint3DTriangulator.MIN_REQUIRED_VIEWS || points.size() != cameras.size()) {
                    // point cannot be triangulated
                    continue;
                }

                Point3D point3D;
                if (robustTriangulator != null) {
                    robustTriangulator.setPointsAndCameras(points, cameras);
                    if (qualityScoresRequired) {
                        // copy quality scores
                        final var qualityScores = new double[numValid];
                        var j = 0;
                        for (var i = 0; i < samples.length; i++) {
                            if (validSamples.get(i)) {
                                qualityScores[j] = samples[i].getQualityScore();
                                j++;
                            }
                        }
                        robustTriangulator.setQualityScores(qualityScores);
                    }

                    point3D = robustTriangulator.triangulate();

                } else if (triangulator != null) {
                    triangulator.setPointsAndCameras(points, cameras);
                    point3D = triangulator.triangulate();

                } else {
                    continue;
                }

                // save triangulated point
                reconstructedPoint = new ReconstructedPoint3D();
                reconstructedPoint.setPoint(point3D);
                reconstructedPoint.setInlier(true);
                reconstructedPoint.setId(String.valueOf(matchPos));
                match.setReconstructedPoint(reconstructedPoint);

                activeMetricReconstructedPoints.add(reconstructedPoint);

                matchPos++;
            }
        } catch (final Exception e) {
            // something failed
            failed = true;
            //noinspection all
            listener.onFail((R) this);
        }
    }

    /**
     * Setups current matched 3D/2D points to estimate a pinhole camera.
     *
     * @param points3D 3D matched points.
     * @param points2D 2D matched points.
     * @return quality scores for matched points.
     */
    private double[] setUpCameraEstimatorMatches(final List<Point3D> points3D, final List<Point2D> points2D) {
        if (matches.isEmpty()) {
            return null;
        }

        points3D.clear();
        points2D.clear();

        final var qualityScoresRequired =
                configuration.getAdditionalCamerasRobustEstimationMethod() == RobustEstimatorMethod.PROSAC
                        || configuration.getAdditionalCamerasRobustEstimationMethod() == RobustEstimatorMethod.PROMEDS;


        int[] positions = null;
        if (qualityScoresRequired) {
            positions = new int[matches.size()];
        }

        var numMatches = 0;
        var i = 0;
        for (final var match : matches) {
            final var samples = match.getSamples();
            final var viewIds = match.getViewIds();
            final var pos = getPositionForViewId(viewIds, viewCount);
            if (pos < 0) {
                continue;
            }
            if (positions != null) {
                positions[i] = pos;
            }

            final var sample = samples[pos];
            final var reconstructedPoint3D = match.getReconstructedPoint();

            if (sample == null || sample.getPoint() == null || reconstructedPoint3D == null
                    || reconstructedPoint3D.getPoint() == null) {
                if (positions != null) {
                    positions[i] = -1;
                }
            } else {
                points2D.add(sample.getPoint());
                points3D.add(reconstructedPoint3D.getPoint());
                numMatches++;
            }

            i++;
        }

        // pick quality scores
        double[] qualityScores = null;
        if (qualityScoresRequired && numMatches > 0) {
            qualityScores = new double[numMatches];
            var j = 0;
            for (i = 0; i < positions.length; i++) {
                if (positions[i] < 0) {
                    continue;
                }

                qualityScores[j] = matches.get(i).getQualityScore();
                j++;
            }
        }

        return qualityScores;
    }

    /**
     * Estimates additional camera intrinsics using DIAC (Dual Image of Absolute Conic) method.
     *
     * @return additional camera intrinsics or null if something fails.
     */
    private PinholeCameraIntrinsicParameters estimateIntrinsicsDIAC() {
        final var fundamentalMatrix = currentEstimatedFundamentalMatrix.getFundamentalMatrix();

        try {
            final var diacEstimator = new KruppaDualImageOfAbsoluteConicEstimator(fundamentalMatrix);
            diacEstimator.setPrincipalPointX(configuration.getAdditionalCamerasHorizontalPrincipalPoint());
            diacEstimator.setPrincipalPointY(configuration.getAdditionalCamerasVerticalPrincipalPoint());
            diacEstimator.setFocalDistanceAspectRatioKnown(true);
            diacEstimator.setFocalDistanceAspectRatio(configuration.getAdditionalCamerasAspectRatio());

            final var diac = diacEstimator.estimate();
            return diac.getIntrinsicParameters();

        } catch (final Exception e) {
            return null;
        }
    }

    /**
     * Estimates additional cameras intrinsics using DAQ (Dual Absolute Quadric) method.
     *
     * @return additional camera intrinsics or null if something fails.
     */
    private PinholeCameraIntrinsicParameters estimateIntrinsicsDAQ() {
        try {
            final var fundamentalMatrix = currentEstimatedFundamentalMatrix.getFundamentalMatrix();
            fundamentalMatrix.normalize();

            final var estimator = new DualAbsoluteQuadricInitialCamerasEstimator(fundamentalMatrix);
            estimator.setAspectRatio(configuration.getInitialCamerasAspectRatio());
            estimator.estimate();

            final var camera = estimator.getEstimatedLeftCamera();
            camera.decompose();
            return camera.getIntrinsicParameters();

        } catch (final Exception e) {
            return null;
        }
    }

    /**
     * Indicates whether there are enough matched points to estimate an additional camera.
     *
     * @param points3D 3D matched points to check.
     * @param points2D 2D matched points to check.
     * @return true if there are enough matched points, false otherwise.
     */
    private boolean hasEnoughSamplesForCameraEstimation(final List<Point3D> points3D, final List<Point2D> points2D) {
        return points3D != null && points2D != null && points3D.size() == points2D.size()
                && hasEnoughSamplesOrMatchesForCameraEstimation(points3D.size());
    }

    /**
     * Indicates whether there are enough matches to estimate an additional camera.
     *
     * @param matches matches to check.
     * @return true if there are enough matches, false otherwise.
     */
    private boolean hasEnoughMatchesForCameraEstimation(final List<MatchedSamples> matches) {
        return hasEnoughSamplesOrMatchesForCameraEstimation(matches != null ? matches.size() : 0);
    }

    /**
     * Indicates whether there are enough matches or samples to estimate an additional
     * camera.
     *
     * @param count number of matches or samples.
     * @return true if there are enough matches or samples, false otherwise.
     */
    private boolean hasEnoughSamplesOrMatchesForCameraEstimation(final int count) {
        if (configuration.getUseDAQForAdditionalCamerasIntrinsics()
                || configuration.getUseDIACForAdditionalCamerasIntrinsics()) {
            // when DAQ or DIAC is required for additional cameras, fundamental matrix
            // also needs to be computed, which requires 7 or 8 matches.
            return hasEnoughSamplesOrMatchesForFundamentalMatrixEstimation(count);
        } else {
            // EPnP, UPnP or DLT is used for additional cameras estimation without fundamental
            // matrix. Only 6 matches are required
            return count >= PointCorrespondencePinholeCameraRobustEstimator.MIN_NUMBER_OF_POINT_CORRESPONDENCES;
        }
    }

    /**
     * Indicates whether there are enough samples to estimate a fundamental matrix.
     *
     * @param samples samples to check.
     * @return true if there are enough samples, false otherwise.
     */
    private boolean hasEnoughSamplesForFundamentalMatrixEstimation(final List<Sample2D> samples) {
        return hasEnoughSamplesOrMatchesForFundamentalMatrixEstimation(samples != null ? samples.size() : 0);
    }

    /**
     * Indicates whether there are enough matches to estimate a fundamental matrix.
     *
     * @param matches matches to check.
     * @return true if there are enough matches, false otherwise.
     */
    private boolean hasEnoughMatchesForFundamentalMatrixEstimation(final List<MatchedSamples> matches) {
        return hasEnoughSamplesOrMatchesForFundamentalMatrixEstimation(matches != null ? matches.size() : 0);
    }

    /**
     * Indicates whether there are enough matches or samples to estimate a fundamental
     * matrix.
     *
     * @param count number of matches or samples.
     * @return true if there are enough matches or samples, false otherwise.
     */
    private boolean hasEnoughSamplesOrMatchesForFundamentalMatrixEstimation(final int count) {
        if (configuration.isGeneralSceneAllowed()) {
            if (configuration.getNonRobustFundamentalMatrixEstimatorMethod()
                    == FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM) {
                return count >= EightPointsFundamentalMatrixEstimator.MIN_REQUIRED_POINTS;
            } else if (configuration.getNonRobustFundamentalMatrixEstimatorMethod()
                    == FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM) {
                return count >= SevenPointsFundamentalMatrixEstimator.MIN_REQUIRED_POINTS;
            }
        } else if (configuration.isPlanarSceneAllowed()) {
            return count >= ProjectiveTransformation2DRobustEstimator.MINIMUM_SIZE;
        }
        return false;
    }

    /**
     * Estimates fundamental matrix for provided matches, when 3D points lay in a general
     * non-degenerate 3D configuration.
     *
     * @param matches              pairs of matches to find fundamental matrix.
     * @param viewId1              id of first view being related by estimated fundamental matrix.
     * @param viewId2              id of second view being related by estimated fundamental matrix.
     * @param isInitialPairOfViews true if fundamental matrix needs to be estimated for the initial
     *                             pair of views, false otherwise.
     * @return true if estimation succeeded, false otherwise.
     */
    private boolean estimateFundamentalMatrix(
            final List<MatchedSamples> matches, final int viewId1, final int viewId2,
            final boolean isInitialPairOfViews) {
        if (matches == null) {
            return false;
        }

        final var count = matches.size();
        final var leftSamples = new ArrayList<Sample2D>(count);
        final var rightSamples = new ArrayList<Sample2D>(count);
        final var leftPoints = new ArrayList<Point2D>(count);
        final var rightPoints = new ArrayList<Point2D>(count);
        final var qualityScores = new double[count];
        final double principalPointX;
        final double principalPointY;
        if (isInitialPairOfViews) {
            if (configuration.getInitialCamerasEstimatorMethod() == InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC
                    || configuration.getInitialCamerasEstimatorMethod()
                    == InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX) {
                principalPointX = configuration.getPrincipalPointX();
                principalPointY = configuration.getPrincipalPointY();
            } else {
                principalPointX = principalPointY = 0.0;
            }
        } else {
            if (configuration.getUseDIACForAdditionalCamerasIntrinsics()
                    || configuration.getUseDAQForAdditionalCamerasIntrinsics()) {
                principalPointX = configuration.getAdditionalCamerasHorizontalPrincipalPoint();
                principalPointY = configuration.getAdditionalCamerasVerticalPrincipalPoint();
            } else {
                principalPointX = principalPointY = 0.0;
            }
        }

        var i = 0;
        for (final var match : matches) {
            final var samples = match.getSamples();
            if (samples.length < MIN_NUMBER_OF_VIEWS) {
                return false;
            }

            final var viewIds = match.getViewIds();
            final var pos1 = getPositionForViewId(viewIds, viewId1);
            if (pos1 < 0) {
                return false;
            }

            final var pos2 = getPositionForViewId(viewIds, viewId2);
            if (pos2 < 0) {
                return false;
            }

            final var leftSample = samples[pos1];
            final var rightSample = samples[pos2];
            final var p1 = leftSample.getPoint();
            final var p2 = rightSample.getPoint();

            leftSamples.add(leftSample);
            rightSamples.add(rightSample);

            final var leftPoint = Point2D.create();
            leftPoint.setInhomogeneousCoordinates(p1.getInhomX() - principalPointX,
                    p1.getInhomY() - principalPointY);
            leftPoints.add(leftPoint);

            final var rightPoint = Point2D.create();
            rightPoint.setInhomogeneousCoordinates(p2.getInhomX() - principalPointX,
                    p2.getInhomY() - principalPointY);
            rightPoints.add(rightPoint);

            qualityScores[i] = match.getQualityScore();
            i++;
        }

        try {
            final var estimator = FundamentalMatrixRobustEstimator.create(leftPoints, rightPoints, qualityScores,
                    configuration.getRobustFundamentalMatrixEstimatorMethod());
            estimator.setNonRobustFundamentalMatrixEstimatorMethod(
                    configuration.getNonRobustFundamentalMatrixEstimatorMethod());
            estimator.setResultRefined(configuration.isFundamentalMatrixRefined());
            estimator.setCovarianceKept(configuration.isFundamentalMatrixCovarianceKept());
            estimator.setConfidence(configuration.getFundamentalMatrixConfidence());
            estimator.setMaxIterations(configuration.getFundamentalMatrixMaxIterations());

            switch (configuration.getRobustFundamentalMatrixEstimatorMethod()) {
                case LMEDS:
                    ((LMedSFundamentalMatrixRobustEstimator) estimator)
                            .setStopThreshold(configuration.getFundamentalMatrixThreshold());
                    break;
                case MSAC:
                    ((MSACFundamentalMatrixRobustEstimator) estimator)
                            .setThreshold(configuration.getFundamentalMatrixThreshold());
                    break;
                case PROMEDS:
                    ((PROMedSFundamentalMatrixRobustEstimator) estimator)
                            .setStopThreshold(configuration.getFundamentalMatrixThreshold());
                    break;
                case PROSAC:
                    var prosacEstimator = (PROSACFundamentalMatrixRobustEstimator) estimator;
                    prosacEstimator.setThreshold(configuration.getFundamentalMatrixThreshold());
                    prosacEstimator.setComputeAndKeepInliersEnabled(
                            configuration.getFundamentalMatrixComputeAndKeepInliers());
                    prosacEstimator.setComputeAndKeepResidualsEnabled(
                            configuration.getFundamentalMatrixComputeAndKeepResiduals());
                    break;
                case RANSAC:
                    var ransacEstimator = (RANSACFundamentalMatrixRobustEstimator) estimator;
                    ransacEstimator.setThreshold(configuration.getFundamentalMatrixThreshold());
                    ransacEstimator.setComputeAndKeepInliersEnabled(
                            configuration.getFundamentalMatrixComputeAndKeepInliers());
                    ransacEstimator.setComputeAndKeepResidualsEnabled(
                            configuration.getFundamentalMatrixComputeAndKeepResiduals());
                    break;
                default:
                    break;
            }

            final var fundamentalMatrix = estimator.estimate();

            currentEstimatedFundamentalMatrix = new EstimatedFundamentalMatrix();
            currentEstimatedFundamentalMatrix.setFundamentalMatrix(fundamentalMatrix);
            currentEstimatedFundamentalMatrix.setViewId1(viewId1);
            currentEstimatedFundamentalMatrix.setViewId2(viewId2);
            currentEstimatedFundamentalMatrix.setCovariance(estimator.getCovariance());

            // determine quality score and inliers
            final var inliersData = estimator.getInliersData();
            if (inliersData != null) {
                final var numInliers = inliersData.getNumInliers();
                final var inliers = inliersData.getInliers();
                final var length = inliers.length();
                var fundamentalMatrixQualityScore = 0.0;
                for (i = 0; i < length; i++) {
                    if (inliers.get(i)) {
                        // inlier
                        fundamentalMatrixQualityScore += qualityScores[i] / numInliers;
                    }
                }
                currentEstimatedFundamentalMatrix.setQualityScore(fundamentalMatrixQualityScore);
                currentEstimatedFundamentalMatrix.setInliers(inliers);
            }

            // store left/right samples
            currentEstimatedFundamentalMatrix.setLeftSamples(leftSamples);
            currentEstimatedFundamentalMatrix.setRightSamples(rightSamples);

            return true;
        } catch (final Exception e) {
            return false;
        }
    }

    /**
     * Estimates fundamental matrix for provided matches, when 3D points lay in a planar 3D scene.
     *
     * @param matches              pairs of matches to find fundamental matrix.
     * @param viewId1              id of first view being related by estimated fundamental matrix.
     * @param viewId2              id of second view being related by estimated fundamental matrix.
     * @param isInitialPairOfViews true if fundamental matrix needs to be estimated for the initial
     *                             pair of views, false otherwise.
     * @return true if estimation succeeded, false otherwise.
     */
    private boolean estimatePlanarFundamentalMatrix(
            final List<MatchedSamples> matches, final int viewId1, final int viewId2,
            final boolean isInitialPairOfViews) {
        if (matches == null) {
            return false;
        }

        final var count = matches.size();
        final var leftSamples = new ArrayList<Sample2D>(count);
        final var rightSamples = new ArrayList<Sample2D>(count);
        final var leftPoints = new ArrayList<Point2D>(count);
        final var rightPoints = new ArrayList<Point2D>(count);
        final var qualityScores = new double[count];
        double principalPointX;
        double principalPointY;
        if (isInitialPairOfViews) {
            if (configuration.getInitialCamerasEstimatorMethod() == InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC
                    || configuration.getInitialCamerasEstimatorMethod()
                    == InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX) {
                principalPointX = configuration.getPrincipalPointX();
                principalPointY = configuration.getPrincipalPointY();
            } else {
                principalPointX = principalPointY = 0.0;
            }
        } else {
            if (configuration.getUseDIACForAdditionalCamerasIntrinsics()
                    || configuration.getUseDAQForAdditionalCamerasIntrinsics()) {
                principalPointX = configuration.getAdditionalCamerasHorizontalPrincipalPoint();
                principalPointY = configuration.getAdditionalCamerasVerticalPrincipalPoint();
            } else {
                principalPointX = principalPointY = 0.0;
            }
        }

        var i = 0;
        for (final var match : matches) {
            final var samples = match.getSamples();
            if (samples.length < MIN_NUMBER_OF_VIEWS) {
                return false;
            }

            final var viewIds = match.getViewIds();
            final var pos1 = getPositionForViewId(viewIds, viewId1);
            if (pos1 < 0) {
                return false;
            }

            final var pos2 = getPositionForViewId(viewIds, viewId2);
            if (pos2 < 0) {
                return false;
            }

            final var leftSample = samples[pos1];
            final var rightSample = samples[pos2];
            final var p1 = leftSample.getPoint();
            final var p2 = rightSample.getPoint();

            leftSamples.add(leftSample);
            rightSamples.add(rightSample);

            final var leftPoint = Point2D.create();
            leftPoint.setInhomogeneousCoordinates(p1.getInhomX() - principalPointX,
                    p1.getInhomY() - principalPointY);
            leftPoints.add(leftPoint);

            final var rightPoint = Point2D.create();
            rightPoint.setInhomogeneousCoordinates(p2.getInhomX() - principalPointX,
                    p2.getInhomY() - principalPointY);
            rightPoints.add(rightPoint);

            qualityScores[i] = match.getQualityScore();
            i++;
        }

        try {
            final var homographyEstimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(
                    configuration.getRobustPlanarHomographyEstimatorMethod());
            homographyEstimator.setResultRefined(configuration.isPlanarHomographyRefined());
            homographyEstimator.setCovarianceKept(configuration.isPlanarHomographyCovarianceKept());
            homographyEstimator.setConfidence(configuration.getPlanarHomographyConfidence());
            homographyEstimator.setMaxIterations(configuration.getPlanarHomographyMaxIterations());

            switch (configuration.getRobustPlanarHomographyEstimatorMethod()) {
                case LMEDS:
                    ((LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator)
                            .setStopThreshold(configuration.getPlanarHomographyThreshold());
                    break;
                case MSAC:
                    ((MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator)
                            .setThreshold(configuration.getPlanarHomographyThreshold());
                    break;
                case PROMEDS:
                    ((PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator)
                            .setStopThreshold(configuration.getPlanarHomographyThreshold());
                    break;
                case PROSAC:
                    final var prosacHomographyEstimator =
                            (PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator;

                    prosacHomographyEstimator.setThreshold(configuration.getPlanarHomographyThreshold());
                    prosacHomographyEstimator.setComputeAndKeepInliersEnabled(
                            configuration.getPlanarHomographyComputeAndKeepInliers());
                    prosacHomographyEstimator.setComputeAndKeepResidualsEnabled(
                            configuration.getPlanarHomographyComputeAndKeepResiduals());
                    break;
                case RANSAC:
                    final var ransacHomographyEstimator =
                            (RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator;

                    ransacHomographyEstimator.setThreshold(configuration.getPlanarHomographyThreshold());
                    ransacHomographyEstimator.setComputeAndKeepInliersEnabled(
                            configuration.getPlanarHomographyComputeAndKeepInliers());
                    ransacHomographyEstimator.setComputeAndKeepResidualsEnabled(
                            configuration.getPlanarHomographyComputeAndKeepResiduals());
                    break;
                default:
                    break;
            }

            final var fundamentalMatrixEstimator = new PlanarBestFundamentalMatrixEstimatorAndReconstructor();
            fundamentalMatrixEstimator.setHomographyEstimator(homographyEstimator);
            fundamentalMatrixEstimator.setLeftAndRightPoints(leftPoints, rightPoints);
            fundamentalMatrixEstimator.setQualityScores(qualityScores);

            var intrinsic1 = configuration.getInitialIntrinsic1();
            var intrinsic2 = configuration.getInitialIntrinsic1();
            if (intrinsic1 == null && intrinsic2 == null) {
                // estimate homography
                final var homography = homographyEstimator.estimate();

                // estimate intrinsic parameters using the Image of Absolute
                // Conic (IAC)
                final var homographies = new ArrayList<Transformation2D>();
                homographies.add(homography);

                final var iacEstimator = new LMSEImageOfAbsoluteConicEstimator(homographies);
                final var iac = iacEstimator.estimate();

                intrinsic1 = intrinsic2 = iac.getIntrinsicParameters();

            } else if (intrinsic1 == null) { //&& intrinsic2 != null
                intrinsic1 = intrinsic2;
            } else if (intrinsic2 == null) { //&& intrinsic1 != null
                intrinsic2 = intrinsic1;
            }
            fundamentalMatrixEstimator.setLeftIntrinsics(intrinsic1);
            fundamentalMatrixEstimator.setRightIntrinsics(intrinsic2);

            fundamentalMatrixEstimator.estimateAndReconstruct();

            final var fundamentalMatrix = fundamentalMatrixEstimator.getFundamentalMatrix();

            currentEstimatedFundamentalMatrix = new EstimatedFundamentalMatrix();
            currentEstimatedFundamentalMatrix.setFundamentalMatrix(fundamentalMatrix);
            currentEstimatedFundamentalMatrix.setViewId1(viewId1);
            currentEstimatedFundamentalMatrix.setViewId2(viewId2);

            // determine quality score and inliers
            final var inliersData = homographyEstimator.getInliersData();
            if (inliersData != null) {
                final var numInliers = inliersData.getNumInliers();
                final var inliers = inliersData.getInliers();
                final var length = inliers.length();
                var fundamentalMatrixQualityScore = 0.0;
                for (i = 0; i < length; i++) {
                    if (inliers.get(i)) {
                        // inlier
                        fundamentalMatrixQualityScore += qualityScores[i] / numInliers;
                    }
                }
                currentEstimatedFundamentalMatrix.setQualityScore(fundamentalMatrixQualityScore);
                currentEstimatedFundamentalMatrix.setInliers(inliers);
            }

            // store left/right samples
            currentEstimatedFundamentalMatrix.setLeftSamples(leftSamples);
            currentEstimatedFundamentalMatrix.setRightSamples(rightSamples);

            return true;
        } catch (final Exception e) {
            return false;
        }
    }

    /**
     * Gets position of a view id within provided array of view id's.
     *
     * @param viewIds array of view IDs where search is done.
     * @param viewId  view id to be searched.
     * @return position where view id is found or -1 if not found.
     */
    private int getPositionForViewId(final int[] viewIds, final int viewId) {
        final var length = viewIds.length;
        for (var i = 0; i < length; i++) {
            if (viewIds[i] == viewId) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Estimates initial cameras and reconstructed points.
     *
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPoints() {
        return switch (configuration.getInitialCamerasEstimatorMethod()) {
            case ESSENTIAL_MATRIX -> estimateInitialCamerasAndPointsEssential();
            case DUAL_IMAGE_OF_ABSOLUTE_CONIC -> estimateInitialCamerasAndPointsDIAC();
            case DUAL_ABSOLUTE_QUADRIC -> estimateInitialCamerasAndPointsDAQ();
            default -> estimateInitialCamerasAndPointsDAQAndEssential();
        };
    }

    /**
     * Estimates initial cameras and reconstructed points using the Dual
     * Absolute Quadric to estimate intrinsic parameters and then use those
     * intrinsic parameters with the essential matrix.
     *
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsDAQAndEssential() {
        try {
            final var fundamentalMatrix = currentEstimatedFundamentalMatrix.getFundamentalMatrix();

            final var estimator = new DualAbsoluteQuadricInitialCamerasEstimator(fundamentalMatrix);
            estimator.setAspectRatio(configuration.getInitialCamerasAspectRatio());
            estimator.estimate();

            final var camera1 = estimator.getEstimatedLeftCamera();
            final var camera2 = estimator.getEstimatedRightCamera();

            camera1.decompose();
            camera2.decompose();

            final var intrinsicZeroPrincipalPoint1 = camera1.getIntrinsicParameters();
            final var intrinsicZeroPrincipalPoint2 = camera2.getIntrinsicParameters();

            final double principalPointX = configuration.getPrincipalPointX();
            final double principalPointY = configuration.getPrincipalPointY();

            final var intrinsic1 = new PinholeCameraIntrinsicParameters(intrinsicZeroPrincipalPoint1);
            intrinsic1.setHorizontalPrincipalPoint(intrinsic1.getHorizontalPrincipalPoint() + principalPointX);
            intrinsic1.setVerticalPrincipalPoint(intrinsic1.getVerticalPrincipalPoint() + principalPointY);

            final var intrinsic2 = new PinholeCameraIntrinsicParameters(intrinsicZeroPrincipalPoint2);
            intrinsic2.setHorizontalPrincipalPoint(intrinsic2.getHorizontalPrincipalPoint() + principalPointX);
            intrinsic2.setVerticalPrincipalPoint(intrinsic2.getVerticalPrincipalPoint() + principalPointY);

            // fix fundamental matrix to account for principal point different
            // from zero
            fixFundamentalMatrix(fundamentalMatrix, intrinsicZeroPrincipalPoint1, intrinsicZeroPrincipalPoint2,
                    intrinsic1, intrinsic2);

            return estimateInitialCamerasAndPointsEssential(intrinsic1, intrinsic2);
        } catch (final Exception e) {
            return false;
        }
    }

    /**
     * Estimates initial cameras and reconstructed points using the Dual
     * Absolute Quadric.
     *
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsDAQ() {
        try {
            final var fundamentalMatrix = currentEstimatedFundamentalMatrix.getFundamentalMatrix();
            fundamentalMatrix.normalize();

            final var estimator = new DualAbsoluteQuadricInitialCamerasEstimator(fundamentalMatrix);
            estimator.setAspectRatio(configuration.getInitialCamerasAspectRatio());
            estimator.estimate();

            final var camera1 = estimator.getEstimatedLeftCamera();
            final var camera2 = estimator.getEstimatedRightCamera();

            camera1.decompose();
            camera2.decompose();

            final var intrinsicZeroPrincipalPoint1 = camera1.getIntrinsicParameters();
            final var intrinsicZeroPrincipalPoint2 = camera2.getIntrinsicParameters();

            final var principalPointX = configuration.getPrincipalPointX();
            final var principalPointY = configuration.getPrincipalPointY();

            final var intrinsic1 = new PinholeCameraIntrinsicParameters(intrinsicZeroPrincipalPoint1);
            intrinsic1.setHorizontalPrincipalPoint(intrinsic1.getHorizontalPrincipalPoint() + principalPointX);
            intrinsic1.setVerticalPrincipalPoint(intrinsic1.getVerticalPrincipalPoint() + principalPointY);
            camera1.setIntrinsicParameters(intrinsic1);

            final var intrinsic2 = new PinholeCameraIntrinsicParameters(intrinsicZeroPrincipalPoint2);
            intrinsic2.setHorizontalPrincipalPoint(intrinsic2.getHorizontalPrincipalPoint() + principalPointX);
            intrinsic2.setVerticalPrincipalPoint(intrinsic2.getVerticalPrincipalPoint() + principalPointY);
            camera2.setIntrinsicParameters(intrinsic2);

            previousMetricEstimatedCamera = new EstimatedCamera();
            previousMetricEstimatedCamera.setCamera(camera1);
            previousMetricEstimatedCamera.setViewId(previousViewId);

            currentMetricEstimatedCamera = new EstimatedCamera();
            currentMetricEstimatedCamera.setCamera(camera2);
            currentMetricEstimatedCamera.setViewId(currentViewId);

            // fix fundamental matrix to account for principal point different
            // from zero
            fixFundamentalMatrix(fundamentalMatrix, intrinsicZeroPrincipalPoint1, intrinsicZeroPrincipalPoint2,
                    intrinsic1, intrinsic2);

            // triangulate points
            Corrector corrector = null;
            if (configuration.getInitialCamerasCorrectorType() != null) {
                corrector = Corrector.create(fundamentalMatrix, configuration.getInitialCamerasCorrectorType());
            }

            // use all points used for fundamental matrix estimation
            final var samples1 = currentEstimatedFundamentalMatrix.getLeftSamples();
            final var samples2 = currentEstimatedFundamentalMatrix.getRightSamples();

            final var points1 = new ArrayList<Point2D>();
            final var points2 = new ArrayList<Point2D>();
            final var length = samples1.size();
            for (var i = 0; i < length; i++) {
                final var sample1 = samples1.get(i);
                final var sample2 = samples2.get(i);

                final var point1 = sample1.getPoint();
                final var point2 = sample2.getPoint();

                points1.add(point1);
                points2.add(point2);
            }

            // correct points if needed
            final List<Point2D> correctedPoints1;
            final List<Point2D> correctedPoints2;
            if (corrector != null) {
                corrector.setLeftAndRightPoints(points1, points2);
                corrector.correct();

                correctedPoints1 = corrector.getLeftCorrectedPoints();
                correctedPoints2 = corrector.getRightCorrectedPoints();
            } else {
                correctedPoints1 = points1;
                correctedPoints2 = points2;
            }

            // triangulate points
            final SinglePoint3DTriangulator triangulator;
            if (configuration.getDaqUseHomogeneousPointTriangulator()) {
                triangulator = SinglePoint3DTriangulator.create(Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR);
            } else {
                triangulator = SinglePoint3DTriangulator.create(
                        Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR);
            }

            final var cameras = new ArrayList<PinholeCamera>();
            cameras.add(camera1);
            cameras.add(camera2);

            activeMetricReconstructedPoints = new ArrayList<>();
            final var points = new ArrayList<Point2D>();
            final var numPoints = correctedPoints1.size();

            Point3D triangulatedPoint;
            ReconstructedPoint3D reconstructedPoint;
            for (var i = 0; i < numPoints; i++) {
                points.clear();
                points.add(correctedPoints1.get(i));
                points.add(correctedPoints2.get(i));

                triangulator.setPointsAndCameras(points, cameras);
                triangulatedPoint = triangulator.triangulate();

                reconstructedPoint = new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoint);

                // only points reconstructed in front of both cameras are
                // considered valid
                final var front1 = camera1.isPointInFrontOfCamera(triangulatedPoint);
                final var front2 = camera2.isPointInFrontOfCamera(triangulatedPoint);
                final var inlier = front1 && front2;
                reconstructedPoint.setInlier(inlier);

                activeMetricReconstructedPoints.add(reconstructedPoint);

                if (inlier) {
                    matches.get(i).setReconstructedPoint(reconstructedPoint);
                }
            }

            return true;
        } catch (final Exception e) {
            return false;
        }
    }

    /**
     * Estimates initial cameras and reconstructed points using Dual Image of
     * Absolute Conic.
     *
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsDIAC() {
        final var fundamentalMatrix = currentEstimatedFundamentalMatrix.getFundamentalMatrix();

        // use inlier points used for fundamental matrix estimation
        final var samples1 = currentEstimatedFundamentalMatrix.getLeftSamples();
        final var samples2 = currentEstimatedFundamentalMatrix.getRightSamples();

        final var points1 = new ArrayList<Point2D>();
        final var points2 = new ArrayList<Point2D>();
        final var length = samples1.size();
        for (var i = 0; i < length; i++) {
            final var sample1 = samples1.get(i);
            final var sample2 = samples2.get(i);

            final var point1 = sample1.getPoint();
            final var point2 = sample2.getPoint();

            points1.add(point1);
            points2.add(point2);
        }

        try {
            final var estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator(fundamentalMatrix, points1,
                    points2);
            estimator.setPrincipalPoint(configuration.getPrincipalPointX(), configuration.getPrincipalPointY());
            estimator.setAspectRatio(configuration.getInitialCamerasAspectRatio());
            estimator.setCorrectorType(configuration.getInitialCamerasCorrectorType());
            estimator.setPointsTriangulated(true);
            estimator.setValidTriangulatedPointsMarked(configuration.getInitialCamerasMarkValidTriangulatedPoints());

            estimator.estimate();

            // store cameras
            final var camera1 = estimator.getEstimatedLeftCamera();
            final var camera2 = estimator.getEstimatedRightCamera();

            previousMetricEstimatedCamera = new EstimatedCamera();
            previousMetricEstimatedCamera.setCamera(camera1);

            currentMetricEstimatedCamera = new EstimatedCamera();
            currentMetricEstimatedCamera.setCamera(camera2);

            // store points
            final var triangulatedPoints = estimator.getTriangulatedPoints();
            final var validTriangulatedPoints = estimator.getValidTriangulatedPoints();

            activeMetricReconstructedPoints = new ArrayList<>();
            final var triangulatedPointsSize = triangulatedPoints.size();
            for (var i = 0; i < triangulatedPointsSize; i++) {
                final var reconstructedPoint = new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoints.get(i));
                reconstructedPoint.setInlier(validTriangulatedPoints.get(i));
                activeMetricReconstructedPoints.add(reconstructedPoint);

                if (validTriangulatedPoints.get(i)) {
                    matches.get(i).setReconstructedPoint(reconstructedPoint);
                }
            }

            return true;
        } catch (final Exception e) {
            return false;
        }
    }

    /**
     * Estimates initial cameras and reconstructed points using the essential
     * matrix and provided intrinsic parameters that must have been set during
     * offline calibration.
     *
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsEssential() {
        final var intrinsic1 = configuration.getInitialIntrinsic1();
        final var intrinsic2 = configuration.getInitialIntrinsic2();
        return estimateInitialCamerasAndPointsEssential(intrinsic1, intrinsic2);
    }

    /**
     * Estimates initial cameras and reconstructed points using the essential
     * matrix and provided intrinsic parameters that must have been set during
     * offline calibration.
     *
     * @param intrinsic1 intrinsic parameters of 1st camera.
     * @param intrinsic2 intrinsic parameters of 2nd camera.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsEssential(
            final PinholeCameraIntrinsicParameters intrinsic1, final PinholeCameraIntrinsicParameters intrinsic2) {
        final var fundamentalMatrix = currentEstimatedFundamentalMatrix.getFundamentalMatrix();

        // use all points used for fundamental matrix estimation
        final var samples1 = currentEstimatedFundamentalMatrix.getLeftSamples();
        final var samples2 = currentEstimatedFundamentalMatrix.getRightSamples();

        final var points1 = new ArrayList<Point2D>();
        final var points2 = new ArrayList<Point2D>();
        final var length = samples1.size();
        for (var i = 0; i < length; i++) {
            final var sample1 = samples1.get(i);
            final var sample2 = samples2.get(i);

            final var point1 = sample1.getPoint();
            final var point2 = sample2.getPoint();

            points1.add(point1);
            points2.add(point2);
        }

        try {
            final var estimator = new EssentialMatrixInitialCamerasEstimator(fundamentalMatrix, intrinsic1, intrinsic2,
                    points1, points2);

            estimator.setCorrectorType(configuration.getInitialCamerasCorrectorType());
            estimator.setPointsTriangulated(true);
            estimator.setValidTriangulatedPointsMarked(configuration.getInitialCamerasMarkValidTriangulatedPoints());

            estimator.estimate();

            // store cameras
            final var camera1 = estimator.getEstimatedLeftCamera();
            final var camera2 = estimator.getEstimatedRightCamera();

            previousMetricEstimatedCamera = new EstimatedCamera();
            previousMetricEstimatedCamera.setCamera(camera1);
            previousMetricEstimatedCamera.setViewId(previousViewId);

            currentMetricEstimatedCamera = new EstimatedCamera();
            currentMetricEstimatedCamera.setCamera(camera2);
            currentMetricEstimatedCamera.setViewId(currentViewId);

            // store points
            final var triangulatedPoints = estimator.getTriangulatedPoints();
            final var validTriangulatedPoints = estimator.getValidTriangulatedPoints();

            activeMetricReconstructedPoints = new ArrayList<>();
            final var triangulatedPointsSize = triangulatedPoints.size();
            final var matchesSize = matches.size();
            var j = 0;
            for (var i = 0; i < triangulatedPointsSize && j < matchesSize; i++, j++) {
                if (!validTriangulatedPoints.get(i)) {
                    continue;
                }

                final var reconstructedPoint = new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoints.get(i));
                reconstructedPoint.setInlier(validTriangulatedPoints.get(i));
                activeMetricReconstructedPoints.add(reconstructedPoint);

                matches.get(j).setReconstructedPoint(reconstructedPoint);
            }

            return true;
        } catch (final Exception e) {
            return false;
        }
    }

    /**
     * Fixes fundamental matrix to account for principal point different from
     * zero when using DAQ estimation.
     *
     * @param fundamentalMatrix            fundamental matrix to be fixed.
     * @param intrinsicZeroPrincipalPoint1 intrinsic parameters of camera 1
     *                                     assuming zero principal point.
     * @param intrinsicZeroPrincipalPoint2 intrinsic parameters of camera 2
     *                                     assuming zero principal point.
     * @param intrinsicPrincipalPoint1     intrinsic parameters of camera 1 using
     *                                     proper principal point.
     * @param intrinsicPrincipalPoint2     intrinsic parameters of camera 2 using
     *                                     proper principal point.
     * @throws EpipolarException if something fails.
     * @throws NotReadyException never happens.
     */
    private void fixFundamentalMatrix(
            final FundamentalMatrix fundamentalMatrix,
            final PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint1,
            final PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint2,
            final PinholeCameraIntrinsicParameters intrinsicPrincipalPoint1,
            final PinholeCameraIntrinsicParameters intrinsicPrincipalPoint2)
            throws EpipolarException, NotReadyException {

        // first compute essential matrix as E = K2a'F*K1a
        final var essential = new EssentialMatrix(fundamentalMatrix, intrinsicZeroPrincipalPoint1,
                intrinsicZeroPrincipalPoint2);
        final var fixedFundamentalMatrix = essential.toFundamentalMatrix(intrinsicPrincipalPoint1,
                intrinsicPrincipalPoint2);
        fixedFundamentalMatrix.normalize();
        currentEstimatedFundamentalMatrix.setFundamentalMatrix(fixedFundamentalMatrix);
        currentEstimatedFundamentalMatrix.setCovariance(null);
    }
}
