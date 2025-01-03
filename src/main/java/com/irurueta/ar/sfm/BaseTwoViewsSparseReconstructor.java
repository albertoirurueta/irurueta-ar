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
import com.irurueta.geometry.estimators.LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.PointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.ProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;

import java.util.ArrayList;
import java.util.List;

/**
 * Base class in charge of estimating cameras and 3D reconstructed points from
 * sparse image point correspondences in two views.
 *
 * @param <C> type of configuration.
 * @param <R> type of re-constructor.
 * @param <L> type of listener.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class BaseTwoViewsSparseReconstructor<
        C extends BaseTwoViewsSparseReconstructorConfiguration<C>,
        R extends BaseTwoViewsSparseReconstructor<C, R, L>,
        L extends BaseTwoViewsSparseReconstructorListener<R>> {

    /**
     * Number of views.
     */
    public static final int NUMBER_OF_VIEWS = 2;

    /**
     * Estimated fundamental matrix.
     */
    protected EstimatedFundamentalMatrix estimatedFundamentalMatrix;

    /**
     * Estimated first camera.
     */
    protected EstimatedCamera estimatedCamera1;

    /**
     * Estimated second camera.
     */
    protected EstimatedCamera estimatedCamera2;

    /**
     * Reconstructed 3D points.
     */
    protected List<ReconstructedPoint3D> reconstructedPoints;

    /**
     * Configuration for this re-constructor.
     */
    protected C configuration;

    /**
     * Listener in charge of handling events such as when reconstruction starts,
     * ends, when certain data is needed or when estimation of data has been
     * computed.
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
     * Samples on first view.
     */
    private List<Sample2D> firstViewSamples = null;

    /**
     * Samples on last processed view (i.e. current view).
     */
    private List<Sample2D> currentViewSamples;

    /**
     * Matches between first and current view.
     */
    private final List<MatchedSamples> matches = new ArrayList<>();

    /**
     * ID of first view.
     */
    private int firstViewId = 0;

    /**
     * Constructor.
     *
     * @param configuration configuration for this re-constructor.
     * @param listener      listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     *                              provided.
     */
    protected BaseTwoViewsSparseReconstructor(final C configuration, final L listener) {
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
     * Gets listener in charge of handling events such as when reconstruction
     * starts, ends, when certain data is needed or when estimation of data has
     * been computed.
     *
     * @return listener in charge of handling events.
     */
    public BaseTwoViewsSparseReconstructorListener<R> getListener() {
        return listener;
    }

    /**
     * Indicates whether reconstruction is running or not.
     *
     * @return true if reconstruction is running, false if reconstruction has
     * stopped for any reason.
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
     * Gets estimated fundamental matrix.
     *
     * @return estimated fundamental matrix.
     */
    public EstimatedFundamentalMatrix getEstimatedFundamentalMatrix() {
        return estimatedFundamentalMatrix;
    }

    /**
     * Gets estimated first camera.
     *
     * @return estimated first camera.
     */
    public EstimatedCamera getEstimatedCamera1() {
        return estimatedCamera1;
    }

    /**
     * Gets estimated second camera.
     *
     * @return estimated second camera.
     */
    public EstimatedCamera getEstimatedCamera2() {
        return estimatedCamera2;
    }

    /**
     * Gets reconstructed 3D points.
     *
     * @return reconstructed 3D points.
     */
    public List<ReconstructedPoint3D> getReconstructedPoints() {
        return reconstructedPoints;
    }

    /**
     * Process one view of all the available data during the reconstruction.
     * This method can be called multiple times instead of {@link #start()} to build the reconstruction
     * step by step, one view at a time.
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
            return false;
        }

        estimatedFundamentalMatrix = null;
        currentViewSamples = new ArrayList<>();
        //noinspection unchecked
        listener.onRequestSamplesForCurrentView((R) this, viewCount, currentViewSamples);

        if (firstViewSamples == null) {
            // for first view we simply keep samples (if enough are provided)
            if (hasEnoughSamples(currentViewSamples)) {
                //noinspection unchecked
                listener.onSamplesAccepted((R) this, viewCount, currentViewSamples);
                firstViewSamples = currentViewSamples;
                firstViewId = viewCount;
            }

        } else {

            // for second view, check that we have enough samples
            if (hasEnoughSamples(currentViewSamples)) {

                // find matches
                matches.clear();
                //noinspection unchecked
                listener.onRequestMatches((R) this, firstViewSamples, currentViewSamples, firstViewId, viewCount,
                        matches);

                if (hasEnoughMatches(matches)) {
                    // if enough matches are retrieved, attempt to compute
                    // fundamental matrix
                    if ((configuration.isGeneralSceneAllowed()
                            && estimateFundamentalMatrix(matches, firstViewId, viewCount))
                            || (configuration.isPlanarSceneAllowed()
                            && estimatePlanarFundamentalMatrix(matches, firstViewId, viewCount))) {
                        // fundamental matrix could be estimated
                        //noinspection unchecked
                        listener.onSamplesAccepted((R) this, viewCount, currentViewSamples);
                        var secondViewId = viewCount;

                        //noinspection unchecked
                        listener.onFundamentalMatrixEstimated((R) this, estimatedFundamentalMatrix);

                        if (estimateInitialCamerasAndPoints()) {
                            // cameras and points have been estimated
                            //noinspection unchecked
                            listener.onCamerasEstimated((R) this, firstViewId, secondViewId, estimatedCamera1,
                                    estimatedCamera2);
                            //noinspection unchecked
                            listener.onReconstructedPointsEstimated((R) this, matches, reconstructedPoints);
                            if (postProcessOne()) {
                                //noinspection unchecked
                                listener.onFinish((R) this);
                                running = false;
                                finished = true;
                            }
                        } else {
                            // initial cameras failed
                            failed = true;
                            //noinspection unchecked
                            listener.onFail((R) this);
                        }
                    } else {
                        // estimation of fundamental matrix failed
                        //noinspection unchecked
                        listener.onSamplesRejected((R) this, viewCount, currentViewSamples);
                    }
                }
            }
        }

        viewCount++;

        if (cancelled) {
            //noinspection unchecked
            listener.onCancel((R) this);
        }

        return !finished;
    }

    /**
     * Starts reconstruction of all available data to reconstruct the whole scene.
     * If reconstruction has already started and is running, calling this method
     * has no effect.
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
     * If reconstruction has already been cancelled, calling this method has no
     * effect.
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
        firstViewSamples = currentViewSamples = null;

        cancelled = failed = false;
        viewCount = 0;
        running = false;

        estimatedFundamentalMatrix = null;
        estimatedCamera1 = estimatedCamera2 = null;
        reconstructedPoints = null;

        finished = false;
    }

    /**
     * Called when processing one frame is successfully finished. This can be done to estimate scale on
     * those implementations where scale can be measured or is already known.
     *
     * @return true if post-processing succeeded, false otherwise.
     */
    protected abstract boolean postProcessOne();

    /**
     * Indicates whether there are enough samples to estimate a fundamental
     * matrix.
     *
     * @param samples samples to check.
     * @return true if there are enough samples, false otherwise.
     */
    private boolean hasEnoughSamples(final List<Sample2D> samples) {
        return hasEnoughSamplesOrMatches(samples != null ? samples.size() : 0);
    }

    /**
     * Indicates whether there are enough matches to estimate a fundamental
     * matrix.
     *
     * @param matches matches to check.
     * @return true if there are enough matches, false otherwise.
     */
    private boolean hasEnoughMatches(final List<MatchedSamples> matches) {
        return hasEnoughSamplesOrMatches(matches != null ? matches.size() : 0);
    }

    /**
     * Indicates whether there are enough matches or samples to estimate a
     * fundamental matrix.
     *
     * @param count number of matches or samples.
     * @return true if there are enough matches or samples, false otherwise.
     */
    private boolean hasEnoughSamplesOrMatches(final int count) {
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
     * Estimates fundamental matrix for provided matches, when 3D points lay in
     * a general non-degenerate 3D configuration.
     *
     * @param matches pairs of matches to find fundamental matrix.
     * @param viewId1 id of first view.
     * @param viewId2 id of second view.
     * @return true if estimation succeeded, false otherwise.
     */
    private boolean estimateFundamentalMatrix(final List<MatchedSamples> matches, final int viewId1,
                                              final int viewId2) {
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
        if (configuration.getInitialCamerasEstimatorMethod() == InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC
                || configuration.getInitialCamerasEstimatorMethod()
                == InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX) {
            principalPointX = configuration.getPrincipalPointX();
            principalPointY = configuration.getPrincipalPointY();
        } else {
            principalPointX = principalPointY = 0.0;
        }

        var i = 0;
        for (final var match : matches) {
            final var samples = match.getSamples();
            if (samples.length != NUMBER_OF_VIEWS) {
                return false;
            }

            leftSamples.add(samples[0]);
            rightSamples.add(samples[1]);

            final var leftPoint = Point2D.create();
            leftPoint.setInhomogeneousCoordinates(
                    samples[0].getPoint().getInhomX() - principalPointX,
                    samples[0].getPoint().getInhomY() - principalPointY);
            leftPoints.add(leftPoint);

            final var rightPoint = Point2D.create();
            rightPoint.setInhomogeneousCoordinates(
                    samples[1].getPoint().getInhomX() - principalPointX,
                    samples[1].getPoint().getInhomY() - principalPointY);
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

            estimatedFundamentalMatrix = new EstimatedFundamentalMatrix();
            estimatedFundamentalMatrix.setFundamentalMatrix(fundamentalMatrix);
            estimatedFundamentalMatrix.setViewId1(viewId1);
            estimatedFundamentalMatrix.setViewId2(viewId2);
            estimatedFundamentalMatrix.setCovariance(estimator.getCovariance());

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
                estimatedFundamentalMatrix.setQualityScore(fundamentalMatrixQualityScore);
                estimatedFundamentalMatrix.setInliers(inliers);
            }

            // store left/right samples
            estimatedFundamentalMatrix.setLeftSamples(leftSamples);
            estimatedFundamentalMatrix.setRightSamples(rightSamples);

            return true;
        } catch (final Exception e) {
            return false;
        }
    }

    /**
     * Estimates fundamental matrix for provided matches, when 3D points lay in
     * a planar 3D scene.
     *
     * @param matches pairs of matches to find fundamental matrix.
     * @param viewId1 id of first view.
     * @param viewId2 id of second view.
     * @return true if estimation succeeded, false otherwise.
     */
    private boolean estimatePlanarFundamentalMatrix(
            final List<MatchedSamples> matches, final int viewId1, final int viewId2) {
        if (matches == null) {
            return false;
        }

        final var count = matches.size();
        final var leftSamples = new ArrayList<Sample2D>();
        final var rightSamples = new ArrayList<Sample2D>();
        final var leftPoints = new ArrayList<Point2D>();
        final var rightPoints = new ArrayList<Point2D>();
        final var qualityScores = new double[count];
        final double principalPointX;
        final double principalPointY;
        if (configuration.getInitialCamerasEstimatorMethod() == InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC
                || configuration.getInitialCamerasEstimatorMethod()
                == InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX) {
            principalPointX = configuration.getPrincipalPointX();
            principalPointY = configuration.getPrincipalPointY();
        } else {
            principalPointX = principalPointY = 0.0;
        }

        var i = 0;
        for (final var match : matches) {
            final var samples = match.getSamples();
            if (samples.length != NUMBER_OF_VIEWS) {
                return false;
            }

            leftSamples.add(samples[0]);
            rightSamples.add(samples[1]);

            final var leftPoint = Point2D.create();
            leftPoint.setInhomogeneousCoordinates(
                    samples[0].getPoint().getInhomX() - principalPointX,
                    samples[0].getPoint().getInhomY() - principalPointY);
            leftPoints.add(leftPoint);

            final var rightPoint = Point2D.create();
            rightPoint.setInhomogeneousCoordinates(
                    samples[1].getPoint().getInhomX() - principalPointX,
                    samples[1].getPoint().getInhomY() - principalPointY);
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
                default:
                    final var ransacHomographyEstimator =
                            (RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator;

                    ransacHomographyEstimator.setThreshold(configuration.getPlanarHomographyThreshold());
                    ransacHomographyEstimator.setComputeAndKeepInliersEnabled(
                            configuration.getPlanarHomographyComputeAndKeepInliers());
                    ransacHomographyEstimator.setComputeAndKeepResidualsEnabled(
                            configuration.getPlanarHomographyComputeAndKeepResiduals());
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

            } else if (intrinsic1 == null) { // && intrinsic2 != null
                intrinsic1 = intrinsic2;
            } else if (intrinsic2 == null) { // && intrinsic1 != null
                intrinsic2 = intrinsic1;
            }
            fundamentalMatrixEstimator.setLeftIntrinsics(intrinsic1);
            fundamentalMatrixEstimator.setRightIntrinsics(intrinsic2);

            fundamentalMatrixEstimator.estimateAndReconstruct();

            final var fundamentalMatrix = fundamentalMatrixEstimator.getFundamentalMatrix();

            estimatedFundamentalMatrix = new EstimatedFundamentalMatrix();
            estimatedFundamentalMatrix.setFundamentalMatrix(fundamentalMatrix);
            estimatedFundamentalMatrix.setViewId1(viewId1);
            estimatedFundamentalMatrix.setViewId2(viewId2);

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
                estimatedFundamentalMatrix.setQualityScore(fundamentalMatrixQualityScore);
                estimatedFundamentalMatrix.setInliers(inliers);
            }

            // store left/right samples
            estimatedFundamentalMatrix.setLeftSamples(leftSamples);
            estimatedFundamentalMatrix.setRightSamples(rightSamples);

            return true;
        } catch (final Exception e) {
            return false;
        }
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
            final var fundamentalMatrix = estimatedFundamentalMatrix.getFundamentalMatrix();

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
            final var fundamentalMatrix = estimatedFundamentalMatrix.getFundamentalMatrix();
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

            estimatedCamera1 = new EstimatedCamera();
            estimatedCamera1.setCamera(camera1);

            estimatedCamera2 = new EstimatedCamera();
            estimatedCamera2.setCamera(camera2);

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
            final var samples1 = estimatedFundamentalMatrix.getLeftSamples();
            final var samples2 = estimatedFundamentalMatrix.getRightSamples();

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
            List<Point2D> correctedPoints1;
            List<Point2D> correctedPoints2;
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

            reconstructedPoints = new ArrayList<>();
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
                reconstructedPoint.setInlier(front1 && front2);

                reconstructedPoints.add(reconstructedPoint);
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
        final var fundamentalMatrix = estimatedFundamentalMatrix.getFundamentalMatrix();

        // use inlier points used for fundamental matrix estimation
        final var samples1 = estimatedFundamentalMatrix.getLeftSamples();
        final var samples2 = estimatedFundamentalMatrix.getRightSamples();

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

            estimatedCamera1 = new EstimatedCamera();
            estimatedCamera1.setCamera(camera1);

            estimatedCamera2 = new EstimatedCamera();
            estimatedCamera2.setCamera(camera2);

            // store points
            final var triangulatedPoints = estimator.getTriangulatedPoints();
            final var validTriangulatedPoints = estimator.getValidTriangulatedPoints();

            reconstructedPoints = new ArrayList<>();
            final var size = triangulatedPoints.size();
            for (var i = 0; i < size; i++) {
                final var reconstructedPoint = new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoints.get(i));
                reconstructedPoint.setInlier(validTriangulatedPoints.get(i));
                reconstructedPoints.add(reconstructedPoint);
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
            final PinholeCameraIntrinsicParameters intrinsic1,
            final PinholeCameraIntrinsicParameters intrinsic2) {
        final var fundamentalMatrix = estimatedFundamentalMatrix.getFundamentalMatrix();

        // use all points used for fundamental matrix estimation
        final var samples1 = estimatedFundamentalMatrix.getLeftSamples();
        final var samples2 = estimatedFundamentalMatrix.getRightSamples();

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

            estimatedCamera1 = new EstimatedCamera();
            estimatedCamera1.setCamera(camera1);

            estimatedCamera2 = new EstimatedCamera();
            estimatedCamera2.setCamera(camera2);

            // store points
            final var triangulatedPoints = estimator.getTriangulatedPoints();
            final var validTriangulatedPoints = estimator.getValidTriangulatedPoints();

            reconstructedPoints = new ArrayList<>();
            final var size = triangulatedPoints.size();
            for (var i = 0; i < size; i++) {
                final var reconstructedPoint = new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoints.get(i));
                reconstructedPoint.setInlier(validTriangulatedPoints.get(i));
                reconstructedPoints.add(reconstructedPoint);
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
        estimatedFundamentalMatrix.setFundamentalMatrix(fixedFundamentalMatrix);
        estimatedFundamentalMatrix.setCovariance(null);
    }
}
