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

import com.irurueta.algebra.AlgebraException;
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
import com.irurueta.geometry.*;
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
 * Base class in charge of estimating cameras and 3D reconstructed points from sparse
 * image point correspondences in pairs of views.
 * Views are processed in pairs so that fundamental matrix is estimated and pairs of
 * cameras and reconstructed points are computed.
 * Because view pairs are processed separately, the scale of each view pair is
 * estimated individually, hence the scale will need to be
 *
 * @param <C> type of configuration.
 * @param <R> type of re-constructor.
 * @param <L> type of listener.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class BasePairedViewsSparseReconstructor<
        C extends BasePairedViewsSparseReconstructorConfiguration<C>,
        R extends BasePairedViewsSparseReconstructor<C, R, L>,
        L extends BasePairedViewsSparseReconstructorListener<R>> {

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
     * Reconstructed 3D points for current pair of views in a metric stratum (i.e. up to scale).
     */
    protected List<ReconstructedPoint3D> metricReconstructedPoints;

    /**
     * Transformation to set reference frame on estimated pair of Euclidean cameras.
     * This is used when estimating a new pair of Euclidean cameras to transform such pair to
     * the location and rotation of the last estimated Euclidean camera so that the first camera
     * of the pair is not referred to the world origin.
     */
    protected MetricTransformation3D referenceEuclideanTransformation;

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
     * Reconstructed 3D points for current pair of views in Euclidean stratum (i.e. with actual
     * scale).
     */
    protected List<ReconstructedPoint3D> euclideanReconstructedPoints;

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
     * ID of previous view.
     */
    protected int previousViewId = 0;

    /**
     * ID of current view.
     */
    protected int currentViewId;

    /**
     * Center of current Euclidean camera on last view pair.
     */
    protected Point3D lastEuclideanCameraCenter = new InhomogeneousPoint3D();

    /**
     * Rotation of current Euclidean camera on last view pair.
     */
    protected Rotation3D lastEuclideanCameraRotation;

    /**
     * Center of current metric camera on last view pair.
     */
    private Point3D lastMetricCameraCenter;

    /**
     * Rotation of current metric camera on last view pair.
     */
    private Rotation3D mLastMetricCameraRotation;

    /**
     * Inverse metric camera rotation. This is reused for memory efficiency.
     */
    private Rotation3D invMetricCameraRotation;

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
     * Samples on previous view.
     */
    private List<Sample2D> previousViewSamples;

    /**
     * Samples on last processed view (i.e. current view).
     */
    private List<Sample2D> currentViewSamples;

    /**
     * Matches between first and current view.
     * Views are always processed in pairs.
     */
    private final List<MatchedSamples> matches = new ArrayList<>();

    /**
     * Transformation to set reference frame on estimated pair of metric cameras.
     * This is used when estimating a new pair of metric cameras to transform such pair to
     * the location and rotation of last estimated metric camera so that the first camera of
     * the pair is not referred to the world origin.
     */
    private EuclideanTransformation3D referenceMetricTransformation;

    /**
     * Constructor.
     *
     * @param configuration configuration for this re-constructor.
     * @param listener      listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     *                              provided.
     */
    protected BasePairedViewsSparseReconstructor(final C configuration, final L listener) {
        if (configuration == null || listener == null) {
            throw new NullPointerException();
        }
        this.configuration = configuration;
        this.listener = listener;
    }

    /**
     * Gets configuration for this re-constructor.
     *
     * @return configuration for this reconstructor.
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
    public L getListener() {
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
     * Gets estimated fundamental matrix for current view.
     * This fundamental matrix relates current view with the previously processed one.
     *
     * @return current estimated fundamental matrix.
     */
    public EstimatedFundamentalMatrix getCurrentEstimatedFundamentalMatrix() {
        return currentEstimatedFundamentalMatrix;
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
     * Gets Euclidean reconstructed 3D points (i.e. with actual scale) for current
     * pair of views.
     *
     * @return active euclidean reconstructed 3D points.
     */
    public List<ReconstructedPoint3D> getEuclideanReconstructedPoints() {
        return euclideanReconstructedPoints;
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
     * Gets samples on previous view.
     *
     * @return samples on previous view.
     */
    public List<Sample2D> getPreviousViewSamples() {
        return previousViewSamples;
    }

    /**
     * Gets samples on current view.
     *
     * @return samples on current view.
     */
    public List<Sample2D> getCurrentViewSamples() {
        return currentViewSamples;
    }

    /**
     * Process one view-pair of all the available data during the reconstruction.
     * This method can be called multiple times instead of {@link #start()} to build the
     * reconstruction step by step, one view pair at a time.
     * This method is useful when data is gathered on real time from a camera and the
     * number of views is unknown.
     *
     * @return true if more views can be processed, false when reconstruction has finished.
     */
    public boolean processOneViewPair() {
        if (viewCount == 0 && !running) {

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

        previousViewSamples = new ArrayList<>();
        currentViewSamples = new ArrayList<>();
        //noinspection unchecked
        listener.onRequestSamplesForCurrentViewPair((R) this, viewCount, viewCount + 1,
                previousViewSamples, currentViewSamples);

        final boolean processed;
        currentEstimatedFundamentalMatrix = null;
        if (isFirstViewPair()) {
            // for first view we simply keep samples (if enough are provided)
            processed = processFirstViewPair();
        } else {
            processed = processAdditionalViewPair();
        }

        if (processed) {
            viewCount += 2;
        }

        if (cancelled) {
            //noinspection unchecked
            listener.onCancel((R) this);
        }

        return !finished;
    }

    /**
     * Indicates whether current view pair is the first one.
     *
     * @return true if current view pair is the first one, false otherwise.
     */
    public boolean isFirstViewPair() {
        return viewCount == 0;
    }

    /**
     * Indicates whether current view pair is an additional one.
     *
     * @return true if current view pair is an additional one, false otherwise.
     */
    public boolean isAdditionalViewPair() {
        return !isFirstViewPair();
    }

    /**
     * Starts reconstruction of all available data to reconstruct the whole scene.
     * If reconstruction has already started and is running, calling this method
     * has no effect.
     * This method is useful when all data is available before starting the reconstruction.
     */
    public void start() {
        if (running) {
            // already started
            return;
        }

        while (processOneViewPair()) {
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
        if (previousViewSamples != null) {
            previousViewSamples.clear();
        }
        if (currentViewSamples != null) {
            currentViewSamples.clear();
        }

        matches.clear();

        cancelled = failed = false;
        viewCount = 0;
        running = false;

        currentEstimatedFundamentalMatrix = null;
        currentMetricEstimatedCamera = previousMetricEstimatedCamera = null;
        metricReconstructedPoints = null;
        currentScale = DEFAULT_SCALE;
        currentEuclideanEstimatedCamera = previousEuclideanEstimatedCamera = null;
        euclideanReconstructedPoints = null;

        previousViewId = 0;
        currentViewId = 0;

        finished = false;
    }

    /**
     * Gets estimated metric camera for current view (i.e. up to scale).
     *
     * @return current estimated metric camera.
     */
    protected EstimatedCamera getCurrentMetricEstimatedCamera() {
        return currentMetricEstimatedCamera;
    }

    /**
     * Gets estimated camera for previous view (i.e. up to scale).
     *
     * @return previous estimated metric camera.
     */
    protected EstimatedCamera getPreviousMetricEstimatedCamera() {
        return previousMetricEstimatedCamera;
    }

    /**
     * Gets metric reconstructed 3D points (i.e. up to scale) for current pair of views.
     *
     * @return active metric reconstructed 3D points.
     */
    protected List<ReconstructedPoint3D> getMetricReconstructedPoints() {
        return metricReconstructedPoints;
    }

    /**
     * Transforms cameras on current pair of views so that they are referred to
     * last kept location and rotation and upgrades cameras from metric stratum to
     * Euclidean stratum.
     *
     * @param isInitialPairOfViews   true if initial pair of views is being processed, false otherwise.
     * @param hasAbsoluteOrientation true if absolute orientation is required, false otherwise.
     * @return true if cameras were successfully transformed.
     */
    protected boolean transformPairOfCamerasAndPoints(
            final boolean isInitialPairOfViews, final boolean hasAbsoluteOrientation) {
        if (isInitialPairOfViews) {
            // initial pair does not need transformation
            return true;
        }

        if (previousMetricEstimatedCamera == null || currentMetricEstimatedCamera == null) {
            return false;
        }

        final var previousMetricCamera = previousMetricEstimatedCamera.getCamera();
        final var currentMetricCamera = currentMetricEstimatedCamera.getCamera();
        if (previousMetricCamera == null || currentMetricCamera == null) {
            return false;
        }

        if (invMetricCameraRotation == null) {
            invMetricCameraRotation = mLastMetricCameraRotation.inverseRotationAndReturnNew();
        } else {
            mLastMetricCameraRotation.inverseRotation(invMetricCameraRotation);
        }

        if (referenceMetricTransformation == null) {
            referenceMetricTransformation = new EuclideanTransformation3D(invMetricCameraRotation);
        } else {
            referenceMetricTransformation.setRotation(invMetricCameraRotation);
        }
        referenceMetricTransformation.setTranslation(lastMetricCameraCenter);

        try {
            referenceMetricTransformation.transform(previousMetricCamera);
            referenceMetricTransformation.transform(currentMetricCamera);

            Point3D p;
            for (final var metricReconstructedPoint : metricReconstructedPoints) {
                p = metricReconstructedPoint.getPoint();
                referenceMetricTransformation.transform(p, p);
            }
            return true;
        } catch (final AlgebraException e) {
            return false;
        }
    }

    /**
     * Processes data for the first view pair.
     *
     * @return true if view pair was successfully processed, false otherwise.
     */
    private boolean processFirstViewPair() {
        return processViewPair(true);
    }

    /**
     * Processes data for an additional view pair.
     *
     * @return true if view pair was successfully processed, false otherwise.
     */
    private boolean processAdditionalViewPair() {
        return processViewPair(false);
    }

    /**
     * Processed data for a view pair.
     *
     * @param isInitialPairOfViews true if initial pair of views is being processed,
     *                             false otherwise.
     * @return true if view pair was successfully processed, false otherwise.
     */
    private boolean processViewPair(final boolean isInitialPairOfViews) {
        // for second view, check that we have enough samples
        if (hasEnoughSamples(currentViewSamples)) {

            // find matches
            matches.clear();
            var viewId1 = viewCount;
            var viewId2 = viewCount + 1;
            //noinspection unchecked
            listener.onRequestMatches((R) this, viewId1, viewId2, previousViewSamples, currentViewSamples, matches);

            if (hasEnoughMatches(matches)) {
                // if enough matches are retrieved, attempt to compute
                // fundamental matrix
                if ((configuration.isGeneralSceneAllowed() && estimateFundamentalMatrix(matches, viewId1, viewId2))
                        || (configuration.isPlanarSceneAllowed()
                        && estimatePlanarFundamentalMatrix(matches, viewId1, viewId2))) {
                    // fundamental matrix could be estimated
                    // noinspection unchecked
                    listener.onSamplesAccepted((R) this, viewId1, viewId2, previousViewSamples, currentViewSamples);
                    previousViewId = viewId1;
                    currentViewId = viewId2;

                    //noinspection unchecked
                    listener.onFundamentalMatrixEstimated((R) this, viewId1, viewId2,
                            currentEstimatedFundamentalMatrix);

                    if (estimatePairOfCamerasAndPoints(isInitialPairOfViews)) {
                        //noinspection unchecked
                        listener.onEuclideanCameraPairEstimated((R) this, previousViewId, currentViewId, currentScale,
                                previousEuclideanEstimatedCamera, currentEuclideanEstimatedCamera);
                        //noinspection unchecked
                        listener.onEuclideanReconstructedPointsEstimated((R) this, previousViewId, currentViewId,
                                currentScale, euclideanReconstructedPoints);
                        return true;
                    } else {
                        // pair of cameras estimation failed
                        failed = true;
                        //noinspection unchecked
                        listener.onFail((R) this);
                        return false;
                    }
                } else {
                    // estimation of fundamental matrix failed
                    //noinspection unchecked
                    listener.onSamplesRejected((R) this, previousViewId, currentViewId, previousViewSamples,
                            currentViewSamples);
                    return false;
                }
            }
        }

        //noinspection unchecked
        listener.onSamplesRejected((R) this, previousViewId, currentViewId, previousViewSamples, currentViewSamples);
        return false;
    }

    /**
     * Indicates whether implementations of a re-constructor uses absolute orientation or
     * not.
     *
     * @return true if absolute orientation is used, false, otherwise.
     */
    protected abstract boolean hasAbsoluteOrientation();

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
        double principalPointX;
        double principalPointY;
        if (configuration.getPairedCamerasEstimatorMethod() == InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC
                || configuration.getPairedCamerasEstimatorMethod()
                == InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX) {
            principalPointX = configuration.getPrincipalPointX();
            principalPointY = configuration.getPrincipalPointY();
        } else {
            principalPointX = principalPointY = 0.0;
        }

        var i = 0;
        for (final var match : matches) {
            final var samples = match.getSamples();
            if (samples.length != MIN_NUMBER_OF_VIEWS) {
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
                    ((LMedSFundamentalMatrixRobustEstimator) estimator).setStopThreshold(
                            configuration.getFundamentalMatrixThreshold());
                    break;
                case MSAC:
                    ((MSACFundamentalMatrixRobustEstimator) estimator).setThreshold(
                            configuration.getFundamentalMatrixThreshold());
                    break;
                case PROMEDS:
                    ((PROMedSFundamentalMatrixRobustEstimator) estimator).setStopThreshold(
                            configuration.getFundamentalMatrixThreshold());
                    break;
                case PROSAC:
                    final PROSACFundamentalMatrixRobustEstimator prosacEstimator =
                            (PROSACFundamentalMatrixRobustEstimator) estimator;
                    prosacEstimator.setThreshold(configuration.getFundamentalMatrixThreshold());
                    prosacEstimator.setComputeAndKeepInliersEnabled(
                            configuration.getFundamentalMatrixComputeAndKeepInliers());
                    prosacEstimator.setComputeAndKeepResidualsEnabled(
                            configuration.getFundamentalMatrixComputeAndKeepResiduals());
                    break;
                case RANSAC:
                    final RANSACFundamentalMatrixRobustEstimator ransacEstimator =
                            (RANSACFundamentalMatrixRobustEstimator) estimator;
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
     * Estimates fundamental matrix for provided matches, when 3D points lay in
     * a planar 3D scene.
     *
     * @param matches pairs of matches to find fundamental matrix.
     * @param viewId1 id of first view.
     * @param viewId2 id of second view.
     * @return true if estimation succeeded, false otherwise.
     */
    private boolean estimatePlanarFundamentalMatrix(final List<MatchedSamples> matches, final int viewId1,
                                                    final int viewId2) {
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
        if (configuration.getPairedCamerasEstimatorMethod() == InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC
                || configuration.getPairedCamerasEstimatorMethod()
                == InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX) {
            principalPointX = configuration.getPrincipalPointX();
            principalPointY = configuration.getPrincipalPointY();
        } else {
            principalPointX = principalPointY = 0.0;
        }

        var i = 0;
        for (final var match : matches) {
            final var samples = match.getSamples();
            if (samples.length != MIN_NUMBER_OF_VIEWS) {
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
                    PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator prosacHomographyEstimator =
                            (PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator;

                    prosacHomographyEstimator.setThreshold(configuration.getPlanarHomographyThreshold());
                    prosacHomographyEstimator.setComputeAndKeepInliersEnabled(
                            configuration.getPlanarHomographyComputeAndKeepInliers());
                    prosacHomographyEstimator.setComputeAndKeepResidualsEnabled(
                            configuration.getPlanarHomographyComputeAndKeepResiduals());
                    break;
                case RANSAC:
                    RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator ransacHomographyEstimator =
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

            final PlanarBestFundamentalMatrixEstimatorAndReconstructor fundamentalMatrixEstimator =
                    new PlanarBestFundamentalMatrixEstimatorAndReconstructor();
            fundamentalMatrixEstimator.setHomographyEstimator(homographyEstimator);
            fundamentalMatrixEstimator.setLeftAndRightPoints(leftPoints, rightPoints);
            fundamentalMatrixEstimator.setQualityScores(qualityScores);

            PinholeCameraIntrinsicParameters intrinsic1 = null;
            PinholeCameraIntrinsicParameters intrinsic2 = null;
            if (configuration.areIntrinsicParametersKnown()) {
                //noinspection unchecked
                intrinsic1 = listener.onIntrinsicParametersRequested((R) this, viewId1);
                //noinspection unchecked
                intrinsic2 = listener.onIntrinsicParametersRequested((R) this, viewId2);
            }
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
                currentEstimatedFundamentalMatrix.setQualityScore(
                        fundamentalMatrixQualityScore);
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
     * Estimates a pair of cameras and reconstructed points.
     *
     * @param isInitialPairOfViews true if initial pair of views is being processed,
     *                             false otherwise.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimatePairOfCamerasAndPoints(final boolean isInitialPairOfViews) {
        return switch (configuration.getPairedCamerasEstimatorMethod()) {
            case ESSENTIAL_MATRIX -> estimateInitialCamerasAndPointsEssential(isInitialPairOfViews);
            case DUAL_IMAGE_OF_ABSOLUTE_CONIC -> estimateInitialCamerasAndPointsDIAC(isInitialPairOfViews);
            case DUAL_ABSOLUTE_QUADRIC -> estimateInitialCamerasAndPointsDAQ(isInitialPairOfViews);
            default -> estimateInitialCamerasAndPointsDAQAndEssential(isInitialPairOfViews);
        };
    }

    /**
     * Estimates initial cameras and reconstructed points using the Dual
     * Absolute Quadric to estimate intrinsic parameters and then use those
     * intrinsic parameters with the essential matrix.
     *
     * @param isInitialPairOfViews true if initial pair of views is being processed,
     *                             false otherwise.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsDAQAndEssential(final boolean isInitialPairOfViews) {
        // for non-initial view, keep last center and rotation
        if (!isInitialPairOfViews && keepLastCenterAndRotation()) {
            return false;
        }

        try {
            final var fundamentalMatrix = currentEstimatedFundamentalMatrix.getFundamentalMatrix();

            final var estimator = new DualAbsoluteQuadricInitialCamerasEstimator(fundamentalMatrix);
            estimator.setAspectRatio(configuration.getPairedCamerasAspectRatio());
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

            return estimateInitialCamerasAndPointsEssential(intrinsic1, intrinsic2)
                    && transformPairOfCamerasAndPoints(isInitialPairOfViews, hasAbsoluteOrientation());
        } catch (final Exception e) {
            return false;
        }
    }

    /**
     * Estimates initial cameras and reconstructed points using the Dual
     * Absolute Quadric.
     *
     * @param isInitialPairOfViews true if initial pair of views is being processed,
     *                             false otherwise.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsDAQ(final boolean isInitialPairOfViews) {
        // for non-initial view, keep last center and rotation
        if (!isInitialPairOfViews && keepLastCenterAndRotation()) {
            return false;
        }

        try {
            final var fundamentalMatrix = currentEstimatedFundamentalMatrix.getFundamentalMatrix();
            fundamentalMatrix.normalize();

            final var estimator = new DualAbsoluteQuadricInitialCamerasEstimator(fundamentalMatrix);
            estimator.setAspectRatio(configuration.getPairedCamerasAspectRatio());
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

            currentMetricEstimatedCamera = new EstimatedCamera();
            currentMetricEstimatedCamera.setCamera(camera2);

            // fix fundamental matrix to account for principal point different
            // from zero
            fixFundamentalMatrix(fundamentalMatrix, intrinsicZeroPrincipalPoint1, intrinsicZeroPrincipalPoint2,
                    intrinsic1, intrinsic2);

            // triangulate points
            Corrector corrector = null;
            if (configuration.getPairedCamerasCorrectorType() != null) {
                corrector = Corrector.create(fundamentalMatrix, configuration.getPairedCamerasCorrectorType());
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

            metricReconstructedPoints = new ArrayList<>();
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

                metricReconstructedPoints.add(reconstructedPoint);
            }

            return transformPairOfCamerasAndPoints(isInitialPairOfViews, hasAbsoluteOrientation());
        } catch (final Exception e) {
            return false;
        }
    }

    /**
     * Estimates initial cameras and reconstructed points using Dual Image of
     * Absolute Conic.
     *
     * @param isInitialPairOfViews true if initial pair of views is being processed,
     *                             false otherwise.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsDIAC(final boolean isInitialPairOfViews) {
        // for non-initial view, keep last center and rotation
        if (!isInitialPairOfViews && keepLastCenterAndRotation()) {
            return false;
        }

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
            estimator.setAspectRatio(configuration.getPairedCamerasAspectRatio());
            estimator.setCorrectorType(configuration.getPairedCamerasCorrectorType());
            estimator.setPointsTriangulated(true);
            estimator.setValidTriangulatedPointsMarked(configuration.getPairedCamerasMarkValidTriangulatedPoints());

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

            metricReconstructedPoints = new ArrayList<>();
            final var size = triangulatedPoints.size();
            for (var i = 0; i < size; i++) {
                final var reconstructedPoint = new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoints.get(i));
                reconstructedPoint.setInlier(validTriangulatedPoints.get(i));
                metricReconstructedPoints.add(reconstructedPoint);
            }

            return transformPairOfCamerasAndPoints(isInitialPairOfViews, hasAbsoluteOrientation());
        } catch (final Exception e) {
            return false;
        }
    }

    /**
     * Estimates initial cameras and reconstructed points using the essential
     * matrix and provided intrinsic parameters that must have been set during
     * offline calibration.
     *
     * @param isInitialPairOfViews true if initial pair of views is being processed,
     *                             false otherwise.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsEssential(final boolean isInitialPairOfViews) {
        // for non-initial view, keep last center and rotation
        if (!isInitialPairOfViews && keepLastCenterAndRotation()) {
            return false;
        }

        PinholeCameraIntrinsicParameters intrinsic1 = null;
        PinholeCameraIntrinsicParameters intrinsic2 = null;
        if (configuration.areIntrinsicParametersKnown()) {
            //noinspection unchecked
            intrinsic1 = listener.onIntrinsicParametersRequested((R) this, previousViewId);
            //noinspection unchecked
            intrinsic2 = listener.onIntrinsicParametersRequested((R) this, currentViewId);
        }

        if (intrinsic1 != null && intrinsic2 != null) {
            return estimateInitialCamerasAndPointsEssential(intrinsic1, intrinsic2)
                    && transformPairOfCamerasAndPoints(isInitialPairOfViews, hasAbsoluteOrientation());
        } else {
            // missing intrinsic parameters

            failed = true;
            //noinspection unchecked
            listener.onFail((R) this);
            return false;
        }
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

            estimator.setCorrectorType(configuration.getPairedCamerasCorrectorType());
            estimator.setPointsTriangulated(true);
            estimator.setValidTriangulatedPointsMarked(configuration.getPairedCamerasMarkValidTriangulatedPoints());

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

            metricReconstructedPoints = new ArrayList<>();
            final var size = triangulatedPoints.size();
            for (var i = 0; i < size; i++) {
                final var reconstructedPoint = new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoints.get(i));
                reconstructedPoint.setInlier(validTriangulatedPoints.get(i));
                metricReconstructedPoints.add(reconstructedPoint);
            }

            return true;
        } catch (final Exception e) {
            return false;
        }
    }


    /**
     * Keeps center and rotation of last camera (current camera on previous view pair).
     *
     * @return false if camera and rotation were successfully kept, true otherwise.
     */
    private boolean keepLastCenterAndRotation() {
        // keep last metric center and rotation
        if (currentMetricEstimatedCamera == null || currentEuclideanEstimatedCamera == null) {
            return true;
        }

        final var metricCamera = currentMetricEstimatedCamera.getCamera();
        if (metricCamera == null) {
            return true;
        }

        try {
            // decompose camera if needed
            if (!metricCamera.isCameraCenterAvailable() || !metricCamera.isCameraRotationAvailable()) {
                metricCamera.decompose();
            }

            lastMetricCameraCenter = metricCamera.getCameraCenter();
            mLastMetricCameraRotation = metricCamera.getCameraRotation();

        } catch (final GeometryException e) {
            return true;
        }

        // keep last Euclidean center and rotation
        final var euclideanCamera = currentEuclideanEstimatedCamera.getCamera();
        if (euclideanCamera == null) {
            return true;
        }

        try {
            // decompose camera if needed
            if (!euclideanCamera.isCameraCenterAvailable() || !euclideanCamera.isCameraRotationAvailable()) {
                euclideanCamera.decompose();
            }

            lastEuclideanCameraCenter = euclideanCamera.getCameraCenter();
            lastEuclideanCameraRotation = euclideanCamera.getCameraRotation();

            return false;
        } catch (final GeometryException e) {
            return true;
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
    private void fixFundamentalMatrix(final FundamentalMatrix fundamentalMatrix,
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
