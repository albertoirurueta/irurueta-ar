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

import com.irurueta.ar.calibration.ImageOfAbsoluteConic;
import com.irurueta.ar.calibration.estimators.ImageOfAbsoluteConicEstimator;
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
import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.geometry.Transformation2D;
import com.irurueta.geometry.estimators.LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.PointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.ProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.InliersData;

import java.util.ArrayList;
import java.util.BitSet;
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
    protected EstimatedFundamentalMatrix mEstimatedFundamentalMatrix;

    /**
     * Estimated first camera.
     */
    protected EstimatedCamera mEstimatedCamera1;

    /**
     * Estimated second camera.
     */
    protected EstimatedCamera mEstimatedCamera2;

    /**
     * Reconstructed 3D points.
     */
    protected List<ReconstructedPoint3D> mReconstructedPoints;

    /**
     * Configuration for this re-constructor.
     */
    protected C mConfiguration;

    /**
     * Listener in charge of handling events such as when reconstruction starts,
     * ends, when certain data is needed or when estimation of data has been
     * computed.
     */
    protected L mListener;

    /**
     * Indicates whether reconstruction has failed or not.
     */
    protected volatile boolean mFailed;

    /**
     * Indicates whether reconstruction is running or not.
     */
    protected volatile boolean mRunning;

    /**
     * Indicates whether reconstruction has been cancelled or not.
     */
    private volatile boolean mCancelled;

    /**
     * Counter of number of processed views.
     */
    private int mViewCount;

    /**
     * Indicates whether reconstruction has finished or not.
     */
    private boolean mFinished = false;

    /**
     * Samples on first view.
     */
    private List<Sample2D> mFirstViewSamples = null;

    /**
     * Samples on last processed view (i.e. current view).
     */
    private List<Sample2D> mCurrentViewSamples;

    /**
     * Matches between first and current view.
     */
    private final List<MatchedSamples> mMatches = new ArrayList<>();

    /**
     * ID of first view.
     */
    private int mFirstViewId = 0;

    /**
     * Constructor.
     *
     * @param configuration configuration for this re-constructor.
     * @param listener      listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     *                              provided.
     */
    protected BaseTwoViewsSparseReconstructor(
            final C configuration, final L listener) {
        if (configuration == null || listener == null) {
            throw new NullPointerException();
        }
        mConfiguration = configuration;
        mListener = listener;
    }

    /**
     * Gets configuration for this re-constructor.
     *
     * @return configuration for this re-constructor.
     */
    public C getConfiguration() {
        return mConfiguration;
    }

    /**
     * Gets listener in charge of handling events such as when reconstruction
     * starts, ends, when certain data is needed or when estimation of data has
     * been computed.
     *
     * @return listener in charge of handling events.
     */
    public BaseTwoViewsSparseReconstructorListener<R> getListener() {
        return mListener;
    }

    /**
     * Indicates whether reconstruction is running or not.
     *
     * @return true if reconstruction is running, false if reconstruction has
     * stopped for any reason.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Indicates whether reconstruction has been cancelled or not.
     *
     * @return true if reconstruction has been cancelled, false otherwise.
     */
    public boolean isCancelled() {
        return mCancelled;
    }

    /**
     * Indicates whether reconstruction has failed or not.
     *
     * @return true if reconstruction has failed, false otherwise.
     */
    public boolean hasFailed() {
        return mFailed;
    }

    /**
     * Indicates whether the reconstruction has finished.
     *
     * @return true if reconstruction has finished, false otherwise.
     */
    public boolean isFinished() {
        return mFinished;
    }

    /**
     * Gets counter of number of processed views.
     *
     * @return counter of number of processed views.
     */
    public int getViewCount() {
        return mViewCount;
    }

    /**
     * Gets estimated fundamental matrix.
     *
     * @return estimated fundamental matrix.
     */
    public EstimatedFundamentalMatrix getEstimatedFundamentalMatrix() {
        return mEstimatedFundamentalMatrix;
    }

    /**
     * Gets estimated first camera.
     *
     * @return estimated first camera.
     */
    public EstimatedCamera getEstimatedCamera1() {
        return mEstimatedCamera1;
    }

    /**
     * Gets estimated second camera.
     *
     * @return estimated second camera.
     */
    public EstimatedCamera getEstimatedCamera2() {
        return mEstimatedCamera2;
    }

    /**
     * Gets reconstructed 3D points.
     *
     * @return reconstructed 3D points.
     */
    public List<ReconstructedPoint3D> getReconstructedPoints() {
        return mReconstructedPoints;
    }

    /**
     * Process one view of all the available data during the reconstruction.
     * This method can be called multiple times instead of {@link #start()} to build the reconstruction
     * step by step, one view at a time.
     *
     * @return true if more views can be processed, false when reconstruction has finished.
     */
    public boolean processOneView() {
        if (mViewCount == 0) {
            if (mRunning) {
                // already started
                return true;
            }

            reset();
            mRunning = true;

            //noinspection unchecked
            mListener.onStart((R) this);
        }

        //noinspection unchecked
        if (!mListener.hasMoreViewsAvailable((R) this)) {
            return false;
        }

        mEstimatedFundamentalMatrix = null;
        mCurrentViewSamples = new ArrayList<>();
        //noinspection unchecked
        mListener.onRequestSamplesForCurrentView((R) this, mViewCount,
                mCurrentViewSamples);

        if (mFirstViewSamples == null) {
            // for first view we simply keep samples (if enough are provided)
            if (hasEnoughSamples(mCurrentViewSamples)) {
                //noinspection unchecked
                mListener.onSamplesAccepted((R) this, mViewCount,
                        mCurrentViewSamples);
                mFirstViewSamples = mCurrentViewSamples;
                mFirstViewId = mViewCount;
            }

        } else {

            // for second view, check that we have enough samples
            if (hasEnoughSamples(mCurrentViewSamples)) {

                // find matches
                mMatches.clear();
                //noinspection unchecked
                mListener.onRequestMatches((R) this, mFirstViewSamples,
                        mCurrentViewSamples, mFirstViewId, mViewCount,
                        mMatches);

                if (hasEnoughMatches(mMatches)) {
                    // if enough matches are retrieved, attempt to compute
                    // fundamental matrix
                    if ((mConfiguration.isGeneralSceneAllowed() &&
                            estimateFundamentalMatrix(mMatches, mFirstViewId,
                                    mViewCount)) ||
                            (mConfiguration.isPlanarSceneAllowed() &&
                                    estimatePlanarFundamentalMatrix(mMatches,
                                            mFirstViewId, mViewCount))) {
                        // fundamental matrix could be estimated
                        //noinspection unchecked
                        mListener.onSamplesAccepted((R) this, mViewCount,
                                mCurrentViewSamples);
                        int secondViewId = mViewCount;

                        //noinspection unchecked
                        mListener.onFundamentalMatrixEstimated((R) this,
                                mEstimatedFundamentalMatrix);

                        if (estimateInitialCamerasAndPoints()) {
                            // cameras and points have been estimated
                            //noinspection unchecked
                            mListener.onCamerasEstimated((R) this,
                                    mFirstViewId, secondViewId,
                                    mEstimatedCamera1, mEstimatedCamera2);
                            //noinspection unchecked
                            mListener.onReconstructedPointsEstimated(
                                    (R) this, mMatches, mReconstructedPoints);
                            if (postProcessOne()) {
                                //noinspection unchecked
                                mListener.onFinish((R) this);
                                mRunning = false;
                                mFinished = true;
                            }
                        } else {
                            // initial cameras failed
                            mFailed = true;
                            //noinspection unchecked
                            mListener.onFail((R) this);
                        }
                    } else {
                        // estimation of fundamental matrix failed
                        //noinspection unchecked
                        mListener.onSamplesRejected((R) this, mViewCount,
                                mCurrentViewSamples);
                    }
                }
            }
        }

        mViewCount++;

        if (mCancelled) {
            //noinspection unchecked
            mListener.onCancel((R) this);
        }

        return !mFinished;
    }

    /**
     * Starts reconstruction of all available data to reconstruct the whole scene.
     * If reconstruction has already started and is running, calling this method
     * has no effect.
     */
    public void start() {
        while (processOneView()) {
            if (mCancelled) {
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
        if (mCancelled) {
            // already cancelled
            return;
        }

        mCancelled = true;
    }

    /**
     * Resets this instance so that a reconstruction can be started from the beginning without cancelling
     * current one.
     */
    public void reset() {
        mFirstViewSamples = mCurrentViewSamples = null;

        mCancelled = mFailed = false;
        mViewCount = 0;
        mRunning = false;

        mEstimatedFundamentalMatrix = null;
        mEstimatedCamera1 = mEstimatedCamera2 = null;
        mReconstructedPoints = null;

        mFinished = false;
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
        if (mConfiguration.isGeneralSceneAllowed()) {
            if (mConfiguration.getNonRobustFundamentalMatrixEstimatorMethod() ==
                    FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM) {
                return count >= EightPointsFundamentalMatrixEstimator.
                        MIN_REQUIRED_POINTS;
            } else if (mConfiguration.getNonRobustFundamentalMatrixEstimatorMethod() ==
                    FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM) {
                return count >= SevenPointsFundamentalMatrixEstimator.
                        MIN_REQUIRED_POINTS;
            }
        } else if (mConfiguration.isPlanarSceneAllowed()) {
            return count >= ProjectiveTransformation2DRobustEstimator.
                    MINIMUM_SIZE;
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
    private boolean estimateFundamentalMatrix(final List<MatchedSamples> matches,
                                              final int viewId1, final int viewId2) {
        if (matches == null) {
            return false;
        }

        final int count = matches.size();
        final List<Sample2D> leftSamples = new ArrayList<>(count);
        final List<Sample2D> rightSamples = new ArrayList<>(count);
        final List<Point2D> leftPoints = new ArrayList<>(count);
        final List<Point2D> rightPoints = new ArrayList<>(count);
        final double[] qualityScores = new double[count];
        final double principalPointX;
        final double principalPointY;
        if (mConfiguration.getInitialCamerasEstimatorMethod() ==
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC ||
                mConfiguration.getInitialCamerasEstimatorMethod() ==
                        InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX) {
            principalPointX = mConfiguration.getPrincipalPointX();
            principalPointY = mConfiguration.getPrincipalPointY();
        } else {
            principalPointX = principalPointY = 0.0;
        }

        int i = 0;
        for (final MatchedSamples match : matches) {
            final Sample2D[] samples = match.getSamples();
            if (samples.length != NUMBER_OF_VIEWS) {
                return false;
            }

            leftSamples.add(samples[0]);
            rightSamples.add(samples[1]);

            final Point2D leftPoint = Point2D.create();
            leftPoint.setInhomogeneousCoordinates(
                    samples[0].getPoint().getInhomX() - principalPointX,
                    samples[0].getPoint().getInhomY() - principalPointY);
            leftPoints.add(leftPoint);

            final Point2D rightPoint = Point2D.create();
            rightPoint.setInhomogeneousCoordinates(
                    samples[1].getPoint().getInhomX() - principalPointX,
                    samples[1].getPoint().getInhomY() - principalPointY);
            rightPoints.add(rightPoint);

            qualityScores[i] = match.getQualityScore();
            i++;
        }

        try {
            final FundamentalMatrixRobustEstimator estimator =
                    FundamentalMatrixRobustEstimator.create(leftPoints,
                            rightPoints, qualityScores, mConfiguration.
                                    getRobustFundamentalMatrixEstimatorMethod());
            estimator.setNonRobustFundamentalMatrixEstimatorMethod(
                    mConfiguration.
                            getNonRobustFundamentalMatrixEstimatorMethod());
            estimator.setResultRefined(
                    mConfiguration.isFundamentalMatrixRefined());
            estimator.setCovarianceKept(
                    mConfiguration.isFundamentalMatrixCovarianceKept());
            estimator.setConfidence(
                    mConfiguration.getFundamentalMatrixConfidence());
            estimator.setMaxIterations(
                    mConfiguration.getFundamentalMatrixMaxIterations());

            switch (mConfiguration.getRobustFundamentalMatrixEstimatorMethod()) {
                case LMEDS:
                    ((LMedSFundamentalMatrixRobustEstimator) estimator).
                            setStopThreshold(mConfiguration.
                                    getFundamentalMatrixThreshold());
                    break;
                case MSAC:
                    ((MSACFundamentalMatrixRobustEstimator) estimator).
                            setThreshold(mConfiguration.
                                    getFundamentalMatrixThreshold());
                    break;
                case PROMEDS:
                    ((PROMedSFundamentalMatrixRobustEstimator) estimator).
                            setStopThreshold(mConfiguration.
                                    getFundamentalMatrixThreshold());
                    break;
                case PROSAC:
                    PROSACFundamentalMatrixRobustEstimator prosacEstimator =
                            (PROSACFundamentalMatrixRobustEstimator) estimator;
                    prosacEstimator.setThreshold(
                            mConfiguration.getFundamentalMatrixThreshold());
                    prosacEstimator.setComputeAndKeepInliersEnabled(
                            mConfiguration.
                                    getFundamentalMatrixComputeAndKeepInliers());
                    prosacEstimator.setComputeAndKeepResidualsEnabled(
                            mConfiguration.
                                    getFundamentalMatrixComputeAndKeepResiduals());
                    break;
                case RANSAC:
                    RANSACFundamentalMatrixRobustEstimator ransacEstimator =
                            (RANSACFundamentalMatrixRobustEstimator) estimator;
                    ransacEstimator.setThreshold(
                            mConfiguration.getFundamentalMatrixThreshold());
                    ransacEstimator.setComputeAndKeepInliersEnabled(
                            mConfiguration.
                                    getFundamentalMatrixComputeAndKeepInliers());
                    ransacEstimator.setComputeAndKeepResidualsEnabled(
                            mConfiguration.
                                    getFundamentalMatrixComputeAndKeepResiduals());
                    break;
                default:
                    break;
            }


            final FundamentalMatrix fundamentalMatrix = estimator.estimate();

            mEstimatedFundamentalMatrix = new EstimatedFundamentalMatrix();
            mEstimatedFundamentalMatrix.setFundamentalMatrix(fundamentalMatrix);
            mEstimatedFundamentalMatrix.setViewId1(viewId1);
            mEstimatedFundamentalMatrix.setViewId2(viewId2);
            mEstimatedFundamentalMatrix.setCovariance(
                    estimator.getCovariance());

            // determine quality score and inliers
            final InliersData inliersData = estimator.getInliersData();
            if (inliersData != null) {
                final int numInliers = inliersData.getNumInliers();
                final BitSet inliers = inliersData.getInliers();
                final int length = inliers.length();
                double fundamentalMatrixQualityScore = 0.0;
                for (i = 0; i < length; i++) {
                    if (inliers.get(i)) {
                        // inlier
                        fundamentalMatrixQualityScore +=
                                qualityScores[i] / numInliers;
                    }
                }
                mEstimatedFundamentalMatrix.setQualityScore(
                        fundamentalMatrixQualityScore);
                mEstimatedFundamentalMatrix.setInliers(inliers);
            }

            // store left/right samples
            mEstimatedFundamentalMatrix.setLeftSamples(leftSamples);
            mEstimatedFundamentalMatrix.setRightSamples(rightSamples);

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

        final int count = matches.size();
        final List<Sample2D> leftSamples = new ArrayList<>();
        final List<Sample2D> rightSamples = new ArrayList<>();
        final List<Point2D> leftPoints = new ArrayList<>();
        final List<Point2D> rightPoints = new ArrayList<>();
        final double[] qualityScores = new double[count];
        final double principalPointX;
        final double principalPointY;
        if (mConfiguration.getInitialCamerasEstimatorMethod() ==
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC ||
                mConfiguration.getInitialCamerasEstimatorMethod() ==
                        InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX) {
            principalPointX = mConfiguration.getPrincipalPointX();
            principalPointY = mConfiguration.getPrincipalPointY();
        } else {
            principalPointX = principalPointY = 0.0;
        }

        int i = 0;
        for (final MatchedSamples match : matches) {
            final Sample2D[] samples = match.getSamples();
            if (samples.length != NUMBER_OF_VIEWS) {
                return false;
            }

            leftSamples.add(samples[0]);
            rightSamples.add(samples[1]);

            final Point2D leftPoint = Point2D.create();
            leftPoint.setInhomogeneousCoordinates(
                    samples[0].getPoint().getInhomX() - principalPointX,
                    samples[0].getPoint().getInhomY() - principalPointY);
            leftPoints.add(leftPoint);

            final Point2D rightPoint = Point2D.create();
            rightPoint.setInhomogeneousCoordinates(
                    samples[1].getPoint().getInhomX() - principalPointX,
                    samples[1].getPoint().getInhomY() - principalPointY);
            rightPoints.add(rightPoint);

            qualityScores[i] = match.getQualityScore();
            i++;
        }

        try {
            final PointCorrespondenceProjectiveTransformation2DRobustEstimator
                    homographyEstimator =
                    PointCorrespondenceProjectiveTransformation2DRobustEstimator.
                            create(mConfiguration.
                                    getRobustPlanarHomographyEstimatorMethod());
            homographyEstimator.setResultRefined(
                    mConfiguration.isPlanarHomographyRefined());
            homographyEstimator.setCovarianceKept(
                    mConfiguration.isPlanarHomographyCovarianceKept());
            homographyEstimator.setConfidence(
                    mConfiguration.getPlanarHomographyConfidence());
            homographyEstimator.setMaxIterations(
                    mConfiguration.getPlanarHomographyMaxIterations());

            switch (mConfiguration.getRobustPlanarHomographyEstimatorMethod()) {
                case LMEDS:
                    ((LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                            homographyEstimator).setStopThreshold(
                            mConfiguration.getPlanarHomographyThreshold());
                    break;
                case MSAC:
                    ((MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                            homographyEstimator).setThreshold(
                            mConfiguration.getPlanarHomographyThreshold());
                    break;
                case PROMEDS:
                    ((PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                            homographyEstimator).setStopThreshold(
                            mConfiguration.getPlanarHomographyThreshold());
                    break;
                case PROSAC:
                    final PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator prosacHomographyEstimator =
                            (PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator;

                    prosacHomographyEstimator.setThreshold(
                            mConfiguration.getPlanarHomographyThreshold());
                    prosacHomographyEstimator.setComputeAndKeepInliersEnabled(
                            mConfiguration.getPlanarHomographyComputeAndKeepInliers());
                    prosacHomographyEstimator.setComputeAndKeepResidualsEnabled(
                            mConfiguration.getPlanarHomographyComputeAndKeepResiduals());
                    break;
                case RANSAC:
                default:
                    final RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator ransacHomographyEstimator =
                            (RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator;

                    ransacHomographyEstimator.setThreshold(
                            mConfiguration.getPlanarHomographyThreshold());
                    ransacHomographyEstimator.setComputeAndKeepInliersEnabled(
                            mConfiguration.getPlanarHomographyComputeAndKeepInliers());
                    ransacHomographyEstimator.setComputeAndKeepResidualsEnabled(
                            mConfiguration.getPlanarHomographyComputeAndKeepResiduals());
                    break;
            }

            final PlanarBestFundamentalMatrixEstimatorAndReconstructor
                    fundamentalMatrixEstimator =
                    new PlanarBestFundamentalMatrixEstimatorAndReconstructor();
            fundamentalMatrixEstimator.setHomographyEstimator(
                    homographyEstimator);
            fundamentalMatrixEstimator.setLeftAndRightPoints(leftPoints,
                    rightPoints);
            fundamentalMatrixEstimator.setQualityScores(qualityScores);

            PinholeCameraIntrinsicParameters intrinsic1 =
                    mConfiguration.getInitialIntrinsic1();
            PinholeCameraIntrinsicParameters intrinsic2 =
                    mConfiguration.getInitialIntrinsic1();
            if (intrinsic1 == null && intrinsic2 == null) {
                // estimate homography
                final ProjectiveTransformation2D homography = homographyEstimator.
                        estimate();

                // estimate intrinsic parameters using the Image of Absolute
                // Conic (IAC)
                final List<Transformation2D> homographies =
                        new ArrayList<>();
                homographies.add(homography);

                final ImageOfAbsoluteConicEstimator iacEstimator =
                        new LMSEImageOfAbsoluteConicEstimator(homographies);
                final ImageOfAbsoluteConic iac = iacEstimator.estimate();

                intrinsic1 = intrinsic2 = iac.getIntrinsicParameters();

            } else if (intrinsic1 == null) { // && intrinsic2 != null
                intrinsic1 = intrinsic2;
            } else if (intrinsic2 == null) { // && intrinsic1 != null
                intrinsic2 = intrinsic1;
            }
            fundamentalMatrixEstimator.setLeftIntrinsics(intrinsic1);
            fundamentalMatrixEstimator.setRightIntrinsics(intrinsic2);

            fundamentalMatrixEstimator.estimateAndReconstruct();

            final FundamentalMatrix fundamentalMatrix =
                    fundamentalMatrixEstimator.getFundamentalMatrix();

            mEstimatedFundamentalMatrix = new EstimatedFundamentalMatrix();
            mEstimatedFundamentalMatrix.setFundamentalMatrix(fundamentalMatrix);
            mEstimatedFundamentalMatrix.setViewId1(viewId1);
            mEstimatedFundamentalMatrix.setViewId2(viewId2);

            // determine quality score and inliers
            final InliersData inliersData = homographyEstimator.getInliersData();
            if (inliersData != null) {
                final int numInliers = inliersData.getNumInliers();
                final BitSet inliers = inliersData.getInliers();
                final int length = inliers.length();
                double fundamentalMatrixQualityScore = 0.0;
                for (i = 0; i < length; i++) {
                    if (inliers.get(i)) {
                        // inlier
                        fundamentalMatrixQualityScore +=
                                qualityScores[i] / numInliers;
                    }
                }
                mEstimatedFundamentalMatrix.setQualityScore(
                        fundamentalMatrixQualityScore);
                mEstimatedFundamentalMatrix.setInliers(inliers);
            }

            // store left/right samples
            mEstimatedFundamentalMatrix.setLeftSamples(leftSamples);
            mEstimatedFundamentalMatrix.setRightSamples(rightSamples);

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
        switch (mConfiguration.getInitialCamerasEstimatorMethod()) {
            case ESSENTIAL_MATRIX:
                return estimateInitialCamerasAndPointsEssential();
            case DUAL_IMAGE_OF_ABSOLUTE_CONIC:
                return estimateInitialCamerasAndPointsDIAC();
            case DUAL_ABSOLUTE_QUADRIC:
                return estimateInitialCamerasAndPointsDAQ();
            case DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX:
            default:
                return estimateInitialCamerasAndPointsDAQAndEssential();
        }
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
            final FundamentalMatrix fundamentalMatrix =
                    mEstimatedFundamentalMatrix.getFundamentalMatrix();

            final DualAbsoluteQuadricInitialCamerasEstimator estimator =
                    new DualAbsoluteQuadricInitialCamerasEstimator(
                            fundamentalMatrix);
            estimator.setAspectRatio(
                    mConfiguration.getInitialCamerasAspectRatio());
            estimator.estimate();

            final PinholeCamera camera1 = estimator.getEstimatedLeftCamera();
            final PinholeCamera camera2 = estimator.getEstimatedRightCamera();

            camera1.decompose();
            camera2.decompose();

            final PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint1 =
                    camera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint2 =
                    camera2.getIntrinsicParameters();

            final double principalPointX = mConfiguration.getPrincipalPointX();
            final double principalPointY = mConfiguration.getPrincipalPointY();

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(
                            intrinsicZeroPrincipalPoint1);
            intrinsic1.setHorizontalPrincipalPoint(
                    intrinsic1.getHorizontalPrincipalPoint() + principalPointX);
            intrinsic1.setVerticalPrincipalPoint(
                    intrinsic1.getVerticalPrincipalPoint() + principalPointY);

            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(
                            intrinsicZeroPrincipalPoint2);
            intrinsic2.setHorizontalPrincipalPoint(
                    intrinsic2.getHorizontalPrincipalPoint() + principalPointX);
            intrinsic2.setVerticalPrincipalPoint(
                    intrinsic2.getVerticalPrincipalPoint() + principalPointY);

            // fix fundamental matrix to account for principal point different
            // from zero
            fixFundamentalMatrix(fundamentalMatrix,
                    intrinsicZeroPrincipalPoint1, intrinsicZeroPrincipalPoint2,
                    intrinsic1, intrinsic2);

            return estimateInitialCamerasAndPointsEssential(intrinsic1,
                    intrinsic2);
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
            final FundamentalMatrix fundamentalMatrix =
                    mEstimatedFundamentalMatrix.getFundamentalMatrix();
            fundamentalMatrix.normalize();

            final DualAbsoluteQuadricInitialCamerasEstimator estimator =
                    new DualAbsoluteQuadricInitialCamerasEstimator(
                            fundamentalMatrix);
            estimator.setAspectRatio(
                    mConfiguration.getInitialCamerasAspectRatio());
            estimator.estimate();

            final PinholeCamera camera1 = estimator.getEstimatedLeftCamera();
            final PinholeCamera camera2 = estimator.getEstimatedRightCamera();

            camera1.decompose();
            camera2.decompose();

            final PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint1 =
                    camera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint2 =
                    camera2.getIntrinsicParameters();

            final double principalPointX = mConfiguration.getPrincipalPointX();
            final double principalPointY = mConfiguration.getPrincipalPointY();

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(
                            intrinsicZeroPrincipalPoint1);
            intrinsic1.setHorizontalPrincipalPoint(
                    intrinsic1.getHorizontalPrincipalPoint() + principalPointX);
            intrinsic1.setVerticalPrincipalPoint(
                    intrinsic1.getVerticalPrincipalPoint() + principalPointY);
            camera1.setIntrinsicParameters(intrinsic1);

            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(
                            intrinsicZeroPrincipalPoint2);
            intrinsic2.setHorizontalPrincipalPoint(
                    intrinsic2.getHorizontalPrincipalPoint() + principalPointX);
            intrinsic2.setVerticalPrincipalPoint(
                    intrinsic2.getVerticalPrincipalPoint() + principalPointY);
            camera2.setIntrinsicParameters(intrinsic2);

            mEstimatedCamera1 = new EstimatedCamera();
            mEstimatedCamera1.setCamera(camera1);

            mEstimatedCamera2 = new EstimatedCamera();
            mEstimatedCamera2.setCamera(camera2);

            // fix fundamental matrix to account for principal point different
            // from zero
            fixFundamentalMatrix(fundamentalMatrix,
                    intrinsicZeroPrincipalPoint1, intrinsicZeroPrincipalPoint2,
                    intrinsic1, intrinsic2);

            // triangulate points
            Corrector corrector = null;
            if (mConfiguration.getInitialCamerasCorrectorType() != null) {
                corrector = Corrector.create(fundamentalMatrix,
                        mConfiguration.getInitialCamerasCorrectorType());
            }

            // use all points used for fundamental matrix estimation
            final List<Sample2D> samples1 = mEstimatedFundamentalMatrix.getLeftSamples();
            final List<Sample2D> samples2 = mEstimatedFundamentalMatrix.getRightSamples();

            final List<Point2D> points1 = new ArrayList<>();
            final List<Point2D> points2 = new ArrayList<>();
            final int length = samples1.size();
            for (int i = 0; i < length; i++) {
                final Sample2D sample1 = samples1.get(i);
                final Sample2D sample2 = samples2.get(i);

                final Point2D point1 = sample1.getPoint();
                final Point2D point2 = sample2.getPoint();

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
            if (mConfiguration.getDaqUseHomogeneousPointTriangulator()) {
                triangulator = SinglePoint3DTriangulator.create(
                        Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR);
            } else {
                triangulator = SinglePoint3DTriangulator.create(
                        Point3DTriangulatorType.
                                LMSE_INHOMOGENEOUS_TRIANGULATOR);
            }

            final List<PinholeCamera> cameras = new ArrayList<>();
            cameras.add(camera1);
            cameras.add(camera2);

            mReconstructedPoints = new ArrayList<>();
            final List<Point2D> points = new ArrayList<>();
            final int numPoints = correctedPoints1.size();
            Point3D triangulatedPoint;
            ReconstructedPoint3D reconstructedPoint;
            for (int i = 0; i < numPoints; i++) {
                points.clear();
                points.add(correctedPoints1.get(i));
                points.add(correctedPoints2.get(i));

                triangulator.setPointsAndCameras(points, cameras);
                triangulatedPoint = triangulator.triangulate();

                reconstructedPoint = new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoint);

                // only points reconstructed in front of both cameras are
                // considered valid
                final boolean front1 = camera1.isPointInFrontOfCamera(
                        triangulatedPoint);
                final boolean front2 = camera2.isPointInFrontOfCamera(
                        triangulatedPoint);
                reconstructedPoint.setInlier(front1 && front2);

                mReconstructedPoints.add(reconstructedPoint);
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
        final FundamentalMatrix fundamentalMatrix =
                mEstimatedFundamentalMatrix.getFundamentalMatrix();

        // use inlier points used for fundamental matrix estimation
        final List<Sample2D> samples1 = mEstimatedFundamentalMatrix.getLeftSamples();
        final List<Sample2D> samples2 = mEstimatedFundamentalMatrix.getRightSamples();

        final List<Point2D> points1 = new ArrayList<>();
        final List<Point2D> points2 = new ArrayList<>();
        final int length = samples1.size();
        for (int i = 0; i < length; i++) {
            final Sample2D sample1 = samples1.get(i);
            final Sample2D sample2 = samples2.get(i);

            final Point2D point1 = sample1.getPoint();
            final Point2D point2 = sample2.getPoint();

            points1.add(point1);
            points2.add(point2);
        }

        try {
            final DualImageOfAbsoluteConicInitialCamerasEstimator estimator =
                    new DualImageOfAbsoluteConicInitialCamerasEstimator(
                            fundamentalMatrix, points1, points2);
            estimator.setPrincipalPoint(mConfiguration.getPrincipalPointX(),
                    mConfiguration.getPrincipalPointY());
            estimator.setAspectRatio(
                    mConfiguration.getInitialCamerasAspectRatio());
            estimator.setCorrectorType(
                    mConfiguration.getInitialCamerasCorrectorType());
            estimator.setPointsTriangulated(true);
            estimator.setValidTriangulatedPointsMarked(
                    mConfiguration.getInitialCamerasMarkValidTriangulatedPoints());

            estimator.estimate();

            // store cameras
            final PinholeCamera camera1 = estimator.getEstimatedLeftCamera();
            final PinholeCamera camera2 = estimator.getEstimatedRightCamera();

            mEstimatedCamera1 = new EstimatedCamera();
            mEstimatedCamera1.setCamera(camera1);

            mEstimatedCamera2 = new EstimatedCamera();
            mEstimatedCamera2.setCamera(camera2);

            // store points
            final List<Point3D> triangulatedPoints =
                    estimator.getTriangulatedPoints();
            final BitSet validTriangulatedPoints =
                    estimator.getValidTriangulatedPoints();

            mReconstructedPoints = new ArrayList<>();
            final int size = triangulatedPoints.size();
            for (int i = 0; i < size; i++) {
                final ReconstructedPoint3D reconstructedPoint =
                        new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoints.get(i));
                reconstructedPoint.setInlier(validTriangulatedPoints.get(i));
                mReconstructedPoints.add(reconstructedPoint);
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
        final PinholeCameraIntrinsicParameters intrinsic1 =
                mConfiguration.getInitialIntrinsic1();
        final PinholeCameraIntrinsicParameters intrinsic2 =
                mConfiguration.getInitialIntrinsic2();
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
        final FundamentalMatrix fundamentalMatrix =
                mEstimatedFundamentalMatrix.getFundamentalMatrix();

        // use all points used for fundamental matrix estimation
        final List<Sample2D> samples1 = mEstimatedFundamentalMatrix.getLeftSamples();
        final List<Sample2D> samples2 = mEstimatedFundamentalMatrix.getRightSamples();

        final List<Point2D> points1 = new ArrayList<>();
        final List<Point2D> points2 = new ArrayList<>();
        final int length = samples1.size();
        for (int i = 0; i < length; i++) {
            final Sample2D sample1 = samples1.get(i);
            final Sample2D sample2 = samples2.get(i);

            final Point2D point1 = sample1.getPoint();
            final Point2D point2 = sample2.getPoint();

            points1.add(point1);
            points2.add(point2);
        }

        try {
            final EssentialMatrixInitialCamerasEstimator estimator =
                    new EssentialMatrixInitialCamerasEstimator(
                            fundamentalMatrix, intrinsic1, intrinsic2,
                            points1, points2);

            estimator.setCorrectorType(
                    mConfiguration.getInitialCamerasCorrectorType());
            estimator.setPointsTriangulated(true);
            estimator.setValidTriangulatedPointsMarked(
                    mConfiguration.getInitialCamerasMarkValidTriangulatedPoints());

            estimator.estimate();

            // store cameras
            final PinholeCamera camera1 = estimator.getEstimatedLeftCamera();
            final PinholeCamera camera2 = estimator.getEstimatedRightCamera();

            mEstimatedCamera1 = new EstimatedCamera();
            mEstimatedCamera1.setCamera(camera1);

            mEstimatedCamera2 = new EstimatedCamera();
            mEstimatedCamera2.setCamera(camera2);

            // store points
            final List<Point3D> triangulatedPoints =
                    estimator.getTriangulatedPoints();
            final BitSet validTriangulatedPoints =
                    estimator.getValidTriangulatedPoints();

            mReconstructedPoints = new ArrayList<>();
            final int size = triangulatedPoints.size();
            for (int i = 0; i < size; i++) {
                final ReconstructedPoint3D reconstructedPoint =
                        new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoints.get(i));
                reconstructedPoint.setInlier(validTriangulatedPoints.get(i));
                mReconstructedPoints.add(reconstructedPoint);
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
        final EssentialMatrix essential = new EssentialMatrix(fundamentalMatrix,
                intrinsicZeroPrincipalPoint1, intrinsicZeroPrincipalPoint2);
        final FundamentalMatrix fixedFundamentalMatrix =
                essential.toFundamentalMatrix(intrinsicPrincipalPoint1,
                        intrinsicPrincipalPoint2);
        fixedFundamentalMatrix.normalize();
        mEstimatedFundamentalMatrix.setFundamentalMatrix(
                fixedFundamentalMatrix);
        mEstimatedFundamentalMatrix.setCovariance(null);
    }
}
