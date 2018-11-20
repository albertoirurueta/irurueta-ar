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
import com.irurueta.ar.calibration.ImageOfAbsoluteConic;
import com.irurueta.ar.calibration.estimators.ImageOfAbsoluteConicEstimator;
import com.irurueta.ar.calibration.estimators.LMSEImageOfAbsoluteConicEstimator;
import com.irurueta.ar.epipolar.Corrector;
import com.irurueta.ar.epipolar.EpipolarException;
import com.irurueta.ar.epipolar.EssentialMatrix;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.estimators.*;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.*;
import com.irurueta.numerical.robust.InliersData;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * Base class in charge of estimating cameras and 3D reconstructed points from sparse
 * image point correspondences in pairs of views.
 * Views are processed in pairs so that fundamental matrix is estimated and pairs of
 * cameras and reconstructed points are computed.
 * Because view pairs are processed separately, the scale of each view pair is
 * estimated individually, hence the scale will need to be
 * @param <C> type of configuration.
 * @param <R> type of reconstructor.
 * @param <L> type of listener.
 */
public abstract class BasePairedViewsSparseReconstructor<
        C extends BasePairedViewsSparseReconstructorConfiguration,
        R extends BasePairedViewsSparseReconstructor,
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
    protected EstimatedCamera mCurrentMetricEstimatedCamera;

    /**
     * Previous estimated camera in a metric stratum (i.e. up to scale).
     */
    protected EstimatedCamera mPreviousMetricEstimatedCamera;

    /**
     * Reconstructed 3D points for current pair of views in a metric stratum (i.e. up to scale).
     */
    protected List<ReconstructedPoint3D> mMetricReconstructedPoints;

    /**
     * Transformation to set reference frame on estimated pair of euclidean cameras.
     * This is used when estimating new pair of euclidean cameras to transform such pair to
     * the location and rotation of last estimasted euclidean camera so that the first camera
     * of the pair is not referred to the world origin.
     */
    protected MetricTransformation3D mReferenceEuclideanTransformation;

    /**
     * Current estimated scale. This will typically converge to a constant value as more views are processed.
     * The smaller the variance of estimated scale, the more accurate the scale will be.
     */
    protected double mCurrentScale = DEFAULT_SCALE;

    /**
     * Current estimated camera in euclidean stratum (i.e. with actual scale).
     */
    protected EstimatedCamera mCurrentEuclideanEstimatedCamera;

    /**
     * Previous estimated camera in euclidean stratum (i.e. with actual scale).
     */
    protected EstimatedCamera mPreviousEuclideanEstimatedCamera;

    /**
     * Reconstructed 3D points for current pair of views in euclidean stratum (i.e. with actual
     * scale).
     */
    protected List<ReconstructedPoint3D> mEuclideanReconstructedPoints;

    /**
     * Configuration for this reconstructor.
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
     * Id of previous view.
     */
    protected int mPreviousViewId = 0;

    /**
     * Id of current view.
     */
    protected int mCurrentViewId;

    /**
     * Center of current euclidean camera on last view pair.
     */
    protected Point3D mLastEuclideanCameraCenter = new InhomogeneousPoint3D();

    /**
     * Rotation of current euclidean camera on last view pair.
     */
    protected Rotation3D mLastEuclideanCameraRotation;

    /**
     * Center of current metric camera on last view pair.
     */
    private Point3D mLastMetricCameraCenter;

    /**
     * Rotation of current metric camera on last view pair.
     */
    private Rotation3D mLastMetricCameraRotation;

    /**
     * Inverse metric camera rotation. This is reused for memory efficiency.
     */
    private Rotation3D mInvMetricCameraRotation;

    /**
     * Current estimated fundamental matrix.
     */
    private EstimatedFundamentalMatrix mCurrentEstimatedFundamentalMatrix;

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
     * Samples on previous view.
     */
    private List<Sample2D> mPreviousViewSamples;

    /**
     * Samples on last processed view (i.e. current view).
     */
    private List<Sample2D> mCurrentViewSamples;

    /**
     * Matches between first and current view.
     * Views are always processed in pairs.
     */
    private List<MatchedSamples> mMatches = new ArrayList<>();

    /**
     * Transformation to set reference frame on estimated pair of metric cameras.
     * This is used when estimating new pair of metric cameras to transform such pair to
     * the location and rotation of last estimated metric camera so that the first camera of
     * the pair is not referred to the world origin.
     */
    private EuclideanTransformation3D mReferenceMetricTransformation;

    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     * provided.
     */
    public BasePairedViewsSparseReconstructor(C configuration,
                                              L listener) throws NullPointerException {
        if  (configuration == null || listener == null) {
            throw new NullPointerException();
        }
        mConfiguration = configuration;
        mListener = listener;
    }

    /**
     * Gets configuration for this reconstructor.
     * @return configuration for this reconstructor.
     */
    public C getConfiguration() {
        return mConfiguration;
    }

    /**
     * Gets listener in charge of handling events such as when reconstruction
     * starts, ends, when certain data is needed or when estimation of data has
     * been computed.
     * @return listener in charge of handling events.
     */
    public L getListener() {
        return mListener;
    }

    /**
     * Indicates whether reconstruction is running or not.
     * @return true if reconstruction is running, false if reconstruction has
     * stopped for any reason.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Indicates whether reconstruction has been cancelled or not.
     * @return true if reconstruction has been cancelled, false otherwise.
     */
    public boolean isCancelled() {
        return mCancelled;
    }

    /**
     * Indicates whether reconstruction has failed or not.
     * @return true if reconstruction has failed, false otherwise.
     */
    public boolean hasFailed() {
        return mFailed;
    }

    /**
     * Indicates whether the reconstruction has finished.
     * @return true if reconstruction has finished, false otherwise.
     */
    public boolean isFinished() {
        return mFinished;
    }

    /**
     * Gets counter of number of processed views.
     * @return counter of number of processed views.
     */
    public int getViewCount() {
        return mViewCount;
    }

    /**
     * Gets estimated fundamental matrix for current view.
     * This fundamental matrix relates current view with the previously processed one.
     * @return current estimated fundamental matrix.
     */
    public EstimatedFundamentalMatrix getCurrentEstimatedFundamentalMatrix() {
        return mCurrentEstimatedFundamentalMatrix;
    }

    /**
     * Gets estimated euclidean camera for current view (i.e. with actual scale).
     * @return current estimated euclidean camera.
     */
    public EstimatedCamera getCurrentEuclideanEstimatedCamera() {
        return mCurrentEuclideanEstimatedCamera;
    }

    /**
     * Gets estimated euclidean camera for previous view (i.e. with actual scale).
     * @return previous estimated euclidean camera.
     */
    public EstimatedCamera getPreviousEuclideanEstimatedCamera() {
        return mPreviousEuclideanEstimatedCamera;
    }

    /**
     * Gets euclidean reconstructed 3D points (i.e. with actual scale) for current
     * pair of views.
     * @return active euclidean reconstructed 3D points.
     */
    public List<ReconstructedPoint3D> getEuclideanReconstructedPoints() {
        return mEuclideanReconstructedPoints;
    }

    /**
     * Gets current estimated scale. This will typically converge to a constant value as more views are processed.
     * The smaller the variance of estimated scale, the more accurate the scale will be.
     * @return current estimated scale.
     */
    public double getCurrentScale() {
        return mCurrentScale;
    }

    /**
     * Gets samples on previous view.
     * @return samples on previous view.
     */
    public List<Sample2D> getPreviousViewSamples() {
        return mPreviousViewSamples;
    }

    /**
     * Gets samples on current view.
     * @return samples on current view.
     */
    public List<Sample2D> getCurrentViewSamples() {
        return mCurrentViewSamples;
    }

    /**
     * Process one view pair of all the available data during the reconstruction.
     * This method can be called multiple times instead of {@link #start()} to build the
     * reconstruction step by step, one view pair at a time.
     * This method is useful when data is gathered on real time from a camera and the
     * number of views is unknown.
     * @return true if more views can be processed, false when reconstruction has finished.
     */
    public boolean processOneViewPair() {
        if (mViewCount == 0 && !mRunning) {

            reset();
            mRunning = true;

            //noinspection unchecked
            mListener.onStart((R)this);
        }

        //noinspection unchecked
        if(!mListener.hasMoreViewsAvailable((R)this)) {
            //noinspection unchecked
            mListener.onFinish((R)this);
            mRunning = false;
            mFinished = true;
            return false;
        }

        mPreviousViewSamples = new ArrayList<>();
        mCurrentViewSamples = new ArrayList<>();
        //noinspection unchecked
        mListener.onRequestSamplesForCurrentViewPair((R)this,
                mViewCount, mViewCount + 1,
                mPreviousViewSamples, mCurrentViewSamples);

        boolean processed;
        mCurrentEstimatedFundamentalMatrix = null;
        if (isFirstViewPair()) {
            //for first view we simply keep samples (if enough are provided)
            processed = processFirstViewPair();
        } else {
            processed = processAdditionalViewPair();
        }

        if (processed) {
            mViewCount += 2;
        }

        if (mCancelled) {
            //noinspection unchecked
            mListener.onCancel((R)this);
        }

        return !mFinished;
    }

    /**
     * Indicates whether current view pair is the first one.
     * @return true if current view pair is the first one, false otherwise.
     */
    public boolean isFirstViewPair() {
        return mViewCount == 0;
    }

    /**
     * Indicates whether current view pair is an additional one.
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
        if (mRunning) {
            //already started
            return;
        }

        while(processOneViewPair()) {
            if (mCancelled) {
                break;
            }
        }
    }

    /**
     * Cancels reconstruction.
     * If reconstruction has already been cancelled, calling this method has no effect.
     */
    public void cancel() {
        if (mCancelled) {
            //already cancelled
            return;
        }

        mCancelled = true;
    }

    /**
     * Resets this instance so that a reconstruction can be started from the beginning without cancelling current one.
     */
    public void reset() {
        if (mPreviousViewSamples != null) {
            mPreviousViewSamples.clear();
        }
        if (mCurrentViewSamples != null) {
            mCurrentViewSamples.clear();
        }
        if (mMatches != null) {
            mMatches.clear();
        }

        mCancelled = mFailed = false;
        mViewCount = 0;
        mRunning = false;

        mCurrentEstimatedFundamentalMatrix = null;
        mCurrentMetricEstimatedCamera = mPreviousMetricEstimatedCamera = null;
        mMetricReconstructedPoints = null;
        mCurrentScale = DEFAULT_SCALE;
        mCurrentEuclideanEstimatedCamera = mPreviousEuclideanEstimatedCamera = null;
        mEuclideanReconstructedPoints = null;

        mPreviousViewId = 0;
        mCurrentViewId = 0;

        mFinished = false;
    }

    /**
     * Gets estimated metric camera for current view (i.e. up to scale).
     * @return current estimated metric camera.
     */
    protected EstimatedCamera getCurrentMetricEstimatedCamera() {
        return mCurrentMetricEstimatedCamera;
    }

    /**
     * Gets estimated camera for previous view (i.e. up to scale).
     * @return previous estimated metric camera.
     */
    protected EstimatedCamera getPreviousMetricEstimatedCamera() {
        return mPreviousMetricEstimatedCamera;
    }

    /**
     * Gets metric reconstructed 3D points (i.e. up to scale) for current pair of views.
     * @return active metric reconstructed 3D points.
     */
    protected List<ReconstructedPoint3D> getMetricReconstructedPoints() {
        return mMetricReconstructedPoints;
    }

    /**
     * Transforms cameras on current pair of views so that they are referred to
     * last kept location and rotation and upgrades cameras from metric stratum to
     * euclidean stratum.
     * @param isInitialPairOfViews true if initial pair of views is being processed, false otherwise.
     * @param hasAbsoluteOrientation true if absolute orientation is required, false otherwise.
     * @return true if cameras were successfully transformed.
     */
    protected boolean transformPairOfCamerasAndPoints(boolean isInitialPairOfViews,
                                                      boolean hasAbsoluteOrientation) {
        if (isInitialPairOfViews) {
            //initial pair does not need transformation
            return true;
        }

        if (mPreviousMetricEstimatedCamera == null || mCurrentMetricEstimatedCamera == null) {
            return false;
        }

        PinholeCamera previousMetricCamera = mPreviousMetricEstimatedCamera.getCamera();
        PinholeCamera currentMetricCamera = mCurrentMetricEstimatedCamera.getCamera();
        if (previousMetricCamera == null || currentMetricCamera == null) {
            return false;
        }

        if (mInvMetricCameraRotation == null) {
            mInvMetricCameraRotation = mLastMetricCameraRotation.inverseRotationAndReturnNew();
        } else {
            mLastMetricCameraRotation.inverseRotation(mInvMetricCameraRotation);
        }

        if (mReferenceMetricTransformation == null) {
            mReferenceMetricTransformation = new EuclideanTransformation3D(mInvMetricCameraRotation);
            mReferenceMetricTransformation.setTranslation(mLastMetricCameraCenter);
        } else {
            mReferenceMetricTransformation.setRotation(mInvMetricCameraRotation);
            mReferenceMetricTransformation.setTranslation(mLastMetricCameraCenter);
        }

        try {
            mReferenceMetricTransformation.transform(previousMetricCamera);
            mReferenceMetricTransformation.transform(currentMetricCamera);

            Point3D p;
            for (ReconstructedPoint3D metricReconstructedPoint : mMetricReconstructedPoints) {
                p = metricReconstructedPoint.getPoint();
                mReferenceMetricTransformation.transform(p, p);
            }
            return true;
        } catch (AlgebraException e) {
            return false;
        }
    }

    /**
     * Processes data for the first view pair.
     * @return true if view pair was successfully processed, false otherwise.
     */
    private boolean processFirstViewPair() {
        return processViewPair(true);
    }

    /**
     * Processes data for an additional view pair.
     * @return true if view pair was successfully processed, false otherwise.
     */
    private boolean processAdditionalViewPair() {
        return processViewPair(false);
    }

    /**
     * Processed data for a view pair.
     * @param isInitialPairOfViews true if initial pair of views is being processed,
     *                             false otherwise.
     * @return true if view pair was successfully processed, false otherwise.
     */
    private boolean processViewPair(boolean isInitialPairOfViews) {
        //for second view, check that we have enough samples
        if (hasEnoughSamples(mCurrentViewSamples)) {

            //find matches
            mMatches.clear();
            //noinspection unchecked
            int viewId1 = mViewCount;
            int viewId2 = mViewCount + 1;
            //noinspection unchecked
            mListener.onRequestMatches((R)this, viewId1, viewId2,
                    mPreviousViewSamples, mCurrentViewSamples, mMatches);

            if (hasEnoughMatches(mMatches)) {
                //if enough matches are retrieved, attempt to compute
                //fundamental matrix
                if ((mConfiguration.isGeneralSceneAllowed() &&
                        estimateFundamentalMatrix(mMatches, viewId1, viewId2)) ||
                        (mConfiguration.isPlanarSceneAllowed() &&
                        estimatePlanarFundamentalMatrix(mMatches, viewId1, viewId2))) {
                    //fundamental matrix could be estimated
                    //noinspection unchecked
                    mListener.onSamplesAccepted((R)this, viewId1, viewId2,
                            mPreviousViewSamples, mCurrentViewSamples);
                    mPreviousViewId = viewId1;
                    mCurrentViewId = viewId2;

                    //noinspection unchecked
                    mListener.onFundamentalMatrixEstimated((R)this, viewId1, viewId2,
                            mCurrentEstimatedFundamentalMatrix);

                    if(estimatePairOfCamerasAndPoints(isInitialPairOfViews)) {
                        //noinspection unchecked
                        mListener.onEuclideanCameraPairEstimated((R)this,
                                mPreviousViewId, mCurrentViewId, mCurrentScale,
                                mPreviousEuclideanEstimatedCamera,
                                mCurrentEuclideanEstimatedCamera);
                        //noinspection unchecked
                        mListener.onEuclideanReconstructedPointsEstimated((R)this,
                                mPreviousViewId, mCurrentViewId, mCurrentScale,
                                mEuclideanReconstructedPoints);
                        return true;
                    } else {
                        //pair of cameras estimation failed
                        mFailed = true;
                        //noinspection unchecked
                        mListener.onFail((R)this);
                        return false;
                    }
                } else {
                    //estimation of fundamental matrix failed
                    //noinspection unchecked
                    mListener.onSamplesRejected((R)this,
                            mPreviousViewId, mCurrentViewId,
                            mPreviousViewSamples, mCurrentViewSamples);
                    return false;
                }
            }
        }

        //noinspection unchecked
        mListener.onSamplesRejected((R)this,
                mPreviousViewId, mCurrentViewId,
                mPreviousViewSamples, mCurrentViewSamples);
        return false;
    }

    /**
     * Indicates whether implementations of a reconstructor uses absolute orientation or
     * not.
     * @return true if absolute orientation is used, false, otherwise.
     */
    protected abstract boolean hasAbsoluteOrientation();

    /**
     * Indicates whether there are enough samples to estimate a fundamental
     * matrix.
     * @param samples samples to check.
     * @return true if there are enough samples, false otherwise.
     */
    private boolean hasEnoughSamples(List<Sample2D> samples) {
        return hasEnoughSamplesOrMatches(samples != null ? samples.size() : 0);
    }

    /**
     * Indicates whether there are enough matches to estimate a fundamental
     * matrix.
     * @param matches matches to check.
     * @return true if there are enough matches, false otherwise.
     */
    private boolean hasEnoughMatches(List<MatchedSamples> matches) {
        return hasEnoughSamplesOrMatches(matches != null ? matches.size() : 0);
    }

    /**
     * Indicates whether there are enough matches or samples to estimate a
     * fundamental matrix.
     * @param count number of matches or samples.
     * @return true if there are enough matches or samples, false otherwise.
     */
    private boolean hasEnoughSamplesOrMatches(int count) {
        if (mConfiguration.isGeneralSceneAllowed()) {
            switch (mConfiguration.getNonRobustFundamentalMatrixEstimatorMethod()) {
                case EIGHT_POINTS_ALGORITHM:
                    return count >= EightPointsFundamentalMatrixEstimator.
                            MIN_REQUIRED_POINTS;
                case SEVEN_POINTS_ALGORITHM:
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
     * a general non degenerate 3D configuration.
     * @param matches pairs of matches to find fundamental matrix.
     * @param viewId1 id of first view.
     * @param viewId2 id of second view.
     * @return true if estimation succeeded, false otherwise.
     */
    private boolean estimateFundamentalMatrix(List<MatchedSamples> matches,
                                              int viewId1, int viewId2) {
        if (matches == null) {
            return false;
        }

        int count = matches.size();
        List<Sample2D> leftSamples = new ArrayList<>(count);
        List<Sample2D> rightSamples = new ArrayList<>(count);
        List<Point2D> leftPoints = new ArrayList<>(count);
        List<Point2D> rightPoints = new ArrayList<>(count);
        double[] qualityScores = new double[count];
        double principalPointX, principalPointY;
        if (mConfiguration.getPairedCamerasEstimatorMethod() ==
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC ||
                mConfiguration.getPairedCamerasEstimatorMethod() ==
                        InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX) {
            principalPointX = mConfiguration.getPrincipalPointX();
            principalPointY = mConfiguration.getPrincipalPointY();
        } else {
            principalPointX = principalPointY = 0.0;
        }

        int i = 0;
        for (MatchedSamples match : matches) {
            Sample2D[] samples = match.getSamples();
            if(samples.length != MIN_NUMBER_OF_VIEWS) {
                return false;
            }

            leftSamples.add(samples[0]);
            rightSamples.add(samples[1]);

            Point2D leftPoint = Point2D.create();
            leftPoint.setInhomogeneousCoordinates(
                    samples[0].getPoint().getInhomX() - principalPointX,
                    samples[0].getPoint().getInhomY() - principalPointY);
            leftPoints.add(leftPoint);

            Point2D rightPoint = Point2D.create();
            rightPoint.setInhomogeneousCoordinates(
                    samples[1].getPoint().getInhomX() - principalPointX,
                    samples[1].getPoint().getInhomY() - principalPointY);
            rightPoints.add(rightPoint);

            qualityScores[i] = match.getQualityScore();
            i++;
        }

        try {
            FundamentalMatrixRobustEstimator estimator =
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

            switch(mConfiguration.getRobustFundamentalMatrixEstimatorMethod()) {
                case LMedS:
                    ((LMedSFundamentalMatrixRobustEstimator)estimator).
                            setStopThreshold(mConfiguration.
                                    getFundamentalMatrixThreshold());
                    break;
                case MSAC:
                    ((MSACFundamentalMatrixRobustEstimator)estimator).
                            setThreshold(mConfiguration.
                                    getFundamentalMatrixThreshold());
                    break;
                case PROMedS:
                    ((PROMedSFundamentalMatrixRobustEstimator)estimator).
                            setStopThreshold(mConfiguration.
                                    getFundamentalMatrixThreshold());
                    break;
                case PROSAC:
                    PROSACFundamentalMatrixRobustEstimator prosacEstimator =
                            (PROSACFundamentalMatrixRobustEstimator)estimator;
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
                            (RANSACFundamentalMatrixRobustEstimator)estimator;
                    ransacEstimator.setThreshold(
                            mConfiguration.getFundamentalMatrixThreshold());
                    ransacEstimator.setComputeAndKeepInliersEnabled(
                            mConfiguration.
                                    getFundamentalMatrixComputeAndKeepInliers());
                    ransacEstimator.setComputeAndKeepResidualsEnabled(
                            mConfiguration.
                                    getFundamentalMatrixComputeAndKeepResiduals());
                    break;
            }


            FundamentalMatrix fundamentalMatrix = estimator.estimate();

            mCurrentEstimatedFundamentalMatrix = new EstimatedFundamentalMatrix();
            mCurrentEstimatedFundamentalMatrix.setFundamentalMatrix(fundamentalMatrix);
            mCurrentEstimatedFundamentalMatrix.setViewId1(viewId1);
            mCurrentEstimatedFundamentalMatrix.setViewId2(viewId2);
            mCurrentEstimatedFundamentalMatrix.setCovariance(
                    estimator.getCovariance());

            //determine quality score and inliers
            InliersData inliersData = estimator.getInliersData();
            if (inliersData != null) {
                int numInliers = inliersData.getNumInliers();
                BitSet inliers = inliersData.getInliers();
                int length = inliers.length();
                double fundamentalMatrixQualityScore = 0.0;
                for (i = 0; i < length; i++) {
                    if (inliers.get(i)) {
                        //inlier
                        fundamentalMatrixQualityScore +=
                                qualityScores[i] / numInliers;
                    }
                }
                mCurrentEstimatedFundamentalMatrix.setQualityScore(
                        fundamentalMatrixQualityScore);
                mCurrentEstimatedFundamentalMatrix.setInliers(inliers);
            }

            //store left/right samples
            mCurrentEstimatedFundamentalMatrix.setLeftSamples(leftSamples);
            mCurrentEstimatedFundamentalMatrix.setRightSamples(rightSamples);

            return true;
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Estimates fundamental matrix for provided matches, when 3D points lay in
     * a planar 3D scene.
     * @param matches pairs of matches to find fundamental matrix.
     * @param viewId1 id of first view.
     * @param viewId2 id of second view.
     * @return true if estimation succeeeded, false otherwise.
     */
    private boolean estimatePlanarFundamentalMatrix(
            List<MatchedSamples> matches, int viewId1, int viewId2) {
        if (matches == null) {
            return false;
        }

        int count = matches.size();
        List<Sample2D> leftSamples = new ArrayList<>();
        List<Sample2D> rightSamples = new ArrayList<>();
        List<Point2D> leftPoints = new ArrayList<>();
        List<Point2D> rightPoints = new ArrayList<>();
        double[] qualityScores = new double[count];
        double principalPointX, principalPointY;
        if (mConfiguration.getPairedCamerasEstimatorMethod() ==
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC ||
                mConfiguration.getPairedCamerasEstimatorMethod() ==
                        InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX) {
            principalPointX = mConfiguration.getPrincipalPointX();
            principalPointY = mConfiguration.getPrincipalPointY();
        } else {
            principalPointX = principalPointY = 0.0;
        }

        int i = 0;
        for (MatchedSamples match : matches) {
            Sample2D[] samples = match.getSamples();
            if (samples.length != MIN_NUMBER_OF_VIEWS) {
                return false;
            }

            leftSamples.add(samples[0]);
            rightSamples.add(samples[1]);

            Point2D leftPoint = Point2D.create();
            leftPoint.setInhomogeneousCoordinates(
                    samples[0].getPoint().getInhomX() - principalPointX,
                    samples[0].getPoint().getInhomY() - principalPointY);
            leftPoints.add(leftPoint);

            Point2D rightPoint = Point2D.create();
            rightPoint.setInhomogeneousCoordinates(
                    samples[1].getPoint().getInhomX() - principalPointX,
                    samples[1].getPoint().getInhomY() - principalPointY);
            rightPoints.add(rightPoint);

            qualityScores[i] = match.getQualityScore();
            i++;
        }

        try {
            PointCorrespondenceProjectiveTransformation2DRobustEstimator
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

            switch(mConfiguration.getRobustPlanarHomographyEstimatorMethod()) {
                case LMedS:
                    ((LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                            homographyEstimator).setStopThreshold(
                            mConfiguration.getPlanarHomographyThreshold());
                    break;
                case MSAC:
                    ((MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                            homographyEstimator).setThreshold(
                            mConfiguration.getPlanarHomographyThreshold());
                    break;
                case PROMedS:
                    ((PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator)
                            homographyEstimator).setStopThreshold(
                            mConfiguration.getPlanarHomographyThreshold());
                    break;
                case PROSAC:
                    PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator prosacHomographyEstimator =
                            (PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)homographyEstimator;

                    prosacHomographyEstimator.setThreshold(
                            mConfiguration.getPlanarHomographyThreshold());
                    prosacHomographyEstimator.setComputeAndKeepInliersEnabled(
                            mConfiguration.getPlanarHomographyComputeAndKeepInliers());
                    prosacHomographyEstimator.setComputeAndKeepResidualsEnabled(
                            mConfiguration.getPlanarHomographyComputeAndKeepResiduals());
                    break;
                case RANSAC:
                    RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator ransacHomographyEstimator =
                            (RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator)homographyEstimator;

                    ransacHomographyEstimator.setThreshold(
                            mConfiguration.getPlanarHomographyThreshold());
                    ransacHomographyEstimator.setComputeAndKeepInliersEnabled(
                            mConfiguration.getPlanarHomographyComputeAndKeepInliers());
                    ransacHomographyEstimator.setComputeAndKeepResidualsEnabled(
                            mConfiguration.getPlanarHomographyComputeAndKeepResiduals());
                    break;
            }

            PlanarBestFundamentalMatrixEstimatorAndReconstructor
                    fundamentalMatrixEstimator =
                    new PlanarBestFundamentalMatrixEstimatorAndReconstructor();
            fundamentalMatrixEstimator.setHomographyEstimator(
                    homographyEstimator);
            fundamentalMatrixEstimator.setLeftAndRightPoints(leftPoints,
                    rightPoints);
            fundamentalMatrixEstimator.setQualityScores(qualityScores);

            PinholeCameraIntrinsicParameters intrinsic1 = null;
            PinholeCameraIntrinsicParameters intrinsic2 = null;
            if (mConfiguration.areIntrinsicParametersKnown()) {
                //noinspection unchecked
                intrinsic1 = mListener.onIntrinsicParametersRequested((R)this, viewId1);
                //noinspection unchecked
                intrinsic2 = mListener.onIntrinsicParametersRequested((R)this, viewId2);
            }
            if (intrinsic1 == null && intrinsic2 == null) {
                //estimate homography
                ProjectiveTransformation2D homography = homographyEstimator.
                        estimate();

                //estimate intrinsic parameters using the Image of Absolute
                //Conic (IAC)
                List<Transformation2D> homographies = new ArrayList<>();
                homographies.add(homography);

                ImageOfAbsoluteConicEstimator iacEstimator =
                        new LMSEImageOfAbsoluteConicEstimator(homographies);
                ImageOfAbsoluteConic iac = iacEstimator.estimate();

                intrinsic1 = intrinsic2 = iac.getIntrinsicParameters();

            } else if (intrinsic1 == null) { //&& intrinsic2 != null
                intrinsic1 = intrinsic2;
            } else if (intrinsic2 == null) { //&& intrinsic1 != null
                intrinsic2 = intrinsic1;
            }
            fundamentalMatrixEstimator.setLeftIntrinsics(intrinsic1);
            fundamentalMatrixEstimator.setRightIntrinsics(intrinsic2);

            fundamentalMatrixEstimator.estimateAndReconstruct();

            FundamentalMatrix fundamentalMatrix =
                    fundamentalMatrixEstimator.getFundamentalMatrix();

            mCurrentEstimatedFundamentalMatrix = new EstimatedFundamentalMatrix();
            mCurrentEstimatedFundamentalMatrix.setFundamentalMatrix(fundamentalMatrix);
            mCurrentEstimatedFundamentalMatrix.setViewId1(viewId1);
            mCurrentEstimatedFundamentalMatrix.setViewId2(viewId2);

            //determine quality score and inliers
            InliersData inliersData = homographyEstimator.getInliersData();
            if (inliersData != null) {
                int numInliers = inliersData.getNumInliers();
                BitSet inliers = inliersData.getInliers();
                int length = inliers.length();
                double fundamentalMatrixQualityScore = 0.0;
                for (i = 0; i < length; i++) {
                    if (inliers.get(i)) {
                        //inlier
                        fundamentalMatrixQualityScore +=
                                qualityScores[i] / numInliers;
                    }
                }
                mCurrentEstimatedFundamentalMatrix.setQualityScore(
                        fundamentalMatrixQualityScore);
                mCurrentEstimatedFundamentalMatrix.setInliers(inliers);
            }

            //store left/right samples
            mCurrentEstimatedFundamentalMatrix.setLeftSamples(leftSamples);
            mCurrentEstimatedFundamentalMatrix.setRightSamples(rightSamples);

            return true;
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Estimates pair of cameras and reconstructed points.
     * @param isInitialPairOfViews true if initial pair of views is being processed,
     *                             false otherwise.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimatePairOfCamerasAndPoints(boolean isInitialPairOfViews) {
        switch(mConfiguration.getPairedCamerasEstimatorMethod()) {
            case ESSENTIAL_MATRIX:
                return estimateInitialCamerasAndPointsEssential(isInitialPairOfViews);
            case DUAL_IMAGE_OF_ABSOLUTE_CONIC:
                return estimateInitialCamerasAndPointsDIAC(isInitialPairOfViews);
            case DUAL_ABSOLUTE_QUADRIC:
                return estimateInitialCamerasAndPointsDAQ(isInitialPairOfViews);
            case DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX:
            default:
                return estimateInitialCamerasAndPointsDAQAndEssential(isInitialPairOfViews);
        }
    }

    /**
     * Estimates initial cameras and reconstructed points using the Dual
     * Absolute Quadric to estimate intrinsic parameters and then use those
     * intrinsic parameters with the essential matrix.
     * @param isInitialPairOfViews true if initial pair of views is being processed,
     *                             false otherwise.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsDAQAndEssential(
            boolean isInitialPairOfViews) {
        //for non initial view, keep last center and rotation
        if (!isInitialPairOfViews && keepLastCenterAndRotation()) {
            return false;
        }

        try {
            FundamentalMatrix fundamentalMatrix =
                    mCurrentEstimatedFundamentalMatrix.getFundamentalMatrix();

            DualAbsoluteQuadricInitialCamerasEstimator estimator =
                    new DualAbsoluteQuadricInitialCamerasEstimator(
                            fundamentalMatrix);
            estimator.setAspectRatio(
                    mConfiguration.getPairedCamerasAspectRatio());
            estimator.estimate();

            PinholeCamera camera1 = estimator.getEstimatedLeftCamera();
            PinholeCamera camera2 = estimator.getEstimatedRightCamera();

            camera1.decompose();
            camera2.decompose();

            PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint1 =
                    camera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint2 =
                    camera2.getIntrinsicParameters();

            double principalPointX = mConfiguration.getPrincipalPointX();
            double principalPointY = mConfiguration.getPrincipalPointY();

            PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(
                            intrinsicZeroPrincipalPoint1);
            intrinsic1.setHorizontalPrincipalPoint(
                    intrinsic1.getHorizontalPrincipalPoint() + principalPointX);
            intrinsic1.setVerticalPrincipalPoint(
                    intrinsic1.getVerticalPrincipalPoint() + principalPointY);

            PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(
                            intrinsicZeroPrincipalPoint2);
            intrinsic2.setHorizontalPrincipalPoint(
                    intrinsic2.getHorizontalPrincipalPoint() + principalPointX);
            intrinsic2.setVerticalPrincipalPoint(
                    intrinsic2.getVerticalPrincipalPoint() + principalPointY);

            //fix fundamental matrix to account for principal point different
            //from zero
            fixFundamentalMatrix(fundamentalMatrix,
                    intrinsicZeroPrincipalPoint1, intrinsicZeroPrincipalPoint2,
                    intrinsic1, intrinsic2);

            return estimateInitialCamerasAndPointsEssential(intrinsic1, intrinsic2) &&
                    transformPairOfCamerasAndPoints(isInitialPairOfViews,
                    hasAbsoluteOrientation());
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Estimates initial cameras and reconstructed points using the Dual
     * Absolute Quadric.
     * @param isInitialPairOfViews true if initial pair of views is being processed,
     *                             false otherwise.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsDAQ(boolean isInitialPairOfViews) {
        //for non initial view, keep last center and rotation
        if (!isInitialPairOfViews && keepLastCenterAndRotation()) {
            return false;
        }

        try {
            FundamentalMatrix fundamentalMatrix =
                    mCurrentEstimatedFundamentalMatrix.getFundamentalMatrix();
            fundamentalMatrix.normalize();

            DualAbsoluteQuadricInitialCamerasEstimator estimator =
                    new DualAbsoluteQuadricInitialCamerasEstimator(
                            fundamentalMatrix);
            estimator.setAspectRatio(
                    mConfiguration.getPairedCamerasAspectRatio());
            estimator.estimate();

            PinholeCamera camera1 = estimator.getEstimatedLeftCamera();
            PinholeCamera camera2 = estimator.getEstimatedRightCamera();

            camera1.decompose();
            camera2.decompose();

            PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint1 =
                    camera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint2 =
                    camera2.getIntrinsicParameters();

            double principalPointX = mConfiguration.getPrincipalPointX();
            double principalPointY = mConfiguration.getPrincipalPointY();

            PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(
                            intrinsicZeroPrincipalPoint1);
            intrinsic1.setHorizontalPrincipalPoint(
                    intrinsic1.getHorizontalPrincipalPoint() + principalPointX);
            intrinsic1.setVerticalPrincipalPoint(
                    intrinsic1.getVerticalPrincipalPoint() + principalPointY);
            camera1.setIntrinsicParameters(intrinsic1);

            PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(
                            intrinsicZeroPrincipalPoint2);
            intrinsic2.setHorizontalPrincipalPoint(
                    intrinsic2.getHorizontalPrincipalPoint() + principalPointX);
            intrinsic2.setVerticalPrincipalPoint(
                    intrinsic2.getVerticalPrincipalPoint() + principalPointY);
            camera2.setIntrinsicParameters(intrinsic2);

            mPreviousMetricEstimatedCamera = new EstimatedCamera();
            mPreviousMetricEstimatedCamera.setCamera(camera1);

            mCurrentMetricEstimatedCamera = new EstimatedCamera();
            mCurrentMetricEstimatedCamera.setCamera(camera2);

            //fix fundamental matrix to account for principal point different
            //from zero
            fixFundamentalMatrix(fundamentalMatrix,
                    intrinsicZeroPrincipalPoint1, intrinsicZeroPrincipalPoint2,
                    intrinsic1, intrinsic2);

            //triangulate points
            Corrector corrector = null;
            if (mConfiguration.getPairedCamerasCorrectorType() != null) {
                corrector = Corrector.create(fundamentalMatrix,
                        mConfiguration.getPairedCamerasCorrectorType());
            }

            //use all points used for fundamental matrix estimation
            List<Sample2D> samples1 = mCurrentEstimatedFundamentalMatrix.getLeftSamples();
            List<Sample2D> samples2 = mCurrentEstimatedFundamentalMatrix.getRightSamples();

            List<Point2D> points1 = new ArrayList<>();
            List<Point2D> points2 = new ArrayList<>();
            int length = samples1.size();
            for (int i = 0; i < length; i++) {
                Sample2D sample1 = samples1.get(i);
                Sample2D sample2 = samples2.get(i);

                Point2D point1 = sample1.getPoint();
                Point2D point2 = sample2.getPoint();

                points1.add(point1);
                points2.add(point2);
            }

            //correct points if needed
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


            //triangulate points
            SinglePoint3DTriangulator triangulator;
            if (mConfiguration.getDaqUseHomogeneousPointTriangulator()) {
                triangulator = SinglePoint3DTriangulator.create(
                        Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR);
            } else {
                triangulator = SinglePoint3DTriangulator.create(
                        Point3DTriangulatorType.
                                LMSE_INHOMOGENEOUS_TRIANGULATOR);
            }

            List<PinholeCamera> cameras = new ArrayList<>();
            cameras.add(camera1);
            cameras.add(camera2);

            mMetricReconstructedPoints = new ArrayList<>();
            List<Point2D> points = new ArrayList<>();
            int numPoints = correctedPoints1.size();
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

                //only points reconstructed in front of both cameras are
                //considered valid
                boolean front1 = camera1.isPointInFrontOfCamera(
                        triangulatedPoint);
                boolean front2 = camera2.isPointInFrontOfCamera(
                        triangulatedPoint);
                reconstructedPoint.setInlier(front1 && front2);

                mMetricReconstructedPoints.add(reconstructedPoint);
            }

            return transformPairOfCamerasAndPoints(isInitialPairOfViews,
                    hasAbsoluteOrientation());
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Estimates initial cameras and reconstructed points using Dual Image of
     * Absolute Conic.
     * @param isInitialPairOfViews true if initial pair of views is being processed,
     *                             false otherwise.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsDIAC(boolean isInitialPairOfViews) {
        //for non initial view, keep last center and rotation
        if (!isInitialPairOfViews && keepLastCenterAndRotation()) {
            return false;
        }

        FundamentalMatrix fundamentalMatrix =
                mCurrentEstimatedFundamentalMatrix.getFundamentalMatrix();

        //use inlier points used for fundamental matrix estimation
        List<Sample2D> samples1 = mCurrentEstimatedFundamentalMatrix.getLeftSamples();
        List<Sample2D> samples2 = mCurrentEstimatedFundamentalMatrix.getRightSamples();

        List<Point2D> points1 = new ArrayList<>();
        List<Point2D> points2 = new ArrayList<>();
        int length = samples1.size();
        for (int i = 0; i < length; i++) {
            Sample2D sample1 = samples1.get(i);
            Sample2D sample2 = samples2.get(i);

            Point2D point1 = sample1.getPoint();
            Point2D point2 = sample2.getPoint();

            points1.add(point1);
            points2.add(point2);
        }

        try {
            DualImageOfAbsoluteConicInitialCamerasEstimator estimator =
                    new DualImageOfAbsoluteConicInitialCamerasEstimator(
                            fundamentalMatrix, points1, points2);
            estimator.setPrincipalPoint(mConfiguration.getPrincipalPointX(),
                    mConfiguration.getPrincipalPointY());
            estimator.setAspectRatio(
                    mConfiguration.getPairedCamerasAspectRatio());
            estimator.setCorrectorType(
                    mConfiguration.getPairedCamerasCorrectorType());
            estimator.setPointsTriangulated(true);
            estimator.setValidTriangulatedPointsMarked(
                    mConfiguration.getPairedCamerasMarkValidTriangulatedPoints());

            estimator.estimate();

            //store cameras
            PinholeCamera camera1 = estimator.getEstimatedLeftCamera();
            PinholeCamera camera2 = estimator.getEstimatedRightCamera();

            mPreviousMetricEstimatedCamera = new EstimatedCamera();
            mPreviousMetricEstimatedCamera.setCamera(camera1);

            mCurrentMetricEstimatedCamera = new EstimatedCamera();
            mCurrentMetricEstimatedCamera.setCamera(camera2);

            //store points
            List<Point3D> triangulatedPoints =
                    estimator.getTriangulatedPoints();
            BitSet validTriangulatedPoints =
                    estimator.getValidTriangulatedPoints();

            mMetricReconstructedPoints = new ArrayList<>();
            int size = triangulatedPoints.size();
            for (int i = 0; i < size; i++) {
                ReconstructedPoint3D reconstructedPoint =
                        new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoints.get(i));
                reconstructedPoint.setInlier(validTriangulatedPoints.get(i));
                mMetricReconstructedPoints.add(reconstructedPoint);
            }

            return transformPairOfCamerasAndPoints(isInitialPairOfViews,
                    hasAbsoluteOrientation());
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Estimates initial cameras and reconstructed points using the essential
     * matrix and provided intrinsic parameters that must have been set during
     * offline calibration.
     * @param isInitialPairOfViews true if initial pair of views is being processed,
     *                             false otherwise.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsEssential(boolean isInitialPairOfViews) {
        //for non initial view, keep last center and rotation
        if (!isInitialPairOfViews && keepLastCenterAndRotation()) {
            return false;
        }

        PinholeCameraIntrinsicParameters intrinsic1 = null;
        PinholeCameraIntrinsicParameters intrinsic2 = null;
        if (mConfiguration.areIntrinsicParametersKnown()) {
            //noinspection unchecked
            intrinsic1 = mListener.onIntrinsicParametersRequested((R)this, mPreviousViewId);
            //noinspection unchecked
            intrinsic2 = mListener.onIntrinsicParametersRequested((R)this, mCurrentViewId);
        }

        if (intrinsic1 != null && intrinsic2 != null) {
            return estimateInitialCamerasAndPointsEssential(intrinsic1, intrinsic2) &&
                    transformPairOfCamerasAndPoints(isInitialPairOfViews,
                    hasAbsoluteOrientation());
        } else {
            //missing intrinsic parameters

            mFailed = true;
            //noinspection unchecked
            mListener.onFail((R)this);
            return false;
        }
    }

    /**
     * Estimates initial cameras and reconstructed points using the essential
     * matrix and provided intrinsic parameters that must have been set during
     * offline calibration.
     * @param intrinsic1 intrinsic parameters of 1st camera.
     * @param intrinsic2 intrinsic parameters of 2nd camera.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsEssential(
            PinholeCameraIntrinsicParameters intrinsic1,
            PinholeCameraIntrinsicParameters intrinsic2) {
        FundamentalMatrix fundamentalMatrix =
                mCurrentEstimatedFundamentalMatrix.getFundamentalMatrix();

        //use all points used for fundamental matrix estimation
        List<Sample2D> samples1 = mCurrentEstimatedFundamentalMatrix.getLeftSamples();
        List<Sample2D> samples2 = mCurrentEstimatedFundamentalMatrix.getRightSamples();

        List<Point2D> points1 = new ArrayList<>();
        List<Point2D> points2 = new ArrayList<>();
        int length = samples1.size();
        for (int i = 0; i < length; i++) {
            Sample2D sample1 = samples1.get(i);
            Sample2D sample2 = samples2.get(i);

            Point2D point1 = sample1.getPoint();
            Point2D point2 = sample2.getPoint();

            points1.add(point1);
            points2.add(point2);
        }

        try {
            EssentialMatrixInitialCamerasEstimator estimator =
                    new EssentialMatrixInitialCamerasEstimator(
                            fundamentalMatrix, intrinsic1, intrinsic2,
                            points1, points2);

            estimator.setCorrectorType(
                    mConfiguration.getPairedCamerasCorrectorType());
            estimator.setPointsTriangulated(true);
            estimator.setValidTriangulatedPointsMarked(
                    mConfiguration.getPairedCamerasMarkValidTriangulatedPoints());

            estimator.estimate();

            //store cameras
            PinholeCamera camera1 = estimator.getEstimatedLeftCamera();
            PinholeCamera camera2 = estimator.getEstimatedRightCamera();

            mPreviousMetricEstimatedCamera = new EstimatedCamera();
            mPreviousMetricEstimatedCamera.setCamera(camera1);

            mCurrentMetricEstimatedCamera = new EstimatedCamera();
            mCurrentMetricEstimatedCamera.setCamera(camera2);

            //store points
            List<Point3D> triangulatedPoints =
                    estimator.getTriangulatedPoints();
            BitSet validTriangulatedPoints =
                    estimator.getValidTriangulatedPoints();

            mMetricReconstructedPoints = new ArrayList<>();
            int size = triangulatedPoints.size();
            for (int i = 0; i < size; i++) {
                ReconstructedPoint3D reconstructedPoint =
                        new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoints.get(i));
                reconstructedPoint.setInlier(validTriangulatedPoints.get(i));
                mMetricReconstructedPoints.add(reconstructedPoint);
            }

            return true;
        } catch (Exception e) {
            return false;
        }
    }


    /**
     * Keeps center and rotation of last camera (current camera on previous view pair).
     * @return false if camera and rotation were successfully kept, true otherwise.
     */
    private boolean keepLastCenterAndRotation() {
        //keep last metric center and rotation
        if (mCurrentMetricEstimatedCamera == null || mCurrentEuclideanEstimatedCamera == null) {
            return true;
        }

        PinholeCamera metricCamera = mCurrentMetricEstimatedCamera.getCamera();
        if (metricCamera == null) {
            return true;
        }

        try {
            //decompose camera if needed
            if (!metricCamera.isCameraCenterAvailable() || !metricCamera.isCameraRotationAvailable()) {
                metricCamera.decompose();
            }

            mLastMetricCameraCenter = metricCamera.getCameraCenter();
            mLastMetricCameraRotation = metricCamera.getCameraRotation();

        } catch (GeometryException e) {
            return true;
        }

        //keep last euclidean center and rotation
        PinholeCamera euclideanCamera = mCurrentEuclideanEstimatedCamera.getCamera();
        if (euclideanCamera == null) {
            return true;
        }

        try {
            //decompose camera if needed
            if (!euclideanCamera.isCameraCenterAvailable() || !euclideanCamera.isCameraRotationAvailable()) {
                euclideanCamera.decompose();
            }

            mLastEuclideanCameraCenter = euclideanCamera.getCameraCenter();
            mLastEuclideanCameraRotation = euclideanCamera.getCameraRotation();

            return false;
        } catch (GeometryException e) {
            return true;
        }
    }

    /**
     * Fixes fundamental matrix to account for principal point different from
     * zero when using DAQ estimation.
     * @param fundamentalMatrix fundamental matrix to be fixed.
     * @param intrinsicZeroPrincipalPoint1 intrinsic parameters of camera 1
     * assuming zero principal point.
     * @param intrinsicZeroPrincipalPoint2 intrinsic parameters of camera 2
     * assuming zero principal point.
     * @param intrinsicPrincipalPoint1 intrinsic parameters of camera 1 using
     * proper principal point.
     * @param intrinsicPrincipalPoint2 intrinsic parameters of camera 2 using
     * proper principal point.
     * @throws EpipolarException if something fails.
     * @throws NotReadyException never happens.
     */
    private void fixFundamentalMatrix(FundamentalMatrix fundamentalMatrix,
                                      PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint1,
                                      PinholeCameraIntrinsicParameters intrinsicZeroPrincipalPoint2,
                                      PinholeCameraIntrinsicParameters intrinsicPrincipalPoint1,
                                      PinholeCameraIntrinsicParameters intrinsicPrincipalPoint2)
            throws EpipolarException, NotReadyException {

        //first compute essential matrix as E = K2a'F*K1a
        EssentialMatrix essential = new EssentialMatrix(fundamentalMatrix,
                intrinsicZeroPrincipalPoint1, intrinsicZeroPrincipalPoint2);
        FundamentalMatrix fixedFundamentalMatrix =
                essential.toFundamentalMatrix(intrinsicPrincipalPoint1,
                        intrinsicPrincipalPoint2);
        fixedFundamentalMatrix.normalize();
        mCurrentEstimatedFundamentalMatrix.setFundamentalMatrix(
                fixedFundamentalMatrix);
        mCurrentEstimatedFundamentalMatrix.setCovariance(null);
    }
}
