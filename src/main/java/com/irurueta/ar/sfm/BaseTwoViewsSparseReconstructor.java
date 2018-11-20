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
import com.irurueta.ar.epipolar.estimators.*;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.*;
import com.irurueta.numerical.robust.InliersData;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * Base class in charge of estimating cameras and 3D reconstructed points from 
 * sparse image point correspondences in two views.
 * @param <C> type of configuration.
 * @param <R> type of reconstructor.
 * @param <L> type of listener.
 */
@SuppressWarnings("WeakerAccess")
public abstract class BaseTwoViewsSparseReconstructor<
        C extends BaseTwoViewsSparseReconstructorConfiguration,
        R extends BaseTwoViewsSparseReconstructor,
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
    private List<MatchedSamples> mMatches = new ArrayList<>();

    /**
     * Id of first view.
     */
    private int mFirstViewId = 0;

    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not 
     * provided.
     */
    public BaseTwoViewsSparseReconstructor(C configuration,
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
    public BaseTwoViewsSparseReconstructorListener<R> getListener() {
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
     * Gets estimated fundamental matrix.
     * @return estimated fundamental matrix.
     */
    public EstimatedFundamentalMatrix getEstimatedFundamentalMatrix() {
        return mEstimatedFundamentalMatrix;
    }
    
    /**
     * Gets estimated first camera.
     * @return estimated first camera.
     */
    public EstimatedCamera getEstimatedCamera1() {
        return mEstimatedCamera1;
    }

    /**
     * Gets estimated second camera.
     * @return estimated second camera.
     */
    public EstimatedCamera getEstimatedCamera2() {
        return mEstimatedCamera2;
    }
    
    /**
     * Gets reconstructed 3D points.
     * @return reconstructed 3D points.
     */
    public List<ReconstructedPoint3D> getReconstructedPoints() {
        return mReconstructedPoints;
    }

    /**
     * Process one view of all the available data during the reconstruction.
     * This method can be called multiple times instead of {@link #start()} to build the reconstruction step by step,
     * one view at a time.
     * @return true if more views can be processed, false when reconstruction has finished.
     */
    public boolean processOneView() {
        if (mViewCount == 0) {
            if (mRunning) {
                //already started
                return true;
            }

            reset();
            mRunning = true;

            //noinspection all
            mListener.onStart((R)this);
        }

        //noinspection all
        if(!mListener.hasMoreViewsAvailable((R)this)) {
            return false;
        }

        mEstimatedFundamentalMatrix = null;
        mCurrentViewSamples = new ArrayList<>();
        //noinspection all
        mListener.onRequestSamplesForCurrentView((R)this, mViewCount,
                mCurrentViewSamples);

        if (mFirstViewSamples == null) {
            //for first view we simply keep samples (if enough are provided)
            if(hasEnoughSamples(mCurrentViewSamples)) {
                //noinspection all
                mListener.onSamplesAccepted((R)this, mViewCount,
                        mCurrentViewSamples);
                mFirstViewSamples = mCurrentViewSamples;
                mFirstViewId = mViewCount;
            }

        } else {

            //for second view, check that we have enough samples
            if (hasEnoughSamples(mCurrentViewSamples)) {

                //find matches
                mMatches.clear();
                //noinspection all
                mListener.onRequestMatches((R)this, mFirstViewSamples,
                        mCurrentViewSamples, mFirstViewId, mViewCount,
                        mMatches);

                if (hasEnoughMatches(mMatches)) {
                    //if enough matches are retrieved, attempt to compute
                    //fundamental matrix
                    if ((mConfiguration.isGeneralSceneAllowed() &&
                            estimateFundamentalMatrix(mMatches, mFirstViewId,
                                    mViewCount)) ||
                            (mConfiguration.isPlanarSceneAllowed() &&
                                    estimatePlanarFundamentalMatrix(mMatches,
                                            mFirstViewId, mViewCount))) {
                        //fundamental matrix could be estimated
                        //noinspection all
                        mListener.onSamplesAccepted((R)this, mViewCount,
                                mCurrentViewSamples);
                        int secondViewId = mViewCount;

                        //noinspection all
                        mListener.onFundamentalMatrixEstimated((R)this,
                                mEstimatedFundamentalMatrix);

                        if(estimateInitialCamerasAndPoints()) {
                            //cameras and points have been estimated
                            //noinspection all
                            mListener.onCamerasEstimated((R)this,
                                    mFirstViewId, secondViewId,
                                    mEstimatedCamera1, mEstimatedCamera2);
                            //noinspection all
                            mListener.onReconstructedPointsEstimated(
                                    (R)this, mMatches, mReconstructedPoints);
                            if(postProcessOne()) {
                                //noinspection all
                                mListener.onFinish((R) this);
                                mRunning = false;
                                mFinished = true;
                            }
                        } else {
                            //initial cameras failed
                            mFailed = true;
                            //noinspection all
                            mListener.onFail((R)this);
                        }
                    } else {
                        //estimation of fundamental matrix failed
                        //noinspection all
                        mListener.onSamplesRejected((R)this, mViewCount,
                                mCurrentViewSamples);
                    }
                }
            }
        }

        mViewCount++;

        if (mCancelled) {
            //noinspection all
            mListener.onCancel((R)this);
        }

        return !mFinished;
    }
    
    /**
     * Starts reconstruction of all available data to reconstruct the whole scene.
     * If reconstruction has already started and is running, calling this method
     * has no effect.
     */
    public void start() {
        while(processOneView()) {
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
            //already cancelled
            return;
        }
        
        mCancelled = true;
    }
    
    /**
     * Resets this instance so that a reconstruction can be started from the beginning without cancelling current one.
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
     * Called when processing one frame is successfully finished. This can be done to estimate scale on those
     * implementations where scale can be measured or is already known.
     * @return true if post processing succeeded, false otherwise.
     */
    protected abstract boolean postProcessOne();

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
        for (MatchedSamples match : matches) {
            Sample2D[] samples = match.getSamples();
            if(samples.length != NUMBER_OF_VIEWS) {
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
            
            mEstimatedFundamentalMatrix = new EstimatedFundamentalMatrix();
            mEstimatedFundamentalMatrix.setFundamentalMatrix(fundamentalMatrix);
            mEstimatedFundamentalMatrix.setViewId1(viewId1);
            mEstimatedFundamentalMatrix.setViewId2(viewId2);
            mEstimatedFundamentalMatrix.setCovariance(
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
                mEstimatedFundamentalMatrix.setQualityScore(
                        fundamentalMatrixQualityScore);
                mEstimatedFundamentalMatrix.setInliers(inliers);                
            }
            
            //store left/right samples
            mEstimatedFundamentalMatrix.setLeftSamples(leftSamples);
            mEstimatedFundamentalMatrix.setRightSamples(rightSamples);
            
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
    @SuppressWarnings("all")
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
        for (MatchedSamples match : matches) {
            Sample2D[] samples = match.getSamples();
            if (samples.length != NUMBER_OF_VIEWS) {
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
            
            PinholeCameraIntrinsicParameters intrinsic1 = 
                    mConfiguration.getInitialIntrinsic1();
            PinholeCameraIntrinsicParameters intrinsic2 =
                    mConfiguration.getInitialIntrinsic1();
            if (intrinsic1 == null && intrinsic2 == null) {
                //estimate homography
                ProjectiveTransformation2D homography = homographyEstimator.
                        estimate();
                
                //estimate intrinsic parameters using the Image of Absolute 
                //Conic (IAC)
                List<Transformation2D> homographies = 
                        new ArrayList<>();
                homographies.add(homography);
                
                ImageOfAbsoluteConicEstimator iacEstimator = 
                        new LMSEImageOfAbsoluteConicEstimator(homographies);
                ImageOfAbsoluteConic iac = iacEstimator.estimate();
                
                intrinsic1 = intrinsic2 = iac.getIntrinsicParameters();

            } else if (intrinsic1 == null && intrinsic2 != null) {
                intrinsic1 = intrinsic2;
            } else if (intrinsic1 != null && intrinsic2 == null) {
                intrinsic2 = intrinsic1;
            }
            fundamentalMatrixEstimator.setLeftIntrinsics(intrinsic1);
            fundamentalMatrixEstimator.setRightIntrinsics(intrinsic2);
            
            fundamentalMatrixEstimator.estimateAndReconstruct();
            
            FundamentalMatrix fundamentalMatrix = 
                    fundamentalMatrixEstimator.getFundamentalMatrix();
            
            mEstimatedFundamentalMatrix = new EstimatedFundamentalMatrix();
            mEstimatedFundamentalMatrix.setFundamentalMatrix(fundamentalMatrix);
            mEstimatedFundamentalMatrix.setViewId1(viewId1);
            mEstimatedFundamentalMatrix.setViewId2(viewId2);
            
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
                mEstimatedFundamentalMatrix.setQualityScore(
                        fundamentalMatrixQualityScore);
                mEstimatedFundamentalMatrix.setInliers(inliers);                
            }
                        
            //store left/right samples
            mEstimatedFundamentalMatrix.setLeftSamples(leftSamples);
            mEstimatedFundamentalMatrix.setRightSamples(rightSamples);
            
            return true;
        } catch (Exception e) {
            return false;
        }
    }
    
    /**
     * Estimates initial cameras and reconstructed points.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPoints() {
        switch(mConfiguration.getInitialCamerasEstimatorMethod()) {
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
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsDAQAndEssential() {
        try {
            FundamentalMatrix fundamentalMatrix =
                    mEstimatedFundamentalMatrix.getFundamentalMatrix();
            
            DualAbsoluteQuadricInitialCamerasEstimator estimator =
                    new DualAbsoluteQuadricInitialCamerasEstimator(
                            fundamentalMatrix);
            estimator.setAspectRatio(
                    mConfiguration.getInitialCamerasAspectRatio());
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
            
            return estimateInitialCamerasAndPointsEssential(intrinsic1,
                    intrinsic2);
        } catch (Exception e) {
            return false;
        }
    }
        
    /**
     * Estimates initial cameras and reconstructed points using the Dual 
     * Absolute Quadric.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsDAQ() {
        try {
            FundamentalMatrix fundamentalMatrix = 
                    mEstimatedFundamentalMatrix.getFundamentalMatrix();
            fundamentalMatrix.normalize();            
        
            DualAbsoluteQuadricInitialCamerasEstimator estimator =
                    new DualAbsoluteQuadricInitialCamerasEstimator(
                            fundamentalMatrix);
            estimator.setAspectRatio(
                    mConfiguration.getInitialCamerasAspectRatio());
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
            
            mEstimatedCamera1 = new EstimatedCamera();
            mEstimatedCamera1.setCamera(camera1);
        
            mEstimatedCamera2 = new EstimatedCamera();
            mEstimatedCamera2.setCamera(camera2);
            
            //fix fundamental matrix to account for principal point different 
            //from zero
            fixFundamentalMatrix(fundamentalMatrix, 
                    intrinsicZeroPrincipalPoint1, intrinsicZeroPrincipalPoint2,
                    intrinsic1, intrinsic2);
            
            //triangulate points
            Corrector corrector = null;
            if (mConfiguration.getInitialCamerasCorrectorType() != null) {
                corrector = Corrector.create(fundamentalMatrix,
                        mConfiguration.getInitialCamerasCorrectorType());
            }

            //use all points used for fundamental matrix estimation
            List<Sample2D> samples1 = mEstimatedFundamentalMatrix.getLeftSamples();
            List<Sample2D> samples2 = mEstimatedFundamentalMatrix.getRightSamples();
        
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
            
            mReconstructedPoints = new ArrayList<>();
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

                mReconstructedPoints.add(reconstructedPoint);                
            }
            
            return true;
        } catch (Exception e) {
            return false;
        }
    }
    
    /**
     * Estimates initial cameras and reconstructed points using Dual Image of 
     * Absolute Conic.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */
    private boolean estimateInitialCamerasAndPointsDIAC() {
        FundamentalMatrix fundamentalMatrix = 
                mEstimatedFundamentalMatrix.getFundamentalMatrix();
                
        //use inlier points used for fundamental matrix estimation
        List<Sample2D> samples1 = mEstimatedFundamentalMatrix.getLeftSamples();
        List<Sample2D> samples2 = mEstimatedFundamentalMatrix.getRightSamples();
        
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
                    mConfiguration.getInitialCamerasAspectRatio());
            estimator.setCorrectorType(
                    mConfiguration.getInitialCamerasCorrectorType());
            estimator.setPointsTriangulated(true);
            estimator.setValidTriangulatedPointsMarked(
                    mConfiguration.getInitialCamerasMarkValidTriangulatedPoints());
        
            estimator.estimate();
        
            //store cameras
            PinholeCamera camera1 = estimator.getEstimatedLeftCamera();
            PinholeCamera camera2 = estimator.getEstimatedRightCamera();
            
            mEstimatedCamera1 = new EstimatedCamera();
            mEstimatedCamera1.setCamera(camera1);
        
            mEstimatedCamera2 = new EstimatedCamera();
            mEstimatedCamera2.setCamera(camera2);
            
            //store points
            List<Point3D> triangulatedPoints = 
                    estimator.getTriangulatedPoints();
            BitSet validTriangulatedPoints =
                    estimator.getValidTriangulatedPoints();
            
            mReconstructedPoints = new ArrayList<>();
            int size = triangulatedPoints.size();
            for (int i = 0; i < size; i++) {
                ReconstructedPoint3D reconstructedPoint =
                        new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoints.get(i));
                reconstructedPoint.setInlier(validTriangulatedPoints.get(i));
                mReconstructedPoints.add(reconstructedPoint);                
            }
            
            return true;        
        } catch (Exception e) {
            return false;          
        }
    }
    
    /**
     * Estimates initial cameras and reconstructed points using the essential
     * matrix and provided intrinsic parameters that must have been set during 
     * offline calibration.
     * @return true if cameras and points could be estimated, false if something
     * failed.
     */    
    private boolean estimateInitialCamerasAndPointsEssential() {
        PinholeCameraIntrinsicParameters intrinsic1 = 
                mConfiguration.getInitialIntrinsic1();
        PinholeCameraIntrinsicParameters intrinsic2 =
                mConfiguration.getInitialIntrinsic2();
        return estimateInitialCamerasAndPointsEssential(intrinsic1, intrinsic2);
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
                mEstimatedFundamentalMatrix.getFundamentalMatrix();

        //use all points used for fundamental matrix estimation
        List<Sample2D> samples1 = mEstimatedFundamentalMatrix.getLeftSamples();
        List<Sample2D> samples2 = mEstimatedFundamentalMatrix.getRightSamples();
        
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
                    mConfiguration.getInitialCamerasCorrectorType());
            estimator.setPointsTriangulated(true);
            estimator.setValidTriangulatedPointsMarked(
                    mConfiguration.getInitialCamerasMarkValidTriangulatedPoints());
        
            estimator.estimate();
        
            //store cameras
            PinholeCamera camera1 = estimator.getEstimatedLeftCamera();
            PinholeCamera camera2 = estimator.getEstimatedRightCamera();
            
            mEstimatedCamera1 = new EstimatedCamera();
            mEstimatedCamera1.setCamera(camera1);
        
            mEstimatedCamera2 = new EstimatedCamera();
            mEstimatedCamera2.setCamera(camera2);
            
            //store points
            List<Point3D> triangulatedPoints = 
                    estimator.getTriangulatedPoints();
            BitSet validTriangulatedPoints =
                    estimator.getValidTriangulatedPoints();
            
            mReconstructedPoints = new ArrayList<>();
            int size = triangulatedPoints.size();
            for (int i = 0; i < size; i++) {
                ReconstructedPoint3D reconstructedPoint =
                        new ReconstructedPoint3D();
                reconstructedPoint.setPoint(triangulatedPoints.get(i));
                reconstructedPoint.setInlier(validTriangulatedPoints.get(i));
                mReconstructedPoints.add(reconstructedPoint);
            }
            
            return true;        
        } catch (Exception e) {
            return false;          
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
        mEstimatedFundamentalMatrix.setFundamentalMatrix(
                fixedFundamentalMatrix);        
        mEstimatedFundamentalMatrix.setCovariance(null);
    }        
}
