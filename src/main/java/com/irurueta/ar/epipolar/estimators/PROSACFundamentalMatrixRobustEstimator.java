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
package com.irurueta.ar.epipolar.estimators;

import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best fundamental matrix for provided collections of matched 2D
 * points using PROSAC algorithm.
 */
public class PROSACFundamentalMatrixRobustEstimator extends
        FundamentalMatrixRobustEstimator {
    
    /**
     * Constant defining default threshold to determine whether points are
     * inliers or not.
     * By default 1.0 is considered a good value for cases where measures are
     * done in pixels, since typically the minimum resolution is 1 pixel.
     */
    public static final double DEFAULT_THRESHOLD = 1.0;
    
    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Indicates that by default inliers will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_INLIERS = false;
    
    /**
     * Indicates that by default residuals will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_RESIDUALS = false;        
    
    /**
     * Threshold to determine whether pairs of matched points are inliers or not
     * when testing possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance) a given
     * point has respect to the epipolar line generated by its matched point.
     */
    private double mThreshold;
    
    /**
     * Quality scores corresponding to each provided point.
     * The larger the score value the better the quality of the sample.
     */
    private double[] mQualityScores;
    
    /**
     * Indicates whether inliers must be computed and kept.
     */
    private boolean mComputeAndKeepInliers;
    
    /**
     * Indicates whether residuals must be computed and kept.
     */
    private boolean mComputeAndKeepResiduals;    
    
    /**
     * Constructor.
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix 
     * estimator.
     */
    public PROSACFundamentalMatrixRobustEstimator(
            FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod) {
        super(fundMatrixEstimatorMethod);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
    }
    
    /**
     * Constructor.
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix 
     * estimator.
     * @param listener listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     */
    public PROSACFundamentalMatrixRobustEstimator(
            FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod,
            FundamentalMatrixRobustEstimatorListener listener) {
        super(fundMatrixEstimatorMethod, listener);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    /**
     * Constructor.
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix 
     * estimator.
     * @param leftPoints 2D points on left view.
     * @param rightPoints 2D points on right view.
     * @throws IllegalArgumentException if provided list of points do not have
     * the same length or their length is less than 7 points.
     */
    public PROSACFundamentalMatrixRobustEstimator(
            FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod,
            List<Point2D> leftPoints, List<Point2D> rightPoints) 
            throws IllegalArgumentException {
        super(fundMatrixEstimatorMethod, leftPoints, rightPoints);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    /**
     * Constructor.
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix 
     * estimator.
     * @param leftPoints 2D points on left view.
     * @param rightPoints 2D points on right view.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided list of points do not have
     * the same length or their length is less than 7 points.
     */
    public PROSACFundamentalMatrixRobustEstimator(
            FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod,
            List<Point2D> leftPoints, List<Point2D> rightPoints, 
            FundamentalMatrixRobustEstimatorListener listener) 
            throws IllegalArgumentException {
        super(fundMatrixEstimatorMethod, leftPoints, rightPoints, listener);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }

    /**
     * Constructor.
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix 
     * estimator.
     * @param qualityScores quality scores corresponding to each provided pair
     * of matched points.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than required size (i.e. 7 matched pair of points).
     */
    public PROSACFundamentalMatrixRobustEstimator(
            FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod,
            double[] qualityScores) throws IllegalArgumentException {
        this(fundMatrixEstimatorMethod);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix 
     * estimator.
     * @param qualityScores quality scores corresponding to each provided pair
     * of matched points.
     * @param listener listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than required size (i.e. 7 matched pair of points).
     */
    public PROSACFundamentalMatrixRobustEstimator(
            FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod,
            double[] qualityScores, 
            FundamentalMatrixRobustEstimatorListener listener) 
            throws IllegalArgumentException {
        this(fundMatrixEstimatorMethod, listener);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix 
     * estimator.
     * @param qualityScores quality scores corresponding to each provided pair
     * of matched points.
     * @param leftPoints 2D points on left view.
     * @param rightPoints 2D points on right view.
     * @throws IllegalArgumentException if provided list of points or quality 
     * scores do not have the same length or their length is less than 7 points.
     */
    public PROSACFundamentalMatrixRobustEstimator(
            FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod,
            double[] qualityScores, List<Point2D> leftPoints, 
            List<Point2D> rightPoints) throws IllegalArgumentException {
        this(fundMatrixEstimatorMethod, leftPoints, rightPoints);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix 
     * estimator.
     * @param qualityScores quality scores corresponding to each provided pair
     * of matched points.
     * @param leftPoints 2D points on left view.
     * @param rightPoints 2D points on right view.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided list of points or quality 
     * scores do not have the same length or their length is less than 7 points.
     */
    public PROSACFundamentalMatrixRobustEstimator(
            FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod,
            double[] qualityScores, List<Point2D> leftPoints, 
            List<Point2D> rightPoints, 
            FundamentalMatrixRobustEstimatorListener listener) 
            throws IllegalArgumentException {
        this(fundMatrixEstimatorMethod, leftPoints, rightPoints, listener);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     */
    public PROSACFundamentalMatrixRobustEstimator() {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     */
    public PROSACFundamentalMatrixRobustEstimator(
            FundamentalMatrixRobustEstimatorListener listener) {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, listener);
    }
    
    /**
     * Constructor.
     * @param leftPoints 2D points on left view.
     * @param rightPoints 2D points on right view.
     * @throws IllegalArgumentException if provided list of points do not have
     * the same length or their length is less than 7 points.
     */
    public PROSACFundamentalMatrixRobustEstimator(List<Point2D> leftPoints,
            List<Point2D> rightPoints) throws IllegalArgumentException {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, leftPoints, 
                rightPoints);
    }
    
    /**
     * Constructor.
     * @param leftPoints 2D points on left view.
     * @param rightPoints 2D points on right view.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided list of points do not have
     * the same length or their length is less than 7 points.
     */
    public PROSACFundamentalMatrixRobustEstimator(List<Point2D> leftPoints,
            List<Point2D> rightPoints, 
            FundamentalMatrixRobustEstimatorListener listener) 
            throws IllegalArgumentException {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, leftPoints, 
                rightPoints, listener);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided pair
     * of matched points.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than required size (i.e. 7 matched pair of points).
     */
    public PROSACFundamentalMatrixRobustEstimator(double[] qualityScores)
            throws IllegalArgumentException {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, qualityScores);
    }
    
    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided pair
     * of matched points.
     * @param listener listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than required size (i.e. 7 matched pair of points).
     */
    public PROSACFundamentalMatrixRobustEstimator(double[] qualityScores,
            FundamentalMatrixRobustEstimatorListener listener) 
            throws IllegalArgumentException {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, qualityScores, 
                listener);
    }
    
    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided pair
     * of matched points.
     * @param leftPoints 2D points on left view.
     * @param rightPoints 2D points on right view.
     * @throws IllegalArgumentException if provided list of points or quality 
     * scores do not have the same length or their length is less than 7 points.
     */
    public PROSACFundamentalMatrixRobustEstimator(double[] qualityScores,
            List<Point2D> leftPoints, List<Point2D> rightPoints) 
            throws IllegalArgumentException {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, qualityScores, 
                leftPoints, rightPoints);
    }
    
    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided pair
     * of matched points.
     * @param leftPoints 2D points on left view.
     * @param rightPoints 2D points on right view.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided list of points or quality 
     * scores do not have the same length or their length is less than 7 points.
     */
    public PROSACFundamentalMatrixRobustEstimator(double[] qualityScores,
            List<Point2D> leftPoints, List<Point2D> rightPoints, 
            FundamentalMatrixRobustEstimatorListener listener) 
            throws IllegalArgumentException {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, qualityScores, 
                leftPoints, rightPoints, listener);
    }
    
    /**
     * Returns threshold to determine whether matched pairs of points are 
     * inliers or not when testing possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance) a given
     * point has respect to the epipolar line generated by its matched point.
     * @return threshold to determine whether matched pairs of points are
     * inliers or not.
     */
    public double getThreshold() {
        return mThreshold;
    }
    
    /**
     * Sets threshodl to determine whether matched pairs of points are inliers
     * or not when testing possible estimation solutions.
     * @param threshold threshold to be set.
     * @throws IllegalArgumentException if provided value is equal or less than
     * zero.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     */
    public void setThreshold(double threshold) throws IllegalArgumentException,
            LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (threshold <= MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        mThreshold = threshold;
    }

    /**
     * Returns quality scores corresponding to each provided pair of points.
     * The larger the score value the better the quality of the sampled matched 
     * pair of points.
     * @return quality scores corresponding to each pair of points.
     */
    @Override
    public double[] getQualityScores() {
        return mQualityScores;
    }
    
    /**
     * Sets quality scores corresponding to each provided pair of points.
     * The larger the score value the better the quality of the sampled matched 
     * pair of points.
     * @param qualityScores quality scores corresponding to each pair of points.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    @Override
    public void setQualityScores(double[] qualityScores) throws LockedException,
            IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }    
    
    /**
     * Returns value indicating whether required data has been provided so that
     * fundamental matrix estimation can start.
     * This is true when input data (i.e. 7 pairs of matched 2D points and their
     * quality scores) are provided.
     * If true, estimator is ready to compute a fundamental matrix, otherwise
     * more data needs to be provided.
     * @return true if estimator is ready, false otherwise.
     */    
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null && 
                mQualityScores.length == mLeftPoints.size();
    }     
    
    /**
     * Indicates whether inliers must be computed and kept.
     * @return true if inliers must be computed and kept, false if inliers
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepInliersEnabled() {
        return mComputeAndKeepInliers;
    }
    
    /**
     * Specifies whether inliers must be computed and kept.
     * @param computeAndKeepInliers true if inliers must be computed and kept,
     * false if inliers only need to be computed but not kept.
     * @throws LockedException if estimator is locked.
     */
    public void setComputeAndKeepInliersEnabled(boolean computeAndKeepInliers) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mComputeAndKeepInliers = computeAndKeepInliers;
    }
    
    /**
     * Indicates whether residuals must be computed and kept.
     * @return true if residuals must be computed and kept, false if residuals
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepResidualsEnabled() {
        return mComputeAndKeepResiduals;
    }
    
    /**
     * Specifies whether residuals must be computed and kept.
     * @param computeAndKeepResiduals true if residuals must be computed and
     * kept, false if residuals only need to be computed but not kept.
     * @throws LockedException if estimator is locked.
     */
    public void setComputeAndKeepResidualsEnabled(
            boolean computeAndKeepResiduals) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mComputeAndKeepResiduals = computeAndKeepResiduals;
    }
    
    /**
     * Estimates a radial distortion using a robust estimator and
     * the best set of matched 2D points found using the robust estimator.
     * @return a radial distortion.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */        
    @Override
    public FundamentalMatrix estimate() throws LockedException, 
            NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        PROSACRobustEstimator<FundamentalMatrix> innerEstimator =
                new PROSACRobustEstimator<>(
                new PROSACRobustEstimatorListener<FundamentalMatrix>() {
                                
            //subset of left points
            private List<Point2D> mSubsetLeftPoints = new ArrayList<>();
            
            //subset of right points
            private List<Point2D> mSubsetRightPoints = new ArrayList<>();

            @Override
            public double getThreshold() {
                return mThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mLeftPoints.size();
            }

            @Override
            public int getSubsetSize() {
                return getMinRequiredPoints();
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<FundamentalMatrix> solutions) {
                
                mSubsetLeftPoints.clear();
                mSubsetRightPoints.clear();
                for (int samplesIndex : samplesIndices) {
                    mSubsetLeftPoints.add(mLeftPoints.get(samplesIndex));
                    mSubsetRightPoints.add(mRightPoints.get(samplesIndex));
                }
                
                nonRobustEstimate(solutions, mSubsetLeftPoints, 
                        mSubsetRightPoints);
            }

            @Override
            public double computeResidual(FundamentalMatrix currentEstimation, 
                    int i) {
                Point2D leftPoint = mLeftPoints.get(i);
                Point2D rightPoint = mRightPoints.get(i);
                return residual(currentEstimation, leftPoint, rightPoint);
            }

            @Override
            public boolean isReady() {
                return PROSACFundamentalMatrixRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<FundamentalMatrix> estimator) {
                if (mListener != null) {
                    mListener.onEstimateStart(
                            PROSACFundamentalMatrixRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<FundamentalMatrix> estimator) {
                if (mListener != null) {
                    mListener.onEstimateEnd(
                            PROSACFundamentalMatrixRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<FundamentalMatrix> estimator, 
                    int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            PROSACFundamentalMatrixRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<FundamentalMatrix> estimator, 
                    float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            PROSACFundamentalMatrixRobustEstimator.this, 
                            progress);
                }
            }
            
            @Override
            public double[] getQualityScores() {
                return mQualityScores;
            }                                         
        });
        
        try {
            mLocked = true;
            mInliersData = null;
            innerEstimator.setComputeAndKeepInliersEnabled(
                    mComputeAndKeepInliers || mRefineResult);
            innerEstimator.setComputeAndKeepResidualsEnabled(
                    mComputeAndKeepResiduals || mRefineResult);
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            FundamentalMatrix result = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();
            return attemptRefine(result);
        } catch (com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } finally {
            mLocked = false;
        }
    }

    /**
     * Returns method being used for robust estimation.
     * @return method being used for robust estimation.
     */        
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROSAC;
    }    
    
    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during 
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically this value is related to the threshold used on each robust 
     * estimation, since residuals of found inliers are within the range of 
     * such threshold.
     * @return standard deviation used for refinement.
     */
    @Override
    protected double getRefinementStandardDeviation() {
        return mThreshold;
    }
    
    /**
     * Sets quality scores corresponding to each provided pair of matched 
     * points.
     * This method is used internally and does not check whether instance is
     * locked or not.
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than 7 points.
     */
    private void internalSetQualityScores(double[] qualityScores) 
            throws IllegalArgumentException {
        if (qualityScores.length < getMinRequiredPoints()) {
            throw new IllegalArgumentException();
        }
        
        mQualityScores = qualityScores;        
    }     
}
