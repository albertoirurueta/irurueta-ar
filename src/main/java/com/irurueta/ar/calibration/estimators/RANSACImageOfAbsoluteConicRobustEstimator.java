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
package com.irurueta.ar.calibration.estimators;

import com.irurueta.ar.calibration.ImageOfAbsoluteConic;
import com.irurueta.geometry.Transformation2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best Image of Absolute Conic (IAC) using RANSAC algorithm.
 */
public class RANSACImageOfAbsoluteConicRobustEstimator extends 
        ImageOfAbsoluteConicRobustEstimator {

    /**
     * Constant defining default threshold to determine whether homographies are 
     * inliers or not.
     * Threshold is defined by equations h1'*IAC*h2 = 0 and 
     * h1'*IAC*h1 = h2'*IAC*h2 --&lt; h1'*IAC*h1 - h2'*IAC*h2 = 0, where
     * h1 and h2 are the 1st and 2nd columns of an homography (2D 
     * transformation).
     * These equations are derived from the fact that rotation matrices are
     * orthonormal.
     */
    public static final double DEFAULT_THRESHOLD = 1e-6;
        
    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Threshold to determine whether homographies are inliers or not when 
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on 
     * the orthonormality assumption of rotation matrices.
     */
    private double mThreshold;    
    
    /**
     * Constructor.
     */
    public RANSACImageOfAbsoluteConicRobustEstimator() {
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     */
    public RANSACImageOfAbsoluteConicRobustEstimator(
            ImageOfAbsoluteConicRobustEstimatorListener listener) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param homographies list of homographies (2D transformations) used to
     * estimate the image of absolute conic (IAC), which can be used to obtain
     * pinhole camera intrinsic parameters.
     * @throws IllegalArgumentException if not enough homographies are provided
     * for default settings. Hence, at least 1 homography must be provided.
     */
    public RANSACImageOfAbsoluteConicRobustEstimator(
            List<Transformation2D> homographies) 
            throws IllegalArgumentException {
        super(homographies);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param homographies list of homographies (2D transformations) used to
     * estimate the image of absolute conic (IAC), which can be used to obtain
     * pinhole camera intrinsic parameters.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if not enough homographies are provided
     * for default settings. Hence, at least 1 homography must be provided.
     */
    public RANSACImageOfAbsoluteConicRobustEstimator(
            List<Transformation2D> homographies, 
            ImageOfAbsoluteConicRobustEstimatorListener listener)
            throws IllegalArgumentException {
        super(homographies, listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Returns threshold to determine whether homographies are inliers or not
     * when testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on
     * the orthonormality assumption of rotation matrices.
     * @return threshold to determine whether homographies are inliers or not
     * when testing possible estimation solutions.
     */
    public double getThreshold() {
        return mThreshold;
    }
    
    /**
     * Sets threshold to determine whether homographies are iliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on
     * the orthonormality assumption of rotation matrices.
     * @param threshold threshold to determine whether homographies are inliers 
     * or not when testing possible estimation solutions.
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
     * Estimates Image of Absolute Conic (IAC).
     * @return estimated IAC.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numberical instability, no solution available, etc).
     */    
    @Override
    public ImageOfAbsoluteConic estimate() throws LockedException, 
            NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        RANSACRobustEstimator<ImageOfAbsoluteConic> innerEstimator =
                new RANSACRobustEstimator<>(
                new RANSACRobustEstimatorListener<ImageOfAbsoluteConic>(){

            //subset of homographies picked on each iteration
            private List<Transformation2D> mSubsetHomographies = 
                    new ArrayList<>();
                    
            @Override
            public double getThreshold() {
                return mThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mHomographies.size();
            }

            @Override
            public int getSubsetSize() {
                return mIACEstimator.getMinNumberOfRequiredHomographies();
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<ImageOfAbsoluteConic> solutions) {
                mSubsetHomographies.clear();
                for (int samplesIndex : samplesIndices) {
                    mSubsetHomographies.add(mHomographies.get(
                            samplesIndex));
                }
                
                try {
                    mIACEstimator.setLMSESolutionAllowed(false);
                    mIACEstimator.setHomographies(mSubsetHomographies);
                    
                    ImageOfAbsoluteConic iac = mIACEstimator.estimate();
                    solutions.add(iac);
                } catch (Exception e) {
                    //if anything fails, no solution is added
                }
            }

            @Override
            public double computeResidual(
                    ImageOfAbsoluteConic currentEstimation, int i) {
                return residual(currentEstimation, mHomographies.get(i));
            }

            @Override
            public boolean isReady() {
                return RANSACImageOfAbsoluteConicRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<ImageOfAbsoluteConic> estimator) {
                if (mListener != null) {
                    mListener.onEstimateStart(
                            RANSACImageOfAbsoluteConicRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<ImageOfAbsoluteConic> estimator) {
                if (mListener != null) {
                    mListener.onEstimateEnd(
                            RANSACImageOfAbsoluteConicRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<ImageOfAbsoluteConic> estimator, 
                    int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            RANSACImageOfAbsoluteConicRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<ImageOfAbsoluteConic> estimator, 
                    float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            RANSACImageOfAbsoluteConicRobustEstimator.this, 
                            progress);
                }
            }
        });
        
        try {
            mLocked = true;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            return innerEstimator.estimate();
        } catch(com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch(com.irurueta.numerical.NotReadyException e) {
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
        return RobustEstimatorMethod.RANSAC;
    }    
}
