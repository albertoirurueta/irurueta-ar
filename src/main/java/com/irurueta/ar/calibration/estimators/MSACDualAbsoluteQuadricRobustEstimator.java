/*
 * Copyright (C) 2016 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.ar.calibration.DualAbsoluteQuadric;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best dual absolute quadric (DAQ) for provided collection of
 * cameras using MSAC algorithm.
 */
public class MSACDualAbsoluteQuadricRobustEstimator extends 
        DualAbsoluteQuadricRobustEstimator {
    
    /**
     * Constant defining default threshold to determine whether cameras are 
     * inliers or not.
     * Threshold is defined by the equations used to estimate the DAQ depending
     * on the required settings (zero skewness, principal point at origin, and 
     * known aspect ratio).
     */
    public static final double DEFAULT_THRESHOLD = 1e-6;
    
    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Threshold to determine whether cameras are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has.
     */
    private double mThreshold;
    
    /**
     * Constructor.
     */
    public MSACDualAbsoluteQuadricRobustEstimator() {
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     */
    public MSACDualAbsoluteQuadricRobustEstimator(
            DualAbsoluteQuadricRobustEstimatorListener listener) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param cameras list of cameras used to estimate the dual absolute quadric
     * (DAQ), which can be used to obtain pinhole camera intrinsic parameters.
     * @throws IllegalArgumentException if not enough cameras are provided for
     * dfault settings. Hence, at least 2 cameras must be provided.
     */
    public MSACDualAbsoluteQuadricRobustEstimator(List<PinholeCamera> cameras)
            throws IllegalArgumentException {
        super(cameras);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param cameras list of cameras used to estimate the dual absolute quadric
     * (DAQ), which can be used to obtain pinhole camera intrinsic parameters.
     * @param listener listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if not enough cameras are provided for
     * dfault settings. Hence, at least 2 cameras must be provided.
     */
    public MSACDualAbsoluteQuadricRobustEstimator(List<PinholeCamera> cameras,
            DualAbsoluteQuadricRobustEstimatorListener listener) 
            throws IllegalArgumentException {
        super(cameras, listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Returns threshold to determine whether cameras are inliers or not
     * when testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has.
     * @return threshold to determine whether cameras are inliers when testing
     * possible estimation solutions.
     */
    public double getThreshold() {
        return mThreshold;
    }
    
    /**
     * Sets threshold to determine whether cameras are inliers or not when 
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has.
     * @param threshold threshold to determine whether cameras are inliers or 
     * not when testing possible estimation solutions.
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
     * Estimates the Dual Absolute Quadric using provided cameras.
     * @return estimated Dual Absolute Quadric (DAQ).
     * @throws LockedException if robust estimator is locked.
     * @throws NotReadyException if no valid input data has already been 
     * provided.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */        
    @Override
    public DualAbsoluteQuadric estimate() throws LockedException, 
            NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        MSACRobustEstimator<DualAbsoluteQuadric> innerEstimator =
                new MSACRobustEstimator<>(
                new MSACRobustEstimatorListener<DualAbsoluteQuadric>() {
                    
            //subset of cameras picked on each iteration
            private List<PinholeCamera> mSubsetCameras = 
                    new ArrayList<>();
            
            @Override
            public double getThreshold() {
                return mThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mCameras.size();
            }

            @Override
            public int getSubsetSize() {
                return mDAQEstimator.getMinNumberOfRequiredCameras();
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<DualAbsoluteQuadric> solutions) {
                mSubsetCameras.clear();
                for (int samplesIndex : samplesIndices) {
                    mSubsetCameras.add(mCameras.get(samplesIndex));
                }
                
                try {
                    mDAQEstimator.setLMSESolutionAllowed(false);
                    mDAQEstimator.setCameras(mSubsetCameras);
                    
                    DualAbsoluteQuadric daq = mDAQEstimator.estimate();
                    solutions.add(daq);
                } catch (Exception e) {
                    //if anything fails, no solution is added
                }
            }

            @Override
            public double computeResidual(DualAbsoluteQuadric currentEstimation,
                    int i) {
                return residual(currentEstimation, mCameras.get(i));
            }

            @Override
            public boolean isReady() {
                return MSACDualAbsoluteQuadricRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<DualAbsoluteQuadric> estimator) {
                if (mListener != null) {
                    mListener.onEstimateStart(
                            MSACDualAbsoluteQuadricRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<DualAbsoluteQuadric> estimator) {
                if (mListener != null) {
                    mListener.onEstimateEnd(
                            MSACDualAbsoluteQuadricRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<DualAbsoluteQuadric> estimator, 
                    int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            MSACDualAbsoluteQuadricRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<DualAbsoluteQuadric> estimator, 
                    float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            MSACDualAbsoluteQuadricRobustEstimator.this, 
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
        return RobustEstimatorMethod.MSAC;
    }    
}
