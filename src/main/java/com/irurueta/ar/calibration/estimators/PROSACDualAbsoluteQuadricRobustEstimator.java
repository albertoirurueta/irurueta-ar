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
 * cameras using PROSAC algorithm.
 */
public class PROSACDualAbsoluteQuadricRobustEstimator extends 
        DualAbsoluteQuadricRobustEstimator {
    
    /**
     * Constant defining default threshold to determine whether cameras are
     * inliers or not.
     * Threshold is defined by the equations used to estimate the DAQ depending
     * on the required settings (zero skewness, principal point at origin, and 
     * known aspect ratio).
     */
    public static final double DEFAULT_THRESHOLD = 1e-3;
    
    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Threshold to determine whether cameras are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has.
     */
    private double mThreshold;
    
    /**
     * Quality scores corresponding to each provided homography.
     * The larger the score value the better the quality of the sample.
     */
    private double[] mQualityScores;
    
    /**
     * Constructor.
     */
    public PROSACDualAbsoluteQuadricRobustEstimator() {
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public PROSACDualAbsoluteQuadricRobustEstimator(
            DualAbsoluteQuadricRobustEstimatorListener listener) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param cameras list of cameras used to estimate de fual absolute quadric 
     * (DAQ), which can be used to obtain pinhole camera intrinsic parameters.
     * @throws IllegalArgumentException if not enough cameras are provided for 
     * default settings. Hence, at least 2 cameras must be provided.
     */
    public PROSACDualAbsoluteQuadricRobustEstimator(List<PinholeCamera> cameras)
            throws IllegalArgumentException {
        super(cameras);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param cameras list of cameras used to estimate de fual absolute quadric 
     * (DAQ), which can be used to obtain pinhole camera intrinsic parameters.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if not enough cameras are provided for 
     * default settings. Hence, at least 2 cameras must be provided.
     */
    public PROSACDualAbsoluteQuadricRobustEstimator(List<PinholeCamera> cameras,
            DualAbsoluteQuadricRobustEstimatorListener listener)
            throws IllegalArgumentException {
        super(cameras, listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided 
     * camera.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than required number of cameras for default settings (i.e. 2 
     * cameras).
     */
    public PROSACDualAbsoluteQuadricRobustEstimator(double[] qualityScores)
            throws IllegalArgumentException {
        this();
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided 
     * camera.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than required number of cameras for default settings (i.e. 2 
     * cameras).
     */
    public PROSACDualAbsoluteQuadricRobustEstimator(double[] qualityScores,
            DualAbsoluteQuadricRobustEstimatorListener listener)
            throws IllegalArgumentException {
        this(listener);
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param cameras list of cameras used to estimate the dual absolute quadric
     * (DAQ), which can be used to obtain pinhole camera intrinsic parameters.
     * @param qualityScores quality scores corresponding to each provided 
     * camera.
     * @throws IllegalArgumentException if not enough cameras are provided  for
     * default settings (i.e. 2 cameras) or quality scores and cameras don't 
     * have the same size.
     */
    public PROSACDualAbsoluteQuadricRobustEstimator(List<PinholeCamera> cameras, 
            double[] qualityScores) throws IllegalArgumentException {
        this(cameras);
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param cameras list of cameras used to estimate the dual absolute quadric
     * (DAQ), which can be used to obtain pinhole camera intrinsic parameters.
     * @param qualityScores quality scores corresponding to each provided 
     * camera.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if not enough cameras are provided
     * for default settings (i.e. 2 cameras) or quality scores and cameras
     * don't have the same size.
     */
    public PROSACDualAbsoluteQuadricRobustEstimator(List<PinholeCamera> cameras,
            double[] qualityScores, 
            DualAbsoluteQuadricRobustEstimatorListener listener) 
            throws IllegalArgumentException {
        this(cameras, listener);
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Returns threshold to determine whether cameras are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has.
     * @return threshold to determine whether cameras are inliers or not when
     * testing possible estimation solutions.
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
     * estimation is already in progres.
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
     * Returns quality scores corresponding to each provided camera.
     * The larger the score value the better the quality of the sampled camera.
     * @return quality scores corresponding to each camera.
     */
    @Override
    public double[] getQualityScores() {
        return mQualityScores;
    }
    
    /**
     * Sets quality scores corresponding to each provided camera.
     * The larger the score value the better the quality of the smapled camera.
     * @param qualityScores quality scores corresponding to each camera.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than minimum required number of cameras (i.e. 2 cameras).
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
     * Indicates if estimator is ready to start the DAQ estimation.
     * This is true when input data (i.e. cameras) is provided and contains
     * at least the minimum number of required cameras, and also quality scores
     * are provided.
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null &&
                mQualityScores.length == mCameras.size();
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

        PROSACRobustEstimator<DualAbsoluteQuadric> innerEstimator =
                new PROSACRobustEstimator<>(
                new PROSACRobustEstimatorListener<DualAbsoluteQuadric>() {
                    
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
                return PROSACDualAbsoluteQuadricRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<DualAbsoluteQuadric> estimator) {
                if (mListener != null) {
                    mListener.onEstimateStart(
                            PROSACDualAbsoluteQuadricRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<DualAbsoluteQuadric> estimator) {
                if (mListener != null) {
                    mListener.onEstimateEnd(
                            PROSACDualAbsoluteQuadricRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<DualAbsoluteQuadric> estimator, 
                    int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            PROSACDualAbsoluteQuadricRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<DualAbsoluteQuadric> estimator, 
                    float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            PROSACDualAbsoluteQuadricRobustEstimator.this, 
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
        return RobustEstimatorMethod.PROSAC;
    }   
    
    /**
     * Sets quality scores corresponding to each camera.
     * This method is used internally and does not check whether instance is
     * locked or not.
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than the minimum number of required cameras for current settings.
     */
    private void internalSetQualityScores(double[] qualityScores)
            throws IllegalArgumentException {
        if (qualityScores.length < mDAQEstimator.getMinNumberOfRequiredCameras()) {
            throw new IllegalArgumentException();
        }
        mQualityScores = qualityScores;
    }
}
