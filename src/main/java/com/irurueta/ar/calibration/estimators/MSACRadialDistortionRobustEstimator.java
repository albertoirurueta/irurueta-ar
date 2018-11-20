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

import com.irurueta.ar.calibration.RadialDistortion;
import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.*;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Finds the best radial distortion for provided collections of 2D points using
 * MSAC algorithm.
 */
public class MSACRadialDistortionRobustEstimator extends 
        RadialDistortionRobustEstimator {
    
    /**
     * Constant defining default threshold to determine whether points are 
     * inliers or not.
     * By defaul 1.0 is considered a good value for cases where measures are
     * done on pixels, since typically the minimum resolution is 1 pixel.
     */
    public static final double DEFAULT_THRESHOLD = 1.0;
        
    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Threshold to determine whether points are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance) a possible 
     * solution has on a matched pair of points.
     */
    private double mThreshold;    
    
    /**
     * Constructor.
     */
    public MSACRadialDistortionRobustEstimator() {
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     */
    public MSACRadialDistortionRobustEstimator(
            RadialDistortionRobustEstimatorListener listener) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MIN_NUMBER_OF_POINTS.
     */
    public MSACRadialDistortionRobustEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints) throws IllegalArgumentException {
        super(distortedPoints, undistortedPoints);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param listener listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MIN_NUMBER_OF_POINTS.
     */
    public MSACRadialDistortionRobustEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, 
            RadialDistortionRobustEstimatorListener listener) 
            throws IllegalArgumentException {
        super(distortedPoints, undistortedPoints, listener);
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param distortionCenter radial distortion center. If null it is assumed 
     * to be the origin of coordinates, otherwise this is typically equal to
     * the camera principal point.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MIN_NUMBER_OF_POINTS.
     */
    public MSACRadialDistortionRobustEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, Point2D distortionCenter) 
            throws IllegalArgumentException {
        super(distortedPoints, undistortedPoints, distortionCenter);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param distortionCenter radial distortion center. If null it is assumed 
     * to be the origin of coordinates, otherwise this is typically equal to
     * the camera principal point.
     * @param listener listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MIN_NUMBER_OF_POINTS.
     */
    public MSACRadialDistortionRobustEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, Point2D distortionCenter,
            RadialDistortionRobustEstimatorListener listener) 
            throws IllegalArgumentException {
        super(distortedPoints, undistortedPoints, distortionCenter, listener);
        mThreshold = DEFAULT_THRESHOLD;
    }    

    /**
     * Returns threshold to determine whether points are inliers or not when 
     * testing possible estimation solutions.
     * The threshold refers to the amount of error (i.e. euclidean distance) a 
     * possible solution has on projected 2D points.
     * @return threshold to determine whether points are inliers or not when 
     * testing possible estimation solutions.
     */
    public double getThreshold() {
        return mThreshold;
    }
    
    /**
     * Sets threshold to determine whether points are inliers or not when 
     * testing possible estimation solutions.
     * Thre threshold refers to the amount of error (i.e. euclidean distance) a
     * possible solution has on projected 2D points.
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
    public RadialDistortion estimate() throws LockedException, 
            NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        MSACRobustEstimator<RadialDistortion> innerEstimator = 
                new MSACRobustEstimator<>(
                new MSACRobustEstimatorListener<RadialDistortion>() {
                    
                    //point to be reused when computing residuals
            private Point2D mTestPoint = Point2D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES);
                    
            //non-robust radial distortion estimator
            private LMSERadialDistortionEstimator mRadialDistortionEstimator = 
                    new LMSERadialDistortionEstimator();
            
            //subset of distorted (i.e. measured) points
            private List<Point2D> mSubsetDistorted = new ArrayList<>();
            
            //subset of undistorted (i.e. ideal) points
            private List<Point2D> mSubsetUndistorted = new ArrayList<>();

            @Override
            public double getThreshold() {
                return mThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mDistortedPoints.size();
            }

            @Override
            public int getSubsetSize() {
                return RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, List<RadialDistortion> solutions) {
                mSubsetDistorted.clear();
                mSubsetDistorted.add(mDistortedPoints.get(samplesIndices[0]));
                mSubsetDistorted.add(mDistortedPoints.get(samplesIndices[1]));
                
                mSubsetUndistorted.clear();
                mSubsetUndistorted.add(mUndistortedPoints.get(samplesIndices[0]));
                mSubsetUndistorted.add(mUndistortedPoints.get(samplesIndices[1]));
                
                try {
                    mRadialDistortionEstimator.setPoints(mDistortedPoints, 
                            mUndistortedPoints);
                    mRadialDistortionEstimator.setPoints(mSubsetDistorted, 
                            mSubsetUndistorted);
                    
                    RadialDistortion distortion = mRadialDistortionEstimator.
                            estimate();
                    solutions.add(distortion);
                } catch (Exception e) {
                    //if anything fails, no solution is added
                }
            }

            @Override
            public double computeResidual(RadialDistortion currentEstimation, int i) {
                Point2D distortedPoint = mDistortedPoints.get(i);
                Point2D undistortedPoint = mUndistortedPoints.get(i);
                
                currentEstimation.distort(undistortedPoint, mTestPoint);
                
                return mTestPoint.distanceTo(distortedPoint);
            }

            @Override
            public boolean isReady() {
                return MSACRadialDistortionRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<RadialDistortion> estimator) {
                try {
                    mRadialDistortionEstimator.setLMSESolutionAllowed(false);
                    mRadialDistortionEstimator.setIntrinsic(getIntrinsic());
                } catch (Exception e) {
                    Logger.getLogger(
                            MSACRadialDistortionRobustEstimator.class.getName()).
                            log(Level.WARNING, 
                            "Could not set intrinsic parameters on radial distortion estimator", e);
                }
                
                if (mListener != null) {
                    mListener.onEstimateStart(MSACRadialDistortionRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<RadialDistortion> estimator) {
                if (mListener != null) {
                    mListener.onEstimateEnd(MSACRadialDistortionRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<RadialDistortion> estimator, 
                    int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            MSACRadialDistortionRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<RadialDistortion> estimator, 
                    float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            MSACRadialDistortionRobustEstimator.this, 
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
     * Returns method being used for robust estimation
     * @return method being used for robust estimation
     */    
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.MSAC;
    }    
}
