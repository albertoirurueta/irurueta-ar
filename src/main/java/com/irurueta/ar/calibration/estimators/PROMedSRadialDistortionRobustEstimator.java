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
 * PROMedS algorithm.
 */
public class PROMedSRadialDistortionRobustEstimator extends 
        RadialDistortionRobustEstimator {
    
    /**
     * Default value to be used for stop threshold. Stop threshold can be used 
     * to keep the algorithm iterating in case that best estimated threshold 
     * using median of residuals is not small enough. Once a solution is found 
     * that generates a threshold below this value, the algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close 
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would 
     * iterate for a long time trying to find the best solution when indeed 
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    public static final double DEFAULT_STOP_THRESHOLD = 1e-3;
        
    /**
     * Minimum allowed stop threshold value.
     */
    public static final double MIN_STOP_THRESHOLD = 0.0;
    
    /**
     * Threshold to be used to keep the algorithm iterating in case that best 
     * estimated threshold using median of residuals is not small enough. Once 
     * a solution is found that generates a threshold below this value, the 
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close 
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would 
     * iterate for a long time trying to find the best solution when indeed 
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    private double mStopThreshold;    
    
    /**
     * Quality scores corresponding to each provided point.
     * The larger the score value the betther the quality of the sample.
     */
    private double[] mQualityScores;    
    
    /**
     * Constructor.
     */
    public PROMedSRadialDistortionRobustEstimator() {
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     */
    public PROMedSRadialDistortionRobustEstimator(
            RadialDistortionRobustEstimatorListener listener) {
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MIN_NUMBER_OF_POINTS.
     */
    public PROMedSRadialDistortionRobustEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints) throws IllegalArgumentException {
        super(distortedPoints, undistortedPoints);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
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
    public PROMedSRadialDistortionRobustEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, 
            RadialDistortionRobustEstimatorListener listener) 
            throws IllegalArgumentException {
        super(distortedPoints, undistortedPoints, listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
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
    public PROMedSRadialDistortionRobustEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, Point2D distortionCenter) 
            throws IllegalArgumentException {
        super(distortedPoints, undistortedPoints, distortionCenter);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
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
    public PROMedSRadialDistortionRobustEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, Point2D distortionCenter,
            RadialDistortionRobustEstimatorListener listener) 
            throws IllegalArgumentException {
        super(distortedPoints, undistortedPoints, distortionCenter, listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }    

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than required size (i.e. 2 points).
     */
    public PROMedSRadialDistortionRobustEstimator(double[] qualityScores)
            throws IllegalArgumentException {
        this();
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided point.
     * @param listener listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than required size (i.e. 2 points).
     */
    public PROMedSRadialDistortionRobustEstimator(double[] qualityScores,
            RadialDistortionRobustEstimatorListener listener) throws IllegalArgumentException {
        this(listener);
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided lists of points and quality 
     * scores don't have the same size or their size is smaller than 
     * MIN_NUMBER_OF_POINTS (i.e. 2 points).
     */
    public PROMedSRadialDistortionRobustEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, double[] qualityScores) 
            throws IllegalArgumentException {
        this(distortedPoints, undistortedPoints);
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param qualityScores quality scores corresponding to each provided point.
     * @param listener listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided lists of points or quality 
     * scores don't have the same size or their size is smaller than 
     * MIN_NUMBER_OF_POINTS (i.e. 2 points).
     */
    public PROMedSRadialDistortionRobustEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, double[] qualityScores,
            RadialDistortionRobustEstimatorListener listener) 
            throws IllegalArgumentException {
        this(distortedPoints, undistortedPoints, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param qualityScores quality scores corresponding to each provided point.
     * @param distortionCenter radial distortion center. If null it is assumed 
     * to be the origin of coordinates, otherwise this is typically equal to
     * the camera principal point.
     * @throws IllegalArgumentException if provided lists of points or quality 
     * scores don't have the same size or their size is smaller than 
     * MIN_NUMBER_OF_POINTS (i.e. 2 points).
     */
    public PROMedSRadialDistortionRobustEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, double[] qualityScores, 
            Point2D distortionCenter) throws IllegalArgumentException {
        this(distortedPoints, undistortedPoints, distortionCenter);
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param qualityScores quality scores corresponding to each provided point.
     * @param distortionCenter radial distortion center. If null it is assumed 
     * to be the origin of coordinates, otherwise this is typically equal to
     * the camera principal point.
     * @param listener listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided lists of points or quality 
     * scores don't have the same size or their size is smaller than 
     * MIN_NUMBER_OF_POINTS (i.e. 2 points).
     */
    public PROMedSRadialDistortionRobustEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, double[] qualityScores, 
            Point2D distortionCenter,
            RadialDistortionRobustEstimatorListener listener) 
            throws IllegalArgumentException {
        this(distortedPoints, undistortedPoints, distortionCenter, listener);
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Returns threshold to be used to keep the algorithm iterating in case that 
     * best estimated threshold using median of residuals is not small enough. 
     * Once a solution is found that generates a threshold below this value, the 
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close 
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would 
     * iterate for a long time trying to find the best solution when indeed 
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     * @return stop threshold to stop the algorithm prematurely when a certain
     * accuracy has been reached.
     */
    public double getStopThreshold() {
        return mStopThreshold;
    }
    
    /**
     * Sets threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough. 
     * Once a solution is found that generates a threshold below this value, the 
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close 
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would 
     * iterate for a long time trying to find the best solution when indeed 
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     * @param stopThreshold stop threshold to stop the algorithm prematurely 
     * when a certain accuracy has been reached.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     */
    public void setStopThreshold(double stopThreshold) 
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (stopThreshold <= MIN_STOP_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        
        mStopThreshold = stopThreshold;
    }
    
    /**
     * Returns quality scores corresponding to each provided point.
     * The larger the score value the betther the quality of the sampled point.
     * @return quality scores corresponding to each point.
     */
    @Override
    public double[] getQualityScores() {
        return mQualityScores;
    }
    
    /**
     * Sets quality scores corresponding to each provided point.
     * The larger the score value the better the quality of the sampled point.
     * @param qualityScores quality scores corresponding to each point.
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
     * Indicates if eatimator is ready to start the radial distortion 
     * estimation.
     * This is true when input data (i.e. 2D points and quality scores) are 
     * provided and a minimum of 2 points are available.
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null && 
                mQualityScores.length == mDistortedPoints.size();
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
        
        PROMedSRobustEstimator<RadialDistortion> innerEstimator = 
                new PROMedSRobustEstimator<>(
                new PROMedSRobustEstimatorListener<RadialDistortion>() {
                    
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
                return mStopThreshold;
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
                return PROMedSRadialDistortionRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<RadialDistortion> estimator) {
                try {
                    mRadialDistortionEstimator.setLMSESolutionAllowed(false);
                    mRadialDistortionEstimator.setIntrinsic(getIntrinsic());
                } catch (Exception e) {
                    Logger.getLogger(
                            PROMedSRadialDistortionRobustEstimator.class.getName()).
                            log(Level.WARNING, 
                            "Could not set intrinsic parameters on radial distortion estimator", e);
                }
                
                if (mListener != null) {
                    mListener.onEstimateStart(
                            PROMedSRadialDistortionRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<RadialDistortion> estimator) {
                if (mListener != null) {
                    mListener.onEstimateEnd(
                            PROMedSRadialDistortionRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<RadialDistortion> estimator, 
                    int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            PROMedSRadialDistortionRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<RadialDistortion> estimator, 
                    float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            PROMedSRadialDistortionRobustEstimator.this, 
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
     * Returns method being used for robust estimation
     * @return method being used for robust estimation
     */    
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROMedS;
    }   
    
    /**
     * Sets quality scores corresponding to each provided point.
     * This method is used internally and does not check whether instance is
     * locked or not.
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE.
     */
    private void internalSetQualityScores(double[] qualityScores) 
            throws IllegalArgumentException {
        if (qualityScores.length < MIN_NUMBER_OF_POINTS) {
            throw new IllegalArgumentException();
        }
        
        mQualityScores = qualityScores;        
    }         
}
