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
package com.irurueta.ar.sfm;

import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Robustly triangulates 3D points from matched 2D points and their
 * corresponding cameras on several views using PROMedS algorithm.
 */
public class PROMedSRobustSinglePoint3DTriangulator extends 
        RobustSinglePoint3DTriangulator {
    
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
     * The larger the score value the better the quality of the sample.
     */
    private double[] mQualityScores;    
    
    /**
     * Constructor.
     */
    public PROMedSRobustSinglePoint3DTriangulator() {
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public PROMedSRobustSinglePoint3DTriangulator(
            RobustSinglePoint3DTriangulatorListener listener) {
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param points Matched 2D points. Each point in the list is assumed to be 
     * projected by the corresponding camera in the list.
     * @param cameras List of cameras associated to the matched 2D point on the 
     * same position as the camera on the list.
     * @throws IllegalArgumentException if provided lists don't have the same
     * length or their length is less than 2 views, which is the minimum 
     * required to compute triangulation.
     */
    public PROMedSRobustSinglePoint3DTriangulator(List<Point2D> points, 
            List<PinholeCamera> cameras) throws IllegalArgumentException {
        super(points, cameras);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param points Matched 2D points. Each point in the list is assumed to be 
     * projected by the corresponding camera in the list.
     * @param cameras List of cameras associated to the matched 2D point on the 
     * same position as the camera on the list.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided lists don't have the same
     * length or their length is less than 2 views, which is the minimum 
     * required to compute triangulation.
     */
    public PROMedSRobustSinglePoint3DTriangulator(List<Point2D> points,
            List<PinholeCamera> cameras, 
            RobustSinglePoint3DTriangulatorListener listener) 
            throws IllegalArgumentException {
        super(points, cameras, listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided view.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than requiredsize (i.e. 2 views).
     */
    public PROMedSRobustSinglePoint3DTriangulator(double[] qualityScores) 
            throws IllegalArgumentException {
        this();
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided view.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than requiredsize (i.e. 2 views).
     */
    public PROMedSRobustSinglePoint3DTriangulator(double[] qualityScores,
            RobustSinglePoint3DTriangulatorListener listener) {
        this(listener);
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param points Matched 2D points. Each point in the list is assumed to be 
     * projected by the corresponding camera in the list.
     * @param cameras List of cameras associated to the matched 2D point on the 
     * same position as the camera on the list.
     * @param qualityScores quality scores corresponding to each provided view.
     * @throws IllegalArgumentException if provided lists or quality scores 
     * don't have the same length or their length is less than 2 views, which 
     * is the minimum required to compute triangulation.
     */
    public PROMedSRobustSinglePoint3DTriangulator(List<Point2D> points, 
            List<PinholeCamera> cameras, double[] qualityScores) 
            throws IllegalArgumentException {
        this(points, cameras);
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param points Matched 2D points. Each point in the list is assumed to be 
     * projected by the corresponding camera in the list.
     * @param cameras List of cameras associated to the matched 2D point on the 
     * same position as the camera on the list.
     * @param qualityScores quality scores corresponding to each provided view.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided lists or quality scores 
     * don't have the same length or their length is less than 2 views, which is 
     * the minimum required to compute triangulation.
     */
    public PROMedSRobustSinglePoint3DTriangulator(List<Point2D> points,
            List<PinholeCamera> cameras, double[] qualityScores,
            RobustSinglePoint3DTriangulatorListener listener) 
            throws IllegalArgumentException {
        this(points, cameras, listener);
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
     * Returns quality scores corresponding to each provided view.
     * The larger the score value the better the quality of the sampled view.
     * @return quality scores corresponding to each view.
     */
    @Override
    public double[] getQualityScores() {
        return mQualityScores;
    }
    
    /**
     * Sets quality scores corresponding to each provided view.
     * The larger the score value the better the quality of the sampled view.
     * @param qualityScores quality scores corresponding to each view.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than MIN_REQUIRED_VIEWS (i.e. 2 views).
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
     * Indicates if triangulator is ready to start the 3D point triangulation.
     * This is true when input data (i.e. 2D points, cameras and quality scores) 
     * are provided and a minimum of 2 views are available.
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null && 
                mQualityScores.length == mPoints2D.size();
    }     
    
    
    /**
     * Triangulates provided matched 2D points being projected by each 
     * corresponding camera into a single 3D point.
     * At least 2 matched 2D points and their corresponding 2 cameras are 
     * required to compute triangulation. If more views are provided, an 
     * averaged solution can be found.
     * @return computed triangulated 3D point.
     * @throws LockedException if this instance is locked.
     * @throws NotReadyException if lists of points and cameras don't have the
     * same length or less than 2 views are provided.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */    
    @Override
    public Point3D triangulate() throws LockedException, NotReadyException, 
            RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        PROMedSRobustEstimator<Point3D> innerEstimator =
                new PROMedSRobustEstimator<>(
                new PROMedSRobustEstimatorListener<Point3D>() {
                    
            //point to be reused when computing residuals
            private Point2D mTestPoint = Point2D.create(
                    CoordinatesType.HOMOGENEOUS_COORDINATES);

            //non-robust 3D point triangulator
            private SinglePoint3DTriangulator mTriangulator = 
                    SinglePoint3DTriangulator.create(mUseHomogeneousSolution ?
                    Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR :
                    Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR);
            
            //subset of 2D points
            private List<Point2D> mSubsetPoints = new ArrayList<>();
            
            //subst of cameras
            private List<PinholeCamera> mSubsetCameras = 
                    new ArrayList<>();
            
            @Override
            public double getThreshold() {
                return mStopThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mPoints2D.size();
            }

            @Override
            public int getSubsetSize() {
                return MIN_REQUIRED_VIEWS;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<Point3D> solutions) {
                mSubsetPoints.clear();
                mSubsetPoints.add(mPoints2D.get(samplesIndices[0]));
                mSubsetPoints.add(mPoints2D.get(samplesIndices[1]));
                
                mSubsetCameras.clear();
                mSubsetCameras.add(mCameras.get(samplesIndices[0]));
                mSubsetCameras.add(mCameras.get(samplesIndices[1]));
                
                try {
                    mTriangulator.setPointsAndCameras(mSubsetPoints, 
                            mSubsetCameras);
                    Point3D triangulated = mTriangulator.triangulate();
                    solutions.add(triangulated);
                } catch (Exception e) {
                    //if anything fails, no solution is added
                }
            }

            @Override
            public double computeResidual(Point3D currentEstimation, int i) {
                Point2D point2D = mPoints2D.get(i);
                PinholeCamera camera = mCameras.get(i);
                
                //project estimated point with camera
                camera.project(currentEstimation, mTestPoint);
                
                //return distance of projected point respect to the original one
                //as a residual
                return mTestPoint.distanceTo(point2D);
            }

            @Override
            public boolean isReady() {
                return PROMedSRobustSinglePoint3DTriangulator.this.isReady();
            }

            @Override
            public void onEstimateStart(RobustEstimator<Point3D> estimator) {
                if (mListener != null) {
                    mListener.onTriangulateStart(
                            PROMedSRobustSinglePoint3DTriangulator.this);
                }
            }

            @Override
            public void onEstimateEnd(RobustEstimator<Point3D> estimator) {
                if (mListener != null) {
                    mListener.onTriangulateEnd(
                            PROMedSRobustSinglePoint3DTriangulator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<Point3D> estimator, int iteration) {
                if (mListener != null) {
                    mListener.onTriangulateNextIteration(
                            PROMedSRobustSinglePoint3DTriangulator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<Point3D> estimator, float progress) {
                if (mListener != null) {
                    mListener.onTriangulateProgressChange(
                            PROMedSRobustSinglePoint3DTriangulator.this, 
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
        return RobustEstimatorMethod.PROMedS;
    }        
    
    /**
     * Sets quality scores corresponding to each provided view.
     * This method is used internally and does not check whether instance is
     * locked or not.
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE.
     */
    private void internalSetQualityScores(double[] qualityScores) 
            throws IllegalArgumentException {
        if (qualityScores.length < MIN_REQUIRED_VIEWS) {
            throw new IllegalArgumentException();
        }
        
        mQualityScores = qualityScores;        
    }          
}
