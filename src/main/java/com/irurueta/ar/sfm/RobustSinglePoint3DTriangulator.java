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

import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Abstract class for algorithms to robustly triangulate 3D points from matched
 * 2D points and their corresponding cameras on several views.
 * Robust estimators can be used to estimate a more precise triangulation when
 * there are more than 2 views where points have been matched.
 */
public abstract class RobustSinglePoint3DTriangulator {
    
    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD =
            RobustEstimatorMethod.PROMedS;
    
    /**
     * Default amount of progress variation before notifying a change in
     * estimation progress. By default this is set to 5%.
     */
    public static final float DEFAULT_PROGRESS_DELTA = 0.05f;
    
    /**
     * Minimum allowed value for progress delta.
     */
    public static final float MIN_PROGRESS_DELTA = 0.0f;
    
    /**
     * Maximum allowed value for progress delta.
     */
    public static final float MAX_PROGRESS_DELTA = 1.0f;
    
    /**
     * Constant defining default confidence of the estimated result, which is 
     * 99%. This means that with a probability of 99% estimation will be
     * accurate because chose subsamples will be inliers.
     */
    public static final double DEFAULT_CONFIDENCE = 0.99;
    
    /**
     * Default maximum allowed number of iterations.
     */
    public static final int DEFAULT_MAX_ITERATIONS = 5000;
    
    /**
     * Minimum allowed confidence value.
     */
    public static final double MIN_CONFIDENCE = 0.0;
    
    /**
     * Maximum allowed confidence value.
     */
    public static final double MAX_CONFIDENCE = 1.0;
    
    /**
     * Minimum allowed number of iterations.
     */
    public static final int MIN_ITERATIONS = 1;
    
    /**
     * Minimum required number of views to triangulate 3D points.
     */
    public static final int MIN_REQUIRED_VIEWS = 2;
    
    /**
     * Indicates whether by default a solution to an homogeneous system of
     * equations should be found.
     */
    public static final boolean DEFAULT_USE_HOMOGENEOUS_SOLUTION = true;
    
    /**
     * Matched 2D points. Each point in the list is assumed to be projected by
     * the corresponding camera in the list.
     */
    protected List<Point2D> mPoints2D;
    
    /**
     * List of cameras associated to the matched 2D point on the same position
     * as the camera on the list.
     */
    protected List<PinholeCamera> mCameras;
            
    /**
     * Listener to be notified of events such as when estimation starts, ends or
     * its progress significantly changes.
     */
    protected RobustSinglePoint3DTriangulatorListener mListener;
    
    /**
     * Indicates whether a solution to an homogeneous system of equations should
     * be found. Typically this should be true, since even points and cameras
     * at infinity can be used. If points are close and geometry is well 
     * defined, false can be used to solve an inhomogeneous system of equations
     * and obtain a slightly better accuracy.
     */
    protected boolean mUseHomogeneousSolution;
    
    /**
     * Indicates if this estimator is locked because an estimation is being 
     * computed.
     */
    protected volatile boolean mLocked;    

    /**
     * Amount of progress variation before notifying a progress change during
     * estimation.
     */
    protected float mProgressDelta;
    
    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%). The amount of confidence indicates the probability
     * that the estimated result is correct. Usually this value will be close
     * to 1.0, but not exactly 1.0.
     */
    protected double mConfidence;
    
    /**
     * Maximum allowed number of iterations. When the maximum number of 
     * iterations is exceeded, result will not be available, however an 
     * approximate result will be available for retrieval.
     */
    protected int mMaxIterations;
    
    /**
     * Constructor.
     */
    public RobustSinglePoint3DTriangulator() {
        mUseHomogeneousSolution = DEFAULT_USE_HOMOGENEOUS_SOLUTION;
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public RobustSinglePoint3DTriangulator(
            RobustSinglePoint3DTriangulatorListener listener) {
        this();
        mListener = listener;
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
    public RobustSinglePoint3DTriangulator(List<Point2D> points, 
            List<PinholeCamera> cameras) throws IllegalArgumentException {
        this();
        internalSetPointsAndCameras(points, cameras);
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
    public RobustSinglePoint3DTriangulator(List<Point2D> points,
            List<PinholeCamera> cameras, 
            RobustSinglePoint3DTriangulatorListener listener) 
            throws IllegalArgumentException {
        this(points, cameras);
        mListener = listener;
    }
    
    /**
     * Returns reference to listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     * @return listener to be notified of events.
     */
    public RobustSinglePoint3DTriangulatorListener getListener() {
        return mListener;
    }
    
    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or its progress significantly changes.
     * @param listener listener to be notified of events.
     * @throws LockedException if robust estimator is locked.
     */
    public void setListener(RobustSinglePoint3DTriangulatorListener listener)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mListener = listener;
    }
    
    /**
     * Indicates whether listener has been provided and is available for 
     * retrieval.
     * @return true if available, false otherwise.
     */
    public boolean isListenerAvailable() {
        return mListener != null;
    }
    
    /**
     * Indicates whether a solution to an homogeneous system of equations should
     * be found. Typically this should be true, since even points and cameras
     * at infinity can be used. If points are close and geometry is well 
     * defined, false can be used to solve an inhomogeneous system of equations
     * and obtain a slightly better accuracy.
     * @return true if an homogeneous solution must be found (default value),
     * false otherwise.
     */
    public boolean isUseHomogeneousSolution() {
        return mUseHomogeneousSolution;
    }    
    
    /**
     * Sets boolean indicating whether a solution to an homogeneous system of 
     * equations should be found. Typically this should be true, since even 
     * points and cameras at infinity can be used. If points are close and 
     * geometry is well defined, false can be used to solve an inhomogeneous 
     * system of equations and obtain a slightly better accuracy.
     * @param useHomogeneousSolution true if an homogeneous solution will be
     * found, false if an inhomogeneous solution will be found instead.
     * @throws LockedException if this instance is locked.
     */
    public void setUseHomogeneousSolution(boolean useHomogeneousSolution)
            throws LockedException{
        if (isLocked()) {
            throw new LockedException();
        }
        mUseHomogeneousSolution = useHomogeneousSolution;
    }
    
    /**
     * Indicates if this instance is locked because triangulation is being
     * computed.
     * @return true if locked, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }
    
    /**
     * Returns amount of progress variation before notifying a progress change
     * during estimation.
     * @return amount of progress variation before notifying a progress change
     * during estimation.
     */
    public float getProgressDelta() {
        return mProgressDelta;
    }
    
    /**
     * Sets amount of progress variation before notifying a progress change
     * during estimation.
     * @param progressDelta amount of progress variation before notifying a
     * progress change during estimation.
     * @throws IllegalArgumentException if progress delta is less than zero or
     * greater than 1.
     * @throws LockedException if this estimator is locked because an estimation
     * is being computed.
     */
    public void setProgressDelta(float progressDelta) 
            throws IllegalArgumentException, LockedException{
        if (isLocked()) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA ||
                progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        mProgressDelta = progressDelta;
    }
    
    /**
     * Returns amount of confidence expressed as a value between 0.0 and 1.0
     * (which is equivalent to 100%). The amount of confidence indicates the
     * probability that the estimated result is correct. Usually this value will
     * be close to 1.0, but not exactly 1.0.
     * @return amount of confidence as a value between 0.0 and 1.0.
     */
    public double getConfidence() {
        return mConfidence;
    }
    
    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which
     * is equivalent to 100%). The amount of confidence indicates the 
     * probability that the estimated result is correct. Usually this value will
     * be close to 1.0, but not exactly 1.0.
     * @param confidence confidence to be set as a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 
     * 1.0.
     * @throws LockedException if this estimator is locked because an estimator 
     * is being computed.
     */
    public void setConfidence(double confidence)
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (confidence < MIN_CONFIDENCE || confidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        mConfidence = confidence;
    }
    
    /**
     * Returns maximum allowed number of iterations. If maximum allowed number
     * of iterations is achieved without converging to a result when calling 
     * estimate(), a RobustEstimatorException will be raised.
     * @return maximum allowed number of iterations.
     */
    public int getMaxIterations() {
        return mMaxIterations;
    }
    
    /**
     * Sets maximum allowed number of iterations. When the maximum number of
     * iterations is exceeded, result will not be available, however an 
     * approximate result will be available for retrieval.
     * @param maxIterations maximum allowed number of iterations to be set.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException if this estimator is locked because an estimation
     * is being computed.
     */
    public void setMaxIterations(int maxIterations) 
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (maxIterations < MIN_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        mMaxIterations = maxIterations;
    }    
    
    /**
     * Sets list of matched 2D points for each view and their corresponding
     * cameras used to project them.
     * @param points2D list of matched 2D points on each view. Each point in the
     * list is assumed to be projected by the corresponding camera in the list.
     * @param cameras cameras for each view where 2D points are represented.
     * @throws LockedException if this instance is locked.
     * @throws IllegalArgumentException if provided lists don't have the same
     * length or their length is less than 2 views, which is the minimum 
     * required to compute triangulation.
     */
    public void setPointsAndCameras(List<Point2D> points2D, 
            List<PinholeCamera> cameras) throws LockedException, 
            IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPointsAndCameras(points2D, cameras);
    }

    /**
     * Returns list of matched 2D points on each view. Each point in the list is
     * assumed to be projected by the corresponding camera.
     * @return list of matched 2D points on each view.
     */
    public List<Point2D> getPoints2D() {
        return mPoints2D;
    }
    
    /**
     * Returns cameras for each view where 2D points are represented.
     * @return cameras for each view where 2D points are represented.
     */
    public List<PinholeCamera> getCameras() {
        return mCameras;
    }    
    
    /**
     * Returns quality scores corresponding to each view.
     * The larger the score value the better the quality of the view measure.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behaviour.
     * @return quality scores corresponding to each view.
     */
    public double[] getQualityScores() {
        return null;
    }
    
    /**
     * Sets quality scores corresponding to each view.
     * The larger the score value the better the quality of the view sample.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     * @param qualityScores quality scores corresponding to each view.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MIN_REQUIRED_VIEWS (i.e. 2 views).
     */
    public void setQualityScores(double[] qualityScores) throws LockedException,
            IllegalArgumentException { }
    
    
    /**
     * Indicates whether this instance is ready to start the triangulation.
     * An instance is ready when both lists of 2D points and cameras are 
     * provided, both lists have the same length and at least data for 2 views
     * is provided.
     * @return true if this instance is ready, false otherwise.
     */
    public boolean isReady() {
        return SinglePoint3DTriangulator.areValidPointsAndCameras(mPoints2D, 
                mCameras);
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
    public abstract Point3D triangulate() throws LockedException, 
            NotReadyException, RobustEstimatorException;
    
    /**
     * Returns method being used for robust estimation.
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Creates a robust single 3D point triangulator using provided robust 
     * method.
     * @param method method of a robust estimator algorithm to estimate best
     * triangulation.
     * @return an instance of a robust single 3D point triangulator.
     */
    public static RobustSinglePoint3DTriangulator create(
            RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustSinglePoint3DTriangulator();
            case LMedS:
                return new LMedSRobustSinglePoint3DTriangulator();
            case MSAC:
                return new MSACRobustSinglePoint3DTriangulator();
            case PROSAC:
                return new PROSACRobustSinglePoint3DTriangulator();
            case PROMedS:
            default:
                return new PROMedSRobustSinglePoint3DTriangulator();
        }
    }
    
    /**
     * Creates a robust single 3D point triangulator using provided points, 
     * cameras and robust method.
     * @param points matched 2D points. Each point in the list is assumed to
     * be projected by the corresponding camera in the list.
     * @param cameras list of cameras associated to the matched 2D point on the
     * same position as the camera on the list.
     * @param method method of a robust estimator algorithm to estimate best
     * triangulation.
     * @throws IllegalArgumentException if provided lists don't have the same
     * length or their length is less than 2 views, which is the minimum 
     * required to compute triangulation.
     * @return an instance of a robust single 3D point triangulator.
     */
    public static RobustSinglePoint3DTriangulator create(List<Point2D> points,
            List<PinholeCamera> cameras, RobustEstimatorMethod method) 
            throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustSinglePoint3DTriangulator(points, 
                        cameras);
            case LMedS:
                return new LMedSRobustSinglePoint3DTriangulator(points, 
                        cameras);
            case MSAC:
                return new MSACRobustSinglePoint3DTriangulator(points, cameras);
            case PROSAC:
                return new PROSACRobustSinglePoint3DTriangulator(points, 
                        cameras);
            case PROMedS:
            default:
                return new PROMedSRobustSinglePoint3DTriangulator(points, 
                        cameras);
        }
    }

    /**
     * Creates a robust single 3D point triangulator using provided points, 
     * cameras and robust method.
     * @param points matched 2D points. Each point in the list is assumed to
     * be projected by the corresponding camera in the list.
     * @param cameras list of cameras associated to the matched 2D point on the
     * same position as the camera on the list.
     * @param qualityScores quality scores corresponding to each point.
     * @param method method of a robust estimator algorithm to estimate best
     * triangulation.
     * @throws IllegalArgumentException if provided lists or quality scores 
     * don't have the same length or their length is less than 2 views, which 
     * is the minimum required to compute triangulation.
     * @return an instance of a robust single 3D point triangulator.
     */
    public static RobustSinglePoint3DTriangulator create(List<Point2D> points,
            List<PinholeCamera> cameras, double[] qualityScores, 
            RobustEstimatorMethod method) 
            throws IllegalArgumentException {
        switch (method) {
            case RANSAC:
                return new RANSACRobustSinglePoint3DTriangulator(points, 
                        cameras);
            case LMedS:
                return new LMedSRobustSinglePoint3DTriangulator(points, 
                        cameras);
            case MSAC:
                return new MSACRobustSinglePoint3DTriangulator(points, cameras);
            case PROSAC:
                return new PROSACRobustSinglePoint3DTriangulator(points, 
                        cameras, qualityScores);
            case PROMedS:
            default:
                return new PROMedSRobustSinglePoint3DTriangulator(points, 
                        cameras, qualityScores);
        }
    }
    
    /**
     * Creates a robust single 3D point triangulator using default robust 
     * method.
     * @return an instance of a robust single 3D point triangulator.
     */
    public static RobustSinglePoint3DTriangulator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a robust single 3D point triangulator using provided points, 
     * cameras and default robust method.
     * @param points matched 2D points. Each point in the list is assumed to
     * be projected by the corresponding camera in the list.
     * @param cameras list of cameras associated to the matched 2D point on the
     * same position as the camera on the list.
     * @throws IllegalArgumentException if provided lists don't have the same
     * length or their length is less than 2 views, which is the minimum 
     * required to compute triangulation.
     * @return an instance of a robust single 3D point triangulator.
     */
    public static RobustSinglePoint3DTriangulator create(List<Point2D> points,
            List<PinholeCamera> cameras) throws IllegalArgumentException {
        return create(points, cameras, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust single 3D point triangulator using provided points, 
     * cameras and default robust method.
     * @param points matched 2D points. Each point in the list is assumed to
     * be projected by the corresponding camera in the list.
     * @param cameras list of cameras associated to the matched 2D point on the
     * same position as the camera on the list.
     * @param qualityScores quality scores corresponding to each point.
     * @throws IllegalArgumentException if provided lists or quality scores 
     * don't have the same length or their length is less than 2 views, which 
     * is the minimum required to compute triangulation.
     * @return an instance of a robust single 3D point triangulator.
     */
    public static RobustSinglePoint3DTriangulator create(List<Point2D> points,
            List<PinholeCamera> cameras, double[] qualityScores) 
            throws IllegalArgumentException {
        return create(points, cameras, qualityScores, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Internal method to sets list of matched 2D points for each view and their 
     * corresponding cameras used to project them.
     * This method does not check whether instance is locked.
     * @param points2D list of matched 2D points on each view. Each point in the
     * list is assumed to be projected by the corresponding camera in the list.
     * @param cameras cameras for each view where 2D points are represented.
     * @throws IllegalArgumentException if provided lists don't have the same
     * length or their length is less than 2 views, which is the minimum 
     * required to compute triangulation.
     */    
    private void internalSetPointsAndCameras(List<Point2D> points2D,
            List<PinholeCamera> cameras) throws IllegalArgumentException {
        
        if (!SinglePoint3DTriangulator.areValidPointsAndCameras(points2D, cameras)) {
            throw new IllegalArgumentException();
        }
        
        mPoints2D = points2D;
        mCameras = cameras;
    }        
}
