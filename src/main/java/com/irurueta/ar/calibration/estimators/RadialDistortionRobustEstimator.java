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
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best
 * RadialDistortion for provided collections of matched distorted and 
 * undistorted 2D points.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution
 */
@SuppressWarnings("WeakerAccess")
public abstract class RadialDistortionRobustEstimator {
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
     * accurate because chosen subsamples will be inliers.
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
     * Minimum number of required point correspondences to estimate a radial 
     * distortion.
     */
    public static final int MIN_NUMBER_OF_POINTS = 2;
    
    /**
     * Default number of radial distortion parameters.
     */
    public static final int DEFAULT_NUM_K_PARAMS = 2;
    
    /**
     * Minimum number of radial distortion parameters.
     */
    public static final int MIN_K_PARAMS = 1;
    
    /**
     * Defines default focal length if none is defined.
     */
    public static final double DEFAULT_FOCAL_LENGTH = 1.0;
    
    /**
     * Defines default skewness if none is defined.
     */
    public static final double DEFAULT_SKEW = 0.0;        
    
    /**
     * List of distorted points (aka measured points).
     */
    protected List<Point2D> mDistortedPoints;
    
    /**
     * List of undistorted points (aka ideal points).
     */
    protected List<Point2D> mUndistortedPoints;
    
    /**
     * Distortion center.
     */
    protected Point2D mDistortionCenter;
    
    /**
     * Horizontal focal length expressed in pixels.
     */
    protected double mHorizontalFocalLength;
    
    /**
     * Vertical focal length expressed in pixels.
     */
    protected double mVerticalFocalLength;
    
    /**
     * Skew in pixels.
     */
    protected double mSkew;
    
    /**
     * Number of radial distortion parameters to estimate.
     */
    protected int mNumKParams;    
    
    /**
     * Listener to be notified of events such as when estimation starts, ends
     * or its progress significantly changes.
     */
    protected RadialDistortionRobustEstimatorListener mListener;
    
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
    public RadialDistortionRobustEstimator() {
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
        mConfidence = DEFAULT_CONFIDENCE;
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
        mHorizontalFocalLength = mVerticalFocalLength = DEFAULT_FOCAL_LENGTH;
        mSkew = DEFAULT_SKEW;
        mNumKParams = DEFAULT_NUM_K_PARAMS;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     */
    public RadialDistortionRobustEstimator(
            RadialDistortionRobustEstimatorListener listener) {
        this();
        mListener = listener;
    }
    
    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MIN_NUMBER_OF_POINTS.
     */
    public RadialDistortionRobustEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints) throws IllegalArgumentException {
        this();
        internalSetPoints(distortedPoints, undistortedPoints);
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
    public RadialDistortionRobustEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, 
            RadialDistortionRobustEstimatorListener listener) throws IllegalArgumentException {
        this(listener);
        internalSetPoints(distortedPoints, undistortedPoints);
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
    public RadialDistortionRobustEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, Point2D distortionCenter) 
            throws IllegalArgumentException {
        this(distortedPoints, undistortedPoints);
        mDistortionCenter = distortionCenter;
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
    public RadialDistortionRobustEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, Point2D distortionCenter,
            RadialDistortionRobustEstimatorListener listener) throws IllegalArgumentException {
        this(distortedPoints, undistortedPoints, listener);
        mDistortionCenter = distortionCenter;
    }
    
    /**
     * Returns reference to listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     * @return listener to be notified of events.
     */
    public RadialDistortionRobustEstimatorListener getListener() {
        return mListener;
    }
    
    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or its progress significantly changes.
     * @param listener listener to be notified of events.
     * @throws LockedException if robust estimator is locked.
     */
    public void setListener(RadialDistortionRobustEstimatorListener listener) 
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
     * Indicates if this instance is locked because estimation is being computed.
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
            throws IllegalArgumentException, LockedException {
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
     * approximate result will be available for retrieval
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
     * Sets list of corresponding distorted and undistorted points.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */
    public void setPoints(List<Point2D> distortedPoints, 
            List<Point2D> undistortedPoints) throws LockedException, 
            IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        internalSetPoints(distortedPoints, undistortedPoints);
    }
    
    /**
     * Returns list of distorted points. Distorted points are obtained after
     * radial distorsion is applied to an undistorted point.
     * @return list of distorted points.
     */
    public List<Point2D> getDistortedPoints() {
        return mDistortedPoints;
    }
    
    /**
     * Returns list of undistorted points.
     * @return list of undistorted points.
     */
    public List<Point2D> getUndistortedPoints() {
        return mUndistortedPoints;
    }
    
    /**
     * Returns distortion center. This is usually equal to the principal point
     * of an estimated camera. If not set it is assumed to be at the origin of
     * coordinates (0,0).
     * @return distortion center or null.
     */
    public Point2D getDistortionCenter() {
        return mDistortionCenter;
    }
        
    /**
     * Sets distortion center. This is usually equal to the principal point of
     * an estimated camera. If not set it is assumed to be at the origin of
     * coordinates (0,0).
     * @param distortionCenter distortion center, or null if set at origin of
     * coordinates.
     * @throws LockedException if estimator is locked.
     */
    public void setDistortionCenter(Point2D distortionCenter) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mDistortionCenter = distortionCenter;
    }    
    
    /**
     * Returns horizontal focal length expressed in pixels.
     * @return horizontal focal length expressed in pixels.
     */
    public double getHorizontalFocalLength() {
        return mHorizontalFocalLength;
    }
    
    /**
     * Sets horizontal focal length expressed in pixels.
     * @param horizontalFocalLength horizontal focal length expressed in pixels.
     * @throws LockedException if estimator is locked.
     */
    public void setHorizontalFocalLength(double horizontalFocalLength)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mHorizontalFocalLength = horizontalFocalLength;
    }
    
    /**
     * Returns vertical focal length expressed in pixels.
     * @return vertical focal length expressed in pixels.
     */
    public double getVerticalFocalLength() {
        return mVerticalFocalLength;
    }
    
    /**
     * Sets vertical focal length expressed in pixels.
     * @param verticalFocalLength vertical focal length expressed in pixels.
     * @throws LockedException if estimator is locked.
     */
    public void setVerticalFocalLength(double verticalFocalLength)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mVerticalFocalLength = verticalFocalLength;
    }
    
    /**
     * Returns skew expressed in pixels.
     * @return skew expressed in pixels.
     */
    public double getSkew(){
        return mSkew;
    }
    
    /**
     * Sets skew expressed in pixels.
     * @param skew skew expressed in pixels.
     * @throws LockedException if estimator is locked.
     */
    public void setSkew(double skew) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mSkew = skew;
    }
    
    /**
     * Returns pinhole camera intrinsic parameters associated to this estimator.
     * @return pinhole camera intrinsic parameters associated to this estimator.
     */
    public PinholeCameraIntrinsicParameters getIntrinsic() {
        return new PinholeCameraIntrinsicParameters(mHorizontalFocalLength,
                mVerticalFocalLength, 
                mDistortionCenter != null ? mDistortionCenter.getInhomX() : 0.0, 
                mDistortionCenter != null ? mDistortionCenter.getInhomY() : 0.0,
                mSkew);
    }
    
    /**
     * Sets intrinsic parameters for this estimator from pinhole camera
     * intrinsic parameters.
     * @param intrinsic intrinsic parameters.
     * @throws LockedException if estimator is locked.
     */
    public void setIntrinsic(PinholeCameraIntrinsicParameters intrinsic)
            throws LockedException {
        setIntrinsic(
                new InhomogeneousPoint2D(intrinsic.getHorizontalPrincipalPoint(),
                intrinsic.getVerticalPrincipalPoint()), 
                intrinsic.getHorizontalFocalLength(),
                intrinsic.getVerticalFocalLength(),
                intrinsic.getSkewness());
    }
    
    /**
     * Sets intrinsic parameters for this estimator.
     * @param distortionCenter distortion center.
     * @param horizontalFocalLength horizontal focal length in pixels.
     * @param verticalFocalLength vertical focal length in pixels.
     * @param skew skew in pixels.
     * @throws LockedException if estimator is locked.
     */
    public void setIntrinsic(Point2D distortionCenter, 
            double horizontalFocalLength, double verticalFocalLength,
            double skew) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mDistortionCenter = distortionCenter;
        mHorizontalFocalLength = horizontalFocalLength;
        mVerticalFocalLength = verticalFocalLength;
        mSkew = skew;
    }
    
    /**
     * Returns number of radial distortion parameters to estimate.
     * @return number of radial distortion parameters to estimate.
     */
    public int getNumKParams() {
        return mNumKParams;
    }
    
    /**
     * Sets number of radial distortion parameters to estimate.
     * @param numKParams number of radial distortion parameters to estimate.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if number of parameters is less than 1.
     */
    public void setNumKParams(int numKParams) throws LockedException, 
            IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (numKParams < MIN_K_PARAMS) {
            throw new IllegalArgumentException();
        }
        
        mNumKParams = numKParams;
    }    
    
    /**
     * Indicates if lists of corresponding points have already been provided and
     * are available for retrieval.
     * @return true if available, false otherwise.
     */
    public boolean arePointsAvailable() {
        return mDistortedPoints != null && mUndistortedPoints != null;
    }    
    
    /**
     * Indicates if this estimator is ready to start th estimation.
     * This is true when lists of points are provided, having equal size and
     * at least 2 points.
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return arePointsAvailable() && 
                areValidPoints(mDistortedPoints, mUndistortedPoints);
    }    
    
    /**
     * Returns quality scores corresponding to each point.
     * The larger the score value the better the quality of the point measure.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behaviour.
     * @return quality scores corresponding to each point.
     */
    public double[] getQualityScores() {
        return null;
    }
    
    /**
     * Sets quality scores corresponding to each point.
     * The larger the score value the better the quality of the point sample.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     * @param qualityScores quality scores corresponding to each sampled point.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    public void setQualityScores(double[] qualityScores) throws LockedException,
            IllegalArgumentException { }

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
    public abstract RadialDistortion estimate() throws LockedException,
            NotReadyException, RobustEstimatorException;
    
    /**
     * Returns method being used for robust estimation.
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();
    
    /**
     * Creates a radial distortion robust estimator using provided robust method.
     * @param method method of a robust estimator algorithm to estimate best
     * radial distortion.
     * @return an instance of a radial distortion robust estimator.
     */
    public static RadialDistortionRobustEstimator create(
            RobustEstimatorMethod method) {
        switch (method) {
            case LMedS:
                return new LMedSRadialDistortionRobustEstimator();
            case MSAC:
                return new MSACRadialDistortionRobustEstimator();
            case PROSAC:
                return new PROSACRadialDistortionRobustEstimator();
            case PROMedS:
                return new PROMedSRadialDistortionRobustEstimator();
            case RANSAC:
            default:
                return new RANSACRadialDistortionRobustEstimator();
        }
    }
    
    /**
     * Creates a radial distortion robust estimator using provided distorted and
     * undistorted points, as well as the distortion center. If no distortion
     * center is provided, it is assumed to be at the origin of coordinates.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param qualityScores quality scores corresponding to each point.
     * @param distortionCenter Distortion center. This is usually equal to the 
     * principal point of an estimated camera. If not set it is assumed to be at
     * the origin of coordinates (0,0).
     * @param method method of a robust estimator algorithm to estimate best
     * radial distortion.
     * @return an instance of a radial distortion robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */
    public static RadialDistortionRobustEstimator create(
            List<Point2D> distortedPoints, List<Point2D> undistortedPoints, 
            double[] qualityScores, Point2D distortionCenter, 
            RobustEstimatorMethod method) throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSRadialDistortionRobustEstimator(distortedPoints,
                        undistortedPoints, distortionCenter);
            case MSAC:
                return new MSACRadialDistortionRobustEstimator(distortedPoints,
                        undistortedPoints, distortionCenter);
            case PROSAC:
                return new PROSACRadialDistortionRobustEstimator(
                        distortedPoints, undistortedPoints, qualityScores, 
                        distortionCenter);
            case PROMedS:
                return new PROMedSRadialDistortionRobustEstimator(
                        distortedPoints, undistortedPoints, qualityScores, 
                        distortionCenter);                
            case RANSAC:
            default:
                return new RANSACRadialDistortionRobustEstimator(
                        distortedPoints, undistortedPoints, distortionCenter);
        }
    }

    /**
     * Creates a radial distortion robust estimator using provided distorted and
     * undistorted points, as well as the distortion center. If no distortion
     * center is provided, it is assumed to be at the origin of coordinates.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param distortionCenter Distortion center. This is usually equal to the 
     * principal point of an estimated camera. If not set it is assumed to be at
     * the origin of coordinates (0,0).
     * @param method method of a robust estimator algorithm to estimate best
     * radial distortion.
     * @return an instance of a radial distortion robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */
    public static RadialDistortionRobustEstimator create(
            List<Point2D> distortedPoints, List<Point2D> undistortedPoints, 
            Point2D distortionCenter, RobustEstimatorMethod method) 
            throws IllegalArgumentException {
        switch (method) {
            case LMedS:
                return new LMedSRadialDistortionRobustEstimator(distortedPoints,
                        undistortedPoints, distortionCenter);
            case MSAC:
                return new MSACRadialDistortionRobustEstimator(distortedPoints,
                        undistortedPoints, distortionCenter);
            case PROSAC:
                return new PROSACRadialDistortionRobustEstimator(
                        distortedPoints, undistortedPoints, distortionCenter);
            case PROMedS:
                return new PROMedSRadialDistortionRobustEstimator(
                        distortedPoints, undistortedPoints, distortionCenter);                
            case RANSAC:
            default:
                return new RANSACRadialDistortionRobustEstimator(
                        distortedPoints, undistortedPoints, distortionCenter);
        }
    }
    
    /**
     * Creates a radial distortion robust estimator using provided distorted and
     * undistorted points and assuming that distortion center is at origin of
     * coordinates.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param qualityScores quality scores corresponding to each point.
     * @param method method of a robust estimator algorithm to estimate best
     * radial distortion.
     * @return an instance of a radial distortion robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */
    public static RadialDistortionRobustEstimator create(
            List<Point2D> distortedPoints, List<Point2D> undistortedPoints,
            double[] qualityScores, RobustEstimatorMethod method) 
            throws IllegalArgumentException {
        return create(distortedPoints, undistortedPoints, qualityScores, null, 
                method);
    }

    /**
     * Creates a radial distortion robust estimator using provided distorted and
     * undistorted points and assuming that distortion center is at origin of
     * coordinates.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param method method of a robust estimator algorithm to estimate best
     * radial distortion.
     * @return an instance of a radial distortion robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */
    public static RadialDistortionRobustEstimator create(
            List<Point2D> distortedPoints, List<Point2D> undistortedPoints,
            RobustEstimatorMethod method) throws IllegalArgumentException{
        return create(distortedPoints, undistortedPoints, (Point2D)null, 
                method);
    }
    
    /**
     * Creates a radial distortion robust estimator using default robust method.
     * @return an instance of a radial distortion robust estimator.
     */
    public static RadialDistortionRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }
    
    
    /**
     * Creates a radial distortion robust estimator using provided distorted and
     * undistorted points assuming that distortion center is at origin of 
     * coordinates and using default robust estimation method.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param qualityScores quality scores corresponding to each point
     * @return an instance of a radial distortion robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */    
    public static RadialDistortionRobustEstimator create(
            List<Point2D> distortedPoints, List<Point2D> undistortedPoints,
            double[] qualityScores) throws IllegalArgumentException {
        return create(distortedPoints, undistortedPoints, qualityScores,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a radial distortion robust estimator using provided distorted and
     * undistorted points assuming that distortion center is at origin of 
     * coordinates and using default robust estimation method.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @return an instance of a radial distortion robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */    
    public static RadialDistortionRobustEstimator create(
            List<Point2D> distortedPoints, List<Point2D> undistortedPoints)
            throws IllegalArgumentException {
        return create(distortedPoints, undistortedPoints, 
                DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a radial distortion robust estimator using provided distorted and
     * undistorted points, as well as the distortion center using the default
     * robust estimation method. If no distortion center is provided, it is 
     * assumed to be at the origin of coordinates.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param qualityScores quality scores corresponding to each point.
     * @param distortionCenter Distortion center. This is usually equal to the 
     * principal point of an estimated camera. If not set it is assumed to be at
     * the origin of coordinates (0,0).
     * @return an instance of a radial distortion robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */    
    public static RadialDistortionRobustEstimator create(
            List<Point2D> distortedPoints, List<Point2D> undistortedPoints,
            double[] qualityScores, Point2D distortionCenter) 
            throws IllegalArgumentException {
        return create(distortedPoints, undistortedPoints, qualityScores, 
                distortionCenter, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a radial distortion robust estimator using provided distorted and
     * undistorted points, as well as the distortion center using the default
     * robust estimation method. If no distortion center is provided, it is 
     * assumed to be at the origin of coordinates.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param distortionCenter Distortion center. This is usually equal to the 
     * principal point of an estimated camera. If not set it is assumed to be at
     * the origin of coordinates (0,0).
     * @return an instance of a radial distortion robust estimator.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */    
    public static RadialDistortionRobustEstimator create(
            List<Point2D> distortedPoints, List<Point2D> undistortedPoints,
            Point2D distortionCenter) throws IllegalArgumentException {
        return create(distortedPoints, undistortedPoints, distortionCenter,
                DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Indicates if lists of corresponding distorted and undistorted points are
     * valid.
     * Lists are considered valid if they have the same number of points and
     * both have more than the required minimum of correspondences (which is 2).
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @return true if lists of points are valid, false otherwise.
     */
    public static boolean areValidPoints(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints) {
        if (distortedPoints == null || undistortedPoints == null) {
            return false;
        }
        return distortedPoints.size() == undistortedPoints.size() &&
                distortedPoints.size() >= MIN_NUMBER_OF_POINTS;
    }    
    
    /**
     * Sets list of corresponding distorted and undistorted points.
     * This method does not check whether estimator is locked.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */
    private void internalSetPoints(List<Point2D> distortedPoints, 
            List<Point2D> undistortedPoints) throws IllegalArgumentException {

        if (distortedPoints == null || undistortedPoints == null) {
            throw new IllegalArgumentException();
        }
        
        if (!areValidPoints(distortedPoints, undistortedPoints)) {
            throw new IllegalArgumentException();
        }
        
        mDistortedPoints = distortedPoints;
        mUndistortedPoints = undistortedPoints;
    }    
}
