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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.ar.calibration.RadialDistortion;
import com.irurueta.ar.calibration.RadialDistortionException;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

import java.util.List;

/**
 * This class defines the interface for an estimator of radial distortion
 */
@SuppressWarnings("WeakerAccess")
public abstract class RadialDistortionEstimator {
    
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
     * Default estimator type.
     */
    public static final RadialDistortionEstimatorType DEFAULT_ESTIMATOR_TYPE =
            RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR;
    
    /**
     * True when estimator is estimating radial distortion.
     */
    protected boolean mLocked;
    
    /**
     * Listener to be notified of events such as when estimation starts, ends or
     * estimation progress changes.
     */
    protected RadialDistortionEstimatorListener mListener;
    
    /**
     * Distortion center. This is usually equal to the principal point
     * of an estimated camera. If not set it is assumed to be at the origin of
     * coordinates (0,0).
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
     * Intrinsic parameters matrix.
     */
    protected Matrix mKinv;
    
    /**
     * List of distorted points. Distorted points are obtained after radial
     * distorsion is applied to an undistorted point.
     */
    protected List<Point2D> mDistortedPoints;
    
    /**
     * List of undistorted points.
     */
    protected List<Point2D> mUndistortedPoints;
    
    /**
     * Number of radial distortion parameters to estimate.
     */
    protected int mNumKParams;
    
    /**
     * Constructor.
     */
    public RadialDistortionEstimator() {
        mLocked = false;
        mListener = null;
        mNumKParams = DEFAULT_NUM_K_PARAMS;
        try {
            setInternalIntrinsic(null, DEFAULT_FOCAL_LENGTH, 
                    DEFAULT_FOCAL_LENGTH, DEFAULT_SKEW);
        } catch (RadialDistortionException ignore) { }
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public RadialDistortionEstimator(RadialDistortionEstimatorListener listener) {
        mLocked = false;
        mListener = listener;
        mNumKParams = DEFAULT_NUM_K_PARAMS;
        try {
            setInternalIntrinsic(null, DEFAULT_FOCAL_LENGTH, 
                    DEFAULT_FOCAL_LENGTH, DEFAULT_SKEW);        
        } catch (RadialDistortionException ignore) { }
    }
    
    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */
    public RadialDistortionEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints) throws IllegalArgumentException {
        this();
        internalSetPoints(distortedPoints, undistortedPoints);
    }
    
    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */
    public RadialDistortionEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, 
            RadialDistortionEstimatorListener listener) 
            throws IllegalArgumentException {
        this(listener);
        internalSetPoints(distortedPoints, undistortedPoints);
    }
    
    /**
     * Constructor with distortion center.
     * @param distortionCenter Distortion center. This is usually equal to the 
     * principal point of an estimated camera. If not set it is assumed to be at
     * the origin of coordinates (0,0).
     */
    public RadialDistortionEstimator(Point2D distortionCenter) {
        mLocked = false;
        mListener = null;
        mNumKParams = DEFAULT_NUM_K_PARAMS;
        try {
            setInternalIntrinsic(distortionCenter, DEFAULT_FOCAL_LENGTH, 
                    DEFAULT_FOCAL_LENGTH, DEFAULT_SKEW);
        } catch (RadialDistortionException ignore) { }
    }
    
    /**
     * Constructor with listener and distortion center.
     * @param distortionCenter Distortion center. This is usually equal to the 
     * principal point of an estimated camera. If not set it is assumed to be at
     * the origin of coordinates (0,0).
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */    
    public RadialDistortionEstimator(Point2D distortionCenter,
            RadialDistortionEstimatorListener listener) {
        this(listener);
        try {
            setInternalIntrinsic(distortionCenter, DEFAULT_FOCAL_LENGTH, 
                    DEFAULT_FOCAL_LENGTH, DEFAULT_SKEW);
        } catch (RadialDistortionException ignore) { }
    }
    
    /**
     * Constructor with points and distortion center.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param distortionCenter Distortion center. This is usually equal to the 
     * principal point of an estimated camera. If not set it is assumed to be at
     * the origin of coordinates (0,0).
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */    
    public RadialDistortionEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, Point2D distortionCenter)
            throws IllegalArgumentException {
        this(distortedPoints, undistortedPoints);
        try {
            setInternalIntrinsic(distortionCenter, DEFAULT_FOCAL_LENGTH, 
                    DEFAULT_FOCAL_LENGTH, DEFAULT_SKEW);
        } catch (RadialDistortionException ignore) { }
    }
    
    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param distortionCenter Distortion center. This is usually equal to the 
     * principal point of an estimated camera. If not set it is assumed to be at
     * the origin of coordinates (0,0).
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */ 
    public RadialDistortionEstimator(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, Point2D distortionCenter,
            RadialDistortionEstimatorListener listener)
            throws IllegalArgumentException {
        this(distortedPoints, undistortedPoints, listener);
        try {
            setInternalIntrinsic(distortionCenter, DEFAULT_FOCAL_LENGTH, 
                    DEFAULT_FOCAL_LENGTH, DEFAULT_SKEW);        
        } catch (RadialDistortionException ignore) { }
    }
    
    /**
     * Returns listener to be notified of events such as when estimation starts,
     * ends or estimation progress changes.
     * @return listener to be notified of events.
     */
    public RadialDistortionEstimatorListener getListener() {
        return mListener;
    }
    
    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or estimation progress changes.
     * @param listener listener to be notified of events.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(RadialDistortionEstimatorListener listener)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mListener = listener;
    }
    
    /**
     * Indicates whether this instance is locked.
     * @return true if this estimator is busy estimating a camera, false
     * otherwise.
     */
    public boolean isLocked() {
        return mLocked;
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
        
        try {
            setInternalIntrinsic(distortionCenter, mHorizontalFocalLength, 
                    mVerticalFocalLength, mSkew);
        } catch (RadialDistortionException ignore) { }
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
     * @throws RadialDistortionException if provided value is degenerate (i.e.
     * zero).
     */
    public void setHorizontalFocalLength(double horizontalFocalLength)
            throws LockedException, RadialDistortionException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        setInternalIntrinsic(mDistortionCenter, horizontalFocalLength, 
                mVerticalFocalLength, mSkew);
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
     * @throws RadialDistortionException if provided value is degenerate (i.e.
     * zero).
     */
    public void setVerticalFocalLength(double verticalFocalLength)
            throws LockedException, RadialDistortionException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        setInternalIntrinsic(mDistortionCenter, mHorizontalFocalLength, 
                verticalFocalLength, mSkew);
    }
    
    /**
     * Returns skew expressed in pixels.
     * @return skew expressed in pixels.
     */
    public double getSkew() {
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
        
        try {
            setInternalIntrinsic(mDistortionCenter, mHorizontalFocalLength, 
                    mVerticalFocalLength, skew);
        } catch (RadialDistortionException ignore) { }
    }
    
    /**
     * Sets intrinsic parameters.
     * @param distortionCenter radial distortion center.
     * @param horizontalFocalLength horizontal focal length expressed in pixels.
     * @param verticalFocalLength vertical focal length expressed in pixels.
     * @param skew skew expressed in pixels.
     * @throws LockedException if estimator is locked.
     * @throws RadialDistortionException if focal length is degenerate (i.e. 
     * zero).
     */
    public final void setIntrinsic(Point2D distortionCenter,
            double horizontalFocalLength, double verticalFocalLength,
            double skew) throws LockedException, RadialDistortionException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        setInternalIntrinsic(distortionCenter, horizontalFocalLength,
                verticalFocalLength, skew);
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
     * @throws RadialDistortionException if focal length is degenerate (i.e.
     * zero).
     */
    public void setIntrinsic(PinholeCameraIntrinsicParameters intrinsic) 
            throws LockedException, RadialDistortionException {
        setIntrinsic(new InhomogeneousPoint2D(
                intrinsic.getHorizontalPrincipalPoint(),
                intrinsic.getVerticalPrincipalPoint()), 
                intrinsic.getHorizontalFocalLength(),
                intrinsic.getVerticalFocalLength(),
                intrinsic.getSkewness());
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
     * Returns minimum number of required matched points to compute the radial
     * distortion. This is equal to the number of radial distortion parameters.
     * The larger the number of radial distortion parameters, the larger the
     * number of matched points required. Typically 2 radial distortion 
     * parameters are enough, since the following terms can be safely neglected
     * @return minimum number of required matched points to compute the radial
     * distortion.
     */
    public int getMinNumberOfMatchedPoints() {
        return mNumKParams;
    }
    
    /**
     * Indicates if lists of corresponding distorted and undistorted points are
     * valid.
     * Lists are considered valid if they have the same number of points and
     * both have more than the required minimum of correspondences (which is 2)
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @return true if lists of points are valid, false otherwise.
     */
    public boolean areValidPoints(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints) {
        if (distortedPoints == null || undistortedPoints == null) {
            return false;
        }
        return distortedPoints.size() == undistortedPoints.size() &&
                distortedPoints.size() >= getMinNumberOfMatchedPoints();
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
     * Estimates a radial distortion.
     * @return estimated radial distortion.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if input has not yet been provided.
     * @throws RadialDistortionEstimatorException if an error occurs during
     * estimation, usually because input data is not valid.
     */
    public abstract RadialDistortion estimate() throws LockedException,
            NotReadyException, RadialDistortionEstimatorException;
    
    /**
     * Returns type of radial distortion estimator.
     * @return type of radial distortion estimator.
     */
    public abstract RadialDistortionEstimatorType getType();
    
    /**
     * Creates an instance of a radial distortion estimator using default
     * type.
     * @return an instance of a radial distortion estimator.
     */
    public static RadialDistortionEstimator create() {
        return create(DEFAULT_ESTIMATOR_TYPE);
    }
    
    /**
     * Creates an instance of a radial distortion estimator using provided
     * type.
     * @param type type of radial distortion estimator.
     * @return an instance of a radial distortion estimator.
     */
    public static RadialDistortionEstimator create(
            RadialDistortionEstimatorType type) {
        switch (type) {
            case WEIGHTED_RADIAL_DISTORTION_ESTIMATOR:     
                return new WeightedRadialDistortionEstimator();
            case LMSE_RADIAL_DISTORTION_ESTIMATOR:
            default:
                return new LMSERadialDistortionEstimator();
        }
    }
    
    /**
     * Internally sets intrinsic parameters.
     * This method does not check whether estimator is locked.
     * @param distortionCenter radial distortion center.
     * @param horizontalFocalLength horizontal focal length expressed in pixels.
     * @param verticalFocalLength vertical focal length expressed in pixels.
     * @param skew skew expressed in pixels.
     * @throws RadialDistortionException if focal length is degenerate (i.e.
     * zero).
     */    
    private void setInternalIntrinsic(Point2D distortionCenter, 
            double horizontalFocalLength, double verticalFocalLength, 
            double skew) throws RadialDistortionException {
        mDistortionCenter = distortionCenter;
        mHorizontalFocalLength = horizontalFocalLength;
        mVerticalFocalLength = verticalFocalLength;
        mSkew = skew;
        
        try {
            if (mKinv == null) {
                mKinv = new Matrix(3, 3);
            }
            
            Matrix K = new Matrix(3, 3); //initially matrix is zero
            
            K.setElementAt(0, 0, horizontalFocalLength);
            K.setElementAt(1, 1, verticalFocalLength);
            K.setElementAt(0, 1, skew);
            if (mDistortionCenter != null) {
                K.setElementAt(0, 2, mDistortionCenter.getInhomX());
                K.setElementAt(1, 2, mDistortionCenter.getInhomY());
            } //if center is not provided, values are zero
            K.setElementAt(2, 2, 1.0);
            
            Utils.inverse(K, mKinv);
        } catch (AlgebraException e) {
            throw new RadialDistortionException(e);
        }
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
