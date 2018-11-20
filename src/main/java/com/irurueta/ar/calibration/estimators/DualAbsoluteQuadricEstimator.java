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

import com.irurueta.algebra.*;
import com.irurueta.ar.calibration.DualAbsoluteQuadric;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.NumericalException;
import com.irurueta.numerical.polynomials.Polynomial;

import java.util.List;

/**
 * This class defines the interface for an estimator of the Dual Absolute 
 * Quadric (DAQ) assuming equal vertical and horizontal focal length, no
 * skewness and principal point at the center of the image. 
 * The projection of the Dual Absolute Quadric through a given
 * pinhole camera P, becomes the Dual Image of Absolute Conic (DIAC).
 * Since the DIAC is directly related to the intrinsic parameters K of the 
 * camera P used for projection, then by using a pair of arbitrary cameras,
 * it is possible to solve their corresponding focal length, assuming that:
 * - cameras are arbitrary (usually the initial camera is the identity and must
 * be discarded) as it creates a numerical degeneracy.
 * - all provided cameras have the same intrinsic parameters.
 * - it is assumed that skewness is zero, the principal point is at the center
 * of the image plane (zero), and both horizontal and vertical focal planes are
 * equal.
 */
@SuppressWarnings("WeakerAccess")
public abstract class DualAbsoluteQuadricEstimator {
    
    /**
     * Constant defining whether zero skewness is assumed.
     */
    public static final boolean DEFAULT_ZERO_SKEWNESS = true;
    
    /**
     * Constant defining whether principal point is assumed to be at origin of
     * coordinates.
     */
    public static final boolean DEFAULT_PRINCIPAL_POINT_AT_ORIGIN = true;
    
    /**
     * Constant defining whether aspect ratio of focal distance (i.e. vertical
     * focal distance divided by horizontal focal distance) is known or not.
     * Notice that focal distance aspect ratio is not related to image size
     * aspect ratio. Typically LCD sensor cells are square and hence aspect
     * ratio of focal distances is known and equal to 1.
     */
    public static final boolean DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN = 
            true;
    
    /**
     * Constant defining default aspect ratio of focal distances. This constant
     * takes into account that typically LCD sensor cells are square and hence
     * aspect ratio of focal distances is known and equal to 1.
     */
    public static final double DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO = 1.0;
    
    /**
     * Constant defining whether rank 3 must be enforced in estimated Dual 
     * Absolute Quadric (DAQ), thus making it singular.
     * DAQ is always singular in any arbitrary projective space, however, due to
     * noise in samples, estimation might not be singular.
     * By enforcing singularity an additional constraint is imposed, which might
     * result in less cameras needed for DAQ estimation.
     */
    public static final boolean DEFAULT_ENFORCE_SINGULARITY = true;
    
    /**
     * Indicates whether enforced singularity will be validated by checking that
     * determinant of estimated Dual Absolute Quadric (DAQ) is below a certain
     * threshold.
     */
    public static final boolean DEFAULT_VALIDATE_ENFORCED_SINGULARITY = true;
    
    /**
     * Default threshold to determine whether estimated Dual Absolute Quadric
     * (DAQ) has rank 3 or not.
     */
    public static final double DEFAULT_DETERMINANT_THRESHOLD = 1e-6;
    
    /**
     * Minimum absolute value allowed for aspect ratio of focal distances.
     */
    public static final double MIN_ABS_FOCAL_DISTANCE_ASPECT_RATIO = 1e-6;
    
    /**
     * Minimum number of required equations to solve DAQ.
     */
    protected static final int MIN_REQUIRED_EQUATIONS = 9;
    
    /**
     * Default type for a DAQ estimator.
     */
    public static final DualAbsoluteQuadricEstimatorType DEFAULT_ESTIMATOR_TYPE =
            DualAbsoluteQuadricEstimatorType.LMSE_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR;
    
    /**
     * Indicates whether camera skewness is assumed to be zero or not.
     */
    protected boolean mZeroSkewness;
    
    /**
     * Indicates whether principal point is assumed to be at origin of
     * coordinates or not.
     * If false, the principal point will be estimated, otherwise it will be 
     * assumed to be at image center (i.e. origin of coordinates).
     */
    protected boolean mPrincipalPointAtOrigin;
    
    /**
     * Indicates whether aspect ratio of focal distances (i.e. vertical focal
     * distance divided by horizontal focal distance) is known or not.
     * Notice that focal distance aspect ratio is not related to image size
     * aspect ratio. Typically LCD sensor cells are square and hence aspect
     * ratio of focal distances is known and equal to 1.
     * This value is only taken into account if skewness is assumed to be zero,
     * otherwise it is ignored.
     */
    protected boolean mFocalDistanceAspectRatioKnown;
    
    /**
     * Contains aspect ratio of focal distances (i.e. vertical focal distance
     * divided by horizontal focal distance).
     * This value is only taken into account if skewness is assumed to be zero
     * and focal distance aspect ratio is marked as known, otherwise it is
     * ignored.
     * By default this is 1.0, since it is taken into account that typically
     * LCD sensor cells are square and hence aspect ratio focal distances is
     * known and equal to 1.
     * Notice that focal distance aspect ratio is not related to image size
     * aspect ratio.
     */
    protected double mFocalDistanceAspectRatio;
    
    /**
     * Indicates whether a singular DAQ is enforced or not.
     * Dual Absolute Quadric is singular (has rank 3) in any projective space,
     * however, due to noise in samples, estimated DAQ might not be fully 
     * singular.
     */
    protected boolean mSingularityEnforced;
    
    /**
     * Indicates whether enforced singularity will be validated by checking that
     * determinant of estimated Dual Absolute Quadric (DAQ) is below a certain
     * threshold.
     */
    protected boolean mValidateEnforcedSingularity;

    /**
     * Threshold to determine whether estimated Dual Absolute Quadric (DAQ)
     * has rank 3 or not when validation is enabled.
     */
    protected double mDeterminantThreshold;
        
    /**
     * True when an estimator is estimating the Dual Absolute Quadric (DAQ).
     */
    protected boolean mLocked;
    
    /**
     * Listener to be notified of events such as when estimation starts, ends or
     * estimation progress changes.
     */
    protected DualAbsoluteQuadricEstimatorListener mListener;
    
    /**
     * List of cameras used to estimate the Dual Absolute Quadric (DAQ).
     */
    protected List<PinholeCamera> mCameras;
    
    /**
     * Constructor.
     */
    public DualAbsoluteQuadricEstimator() {
        mZeroSkewness = DEFAULT_ZERO_SKEWNESS;
        mPrincipalPointAtOrigin = DEFAULT_PRINCIPAL_POINT_AT_ORIGIN;
        mFocalDistanceAspectRatioKnown =
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN;
        mFocalDistanceAspectRatio = DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO;
        mSingularityEnforced = DEFAULT_ENFORCE_SINGULARITY;
        mValidateEnforcedSingularity = DEFAULT_VALIDATE_ENFORCED_SINGULARITY;
        mDeterminantThreshold = DEFAULT_DETERMINANT_THRESHOLD;
        
        mLocked = false;
        mListener = null;
        mCameras = null;
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public DualAbsoluteQuadricEstimator(
            DualAbsoluteQuadricEstimatorListener listener) {
        this();
        mListener = listener;
    }
    
    /**
     * Constructor.
     * @param cameras list of cameras used to estimate the Dual Absolute Quadric 
     * (DAQ).
     * @throws IllegalArgumentException if list of cameras is null or invalid
     * for default constraints.
     */
    public DualAbsoluteQuadricEstimator(List<PinholeCamera> cameras) 
            throws IllegalArgumentException {
        this();
        try {
            setCameras(cameras);
        } catch (LockedException ignore) { /* never thrown */ }
    }
    
    /**
     * Constructor.
     * @param cameras list of cameras used to estimate the Dual Absolute Quadric
     * (DAQ).
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if list of cameras is null or invalid 
     * for default contraints.
     */
    public DualAbsoluteQuadricEstimator(List<PinholeCamera> cameras,
            DualAbsoluteQuadricEstimatorListener listener) 
            throws IllegalArgumentException {
        this(cameras);
        mListener = listener;
    }
    
    /**
     * Returns boolean indicating whether camera skewness is assumed to be zero
     * or not.
     * Skewness determines whether LCD sensor cells are properly aligned or not,
     * where zero indicates perfect alignment.
     * Typically skewness is a value equal or very close to zero.
     * @return true if camera skewness is assumed to be zero, otherwise camera
     * skewness is estimated
     */
    public boolean isZeroSkewness() {
        return mZeroSkewness;
    }
    
    /**
     * Sets boolean indicating whether camera skewness is assumed to be zero or
     * not.
     * Skewness determines whether LCD sensor cells are properly aligned or not,
     * where zero indicates perfect alignment.
     * Typically skewness is a value equal or very close to zero.
     * @param zeroSkewness true if camera skewness is assumed to be zero, 
     * otherwise camera skewness is estimated
     * @throws LockedException if estimator is locked
     */
    public void setZeroSkewness(boolean zeroSkewness) throws LockedException {
        if(isLocked()) {
            throw new LockedException();
        }
        
        mZeroSkewness = zeroSkewness;
    }    
    
    /**
     * Returns boolean indicating whether principal point is assumed to be at
     * origin of coordinates or not.
     * Typically principal point is located at image center (origin of 
     * coordinates), and usually matches the center of radial distortion if
     * it is taken into account.
     * @return true if principal point is assumed to be at origin of 
     * coordinates, false if principal point must be estimated
     */
    public boolean isPrincipalPointAtOrigin() {
        return mPrincipalPointAtOrigin;
    }
    
    /**
     * Sets boolean indicating whether principal point is assumed to be at 
     * origin of coordinates or not.
     * Typically principal point is located at image center (origin of 
     * coordinates), and usually matches the center of radial distortion if it
     * is taken into account.
     * @param principalPointAtOrigin true if principal point is assumed to be at
     * origin of coordinates, false if principal point must be estimated
     * @throws LockedException if estimator is locked
     */
    public void setPrincipalPointAtOrigin(boolean principalPointAtOrigin)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mPrincipalPointAtOrigin = principalPointAtOrigin;
    }    
    
    /**
     * Returns boolean indicating whether aspect ratio of focal distances (i.e.
     * vertical focal distance divided by horizontal focal distance) is known or
     * not.
     * Notice that focal distance aspect ratio is not related to image size
     * aspect ratio. Typically LCD sensor cells are square and hence aspect
     * ratio of focal distances is known and equal to 1.
     * This value is only taken into account if skewness is assumed to be zero,
     * otherwise it is ignored.
     * @return true if focal distance aspect ratio is known, false otherwise
     */
    public boolean isFocalDistanceAspectRatioKnown() {
        return mFocalDistanceAspectRatioKnown;
    }    
    
    /**
     * Sets value indicating whether aspect ratio of focal distances (i.e. 
     * vertical focal distance divided by horizontal focal distance) is known or
     * not.
     * Notice that focal distance aspect ratio is not related to image size
     * aspect ratio. Typically LCD sensor cells are square and hence aspect
     * ratio of focal distances is known and equal to 1.
     * This value is only taken into account if skewness is assumed to be zero,
     * otherwise it is ignored.
     * @param focalDistanceAspectRatioKnown true if focal distance aspect ratio
     * is known, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setFocalDistanceAspectRatioKnown(
            boolean focalDistanceAspectRatioKnown) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mFocalDistanceAspectRatioKnown = focalDistanceAspectRatioKnown;
    }
    
    /**
     * Returns aspect ratio of focal distances (i.e. vertical focal distance
     * divided by horizontal focal distance).
     * This value is only taken into account if skewness is assumed to be zero
     * and focal distance aspect ratio is marked as known, otherwise it is
     * ignored.
     * By default this is 1.0, since it is taken into account that typically
     * LCD sensor cells are square and hence aspect ratio focal distances is
     * known and equal to 1.
     * Notice that focal distance aspect ratio is not related to image size
     * aspect ratio.
     * Notice that a negative aspect ratio indicates that vertical axis is 
     * reversed. This can be useful in some situations where image vertical
     * coordinates are reversed respect to the physical world (i.e. in computer
     * graphics typically image vertical coordinates go downwards, while in
     * physical world they go upwards).
     * @return aspect ratio of focal distances.
     */
    public double getFocalDistanceAspectRatio() {
        return mFocalDistanceAspectRatio;
    }
    
    /**
     * Sets aspect ratio of focal distances (i.e. vertical focal distance
     * divided by horizontal focal distance).
     * This value is only taken into account if skewness is assumed to be zero
     * and focal distance aspect ratio is marked as known, otherwise it is
     * ignored.
     * By default this is 1.0, since it is taken into account that typically
     * LCD sensor cells are square and hence aspect ratio focal distances is
     * known and equal to 1.
     * Notice that focal distance aspect ratio is not related to image size
     * aspect ratio.
     * Notice that a negative aspect ratio indicates that vertical axis is 
     * reversed. This can be useful in some situations where image vertical
     * coordinates are reversed respect to the physical world (i.e. in computer
     * graphics typically image vertical coordinates go downwards, while in
     * physical world they go upwards).
     * @param focalDistanceAspectRatio aspect ratio of focal distances to be set.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if focal distance aspect ratio is too 
     * close to zero, as it might produce numerical instabilities.
     */
    public void setFocalDistanceAspectRatio(double focalDistanceAspectRatio)
            throws LockedException, IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (Math.abs(focalDistanceAspectRatio) <
                MIN_ABS_FOCAL_DISTANCE_ASPECT_RATIO) {
            throw new IllegalArgumentException();
        }

        mFocalDistanceAspectRatio = focalDistanceAspectRatio;
    } 
    
    /**
     * Indicates whether a singular DAQ is enforced or not.
     * Dual Absolute Quadric is singular (has rank 3) in any projective space,
     * however, due to noise in samples, estimated DAQ might not be fully 
     * singular.
     * @return true when singular DAQ is enforced, false otherwise.
     */
    public boolean isSingularityEnforced() {
        return mSingularityEnforced;
    }
    
    /**
     * Specifies whether a singular DAQ is enforced or not.
     * Dual Absolute Quadric is singular (has rank 3) in any projective space,
     * however, due to noise in samples, estimated DAQ might not be fully 
     * singular.
     * @param singularityEnforced true when singular DAQ is enforced, false 
     * otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setSingularityEnforced(boolean singularityEnforced) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mSingularityEnforced = singularityEnforced;
    }    
    
    /**
     * Indicates whether enforced singularity will be validated by checking that
     * determinant of estimated Dual Absolute Quadric (DAQ) is below a certain
     * threshold.
     * @return true if enforced singularity is validated, false otherwise.
     */
    public boolean isEnforcedSingularityValidated() {
        return mValidateEnforcedSingularity;
    }
    
    /**
     * Specifies whether enforced singularity will be validated by checking that
     * detemrinant of estimated Dual Absolute Quadric (DAQ) is below a certain
     * threshold.
     * @param validateEnforcedSingularity true if enforced singularity is 
     * validated, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setEnforcedSingularityValidated(
            boolean validateEnforcedSingularity) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mValidateEnforcedSingularity = validateEnforcedSingularity;
    }
    
    /**
     * Returns threshold to determine whether estimated Dual Absolute Quadric 
     * (DAQ) has rank 3 or not when validation is enabled.
     * @return threshold to determine whether estimated DAQ has rank 3 or not.
     */
    public double getDeterminantThreshold() {
        return mDeterminantThreshold;
    }
    
    /**
     * Sets threshold to determine whether estimated Dual Absolute Quadric (DAQ)
     * has rank 3 or not when validation is enabled.
     * @param determinantThreshold threshold to determine whether estimated DAQ
     * has rank 3 or not.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException if estimator is locked.
     */
    public void setDeterminantThreshold(double determinantThreshold) 
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (determinantThreshold < 0.0) {
            throw new IllegalArgumentException(
                    "threshold must be positive and greater than zero");
        }
        mDeterminantThreshold = determinantThreshold;
    }        
    
    /**
     * Indicates whether this instance is locked.
     * @return true if this estimator is busy estimating the Dual Absolute 
     * Quadric, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }
    
    /**
     * Obtains listener to be notified of events such as when estimation starts,
     * ends or estimation progress changes.
     * @return listener to be notified of events.
     */
    public DualAbsoluteQuadricEstimatorListener getListener() {
        return mListener;
    }
    
    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or estimation progress changes.
     * @param listener listener to be notified of events.
     */
    public void setListener(DualAbsoluteQuadricEstimatorListener listener) {
        mListener = listener;
    }        
    
    /**
     * Obtains the list of cameras used to estimate the Dual Absolute Quadric
     * (DAQ).
     * @return list of cameras to estimate the DAQ.
     */
    public List<PinholeCamera> getCameras() {
        return mCameras;
    }
    
    /**
     * Sets the list of cameras used to estimate the Dual Absolute Quadric 
     * (DAQ).
     * @param cameras list of cameras used to estimate the DAQ.
     * @throws IllegalArgumentException if list is null.
     * @throws LockedException if estimator is locked.
     */
    public final void setCameras(List<PinholeCamera> cameras) 
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (cameras == null ||
                cameras.size() < getMinNumberOfRequiredCameras()) {
            throw new IllegalArgumentException();
        }
        mCameras = cameras;
    }
    
    /**
     * Returns minimum number of required cameras needed to estimate the 
     * Dual Absolute Quadric (DAQ).
     * At least 8 equations are needed to solve the DAQ
     * For each imposed constraint, one less equation is required.
     * Depending on the number of constraints more or less cameras will be 
     * required.
     * If zero skewness is enforced, a solution is available with 8 cameras.
     * If zero skewness and focal distance aspect ratio is known, then a
     * solution is available with 4 cameras.
     * If principal point is located at origin, then a solution is available
     * with 4 cameras.
     * If zero skewness and principal point at origin are enforced, then a 
     * solution is available with 3 cameras.
     * If zero skewness is enforced, focal distance aspect ratio is known and
     * principal point is at origin, then a solution is available with 2 
     * cameras.
     * NOTE: minimum number of cameras considers only the cameras providing
     * additional information. If a camera is equivalent to another one or does
     * not provide additional information (such as a camera at the origin with 
     * no rotation), then more cameras will be needed.
     * @return minimum number of required cameras needed to estimate the Dual
     * Absolute Quadric (DAQ) or -1 if constraints configurations is not valid.
     */
    public int getMinNumberOfRequiredCameras() {
        int numEquations = 0;
        if (mPrincipalPointAtOrigin) {
            numEquations += 2;
            if (mZeroSkewness) {
                numEquations++;
                
                //only a linear solution exists for known aspect ratio if 
                //skewness is zero
                if (mFocalDistanceAspectRatioKnown) {
                    numEquations++;
                }                
            }
        }
        
        if (numEquations == 0) {
            return -1;
        }
        
        int minRequiredEquations = MIN_REQUIRED_EQUATIONS;
        if (mSingularityEnforced) {
            minRequiredEquations--;
        }
        return (minRequiredEquations / numEquations) +
                ((minRequiredEquations % numEquations != 0) ? 1 : 0);
    }
    
    /**
     * Indicates whether current constraints are enough to start the estimation.
     * In order to obtain a linear solution for the DAQ estimation, we need at
     * least the principal point at origin constraint.
     * @return true if constraints are valid, false otherwise.
     */
    public boolean areValidConstraints() {
        return mPrincipalPointAtOrigin && 
                (mFocalDistanceAspectRatioKnown || !mSingularityEnforced);
    }
    
    
    /**
     * Indicates if this estimator is ready to start the estimation.
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mCameras != null && 
                mCameras.size() >= getMinNumberOfRequiredCameras() &&
                areValidConstraints();
    }
    
    
    /**
     * Estimates the Dual Absolute Quadric using provided cameras.
     * @return estimated Dual Absolute Quadric (DAQ).
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if no valid input data has already been 
     * provided.
     * @throws DualAbsoluteQuadricEstimatorException if an error occurs during
     * estimation, usually because input data is not valid or numerically 
     * unstable.
     */
    public DualAbsoluteQuadric estimate() throws LockedException,
            NotReadyException, DualAbsoluteQuadricEstimatorException {
        DualAbsoluteQuadric result = new DualAbsoluteQuadric();
        estimate(result);
        return result;
    }
    
    /**
     * Estimates the Dual Absolute Quadric using provided cameras.
     * @param result instance where estimated Dual Absolute Quadric (DAQ) will
     * be stored.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if no valid input data has already been 
     * provided.
     * @throws DualAbsoluteQuadricEstimatorException if an error occurs during
     * estimation, usually because input data is not valid or numerically
     * unstable.
     */
    public abstract void estimate(DualAbsoluteQuadric result) 
            throws LockedException, NotReadyException, 
            DualAbsoluteQuadricEstimatorException;
    
        
    /**
     * Returns type of Dual Absolute Quadric estimator.
     * @return type of DAQ estimator.
     */
    public abstract DualAbsoluteQuadricEstimatorType getType();
    
    /**
     * Creates an instance of a DAQ estimator using default type.
     * @return an instance of a DAQ estimator.
     */
    public static DualAbsoluteQuadricEstimator create() {
        return create(DEFAULT_ESTIMATOR_TYPE);
    }
    
    /**
     * Creates an instance of Dual Absolute Quadric estimator using provided 
     * listener and default type.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @return an instance of a DAQ estimator.
     */
    public static DualAbsoluteQuadricEstimator create(
            DualAbsoluteQuadricEstimatorListener listener) {
        return create(listener, DEFAULT_ESTIMATOR_TYPE);
    }
    
    /**
     * Creates an instance of Dual Absolute Quadric estimator using provided
     * cameras.
     * @param cameras list of cameras used to estimate the Dual Absolute Quadric 
     * (DAQ).
     * @return an instance of a DAQ estimator.
     * @throws IllegalArgumentException if list of cameras is null or invalid
     * for default constraints.
     */
    public static DualAbsoluteQuadricEstimator create(
            List<PinholeCamera> cameras) throws IllegalArgumentException {
        return create(cameras, DEFAULT_ESTIMATOR_TYPE);
    }
    
    /**
     * Creates an instance of Dual Absolute Quadric estimator using provided
     * cameras and listener.
     * @param cameras list of cameras used to estimate the Dual Absolute Quadric
     * (DAQ).
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @return an instance of a DAQ estimator.
     * @throws IllegalArgumentException if list of cameras is null or invalid 
     * for default constraints.
     */
    public static DualAbsoluteQuadricEstimator create(
            List<PinholeCamera> cameras, 
            DualAbsoluteQuadricEstimatorListener listener) 
            throws IllegalArgumentException {
        return create(cameras, listener, DEFAULT_ESTIMATOR_TYPE);
    }
    
    /**
     * Creates an instance of a DAQ estimator using provided type.
     * @param type type of DAQ estimator.
     * @return an instance of a DAQ estimator.
     */
    public static DualAbsoluteQuadricEstimator create(
            DualAbsoluteQuadricEstimatorType type) {
        switch(type) {
            case WEIGHTED_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR:
                return new WeightedDualAbsoluteQuadricEstimator();
            case LMSE_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR:
            default:    
                return new LMSEDualAbsoluteQuadricEstimator();
        }
    }
    
    /**
     * Creates an instance of a DAQ estimator using provided listener and type.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @param type type of DAQ estimator.
     * @return an instance of a DAQ estimator.
     */
    public static DualAbsoluteQuadricEstimator create(
            DualAbsoluteQuadricEstimatorListener listener, 
            DualAbsoluteQuadricEstimatorType type) {
        switch(type) {
            case WEIGHTED_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR:
                return new WeightedDualAbsoluteQuadricEstimator(listener);
            case LMSE_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR:
            default:
                return new LMSEDualAbsoluteQuadricEstimator(listener);
        }
    }
    
    /**
     * Creates an instance of a DAQ estimator using provided cameras and type.
     * @param cameras list of cameras used to estimate the Dual Absolute Quadric
     * (DAQ).
     * @param type type of DAQ estimator.
     * @return an instance of a DAQ estimator.
     * @throws IllegalArgumentException if list of cameras is null or invalid
     * for default constraints.
     */
    public static DualAbsoluteQuadricEstimator create(
            List<PinholeCamera> cameras, DualAbsoluteQuadricEstimatorType type)
            throws IllegalArgumentException {
        switch (type) {
            case WEIGHTED_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR:
                return new WeightedDualAbsoluteQuadricEstimator(cameras);
            case LMSE_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR:
            default:
                return new LMSEDualAbsoluteQuadricEstimator(cameras);
        }
    }
    
    /**
     * Creates an instance of a DAQ estimator using provided cameras, listener
     * and type.
     * @param cameras list of cameras used to estimate the Dual Absolute Quadric
     * (DAQ).
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @param type type of DAQ estimator.
     * @return an instance of DAQ estimator.
     * @throws IllegalArgumentException if list of cameras is null or invalid
     * for default constraints.
     */
    public static DualAbsoluteQuadricEstimator create(
            List<PinholeCamera> cameras, 
            DualAbsoluteQuadricEstimatorListener listener,
            DualAbsoluteQuadricEstimatorType type) 
            throws IllegalArgumentException {
        switch (type) {
            case WEIGHTED_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR:
                return new WeightedDualAbsoluteQuadricEstimator(cameras, 
                        listener);
            case LMSE_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR:
            default:
                return new LMSEDualAbsoluteQuadricEstimator(cameras, listener);
        }
    }
    
    /**
     * Normalizes i-th row of provided matrix a.
     * @param a matrix whose row must be normalized.
     * @param i row to be normalized.
     */
    protected void normalizeRow(Matrix a, int i) {
        double rowNorm = 0.0;
        int cols = a.getColumns();
        for (int j = 0; j < cols; j++) {
            rowNorm += Math.pow(a.getElementAt(i, j), 2.0);
        }
        
        rowNorm = Math.sqrt(rowNorm);
        
        for (int j = 0; j < cols; j++) {
            a.setElementAt(i, j, a.getElementAt(i, j) / rowNorm);
        }
    }    
    
    /**
     * Fills equation p3^t*Q*p1 in provided row of matrix a.
     * @param p11 element (1,1) of camera matrix.
     * @param p31 element (3,1) of camera matrix.
     * @param p12 element (1,2) of camera matrix.
     * @param p32 element (3,2) of camera matrix.
     * @param p13 element (1,3) of camera matrix.
     * @param p33 element (3,3) of camera matrix.
     * @param p14 element (1,4) of camera matrix.
     * @param p34 element (3,4) of camera matrix.
     * @param a matrix where data is stored.
     * @param row row of matrix a where data is stored.
     */
    protected void fill3rdRowAnd1stRowEquation(
            double p11, double p31,
            double p12, double p32,
            double p13, double p33,
            double p14, double p34, Matrix a, int row) {
        
        a.setElementAt(row, 0, p31*p11); //a
        a.setElementAt(row, 1, p32*p12); //b
        a.setElementAt(row, 2, p33*p13); //c
        a.setElementAt(row, 3, p32*p11 + p31*p12); //d
        a.setElementAt(row, 4, p33*p12 + p32*p13); //e
        a.setElementAt(row, 5, p33*p11 + p31*p13); //f
        a.setElementAt(row, 6, p34*p11 + p31*p14); //g
        a.setElementAt(row, 7, p34*p12 + p32*p14); //h
        a.setElementAt(row, 8, p34*p13 + p33*p14); //i
        a.setElementAt(row, 9, p34*p14); //j
        
        normalizeRow(a, row);
    }
    
    /**
     * Fills equation p3^t*Q*p2 in provided row of matrix a.
     * @param p21 element (2,1) of camera matrix.
     * @param p31 element (3,1) of camera matrix.
     * @param p22 element (2,2) of camera matrix.
     * @param p32 element (3,2) of camera matrix.
     * @param p23 element (2,3) of camera matrix.
     * @param p33 element (3,3) of camera matrix.
     * @param p24 element (2,4) of camera matrix.
     * @param p34 element (3,4) of camera matrix.
     * @param a matrix where data is stored.
     * @param row row of matrix a where data is stored.
     */
    protected void fill3rdRowAnd2ndRowEquation(
            double p21, double p31,
            double p22, double p32,
            double p23, double p33,
            double p24, double p34, Matrix a, int row) {
        
        a.setElementAt(row, 0, p31*p21); //a
        a.setElementAt(row, 1, p32*p22); //b
        a.setElementAt(row, 2, p33*p23); //c
        a.setElementAt(row, 3, p32*p21 + p31*p22); //d
        a.setElementAt(row, 4, p33*p22 + p32*p23); //e
        a.setElementAt(row, 5, p33*p21 + p31*p23); //f
        a.setElementAt(row, 6, p34*p21 + p31*p24); //g
        a.setElementAt(row, 7, p34*p22 + p32*p24); //h
        a.setElementAt(row, 8, p34*p23 + p33*p24); //i
        a.setElementAt(row, 9, p34*p24); //j
        
        normalizeRow(a, row);
    }
    
    /**
     * Fills equation p2^t*Q*p1 in provided row of matrix a.
     * @param p11 element (1,1) of camera matrix.
     * @param p21 element (2,1) of camera matrix.
     * @param p12 element (1,2) of camera matrix.
     * @param p22 element (2,2) of camera matrix.
     * @param p13 element (1,3) of camera matrix.
     * @param p23 element (2,3) of camera matrix.
     * @param p14 element (1,4) of camera matrix.
     * @param p24 element (2,4) of camera matrix.
     * @param a matrix where data is stored.
     * @param row row of matrix a where data is stored.
     */
    protected void fill2ndRowAnd1stRowEquation(
            double p11, double p21,
            double p12, double p22,
            double p13, double p23,
            double p14, double p24, Matrix a, int row) {
        
        a.setElementAt(row, 0, p21*p11); //a
        a.setElementAt(row, 1, p22*p12); //b
        a.setElementAt(row, 2, p23*p13); //c
        a.setElementAt(row, 3, p22*p11 + p21*p12); //d
        a.setElementAt(row, 4, p23*p12 + p22*p13); //e
        a.setElementAt(row, 5, p23*p11 + p21*p13); //f
        a.setElementAt(row, 6, p24*p11 + p21*p14); //g
        a.setElementAt(row, 7, p24*p12 + p22*p14); //h
        a.setElementAt(row, 8, p24*p13 + p23*p14); //i
        a.setElementAt(row, 9, p24*p14); //j
        
        normalizeRow(a, row);
    }
    
    /**
     * Fills equation p1^t*Q*p1 = r^2*p2^t*Q*p2
     * @param p11 element (1,1) of camera matrix.
     * @param p21 element (2,1) of camera matrix.
     * @param p12 element (1,2) of camera matrix.
     * @param p22 element (2,2) of camera matrix.
     * @param p13 element (1,3) of camera matrix.
     * @param p23 element (2,3) of camera matrix.
     * @param p14 element (1,4) of camera matrix.
     * @param p24 element (2,4) of camera matrix.
     * @param a matrix where data is stored.
     * @param row row of matrix a where data is stored.
     */
    protected void fill1stRowEqualTo2ndRowEquation(
            double p11, double p21,
            double p12, double p22,
            double p13, double p23,
            double p14, double p24, Matrix a, int row) {
        
        double r2 = mFocalDistanceAspectRatio * mFocalDistanceAspectRatio;
        
        a.setElementAt(row, 0, p11*p11*r2 - p21*p21); //a
        a.setElementAt(row, 1, p12*p12*r2 - p22*p22); //b
        a.setElementAt(row, 2, p13*p13*r2 - p23*p23); //c
        a.setElementAt(row, 3, 2.0*(p12*p11*r2 - p22*p21)); //d
        a.setElementAt(row, 4, 2.0*(p13*p12*r2 - p23*p22)); //e
        a.setElementAt(row, 5, 2.0*(p13*p11*r2 - p23*p21)); //f
        a.setElementAt(row, 6, 2.0*(p14*p11*r2 - p24*p21)); //g
        a.setElementAt(row, 7, 2.0*(p14*p12*r2 - p24*p22)); //h
        a.setElementAt(row, 8, 2.0*(p14*p13*r2 - p24*p23)); //i
        a.setElementAt(row, 9, p14*p14*r2 - p24*p24); //j
        
        normalizeRow(a, row);
    }
    
    /**
     * Enforces (if needed) rank 3 of estimated quadric by building a polynomial 
     * out of the last columns of the singular value vector matrix to obtain a 
     * linear combination solution.
     * @param decomposer decomposer containing possible solutions after 
     * decomsition.
     * @param result instance where estimated Dual Absolute Quadrics (DAQs) with 
     * rank 3 enforced will be stored.
     * @throws AlgebraException if there are numerical instabilities
     * @throws NumericalException if a solution to the polynomial enforcing
     * rank 3 cannot be found.
     * @throws DualAbsoluteQuadricEstimatorException if something else fails.
     */
    protected void enforceRank3IfNeeded(
            SingularValueDecomposer decomposer, 
            DualAbsoluteQuadric result) throws AlgebraException, 
            NumericalException, DualAbsoluteQuadricEstimatorException {

        decomposer.decompose();
        
        if (mSingularityEnforced) {
            if (decomposer.getNullity() > 2) {
                //provided cameras are degenerate and there is not a single 
                //solution for the DAQ (up to scale)
                throw new DualAbsoluteQuadricEstimatorException();                
            }
            
            Matrix v = decomposer.getV();
            
            //last 2 columns of v contains parameters parameters a, b, c, d, e, 
            //f, g, h, i  that can be obtained as a linear combination of
            //those last columns (i.e. a = a1 + alpha*a2).
            //By enforcing determinant zero, we can obtain a 4th degree 
            //polynomial to determine alpha to find the unique linear 
            //combination of last 2 columns that solve both DAQ equation and 
            //generates rank 3 DAQ.
            double a1 = v.getElementAt(0, 8);
            double b1 = v.getElementAt(1, 8);
            double c1 = v.getElementAt(2, 8);
            double d1 = v.getElementAt(3, 8);
            double e1 = v.getElementAt(4, 8);
            double f1 = v.getElementAt(5, 8);
            double g1 = v.getElementAt(6, 8);
            double h1 = v.getElementAt(7, 8);
            double i1 = v.getElementAt(8, 8);
            double j1 = v.getElementAt(9, 8);
            
            double a2 = v.getElementAt(0, 9);
            double b2 = v.getElementAt(1, 9);
            double c2 = v.getElementAt(2, 9);
            double d2 = v.getElementAt(3, 9);
            double e2 = v.getElementAt(4, 9);
            double f2 = v.getElementAt(5, 9);
            double g2 = v.getElementAt(6, 9);
            double h2 = v.getElementAt(7, 9);
            double i2 = v.getElementAt(8, 9);
            double j2 = v.getElementAt(9, 9);
            
            Polynomial poly = buildPolynomialToEnforceRank3(a1, b1, c1, d1, e1, 
                    f1, g1, h1, i1, j1, a2, b2, c2, d2, e2, f2, g2, h2, i2, j2);
            
            Complex[] roots = poly.getRoots();

            if (roots != null) {
                //pick best solution (closest real root) = evaluation closest to 
                //zero
                double polyEval, bestPolyEval = Double.MAX_VALUE;
                for (Complex root : roots) {
                    double real = root.getReal();
                    polyEval = Math.abs(poly.evaluate(real));
                    if (polyEval < bestPolyEval) {
                        bestPolyEval = polyEval;
                        double a = a1 + real * a2;
                        double b = b1 + real * b2;
                        double c = c1 + real * c2;
                        double d = d1 + real * d2;
                        double e = e1 + real * e2;
                        double f = f1 + real * f2;
                        double g = g1 + real * g2;
                        double h = h1 + real * h2;
                        double i = i1 + real * i2;
                        double j = j1 + real * j2;
                        result.setParameters(a, b, c, d, e, f, g, h, i, j);
                    }
                }    
            } else {                
                //if no roots could be found, it might be due to numerical
                //inaccuracies, so we find minimum or maximum of polynomial
                //which evaluates closest to zero
                double[] extrema = poly.getExtrema();
                if (extrema == null) {
                    //polynomial has no extrema, which means it is degenerate
                    throw new DualAbsoluteQuadricEstimatorException();
                }
                
                double polyEval, bestPolyEval = Double.MAX_VALUE;
                for (double extremum : extrema) {
                    polyEval = Math.abs(poly.evaluate(extremum));
                    if (polyEval < bestPolyEval) {
                        bestPolyEval = polyEval;
                        double a = a1 + extremum * a2;
                        double b = b1 + extremum * b2;
                        double c = c1 + extremum * c2;
                        double d = d1 + extremum * d2;
                        double e = e1 + extremum * e2;
                        double f = f1 + extremum * f2;
                        double g = g1 + extremum * g2;
                        double h = h1 + extremum * h2;
                        double i = i1 + extremum * i2;
                        double j = j1 + extremum * j2;
                        result.setParameters(a, b, c, d, e, f, g, h, i, j);                        
                    }
                }
                
                if (mValidateEnforcedSingularity) {
                    //check that determinant of estimated DAQ is below allowed 
                    //threshold
                    double absDet = Math.abs(Utils.det(result.asMatrix()));
                    if (absDet > mDeterminantThreshold) {
                        //DAQ does not have rank 3
                        throw new DualAbsoluteQuadricEstimatorException();
                    }
                }
            }
                        
        } else {
            if (decomposer.getNullity() > 1) {
                //provided cameras are degenerate and there is not a single 
                //solution for the DAQ (up to scale)
                throw new DualAbsoluteQuadricEstimatorException();
            }
            
            Matrix v = decomposer.getV();
            
            //last column of v contains parameters a, b, c, d, e, f, g, h, i 
            //defining the Dual Absolute Quadric (DAQ) as:
            double a = v.getElementAt(0, 9);
            double b = v.getElementAt(1, 9);
            double c = v.getElementAt(2, 9);
            double d = v.getElementAt(3, 9);
            double e = v.getElementAt(4, 9);
            double f = v.getElementAt(5, 9);
            double g = v.getElementAt(6, 9);
            double h = v.getElementAt(7, 9);
            double i = v.getElementAt(8, 9);
            double j = v.getElementAt(9, 9);
                
            result.setParameters(a, b, c, d, e, f, g, h, i, j);
        }
    }
    
    /**
     * Builds polynomial to enforce rank 3 (zero determinant).
     * @param a1 a of 1st DAQ.
     * @param b1 b of 1st DAQ.
     * @param c1 c of 1st DAQ.
     * @param d1 d of 1st DAQ.
     * @param e1 e of 1st DAQ.
     * @param f1 f of 1st DAQ.
     * @param g1 g of 1st DAQ.
     * @param h1 h of 1st DAQ.
     * @param i1 i of 1st DAQ.
     * @param j1 j of 1st DAQ.
     * @param a2 a of 2nd DAQ.
     * @param b2 b of 2nd DAQ.
     * @param c2 c of 2nd DAQ.
     * @param d2 d of 2nd DAQ.
     * @param e2 e of 2nd DAQ.
     * @param f2 f of 2nd DAQ.
     * @param g2 g of 2nd DAQ.
     * @param h2 h of 2nd DAQ.
     * @param i2 i of 2nd DAQ.
     * @param j2 j of 2nd DAQ.
     * @return polynomial.
     */
    private Polynomial buildPolynomialToEnforceRank3(double a1, double b1, 
            double c1, double d1, double e1, double f1, double g1, double h1, 
            double i1, double j1, double a2, double b2, double c2, double d2, 
            double e2, double f2, double g2, double h2, double i2, double j2) {
        
        Polynomial polyA = new Polynomial(a1, a2);
        Polynomial polyB = new Polynomial(b1, b2);
        Polynomial polyC = new Polynomial(c1, c2);
        Polynomial polyD = new Polynomial(d1, d2);
        Polynomial polyE = new Polynomial(e1, e2);
        Polynomial polyF = new Polynomial(f1, f2);
        Polynomial polyG = new Polynomial(g1, g2);
        Polynomial polyH = new Polynomial(h1, h2);
        Polynomial polyI = new Polynomial(i1, i2);
        Polynomial polyJ = new Polynomial(j1, j2);
        
        Polynomial result = new Polynomial(5);
        
        //(a1 + x*a2) * (b1 + x*b2) * (c1 + x*c2) * (j1 + x*j2)
        Polynomial tmp = polyA.multiplyAndReturnNew(polyB);
        tmp.multiply(polyC);
        tmp.multiply(polyJ);
        
        result.add(tmp);
        
        //2 * (a1 + x*a2) * (e1 + x*e2) * (h1 + x*h2) * (i1 + x*i2)
        tmp = polyA.multiplyAndReturnNew(polyE);
        tmp.multiply(polyH);
        tmp.multiply(polyI);
        tmp.multiplyByScalar(2.0);
        
        result.add(tmp);
        
        //-1 * (a1 + x*a2) * (c1 + x*c2) * (h1 + x*h2)^2
        tmp = polyA.multiplyAndReturnNew(polyC);
        tmp.multiply(polyH);
        tmp.multiply(polyH);
        tmp.multiplyByScalar(-1.0);
        
        result.add(tmp);
        
        //-1 * (a1 + x*a2) * (b1 + x*b2) * (i1 + x*i2)^2
        tmp = polyA.multiplyAndReturnNew(polyB);
        tmp.multiply(polyI);
        tmp.multiply(polyI);
        tmp.multiplyByScalar(-1.0);
        
        result.add(tmp);
        
        //-1 * (a1 + x*a2) * (e1 + x*e2)^2 * (j1 + x*j2)
        tmp = polyA.multiplyAndReturnNew(polyE);
        tmp.multiply(polyE);
        tmp.multiply(polyJ);
        tmp.multiplyByScalar(-1.0);
        
        result.add(tmp);
        
        //-1 * (c1 + x*c2) * (d1 + x*d2)^2 * (j1 + x*j2)
        tmp = polyC.multiplyAndReturnNew(polyD);
        tmp.multiply(polyD);
        tmp.multiply(polyJ);
        tmp.multiplyByScalar(-1.0);
        
        result.add(tmp);
        
        //-2 * (d1 + x*d2) * (f1 + x*f2) * (h1 + x*h2) * (i1 + x*i2)
        tmp = polyD.multiplyAndReturnNew(polyF);
        tmp.multiply(polyH);
        tmp.multiply(polyI);
        tmp.multiplyByScalar(-2.0);
        
        result.add(tmp);
        
        //-2 * (d1 + x*d2) * (e1 + x*e2) * (g1 + x*g2) * (i1 + x*i2)
        tmp = polyD.multiplyAndReturnNew(polyE);
        tmp.multiply(polyG);
        tmp.multiply(polyI);
        tmp.multiplyByScalar(-2.0);
        
        result.add(tmp);
        
        //2 * (c1 + x*c2) * (d1 + x*d2) * (g1 + x*g2) * (h1 + x*h2)
        tmp = polyC.multiplyAndReturnNew(polyD);
        tmp.multiply(polyG);
        tmp.multiply(polyH);
        tmp.multiplyByScalar(2.0);
        
        result.add(tmp);
        
        //(d1 + x*d2)^2 * (i1 + x*i2)^2
        tmp = polyD.multiplyAndReturnNew(polyD);
        tmp.multiply(polyI);
        tmp.multiply(polyI);
        
        result.add(tmp);
        
        //2 * (d1 + x*d2) * (e1 + x*e2) * (f1 + x*f2) * (j1 + x*j2)
        tmp = polyD.multiplyAndReturnNew(polyE);
        tmp.multiply(polyF);
        tmp.multiply(polyJ);
        tmp.multiplyByScalar(2.0);
        
        result.add(tmp);
        
        //(f1 + x*f2)^2 * (h1 + x*h2)^2
        tmp = polyF.multiplyAndReturnNew(polyF);
        tmp.multiply(polyH);
        tmp.multiply(polyH);
        
        result.add(tmp);
        
        //2 * (b1 + x*b2) * (f1 + x*f2) * (g1 + x*g2) * (i1 + x*i2)
        tmp = polyB.multiplyAndReturnNew(polyF);
        tmp.multiply(polyG);
        tmp.multiply(polyI);
        tmp.multiplyByScalar(2.0);
        
        result.add(tmp);
        
        //-2 * (e1 + x*e2) * (f1 + x*r2) * (g1 + x*g2) * (h1 + x*h2)
        tmp = polyE.multiplyAndReturnNew(polyF);
        tmp.multiply(polyG);
        tmp.multiply(polyH);
        tmp.multiplyByScalar(-2.0);
        
        result.add(tmp);
        
        //-1 * (b1 + x*b2) * (f1 + x*f2)^2 * (j1 + x*j2)
        tmp = polyB.multiplyAndReturnNew(polyF);
        tmp.multiply(polyF);
        tmp.multiply(polyJ);
        tmp.multiplyByScalar(-1.0);
        
        result.add(tmp);
        
        //-1 * (b1 + x*b2) * (c1 + x*c2) * (g1 + x*g2)^2
        tmp = polyB.multiplyAndReturnNew(polyC);
        tmp.multiply(polyG);
        tmp.multiply(polyG);
        tmp.multiplyByScalar(-1.0);
        
        result.add(tmp);
        
        //(e1 + x*e2)^2 * (g1 + x*g2)^2 
        tmp = polyE.multiplyAndReturnNew(polyE);
        tmp.multiply(polyG);
        tmp.multiply(polyG);
        
        result.add(tmp);
               
        return result;
    }
}
