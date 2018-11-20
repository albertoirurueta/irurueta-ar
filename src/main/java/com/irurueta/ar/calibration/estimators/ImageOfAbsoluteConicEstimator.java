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

import java.util.Arrays;
import java.util.List;

/**
 * This class defines the interface for an estimator of the Image of Absolute
 * Conic (IAC). The IAC is directly related to pinhole camera intrinsic 
 * parameters, implementations of this estimator can be use for camera 
 * calibration purposes.
 * Depending on the contraints imposed on the intrinsic parameters of the 
 * camera, some parameters of the IAC will be known in advance and less 
 * equations will be needed.
 * If no contraints are imposed, then at least 5 equations will be needed to
 * estimate the DIAC, and hence 3 homographies will be required (2 equations can
 * be obtained for each homography). 2*n &lt;= 5 ==&lt; n &lt;= 5/2 ==&lt; n &lt;= 2.5 ==&lt;
 * n &lt;= 3
 * If skewness is assumed to be known and zero, then one less homography is 
 * required, hence only 2 homographies will be needed. 
 * 2*n + 1 &lt;= 5 ==&lt; n &lt;= 4 / 2 ==&lt; n &lt;= 2
 * If also principal point is assumed to be known and located at image center
 * (origin of coordinates), then 2 less equations are needed, and hence only
 * 1 homography is needed:
 * 2n + 1 + 2 &lt;= 5 ==&lt; n &lt;= 2 / 2 ==&lt; n &lt;= 1
 * By enabling constraints, results are guaranteed to be more stable (estimation
 * fails less times with an exception) and more accurate, so whenever possible
 * constraints should be enabled. The principal point at origin introduces more
 * stability and accuracy than the skewness constraint, but if possible both 
 * should be enabled. By default both constraints are enable.
 */
@SuppressWarnings("WeakerAccess")
public abstract class ImageOfAbsoluteConicEstimator {
    
    /**
     * Minimum number of required equations to solve the IAC linear system of
     * equations.
     */
    protected static final int MIN_REQUIRED_EQUATIONS = 5;
        
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
     * Constant defining whether aspect ratio of focal distances (i.e. vertical
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
     * Minimum absolute value allowed for aspect ratio of focal distances.
     */
    public static final double MIN_ABS_FOCAL_DISTANCE_ASPECT_RATIO = 1e-6;
            
    /**
     * Default estimator type.
     */
    public static final ImageOfAbsoluteConicEstimatorType DEFAULT_ESTIMATOR_TYPE =
            ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR;
    
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
     * This value is only taken into account if skewness is assumed to be
     * zero, otherwise it is ignored.
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
     * True when estimator is estimating IAC.
     */
    protected boolean mLocked;
    
    /**
     * Listener to be notified of events such as when estimation starts, ends or
     * estimation progress changes.
     */
    protected ImageOfAbsoluteConicEstimatorListener mListener;
    
    /**
     * List of homographies (2D transformations) used to estimate the image
     * of absolute conic (IAC), which can be used to obtain pinhole camera
     * intrinsic parameters.
     */
    protected List<Transformation2D> mHomographies;
    
    /**
     * Constructor.
     */
    public ImageOfAbsoluteConicEstimator() {
        mZeroSkewness = DEFAULT_ZERO_SKEWNESS;        
        mPrincipalPointAtOrigin = DEFAULT_PRINCIPAL_POINT_AT_ORIGIN;
        mFocalDistanceAspectRatioKnown = 
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN;
        mFocalDistanceAspectRatio = DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO;
        
        mLocked = false;
        mListener = null;
        mHomographies = null;
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public ImageOfAbsoluteConicEstimator(
            ImageOfAbsoluteConicEstimatorListener listener) {
        this();
        mListener = listener;
    }
    
    /**
     * Constructor.
     * @param homographies list of homographies (2D transformations) used to
     * estimate the image of absolute conic (IAC), which can be used to obtain
     * pinhole camera intrinsic parameters.
     * @throws IllegalArgumentException if not enough homographies are provided
     * for default settings. Hence, at least 1 homography must be provided.
     */
    public ImageOfAbsoluteConicEstimator(List<Transformation2D> homographies)
            throws IllegalArgumentException {
        this();
        internalSetHomographies(homographies);
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
    public ImageOfAbsoluteConicEstimator(List<Transformation2D> homographies,
            ImageOfAbsoluteConicEstimatorListener listener) 
            throws IllegalArgumentException {
        this(listener);
        internalSetHomographies(homographies);
    }
    
    /**
     * Returns boolean indicating whether camera skewness is assumed to be zero
     * or not.
     * Skewness determines whether LCD sensor cells are properly aligned or not,
     * where zero indicates perfect alignment.
     * Typically skewness is a value equal or very close to zero.
     * @return true if camera skewness is assumed to be zero, otherwise camera
     * skewness is estimated.
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
     * otherwise camera skewness is estimated.
     * @throws LockedException if estimator is locked.
     */
    public void setZeroSkewness(boolean zeroSkewness) throws LockedException {
        if (isLocked()) {
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
     * coordinates, false if principal point must be estimated.
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
     * origin of coordinates, false if principal point must be estimated.
     * @throws LockedException if estimator is locked.
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
     * @return true if focal distance aspect ratio is known, false otherwise.
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
     * aspect ratio
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
     * @param focalDistanceAspectRatio aspect ratio of focal distances to be 
     * set.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if focal distance aspect ratio is too 
     * close to zero, as it might produce numerical instabilities.
     */
    public void setFocalDistanceAspectRatio(double focalDistanceAspectRatio)
            throws LockedException, IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (Math.abs(focalDistanceAspectRatio) < MIN_ABS_FOCAL_DISTANCE_ASPECT_RATIO) {
            throw new IllegalArgumentException();
        }

        mFocalDistanceAspectRatio = focalDistanceAspectRatio;
    }    
    
    /**
     * Indicates whether this instance is locked.
     * @return true if this estimator is busy estimating a IAC, false
     * otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }

    /**
     * Returns listener to be notified of events such as when estimation starts,
     * ends or estimation progress changes.
     * @return listener to be notified of events.
     */
    public ImageOfAbsoluteConicEstimatorListener getListener() {
        return mListener;
    }
    
    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or estimation progress changes.
     * @param listener listener to be notified of events.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(ImageOfAbsoluteConicEstimatorListener listener)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mListener = listener;
    }
    
    /**
     * Gets list of homographies to estimate IAC.
     * @return list of homographies to estimate IAC.
     */
    public List<Transformation2D> getHomographies() {
        return mHomographies;
    }
    
    /**
     * Sets list of homographies to estimate IAC.
     * @param homographies list of homographies to estimate IAC.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if provided list of homographies does not
     * contain enough elements to estimate the IAC using current settings.
     */
    public void setHomographies(List<Transformation2D> homographies)
            throws LockedException, IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetHomographies(homographies);
    }
    
    /**
     * Returns minimum number of required homographies needed to estimate the
     * Image of Absolute Conic (IAC).
     * If no constraints are imposed, then at least 3 homographies are required.
     * For each contraint imposed, one less equation will be required,  hence 
     * if skewness is assumed to be known (by using its typical value of zero), 
     * then only 4 equations will be needed (2 homographies).
     * If also the horizontal and vertical coordinates of the principal point 
     * are assumed to be known (by being placed at the 2D origin of coordinates, 
     * which is a typical value), then only 2 equations will be needed 
     * (1 homography). 
     * @return minimum number of required homographies required to estimate the
     * IAC
     */
    public int getMinNumberOfRequiredHomographies() {        
        int numEquations = MIN_REQUIRED_EQUATIONS;
        if (mZeroSkewness) {
            if (mFocalDistanceAspectRatioKnown) {
                numEquations--;
            }
            numEquations--;
        }
        if (mPrincipalPointAtOrigin) {
            numEquations -= 2;
        }
        
        return (numEquations / 2) + (numEquations % 2);
    }
    
    /**
     * Returns value indicating whether required data has been provided so that
     * IAC estimation can start.
     * If true, estimator is ready to compute the IAC, otherwise more data needs
     * to be provided.
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mHomographies != null && 
                mHomographies.size() >= getMinNumberOfRequiredHomographies();
    }
    
    /**
     * Estimates Image of Absolute Conic (IAC).
     * @return estimated IAC.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if input has not yet been provided.
     * @throws ImageOfAbsoluteConicEstimatorException if an error occurs during
     * estimation, usually because repeated homographies are provided, or 
     * homographies corresponding to degenerate camera movements such as pure
     * parallel translations where no additional data is really provided. 
     * Indeed, if provided homographies belong to the group of affine 
     * transformations (or other groups contained within such as metric or 
     * euclidean ones), this exception will raise because camera movements will
     * be degenerate. To avoid this exception, homographies must be purely
     * projective.
     */
    public abstract ImageOfAbsoluteConic estimate() throws LockedException,
            NotReadyException, ImageOfAbsoluteConicEstimatorException;
    
    /**
     * Returns type of IAC estimator.
     * @return type of IAC estimator.
     */
    public abstract ImageOfAbsoluteConicEstimatorType getType();
    
    /**
     * Creates an instance of an IAC estimator using provided default type.
     * @return an instance of an IAC estimator.
     */
    public static ImageOfAbsoluteConicEstimator create() {
        return create(DEFAULT_ESTIMATOR_TYPE);
    }
    
    /**
     * Creates an instance of an IAC estimator using provided listener and
     * default type.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @return an instance of an IAC estimator.
     */
    public static ImageOfAbsoluteConicEstimator create(
            ImageOfAbsoluteConicEstimatorListener listener) {
        return create(listener, DEFAULT_ESTIMATOR_TYPE);
    }
    
    /**
     * Creates an instance of an IAC estimator using provided homographies
     * @param homographies list of homographies (2D transformations) used to
     * estimate the image of absolute conic (IAC), which can be used to obtain
     * pinhole camera intrinsic parameters.
     * @return an instance of an IAC estimator.
     * @throws IllegalArgumentException if not enough homographies are provided
     * for default IAC estimation constraints.
     */
    public static ImageOfAbsoluteConicEstimator create(
            List<Transformation2D> homographies) 
            throws IllegalArgumentException {
        return create(homographies, DEFAULT_ESTIMATOR_TYPE);
    }

    /**
     * Creates an instance of an IAC estimator using provided homographies and 
     * listener.
     * @param homographies list of homographies (2D transformations) used to
     * estimate the image of absolute conic (IAC), which can be used to obtain
     * pinhole camera intrinsic parameters.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @return an instance of an IAC estimator.
     * @throws IllegalArgumentException if not enough homographies are provided
     * for default IAC estimation constraints.
     */    
    public static ImageOfAbsoluteConicEstimator create(
            List<Transformation2D> homographies,
            ImageOfAbsoluteConicEstimatorListener listener) 
            throws IllegalArgumentException {
        return create(homographies, listener, DEFAULT_ESTIMATOR_TYPE);
    }
    
    
    /**
     * Creates an instance of an IAC estimator using provided type.
     * @param type type of IAC estimator to create.
     * @return an instance of an IAC estimator.
     */
    public static ImageOfAbsoluteConicEstimator create(
            ImageOfAbsoluteConicEstimatorType type) {
        switch (type) {
            case WEIGHTED_IAC_ESTIMATOR:
                return new WeightedImageOfAbsoluteConicEstimator();
            case LMSE_IAC_ESTIMATOR:
            default:
                return new LMSEImageOfAbsoluteConicEstimator();
        }
    }
    
    /**
     * Creates an instance of an IAC estimator using provided listener and
     * type.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @param type type of IAC estimator to create.
     * @return an instance of an IAC estimator.
     */
    public static ImageOfAbsoluteConicEstimator create(
            ImageOfAbsoluteConicEstimatorListener listener, 
            ImageOfAbsoluteConicEstimatorType type) {
        switch (type) {
            case WEIGHTED_IAC_ESTIMATOR:
                return new WeightedImageOfAbsoluteConicEstimator(listener);
            case LMSE_IAC_ESTIMATOR:
            default:
                return new LMSEImageOfAbsoluteConicEstimator(listener);
        }
    }
    
    /**
     * Creates an instance of an IAC estimator using provided homographies
     * and type.
     * @param homographies list of homographies (2D transformations) used to
     * estimate the image of absolute conic (IAC), which can be used to obtain
     * pinhole camera intrinsic parameters.
     * @param type type of IAC estimator to create.
     * @return an instance of an IAC estimator.
     * @throws IllegalArgumentException if not enough homographies are provided
     * for default IAC estimation constraints.
     */            
    public static ImageOfAbsoluteConicEstimator create(
            List<Transformation2D> homographies, 
            ImageOfAbsoluteConicEstimatorType type) 
            throws IllegalArgumentException {
        switch (type) {
            case WEIGHTED_IAC_ESTIMATOR:
                double[] weights = new double[homographies.size()];
                Arrays.fill(weights, 1.0);
                return new WeightedImageOfAbsoluteConicEstimator(homographies, 
                        weights);
            case LMSE_IAC_ESTIMATOR:
            default:
                return new LMSEImageOfAbsoluteConicEstimator(homographies);
        }
    }
    
    /**
     * Creates an instance of an IAC estimator using provided homographies,
     * listener and type.
     * @param homographies list of homographies (2D transformations) used to
     * estimate the image of absolute conic (IAC), which can be used to obtain
     * pinhole camera intrinsic parameters.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @param type type of IAC estimator to create.
     * @return an instance of an IAC estimator.
     * @throws IllegalArgumentException if not enough homographies are provided
     * for default IAC estimation constraints.
     */        
    public static ImageOfAbsoluteConicEstimator create(
            List<Transformation2D> homographies,
            ImageOfAbsoluteConicEstimatorListener listener,
            ImageOfAbsoluteConicEstimatorType type) 
            throws IllegalArgumentException {
        switch (type) {
            case WEIGHTED_IAC_ESTIMATOR:
                double[] weights = new double[homographies.size()];
                Arrays.fill(weights, 1.0);
                return new WeightedImageOfAbsoluteConicEstimator(homographies, 
                        weights, listener);
            case LMSE_IAC_ESTIMATOR:
            default:
                return new LMSEImageOfAbsoluteConicEstimator(homographies, 
                        listener);
        }
    }
    
    /**
     * Sets list of homographies to estimate IAC.
     * This method does not check whether estimator is locked.
     * @param homographies list of homographies to estimate IAC.
     * @throws IllegalArgumentException if provided list of homographies does not
     * contain enough elements to estimate the DIAC using current settings.
     */
    protected final void internalSetHomographies(
            List<Transformation2D> homographies)
            throws IllegalArgumentException {
        if (homographies == null ||
                homographies.size() < getMinNumberOfRequiredHomographies()) {
            throw new IllegalArgumentException();
        }
        mHomographies = homographies;
    }
}
