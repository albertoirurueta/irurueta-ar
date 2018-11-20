/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

import java.util.ArrayList;
import java.util.List;

/**
 * This class estimate intrinsic and extrinsic (rotation and camera center) 
 * parameters of a camera by using provided homography.
 * For intrinsic parameters estimation it is assumed that skewness and principal
 * point are zero and that aspect ratio are known.
 * This class can be used in planar scenes where projected points of two views
 * are related by an homography.
 */
public class SingleHomographyPinholeCameraEstimator {
    
    /**
     * Default aspect ratio value.
     */
    public static final double DEFAULT_ASPECT_RATIO = 1.0;
    
    /**
     * Aspect ratio of intrinsic parameters.
     */
    private double mFocalDistanceAspectRatio = DEFAULT_ASPECT_RATIO;
    
    /**
     * Homography relating two views.
     */
    private Transformation2D mHomography;
    
    /**
     * True when estimation is in progress.
     */
    private boolean mLocked = false;
    
    /**
     * Listener to be notified of events such as when estimation starts or ends.
     */
    private SingleHomographyPinholeCameraEstimatorListener mListener;
    
    /**
     * Constructor.
     */
    public SingleHomographyPinholeCameraEstimator() { }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts or ends.
     */
    public SingleHomographyPinholeCameraEstimator(
            SingleHomographyPinholeCameraEstimatorListener listener) {
        mListener = listener;
    }
    
    /**
     * Constructor with homography.
     * @param homography homography to estimate canera and intrinsic parameters.
     * @throws NullPointerException if provided homography is null.
     */
    public SingleHomographyPinholeCameraEstimator(Transformation2D homography) 
            throws NullPointerException {
        internalSetHomography(homography);
    }
    
    /**
     * Constructor with listener and homography.
     * @param homography homography to estimate camera and intrinsic parameters.
     * @param listener listener to be notified of events such as when estimation
     * starts or ends.
     */
    public SingleHomographyPinholeCameraEstimator(Transformation2D homography,
            SingleHomographyPinholeCameraEstimatorListener listener) {
        internalSetHomography(homography);
        mListener = listener;
    }
    
    /**
     * Gets aspect ratio of intrinsic parameters.
     * @return aspect ratio of intrinsic parameters.
     */
    public double getFocalDistanceAspectRatio() {
        return mFocalDistanceAspectRatio;
    }
    
    /**
     * Sets aspect ratio of intrinsic parameters.
     * @param focalDistanceAspectRatio aspect ratio of intrinsic parameters.
     * @throws LockedException if estimator is locked.
     */
    public void setFocalDistanceAspectRatio(double focalDistanceAspectRatio)
            throws LockedException {
        if (mLocked) {
            throw new LockedException();
        }
        mFocalDistanceAspectRatio = focalDistanceAspectRatio;
    }
    
    /**
     * Gets homography relating two views.
     * @return homography relating two views.
     */
    public Transformation2D getHomography() {
        return mHomography;
    }
    
    /**
     * Sets homography relating two views.
     * @param homography homography relating two views.
     * @throws LockedException if estimator is locked.
     */
    public void setHomography(Transformation2D homography) 
            throws LockedException {
        if (mLocked) {
            throw new LockedException();
        }
        mHomography = homography;
    }
            
    /**
     * Indicates whether this instance is locked.
     * @return true if this estimator is budy doing the estimation, false 
     * otherwise.
     */
    public boolean isReady() {
        return mHomography != null;
    }
    
    /**
     * Gets listener to be notified of events such as when estimation starts or
     * ends.
     * @return listener to be notified of events.
     */
    public SingleHomographyPinholeCameraEstimatorListener getListener() {
        return mListener;
    }
    
    /**
     * Sets listener to be notified of events such as when estimation starts or
     * ends.
     * @param listener listener to be notified of events.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(
            SingleHomographyPinholeCameraEstimatorListener listener) 
            throws LockedException {
        if (mLocked) {
            throw new LockedException();
        }
        mListener = listener;
    }
    
    /**
     * Indicates whether this instance is locked or not.
     * @return true if instance is locked, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }
    
    /**
     * Estimates a pinhole camera.
     * @return estimated pinhole camera.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if no homography has been provided yet.
     * @throws SingleHomographyPinholeCameraEstimatorException if estimation 
     * fails.
     */
    public PinholeCamera estimate() throws LockedException, NotReadyException,
            SingleHomographyPinholeCameraEstimatorException {
        PinholeCamera result = new PinholeCamera();
        estimate(result);
        return result;
    }
    
    /**
     * Estimates a pinhole camera.
     * @param result instance where estimated camera will be stored.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if no homography has been provided yet.
     * @throws SingleHomographyPinholeCameraEstimatorException if estimation 
     * fails.
     */
    public void estimate(PinholeCamera result) throws LockedException, 
            NotReadyException, SingleHomographyPinholeCameraEstimatorException {
        if (mLocked) {
            throw new LockedException();
        }
        
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        try {
            mLocked = true;
            
            if (mListener != null) {
                mListener.onEstimateStart(this);
            }
            
            //estimate intrinsic parameters
            List<Transformation2D> homographies = 
                    new ArrayList<>();
            homographies.add(mHomography);
            //also add its inverse
            homographies.add(
                    ((ProjectiveTransformation2D)mHomography).inverseAndReturnNew());
            LMSEImageOfAbsoluteConicEstimator intrinsicEstimator = 
                    new LMSEImageOfAbsoluteConicEstimator(homographies);
            intrinsicEstimator.setPrincipalPointAtOrigin(true);
            intrinsicEstimator.setFocalDistanceAspectRatioKnown(true);
            intrinsicEstimator.setFocalDistanceAspectRatio(
                    mFocalDistanceAspectRatio);
            ImageOfAbsoluteConic iac = intrinsicEstimator.estimate();
            iac.normalize();
            
            PinholeCameraIntrinsicParameters intrinsic = 
                    iac.getIntrinsicParameters();
        
            //estimate camera pose
            MatrixRotation3D rotation = new MatrixRotation3D();
            HomogeneousPoint3D center = new HomogeneousPoint3D();
            CameraPoseEstimator.estimate(intrinsic, mHomography, rotation, 
                    center, result);
            
            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }
            
        } catch (Exception e) {
            throw new SingleHomographyPinholeCameraEstimatorException(e);
        } finally {
            mLocked = false;
        }
    }
    
    /**
     * Sets homography to estimate camera and intrinsic parameters.
     * @param homography homography to be set.
     * @throws NullPointerException if provided homography is null.
     */
    private void internalSetHomography(Transformation2D homography) 
            throws NullPointerException {
        if (homography == null) {
            throw new NullPointerException();
        }
        mHomography = homography;
    }
}
