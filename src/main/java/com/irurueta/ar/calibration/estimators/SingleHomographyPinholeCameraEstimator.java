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
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.geometry.Transformation2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

import java.util.ArrayList;

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
    private double focalDistanceAspectRatio = DEFAULT_ASPECT_RATIO;

    /**
     * Homography relating two views.
     */
    private Transformation2D homography;

    /**
     * True when estimation is in progress.
     */
    private boolean locked = false;

    /**
     * Listener to be notified of events such as when estimation starts or ends.
     */
    private SingleHomographyPinholeCameraEstimatorListener listener;

    /**
     * Constructor.
     */
    public SingleHomographyPinholeCameraEstimator() {
    }

    /**
     * Constructor with listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts or ends.
     */
    public SingleHomographyPinholeCameraEstimator(final SingleHomographyPinholeCameraEstimatorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor with homography.
     *
     * @param homography homography to estimate camera and intrinsic parameters.
     * @throws NullPointerException if provided homography is null.
     */
    public SingleHomographyPinholeCameraEstimator(final Transformation2D homography) {
        internalSetHomography(homography);
    }

    /**
     * Constructor with listener and homography.
     *
     * @param homography homography to estimate camera and intrinsic parameters.
     * @param listener   listener to be notified of events such as when estimation
     *                   starts or ends.
     */
    public SingleHomographyPinholeCameraEstimator(
            final Transformation2D homography, final SingleHomographyPinholeCameraEstimatorListener listener) {
        internalSetHomography(homography);
        this.listener = listener;
    }

    /**
     * Gets aspect ratio of intrinsic parameters.
     *
     * @return aspect ratio of intrinsic parameters.
     */
    public double getFocalDistanceAspectRatio() {
        return focalDistanceAspectRatio;
    }

    /**
     * Sets aspect ratio of intrinsic parameters.
     *
     * @param focalDistanceAspectRatio aspect ratio of intrinsic parameters.
     * @throws LockedException if estimator is locked.
     */
    public void setFocalDistanceAspectRatio(final double focalDistanceAspectRatio) throws LockedException {
        if (locked) {
            throw new LockedException();
        }
        this.focalDistanceAspectRatio = focalDistanceAspectRatio;
    }

    /**
     * Gets homography relating two views.
     *
     * @return homography relating two views.
     */
    public Transformation2D getHomography() {
        return homography;
    }

    /**
     * Sets homography relating two views.
     *
     * @param homography homography relating two views.
     * @throws LockedException if estimator is locked.
     */
    public void setHomography(final Transformation2D homography) throws LockedException {
        if (locked) {
            throw new LockedException();
        }
        this.homography = homography;
    }

    /**
     * Indicates whether this instance is locked.
     *
     * @return true if this estimator is busy doing the estimation, false
     * otherwise.
     */
    public boolean isReady() {
        return homography != null;
    }

    /**
     * Gets listener to be notified of events such as when estimation starts or
     * ends.
     *
     * @return listener to be notified of events.
     */
    public SingleHomographyPinholeCameraEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to be notified of events such as when estimation starts or
     * ends.
     *
     * @param listener listener to be notified of events.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(final SingleHomographyPinholeCameraEstimatorListener listener) throws LockedException {
        if (locked) {
            throw new LockedException();
        }
        this.listener = listener;
    }

    /**
     * Indicates whether this instance is locked or not.
     *
     * @return true if instance is locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Estimates a pinhole camera.
     *
     * @return estimated pinhole camera.
     * @throws LockedException                                 if estimator is locked.
     * @throws NotReadyException                               if no homography has been provided yet.
     * @throws SingleHomographyPinholeCameraEstimatorException if estimation
     *                                                         fails.
     */
    public PinholeCamera estimate() throws LockedException, NotReadyException,
            SingleHomographyPinholeCameraEstimatorException {
        final var result = new PinholeCamera();
        estimate(result);
        return result;
    }

    /**
     * Estimates a pinhole camera.
     *
     * @param result instance where estimated camera will be stored.
     * @throws LockedException                                 if estimator is locked.
     * @throws NotReadyException                               if no homography has been provided yet.
     * @throws SingleHomographyPinholeCameraEstimatorException if estimation
     *                                                         fails.
     */
    public void estimate(PinholeCamera result) throws LockedException, NotReadyException,
            SingleHomographyPinholeCameraEstimatorException {
        if (locked) {
            throw new LockedException();
        }

        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            locked = true;

            if (listener != null) {
                listener.onEstimateStart(this);
            }

            // estimate intrinsic parameters
            final var homographies = new ArrayList<Transformation2D>();
            homographies.add(homography);
            // also add its inverse
            homographies.add(((ProjectiveTransformation2D) homography).inverseAndReturnNew());
            final var intrinsicEstimator = new LMSEImageOfAbsoluteConicEstimator(homographies);
            intrinsicEstimator.setPrincipalPointAtOrigin(true);
            intrinsicEstimator.setFocalDistanceAspectRatioKnown(true);
            intrinsicEstimator.setFocalDistanceAspectRatio(focalDistanceAspectRatio);
            ImageOfAbsoluteConic iac = intrinsicEstimator.estimate();
            iac.normalize();

            final var intrinsic = iac.getIntrinsicParameters();

            // estimate camera pose
            final var rotation = new MatrixRotation3D();
            final var center = new HomogeneousPoint3D();
            CameraPoseEstimator.estimate(intrinsic, homography, rotation, center, result);

            if (listener != null) {
                listener.onEstimateEnd(this);
            }

        } catch (final Exception e) {
            throw new SingleHomographyPinholeCameraEstimatorException(e);
        } finally {
            locked = false;
        }
    }

    /**
     * Sets homography to estimate camera and intrinsic parameters.
     *
     * @param homography homography to be set.
     * @throws NullPointerException if provided homography is null.
     */
    private void internalSetHomography(final Transformation2D homography) {
        if (homography == null) {
            throw new NullPointerException();
        }
        this.homography = homography;
    }
}
