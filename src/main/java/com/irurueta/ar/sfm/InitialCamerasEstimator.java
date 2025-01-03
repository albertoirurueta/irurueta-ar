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
package com.irurueta.ar.sfm;

import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

/**
 * Estimates initial cameras to initialize geometry in a metric stratum.
 * This class uses provided fundamental matrix in order to obtain a pair of
 * cameras and upgrade such cameras into a metric stratum.
 * This class assumes that principal point of intrinsic camera parameters are
 * located at the origin of coordinates, and also that skewness of such
 * intrinsic parameters is zero.
 * This class can upgrade cameras to a metric stratum by either estimating the
 * Dual Absolute Quadric and its corresponding projective to metric
 * transformation, or by solving the Kruppa equations to estimate the Dual
 * Image of Absolute Conic to determine intrinsic parameters and then compute
 * the essential matrix and use 2D point matches to triangulate them and find
 * the initial cameras.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class InitialCamerasEstimator {

    /**
     * Default method.
     */
    public static final InitialCamerasEstimatorMethod DEFAULT_METHOD =
            InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC;

    /**
     * Fundamental matrix relating two views whose cameras need to be estimated.
     */
    protected FundamentalMatrix fundamentalMatrix;

    /**
     * Indicates if this estimator is locked or not.
     */
    protected boolean locked;

    /**
     * Estimated camera for left view.
     */
    protected PinholeCamera estimatedLeftCamera;

    /**
     * Estimated camera for right view.
     */
    protected PinholeCamera estimatedRightCamera;

    /**
     * Listener to handle events raised by this instance.
     */
    protected InitialCamerasEstimatorListener listener;

    /**
     * Constructor.
     */
    protected InitialCamerasEstimator() {
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     */
    protected InitialCamerasEstimator(final FundamentalMatrix fundamentalMatrix) {
        this.fundamentalMatrix = fundamentalMatrix;
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this instance.
     */
    protected InitialCamerasEstimator(final InitialCamerasEstimatorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     * @param listener          listener to handle events raised by this instance.
     */
    protected InitialCamerasEstimator(final FundamentalMatrix fundamentalMatrix,
                                      final InitialCamerasEstimatorListener listener) {
        this.fundamentalMatrix = fundamentalMatrix;
        this.listener = listener;
    }

    /**
     * Gets fundamental matrix relating two views whose cameras need to be
     * estimated.
     *
     * @return fundamental matrix relating two views.
     */
    public FundamentalMatrix getFundamentalMatrix() {
        return fundamentalMatrix;
    }

    /**
     * Sets fundamental matrix relating two views whose cameras need to be
     * estimated.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     * @throws LockedException if estimator is locked.
     */
    public void setFundamentalMatrix(final FundamentalMatrix fundamentalMatrix) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.fundamentalMatrix = fundamentalMatrix;
    }

    /**
     * Gets listener to handle events raised by this instance.
     *
     * @return listener to handle events raised by this instance.
     */
    public InitialCamerasEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this instance.
     *
     * @param listener listener to handle events raised by this instance.
     */
    public void setListener(final InitialCamerasEstimatorListener listener) {
        this.listener = listener;
    }

    /**
     * Indicates if this estimator is locked or not.
     *
     * @return true if this estimator is locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Gets estimated camera for left view.
     *
     * @return estimated camera for left view.
     */
    public PinholeCamera getEstimatedLeftCamera() {
        return estimatedLeftCamera;
    }

    /**
     * Gets estimated camera for right view.
     *
     * @return estimated camera for right view.
     */
    public PinholeCamera getEstimatedRightCamera() {
        return estimatedRightCamera;
    }

    /**
     * Returns method used by this estimator.
     *
     * @return method used by this estimator.
     */
    public abstract InitialCamerasEstimatorMethod getMethod();

    /**
     * Indicates if estimator is ready.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public abstract boolean isReady();

    /**
     * Estimates cameras.
     *
     * @throws LockedException                         if estimator is locked.
     * @throws NotReadyException                       if estimator is not ready.
     * @throws InitialCamerasEstimationFailedException if estimation of cameras
     *                                                 fails for some reason, typically due to numerical
     *                                                 instabilities.
     */
    public abstract void estimate() throws LockedException, NotReadyException, InitialCamerasEstimationFailedException;

    /**
     * Creates an instance of an initial cameras estimator using provided
     * method.
     *
     * @param method method to estimate initial cameras.
     * @return an estimator.
     */
    public static InitialCamerasEstimator create(final InitialCamerasEstimatorMethod method) {
        return switch (method) {
            case ESSENTIAL_MATRIX -> new EssentialMatrixInitialCamerasEstimator();
            case DUAL_IMAGE_OF_ABSOLUTE_CONIC -> new DualImageOfAbsoluteConicInitialCamerasEstimator();
            default -> new DualAbsoluteQuadricInitialCamerasEstimator();
        };
    }

    /**
     * Creates an instance of an initial cameras estimator using provided
     * fundamental matrix and provided method.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     * @param method            method to estimate initial cameras.
     * @return an estimator.
     */
    public static InitialCamerasEstimator create(
            final FundamentalMatrix fundamentalMatrix, final InitialCamerasEstimatorMethod method) {
        return switch (method) {
            case ESSENTIAL_MATRIX -> new EssentialMatrixInitialCamerasEstimator(fundamentalMatrix);
            case DUAL_IMAGE_OF_ABSOLUTE_CONIC -> new DualImageOfAbsoluteConicInitialCamerasEstimator(fundamentalMatrix);
            default -> new DualAbsoluteQuadricInitialCamerasEstimator(fundamentalMatrix);
        };
    }

    /**
     * Creates an instance of an initial cameras estimator using provided
     * listener and method.
     *
     * @param listener listener to handle events.
     * @param method   method to estimate initial cameras.
     * @return an estimator.
     */
    public static InitialCamerasEstimator create(
            final InitialCamerasEstimatorListener listener, final InitialCamerasEstimatorMethod method) {
        return switch (method) {
            case ESSENTIAL_MATRIX -> new EssentialMatrixInitialCamerasEstimator(listener);
            case DUAL_IMAGE_OF_ABSOLUTE_CONIC -> new DualImageOfAbsoluteConicInitialCamerasEstimator(listener);
            default -> new DualAbsoluteQuadricInitialCamerasEstimator(listener);
        };
    }

    /**
     * Creates an instance of an initial cameras estimator using provided
     * fundamental matrix, listener and method.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     * @param listener          listener to handle events.
     * @param method            method to estimate initial cameras.
     * @return an estimator.
     */
    public static InitialCamerasEstimator create(
            final FundamentalMatrix fundamentalMatrix,
            final InitialCamerasEstimatorListener listener,
            final InitialCamerasEstimatorMethod method) {
        return switch (method) {
            case ESSENTIAL_MATRIX -> new EssentialMatrixInitialCamerasEstimator(fundamentalMatrix, listener);
            case DUAL_IMAGE_OF_ABSOLUTE_CONIC -> new DualImageOfAbsoluteConicInitialCamerasEstimator(
                    fundamentalMatrix, listener);
            default -> new DualAbsoluteQuadricInitialCamerasEstimator(fundamentalMatrix, listener);
        };
    }

    /**
     * Creates an instance of an initial cameras estimator using provided
     * method.
     *
     * @return an estimator.
     */
    public static InitialCamerasEstimator create() {
        return create(DEFAULT_METHOD);
    }

    /**
     * Creates an instance of an initial cameras estimator using provided
     * fundamental matrix and provided method.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     * @return an estimator.
     */
    public static InitialCamerasEstimator create(final FundamentalMatrix fundamentalMatrix) {
        return create(fundamentalMatrix, DEFAULT_METHOD);
    }

    /**
     * Creates an instance of an initial cameras estimator using provided
     * listener and method.
     *
     * @param listener listener to handle events.
     * @return an estimator.
     */
    public static InitialCamerasEstimator create(final InitialCamerasEstimatorListener listener) {
        return create(listener, DEFAULT_METHOD);
    }

    /**
     * Creates an instance of an initial cameras estimator using provided
     * fundamental matrix, listener and method.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     * @param listener          listener to handle events.
     * @return an estimator.
     */
    public static InitialCamerasEstimator create(
            final FundamentalMatrix fundamentalMatrix, final InitialCamerasEstimatorListener listener) {
        return create(fundamentalMatrix, listener, DEFAULT_METHOD);
    }
}
