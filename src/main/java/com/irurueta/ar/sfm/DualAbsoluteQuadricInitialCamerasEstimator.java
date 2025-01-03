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

import com.irurueta.ar.calibration.estimators.DualAbsoluteQuadricEstimator;
import com.irurueta.ar.calibration.estimators.LMSEDualAbsoluteQuadricEstimator;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

import java.util.ArrayList;

/**
 * Estimates an initial pair of cameras in the metric stratum (up to an
 * arbitrary scale) using a given fundamental matrix and assuming zero skewness
 * and principal point at the origin for the intrinsic parameters of estimated
 * cameras.
 * Aspect ratio can be configured but by default it is assumed to be 1.0.
 */
public class DualAbsoluteQuadricInitialCamerasEstimator extends InitialCamerasEstimator {

    /**
     * Aspect ratio of intrinsic parameters of cameras.
     * Typically, this value is 1.0 if vertical coordinates increase upwards,
     * or -1.0 if it is the opposite.
     */
    private double aspectRatio = DualAbsoluteQuadricEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO;

    /**
     * Constructor.
     */
    public DualAbsoluteQuadricInitialCamerasEstimator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     */
    public DualAbsoluteQuadricInitialCamerasEstimator(final FundamentalMatrix fundamentalMatrix) {
        super(fundamentalMatrix);
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this instance.
     */
    public DualAbsoluteQuadricInitialCamerasEstimator(final InitialCamerasEstimatorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     * @param listener          listener to handle events raised by this instance.
     */
    public DualAbsoluteQuadricInitialCamerasEstimator(
            final FundamentalMatrix fundamentalMatrix, final InitialCamerasEstimatorListener listener) {
        super(fundamentalMatrix, listener);
    }

    /**
     * Returns method used by this estimator.
     *
     * @return method used by this estimator.
     */
    @Override
    public InitialCamerasEstimatorMethod getMethod() {
        return InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC;
    }

    /**
     * Indicates if estimator is ready.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return fundamentalMatrix != null;
    }

    /**
     * Estimates cameras.
     *
     * @throws LockedException                         if estimator is locked.
     * @throws NotReadyException                       if estimator is not ready.
     * @throws InitialCamerasEstimationFailedException if estimation of cameras
     *                                                 fails for some reason, typically due to numerical
     *                                                 instabilities.
     */
    @Override
    public void estimate() throws LockedException, NotReadyException, InitialCamerasEstimationFailedException {
        if (isLocked()) {
            throw new LockedException();
        }

        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            locked = true;

            if (listener != null) {
                listener.onStart(this);
            }

            if (estimatedLeftCamera == null) {
                estimatedLeftCamera = new PinholeCamera();
            }
            if (estimatedRightCamera == null) {
                estimatedRightCamera = new PinholeCamera();
            }

            generateInitialMetricCamerasUsingDAQ(fundamentalMatrix, aspectRatio, estimatedLeftCamera,
                    estimatedRightCamera);

            if (listener != null) {
                listener.onFinish(this, estimatedLeftCamera, estimatedRightCamera);
            }
        } catch (final InitialCamerasEstimationFailedException e) {
            if (listener != null) {
                listener.onFail(this, e);
            }
            throw e;
        } finally {
            locked = false;
        }
    }

    /**
     * Gets aspect ratio of intrinsic parameters of cameras.
     * Typically, this value is 1.0 if vertical coordinates increase upwards,
     * or -1.0 if it is the opposite.
     *
     * @return aspect ratio of intrinsic parameters of cameras.
     */
    public double getAspectRatio() {
        return aspectRatio;
    }

    /**
     * Sets aspect ratio of intrinsic parameters of cameras.
     * Typically, this value is 1.0 if vertical coordinates increase upwards,
     * or -1.0 if it is the opposite.
     *
     * @param aspectRatio aspect ratio of intrinsic parameters of cameras.
     * @throws LockedException if estimator is locked.
     */
    public void setAspectRatio(final double aspectRatio) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.aspectRatio = aspectRatio;
    }

    /**
     * Generates initial cameras in metric stratum (with arbitrary scale) using
     * provided fundamental matrix by means of estimation of the Dual Absolute
     * Quadric and the required projective to metric transformation.
     *
     * @param fundamentalMatrix input fundamental matrix to estimate cameras.
     * @param leftCamera        instance where left camera will be stored.
     * @param rightCamera       instance where right camera will be stored.
     * @throws InitialCamerasEstimationFailedException if estimation fails.
     */
    public static void generateInitialMetricCamerasUsingDAQ(
            final FundamentalMatrix fundamentalMatrix,
            final PinholeCamera leftCamera,
            final PinholeCamera rightCamera)
            throws InitialCamerasEstimationFailedException {
        generateInitialMetricCamerasUsingDAQ(fundamentalMatrix,
                DualAbsoluteQuadricEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, leftCamera, rightCamera);
    }

    /**
     * Generates initial cameras in metric stratum (with arbitrary scale) using
     * provided fundamental matrix by means of estimation of the Dual Absolute
     * Quadric and the required projective to metric transformation.
     *
     * @param fundamentalMatrix input fundamental matrix to estimate cameras.
     * @param aspectRatio       aspect ratio of intrinsic parameters of cameras.
     *                          Typically, this value is 1.0 if vertical coordinates increase upwards or
     *                          -1.0 if it is the opposite.
     * @param leftCamera        instance where left camera will be stored.
     * @param rightCamera       instance where right camera will be stored.
     * @throws InitialCamerasEstimationFailedException if estimation fails.
     */
    public static void generateInitialMetricCamerasUsingDAQ(
            final FundamentalMatrix fundamentalMatrix,
            final double aspectRatio,
            final PinholeCamera leftCamera,
            final PinholeCamera rightCamera)
            throws InitialCamerasEstimationFailedException {

        try {
            // generate arbitrary projective cameras
            fundamentalMatrix.generateCamerasInArbitraryProjectiveSpace(leftCamera, rightCamera);

            final var cameras = new ArrayList<PinholeCamera>();
            cameras.add(leftCamera);
            cameras.add(rightCamera);

            // estimate dual absolute quadric
            final var daqEstimator = new LMSEDualAbsoluteQuadricEstimator(cameras);
            daqEstimator.setLMSESolutionAllowed(false);
            daqEstimator.setZeroSkewness(true);
            daqEstimator.setPrincipalPointAtOrigin(true);
            daqEstimator.setFocalDistanceAspectRatioKnown(true);
            daqEstimator.setFocalDistanceAspectRatio(aspectRatio);
            daqEstimator.setSingularityEnforced(true);

            final var daq = daqEstimator.estimate();
            final var transformation = daq.getMetricToProjectiveTransformation();
            // inverse transformation to upgrade cameras from projective to
            // metric stratum
            transformation.inverse();

            // transform cameras
            transformation.transform(leftCamera);
            transformation.transform(rightCamera);
        } catch (final Exception e) {
            throw new InitialCamerasEstimationFailedException(e);
        }
    }
}
