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
package com.irurueta.ar.epipolar.estimators;

import com.irurueta.ar.epipolar.EssentialMatrix;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.geometry.GeometryException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Transformation2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

import java.util.ArrayList;
import java.util.List;

/**
 * This class takes an input 2D homography (e.g. transformation) and a given
 * pair of intrinsic parameters for left and right views, and estimates all
 * possible fundamental matrices generating such homography in a planar scene.
 * This estimator will generate either 2 or 4 possible solutions, however only
 * 1 solution will be physically possible, which corresponds to the solution
 * that generates triangulated points located in front of the cameras generating
 * such epipolar geometries.
 * This class is useful in planar scenes where 8-point and 7-point algorithms
 * will fail since this kind of geometry is a degenerate configuration of
 * points.
 */
public class PlanarFundamentalMatrixEstimator {

    /**
     * 2D transformation relating two views (left view to right view).
     */
    private Transformation2D homography;

    /**
     * Intrinsic parameters to be used on left view.
     */
    private PinholeCameraIntrinsicParameters leftIntrinsics;

    /**
     * Intrinsic parameters to be used on right view.
     */
    private PinholeCameraIntrinsicParameters rightIntrinsics;

    /**
     * Listener to attend events generated by this instance.
     */
    private PlanarFundamentalMatrixEstimatorListener listener;

    /**
     * Indicates whether estimator is locked while estimating fundamental
     * matrix.
     */
    private boolean locked;

    /**
     * Constructor.
     */
    public PlanarFundamentalMatrixEstimator() {
    }

    /**
     * Constructor.
     *
     * @param homography      2D transformation relating two views (left view to
     *                        right view).
     * @param leftIntrinsics  intrinsic parameters to be used on left view.
     * @param rightIntrinsics intrinsic parameters to be used on right view.
     */
    public PlanarFundamentalMatrixEstimator(final Transformation2D homography,
                                            final PinholeCameraIntrinsicParameters leftIntrinsics,
                                            final PinholeCameraIntrinsicParameters rightIntrinsics) {
        this.homography = homography;
        this.leftIntrinsics = leftIntrinsics;
        this.rightIntrinsics = rightIntrinsics;
    }

    /**
     * Constructor.
     *
     * @param homography      2D transformation relating two views (left view to
     *                        right view).
     * @param leftIntrinsics  intrinsic parameters to be used on left view.
     * @param rightIntrinsics intrinsic parameters to be used on right view.
     * @param listener        listener to attend events generated by this instance.
     */
    public PlanarFundamentalMatrixEstimator(final Transformation2D homography,
                                            final PinholeCameraIntrinsicParameters leftIntrinsics,
                                            final PinholeCameraIntrinsicParameters rightIntrinsics,
                                            final PlanarFundamentalMatrixEstimatorListener listener) {
        this(homography, leftIntrinsics, rightIntrinsics);
        this.listener = listener;
    }

    /**
     * Gets 2D transformation relating two views (left view to right view).
     *
     * @return 2D transformation relating two views.
     */
    public Transformation2D getHomography() {
        return homography;
    }

    /**
     * Sets 2D transformation relating two views (left view to right view).
     *
     * @param homography 2D transformation relating two views.
     * @throws LockedException if estimator is locked.
     */
    public void setHomography(final Transformation2D homography) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.homography = homography;
    }

    /**
     * Gets intrinsic parameters to be used on left view.
     *
     * @return intrinsic parameters to be used on left view.
     */
    public PinholeCameraIntrinsicParameters getLeftIntrinsics() {
        return leftIntrinsics;
    }

    /**
     * Sets intrinsic parameters to be used on left view.
     *
     * @param leftIntrinsics intrinsic parameters to be used on left view.
     * @throws LockedException if estimator is locked.
     */
    public void setLeftIntrinsics(final PinholeCameraIntrinsicParameters leftIntrinsics) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.leftIntrinsics = leftIntrinsics;
    }

    /**
     * Gets intrinsic parameters to be used on right view.
     *
     * @return intrinsic parameters to be used on right view.
     */
    public PinholeCameraIntrinsicParameters getRightIntrinsics() {
        return rightIntrinsics;
    }

    /**
     * Sets intrinsic parameters to be used on right view.
     *
     * @param rightIntrinsics intrinsic parameters to be used on right view.
     * @throws LockedException if estimator is locked.
     */
    public void setRightIntrinsics(final PinholeCameraIntrinsicParameters rightIntrinsics) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.rightIntrinsics = rightIntrinsics;
    }

    /**
     * Gets listener to attend events generated by this instance.
     *
     * @return listener to attend events generated by this instance.
     */
    public PlanarFundamentalMatrixEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to attend events generated by this instance.
     *
     * @param listener listener to attend events generated by this instance.
     */
    public void setListener(final PlanarFundamentalMatrixEstimatorListener listener) {
        this.listener = listener;
    }

    /**
     * Indicates whether estimator is locked while estimating fundamental
     * matrix.
     *
     * @return true if estimator is locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Indicates whether estimator is ready to start the estimation when all
     * required data has been provided.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return homography != null && leftIntrinsics != null && rightIntrinsics != null;
    }

    /**
     * Estimates fundamental matrices and returns the estimated result.
     *
     * @return estimated fundamental matrices.
     * @throws LockedException                     if estimator is locked.
     * @throws NotReadyException                   if estimator is not ready.
     * @throws FundamentalMatrixEstimatorException if estimation fails for some
     *                                             reason.
     */
    public List<FundamentalMatrix> estimate() throws LockedException, NotReadyException,
            FundamentalMatrixEstimatorException {
        final var result = new ArrayList<FundamentalMatrix>();
        estimate(result);
        return result;
    }

    /**
     * Estimates fundamental matrices and stores result into provided instance.
     *
     * @param result instance where result will be stored.
     * @throws LockedException                     if estimator is locked.
     * @throws NotReadyException                   if estimator is not ready.
     * @throws FundamentalMatrixEstimatorException if estimation fails for some
     *                                             reason.
     */
    public void estimate(final List<FundamentalMatrix> result) throws LockedException, NotReadyException,
            FundamentalMatrixEstimatorException {
        if (isLocked()) {
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

            final var decomposer = new HomographyDecomposer(homography, leftIntrinsics, rightIntrinsics);

            final var decompositions = decomposer.decompose();
            result.clear();
            for (final var decomposition : decompositions) {
                final var rotation = decomposition.getTransformation().getRotation();
                final var translation = new HomogeneousPoint2D(decomposition.getTransformation().getTranslation());
                final var essential = new EssentialMatrix(rotation, translation);
                final var fundamentalMatrix = essential.toFundamentalMatrix(leftIntrinsics, rightIntrinsics);
                result.add(fundamentalMatrix);
            }

            if (listener != null) {
                listener.onEstimateEnd(this, result);
            }

        } catch (final GeometryException e) {
            throw new FundamentalMatrixEstimatorException(e);
        } finally {
            locked = false;
        }
    }
}
