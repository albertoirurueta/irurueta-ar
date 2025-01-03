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
package com.irurueta.ar.epipolar.estimators;

import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

import java.util.List;

/**
 * Base class for a non-robust fundamental matrix estimator.
 */
public abstract class FundamentalMatrixEstimator {

    /**
     * Default method for non-robust fundamental matrix estimation.
     */
    public static final FundamentalMatrixEstimatorMethod DEFAULT_METHOD =
            FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM;

    /**
     * Listener to be notified of events generated by this fundamental matrix
     * estimator.
     */
    protected FundamentalMatrixEstimatorListener listener;

    /**
     * List of 2D points corresponding to left view.
     */
    protected List<Point2D> leftPoints;

    /**
     * List of 2D points corresponding to right view.
     */
    protected List<Point2D> rightPoints;

    /**
     * Indicates whether this instance is locked because estimation is being
     * computed.
     */
    protected boolean locked;

    /**
     * Constructor.
     */
    protected FundamentalMatrixEstimator() {
    }

    /**
     * Constructor with matched 2D points.
     *
     * @param leftPoints  2D points on left view.
     * @param rightPoints 2D points on right view.
     * @throws IllegalArgumentException if provided list of points do not
     *                                  have the same length.
     */
    protected FundamentalMatrixEstimator(final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        internalSetPoints(leftPoints, rightPoints);
    }

    /**
     * Returns matched 2D points on left view.
     *
     * @return 2D points on left view.
     */
    public List<Point2D> getLeftPoints() {
        return leftPoints;
    }

    /**
     * Returns matched 2D points on right view.
     *
     * @return 2D points on right view.
     */
    public List<Point2D> getRightPoints() {
        return rightPoints;
    }

    /**
     * Sets matched 2D points on both left and right views.
     *
     * @param leftPoints  matched 2D points on left view.
     * @param rightPoints matched 2D points on right view.
     * @throws LockedException          if this fundamental matrix estimator is locked.
     * @throws IllegalArgumentException if provided matched points on left and
     *                                  right views do not have the same length.
     */
    public void setPoints(final List<Point2D> leftPoints, final List<Point2D> rightPoints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetPoints(leftPoints, rightPoints);
    }

    /**
     * Returns listener to be notified of events generated by this fundamental
     * matrix estimator.
     *
     * @return listener to be notified of events generated by this fundamental
     * matrix estimator.
     */
    public FundamentalMatrixEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to be notified of events generated by this fundamental
     * matrix estimator.
     *
     * @param listener listener to be notified of events generated by this
     *                 fundamental matrix estimator.
     * @throws LockedException if this estimator is locked.
     */
    public void setListener(final FundamentalMatrixEstimatorListener listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.listener = listener;
    }

    /**
     * Returns boolean indicating if estimator is locked because estimation is
     * under progress.
     *
     * @return true if estimator is locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Returns boolean indicating whether estimator is ready to start the
     * fundamental matrix estimation.
     * This is true when the required number of matched points is provided to
     * obtain a solution.
     *
     * @return true if estimator is ready to start the fundamental matrix
     * estimation, false otherwise.
     */
    public abstract boolean isReady();

    /**
     * Estimates a fundamental matrix using provided lists of matched points on
     * left and right views.
     *
     * @return a fundamental matrix.
     * @throws LockedException                     if estimator is locked doing an estimation.
     * @throws NotReadyException                   if estimator is not ready because required
     *                                             input points have not already been provided.
     * @throws FundamentalMatrixEstimatorException if configuration of provided
     *                                             2D points is degenerate and fundamental matrix
     *                                             estimation fails.
     */
    public abstract FundamentalMatrix estimate() throws LockedException, NotReadyException,
            FundamentalMatrixEstimatorException;

    /**
     * Returns method of non-robust fundamental matrix estimator.
     *
     * @return method of fundamental matrix estimator.
     */
    public abstract FundamentalMatrixEstimatorMethod getMethod();

    /**
     * Returns minimum number of matched pair of points required to start
     * the estimation.
     *
     * @return minimum number of matched pair of points required to start
     * the estimation.
     */
    public abstract int getMinRequiredPoints();

    /**
     * Creates an instance of a fundamental matrix estimator using provided
     * method.
     *
     * @param method a fundamental matrix estimator method.
     * @return an instance of a fundamental matrix estimator.
     */
    public static FundamentalMatrixEstimator create(final FundamentalMatrixEstimatorMethod method) {
        return switch (method) {
            case AFFINE_ALGORITHM -> new AffineFundamentalMatrixEstimator();
            case EIGHT_POINTS_ALGORITHM -> new EightPointsFundamentalMatrixEstimator();
            default -> new SevenPointsFundamentalMatrixEstimator();
        };
    }

    /**
     * Creates an instance of a fundamental matrix estimator using provided
     * matched 2D points on left and right views and provided method.
     *
     * @param leftPoints  matched 2D points on left view.
     * @param rightPoints matched 2D points on right view.
     * @param method      a fundamental matrix estimator method.
     * @return an instance of a fundamental matrix estimator.
     * @throws IllegalArgumentException if provided list of points do not
     *                                  have the same length.
     */
    public static FundamentalMatrixEstimator create(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final FundamentalMatrixEstimatorMethod method) {
        return switch (method) {
            case AFFINE_ALGORITHM -> new AffineFundamentalMatrixEstimator(leftPoints, rightPoints);
            case EIGHT_POINTS_ALGORITHM -> new EightPointsFundamentalMatrixEstimator(leftPoints, rightPoints);
            default -> new SevenPointsFundamentalMatrixEstimator(leftPoints, rightPoints);
        };
    }

    /**
     * Creates an instance of a fundamental matrix estimator using default
     * method.
     *
     * @return an instance of a fundamental matrix estimator.
     */
    public static FundamentalMatrixEstimator create() {
        return create(DEFAULT_METHOD);
    }

    /**
     * Creates an instance of a fundamental matrix estimator using provided
     * matched 2D points on left and right views and default method.
     *
     * @param leftPoints  matched 2D points on left view.
     * @param rightPoints matched 2D points on right view.
     * @return an instance of a fundamental matrix estimator.
     * @throws IllegalArgumentException if provided list of points do not
     *                                  have the same length.
     */
    public static FundamentalMatrixEstimator create(final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        return create(leftPoints, rightPoints, DEFAULT_METHOD);
    }

    /**
     * Sets matched 2D points on left and right views.
     * This method does not check whether instance is locked or not.
     *
     * @param leftPoints  matched 2D points on left view.
     * @param rightPoints matched 2D points on right view.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size.
     */
    private void internalSetPoints(final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        if (leftPoints.size() != rightPoints.size()) {
            throw new IllegalArgumentException();
        }

        this.leftPoints = leftPoints;
        this.rightPoints = rightPoints;
    }
}
