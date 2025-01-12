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
package com.irurueta.ar.epipolar;

import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

import java.util.List;

/**
 * Fixes matched pairs of points so that they perfectly follow a given epipolar
 * geometry.
 * When matching points typically the matching precision is about 1 pixel,
 * however this makes that matched points under a given epipolar geometry (i.e.
 * fundamental or essential matrix), do not lie perfectly on the corresponding
 * epipolar plane or epipolar lines.
 * The consequence is that triangularization of these matches will fail or
 * produce inaccurate results.
 * By fixing matched points using a corrector following a given epipolar
 * geometry, this effect is alleviated.
 * This is an abstract class, subclasses will implement different methods to
 * fix matched points coordinates.
 */
public abstract class Corrector {

    /**
     * Default corrector type.
     */
    public static final CorrectorType DEFAULT_TYPE = CorrectorType.SAMPSON_CORRECTOR;

    /**
     * Default amount of progress variation before notifying a change in
     * estimation progress. By default, this is set to 5%.
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
     * A fundamental matrix defining an epipolar geometry.
     */
    protected FundamentalMatrix fundamentalMatrix;

    /**
     * List of points on left view to be corrected.
     */
    protected List<Point2D> leftPoints;

    /**
     * List of points on right view to be corrected.
     */
    protected List<Point2D> rightPoints;

    /**
     * List of points on left view after correction.
     */
    protected List<Point2D> leftCorrectedPoints;

    /**
     * List of points on right view after correction.
     */
    protected List<Point2D> rightCorrectedPoints;

    /**
     * Listener to notify start, stop and progress events.
     */
    protected CorrectorListener listener;

    /**
     * Indicates whether this instance is locked or not while doing computations.
     */
    protected boolean locked;

    /**
     * Amount of progress variation before notifying a progress change during
     * estimation.
     */
    protected float progressDelta;

    /**
     * Constructor.
     */
    protected Corrector() {
        progressDelta = DEFAULT_PROGRESS_DELTA;
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix to be set.
     */
    protected Corrector(final FundamentalMatrix fundamentalMatrix) {
        this();
        internalSetFundamentalMatrix(fundamentalMatrix);
    }

    /**
     * Constructor.
     *
     * @param leftPoints  points to be corrected on left view.
     * @param rightPoints points to be corrected on right view.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size.
     */
    protected Corrector(final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        this();
        internalSetLeftAndRightPoints(leftPoints, rightPoints);
    }

    /**
     * Constructor.
     *
     * @param leftPoints        points to be corrected on left view.
     * @param rightPoints       points to be corrected on right view.
     * @param fundamentalMatrix fundamental matrix to be set.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size.
     */
    protected Corrector(final List<Point2D> leftPoints, final List<Point2D> rightPoints,
                        final FundamentalMatrix fundamentalMatrix) {
        this();
        internalSetLeftAndRightPoints(leftPoints, rightPoints);
        internalSetFundamentalMatrix(fundamentalMatrix);
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events generated by this class.
     */
    protected Corrector(final CorrectorListener listener) {
        this();
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix to be set.
     * @param listener          listener to handle events generated by this class.
     */
    protected Corrector(final FundamentalMatrix fundamentalMatrix, final CorrectorListener listener) {
        this(fundamentalMatrix);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param leftPoints  points to be corrected on left view.
     * @param rightPoints points to be corrected on right view.
     * @param listener    listener to handle events generated by this class.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size.
     */
    protected Corrector(final List<Point2D> leftPoints, final List<Point2D> rightPoints,
                        final CorrectorListener listener) {
        this(leftPoints, rightPoints);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param leftPoints        points to be corrected on left view.
     * @param rightPoints       points to be corrected on right view.
     * @param fundamentalMatrix fundamental matrix to be set.
     * @param listener          listener to handle events generated by this class.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size.
     */
    protected Corrector(final List<Point2D> leftPoints, final List<Point2D> rightPoints,
                        final FundamentalMatrix fundamentalMatrix, final CorrectorListener listener) {
        this(leftPoints, rightPoints, fundamentalMatrix);
        this.listener = listener;
    }

    /**
     * Sets the fundamental matrix defining the epipolar geometry.
     *
     * @param fundamentalMatrix fundamental matrix to be set.
     * @throws LockedException if this instance is locked.
     */
    public final void setFundamentalMatrix(final FundamentalMatrix fundamentalMatrix) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetFundamentalMatrix(fundamentalMatrix);
    }

    /**
     * Returns fundamental matrix defining epipolar geometry.
     *
     * @return fundamental matrix defining epipolar geometry.
     */
    public FundamentalMatrix getFundamentalMatrix() {
        return fundamentalMatrix;
    }

    /**
     * Returns list of points to be corrected on left view.
     *
     * @return list of points to be corrected on left view.
     */
    public List<Point2D> getLeftPoints() {
        return leftPoints;
    }

    /**
     * Returns list of points to be corrected on right view.
     *
     * @return list of points to be corrected on right view.
     */
    public List<Point2D> getRightPoints() {
        return rightPoints;
    }

    /**
     * Sets lists of points to be corrected on left and right views.
     *
     * @param leftPoints  points to be corrected on left view.
     * @param rightPoints points to be corrected on right view.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size.
     * @throws LockedException          if instance is locked doing computations.
     */
    public void setLeftAndRightPoints(final List<Point2D> leftPoints, final List<Point2D> rightPoints)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetLeftAndRightPoints(leftPoints, rightPoints);
    }

    /**
     * Sets lists of points to be corrected on left and right views and
     * fundamental matrix defining epipolar geometry.
     *
     * @param leftPoints        points to be corrected on left view.
     * @param rightPoints       points to be corrected on right view.
     * @param fundamentalMatrix fundamental matrix to be set.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size.
     * @throws LockedException          if instance is locked doing computations.
     */
    public void setPointsAndFundamentalMatrix(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints, final FundamentalMatrix fundamentalMatrix)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetFundamentalMatrix(fundamentalMatrix);
        internalSetLeftAndRightPoints(leftPoints, rightPoints);
    }

    /**
     * Returns list of points on left view obtained after correction.
     *
     * @return list of points on left view obtained after correction.
     */
    public List<Point2D> getLeftCorrectedPoints() {
        return leftCorrectedPoints;
    }

    /**
     * Returns list of points on right view obtained after correction.
     *
     * @return list of points on right view obtained after correction.
     */
    public List<Point2D> getRightCorrectedPoints() {
        return rightCorrectedPoints;
    }

    /**
     * Returns boolean indicating whether this instance is locked or not doing
     * computations.
     *
     * @return true if instance is locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Returns amount of progress variation before notifying a progress change
     * during estimation.
     *
     * @return amount of progress variation before notifying a progress change
     * during estimation.
     */
    public float getProgressDelta() {
        return progressDelta;
    }

    /**
     * Sets amount of progress variation before notifying a progress change
     * during estimation.
     *
     * @param progressDelta amount of progress variation before notifying a
     *                      progress change during estimation.
     * @throws IllegalArgumentException if progress delta is less than zero or
     *                                  greater than 1.
     * @throws LockedException          if this estimator is locked because an estimation
     *                                  is being computed.
     */
    public void setProgressDelta(final float progressDelta) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA || progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        this.progressDelta = progressDelta;
    }

    /**
     * Returns listener to handle events generated by this class.
     *
     * @return listener to handle events generated by this class.
     */
    public CorrectorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events generated by this class.
     *
     * @param listener listener to handle events generated by this class.
     * @throws LockedException if this estimator is locked because an estimation
     *                         is being computed.
     */
    public void setListener(final CorrectorListener listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.listener = listener;
    }

    /**
     * Indicates whether provided lists of points to be corrected on left and
     * right views are valid.
     * Lists are valid when both have the same size.
     *
     * @param leftPoints  points to be corrected on left view.
     * @param rightPoints points to be corrected on right view.
     * @return true if lists of points are valid, false otherwise.
     */
    public static boolean areValidPoints(final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        return leftPoints != null && rightPoints != null && leftPoints.size() == rightPoints.size();
    }

    /**
     * Indicates whether this instance is ready to correct provided left
     * and right points using provided fundamental matrix.
     *
     * @return true if ready, false otherwise.
     */
    public boolean isReady() {
        return areValidPoints(leftPoints, rightPoints) && fundamentalMatrix != null
                && fundamentalMatrix.isInternalMatrixAvailable();
    }

    /**
     * Corrects the lists of provided matched points to be corrected.
     *
     * @throws NotReadyException   if this instance is not ready (either points or
     *                             fundamental matrix has not been provided yet).
     * @throws LockedException     if this instance is locked doing computations.
     * @throws CorrectionException if correction fails.
     */
    public abstract void correct() throws NotReadyException, LockedException, CorrectionException;

    /**
     * Gets type of correction being used.
     *
     * @return type of correction.
     */
    public abstract CorrectorType getType();

    /**
     * Creates an instance of a corrector using provided type.
     *
     * @param type a corrector type.
     * @return an instance of a corrector.
     */
    public static Corrector create(final CorrectorType type) {
        return create((CorrectorListener) null, type);
    }

    /**
     * Creates an instance of a corrector using provided fundamental matrix
     * and provided type.
     *
     * @param fundamentalMatrix fundamental matrix defining the epipolar
     *                          geometry.
     * @param type              a corrector type.
     * @return an instance of a corrector.
     */
    public static Corrector create(final FundamentalMatrix fundamentalMatrix, final CorrectorType type) {
        return create(fundamentalMatrix, null, type);
    }

    /**
     * Creates an instance of a corrector using provided left and right points
     * to be corrected and provided type.
     *
     * @param leftPoints  matched points on left view to be corrected.
     * @param rightPoints matched points on right view to be corrected.
     * @param type        a corrector type.
     * @return an instance of a corrector.
     */
    public static Corrector create(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints, final CorrectorType type) {
        return create(leftPoints, rightPoints, (CorrectorListener) null, type);
    }

    /**
     * Creates an instance of a corrector using provided left and right points
     * to be corrected, provided fundamental matrix and provided type.
     *
     * @param leftPoints        matched points on left view to be corrected.
     * @param rightPoints       matched points on right view to be corrected.
     * @param fundamentalMatrix fundamental matrix defining the epipolar
     *                          geometry.
     * @param type              a corrector type.
     * @return an instance of a corrector.
     */
    public static Corrector create(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final FundamentalMatrix fundamentalMatrix, final CorrectorType type) {
        return create(leftPoints, rightPoints, fundamentalMatrix, null, type);
    }

    /**
     * Creates an instance of a corrector using provided type.
     *
     * @param listener listener to handle events generated by this class.
     * @param type     a corrector type.
     * @return an instance of a corrector.
     */
    public static Corrector create(final CorrectorListener listener, final CorrectorType type) {
        return type == CorrectorType.SAMPSON_CORRECTOR
                ? new SampsonCorrector(listener) : new GoldStandardCorrector(listener);
    }

    /**
     * Creates an instance of a corrector using provided fundamental matrix
     * and provided type.
     *
     * @param fundamentalMatrix fundamental matrix defining the epipolar
     *                          geometry.
     * @param listener          listener to handle events generated by this class.
     * @param type              a corrector type.
     * @return an instance of a corrector.
     */
    public static Corrector create(
            final FundamentalMatrix fundamentalMatrix, final CorrectorListener listener, final CorrectorType type) {
        return type == CorrectorType.SAMPSON_CORRECTOR
                ? new SampsonCorrector(fundamentalMatrix, listener)
                : new GoldStandardCorrector(fundamentalMatrix, listener);
    }

    /**
     * Creates an instance of a corrector using provided left and right points
     * to be corrected and provided type.
     *
     * @param leftPoints  matched points on left view to be corrected.
     * @param rightPoints matched points on right view to be corrected.
     * @param listener    listener to handle events generated by this class.
     * @param type        a corrector type.
     * @return an instance of a corrector.
     */
    public static Corrector create(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints, final CorrectorListener listener,
            final CorrectorType type) {
        return type == CorrectorType.SAMPSON_CORRECTOR
                ? new SampsonCorrector(leftPoints, rightPoints, listener)
                : new GoldStandardCorrector(leftPoints, rightPoints, listener);
    }

    /**
     * Creates an instance of a corrector using provided left and right points
     * to be corrected, provided fundamental matrix and provided type.
     *
     * @param leftPoints        matched points on left view to be corrected.
     * @param rightPoints       matched points on right view to be corrected.
     * @param fundamentalMatrix fundamental matrix defining the epipolar
     *                          geometry.
     * @param listener          listener to handle events generated by this class.
     * @param type              a corrector type.
     * @return an instance of a corrector.
     */
    public static Corrector create(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints, final FundamentalMatrix fundamentalMatrix,
            final CorrectorListener listener, final CorrectorType type) {
        return type == CorrectorType.SAMPSON_CORRECTOR
                ? new SampsonCorrector(leftPoints, rightPoints, fundamentalMatrix, listener)
                : new GoldStandardCorrector(leftPoints, rightPoints, fundamentalMatrix, listener);
    }

    /**
     * Creates an instance of a corrector using default type.
     *
     * @param listener listener to handle events generated by this class.
     * @return an instance of a corrector.
     */
    public static Corrector create(final CorrectorListener listener) {
        return create(listener, DEFAULT_TYPE);
    }

    /**
     * Creates an instance of a corrector using provided fundamental matrix
     * and default type.
     *
     * @param fundamentalMatrix fundamental matrix defining the epipolar
     *                          geometry.
     * @param listener          listener to handle events generated by this class.
     * @return an instance of a corrector.
     */
    public static Corrector create(final FundamentalMatrix fundamentalMatrix, final CorrectorListener listener) {
        return create(fundamentalMatrix, listener, DEFAULT_TYPE);
    }

    /**
     * Creates an instance of a corrector using provided left and right points
     * to be corrected and provided type.
     *
     * @param leftPoints  matched points on left view to be corrected.
     * @param rightPoints matched points on right view to be corrected.
     * @param listener    listener to handle events generated by this class.
     * @return an instance of a corrector.
     */
    public static Corrector create(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints, final CorrectorListener listener) {
        return create(leftPoints, rightPoints, listener, DEFAULT_TYPE);
    }

    /**
     * Creates an instance of a corrector using provided left and right points
     * to be corrected, provided fundamental matrix and provided type.
     *
     * @param leftPoints        matched points on left view to be corrected.
     * @param rightPoints       matched points on right view to be corrected.
     * @param fundamentalMatrix fundamental matrix defining the epipolar
     *                          geometry.
     * @param listener          listener to handle events generated by this class.
     * @return an instance of a corrector.
     */
    public static Corrector create(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints, final FundamentalMatrix fundamentalMatrix,
            final CorrectorListener listener) {
        return create(leftPoints, rightPoints, fundamentalMatrix, listener, DEFAULT_TYPE);
    }

    /**
     * Creates an instance of a corrector using default type.
     *
     * @return an instance of a corrector.
     */
    public static Corrector create() {
        return create(DEFAULT_TYPE);
    }

    /**
     * Creates an instance of a corrector using provided fundamental matrix
     * and default type.
     *
     * @param fundamentalMatrix fundamental matrix defining the epipolar
     *                          geometry.
     * @return an instance of a corrector.
     */
    public static Corrector create(final FundamentalMatrix fundamentalMatrix) {
        return create(fundamentalMatrix, DEFAULT_TYPE);
    }

    /**
     * Creates an instance of a corrector using provided left and right points
     * to be corrected and default type.
     *
     * @param leftPoints  matched points on left view to be corrected.
     * @param rightPoints matched points on right view to be corrected.
     * @return an instance of a corrector.
     */
    public static Corrector create(final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        return create(leftPoints, rightPoints, DEFAULT_TYPE);
    }

    /**
     * Creates an instance of a corrector using provided left and right points
     * to be corrected, provided fundamental matrix and default type.
     *
     * @param leftPoints        matched points on left view to be corrected.
     * @param rightPoints       matched points on right view to be corrected.
     * @param fundamentalMatrix fundamental matrix defining the epipolar
     *                          geometry.
     * @return an instance of a corrector.
     */
    public static Corrector create(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final FundamentalMatrix fundamentalMatrix) {
        return create(leftPoints, rightPoints, fundamentalMatrix, DEFAULT_TYPE);
    }

    /**
     * Internal method to set fundamental matrix defining the epipolar geometry.
     *
     * @param fundamentalMatrix fundamental matrix to be set.
     */
    private void internalSetFundamentalMatrix(final FundamentalMatrix fundamentalMatrix) {
        this.fundamentalMatrix = fundamentalMatrix;
    }

    /**
     * Internal method to set lists of points to be corrected on left and right
     * views.
     *
     * @param leftPoints  points to be corrected on left view.
     * @param rightPoints points to be corrected on right view.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size.
     */
    private void internalSetLeftAndRightPoints(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        if (!areValidPoints(leftPoints, rightPoints)) {
            throw new IllegalArgumentException();
        }

        this.leftPoints = leftPoints;
        this.rightPoints = rightPoints;
    }
}
