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
import com.irurueta.geometry.estimators.NotReadyException;

/**
 * Fixes a single matched pair of points so that they perfectly follow a given
 * epipolar geometry.
 * When matching points typically the matching precision is about 1 pixel,
 * however this makes that matched points under a given epipolar geometry (i.e.
 * fundamental or essential matrix), do not lie perfectly on the corresponding
 * epipolar plane or epipolar lines.
 * The consequence is that triangularization of these matches will fail or
 * produce inaccurate results.
 * By fixing matched points using a corrector following a given epipolar
 * geometry, this effect is alleviated.
 * This is an abstract class, subclasses will implement different methods to
 * fix matched points coordinates
 */
public abstract class SingleCorrector {

    /**
     * Default corrector type.
     */
    public static final CorrectorType DEFAULT_TYPE = CorrectorType.GOLD_STANDARD;

    /**
     * Left matched point to be corrected.
     */
    protected Point2D leftPoint;

    /**
     * Right matched point to be corrected.
     */
    protected Point2D rightPoint;

    /**
     * Left matched point after correction.
     */
    protected Point2D leftCorrectedPoint;

    /**
     * Right matched point after correction.
     */
    protected Point2D rightCorrectedPoint;

    /**
     * A fundamental matrix defining an epipolar geometry.
     */
    protected FundamentalMatrix fundamentalMatrix;

    /**
     * Constructor.
     */
    protected SingleCorrector() {
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix defining the epipolar
     *                          geometry.
     */
    protected SingleCorrector(final FundamentalMatrix fundamentalMatrix) {
        setFundamentalMatrix(fundamentalMatrix);
    }

    /**
     * Constructor.
     *
     * @param leftPoint  matched point on left view to be corrected.
     * @param rightPoint matched point on right view to be corrected.
     */
    protected SingleCorrector(final Point2D leftPoint, final Point2D rightPoint) {
        setPoints(leftPoint, rightPoint);
    }

    /**
     * Constructor.
     *
     * @param leftPoint         matched point on left view to be corrected.
     * @param rightPoint        matched point on right view to be corrected.
     * @param fundamentalMatrix fundamental matrix defining an epipolar geometry.
     */
    protected SingleCorrector(final Point2D leftPoint, final Point2D rightPoint,
                              final FundamentalMatrix fundamentalMatrix) {
        setPointsAndFundamentalMatrix(leftPoint, rightPoint, fundamentalMatrix);
    }

    /**
     * Sets a matched pair of points to be corrected and a fundamental matrix
     * defining the epipolar geometry.
     *
     * @param leftPoint         matched point on left view to be corrected.
     * @param rightPoint        matched point on right view to be corrected.
     * @param fundamentalMatrix fundamental matrix defining an epipolar geometry.
     */
    public final void setPointsAndFundamentalMatrix(
            final Point2D leftPoint, final Point2D rightPoint, final FundamentalMatrix fundamentalMatrix) {
        setPoints(leftPoint, rightPoint);
        setFundamentalMatrix(fundamentalMatrix);
    }

    /**
     * Sets a matched pair of points to be corrected.
     *
     * @param leftPoint  matched point on left view to be corrected.
     * @param rightPoint matched point on right view to be corrected.
     */
    public final void setPoints(final Point2D leftPoint, final Point2D rightPoint) {
        this.leftPoint = leftPoint;
        this.rightPoint = rightPoint;
    }

    /**
     * Sets the fundamental matrix defining the epipolar geometry.
     *
     * @param fundamentalMatrix fundamental matrix to be set.
     */
    public final void setFundamentalMatrix(final FundamentalMatrix fundamentalMatrix) {
        this.fundamentalMatrix = fundamentalMatrix;
    }

    /**
     * Returns matched point on left view.
     *
     * @return matched point on left view.
     */
    public Point2D getLeftPoint() {
        return leftPoint;
    }

    /**
     * Returns matched point on right view.
     *
     * @return matched point on right view.
     */
    public Point2D getRightPoint() {
        return rightPoint;
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
     * Indicates whether this instance is ready to correct provided left
     * and right points using provided fundamental matrix.
     *
     * @return true if ready, false otherwise.
     */
    public boolean isReady() {
        return leftPoint != null && rightPoint != null && fundamentalMatrix != null
                && fundamentalMatrix.isInternalMatrixAvailable();
    }

    /**
     * Returns matched point on left view after correction.
     *
     * @return matched point on left view after correction.
     */
    public Point2D getLeftCorrectedPoint() {
        return leftCorrectedPoint;
    }

    /**
     * Returns matched point on right view after correction.
     *
     * @return matched point on right view after correction.
     */
    public Point2D getRightCorrectedPoint() {
        return rightCorrectedPoint;
    }

    /**
     * Corrects the pair of provided matched points to be corrected.
     *
     * @throws NotReadyException   if this instance is not ready (either points or
     *                             fundamental matrix has not been provided yet).
     * @throws CorrectionException if correction fails.
     */
    public abstract void correct() throws NotReadyException, CorrectionException;

    /**
     * Gets type of correction being used.
     *
     * @return type of correction.
     */
    public abstract CorrectorType getType();

    /**
     * Creates an instance of a single corrector using provided type.
     *
     * @param type a corrector type.
     * @return an instance of a single corrector.
     */
    public static SingleCorrector create(final CorrectorType type) {
        return type == CorrectorType.SAMPSON_CORRECTOR
                ? new SampsonSingleCorrector() : new GoldStandardSingleCorrector();
    }

    /**
     * Creates an instance of a single corrector using provided fundamental
     * matrix and provided type.
     *
     * @param fundamentalMatrix fundamental matrix defining the epipolar
     *                          geometry.
     * @param type              a corrector type.
     * @return an instance of a single corrector.
     */
    public static SingleCorrector create(final FundamentalMatrix fundamentalMatrix, final CorrectorType type) {
        return type == CorrectorType.SAMPSON_CORRECTOR
                ? new SampsonSingleCorrector(fundamentalMatrix) : new GoldStandardSingleCorrector(fundamentalMatrix);
    }

    /**
     * Creates an instance of a single corrector using provided left and right
     * points to be corrected, and provided type.
     *
     * @param leftPoint  matched point on left view to be corrected.
     * @param rightPoint matched point on right view to be corrected.
     * @param type       a corrector type.
     * @return an instance of a single corrector.
     */
    public static SingleCorrector create(final Point2D leftPoint, final Point2D rightPoint, final CorrectorType type) {
        return type == CorrectorType.SAMPSON_CORRECTOR
                ? new SampsonSingleCorrector(leftPoint, rightPoint)
                : new GoldStandardSingleCorrector(leftPoint, rightPoint);
    }

    /**
     * Creates an instance of a single corrector using provided left and right
     * points to be corrected, fundamental matrix and provided type.
     *
     * @param leftPoint         matched point on left view to be corrected.
     * @param rightPoint        matched point on right view to be corrected.
     * @param fundamentalMatrix fundamental matrix defining the epipolar
     *                          geometry.
     * @param type              a corrector type.
     * @return an instance of a single corrector.
     */
    public static SingleCorrector create(
            final Point2D leftPoint, final Point2D rightPoint, final FundamentalMatrix fundamentalMatrix,
            final CorrectorType type) {
        return type == CorrectorType.SAMPSON_CORRECTOR
                ? new SampsonSingleCorrector(leftPoint, rightPoint, fundamentalMatrix)
                : new GoldStandardSingleCorrector(leftPoint, rightPoint, fundamentalMatrix);
    }

    /**
     * Creates an instance of a single corrector using default type.
     *
     * @return an instance of a single corrector.
     */
    public static SingleCorrector create() {
        return create(DEFAULT_TYPE);
    }

    /**
     * Creates an instance of a single corrector using provided fundamental
     * matrix and default type.
     *
     * @param fundamentalMatrix fundamental matrix defining the epipolar
     *                          geometry.
     * @return an instance of a single corrector.
     */
    public static SingleCorrector create(final FundamentalMatrix fundamentalMatrix) {
        return create(fundamentalMatrix, DEFAULT_TYPE);
    }

    /**
     * Creates an instance of a single corrector using provided left and right
     * points to be corrected.
     *
     * @param leftPoint  matched point on left view to be corrected.
     * @param rightPoint matched point on right view to be corrected.
     * @return an instance of a single corrector.
     */
    public static SingleCorrector create(final Point2D leftPoint, final Point2D rightPoint) {
        return create(leftPoint, rightPoint, DEFAULT_TYPE);
    }

    /**
     * Creates an instance of a single corrector using provided left and right
     * points to be corrected, provided fundamental matrix and default type.
     *
     * @param leftPoint         matched point on left view to be corrected.
     * @param rightPoint        matched point on right view to be corrected.
     * @param fundamentalMatrix fundamental matrix defining the epipolar
     *                          geometry.
     * @return an instance of a single corrector.
     */
    public static SingleCorrector create(
            final Point2D leftPoint, final Point2D rightPoint,
            final FundamentalMatrix fundamentalMatrix) {
        return create(leftPoint, rightPoint, fundamentalMatrix, DEFAULT_TYPE);
    }
}
