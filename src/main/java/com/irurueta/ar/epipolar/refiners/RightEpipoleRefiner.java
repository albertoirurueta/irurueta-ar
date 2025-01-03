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
package com.irurueta.ar.epipolar.refiners;

import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidFundamentalMatrixException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Transformation2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.refiners.PairMatchesAndInliersDataRefiner;
import com.irurueta.geometry.refiners.RefinerException;
import com.irurueta.numerical.robust.InliersData;

import java.util.BitSet;
import java.util.List;

/**
 * Base class to refine the epipole of a fundamental matrix formed by an initial
 * epipole estimation and an estimated homography.
 * Any fundamental matrix can be expressed as F = [e']x*H, where
 * e' is the epipole on the right view and H is a non-degenerate homography.
 * This class refines an initial epipole so that residuals from provided point
 * correspondences generating fundamental matrix F are reduced.
 * This class is especially useful in cases where geometry of the scene is
 * degenerate (e.g. planar scene) and provided point correspondences would
 * generate an inaccurate fundamental matrix.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class RightEpipoleRefiner extends PairMatchesAndInliersDataRefiner<Point2D, Point2D, Point2D> {

    /**
     * Test line to compute epipolar residuals.
     */
    protected final Line2D testLine = new Line2D();

    /**
     * Homography relating two views through a given planar scene.
     */
    protected Transformation2D homography;

    /**
     * Standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically, this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of
     * such threshold.
     */
    private double refinementStandardDeviation;

    /**
     * Constructor.
     */
    protected RightEpipoleRefiner() {
    }

    /**
     * Constructor.
     *
     * @param initialEpipoleEstimation    initial right epipole estimation to be
     *                                    set and refined.
     * @param keepCovariance              true if covariance of estimation must be kept after
     *                                    refinement, false otherwise.
     * @param inliers                     set indicating which of the provided matches are inliers.
     * @param residuals                   residuals for matched samples.
     * @param numInliers                  number of inliers on initial estimation.
     * @param samples1                    1st set of paired samples.
     * @param samples2                    2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for
     *                                    Levenberg-Marquardt fitting.
     * @param homography                  homography relating samples in two views, which is used
     *                                    to generate a fundamental matrix and its corresponding
     *                                    epipolar geometry.
     */
    protected RightEpipoleRefiner(
            final Point2D initialEpipoleEstimation, final boolean keepCovariance, final BitSet inliers,
            final double[] residuals, final int numInliers, final List<Point2D> samples1, final List<Point2D> samples2,
            final double refinementStandardDeviation, final Transformation2D homography) {
        super(initialEpipoleEstimation, keepCovariance, inliers, residuals, numInliers, samples1, samples2);
        this.homography = homography;
        this.refinementStandardDeviation = refinementStandardDeviation;
    }

    /**
     * Constructor.
     *
     * @param initialEpipoleEstimation    initial right epipole estimation to be
     *                                    set and refined.
     * @param keepCovariance              true if covariance of estimation must be kept after
     *                                    refinement, false otherwise.
     * @param inliersData                 inlier data, typically obtained from a robust
     *                                    estimator.
     * @param samples1                    1st set of paired samples.
     * @param samples2                    2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for
     *                                    Levenberg-Marquardt fitting.
     * @param homography                  homography relating samples in two views, which is used
     *                                    to generate a fundamental matrix and its corresponding
     *                                    epipolar geometry.
     */
    protected RightEpipoleRefiner(
            final Point2D initialEpipoleEstimation, final boolean keepCovariance, final InliersData inliersData,
            final List<Point2D> samples1, final List<Point2D> samples2, final double refinementStandardDeviation,
            final Transformation2D homography) {
        super(initialEpipoleEstimation, keepCovariance, inliersData, samples1, samples2);
        this.homography = homography;
        this.refinementStandardDeviation = refinementStandardDeviation;
    }

    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically, this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of such
     * threshold.
     *
     * @return standard deviation used for refinement.
     */
    public double getRefinementStandardDeviation() {
        return refinementStandardDeviation;
    }

    /**
     * Sets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically, this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of such
     * threshold.
     *
     * @param refinementStandardDeviation standard deviation used for
     *                                    refinement.
     * @throws LockedException if estimator is locked.
     */
    public void setRefinementStandardDeviation(final double refinementStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.refinementStandardDeviation = refinementStandardDeviation;
    }

    /**
     * Gets homography relating samples in two views, which is used to generate
     * a fundamental matrix and its corresponding epipolar geometry.
     *
     * @return homography relating samples in two views.
     */
    public Transformation2D getHomography() {
        return homography;
    }

    /**
     * Sets homography relating samples in two views, which is used to generate
     * a fundamental matrix and its corresponding epipolar geometry.
     *
     * @param homography homography relating samples in two views.
     * @throws LockedException if estimator is locked.
     */
    public void setHomography(final Transformation2D homography) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.homography = homography;
    }

    /**
     * Indicates whether this refiner is ready to start refinement computation.
     *
     * @return true if refiner is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return homography != null && super.isReady();
    }

    /**
     * Refines provided initial right epipole estimation.
     *
     * @return refined estimation.
     * @throws NotReadyException if not enough input data has been provided.
     * @throws LockedException   if estimator is locked because refinement is
     *                           already in progress.
     * @throws RefinerException  if refinement fails for some reason (e.g. unable
     *                           to converge to a result).
     */
    @Override
    public Point2D refine() throws NotReadyException, LockedException, RefinerException {
        final var result = new HomogeneousPoint2D();
        refine(result);
        return result;
    }

    /**
     * Computes a fundamental matrix from a 2D homography and provided epipole
     * on right view.
     *
     * @param homography   a 2D homography. Must be invertible.
     * @param rightEpipole epipole on right view.
     * @param result       instance where computed fundamental matrix will be stored.
     * @throws InvalidFundamentalMatrixException if provided homography is not
     *                                           invertible, which would generate a degenerate
     *                                           fundamental matrix.
     */
    public static void computeFundamentalMatrix(
            final Transformation2D homography, final Point2D rightEpipole, final FundamentalMatrix result)
            throws InvalidFundamentalMatrixException {

        result.setFromHomography(homography, rightEpipole);
    }

    /**
     * Computes the residual between a fundamental matrix and a pair of matched
     * points.
     *
     * @param fundamentalMatrix a fundamental matrix.
     * @param leftPoint         left 2D point.
     * @param rightPoint        right 2D point.
     * @return residual (distance of point to epipolar line).
     */
    protected double residual(
            final FundamentalMatrix fundamentalMatrix, final Point2D leftPoint, final Point2D rightPoint) {
        try {
            leftPoint.normalize();
            rightPoint.normalize();
            fundamentalMatrix.normalize();
            fundamentalMatrix.leftEpipolarLine(rightPoint, testLine);
            final var leftDistance = Math.abs(testLine.signedDistance(leftPoint));
            fundamentalMatrix.rightEpipolarLine(leftPoint, testLine);
            final var rightDistance = Math.abs(testLine.signedDistance(rightPoint));
            // return average distance as an error residual
            return 0.5 * (leftDistance + rightDistance);
        } catch (final NotReadyException e) {
            return Double.MAX_VALUE;
        }
    }

    /**
     * Computes total residual among all provided inlier samples.
     *
     * @param fundamentalMatrix a fundamental matrix.
     * @return total residual.
     */
    protected double totalResidual(final FundamentalMatrix fundamentalMatrix) {
        var result = 0.0;

        final var nSamples = inliers.length();
        for (var i = 0; i < nSamples; i++) {
            if (inliers.get(i)) {
                // sample is inlier
                final var leftPoint = samples1.get(i);
                final var rightPoint = samples2.get(i);
                leftPoint.normalize();
                rightPoint.normalize();
                result += residual(fundamentalMatrix, leftPoint, rightPoint);
            }
        }

        return result;
    }
}
