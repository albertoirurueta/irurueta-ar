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

import com.irurueta.algebra.Matrix;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.refiners.FundamentalMatrixRefiner;
import com.irurueta.geometry.GeometryException;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best
 * Fundamental matrix for provided collections of matched 2D points.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
public abstract class FundamentalMatrixRobustEstimator {

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD = RobustEstimatorMethod.PROSAC;

    /**
     * Default non-robust method to estimate a fundamental matrix.
     */
    public static final FundamentalMatrixEstimatorMethod DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD =
            FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM;

    /**
     * Indicates that result is refined by default using Levenberg-Marquardt
     * fitting algorithm over found inliers.
     */
    public static final boolean DEFAULT_REFINE_RESULT = true;

    /**
     * Indicates that covariance is not kept by default after refining result.
     */
    public static final boolean DEFAULT_KEEP_COVARIANCE = false;

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
     * Constant defining default confidence of the estimated result, which is
     * 99%. This means that with a probability of 99% estimation will be
     * accurate because chosen sub-samples will be inliers.
     */
    public static final double DEFAULT_CONFIDENCE = 0.99;

    /**
     * Default maximum allowed number of iterations.
     */
    public static final int DEFAULT_MAX_ITERATIONS = 5000;

    /**
     * Minimum allowed confidence value.
     */
    public static final double MIN_CONFIDENCE = 0.0;

    /**
     * Maximum allowed confidence value.
     */
    public static final double MAX_CONFIDENCE = 1.0;

    /**
     * Minimum allowed number of iterations.
     */
    public static final int MIN_ITERATIONS = 1;

    /**
     * List of 2D points corresponding to left view.
     */
    protected List<Point2D> leftPoints;

    /**
     * List of 2D points corresponding to right view.
     */
    protected List<Point2D> rightPoints;

    /**
     * Listener to be notified of events such as when estimation starts, ends or
     * its progress significantly changes.
     */
    protected FundamentalMatrixRobustEstimatorListener listener;

    /**
     * Indicates if this estimator is locked because an estimation is being
     * computed.
     */
    protected boolean locked;

    /**
     * Amount of progress variation before notifying a progress change during
     * estimation.
     */
    protected float progressDelta;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%). The amount of confidence indicates the probability
     * that the estimated result is correct. Usually this value will be close
     * to 1.0, but not exactly 1.0.
     */
    protected double confidence;

    /**
     * Maximum allowed number of iterations. When the maximum number of
     * iterations is exceeded, result will not be available, however an
     * approximate result will be available for retrieval.
     */
    protected int maxIterations;

    /**
     * Data related to inliers found after estimation.
     */
    protected InliersData inliersData;

    /**
     * Indicates whether result must be refined using Levenberg-Marquardt
     * fitting algorithm over found inliers.
     * If true, inliers will be computed and kept in any implementation
     * regardless of the settings.
     */
    protected boolean refineResult;

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    private boolean keepCovariance;

    /**
     * Estimated covariance of estimated fundamental matrix.
     * This is only available when result has been refined and covariance is
     * kept.
     */
    private Matrix covariance;

    /**
     * Test line to compute epipolar residuals.
     */
    private final Line2D testLine = new Line2D();

    /**
     * Internal non robust estimator of fundamental matrix.
     */
    private FundamentalMatrixEstimator fundMatrixEstimator;

    /**
     * Constructor.
     *
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix
     *                                  estimator.
     */
    protected FundamentalMatrixRobustEstimator(final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod) {
        progressDelta = DEFAULT_PROGRESS_DELTA;
        confidence = DEFAULT_CONFIDENCE;
        maxIterations = DEFAULT_MAX_ITERATIONS;
        fundMatrixEstimator = FundamentalMatrixEstimator.create(fundMatrixEstimatorMethod);
        refineResult = DEFAULT_REFINE_RESULT;
        keepCovariance = DEFAULT_KEEP_COVARIANCE;
    }

    /**
     * Constructor.
     *
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix
     *                                  estimator.
     * @param listener                  listener to be notified of events such as when estimation
     *                                  starts, ends or its progress significantly changes.
     */
    protected FundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod,
            final FundamentalMatrixRobustEstimatorListener listener) {
        this(fundMatrixEstimatorMethod);
        this.listener = listener;
    }

    /**
     * Constructor with matched 2D points.
     *
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix
     *                                  estimator.
     * @param leftPoints                2D points on left view.
     * @param rightPoints               2D points on right view.
     * @throws IllegalArgumentException if provided list of points do not have
     *                                  the same length or their length is less than 7 points.
     */
    protected FundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        this(fundMatrixEstimatorMethod);
        internalSetPoints(leftPoints, rightPoints);
    }

    /**
     * Constructor with matched 2D points.
     *
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix
     *                                  estimator.
     * @param leftPoints                2D points on left view.
     * @param rightPoints               2D points on right view.
     * @param listener                  listener to be notified of events such as when estimation
     *                                  starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided list of points do not have
     *                                  the same length or their length is less than 7 points.
     */
    protected FundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final FundamentalMatrixRobustEstimatorListener listener) {
        this(fundMatrixEstimatorMethod, listener);
        internalSetPoints(leftPoints, rightPoints);
    }

    /**
     * Constructor.
     */
    protected FundamentalMatrixRobustEstimator() {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
    }
    
    /**
     * Returns non-robust method to estimate a fundamental matrix.
     *
     * @return non-robust method to estimate a fundamental matrix.
     */
    public FundamentalMatrixEstimatorMethod getNonRobustFundamentalMatrixEstimatorMethod() {
        return fundMatrixEstimator.getMethod();
    }

    /**
     * Sets non-robust method to estimate a fundamental matrix.
     *
     * @param method non-robust method to estimate a fundamental matrix.
     * @throws LockedException if this fundamental matrix estimator is locked.
     */
    public void setNonRobustFundamentalMatrixEstimatorMethod(final FundamentalMatrixEstimatorMethod method)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        if (method != getNonRobustFundamentalMatrixEstimatorMethod()) {
            // if method changes, recreate internal non-robust fundamental matrix
            // estimator
            fundMatrixEstimator = FundamentalMatrixEstimator.create(method);
        }
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
     *                                  right views do not have the same length or if their length is
     *                                  less than 7 points.
     */
    public void setPoints(final List<Point2D> leftPoints, final List<Point2D> rightPoints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetPoints(leftPoints, rightPoints);
    }

    /**
     * Returns reference to listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     *
     * @return listener to be notified of events.
     */
    public FundamentalMatrixRobustEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or its progress significantly changes.
     *
     * @param listener listener to be notified of events.
     * @throws LockedException if robust estimator is locked.
     */
    public void setListener(final FundamentalMatrixRobustEstimatorListener listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.listener = listener;
    }

    /**
     * Indicates whether listener has been provided and is available for
     * retrieval.
     *
     * @return true if available, false otherwise.
     */
    public boolean isListenerAvailable() {
        return listener != null;
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
     * Returns amount of confidence expressed as a value between 0.0 and 1.0
     * (which is equivalent to 100%). The amount of confidence indicates the
     * probability that the estimated result is correct. Usually this value will
     * be close to 1.0, but not exactly 1.0.
     *
     * @return amount of confidence as a value between 0.0 and 1.0.
     */
    public double getConfidence() {
        return confidence;
    }

    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which
     * is equivalent to 100%). The amount of confidence indicates the
     * probability that the estimated result is correct. Usually this value will
     * be close to 1.0, but not exactly 1.0.
     *
     * @param confidence confidence to be set as a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and
     *                                  1.0.
     * @throws LockedException          if this estimator is locked because an estimator
     *                                  is being computed.
     */
    public void setConfidence(final double confidence) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (confidence < MIN_CONFIDENCE || confidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        this.confidence = confidence;
    }

    /**
     * Returns maximum allowed number of iterations. If maximum allowed number
     * of iterations is achieved without converting to a result when calling
     * estimate(), a RobustEstimatorException will be raised.
     *
     * @return maximum allowed number of iterations.
     */
    public int getMaxIterations() {
        return maxIterations;
    }

    /**
     * Sets maximum allowed number of iterations. When the maximum number of
     * iterations is exceeded, result will not be available, however an
     * approximate result will be available for retrieval.
     *
     * @param maxIterations maximum allowed number of iterations to be set.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException          if this estimator is locked because an estimation
     *                                  is being computed.
     */
    public void setMaxIterations(final int maxIterations) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (maxIterations < MIN_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        this.maxIterations = maxIterations;
    }

    /**
     * Gets data related to inliers found after estimation.
     *
     * @return data related to inliers found after estimation.
     */
    public InliersData getInliersData() {
        return inliersData;
    }

    /**
     * Indicates whether result must be refined using Levenberg-Marquardt
     * fitting algorithm over found inliers.
     * If true, inliers will be computed and kept in any implementation
     * regardless of the settings.
     *
     * @return true to refine result, false to simply use result found by
     * robust estimator without further refining.
     */
    public boolean isResultRefined() {
        return refineResult;
    }

    /**
     * Specifies whether result must be refined using Levenberg-Marquardt
     * fitting algorithm over found inliers.
     *
     * @param refineResult true to refine result, false to simply use result
     *                     found by robust estimator without further refining.
     * @throws LockedException if estimator is locked.
     */
    public void setResultRefined(final boolean refineResult) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.refineResult = refineResult;
    }

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @return true if covariance must be kept after refining result, false
     * otherwise.
     */
    public boolean isCovarianceKept() {
        return keepCovariance;
    }

    /**
     * Specifies whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @param keepCovariance true if covariance must be kept after refining
     *                       result, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setCovarianceKept(final boolean keepCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.keepCovariance = keepCovariance;
    }

    /**
     * Returns minimum number of matched pair of points required to start
     * the estimation.
     *
     * @return minimum number of matched pair of points required to start
     * the estimation.
     */
    public int getMinRequiredPoints() {
        return fundMatrixEstimator.getMinRequiredPoints();
    }

    /**
     * Returns value indicating whether required data has been provided so that
     * fundamental matrix estimation can start.
     * If true, estimator is ready to compute a fundamental matrix, otherwise
     * more data needs to be provided.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return leftPoints != null && rightPoints != null && leftPoints.size() == rightPoints.size()
                && leftPoints.size() >= SevenPointsFundamentalMatrixEstimator.MIN_REQUIRED_POINTS;
    }

    /**
     * Returns quality scores corresponding to each pair of matched points.
     * The larger the score value the better the quality of the pair of matched
     * points.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @return quality scores corresponding to each pair of matched points.
     */
    public double[] getQualityScores() {
        return null;
    }

    /**
     * Sets quality scores corresponding to each pair of matched points.
     * The larger the score value the better the quality of the pair of matched
     * points.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than minimum required number of homographies.
     */
    public void setQualityScores(final double[] qualityScores) throws LockedException {
    }

    /**
     * Gets estimated covariance of estimated fundamental matrix if available.
     * This is only available when result has been refined and covariance is
     * kept.
     *
     * @return estimated covariance or null.
     */
    public Matrix getCovariance() {
        return covariance;
    }

    /**
     * Estimates fundamental matrix.
     *
     * @return estimated fundamental matrix.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    public abstract FundamentalMatrix estimate() throws LockedException, NotReadyException, RobustEstimatorException;

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Creates a fundamental matrix robust estimator using provided method.
     *
     * @param method method of a robust estimator algorithm to estimate the best
     *               fundamental matrix.
     * @return an instance of a fundamental matrix robust estimator.
     */
    public static FundamentalMatrixRobustEstimator create(final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSFundamentalMatrixRobustEstimator();
            case MSAC -> new MSACFundamentalMatrixRobustEstimator();
            case PROSAC -> new PROSACFundamentalMatrixRobustEstimator();
            case PROMEDS -> new PROMedSFundamentalMatrixRobustEstimator();
            default -> new RANSACFundamentalMatrixRobustEstimator();
        };
    }

    /**
     * Creates a fundamental matrix robust estimator using provided lists of
     * matched points and provided method.
     *
     * @param leftPoints  2D points on left view.
     * @param rightPoints 2D points on left view.
     * @param method      method of a robust estimator algorithm to estimate the best
     *                    fundamental matrix.
     * @return an instance of a fundamental matrix robust estimator.
     * @throws IllegalArgumentException if provided list of points do not have
     *                                  the same length or their length is less than 7 points.
     */
    public static FundamentalMatrixRobustEstimator create(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSFundamentalMatrixRobustEstimator(leftPoints, rightPoints);
            case MSAC -> new MSACFundamentalMatrixRobustEstimator(leftPoints, rightPoints);
            case PROSAC -> new PROSACFundamentalMatrixRobustEstimator(leftPoints, rightPoints);
            case PROMEDS -> new PROMedSFundamentalMatrixRobustEstimator(leftPoints, rightPoints);
            default -> new RANSACFundamentalMatrixRobustEstimator(leftPoints, rightPoints);
        };
    }

    /**
     * Creates a fundamental matrix robust estimator using provided lists of
     * matched points and provided method.
     *
     * @param leftPoints    2D points on left view.
     * @param rightPoints   2D points on left view.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @param method        method of a robust estimator algorithm to estimate the best
     *                      fundamental matrix.
     * @return an instance of a fundamental matrix robust estimator.
     * @throws IllegalArgumentException if provided list of points do not have
     *                                  the same length or their length is less than 7 points.
     */
    public static FundamentalMatrixRobustEstimator create(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSFundamentalMatrixRobustEstimator(leftPoints, rightPoints);
            case MSAC -> new MSACFundamentalMatrixRobustEstimator(leftPoints, rightPoints);
            case PROSAC -> new PROSACFundamentalMatrixRobustEstimator(qualityScores, leftPoints, rightPoints);
            case PROMEDS -> new PROMedSFundamentalMatrixRobustEstimator(qualityScores, leftPoints, rightPoints);
            default -> new RANSACFundamentalMatrixRobustEstimator(leftPoints, rightPoints);
        };
    }

    /**
     * Creates a fundamental matrix robust estimator using default method.
     *
     * @return an instance of a fundamental matrix robust estimator.
     */
    public static FundamentalMatrixRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a fundamental matrix robust estimator using provided lists of
     * matched points and default method.
     *
     * @param leftPoints  2D points on left view.
     * @param rightPoints 2D points on left view.
     * @return an instance of a fundamental matrix robust estimator.
     * @throws IllegalArgumentException if provided list of points do not have
     *                                  the same length or their length is less than 7 points.
     */
    public static FundamentalMatrixRobustEstimator create(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        return create(leftPoints, rightPoints, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a fundamental matrix robust estimator using provided lists of
     * matched points and default method.
     *
     * @param leftPoints    2D points on left view.
     * @param rightPoints   2D points on left view.
     * @param qualityScores quality scores corresponding to each pair of matched
     *                      points.
     * @return an instance of a fundamental matrix robust estimator.
     * @throws IllegalArgumentException if provided list of points do not have
     *                                  the same length or their length is less than 7 points.
     */
    public static FundamentalMatrixRobustEstimator create(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints, final double[] qualityScores) {
        return create(leftPoints, rightPoints, qualityScores, DEFAULT_ROBUST_METHOD);
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
    @SuppressWarnings("DuplicatedCode")
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
     * Estimates a fundamental matrix using a non-robust method and provided
     * subset of matched points and stores the solution in provided array of
     * solutions.
     *
     * @param solutions         list where solutions will be stored.
     * @param subsetLeftPoints  subset of left view matched points.
     * @param subsetRightPoints subset of right view matched points.
     */
    protected void nonRobustEstimate(
            final List<FundamentalMatrix> solutions, final List<Point2D> subsetLeftPoints,
            final List<Point2D> subsetRightPoints) {
        try {
            fundMatrixEstimator.setPoints(subsetLeftPoints, subsetRightPoints);
            if (fundMatrixEstimator.getMethod() == FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM) {
                final var matrices = ((SevenPointsFundamentalMatrixEstimator) fundMatrixEstimator).estimateAll();
                solutions.addAll(matrices);
            } else {
                solutions.add(fundMatrixEstimator.estimate());
            }
        } catch (final GeometryException e) {
            // if anything fails, no solution is added
        }
    }

    /**
     * Attempts to refine provided solution if refinement is requested.
     * This method returns a refined solution or the same provided solution
     * if refinement is not requested or has failed.
     * If refinement is enabled, and it is requested to keep covariance, this
     * method will also keep covariance of refined fundamental matrix.
     *
     * @param fundamentalMatrix fundamental matrix estimated by a robust
     *                          estimator without refinement.
     * @return solution after refinement (if requested) or the provided
     * non-refined solution if not requested or if refinement failed.
     */
    protected FundamentalMatrix attemptRefine(final FundamentalMatrix fundamentalMatrix) {
        if (refineResult) {
            final var refiner = new FundamentalMatrixRefiner(fundamentalMatrix, keepCovariance, getInliersData(),
                    leftPoints, rightPoints, getRefinementStandardDeviation());

            try {
                final var result = new FundamentalMatrix();
                final var improved = refiner.refine(result);

                if (keepCovariance) {
                    // keep covariance
                    covariance = refiner.getCovariance();
                }

                return improved ? result : fundamentalMatrix;
            } catch (final Exception e) {
                // refinement failed, so we return input value
                return fundamentalMatrix;
            }

        } else {
            return fundamentalMatrix;
        }
    }

    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically, this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of
     * such threshold.
     *
     * @return standard deviation used for refinement.
     */
    protected abstract double getRefinementStandardDeviation();

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
        if (leftPoints.size() < getMinRequiredPoints()) {
            throw new IllegalArgumentException();
        }

        this.leftPoints = leftPoints;
        this.rightPoints = rightPoints;
    }
}
