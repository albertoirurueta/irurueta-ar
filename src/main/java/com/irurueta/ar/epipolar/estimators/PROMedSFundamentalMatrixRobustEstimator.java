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
import com.irurueta.numerical.robust.PROMedSRobustEstimator;
import com.irurueta.numerical.robust.PROMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best fundamental matrix for provided collections of matched 2D
 * points using PROMedS algorithm.
 */
public class PROMedSFundamentalMatrixRobustEstimator extends FundamentalMatrixRobustEstimator {

    /**
     * Default non-robust method to estimate a fundamental matrix.
     */
    public static final FundamentalMatrixEstimatorMethod DEFAULT_PROMEDS_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD =
            FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM;

    /**
     * Default value to be used for stop threshold. Stop threshold can be used
     * to keep the algorithm iterating in case that best estimated threshold
     * using median of residuals is not small enough. Once a solution is found
     * that generates a threshold below this value, the algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    public static final double DEFAULT_STOP_THRESHOLD = 1e-3;

    /**
     * Minimum allowed stop threshold value.
     */
    public static final double MIN_STOP_THRESHOLD = 0.0;

    /**
     * Threshold to be used to keep the algorithm iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once
     * a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    private double stopThreshold;

    /**
     * Quality scores corresponding to each provided point.
     * The larger the score value the better the quality of the sample.
     */
    private double[] qualityScores;

    /**
     * Constructor.
     *
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix
     *                                  estimator.
     */
    public PROMedSFundamentalMatrixRobustEstimator(final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod) {
        super(fundMatrixEstimatorMethod);
        stopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix
     *                                  estimator.
     * @param listener                  listener to be notified of events such as when
     *                                  estimation starts, ends or its progress significantly changes.
     */
    public PROMedSFundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod,
            final FundamentalMatrixRobustEstimatorListener listener) {
        super(fundMatrixEstimatorMethod, listener);
        stopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix
     *                                  estimator.
     * @param leftPoints                2D points on left view.
     * @param rightPoints               2D points on right view.
     * @throws IllegalArgumentException if provided list of points do not have
     *                                  the same length or their length is less than 8 points.
     */
    public PROMedSFundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        super(fundMatrixEstimatorMethod, leftPoints, rightPoints);
        if (leftPoints.size() < EightPointsFundamentalMatrixEstimator.MIN_REQUIRED_POINTS) {
            throw new IllegalArgumentException();
        }
        stopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix
     *                                  estimator.
     * @param leftPoints                2D points on left view.
     * @param rightPoints               2D points on right view.
     * @param listener                  listener to be notified of events such as when estimation
     *                                  starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided list of points do not have
     *                                  the same length or their length is less than 8 points.
     */
    public PROMedSFundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final FundamentalMatrixRobustEstimatorListener listener) {
        super(fundMatrixEstimatorMethod, leftPoints, rightPoints, listener);
        if (leftPoints.size() < EightPointsFundamentalMatrixEstimator.MIN_REQUIRED_POINTS) {
            throw new IllegalArgumentException();
        }
        stopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix
     *                                  estimator.
     * @param qualityScores             quality scores corresponding to each provided pair
     *                                  of matched points.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required size (i.e. 7 matched pair of points).
     */
    public PROMedSFundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod, final double[] qualityScores) {
        this(fundMatrixEstimatorMethod);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix
     *                                  estimator.
     * @param qualityScores             quality scores corresponding to each provided pair
     *                                  of matched points.
     * @param listener                  listener to be notified of events such as when
     *                                  estimation starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required size (i.e. 7 matched pair of points).
     */
    public PROMedSFundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod, final double[] qualityScores,
            final FundamentalMatrixRobustEstimatorListener listener) {
        this(fundMatrixEstimatorMethod, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix
     *                                  estimator.
     * @param qualityScores             quality scores corresponding to each provided pair
     *                                  of matched points.
     * @param leftPoints                2D points on left view.
     * @param rightPoints               2D points on right view.
     * @throws IllegalArgumentException if provided list of points or quality
     *                                  scores do not have the same length or their length is less
     *                                  than 7 points.
     */
    public PROMedSFundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod, final double[] qualityScores,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        this(fundMatrixEstimatorMethod, leftPoints, rightPoints);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix
     *                                  estimator.
     * @param qualityScores             quality scores corresponding to each provided pair
     *                                  of matched points.
     * @param leftPoints                2D points on left view.
     * @param rightPoints               2D points on right view.
     * @param listener                  listener to be notified of events such as when estimation
     *                                  starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided list of points or quality
     *                                  scores do not have the same length or their length is less
     *                                  than 7 points.
     */
    public PROMedSFundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod, final double[] qualityScores,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final FundamentalMatrixRobustEstimatorListener listener) {
        this(fundMatrixEstimatorMethod, leftPoints, rightPoints, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     */
    public PROMedSFundamentalMatrixRobustEstimator() {
        this(DEFAULT_PROMEDS_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when
     *                 estimation starts, ends or its progress significantly changes.
     */
    public PROMedSFundamentalMatrixRobustEstimator(final FundamentalMatrixRobustEstimatorListener listener) {
        this(DEFAULT_PROMEDS_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, listener);
    }

    /**
     * Constructor.
     *
     * @param leftPoints  2D points on left view.
     * @param rightPoints 2D points on right view.
     * @throws IllegalArgumentException if provided list of points do not have
     *                                  the same length or their length is less than 8 points.
     */
    public PROMedSFundamentalMatrixRobustEstimator(final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        this(DEFAULT_PROMEDS_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, leftPoints, rightPoints);
    }

    /**
     * Constructor.
     *
     * @param leftPoints  2D points on left view.
     * @param rightPoints 2D points on right view.
     * @param listener    listener to be notified of events such as when estimation
     *                    starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided list of points do not have
     *                                  the same length or their length is less than 8 points.
     */
    public PROMedSFundamentalMatrixRobustEstimator(final List<Point2D> leftPoints, final List<Point2D> rightPoints,
                                                   final FundamentalMatrixRobustEstimatorListener listener) {
        this(DEFAULT_PROMEDS_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, leftPoints, rightPoints, listener);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided pair
     *                      of matched points.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required size (i.e. 7 matched pair of points).
     */
    public PROMedSFundamentalMatrixRobustEstimator(final double[] qualityScores) {
        this(DEFAULT_PROMEDS_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided pair
     *                      of matched points.
     * @param listener      listener to be notified of events such as when
     *                      estimation starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required size (i.e. 7 matched pair of points).
     */
    public PROMedSFundamentalMatrixRobustEstimator(
            final double[] qualityScores, final FundamentalMatrixRobustEstimatorListener listener) {
        this(DEFAULT_PROMEDS_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, qualityScores, listener);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided pair
     *                      of matched points.
     * @param leftPoints    2D points on left view.
     * @param rightPoints   2D points on right view.
     * @throws IllegalArgumentException if provided list of points or quality
     *                                  scores do not have the same length or their length is less
     *                                  than 7 points.
     */
    public PROMedSFundamentalMatrixRobustEstimator(
            final double[] qualityScores, final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        this(DEFAULT_PROMEDS_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, qualityScores, leftPoints, rightPoints);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided pair
     *                      of matched points.
     * @param leftPoints    2D points on left view.
     * @param rightPoints   2D points on right view.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided list of points or quality
     *                                  scores do not have the same length or their length is less
     *                                  than 7 points.
     */
    public PROMedSFundamentalMatrixRobustEstimator(
            final double[] qualityScores, final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final FundamentalMatrixRobustEstimatorListener listener) {
        this(DEFAULT_PROMEDS_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, qualityScores, leftPoints, rightPoints, listener);
    }

    /**
     * Sets matched 2D points on both left and right views.
     *
     * @param leftPoints  matched 2D points on left view.
     * @param rightPoints matched 2D points on right view.
     * @throws LockedException          if this fundamental matrix estimator is locked.
     * @throws IllegalArgumentException if provided matched points on left and
     *                                  right views do not have the same length or if their length is
     *                                  less than 8 points.
     */
    @Override
    public void setPoints(final List<Point2D> leftPoints, final List<Point2D> rightPoints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (leftPoints.size() < EightPointsFundamentalMatrixEstimator.MIN_REQUIRED_POINTS) {
            throw new IllegalArgumentException();
        }
        super.setPoints(leftPoints, rightPoints);
    }


    /**
     * Returns threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     *
     * @return stop threshold to stop the algorithm prematurely when a certain
     * accuracy has been reached.
     */
    public double getStopThreshold() {
        return stopThreshold;
    }

    /**
     * Sets threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     *
     * @param stopThreshold stop threshold to stop the algorithm prematurely
     *                      when a certain accuracy has been reached.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     */
    public void setStopThreshold(final double stopThreshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (stopThreshold <= MIN_STOP_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        this.stopThreshold = stopThreshold;
    }

    /**
     * Returns quality scores corresponding to each provided pair of points.
     * The larger the score value the better the quality of the sampled matched
     * pair of points.
     *
     * @return quality scores corresponding to each pair of points.
     */
    @Override
    public double[] getQualityScores() {
        return qualityScores;
    }

    /**
     * Sets quality scores corresponding to each provided pair of points.
     * The larger the score value the better the quality of the sampled matched
     * pair of points.
     *
     * @param qualityScores quality scores corresponding to each pair of points.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    @Override
    public void setQualityScores(final double[] qualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Returns value indicating whether required data has been provided so that
     * fundamental matrix estimation can start.
     * This is true when input data (i.e. 7 pairs of matched 2D points and their
     * quality scores) are provided.
     * If true, estimator is ready to compute a fundamental matrix, otherwise
     * more data needs to be provided.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && qualityScores != null && qualityScores.length == leftPoints.size();
    }

    /**
     * Estimates a radial distortion using a robust estimator and
     * the best set of matched 2D points found using the robust estimator.
     *
     * @return a radial distortion.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public FundamentalMatrix estimate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var innerEstimator = new PROMedSRobustEstimator<FundamentalMatrix>(
                new PROMedSRobustEstimatorListener<>() {

                    // subset of left points
                    private final List<Point2D> subsetLeftPoints = new ArrayList<>();

                    // subset of right points
                    private final List<Point2D> subsetRightPoints = new ArrayList<>();

                    @Override
                    public double getThreshold() {
                        return stopThreshold;
                    }

                    @Override
                    public int getTotalSamples() {
                        return leftPoints.size();
                    }

                    @Override
                    public int getSubsetSize() {
                        return getMinRequiredPoints();
                    }

                    @Override
                    public void estimatePreliminarSolutions(
                            final int[] samplesIndices, final List<FundamentalMatrix> solutions) {

                        subsetLeftPoints.clear();
                        subsetRightPoints.clear();
                        for (final var samplesIndex : samplesIndices) {
                            subsetLeftPoints.add(leftPoints.get(samplesIndex));
                            subsetRightPoints.add(rightPoints.get(samplesIndex));
                        }

                        nonRobustEstimate(solutions, subsetLeftPoints, subsetRightPoints);
                    }

                    @Override
                    public double computeResidual(final FundamentalMatrix currentEstimation, final int i) {
                        final var leftPoint = leftPoints.get(i);
                        final var rightPoint = rightPoints.get(i);
                        return residual(currentEstimation, leftPoint, rightPoint);
                    }

                    @Override
                    public boolean isReady() {
                        return PROMedSFundamentalMatrixRobustEstimator.this.isReady();
                    }

                    @Override
                    public void onEstimateStart(final RobustEstimator<FundamentalMatrix> estimator) {
                        if (listener != null) {
                            listener.onEstimateStart(PROMedSFundamentalMatrixRobustEstimator.this);
                        }
                    }

                    @Override
                    public void onEstimateEnd(final RobustEstimator<FundamentalMatrix> estimator) {
                        if (listener != null) {
                            listener.onEstimateEnd(PROMedSFundamentalMatrixRobustEstimator.this);
                        }
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final RobustEstimator<FundamentalMatrix> estimator, final int iteration) {
                        if (listener != null) {
                            listener.onEstimateNextIteration(
                                    PROMedSFundamentalMatrixRobustEstimator.this, iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final RobustEstimator<FundamentalMatrix> estimator, final float progress) {
                        if (listener != null) {
                            listener.onEstimateProgressChange(
                                    PROMedSFundamentalMatrixRobustEstimator.this, progress);
                        }
                    }

                    @Override
                    public double[] getQualityScores() {
                        return qualityScores;
                    }
                });

        try {
            locked = true;
            inliersData = null;
            innerEstimator.setConfidence(confidence);
            innerEstimator.setMaxIterations(maxIterations);
            innerEstimator.setProgressDelta(progressDelta);
            final var result = innerEstimator.estimate();
            inliersData = innerEstimator.getInliersData();
            return attemptRefine(result);
        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } finally {
            locked = false;
        }
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROMEDS;
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
    @Override
    protected double getRefinementStandardDeviation() {
        final var inliersData = (PROMedSRobustEstimator.PROMedSInliersData) getInliersData();
        return inliersData.getEstimatedThreshold();
    }

    /**
     * Sets quality scores corresponding to each provided pair of matched
     * points.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than 8 points.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores.length < getMinRequiredPoints()) {
            throw new IllegalArgumentException();
        }

        this.qualityScores = qualityScores;
    }
}
