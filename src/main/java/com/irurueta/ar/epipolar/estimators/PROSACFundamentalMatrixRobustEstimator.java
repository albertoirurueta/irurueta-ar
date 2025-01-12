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
import com.irurueta.numerical.robust.PROSACRobustEstimator;
import com.irurueta.numerical.robust.PROSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best fundamental matrix for provided collections of matched 2D
 * points using PROSAC algorithm.
 */
public class PROSACFundamentalMatrixRobustEstimator extends FundamentalMatrixRobustEstimator {

    /**
     * Constant defining default threshold to determine whether points are
     * inliers or not.
     * By default, 1.0 is considered a good value for cases where measures are
     * done in pixels, since typically the minimum resolution is 1 pixel.
     */
    public static final double DEFAULT_THRESHOLD = 1.0;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Indicates that by default inliers will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_INLIERS = false;

    /**
     * Indicates that by default residuals will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_RESIDUALS = false;

    /**
     * Threshold to determine whether pairs of matched points are inliers or not
     * when testing possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance) a given
     * point has respect to the epipolar line generated by its matched point.
     */
    private double threshold;

    /**
     * Quality scores corresponding to each provided point.
     * The larger the score value the better the quality of the sample.
     */
    private double[] qualityScores;

    /**
     * Indicates whether inliers must be computed and kept.
     */
    private boolean computeAndKeepInliers;

    /**
     * Indicates whether residuals must be computed and kept.
     */
    private boolean computeAndKeepResiduals;

    /**
     * Constructor.
     *
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix
     *                                  estimator.
     */
    public PROSACFundamentalMatrixRobustEstimator(final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod) {
        super(fundMatrixEstimatorMethod);
        threshold = DEFAULT_THRESHOLD;
        computeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        computeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
    }

    /**
     * Constructor.
     *
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix
     *                                  estimator.
     * @param listener                  listener to be notified of events such as when
     *                                  estimation starts, ends or its progress significantly changes.
     */
    public PROSACFundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod,
            final FundamentalMatrixRobustEstimatorListener listener) {
        super(fundMatrixEstimatorMethod, listener);
        threshold = DEFAULT_THRESHOLD;
        computeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        computeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
    }

    /**
     * Constructor.
     *
     * @param fundMatrixEstimatorMethod method for non-robust fundamental matrix
     *                                  estimator.
     * @param leftPoints                2D points on left view.
     * @param rightPoints               2D points on right view.
     * @throws IllegalArgumentException if provided list of points do not have
     *                                  the same length or their length is less than 7 points.
     */
    public PROSACFundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        super(fundMatrixEstimatorMethod, leftPoints, rightPoints);
        threshold = DEFAULT_THRESHOLD;
        computeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        computeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
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
     *                                  the same length or their length is less than 7 points.
     */
    public PROSACFundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final FundamentalMatrixRobustEstimatorListener listener) {
        super(fundMatrixEstimatorMethod, leftPoints, rightPoints, listener);
        threshold = DEFAULT_THRESHOLD;
        computeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        computeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
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
    public PROSACFundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod, final double[] qualityScores) {
        this(fundMatrixEstimatorMethod);
        threshold = DEFAULT_THRESHOLD;
        computeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        computeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
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
    public PROSACFundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod, final double[] qualityScores,
            final FundamentalMatrixRobustEstimatorListener listener) {
        this(fundMatrixEstimatorMethod, listener);
        threshold = DEFAULT_THRESHOLD;
        computeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        computeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
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
     *                                  scores do not have the same length or their length is less than
     *                                  7 points.
     */
    public PROSACFundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod, final double[] qualityScores,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        this(fundMatrixEstimatorMethod, leftPoints, rightPoints);
        threshold = DEFAULT_THRESHOLD;
        computeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        computeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
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
     *                                  scores do not have the same length or their length is less than
     *                                  7 points.
     */
    public PROSACFundamentalMatrixRobustEstimator(
            final FundamentalMatrixEstimatorMethod fundMatrixEstimatorMethod, final double[] qualityScores,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final FundamentalMatrixRobustEstimatorListener listener) {
        this(fundMatrixEstimatorMethod, leftPoints, rightPoints, listener);
        threshold = DEFAULT_THRESHOLD;
        computeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        computeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     */
    public PROSACFundamentalMatrixRobustEstimator() {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when
     *                 estimation starts, ends or its progress significantly changes.
     */
    public PROSACFundamentalMatrixRobustEstimator(final FundamentalMatrixRobustEstimatorListener listener) {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, listener);
    }

    /**
     * Constructor.
     *
     * @param leftPoints  2D points on left view.
     * @param rightPoints 2D points on right view.
     * @throws IllegalArgumentException if provided list of points do not have
     *                                  the same length or their length is less than 7 points.
     */
    public PROSACFundamentalMatrixRobustEstimator(final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, leftPoints, rightPoints);
    }

    /**
     * Constructor.
     *
     * @param leftPoints  2D points on left view.
     * @param rightPoints 2D points on right view.
     * @param listener    listener to be notified of events such as when estimation
     *                    starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided list of points do not have
     *                                  the same length or their length is less than 7 points.
     */
    public PROSACFundamentalMatrixRobustEstimator(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final FundamentalMatrixRobustEstimatorListener listener) {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, leftPoints, rightPoints, listener);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided pair
     *                      of matched points.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required size (i.e. 7 matched pair of points).
     */
    public PROSACFundamentalMatrixRobustEstimator(final double[] qualityScores) {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, qualityScores);
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
    public PROSACFundamentalMatrixRobustEstimator(
            final double[] qualityScores, final FundamentalMatrixRobustEstimatorListener listener) {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, qualityScores, listener);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided pair
     *                      of matched points.
     * @param leftPoints    2D points on left view.
     * @param rightPoints   2D points on right view.
     * @throws IllegalArgumentException if provided list of points or quality
     *                                  scores do not have the same length or their length is less than
     *                                  7 points.
     */
    public PROSACFundamentalMatrixRobustEstimator(
            final double[] qualityScores, final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, qualityScores, leftPoints, rightPoints);
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
     *                                  scores do not have the same length or their length is less than
     *                                  7 points.
     */
    public PROSACFundamentalMatrixRobustEstimator(
            final double[] qualityScores, final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final FundamentalMatrixRobustEstimatorListener listener) {
        this(DEFAULT_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD, qualityScores, leftPoints, rightPoints, listener);
    }

    /**
     * Returns threshold to determine whether matched pairs of points are
     * inliers or not when testing possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance) a given
     * point has respect to the epipolar line generated by its matched point.
     *
     * @return threshold to determine whether matched pairs of points are
     * inliers or not.
     */
    public double getThreshold() {
        return threshold;
    }

    /**
     * Sets threshold to determine whether matched pairs of points are inliers
     * or not when testing possible estimation solutions.
     *
     * @param threshold threshold to be set.
     * @throws IllegalArgumentException if provided value is equal or less than
     *                                  zero.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     */
    public void setThreshold(final double threshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (threshold <= MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        this.threshold = threshold;
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
     * Indicates whether inliers must be computed and kept.
     *
     * @return true if inliers must be computed and kept, false if inliers
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepInliersEnabled() {
        return computeAndKeepInliers;
    }

    /**
     * Specifies whether inliers must be computed and kept.
     *
     * @param computeAndKeepInliers true if inliers must be computed and kept,
     *                              false if inliers only need to be computed but not kept.
     * @throws LockedException if estimator is locked.
     */
    public void setComputeAndKeepInliersEnabled(final boolean computeAndKeepInliers) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.computeAndKeepInliers = computeAndKeepInliers;
    }

    /**
     * Indicates whether residuals must be computed and kept.
     *
     * @return true if residuals must be computed and kept, false if residuals
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepResidualsEnabled() {
        return computeAndKeepResiduals;
    }

    /**
     * Specifies whether residuals must be computed and kept.
     *
     * @param computeAndKeepResiduals true if residuals must be computed and
     *                                kept, false if residuals only need to be computed but not kept.
     * @throws LockedException if estimator is locked.
     */
    public void setComputeAndKeepResidualsEnabled(final boolean computeAndKeepResiduals) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.computeAndKeepResiduals = computeAndKeepResiduals;
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

        final var innerEstimator = new PROSACRobustEstimator<FundamentalMatrix>(new PROSACRobustEstimatorListener<>() {

            // subset of left points
            private final List<Point2D> subsetLeftPoints = new ArrayList<>();

            // subset of right points
            private final List<Point2D> subsetRightPoints = new ArrayList<>();

            @Override
            public double getThreshold() {
                return threshold;
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
                return PROSACFundamentalMatrixRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(final RobustEstimator<FundamentalMatrix> estimator) {
                if (listener != null) {
                    listener.onEstimateStart(PROSACFundamentalMatrixRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(final RobustEstimator<FundamentalMatrix> estimator) {
                if (listener != null) {
                    listener.onEstimateEnd(PROSACFundamentalMatrixRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    final RobustEstimator<FundamentalMatrix> estimator, final int iteration) {
                if (listener != null) {
                    listener.onEstimateNextIteration(PROSACFundamentalMatrixRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    final RobustEstimator<FundamentalMatrix> estimator, final float progress) {
                if (listener != null) {
                    listener.onEstimateProgressChange(PROSACFundamentalMatrixRobustEstimator.this, progress);
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
            innerEstimator.setComputeAndKeepInliersEnabled(computeAndKeepInliers || refineResult);
            innerEstimator.setComputeAndKeepResidualsEnabled(computeAndKeepResiduals || refineResult);
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
        return RobustEstimatorMethod.PROSAC;
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
        return threshold;
    }

    /**
     * Sets quality scores corresponding to each provided pair of matched
     * points.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than 7 points.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores.length < getMinRequiredPoints()) {
            throw new IllegalArgumentException();
        }

        this.qualityScores = qualityScores;
    }
}
