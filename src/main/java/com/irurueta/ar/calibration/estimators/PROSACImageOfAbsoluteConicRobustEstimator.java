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
package com.irurueta.ar.calibration.estimators;

import com.irurueta.ar.calibration.ImageOfAbsoluteConic;
import com.irurueta.geometry.Transformation2D;
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
 * Finds the best image of absolute conic (IAC) for provided collection of
 * homographies (2D transformations) using PROSAC algorithm.
 */
public class PROSACImageOfAbsoluteConicRobustEstimator extends ImageOfAbsoluteConicRobustEstimator {

    /**
     * Constant defining default threshold to determine whether homographies are
     * inliers or not.
     * Threshold is defined by equations h1'*IAC*h2 = 0 and
     * h1'*IAC*h1 = h2'*IAC*h2 --&lt; h1'*IAC*h1 - h2'*IAC*h2 = 0, where
     * h1 and h2 are the 1st and 2nd columns of an homography (2D
     * transformation).
     * These equations are derived from the fact that rotation matrices are
     * orthonormal.
     */
    public static final double DEFAULT_THRESHOLD = 1e-6;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Threshold to determine whether homographies are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on
     * the ortho-normality assumption of rotation matrices.
     */
    private double threshold;

    /**
     * Quality scores corresponding to each provided homography.
     * The larger the score value the better the quality of the sample.
     */
    private double[] qualityScores;

    /**
     * Constructor.
     */
    public PROSACImageOfAbsoluteConicRobustEstimator() {
        super();
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when
     *                 estimation starts, ends or its progress significantly changes.
     */
    public PROSACImageOfAbsoluteConicRobustEstimator(final ImageOfAbsoluteConicRobustEstimatorListener listener) {
        super(listener);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param homographies list of homographies (2D transformations) used to
     *                     estimate the image of absolute conic (IAC), which can be used to obtain
     *                     pinhole camera intrinsic parameters.
     * @throws IllegalArgumentException if not enough homographies are provided
     *                                  for default settings. Hence, at least 1 homography must be provided.
     */
    public PROSACImageOfAbsoluteConicRobustEstimator(final List<Transformation2D> homographies) {
        super(homographies);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param homographies list of homographies (2D transformations) used to
     *                     estimate the image of absolute conic (IAC), which can be used to obtain
     *                     pinhole camera intrinsic parameters.
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if not enough homographies are provided
     *                                  for default settings. Hence, at least 1 homography must be provided.
     */
    public PROSACImageOfAbsoluteConicRobustEstimator(
            final List<Transformation2D> homographies, final ImageOfAbsoluteConicRobustEstimatorListener listener) {
        super(homographies, listener);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      homography.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required number of homographies for default settings
     *                                  (i.e. 1 homography).
     */
    public PROSACImageOfAbsoluteConicRobustEstimator(final double[] qualityScores) {
        this();
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      homography.
     * @param listener      listener to be notified of events such as when
     *                      estimation starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required number of homographies for default settings
     *                                  (i.e. 1 homography).
     */
    public PROSACImageOfAbsoluteConicRobustEstimator(
            final double[] qualityScores, final ImageOfAbsoluteConicRobustEstimatorListener listener) {
        this(listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param homographies  list of homographies (2D transformations) used to
     *                      estimate the image of absolute conic (IAC), which can be used to obtain
     *                      pinhole camera intrinsic parameters.
     * @param qualityScores quality scores corresponding to each provided
     *                      homography.
     * @throws IllegalArgumentException if not enough homographies are provided
     *                                  for default settings (i.e. 1 homography) or quality scores and
     *                                  homographies don't have the same size.
     */
    public PROSACImageOfAbsoluteConicRobustEstimator(
            final List<Transformation2D> homographies, final double[] qualityScores) {
        this(homographies);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param homographies  list of homographies (2D transformations) used to
     *                      estimate the image of absolute conic (IAC), which can be used to obtain
     *                      pinhole camera intrinsic parameters.
     * @param qualityScores quality scores corresponding to each provided
     *                      homography.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if not enough homographies are provided
     *                                  for default settings (i.e. 1 homography) or quality scores and
     *                                  homographies don't have the same size.
     */
    public PROSACImageOfAbsoluteConicRobustEstimator(
            final List<Transformation2D> homographies, final double[] qualityScores,
            final ImageOfAbsoluteConicRobustEstimatorListener listener) {
        this(homographies, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Returns threshold to determine whether homographies are inliers or not
     * when testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on
     * the ortho-normality assumption of rotation matrices.
     *
     * @return threshold to determine whether homographies are inliers or not
     * when testing possible estimation solutions.
     */
    public double getThreshold() {
        return threshold;
    }

    /**
     * Sets threshold to determine whether homographies are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on
     * the ortho-normality assumption of rotation matrices.
     *
     * @param threshold threshold to determine whether homographies are inliers
     *                  or not when testing possible estimation solutions.
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
     * Returns quality scores corresponding to each provided homography.
     * The larger the score value the better the quality of the sampled
     * homography
     *
     * @return quality scores corresponding to each homography.
     */
    @Override
    public double[] getQualityScores() {
        return qualityScores;
    }

    /**
     * Sets quality scores corresponding to each provided homography.
     * The larger the score value the better the quality of the sampled
     * homography
     *
     * @param qualityScores quality scores corresponding to each homography
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than minimum required number of homographies (i.e. 1
     *                                  homography)
     */
    @Override
    public void setQualityScores(final double[] qualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates if estimator is ready to start the IAC estimation.
     * This is true when input data (i.e. homographies) is provided and list
     * contains at least the minimum number of required homographies, and
     * also quality scores are provided.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && qualityScores != null && qualityScores.length == homographies.size();
    }

    /**
     * Estimates Image of Absolute Conic (IAC).
     *
     * @return estimated IAC.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public ImageOfAbsoluteConic estimate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var innerEstimator = new PROSACRobustEstimator<ImageOfAbsoluteConic>(
                new PROSACRobustEstimatorListener<>() {

                    // subset of homographies picked on each iteration
                    private final List<Transformation2D> subsetHomographies = new ArrayList<>();

                    @Override
                    public double getThreshold() {
                        return threshold;
                    }

                    @Override
                    public int getTotalSamples() {
                        return homographies.size();
                    }

                    @Override
                    public int getSubsetSize() {
                        return iacEstimator.getMinNumberOfRequiredHomographies();
                    }

                    @Override
                    public void estimatePreliminarSolutions(
                            final int[] samplesIndices, final List<ImageOfAbsoluteConic> solutions) {
                        subsetHomographies.clear();
                        for (final var samplesIndex : samplesIndices) {
                            subsetHomographies.add(homographies.get(samplesIndex));
                        }

                        try {
                            iacEstimator.setLMSESolutionAllowed(false);
                            iacEstimator.setHomographies(subsetHomographies);

                            final var iac = iacEstimator.estimate();
                            solutions.add(iac);
                        } catch (final Exception e) {
                            // if anything fails, no solution is added
                        }
                    }

                    @Override
                    public double computeResidual(final ImageOfAbsoluteConic currentEstimation, final int i) {
                        return residual(currentEstimation, homographies.get(i));
                    }

                    @Override
                    public boolean isReady() {
                        return PROSACImageOfAbsoluteConicRobustEstimator.this.isReady();
                    }

                    @Override
                    public void onEstimateStart(final RobustEstimator<ImageOfAbsoluteConic> estimator) {
                        if (listener != null) {
                            listener.onEstimateStart(PROSACImageOfAbsoluteConicRobustEstimator.this);
                        }
                    }

                    @Override
                    public void onEstimateEnd(final RobustEstimator<ImageOfAbsoluteConic> estimator) {
                        if (listener != null) {
                            listener.onEstimateEnd(PROSACImageOfAbsoluteConicRobustEstimator.this);
                        }
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final RobustEstimator<ImageOfAbsoluteConic> estimator, final int iteration) {
                        if (listener != null) {
                            listener.onEstimateNextIteration(
                                    PROSACImageOfAbsoluteConicRobustEstimator.this, iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final RobustEstimator<ImageOfAbsoluteConic> estimator, final float progress) {
                        if (listener != null) {
                            listener.onEstimateProgressChange(
                                    PROSACImageOfAbsoluteConicRobustEstimator.this, progress);
                        }
                    }

                    @Override
                    public double[] getQualityScores() {
                        return qualityScores;
                    }
                });

        try {
            locked = true;
            innerEstimator.setConfidence(confidence);
            innerEstimator.setMaxIterations(maxIterations);
            innerEstimator.setProgressDelta(progressDelta);
            return innerEstimator.estimate();
        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } finally {
            locked = false;
        }
    }

    /**
     * Returns method being used for robust estimation
     *
     * @return method being used for robust estimation
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROSAC;
    }

    /**
     * Sets quality scores corresponding to each homography.
     * This method is used internally and does not check whether instance is
     * locked or not
     *
     * @param qualityScores quality scores to be set
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than the minimum number of required homographies for current
     *                                  settings.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores.length < iacEstimator.getMinNumberOfRequiredHomographies()) {
            throw new IllegalArgumentException();
        }
        this.qualityScores = qualityScores;
    }
}
