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
import com.irurueta.numerical.robust.RANSACRobustEstimator;
import com.irurueta.numerical.robust.RANSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best Image of Absolute Conic (IAC) using RANSAC algorithm.
 */
public class RANSACImageOfAbsoluteConicRobustEstimator extends ImageOfAbsoluteConicRobustEstimator {

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
     * Constructor.
     */
    public RANSACImageOfAbsoluteConicRobustEstimator() {
        super();
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when
     *                 estimation starts, ends or its progress significantly changes.
     */
    public RANSACImageOfAbsoluteConicRobustEstimator(final ImageOfAbsoluteConicRobustEstimatorListener listener) {
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
    public RANSACImageOfAbsoluteConicRobustEstimator(final List<Transformation2D> homographies) {
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
    public RANSACImageOfAbsoluteConicRobustEstimator(
            final List<Transformation2D> homographies, final ImageOfAbsoluteConicRobustEstimatorListener listener) {
        super(homographies, listener);
        threshold = DEFAULT_THRESHOLD;
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

        final var innerEstimator = new RANSACRobustEstimator<ImageOfAbsoluteConic>(
                new RANSACRobustEstimatorListener<>() {

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
                        for (var samplesIndex : samplesIndices) {
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
                        return RANSACImageOfAbsoluteConicRobustEstimator.this.isReady();
                    }

                    @Override
                    public void onEstimateStart(final RobustEstimator<ImageOfAbsoluteConic> estimator) {
                        if (listener != null) {
                            listener.onEstimateStart(RANSACImageOfAbsoluteConicRobustEstimator.this);
                        }
                    }

                    @Override
                    public void onEstimateEnd(final RobustEstimator<ImageOfAbsoluteConic> estimator) {
                        if (listener != null) {
                            listener.onEstimateEnd(RANSACImageOfAbsoluteConicRobustEstimator.this);
                        }
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final RobustEstimator<ImageOfAbsoluteConic> estimator, final int iteration) {
                        if (listener != null) {
                            listener.onEstimateNextIteration(
                                    RANSACImageOfAbsoluteConicRobustEstimator.this, iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final RobustEstimator<ImageOfAbsoluteConic> estimator, final float progress) {
                        if (listener != null) {
                            listener.onEstimateProgressChange(
                                    RANSACImageOfAbsoluteConicRobustEstimator.this, progress);
                        }
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
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.RANSAC;
    }
}
