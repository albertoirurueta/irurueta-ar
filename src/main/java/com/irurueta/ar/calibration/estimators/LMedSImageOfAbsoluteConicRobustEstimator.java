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
import com.irurueta.numerical.robust.LMedSRobustEstimator;
import com.irurueta.numerical.robust.LMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best Image of AbsoluteConic (IAC) for provided collection of
 * homographies (2D transformations) using LMedS algorithm.
 */
public class LMedSImageOfAbsoluteConicRobustEstimator extends ImageOfAbsoluteConicRobustEstimator {
    /**
     * Default value to be used for stop threshold. Stop threshold can be used
     * to keep the algorithm iterating in case that best estimated threshold
     * using median of residuals is not small enough. Once a solution is found
     * that generates a threshold below this value, the algorithm will stop.
     * Threshold is defined by equations h1'*IAC*h2 = 0 and
     * h1'*IAC*h1 = h2'*IAC*h2 --&lt; h1'*IAC*h1 - h2'*IAC*h2 = 0, where
     * h1 and h2 are the 1st and 2nd columns of an homography (2D
     * transformation).
     * These equations are derived from the fact that rotation matrices are
     * orthonormal.
     */
    public static final double DEFAULT_STOP_THRESHOLD = 1e-6;

    /**
     * Minimum value that can be set as stop threshold.
     * Threshold must be strictly greater than 0.0.
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
     * Constructor.
     */
    public LMedSImageOfAbsoluteConicRobustEstimator() {
        super();
        stopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when
     *                 estimation starts, ends or its progress significantly changes.
     */
    public LMedSImageOfAbsoluteConicRobustEstimator(final ImageOfAbsoluteConicRobustEstimatorListener listener) {
        super(listener);
        stopThreshold = DEFAULT_STOP_THRESHOLD;
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
    public LMedSImageOfAbsoluteConicRobustEstimator(final List<Transformation2D> homographies) {
        super(homographies);
        stopThreshold = DEFAULT_STOP_THRESHOLD;
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
    public LMedSImageOfAbsoluteConicRobustEstimator(
            final List<Transformation2D> homographies, final ImageOfAbsoluteConicRobustEstimatorListener listener) {
        super(homographies, listener);
        stopThreshold = DEFAULT_STOP_THRESHOLD;
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
     * @throws IllegalArgumentException if provided value is zero or negative
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

        final var innerEstimator = new LMedSRobustEstimator<ImageOfAbsoluteConic>(new LMedSRobustEstimatorListener<>() {

            // subset of homographies picked on each iteration
            private final List<Transformation2D> subsetHomographies = new ArrayList<>();

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
                return LMedSImageOfAbsoluteConicRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(final RobustEstimator<ImageOfAbsoluteConic> estimator) {
                if (listener != null) {
                    listener.onEstimateStart(LMedSImageOfAbsoluteConicRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(final RobustEstimator<ImageOfAbsoluteConic> estimator) {
                if (listener != null) {
                    listener.onEstimateEnd(LMedSImageOfAbsoluteConicRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    final RobustEstimator<ImageOfAbsoluteConic> estimator, final int iteration) {
                if (listener != null) {
                    listener.onEstimateNextIteration(
                            LMedSImageOfAbsoluteConicRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    final RobustEstimator<ImageOfAbsoluteConic> estimator, final float progress) {
                if (listener != null) {
                    listener.onEstimateProgressChange(
                            LMedSImageOfAbsoluteConicRobustEstimator.this, progress);
                }
            }
        });

        try {
            locked = true;
            innerEstimator.setConfidence(confidence);
            innerEstimator.setMaxIterations(maxIterations);
            innerEstimator.setProgressDelta(progressDelta);
            innerEstimator.setStopThreshold(stopThreshold);
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
        return RobustEstimatorMethod.LMEDS;
    }
}
