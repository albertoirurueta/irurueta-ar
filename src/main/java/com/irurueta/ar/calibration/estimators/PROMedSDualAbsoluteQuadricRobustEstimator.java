/*
 * Copyright (C) 2016 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.ar.calibration.DualAbsoluteQuadric;
import com.irurueta.geometry.PinholeCamera;
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
 * Finds the best dual absolute quadric (DAQ) for provided collection of
 * cameras using PROMedS algorithm.
 */
public class PROMedSDualAbsoluteQuadricRobustEstimator extends DualAbsoluteQuadricRobustEstimator {

    /**
     * Default value to be used for stop threshold. Stop threshold can be used
     * to keep the algorithm iterating in case that best estimated threshold
     * using median of residuals is not small enough. Once a solution is found
     * that generates a threshold below this value, the algorithm will stop.
     * Threshold is defined by the equations used to estimate the DAQ depending
     * on the required settings (zero skewness, principal point at origin, and
     * known aspect ratio).
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
     * The stop threshold can be used to prevent the PROMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of inliers is very small (close
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
     * Quality scores corresponding to each provided camera.
     * The larger the score value the better the quality of the sample.
     */
    private double[] qualityScores;

    /**
     * Constructor.
     */
    public PROMedSDualAbsoluteQuadricRobustEstimator() {
        super();
        stopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROMedSDualAbsoluteQuadricRobustEstimator(final DualAbsoluteQuadricRobustEstimatorListener listener) {
        super(listener);
        stopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param cameras list of cameras used to estimate the dual absolute quadric
     *                (DAC), which can be used to obtain pinhole camera intrinsic parameters.
     * @throws IllegalArgumentException if not enough cameras are provided for
     *                                  default settings. Hence, at least 2 cameras must be provided.
     */
    public PROMedSDualAbsoluteQuadricRobustEstimator(final List<PinholeCamera> cameras) {
        super(cameras);
        stopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param cameras  list of cameras used to estimate the dual absolute quadric
     *                 (DAQ), which can be used to obtain pinhole camera intrinsic parameters.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if not enough cameras are provided for
     *                                  default settings. Hence, at least 2 cameras must be provided.
     */
    public PROMedSDualAbsoluteQuadricRobustEstimator(
            final List<PinholeCamera> cameras, final DualAbsoluteQuadricRobustEstimatorListener listener) {
        super(cameras, listener);
        stopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      camera.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required number of homographies for default
     *                                  settings (i.e. 2 cameras).
     */
    public PROMedSDualAbsoluteQuadricRobustEstimator(final double[] qualityScores) {
        this();
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      camera.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required number of cameras for default settings (i.e.
     *                                  2 cameras).
     */
    public PROMedSDualAbsoluteQuadricRobustEstimator(
            final double[] qualityScores, final DualAbsoluteQuadricRobustEstimatorListener listener) {
        this(listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param cameras       list of cameras used to estimate the dual absolute quadric
     *                      (DAQ), which can be used to obtain pinhole camera intrinsic parameters.
     * @param qualityScores quality scores corresponding to each provided
     *                      camera.
     * @throws IllegalArgumentException if not enough cameras are provided for
     *                                  default settings (i.e. 2 cameras) or quality scores and cameras
     *                                  don't have the same size.
     */
    public PROMedSDualAbsoluteQuadricRobustEstimator(final List<PinholeCamera> cameras, final double[] qualityScores) {
        this(cameras);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param cameras       list of cameras used to estimate the dual absolute quadric
     *                      (DAQ), which can be used to obtain pinhole camera intrinsic parameters.
     * @param qualityScores quality scores corresponding to each provided
     *                      camera.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if not enough cameras are provided for
     *                                  default settings (i.e. 2 cameras) or quality scores and cameras
     *                                  don't have the same size.
     */
    public PROMedSDualAbsoluteQuadricRobustEstimator(
            final List<PinholeCamera> cameras, final double[] qualityScores,
            final DualAbsoluteQuadricRobustEstimatorListener listener) {
        this(cameras, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Returns threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the PROMedS algorithm iterating
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
     * The stop threshold can be used to prevent the PROMedS algorithm iterating
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
     * Returns quality scores corresponding to each provided camera.
     * The larger the score value the better the quality of the sampled
     * camera.
     *
     * @return quality scores corresponding to each camera.
     */
    @Override
    public double[] getQualityScores() {
        return qualityScores;
    }

    /**
     * Sets quality scores corresponding to each provided camera.
     * The larger the score value the better the quality of the sampled
     * camera.
     *
     * @param qualityScores quality scores corresponding to each camera.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than minimum required number of cameras (i.e. 2 cameras).
     */
    @Override
    public void setQualityScores(final double[] qualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates if estimator is ready to start the DAQ estimation.
     * This is true when input data (i.e. cameras) is provided and list contains
     * at least the minimum number of required cameras, and also quality scores
     * are provided.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && qualityScores != null && qualityScores.length == cameras.size();
    }

    /**
     * Estimates the Dual Absolute Quadric using provided cameras.
     *
     * @return estimated Dual Absolute Quadric (DAQ).
     * @throws LockedException          if robust estimator is locked.
     * @throws NotReadyException        if no valid input data has already been
     *                                  provided.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public DualAbsoluteQuadric estimate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var innerEstimator = new PROMedSRobustEstimator<DualAbsoluteQuadric>(new PROMedSRobustEstimatorListener<>() {

            // subset of cameras picked on each iteration
            private final List<PinholeCamera> subsetCameras = new ArrayList<>();

            @Override
            public double getThreshold() {
                return stopThreshold;
            }

            @Override
            public int getTotalSamples() {
                return cameras.size();
            }

            @Override
            public int getSubsetSize() {
                return daqEstimator.getMinNumberOfRequiredCameras();
            }

            @Override
            public void estimatePreliminarSolutions(
                    final int[] samplesIndices, final List<DualAbsoluteQuadric> solutions) {
                subsetCameras.clear();
                for (final var samplesIndex : samplesIndices) {
                    subsetCameras.add(cameras.get(samplesIndex));
                }

                try {
                    daqEstimator.setLMSESolutionAllowed(false);
                    daqEstimator.setCameras(subsetCameras);

                    final var daq = daqEstimator.estimate();
                    solutions.add(daq);
                } catch (final Exception e) {
                    // if anything fails, no solution is added
                }
            }

            @Override
            public double computeResidual(final DualAbsoluteQuadric currentEstimation, final int i) {
                return residual(currentEstimation, cameras.get(i));
            }

            @Override
            public boolean isReady() {
                return PROMedSDualAbsoluteQuadricRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(final RobustEstimator<DualAbsoluteQuadric> estimator) {
                if (listener != null) {
                    listener.onEstimateStart(PROMedSDualAbsoluteQuadricRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(final RobustEstimator<DualAbsoluteQuadric> estimator) {
                if (listener != null) {
                    listener.onEstimateEnd(PROMedSDualAbsoluteQuadricRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    final RobustEstimator<DualAbsoluteQuadric> estimator, final int iteration) {
                if (listener != null) {
                    listener.onEstimateNextIteration(PROMedSDualAbsoluteQuadricRobustEstimator.this,
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    final RobustEstimator<DualAbsoluteQuadric> estimator, final float progress) {
                if (listener != null) {
                    listener.onEstimateProgressChange(PROMedSDualAbsoluteQuadricRobustEstimator.this,
                            progress);
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
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROMEDS;
    }

    /**
     * Sets quality scores corresponding to each camera.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than the minimum number of required homographies for
     *                                  current settings.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores.length < daqEstimator.getMinNumberOfRequiredCameras()) {
            throw new IllegalArgumentException();
        }

        this.qualityScores = qualityScores;
    }
}
