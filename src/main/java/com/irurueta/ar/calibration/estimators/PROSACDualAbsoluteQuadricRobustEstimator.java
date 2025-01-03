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
import com.irurueta.numerical.robust.PROSACRobustEstimator;
import com.irurueta.numerical.robust.PROSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best dual absolute quadric (DAQ) for provided collection of
 * cameras using PROSAC algorithm.
 */
public class PROSACDualAbsoluteQuadricRobustEstimator extends DualAbsoluteQuadricRobustEstimator {

    /**
     * Constant defining default threshold to determine whether cameras are
     * inliers or not.
     * Threshold is defined by the equations used to estimate the DAQ depending
     * on the required settings (zero skewness, principal point at origin, and
     * known aspect ratio).
     */
    public static final double DEFAULT_THRESHOLD = 1e-3;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Threshold to determine whether cameras are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has.
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
    public PROSACDualAbsoluteQuadricRobustEstimator() {
        super();
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROSACDualAbsoluteQuadricRobustEstimator(final DualAbsoluteQuadricRobustEstimatorListener listener) {
        super(listener);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param cameras list of cameras used to estimate the Dual Absolute Quadric
     *                (DAQ), which can be used to obtain pinhole camera intrinsic parameters.
     * @throws IllegalArgumentException if not enough cameras are provided for
     *                                  default settings. Hence, at least 2 cameras must be provided.
     */
    public PROSACDualAbsoluteQuadricRobustEstimator(final List<PinholeCamera> cameras) {
        super(cameras);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param cameras  list of cameras used to estimate the Dual Absolute Quadric
     *                 (DAQ), which can be used to obtain pinhole camera intrinsic parameters.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if not enough cameras are provided for
     *                                  default settings. Hence, at least 2 cameras must be provided.
     */
    public PROSACDualAbsoluteQuadricRobustEstimator(
            final List<PinholeCamera> cameras, final DualAbsoluteQuadricRobustEstimatorListener listener) {
        super(cameras, listener);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      camera.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required number of cameras for default settings (i.e. 2
     *                                  cameras).
     */
    public PROSACDualAbsoluteQuadricRobustEstimator(final double[] qualityScores) {
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
     *                                  smaller than required number of cameras for default settings (i.e. 2
     *                                  cameras).
     */
    public PROSACDualAbsoluteQuadricRobustEstimator(
            final double[] qualityScores, final DualAbsoluteQuadricRobustEstimatorListener listener) {
        this(listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param cameras       list of cameras used to estimate the Dual Absolute Quadric
     *                      (DAQ), which can be used to obtain pinhole camera intrinsic parameters.
     * @param qualityScores quality scores corresponding to each provided
     *                      camera.
     * @throws IllegalArgumentException if not enough cameras are provided  for
     *                                  default settings (i.e. 2 cameras) or quality scores and cameras don't
     *                                  have the same size.
     */
    public PROSACDualAbsoluteQuadricRobustEstimator(
            final List<PinholeCamera> cameras, final double[] qualityScores) {
        this(cameras);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param cameras       list of cameras used to estimate the Dual Absolute Quadric
     *                      (DAQ), which can be used to obtain pinhole camera intrinsic parameters.
     * @param qualityScores quality scores corresponding to each provided
     *                      camera.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if not enough cameras are provided
     *                                  for default settings (i.e. 2 cameras) or quality scores and cameras
     *                                  don't have the same size.
     */
    public PROSACDualAbsoluteQuadricRobustEstimator(
            final List<PinholeCamera> cameras, final double[] qualityScores,
            final DualAbsoluteQuadricRobustEstimatorListener listener) {
        this(cameras, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Returns threshold to determine whether cameras are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has.
     *
     * @return threshold to determine whether cameras are inliers or not when
     * testing possible estimation solutions.
     */
    public double getThreshold() {
        return threshold;
    }

    /**
     * Sets threshold to determine whether cameras are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has.
     *
     * @param threshold threshold to determine whether cameras are inliers or
     *                  not when testing possible estimation solutions.
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
     * Returns quality scores corresponding to each provided camera.
     * The larger the score value the better the quality of the sampled camera.
     *
     * @return quality scores corresponding to each camera.
     */
    @Override
    public double[] getQualityScores() {
        return qualityScores;
    }

    /**
     * Sets quality scores corresponding to each provided camera.
     * The larger the score value the better the quality of the sampled camera.
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
     * This is true when input data (i.e. cameras) is provided and contains
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

        final var innerEstimator = new PROSACRobustEstimator<DualAbsoluteQuadric>(
                new PROSACRobustEstimatorListener<>() {

                    // subset of cameras picked on each iteration
                    private final List<PinholeCamera> subsetCameras = new ArrayList<>();

                    @Override
                    public double getThreshold() {
                        return threshold;
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
                        return PROSACDualAbsoluteQuadricRobustEstimator.this.isReady();
                    }

                    @Override
                    public void onEstimateStart(final RobustEstimator<DualAbsoluteQuadric> estimator) {
                        if (listener != null) {
                            listener.onEstimateStart(PROSACDualAbsoluteQuadricRobustEstimator.this);
                        }
                    }

                    @Override
                    public void onEstimateEnd(final RobustEstimator<DualAbsoluteQuadric> estimator) {
                        if (listener != null) {
                            listener.onEstimateEnd(PROSACDualAbsoluteQuadricRobustEstimator.this);
                        }
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final RobustEstimator<DualAbsoluteQuadric> estimator, final int iteration) {
                        if (listener != null) {
                            listener.onEstimateNextIteration(
                                    PROSACDualAbsoluteQuadricRobustEstimator.this, iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final RobustEstimator<DualAbsoluteQuadric> estimator, final float progress) {
                        if (listener != null) {
                            listener.onEstimateProgressChange(
                                    PROSACDualAbsoluteQuadricRobustEstimator.this, progress);
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
        return RobustEstimatorMethod.PROSAC;
    }

    /**
     * Sets quality scores corresponding to each camera.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than the minimum number of required cameras for current
     *                                  settings.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores.length < daqEstimator.getMinNumberOfRequiredCameras()) {
            throw new IllegalArgumentException();
        }
        this.qualityScores = qualityScores;
    }
}
