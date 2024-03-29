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

import com.irurueta.ar.calibration.RadialDistortion;
import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.LMedSRobustEstimator;
import com.irurueta.numerical.robust.LMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Finds the best radial distortion for provided collection of 2D points using
 * LMedS algorithm
 */
public class LMedSRadialDistortionRobustEstimator extends
        RadialDistortionRobustEstimator {

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
    private double mStopThreshold;

    /**
     * Constructor.
     */
    public LMedSRadialDistortionRobustEstimator() {
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when
     *                 estimation starts, ends or its progress significantly changes.
     */
    public LMedSRadialDistortionRobustEstimator(
            final RadialDistortionRobustEstimatorListener listener) {
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param distortedPoints   list of distorted points. Distorted points are
     *                          obtained after radial distortion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MIN_NUMBER_OF_POINTS.
     */
    public LMedSRadialDistortionRobustEstimator(final List<Point2D> distortedPoints,
                                                final List<Point2D> undistortedPoints) {
        super(distortedPoints, undistortedPoints);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param distortedPoints   list of distorted points. Distorted points are
     *                          obtained after radial distortion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param listener          listener to be notified of events such as when
     *                          estimation starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MIN_NUMBER_OF_POINTS.
     */
    public LMedSRadialDistortionRobustEstimator(final List<Point2D> distortedPoints,
                                                final List<Point2D> undistortedPoints,
                                                final RadialDistortionRobustEstimatorListener listener) {
        super(distortedPoints, undistortedPoints, listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param distortedPoints   list of distorted points. Distorted points are
     *                          obtained after radial distortion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param distortionCenter  radial distortion center. If null it is assumed
     *                          to be the origin of coordinates, otherwise this is typically equal to
     *                          the camera principal point.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MIN_NUMBER_OF_POINTS.
     */
    public LMedSRadialDistortionRobustEstimator(final List<Point2D> distortedPoints,
                                                final List<Point2D> undistortedPoints,
                                                final Point2D distortionCenter) {
        super(distortedPoints, undistortedPoints, distortionCenter);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param distortedPoints   list of distorted points. Distorted points are
     *                          obtained after radial distortion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param distortionCenter  radial distortion center. If null it is assumed
     *                          to be the origin of coordinates, otherwise this is typically equal to
     *                          the camera principal point.
     * @param listener          listener to be notified of events such as when
     *                          estimation starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MIN_NUMBER_OF_POINTS.
     */
    public LMedSRadialDistortionRobustEstimator(final List<Point2D> distortedPoints,
                                                final List<Point2D> undistortedPoints,
                                                final Point2D distortionCenter,
                                                final RadialDistortionRobustEstimatorListener listener) {
        super(distortedPoints, undistortedPoints, distortionCenter, listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
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
        return mStopThreshold;
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

        mStopThreshold = stopThreshold;
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
    public RadialDistortion estimate() throws LockedException,
            NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final LMedSRobustEstimator<RadialDistortion> innerEstimator =
                new LMedSRobustEstimator<>(
                        new LMedSRobustEstimatorListener<RadialDistortion>() {

                            // point to be reused when computing residuals
                            private final Point2D mTestPoint = Point2D.create(
                                    CoordinatesType.INHOMOGENEOUS_COORDINATES);

                            // non-robust radial distortion estimator
                            private final LMSERadialDistortionEstimator mRadialDistortionEstimator =
                                    new LMSERadialDistortionEstimator();

                            // subset of distorted (i.e. measured) points
                            private final List<Point2D> mSubsetDistorted = new ArrayList<>();

                            // subset of undistorted (i.e. ideal) points
                            private final List<Point2D> mSubsetUndistorted = new ArrayList<>();

                            @Override
                            public int getTotalSamples() {
                                return mDistortedPoints.size();
                            }

                            @Override
                            public int getSubsetSize() {
                                return RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS;
                            }

                            @Override
                            public void estimatePreliminarSolutions(
                                    final int[] samplesIndices, final List<RadialDistortion> solutions) {
                                mSubsetDistorted.clear();
                                mSubsetDistorted.add(mDistortedPoints.get(samplesIndices[0]));
                                mSubsetDistorted.add(mDistortedPoints.get(samplesIndices[1]));

                                mSubsetUndistorted.clear();
                                mSubsetUndistorted.add(mUndistortedPoints.get(samplesIndices[0]));
                                mSubsetUndistorted.add(mUndistortedPoints.get(samplesIndices[1]));

                                try {
                                    mRadialDistortionEstimator.setPoints(mDistortedPoints,
                                            mUndistortedPoints);
                                    mRadialDistortionEstimator.setPoints(mSubsetDistorted,
                                            mSubsetUndistorted);

                                    final RadialDistortion distortion = mRadialDistortionEstimator.
                                            estimate();
                                    solutions.add(distortion);
                                } catch (final Exception e) {
                                    // if anything fails, no solution is added
                                }
                            }

                            @Override
                            public double computeResidual(final RadialDistortion currentEstimation,
                                                          final int i) {
                                final Point2D distortedPoint = mDistortedPoints.get(i);
                                final Point2D undistortedPoint = mUndistortedPoints.get(i);

                                currentEstimation.distort(undistortedPoint, mTestPoint);

                                return mTestPoint.distanceTo(distortedPoint);
                            }

                            @Override
                            public boolean isReady() {
                                return LMedSRadialDistortionRobustEstimator.this.isReady();
                            }

                            @Override
                            public void onEstimateStart(
                                    final RobustEstimator<RadialDistortion> estimator) {
                                try {
                                    mRadialDistortionEstimator.setLMSESolutionAllowed(false);
                                    mRadialDistortionEstimator.setIntrinsic(getIntrinsic());
                                } catch (final Exception e) {
                                    Logger.getLogger(
                                            LMedSRadialDistortionRobustEstimator.class.getName()).
                                            log(Level.WARNING,
                                                    "Could not set intrinsic parameters on radial distortion estimator", e);
                                }

                                if (mListener != null) {
                                    mListener.onEstimateStart(LMedSRadialDistortionRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateEnd(
                                    final RobustEstimator<RadialDistortion> estimator) {
                                if (mListener != null) {
                                    mListener.onEstimateEnd(LMedSRadialDistortionRobustEstimator.this);
                                }
                            }

                            @Override
                            public void onEstimateNextIteration(
                                    final RobustEstimator<RadialDistortion> estimator,
                                    final int iteration) {
                                if (mListener != null) {
                                    mListener.onEstimateNextIteration(
                                            LMedSRadialDistortionRobustEstimator.this,
                                            iteration);
                                }
                            }

                            @Override
                            public void onEstimateProgressChange(
                                    final RobustEstimator<RadialDistortion> estimator,
                                    final float progress) {
                                if (mListener != null) {
                                    mListener.onEstimateProgressChange(
                                            LMedSRadialDistortionRobustEstimator.this,
                                            progress);
                                }
                            }
                        });

        try {
            mLocked = true;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            innerEstimator.setStopThreshold(mStopThreshold);
            return innerEstimator.estimate();
        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } finally {
            mLocked = false;
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
