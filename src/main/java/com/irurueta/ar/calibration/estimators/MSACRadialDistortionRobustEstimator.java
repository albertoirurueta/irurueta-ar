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
import com.irurueta.numerical.robust.MSACRobustEstimator;
import com.irurueta.numerical.robust.MSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Finds the best radial distortion for provided collections of 2D points using
 * MSAC algorithm.
 */
public class MSACRadialDistortionRobustEstimator extends RadialDistortionRobustEstimator {

    /**
     * Constant defining default threshold to determine whether points are
     * inliers or not.
     * By default, 1.0 is considered a good value for cases where measures are
     * done on pixels, since typically the minimum resolution is 1 pixel.
     */
    public static final double DEFAULT_THRESHOLD = 1.0;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Threshold to determine whether points are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance) a possible
     * solution has on a matched pair of points.
     */
    private double threshold;

    /**
     * Constructor.
     */
    public MSACRadialDistortionRobustEstimator() {
        super();
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when
     *                 estimation starts, ends or its progress significantly changes.
     */
    public MSACRadialDistortionRobustEstimator(final RadialDistortionRobustEstimatorListener listener) {
        super(listener);
        threshold = DEFAULT_THRESHOLD;
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
    public MSACRadialDistortionRobustEstimator(final List<Point2D> distortedPoints,
                                               final List<Point2D> undistortedPoints) {
        super(distortedPoints, undistortedPoints);
        threshold = DEFAULT_THRESHOLD;
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
    public MSACRadialDistortionRobustEstimator(final List<Point2D> distortedPoints,
                                               final List<Point2D> undistortedPoints,
                                               final RadialDistortionRobustEstimatorListener listener) {
        super(distortedPoints, undistortedPoints, listener);
        threshold = DEFAULT_THRESHOLD;
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
    public MSACRadialDistortionRobustEstimator(final List<Point2D> distortedPoints,
                                               final List<Point2D> undistortedPoints,
                                               final Point2D distortionCenter) {
        super(distortedPoints, undistortedPoints, distortionCenter);
        threshold = DEFAULT_THRESHOLD;
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
    public MSACRadialDistortionRobustEstimator(
            final List<Point2D> distortedPoints, final List<Point2D> undistortedPoints, final Point2D distortionCenter,
            final RadialDistortionRobustEstimatorListener listener) {
        super(distortedPoints, undistortedPoints, distortionCenter, listener);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Returns threshold to determine whether points are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of error (i.e. Euclidean distance) a
     * possible solution has on projected 2D points.
     *
     * @return threshold to determine whether points are inliers or not when
     * testing possible estimation solutions.
     */
    public double getThreshold() {
        return threshold;
    }

    /**
     * Sets threshold to determine whether points are inliers or not when
     * testing possible estimation solutions.
     * The threshold refers to the amount of error (i.e. Euclidean distance) a
     * possible solution has on projected 2D points.
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
    public RadialDistortion estimate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var innerEstimator = new MSACRobustEstimator<RadialDistortion>(new MSACRobustEstimatorListener<>() {

            // point to be reused when computing residuals
            private final Point2D testPoint = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);

            // non-robust radial distortion estimator
            private final LMSERadialDistortionEstimator radialDistortionEstimator = new LMSERadialDistortionEstimator();

            // subset of distorted (i.e. measured) points
            private final List<Point2D> subsetDistorted = new ArrayList<>();

            // subset of undistorted (i.e. ideal) points
            private final List<Point2D> subsetUndistorted = new ArrayList<>();

            @Override
            public double getThreshold() {
                return threshold;
            }

            @Override
            public int getTotalSamples() {
                return distortedPoints.size();
            }

            @Override
            public int getSubsetSize() {
                return RadialDistortionRobustEstimator.MIN_NUMBER_OF_POINTS;
            }

            @Override
            public void estimatePreliminarSolutions(
                    final int[] samplesIndices, final List<RadialDistortion> solutions) {
                subsetDistorted.clear();
                subsetDistorted.add(distortedPoints.get(samplesIndices[0]));
                subsetDistorted.add(distortedPoints.get(samplesIndices[1]));

                subsetUndistorted.clear();
                subsetUndistorted.add(undistortedPoints.get(samplesIndices[0]));
                subsetUndistorted.add(undistortedPoints.get(samplesIndices[1]));

                try {
                    radialDistortionEstimator.setPoints(distortedPoints, undistortedPoints);
                    radialDistortionEstimator.setPoints(subsetDistorted, subsetUndistorted);

                    final var distortion = radialDistortionEstimator.estimate();
                    solutions.add(distortion);
                } catch (final Exception e) {
                    // if anything fails, no solution is added
                }
            }

            @Override
            public double computeResidual(final RadialDistortion currentEstimation, final int i) {
                final var distortedPoint = distortedPoints.get(i);
                final var undistortedPoint = undistortedPoints.get(i);

                currentEstimation.distort(undistortedPoint, testPoint);

                return testPoint.distanceTo(distortedPoint);
            }

            @Override
            public boolean isReady() {
                return MSACRadialDistortionRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(final RobustEstimator<RadialDistortion> estimator) {
                try {
                    radialDistortionEstimator.setLMSESolutionAllowed(false);
                    radialDistortionEstimator.setIntrinsic(getIntrinsic());
                } catch (final Exception e) {
                    Logger.getLogger(MSACRadialDistortionRobustEstimator.class.getName()).log(Level.WARNING,
                            "Could not set intrinsic parameters on radial distortion estimator", e);
                }

                if (listener != null) {
                    listener.onEstimateStart(MSACRadialDistortionRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(final RobustEstimator<RadialDistortion> estimator) {
                if (listener != null) {
                    listener.onEstimateEnd(
                            MSACRadialDistortionRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    final RobustEstimator<RadialDistortion> estimator, final int iteration) {
                if (listener != null) {
                    listener.onEstimateNextIteration(MSACRadialDistortionRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    final RobustEstimator<RadialDistortion> estimator, final float progress) {
                if (listener != null) {
                    listener.onEstimateProgressChange(MSACRadialDistortionRobustEstimator.this, progress);
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
     * Returns method being used for robust estimation
     *
     * @return method being used for robust estimation
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.MSAC;
    }
}
