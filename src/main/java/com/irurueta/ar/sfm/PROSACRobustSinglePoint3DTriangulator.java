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
package com.irurueta.ar.sfm;

import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
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
 * Robustly triangulates 3D points from matched 2D points and their
 * corresponding cameras on several views using PROSAC algorithm.
 */
public class PROSACRobustSinglePoint3DTriangulator extends RobustSinglePoint3DTriangulator {

    /**
     * Constant defining default threshold to determine whether samples are
     * inliers or not.
     * By default, 1.0 is considered a good value for cases where 2D point
     * measures are done on pixels, since typically the minimum resolution is 1
     * pixel.
     */
    public static final double DEFAULT_THRESHOLD = 1.0;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Threshold to determine whether samples are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of projection error (i.e. distance of
     * projected solution using each camera).
     */
    private double threshold;

    /**
     * Quality scores corresponding to each provided view.
     * The larger the score value the better the quality of the sample.
     */
    private double[] qualityScores;

    /**
     * Constructor.
     */
    public PROSACRobustSinglePoint3DTriangulator() {
        super();
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROSACRobustSinglePoint3DTriangulator(final RobustSinglePoint3DTriangulatorListener listener) {
        super(listener);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param points  Matched 2D points. Each point in the list is assumed to be
     *                projected by the corresponding camera in the list.
     * @param cameras List of cameras associated to the matched 2D point on the
     *                same position as the camera on the list.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  length or their length is less than 2 views, which is the minimum
     *                                  required to compute triangulation.
     */
    public PROSACRobustSinglePoint3DTriangulator(final List<Point2D> points, final List<PinholeCamera> cameras) {
        super(points, cameras);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param points   Matched 2D points. Each point in the list is assumed to be
     *                 projected by the corresponding camera in the list.
     * @param cameras  List of cameras associated to the matched 2D point on the
     *                 same position as the camera on the list.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  length or their length is less than 2 views, which is the minimum
     *                                  required to compute triangulation.
     */
    public PROSACRobustSinglePoint3DTriangulator(
            final List<Point2D> points, final List<PinholeCamera> cameras,
            final RobustSinglePoint3DTriangulatorListener listener) {
        super(points, cameras, listener);
        threshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided view.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required size (i.e. 2 views).
     */
    public PROSACRobustSinglePoint3DTriangulator(final double[] qualityScores) {
        this();
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided view.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than required size (i.e. 2 views).
     */
    public PROSACRobustSinglePoint3DTriangulator(final double[] qualityScores,
                                                 final RobustSinglePoint3DTriangulatorListener listener) {
        this(listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param points        Matched 2D points. Each point in the list is assumed to be
     *                      projected by the corresponding camera in the list.
     * @param cameras       List of cameras associated to the matched 2D point on the
     *                      same position as the camera on the list.
     * @param qualityScores quality scores corresponding to each provided view.
     * @throws IllegalArgumentException if provided lists or quality scores
     *                                  don't have the same length or their length is less than 2 views,
     *                                  which is the minimum required to compute triangulation.
     */
    public PROSACRobustSinglePoint3DTriangulator(final List<Point2D> points,
                                                 final List<PinholeCamera> cameras,
                                                 final double[] qualityScores) {
        this(points, cameras);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param points        Matched 2D points. Each point in the list is assumed to be
     *                      projected by the corresponding camera in the list.
     * @param cameras       List of cameras associated to the matched 2D point on the
     *                      same position as the camera on the list.
     * @param qualityScores quality scores corresponding to each provided view.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided lists or quality scores
     *                                  don't have the same length or their length is less than 2 views,
     *                                  which is the minimum required to compute triangulation.
     */
    public PROSACRobustSinglePoint3DTriangulator(final List<Point2D> points,
                                                 final List<PinholeCamera> cameras,
                                                 final double[] qualityScores,
                                                 final RobustSinglePoint3DTriangulatorListener listener) {
        this(points, cameras, listener);
        internalSetQualityScores(qualityScores);
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
     * Returns quality scores corresponding to each provided view.
     * The larger the score value the better the quality of the sampled view.
     *
     * @return quality scores corresponding to each view.
     */
    @Override
    public double[] getQualityScores() {
        return qualityScores;
    }

    /**
     * Sets quality scores corresponding to each provided view.
     * The larger the score value the better the quality of the sampled view.
     *
     * @param qualityScores quality scores corresponding to each view.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MIN_REQUIRED_VIEWS (i.e. 2 views).
     */
    @Override
    public void setQualityScores(final double[] qualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates if triangulator is ready to start the 3D point triangulation.
     * This is true when input data (i.e. 2D points, cameras and quality scores)
     * are provided and a minimum of 2 views are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && qualityScores != null && qualityScores.length == points2D.size();
    }


    /**
     * Triangulates provided matched 2D points being projected by each
     * corresponding camera into a single 3D point.
     * At least 2 matched 2D points and their corresponding 2 cameras are
     * required to compute triangulation. If more views are provided, an
     * averaged solution can be found.
     *
     * @return computed triangulated 3D point.
     * @throws LockedException          if this instance is locked.
     * @throws NotReadyException        if lists of points and cameras don't have the
     *                                  same length or less than 2 views are provided.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public Point3D triangulate() throws LockedException, NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var innerEstimator = new PROSACRobustEstimator<Point3D>(new PROSACRobustEstimatorListener<>() {

            // point to be reused when computing residuals
            private final Point2D testPoint = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);

            // non-robust 3D point triangulator
            private final SinglePoint3DTriangulator triangulator =
                    SinglePoint3DTriangulator.create(useHomogeneousSolution
                            ? Point3DTriangulatorType.LMSE_HOMOGENEOUS_TRIANGULATOR
                            : Point3DTriangulatorType.LMSE_INHOMOGENEOUS_TRIANGULATOR);

            // subset of 2D points
            private final List<Point2D> subsetPoints = new ArrayList<>();

            // subset of cameras
            private final List<PinholeCamera> subsetCameras = new ArrayList<>();

            @Override
            public double getThreshold() {
                return threshold;
            }

            @Override
            public int getTotalSamples() {
                return points2D.size();
            }

            @Override
            public int getSubsetSize() {
                return MIN_REQUIRED_VIEWS;
            }

            @Override
            public void estimatePreliminarSolutions(final int[] samplesIndices, final List<Point3D> solutions) {
                subsetPoints.clear();
                subsetPoints.add(points2D.get(samplesIndices[0]));
                subsetPoints.add(points2D.get(samplesIndices[1]));

                subsetCameras.clear();
                subsetCameras.add(cameras.get(samplesIndices[0]));
                subsetCameras.add(cameras.get(samplesIndices[1]));

                try {
                    triangulator.setPointsAndCameras(subsetPoints, subsetCameras);
                    final var triangulated = triangulator.triangulate();
                    solutions.add(triangulated);
                } catch (final Exception e) {
                    // if anything fails, no solution is added
                }
            }

            @Override
            public double computeResidual(final Point3D currentEstimation, final int i) {
                final var point2D = points2D.get(i);
                final var camera = cameras.get(i);

                // project estimated point with camera
                camera.project(currentEstimation, testPoint);

                // return distance of projected point respect to the original one
                // as a residual
                return testPoint.distanceTo(point2D);
            }

            @Override
            public boolean isReady() {
                return PROSACRobustSinglePoint3DTriangulator.this.isReady();
            }

            @Override
            public void onEstimateStart(final RobustEstimator<Point3D> estimator) {
                if (listener != null) {
                    listener.onTriangulateStart(PROSACRobustSinglePoint3DTriangulator.this);
                }
            }

            @Override
            public void onEstimateEnd(final RobustEstimator<Point3D> estimator) {
                if (listener != null) {
                    listener.onTriangulateEnd(PROSACRobustSinglePoint3DTriangulator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(final RobustEstimator<Point3D> estimator, final int iteration) {
                if (listener != null) {
                    listener.onTriangulateNextIteration(
                            PROSACRobustSinglePoint3DTriangulator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(final RobustEstimator<Point3D> estimator, final float progress) {
                if (listener != null) {
                    listener.onTriangulateProgressChange(
                            PROSACRobustSinglePoint3DTriangulator.this, progress);
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
     * Sets quality scores corresponding to each provided view.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than MINIMUM_SIZE.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores.length < MIN_REQUIRED_VIEWS) {
            throw new IllegalArgumentException();
        }

        this.qualityScores = qualityScores;
    }
}
