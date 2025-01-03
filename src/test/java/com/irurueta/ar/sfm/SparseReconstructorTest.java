/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidPairOfCamerasException;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.MetricTransformation3DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class SparseReconstructorTest {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_RANDOM_VALUE_PLANAR = -1500.0;
    private static final double MAX_RANDOM_VALUE_PLANAR = 1500.0;

    private static final double MIN_FOCAL_LENGTH_ESSENTIAL = 750.0;
    private static final double MAX_FOCAL_LENGTH_ESSENTIAL = 1500.0;

    private static final double MIN_FOCAL_LENGTH_DIAC = 1.0;
    private static final double MAX_FOCAL_LENGTH_DIAC = 100.0;

    private static final double MIN_PRINCIPAL_POINT_ESSENTIAL = 100.0;
    private static final double MAX_PRINCIPAL_POINT_ESSENTIAL = 400.0;

    private static final double MIN_PRINCIPAL_POINT_DIAC = 10.0;
    private static final double MAX_PRINCIPAL_POINT_DIAC = 20.0;

    private static final double MIN_ANGLE_DEGREES = -30.0;
    private static final double MAX_ANGLE_DEGREES = -15.0;

    private static final double MIN_CAMERA_SEPARATION_DIAC = 5.0;
    private static final double MAX_CAMERA_SEPARATION_DIAC = 10.0;

    private static final double MIN_CAMERA_SEPARATION_ESSENTIAL = 500.0;
    private static final double MAX_CAMERA_SEPARATION_ESSENTIAL = 1000.0;

    private static final int MIN_NUM_POINTS = 25;
    private static final int MAX_NUM_POINTS = 50;

    private static final double MIN_LAMBDA_ESSENTIAL = -1000.0;
    private static final double MAX_LAMBDA_ESSENTIAL = 1000.0;

    private static final double MIN_LAMBDA_DIAC = 100.0;
    private static final double MAX_LAMBDA_DIAC = 500.0;

    private static final int TIMES = 500;
    private static final int MAX_TRIES = 5000;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;

    private static final int MIN_TRACKED_POINTS = 10;
    private static final double NEAREST_THRESHOLD = 1e-6;

    private int viewCount = 0;
    private EstimatedFundamentalMatrix estimatedFundamentalMatrix;
    private EstimatedFundamentalMatrix estimatedFundamentalMatrix2;
    private EstimatedCamera estimatedMetricCamera1;
    private EstimatedCamera estimatedMetricCamera2;
    private EstimatedCamera estimatedMetricCamera3;
    private EstimatedCamera estimatedEuclideanCamera1;
    private EstimatedCamera estimatedEuclideanCamera2;
    private EstimatedCamera estimatedEuclideanCamera3;
    private List<ReconstructedPoint3D> metricReconstructedPoints;
    private List<ReconstructedPoint3D> euclideanReconstructedPoints;

    private double scale;
    private double scale2;

    private boolean started;
    private boolean finished;
    private boolean failed;
    private boolean cancelled;

    @BeforeEach
    void setUp() {
        viewCount = 0;
        estimatedFundamentalMatrix = estimatedFundamentalMatrix2 = null;
        estimatedMetricCamera1 = estimatedMetricCamera2 = estimatedMetricCamera3 = null;
        estimatedEuclideanCamera1 = estimatedEuclideanCamera2 = estimatedEuclideanCamera3 = null;
        metricReconstructedPoints = null;
        euclideanReconstructedPoints = null;
        started = finished = failed = cancelled = false;
    }

    @Test
    void testConstructor() {
        assertEquals(2, SparseReconstructor.MIN_NUMBER_OF_VIEWS);

        final var configuration = new SparseReconstructorConfiguration();
        final var listener = new SparseReconstructorListener() {
            @Override
            public boolean hasMoreViewsAvailable(final SparseReconstructor reconstructor) {
                return false;
            }

            @Override
            public void onRequestSamples(
                    final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                    final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                    final List<Sample2D> currentViewNewlySpawnedSamples) {
                // no action needed
            }

            @Override
            public void onSamplesAccepted(
                    final SparseReconstructor reconstructor, final int viewId,
                    final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples) {
                // no action needed
            }

            @Override
            public void onSamplesRejected(
                    final SparseReconstructor reconstructor, final int viewId,
                    final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples) {
                // no action needed
            }

            @Override
            public void onRequestMatches(
                    final SparseReconstructor reconstructor, final List<Sample2D> allPreviousViewSamples,
                    final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                    final int previousViewId, final int currentViewId, final List<MatchedSamples> matches) {
                // no action needed
            }

            @Override
            public void onFundamentalMatrixEstimated(
                    final SparseReconstructor reconstructor,
                    final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                // no action needed
            }

            @Override
            public void onMetricCameraEstimated(
                    final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                    final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                // no action needed
            }

            @Override
            public void onMetricReconstructedPointsEstimated(
                    final SparseReconstructor reconstructor, final List<MatchedSamples> matches,
                    final List<ReconstructedPoint3D> points) {
                // no action needed
            }

            @Override
            public void onEuclideanCameraEstimated(
                    final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                    final double scale, final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                // no action needed
            }

            @Override
            public void onEuclideanReconstructedPointsEstimated(
                    final SparseReconstructor reconstructor, final double scale,
                    final List<ReconstructedPoint3D> points) {
                // no action needed
            }

            @Override
            public void onStart(final SparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onFinish(final SparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onCancel(final SparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onFail(final SparseReconstructor reconstructor) {
                // no action needed
            }
        };

        // constructor with listener
        var reconstructor = new SparseReconstructor(listener);

        // check default values
        assertNotNull(reconstructor.getConfiguration());
        assertSame(listener, reconstructor.getListener());
        assertFalse(reconstructor.isRunning());
        assertFalse(reconstructor.isCancelled());
        assertFalse(reconstructor.hasFailed());
        assertFalse(reconstructor.isFinished());
        assertEquals(0, reconstructor.getViewCount());
        assertNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
        assertNull(reconstructor.getCurrentMetricEstimatedCamera());
        assertNull(reconstructor.getPreviousMetricEstimatedCamera());
        assertNull(reconstructor.getCurrentEuclideanEstimatedCamera());
        assertNull(reconstructor.getPreviousEuclideanEstimatedCamera());
        assertNull(reconstructor.getActiveMetricReconstructedPoints());
        assertNull(reconstructor.getActiveEuclideanReconstructedPoints());
        assertEquals(BaseSparseReconstructor.DEFAULT_SCALE, reconstructor.getCurrentScale(), 0.0);
        assertNull(reconstructor.getPreviousViewTrackedSamples());
        assertNull(reconstructor.getCurrentViewTrackedSamples());
        assertNull(reconstructor.getCurrentViewNewlySpawnedSamples());
        assertTrue(reconstructor.isFirstView());
        assertFalse(reconstructor.isSecondView());
        assertFalse(reconstructor.isAdditionalView());

        // constructor with configuration and listener
        reconstructor = new SparseReconstructor(configuration, listener);

        // check default values
        assertSame(configuration, reconstructor.getConfiguration());
        assertSame(listener, reconstructor.getListener());
        assertFalse(reconstructor.isRunning());
        assertFalse(reconstructor.isCancelled());
        assertFalse(reconstructor.hasFailed());
        assertFalse(reconstructor.isFinished());
        assertEquals(0, reconstructor.getViewCount());
        assertNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
        assertNull(reconstructor.getCurrentMetricEstimatedCamera());
        assertNull(reconstructor.getPreviousMetricEstimatedCamera());
        assertNull(reconstructor.getCurrentEuclideanEstimatedCamera());
        assertNull(reconstructor.getPreviousEuclideanEstimatedCamera());
        assertNull(reconstructor.getActiveMetricReconstructedPoints());
        assertNull(reconstructor.getActiveEuclideanReconstructedPoints());
        assertEquals(BaseSparseReconstructor.DEFAULT_SCALE, reconstructor.getCurrentScale(), 0.0);
        assertNull(reconstructor.getPreviousViewTrackedSamples());
        assertNull(reconstructor.getCurrentViewTrackedSamples());
        assertNull(reconstructor.getCurrentViewNewlySpawnedSamples());
        assertTrue(reconstructor.isFirstView());
        assertFalse(reconstructor.isSecondView());
        assertFalse(reconstructor.isAdditionalView());
    }

    @Test
    void testGeneralPointsEssentialTwoViews() throws InvalidPairOfCamerasException, AlgebraException, CameraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getInitialCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPoint,
                    principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final var decomposer = new SingularValueDecomposer(planesIntersectionMatrix);
            decomposer.decompose();
            final var v = decomposer.getV();
            final var centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final var numPoints1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPoints2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final var points3D1 = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            for (var i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front1 || !front2);
                points3D1.add(point3D);

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            Point2D projectedPoint2b;
            final var projectedPoints2b = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints2; i++) {
                // generate new spawned point in front of camera 2
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front2);

                // here 3D point is in front of camera 2

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            final var listener = new SparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final SparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamples(
                        final SparseReconstructor reconstructor,
                        final int previousViewId, final int currentViewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples,
                        final List<Sample2D> currentViewNewlySpawnedSamples) {

                    previousViewTrackedSamples.clear();
                    currentViewTrackedSamples.clear();
                    currentViewNewlySpawnedSamples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        // spawned samples
                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewNewlySpawnedSamples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final SparseReconstructor reconstructor, final List<Sample2D> allPreviousViewSamples,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final int previousViewId, final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints1; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                previousViewTrackedSamples.get(i), currentViewTrackedSamples.get(i)
                        });
                        match.setViewIds(new int[]{previousViewId, currentViewId});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final SparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    SparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onMetricCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedMetricCamera1 = previousCamera;
                    estimatedMetricCamera2 = currentCamera;
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final double scale, final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedEuclideanCamera1 = previousCamera;
                    estimatedEuclideanCamera2 = currentCamera;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onStart(final SparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final SparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final SparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final SparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(started);
            assertTrue(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(estimatedMetricCamera2, reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(estimatedMetricCamera1, reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(metricReconstructedPoints, reconstructor.getActiveMetricReconstructedPoints());
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getActiveEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            estimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final var estMetricCam1 = this.estimatedMetricCamera1.getCamera();
            final var estMetricCam2 = this.estimatedMetricCamera2.getCamera();
            assertSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);

            estMetricCam1.decompose();
            estMetricCam2.decompose();

            assertSame(metricReconstructedPoints, euclideanReconstructedPoints);

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints1; i++) {
                final var p = metricReconstructedPoints3D.get(i);
                assertTrue(estMetricCam1.isPointInFrontOfCamera(p));
                assertTrue(estMetricCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estMetricCam1.getCameraCenter();
            final var estimatedCenter2 = estMetricCam2.getCameraCenter();

            // transform points and cameras to account for scale change
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);
            final var s = baseline / estimatedBaseline;
            assertEquals(1.0, this.scale, 0.0);

            final var scaleTransformation = new MetricTransformation3D(s);

            final var scaledCamera1 = scaleTransformation.transformAndReturnNew(estMetricCam1);
            final var scaledCamera2 = scaleTransformation.transformAndReturnNew(estMetricCam2);

            final var scaledReconstructionPoints3D = scaleTransformation.transformPointsAndReturnNew(
                    metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();

            final var scaledCenter1 = scaledCamera1.getCameraCenter();
            final var scaledCenter2 = scaledCamera2.getCameraCenter();

            final var scaledIntrinsic1 = scaledCamera1.getIntrinsicParameters();
            final var scaledIntrinsic2 = scaledCamera2.getIntrinsicParameters();

            final var scaledRotation1 = scaledCamera1.getCameraRotation();
            final var scaledRotation2 = scaledCamera2.getCameraRotation();

            final var scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

            // check cameras are correct
            if (Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(center1.equals(scaledCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR));

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsDIACTwoViews() throws InvalidPairOfCamerasException, AlgebraException, CameraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_DIAC, MAX_FOCAL_LENGTH_DIAC);
            final var aspectRatio = configuration.getInitialCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);
            configuration.setInitialCamerasAspectRatio(aspectRatio);

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_DIAC, MAX_CAMERA_SEPARATION_DIAC);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final var decomposer = new SingularValueDecomposer(planesIntersectionMatrix);
            decomposer.decompose();
            final var v = decomposer.getV();
            final var centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final var numPoints1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPoints2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final var points3D1 = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            var maxTriesReached = false;
            for (var i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        maxTriesReached = true;
                        break;
                    }
                    numTry++;
                } while (!front1 || !front2);

                if (maxTriesReached) {
                    break;
                }

                points3D1.add(point3D);

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            if (maxTriesReached) {
                continue;
            }

            Point2D projectedPoint2b;
            final var projectedPoints2b = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints2; i++) {
                // generate new spawned point in front of camera 2
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        maxTriesReached = true;
                        break;
                    }
                    numTry++;
                } while (!front2);

                if (maxTriesReached) {
                    break;
                }

                // here 3D point is in front of both cameras

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            final var listener = new SparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final SparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamples(
                        final SparseReconstructor reconstructor,
                        final int previousViewId, final int currentViewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples,
                        final List<Sample2D> currentViewNewlySpawnedSamples) {

                    previousViewTrackedSamples.clear();
                    currentViewTrackedSamples.clear();
                    currentViewNewlySpawnedSamples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        // spawned samples
                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewNewlySpawnedSamples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final SparseReconstructor reconstructor, final List<Sample2D> allPreviousViewSamples,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final int previousViewId, final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints1; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                previousViewTrackedSamples.get(i), currentViewTrackedSamples.get(i)
                        });
                        match.setViewIds(new int[]{previousViewId, currentViewId});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final SparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    SparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onMetricCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedMetricCamera1 = previousCamera;
                    estimatedMetricCamera2 = currentCamera;
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final double scale, final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedEuclideanCamera1 = previousCamera;
                    estimatedEuclideanCamera2 = currentCamera;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onStart(final SparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final SparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final SparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final SparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            if (!finished) {
                continue;
            }

            // check correctness
            assertTrue(started);
            assertFalse(cancelled);
            assertFalse(failed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(estimatedMetricCamera2, reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(estimatedMetricCamera1, reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(metricReconstructedPoints, reconstructor.getActiveMetricReconstructedPoints());
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getActiveEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            estimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                            estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final var estMetricCam1 = this.estimatedMetricCamera1.getCamera();
            final var estMetricCam2 = this.estimatedMetricCamera2.getCamera();
            assertSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);

            estMetricCam1.decompose();
            estMetricCam2.decompose();

            assertSame(metricReconstructedPoints, euclideanReconstructedPoints);

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
            }

            // check that most of the points are in front of both cameras
            var valid = 0;
            var invalid = 0;
            for (var i = 0; i < numPoints1; i++) {
                if (metricReconstructedPoints.get(i).isInlier()) {
                    final var p = metricReconstructedPoints3D.get(i);
                    assertTrue(estMetricCam1.isPointInFrontOfCamera(p));
                    assertTrue(estMetricCam2.isPointInFrontOfCamera(p));
                    valid++;
                } else {
                    invalid++;
                }
            }

            assertTrue(valid >= invalid);

            final var estimatedCenter1 = estMetricCam1.getCameraCenter();
            final var estimatedCenter2 = estMetricCam2.getCameraCenter();

            // transform points and cameras to account for scale change
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);
            final var s = baseline / estimatedBaseline;
            assertEquals(1.0, this.scale, 0.0);

            final var scaleTransformation = new MetricTransformation3D(s);

            final var scaledCamera1 = scaleTransformation.transformAndReturnNew(estMetricCam1);
            final var scaledCamera2 = scaleTransformation.transformAndReturnNew(estMetricCam2);

            final var scaledReconstructionPoints3D = scaleTransformation.transformPointsAndReturnNew(
                    metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();

            final var scaledCenter1 = new InhomogeneousPoint3D(scaledCamera1.getCameraCenter());
            final var scaledCenter2 = new InhomogeneousPoint3D(scaledCamera2.getCameraCenter());

            final var scaledIntrinsic1 = scaledCamera1.getIntrinsicParameters();
            final var scaledIntrinsic2 = scaledCamera2.getIntrinsicParameters();

            final var scaledRotation1 = scaledCamera1.getCameraRotation();
            final var scaledRotation2 = scaledCamera2.getCameraRotation();

            final var scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

            // check cameras are correct
            if (Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(center1.equals(scaledCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR));

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsDAQAndEssentialZeroPrincipalPointTwoViews() throws InvalidPairOfCamerasException,
            AlgebraException, CameraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getInitialCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPointX = 0.0;
            final var principalPointY = 0.0;

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final var decomposer = new SingularValueDecomposer(planesIntersectionMatrix);
            decomposer.decompose();
            final var v = decomposer.getV();
            final var centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final var numPoints1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPoints2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final var points3D1 = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean leftFront;
            boolean rightFront;
            var maxTriesReached = false;
            for (var i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    leftFront = camera1.isPointInFrontOfCamera(point3D);
                    rightFront = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        maxTriesReached = true;
                        break;
                    }
                    numTry++;
                } while (!leftFront || !rightFront);

                if (maxTriesReached) {
                    break;
                }

                points3D1.add(point3D);

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            Point2D projectedPoint2b;
            final var projectedPoints2b = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    rightFront = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        maxTriesReached = true;
                        break;
                    }
                    numTry++;
                } while (!rightFront);

                if (maxTriesReached) {
                    break;
                }

                // here 3D point is in front of both cameras

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            if (maxTriesReached) {
                continue;
            }

            final var listener = new SparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final SparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamples(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final List<Sample2D> currentViewNewlySpawnedSamples) {

                    previousViewTrackedSamples.clear();
                    currentViewTrackedSamples.clear();
                    currentViewNewlySpawnedSamples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        // spawned samples
                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewNewlySpawnedSamples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final SparseReconstructor reconstructor, final List<Sample2D> allPreviousViewSamples,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final int previousViewId, final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints1; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                previousViewTrackedSamples.get(i), currentViewTrackedSamples.get(i)
                        });
                        match.setViewIds(new int[]{previousViewId, currentViewId});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final SparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    SparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onMetricCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedMetricCamera1 = previousCamera;
                    estimatedMetricCamera2 = currentCamera;
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final double scale, final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedEuclideanCamera1 = previousCamera;
                    estimatedEuclideanCamera2 = currentCamera;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onStart(final SparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final SparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final SparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final SparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(started);
            assertTrue(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(estimatedMetricCamera2, reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(estimatedMetricCamera1, reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(metricReconstructedPoints, reconstructor.getActiveMetricReconstructedPoints());
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getActiveEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            estimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final var estMetricCam1 = this.estimatedMetricCamera1.getCamera();
            final var estMetricCam2 = this.estimatedMetricCamera2.getCamera();
            assertSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);

            estMetricCam1.decompose();
            estMetricCam2.decompose();

            assertSame(metricReconstructedPoints, euclideanReconstructedPoints);

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints1; i++) {
                final var p = metricReconstructedPoints3D.get(i);
                assertTrue(estMetricCam1.isPointInFrontOfCamera(p));
                assertTrue(estMetricCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estMetricCam1.getCameraCenter();
            final var estimatedCenter2 = estMetricCam2.getCameraCenter();

            // transform points and cameras to account for scale change
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);
            final var s = baseline / estimatedBaseline;
            assertEquals(1.0, this.scale, 0.0);

            final var scaleTransformation = new MetricTransformation3D(s);

            final var scaledCamera1 = scaleTransformation.transformAndReturnNew(estMetricCam1);
            final var scaledCamera2 = scaleTransformation.transformAndReturnNew(estMetricCam2);

            final var scaledReconstructionPoints3D = scaleTransformation.transformPointsAndReturnNew(
                    metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();

            final var scaledCenter1 = new InhomogeneousPoint3D(scaledCamera1.getCameraCenter());
            final var scaledCenter2 = new InhomogeneousPoint3D(scaledCamera2.getCameraCenter());

            final var scaledIntrinsic1 = scaledCamera1.getIntrinsicParameters();
            final var scaledIntrinsic2 = scaledCamera2.getIntrinsicParameters();

            final var scaledRotation1 = scaledCamera1.getCameraRotation();
            final var scaledRotation2 = scaledCamera2.getCameraRotation();

            final var scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

            // check cameras are correct
            if (Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(center1.equals(scaledCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR));

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(), intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(), intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsDAQAndEssentialTwoViews() throws InvalidPairOfCamerasException, AlgebraException,
            CameraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getInitialCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT_ESSENTIAL,
                    MAX_PRINCIPAL_POINT_ESSENTIAL);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT_ESSENTIAL,
                    MAX_PRINCIPAL_POINT_ESSENTIAL);

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final var decomposer = new SingularValueDecomposer(planesIntersectionMatrix);
            decomposer.decompose();
            final var v = decomposer.getV();
            final var centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final var numPoints1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPoints2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final var points3D1 = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean leftFront;
            boolean rightFront;
            var maxTriesReached = false;
            for (var i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    leftFront = camera1.isPointInFrontOfCamera(point3D);
                    rightFront = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        maxTriesReached = true;
                        break;
                    }
                    numTry++;
                } while (!leftFront || !rightFront);

                if (maxTriesReached) {
                    break;
                }

                points3D1.add(point3D);

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            if (maxTriesReached) {
                continue;
            }

            Point2D projectedPoint2b;
            final var projectedPoints2b = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    rightFront = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        maxTriesReached = true;
                        break;
                    }
                    numTry++;
                } while (!rightFront);

                if (maxTriesReached) {
                    break;
                }

                // here 3D point is in front of both cameras

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            if (maxTriesReached) {
                continue;
            }

            final var listener = new SparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final SparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamples(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final List<Sample2D> currentViewNewlySpawnedSamples) {

                    previousViewTrackedSamples.clear();
                    currentViewTrackedSamples.clear();
                    currentViewNewlySpawnedSamples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        // spawned samples
                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewNewlySpawnedSamples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final SparseReconstructor reconstructor, final List<Sample2D> allPreviousViewSamples,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final int previousViewId, final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints1; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                previousViewTrackedSamples.get(i), currentViewTrackedSamples.get(i)
                        });
                        match.setViewIds(new int[]{previousViewId, currentViewId});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final SparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    SparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onMetricCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedMetricCamera1 = previousCamera;
                    estimatedMetricCamera2 = currentCamera;
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final double scale, final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedEuclideanCamera1 = previousCamera;
                    estimatedEuclideanCamera2 = currentCamera;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onStart(final SparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final SparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final SparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final SparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(started);
            assertTrue(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(estimatedMetricCamera2, reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(estimatedMetricCamera1, reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(metricReconstructedPoints, reconstructor.getActiveMetricReconstructedPoints());
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getActiveEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            estimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final var estMetricCam1 = this.estimatedMetricCamera1.getCamera();
            final var estMetricCam2 = this.estimatedMetricCamera2.getCamera();
            assertSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);

            estMetricCam1.decompose();
            estMetricCam2.decompose();

            assertSame(metricReconstructedPoints, euclideanReconstructedPoints);

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints1; i++) {
                final var p = metricReconstructedPoints3D.get(i);
                assertTrue(estMetricCam1.isPointInFrontOfCamera(p));
                assertTrue(estMetricCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estMetricCam1.getCameraCenter();
            final var estimatedCenter2 = estMetricCam2.getCameraCenter();

            // transform points and cameras to account for scale change
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);
            final var s = baseline / estimatedBaseline;
            assertEquals(1.0, this.scale, 0.0);

            final var scaleTransformation = new MetricTransformation3D(s);

            final var scaledCamera1 = scaleTransformation.transformAndReturnNew(estMetricCam1);
            final var scaledCamera2 = scaleTransformation.transformAndReturnNew(estMetricCam2);

            final var scaledReconstructionPoints3D = scaleTransformation.transformPointsAndReturnNew(
                    metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();

            final var scaledCenter1 = new InhomogeneousPoint3D(scaledCamera1.getCameraCenter());
            final var scaledCenter2 = new InhomogeneousPoint3D(scaledCamera2.getCameraCenter());

            final var scaledIntrinsic1 = scaledCamera1.getIntrinsicParameters();
            final var scaledIntrinsic2 = scaledCamera2.getIntrinsicParameters();

            final var scaledRotation1 = scaledCamera1.getCameraRotation();
            final var scaledRotation2 = scaledCamera2.getCameraRotation();

            final var scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

            // check cameras are correct
            if (Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(center1.equals(scaledCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR));

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(), intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(), intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsDAQTwoViews() throws InvalidPairOfCamerasException, AlgebraException, CameraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException,
            com.irurueta.geometry.estimators.LockedException, RobustEstimatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getInitialCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT_ESSENTIAL,
                    MAX_PRINCIPAL_POINT_ESSENTIAL);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT_ESSENTIAL,
                    MAX_PRINCIPAL_POINT_ESSENTIAL);

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final var decomposer = new SingularValueDecomposer(planesIntersectionMatrix);
            decomposer.decompose();
            final var v = decomposer.getV();
            final var centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final var numPoints1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPoints2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final var points3D1 = new ArrayList<Point3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean leftFront;
            boolean rightFront;
            var maxTriesReached = false;
            for (var i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    leftFront = camera1.isPointInFrontOfCamera(point3D);
                    rightFront = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        maxTriesReached = true;
                        break;
                    }
                    numTry++;
                } while (!leftFront || !rightFront);

                if (maxTriesReached) {
                    break;
                }

                points3D1.add(point3D);

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            Point2D projectedPoint2b;
            final var projectedPoints2b = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    rightFront = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        maxTriesReached = true;
                        break;
                    }
                    numTry++;
                } while (!rightFront);

                if (maxTriesReached) {
                    break;
                }

                // here 3D point is in front of both cameras

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            final var listener = new SparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final SparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamples(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final List<Sample2D> currentViewNewlySpawnedSamples) {

                    previousViewTrackedSamples.clear();
                    currentViewTrackedSamples.clear();
                    currentViewNewlySpawnedSamples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        // spawned samples
                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewNewlySpawnedSamples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final SparseReconstructor reconstructor, final List<Sample2D> allPreviousViewSamples,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final int previousViewId, final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints1; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                previousViewTrackedSamples.get(i), currentViewTrackedSamples.get(i)
                        });
                        match.setViewIds(new int[]{previousViewId, currentViewId});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final SparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    SparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onMetricCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedMetricCamera1 = previousCamera;
                    estimatedMetricCamera2 = currentCamera;
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final double scale, final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedEuclideanCamera1 = previousCamera;
                    estimatedEuclideanCamera2 = currentCamera;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onStart(final SparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final SparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final SparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final SparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            if (maxTriesReached) {
                continue;
            }

            final var reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(started);
            assertTrue(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(estimatedMetricCamera2, reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(estimatedMetricCamera1, reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(metricReconstructedPoints, reconstructor.getActiveMetricReconstructedPoints());
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getActiveEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            estimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final var estMetricCam1 = this.estimatedMetricCamera1.getCamera();
            final var estMetricCam2 = this.estimatedMetricCamera2.getCamera();
            assertSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);

            assertSame(metricReconstructedPoints, euclideanReconstructedPoints);

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
            }

            final var transformationEstimator = MetricTransformation3DRobustEstimator.create(
                    metricReconstructedPoints3D, points3D1, RobustEstimatorMethod.LMEDS);

            final var transformation = transformationEstimator.estimate();

            final var transformedCamera1 = transformation.transformAndReturnNew(estMetricCam1);
            final var transformedCamera2 = transformation.transformAndReturnNew(estMetricCam2);

            // check cameras intrinsics are correct (rotation, center and points
            // might contain large errors and for that reason we do not checked)
            transformedCamera1.decompose();
            transformedCamera2.decompose();

            final var transformedIntrinsic1 = transformedCamera1.getIntrinsicParameters();
            final var transformedIntrinsic2 = transformedCamera2.getIntrinsicParameters();

            assertEquals(transformedIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(transformedIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(transformedIntrinsic1.getSkewness(), intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(transformedIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(transformedIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            assertEquals(transformedIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(transformedIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(transformedIntrinsic2.getSkewness(), intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(transformedIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(transformedIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testPlanarPointsEssentialTwoViews() throws InvalidPairOfCamerasException, AlgebraException, CameraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getInitialCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPointX = 0.0;
            final var principalPointY = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras and laying in a plane

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final var decomposer = new SingularValueDecomposer(planesIntersectionMatrix);
            decomposer.decompose();
            final var v = decomposer.getV();
            final var centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            final var principalAxis1 = camera1.getPrincipalAxisArray();
            final var principalAxis2 = camera2.getPrincipalAxisArray();
            final var avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2), 0.5);

            final var plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();

            final var planeA = plane.getA();
            final var planeB = plane.getB();
            final var planeC = plane.getC();
            final var planeD = plane.getD();

            final var numPoints1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPoints2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            final var points3D1 = new ArrayList<HomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            for (var i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final var homW = 1.0;
                    final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) / planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) / planeA;
                    }

                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                    assertTrue(plane.isLocus(point3D));

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front1 || !front2);
                points3D1.add(point3D);

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            Point2D projectedPoint2b;
            final var projectedPoints2b = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final var homW = 1.0;
                    final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) / planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) / planeA;
                    }

                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                    assertTrue(plane.isLocus(point3D));

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front2);

                // here 3D point is in front of both cameras

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            final var listener = new SparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final SparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamples(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final List<Sample2D> currentViewNewlySpawnedSamples) {

                    previousViewTrackedSamples.clear();
                    currentViewTrackedSamples.clear();
                    currentViewNewlySpawnedSamples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        // spawned samples
                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewNewlySpawnedSamples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final SparseReconstructor reconstructor, final List<Sample2D> allPreviousViewSamples,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final int previousViewId, final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints1; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                previousViewTrackedSamples.get(i), currentViewTrackedSamples.get(i)
                        });
                        match.setViewIds(new int[]{previousViewId, currentViewId});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final SparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    SparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onMetricCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedMetricCamera1 = previousCamera;
                    estimatedMetricCamera2 = currentCamera;
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final double scale, final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedEuclideanCamera1 = previousCamera;
                    estimatedEuclideanCamera2 = currentCamera;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onStart(final SparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final SparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final SparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final SparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            if (!finished) {
                continue;
            }
            assertTrue(started);
            assertFalse(cancelled);
            assertFalse(failed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertTrue(reconstructor.getViewCount() > 0);
            if (reconstructor.getCurrentMetricEstimatedCamera() == null) {
                continue;
            }
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(estimatedMetricCamera2, reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(estimatedMetricCamera1, reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(metricReconstructedPoints, reconstructor.getActiveMetricReconstructedPoints());
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getActiveEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
            if (estimatedFundamentalMatrix == null || estimatedFundamentalMatrix.getFundamentalMatrix() == null) {
                continue;
            }

            fundamentalMatrix.normalize();
            estimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final var estMetricCam1 = this.estimatedMetricCamera1.getCamera();
            final var estMetricCam2 = this.estimatedMetricCamera2.getCamera();
            assertSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);

            assertSame(metricReconstructedPoints, euclideanReconstructedPoints);

            estMetricCam1.decompose();
            estMetricCam2.decompose();

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints1; i++) {
                final var p = metricReconstructedPoints3D.get(i);
                assertTrue(estMetricCam1.isPointInFrontOfCamera(p));
                assertTrue(estMetricCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estMetricCam1.getCameraCenter();
            final var estimatedCenter2 = estMetricCam2.getCameraCenter();

            // transform points and cameras to account for scale change
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);
            final var s = baseline / estimatedBaseline;
            assertEquals(1.0, this.scale, 0.0);

            final var scaleTransformation = new MetricTransformation3D(s);

            final var scaledCamera1 = scaleTransformation.transformAndReturnNew(estMetricCam1);
            final var scaledCamera2 = scaleTransformation.transformAndReturnNew(estMetricCam2);

            final var scaledReconstructionPoints3D = scaleTransformation.transformPointsAndReturnNew(
                    metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();

            final var scaledCenter1 = scaledCamera1.getCameraCenter();
            final var scaledCenter2 = scaledCamera2.getCameraCenter();

            final var scaledIntrinsic1 = scaledCamera1.getIntrinsicParameters();
            final var scaledIntrinsic2 = scaledCamera2.getIntrinsicParameters();

            final var scaledRotation1 = scaledCamera1.getCameraRotation();
            final var scaledRotation2 = scaledCamera2.getCameraRotation();

            final var scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

            // check cameras are correct
            if (Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(center1.equals(scaledCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR));

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            if (!scaledRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    LARGE_ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testPlanarPointsDIACTwoViews() throws InvalidPairOfCamerasException, AlgebraException, CameraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getInitialCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);
            configuration.setInitialCamerasAspectRatio(aspectRatio);

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras and laying in a plane

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final var decomposer = new SingularValueDecomposer(planesIntersectionMatrix);
            decomposer.decompose();
            final var v = decomposer.getV();
            final var centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            final var principalAxis1 = camera1.getPrincipalAxisArray();
            final var principalAxis2 = camera2.getPrincipalAxisArray();
            final var avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2), 0.5);

            final var plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();

            final var planeA = plane.getA();
            final var planeB = plane.getB();
            final var planeC = plane.getC();
            final var planeD = plane.getD();

            final var numPoints1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPoints2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            for (var i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final var homW = 1.0;
                    final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) / planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) / planeA;
                    }

                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                    assertTrue(plane.isLocus(point3D));

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front1 || !front2);

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            Point2D projectedPoint2b;
            final var projectedPoints2b = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final var homW = 1.0;
                    final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) / planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) / planeA;
                    }

                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                    assertTrue(plane.isLocus(point3D));

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front2);

                // here 3D point is in front of both cameras

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            final var listener = new SparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final SparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamples(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final List<Sample2D> currentViewNewlySpawnedSamples) {

                    previousViewTrackedSamples.clear();
                    currentViewTrackedSamples.clear();
                    currentViewNewlySpawnedSamples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        // spawned samples
                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewNewlySpawnedSamples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final SparseReconstructor reconstructor, final List<Sample2D> allPreviousViewSamples,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final int previousViewId, final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints1; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                previousViewTrackedSamples.get(i), currentViewTrackedSamples.get(i)
                        });
                        match.setViewIds(new int[]{previousViewId, currentViewId});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final SparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    SparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onMetricCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedMetricCamera1 = previousCamera;
                    estimatedMetricCamera2 = currentCamera;
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final double scale, final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedEuclideanCamera1 = previousCamera;
                    estimatedEuclideanCamera2 = currentCamera;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onStart(final SparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final SparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final SparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final SparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            if (!finished || failed) {
                continue;
            }
            assertTrue(started);
            assertFalse(cancelled);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertTrue(reconstructor.getViewCount() > 0);
            if (reconstructor.getCurrentMetricEstimatedCamera() == null) {
                continue;
            }
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(estimatedMetricCamera2, reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(estimatedMetricCamera1, reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(metricReconstructedPoints, reconstructor.getActiveMetricReconstructedPoints());
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getActiveEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());


            // check that estimated fundamental matrix is correct
            if (estimatedFundamentalMatrix == null || estimatedFundamentalMatrix.getFundamentalMatrix() == null) {
                continue;
            }

            fundamentalMatrix.normalize();
            estimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), LARGE_ABSOLUTE_ERROR)
                    && !fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(),
                    LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), LARGE_ABSOLUTE_ERROR)
                    || fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(),
                    LARGE_ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final var estMetricCam1 = this.estimatedMetricCamera1.getCamera();
            final var estMetricCam2 = this.estimatedMetricCamera2.getCamera();
            assertSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);

            assertSame(metricReconstructedPoints, euclideanReconstructedPoints);

            estMetricCam1.decompose();
            estMetricCam2.decompose();

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
            }

            // check that most of the points are in front of both cameras
            var valid = 0;
            var invalid = 0;
            for (var i = 0; i < numPoints1; i++) {
                if (metricReconstructedPoints.get(i).isInlier()) {
                    final var p = metricReconstructedPoints3D.get(i);
                    assertTrue(estMetricCam1.isPointInFrontOfCamera(p));
                    assertTrue(estMetricCam2.isPointInFrontOfCamera(p));
                    valid++;
                } else {
                    invalid++;
                }
            }

            if (valid < invalid) {
                continue;
            }

            final var estimatedCenter1 = estMetricCam1.getCameraCenter();
            final var estimatedCenter2 = estMetricCam2.getCameraCenter();

            // transform points and cameras to account for scale change
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);
            final var s = baseline / estimatedBaseline;
            assertEquals(1.0, this.scale, 0.0);

            final var scaleTransformation = new MetricTransformation3D(s);

            final var scaledCamera1 = scaleTransformation.transformAndReturnNew(estMetricCam1);
            final var scaledCamera2 = scaleTransformation.transformAndReturnNew(estMetricCam2);

            scaledCamera1.decompose();
            scaledCamera2.decompose();

            final var scaledCenter1 = new InhomogeneousPoint3D(scaledCamera1.getCameraCenter());
            final var scaledCenter2 = new InhomogeneousPoint3D(scaledCamera2.getCameraCenter());

            final var scaledRotation1 = scaledCamera1.getCameraRotation();

            final var scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

            // check cameras are correct
            if (Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testPlanarPointsDAQAndEssentialZeroPrincipalPointTwoViews() throws InvalidPairOfCamerasException,
            AlgebraException, CameraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getInitialCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPointX = 0.0;
            final var principalPointY = 0.0;

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras and laying in a plane

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final var decomposer = new SingularValueDecomposer(planesIntersectionMatrix);
            decomposer.decompose();
            final var v = decomposer.getV();
            final var centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            final var principalAxis1 = camera1.getPrincipalAxisArray();
            final var principalAxis2 = camera2.getPrincipalAxisArray();
            final var avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2), 0.5);

            final var plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();

            final var planeA = plane.getA();
            final var planeB = plane.getB();
            final var planeC = plane.getC();
            final var planeD = plane.getD();

            final var numPoints1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPoints2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            final var points3D1 = new ArrayList<HomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            for (var i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final var homW = 1.0;
                    final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) / planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) / planeA;
                    }

                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                    assertTrue(plane.isLocus(point3D));

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front1 || !front2);
                points3D1.add(point3D);

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            Point2D projectedPoint2b;
            final var projectedPoints2b = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final var homW = 1.0;
                    final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) / planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) / planeA;
                    }

                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                    assertTrue(plane.isLocus(point3D));

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front2);

                // here 3D point is in front of both cameras

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            final var listener = new SparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final SparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamples(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final List<Sample2D> currentViewNewlySpawnedSamples) {

                    previousViewTrackedSamples.clear();
                    currentViewTrackedSamples.clear();
                    currentViewNewlySpawnedSamples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        // spawned samples
                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewNewlySpawnedSamples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final SparseReconstructor reconstructor, final List<Sample2D> allPreviousViewSamples,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final int previousViewId, final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints1; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                previousViewTrackedSamples.get(i), currentViewTrackedSamples.get(i)
                        });
                        match.setViewIds(new int[]{previousViewId, currentViewId});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final SparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    SparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onMetricCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedMetricCamera1 = previousCamera;
                    estimatedMetricCamera2 = currentCamera;
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final double scale, final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedEuclideanCamera1 = previousCamera;
                    estimatedEuclideanCamera2 = currentCamera;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onStart(final SparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final SparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final SparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final SparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            if (!finished || failed) {
                continue;
            }
            assertTrue(started);
            assertFalse(cancelled);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertTrue(reconstructor.getViewCount() > 0);
            if (reconstructor.getCurrentMetricEstimatedCamera() == null) {
                continue;
            }
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(estimatedMetricCamera2, reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(estimatedMetricCamera1, reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(metricReconstructedPoints, reconstructor.getActiveMetricReconstructedPoints());
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getActiveEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
            if (estimatedFundamentalMatrix == null || estimatedFundamentalMatrix.getFundamentalMatrix() == null) {
                continue;
            }

            fundamentalMatrix.normalize();
            estimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), LARGE_ABSOLUTE_ERROR)
                    && !fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(),
                    LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), LARGE_ABSOLUTE_ERROR)
                    || fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(),
                    LARGE_ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final var estMetricCam1 = this.estimatedMetricCamera1.getCamera();
            final var estMetricCam2 = this.estimatedMetricCamera2.getCamera();
            assertSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);

            assertSame(metricReconstructedPoints, euclideanReconstructedPoints);

            estMetricCam1.decompose();
            estMetricCam2.decompose();

            if (metricReconstructedPoints.size() < numPoints1) {
                continue;
            }

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints1; i++) {
                final var p = metricReconstructedPoints3D.get(i);
                assertTrue(estMetricCam1.isPointInFrontOfCamera(p));
                assertTrue(estMetricCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estMetricCam1.getCameraCenter();
            final var estimatedCenter2 = estMetricCam2.getCameraCenter();

            // transform points and cameras to account for scale change
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);
            final var s = baseline / estimatedBaseline;

            final var scaleTransformation = new MetricTransformation3D(s);

            final var scaledCamera1 = scaleTransformation.transformAndReturnNew(estMetricCam1);
            final var scaledCamera2 = scaleTransformation.transformAndReturnNew(estMetricCam2);

            final var scaledReconstructionPoints3D = scaleTransformation.transformPointsAndReturnNew(
                    metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();

            final var scaledCenter1 = scaledCamera1.getCameraCenter();
            final var scaledCenter2 = scaledCamera2.getCameraCenter();

            final var scaledRotation1 = scaledCamera1.getCameraRotation();

            final var scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

            // check cameras are correct
            if (Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(center1.equals(scaledCenter1, LARGE_ABSOLUTE_ERROR));

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testPlanarPointsDAQTwoViews() throws InvalidPairOfCamerasException, AlgebraException, CameraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getInitialCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT_ESSENTIAL,
                    MAX_PRINCIPAL_POINT_ESSENTIAL);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT_ESSENTIAL,
                    MAX_PRINCIPAL_POINT_ESSENTIAL);

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras and laying in a plane

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final var decomposer = new SingularValueDecomposer(planesIntersectionMatrix);
            decomposer.decompose();
            final var v = decomposer.getV();
            final var centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            final var principalAxis1 = camera1.getPrincipalAxisArray();
            final var principalAxis2 = camera2.getPrincipalAxisArray();
            final var avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2), 0.5);

            final var plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();

            final var planeA = plane.getA();
            final var planeB = plane.getB();
            final var planeC = plane.getC();
            final var planeD = plane.getD();

            final var numPoints1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPoints2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            var failedIter = false;
            for (var i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final var homW = 1.0;
                    final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) / planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) / planeA;
                    }

                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                    assertTrue(plane.isLocus(point3D));

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        failedIter = true;
                        break;
                    }
                    numTry++;
                } while (!front1 || !front2);

                if (failedIter) {
                    break;
                }

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            if (failedIter) {
                continue;
            }

            Point2D projectedPoint2b;
            final var projectedPoints2b = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final var homW = 1.0;
                    final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) / planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR, MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) / planeA;
                    }

                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                    assertTrue(plane.isLocus(point3D));

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        failedIter = true;
                        break;
                    }
                    numTry++;
                } while (!front2);

                if (failedIter) {
                    break;
                }

                // here 3D point is in front of both cameras

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            if (failedIter) {
                continue;
            }

            final var listener = new SparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final SparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamples(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final List<Sample2D> currentViewNewlySpawnedSamples) {

                    previousViewTrackedSamples.clear();
                    currentViewTrackedSamples.clear();
                    currentViewNewlySpawnedSamples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        // spawned samples
                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewNewlySpawnedSamples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final SparseReconstructor reconstructor, final List<Sample2D> allPreviousViewSamples,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final int previousViewId, final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints1; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                previousViewTrackedSamples.get(i), currentViewTrackedSamples.get(i)
                        });
                        match.setViewIds(new int[]{previousViewId, currentViewId});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final SparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    SparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onMetricCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedMetricCamera1 = previousCamera;
                    estimatedMetricCamera2 = currentCamera;
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final double scale, final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedEuclideanCamera1 = previousCamera;
                    estimatedEuclideanCamera2 = currentCamera;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    SparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onStart(final SparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final SparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final SparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final SparseReconstructor reconstructor) {
                    SparseReconstructorTest.this.failed = true;
                }
            };

            final var reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(this.failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            if (!finished || this.failed) {
                continue;
            }
            assertTrue(started);
            assertFalse(cancelled);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertTrue(reconstructor.getViewCount() > 0);
            if (reconstructor.getCurrentMetricEstimatedCamera() == null) {
                continue;
            }
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(estimatedMetricCamera2, reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(estimatedMetricCamera1, reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(metricReconstructedPoints, reconstructor.getActiveMetricReconstructedPoints());
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getActiveEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
            if (estimatedFundamentalMatrix == null || estimatedFundamentalMatrix.getFundamentalMatrix() == null) {
                continue;
            }

            fundamentalMatrix.normalize();
            estimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), LARGE_ABSOLUTE_ERROR)
                    && !fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), LARGE_ABSOLUTE_ERROR)
                    || fundamentalMatrix.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(),
                    LARGE_ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final var estMetricCam1 = this.estimatedMetricCamera1.getCamera();
            final var estMetricCam2 = this.estimatedMetricCamera2.getCamera();
            assertSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);

            assertSame(metricReconstructedPoints, euclideanReconstructedPoints);

            estMetricCam1.decompose();
            estMetricCam2.decompose();

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsEssentialEPnPAdditionalIntrinsicThreeViews() throws InvalidPairOfCamerasException,
            AlgebraException, CameraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getInitialCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPoint,
                    principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);
            configuration.setAdditionalCamerasIntrinsics(intrinsic);
            configuration.setUseEPnPForAdditionalCamerasEstimation(true);
            configuration.setUseUPnPForAdditionalCamerasEstimation(false);
            configuration.setUseDAQForAdditionalCamerasIntrinics(false);
            configuration.setUseDIACForAdditionalCamerasIntrinsics(false);

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var alphaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation1 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            final var cameraSeparation2 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation1,
                    center1.getInhomY() + cameraSeparation1,
                    center1.getInhomZ() + cameraSeparation1);
            final var center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);
            final var rotation3 = new MatrixRotation3D(alphaEuler3, betaEuler3, gammaEuler3);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);
            final var camera3 = new PinholeCamera(intrinsic, rotation3, center3);

            final var fundamentalMatrix1 = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of 1st two cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final var decomposer = new SingularValueDecomposer(planesIntersectionMatrix);
            decomposer.decompose();
            final var v = decomposer.getV();
            final var centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final var numPoints1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPoints2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var start = randomizer.nextInt(0, numPoints1 - MIN_TRACKED_POINTS);

            InhomogeneousPoint3D point3D;
            final var points3D1 = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            Point2D projectedPoint3;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            final var projectedPoints3 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            for (var i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front1 || !front2);
                points3D1.add(point3D);

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);

                projectedPoint3 = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3);
                projectedPoints3.add(projectedPoint3);
            }

            final var points3D2 = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint2b;
            Point2D projectedPoint3b;
            final var projectedPoints2b = new ArrayList<Point2D>();
            final var projectedPoints3b = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front2);
                points3D2.add(point3D);

                // here 3D point is in front of both cameras

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);

                projectedPoint3b = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3b);
                projectedPoints3b.add(projectedPoint3b);
            }

            final var listener = new SparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final SparseReconstructor reconstructor) {
                    return viewCount < 3;
                }

                @Override
                public void onRequestSamples(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final List<Sample2D> currentViewNewlySpawnedSamples) {

                    previousViewTrackedSamples.clear();
                    currentViewTrackedSamples.clear();
                    currentViewNewlySpawnedSamples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    } else if (estimatedFundamentalMatrix == null) {
                        // second view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        // spawned samples
                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewNewlySpawnedSamples.add(sample);
                        }
                    } else {
                        // third view
                        for (var i = start; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = start; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints3.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints3b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final SparseReconstructor reconstructor, final List<Sample2D> allPreviousViewSamples,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final int previousViewId, final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    var numCameras = 0;
                    if (estimatedMetricCamera1 != null && (estimatedMetricCamera1.getViewId() == previousViewId
                            || estimatedMetricCamera1.getViewId() == currentViewId)) {
                        numCameras++;
                    }
                    if (estimatedMetricCamera2 != null && (estimatedMetricCamera2.getViewId() == previousViewId
                            || estimatedMetricCamera2.getViewId() == currentViewId)) {
                        numCameras++;
                    }

                    EstimatedCamera[] estimatedCameras = null;
                    if (numCameras > 0) {
                        estimatedCameras = new EstimatedCamera[numCameras];

                        var pos = 0;
                        if (estimatedMetricCamera1 != null && (estimatedMetricCamera1.getViewId() == previousViewId
                                || estimatedMetricCamera1.getViewId() == currentViewId)) {
                            estimatedCameras[pos] = estimatedMetricCamera1;
                            pos++;
                        }
                        if (estimatedMetricCamera2 != null && (estimatedMetricCamera2.getViewId() == previousViewId
                                || estimatedMetricCamera2.getViewId() == currentViewId)) {
                            estimatedCameras[pos] = estimatedMetricCamera2;
                        }
                    }

                    final var allPreviousPoints = new ArrayList<Point2D>();
                    for (final var sample : allPreviousViewSamples) {
                        allPreviousPoints.add(sample.getPoint());
                    }
                    final var tree = new KDTree2D(allPreviousPoints);

                    // search previous view tracked samples within tree
                    final var numTrackedSamples = previousViewTrackedSamples.size();
                    Point2D point;
                    Point2D nearestPoint;
                    int nearestIndex;
                    MatchedSamples match;
                    for (var i = 0; i < numTrackedSamples; i++) {
                        final var previousSample = previousViewTrackedSamples.get(i);
                        point = previousSample.getPoint();
                        nearestIndex = tree.nearestIndex(point);
                        nearestPoint = allPreviousPoints.get(nearestIndex);
                        final var nearestSample = allPreviousViewSamples.get(nearestIndex);

                        if (point.distanceTo(nearestPoint) > NEAREST_THRESHOLD) {
                            continue;
                        }

                        final var currentSample = currentViewTrackedSamples.get(i);

                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                previousSample, currentSample
                        });
                        match.setViewIds(new int[]{previousViewId, currentViewId});

                        match.setReconstructedPoint(nearestSample.getReconstructedPoint());

                        if (estimatedCameras != null) {
                            match.setCameras(estimatedCameras);
                        }

                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final SparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    if (SparseReconstructorTest.this.estimatedFundamentalMatrix == null) {
                        SparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                    } else if (estimatedFundamentalMatrix2 == null) {
                        estimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                    }
                }

                @Override
                public void onMetricCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    if (estimatedMetricCamera2 == null) {
                        estimatedMetricCamera1 = previousCamera;
                        estimatedMetricCamera2 = currentCamera;
                    } else if (estimatedMetricCamera3 == null) {
                        estimatedMetricCamera2 = previousCamera;
                        estimatedMetricCamera3 = currentCamera;
                    }
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final double scale, final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    if (estimatedEuclideanCamera2 == null) {
                        estimatedEuclideanCamera1 = previousCamera;
                        estimatedEuclideanCamera2 = currentCamera;
                        SparseReconstructorTest.this.scale = scale;
                    } else if (estimatedEuclideanCamera3 == null) {
                        estimatedEuclideanCamera2 = previousCamera;
                        estimatedEuclideanCamera3 = currentCamera;
                        scale2 = scale;
                    }
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    if (euclideanReconstructedPoints == null) {
                        SparseReconstructorTest.this.scale = scale;
                    } else {
                        scale2 = scale;
                    }

                    euclideanReconstructedPoints = points;
                }

                @Override
                public void onStart(final SparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final SparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final SparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final SparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(started);
            assertTrue(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(estimatedMetricCamera3, reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(estimatedMetricCamera2, reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera3, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(metricReconstructedPoints, reconstructor.getActiveMetricReconstructedPoints());
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getActiveEuclideanReconstructedPoints());
            assertEquals(scale2, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrices are correct
            fundamentalMatrix1.normalize();
            estimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            assertNull(estimatedFundamentalMatrix2);

            // matrices are equal up to scale
            if (!fundamentalMatrix1.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundamentalMatrix1.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix1.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundamentalMatrix1.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final var estMetricCam1 = this.estimatedMetricCamera1.getCamera();
            final var estMetricCam2 = this.estimatedMetricCamera2.getCamera();
            final var estMetricCam3 = this.estimatedMetricCamera3.getCamera();
            assertSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);
            assertSame(this.estimatedMetricCamera3, estimatedEuclideanCamera3);

            estMetricCam1.decompose();
            estMetricCam2.decompose();
            estMetricCam3.decompose();

            assertSame(metricReconstructedPoints, euclideanReconstructedPoints);

            final var numReconstructedPoints = numPoints1 - start + numPoints2;
            if (metricReconstructedPoints.size() != numReconstructedPoints) {
                continue;
            }

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numReconstructedPoints; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of at least 2nd camera
            for (var i = 0; i < numReconstructedPoints; i++) {
                final var p = metricReconstructedPoints3D.get(i);
                assertTrue(estMetricCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estMetricCam1.getCameraCenter();
            final var estimatedCenter2 = estMetricCam2.getCameraCenter();

            // transform points and cameras to account for scale change
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);
            final var s = baseline / estimatedBaseline;
            assertEquals(1.0, this.scale, 0.0);
            assertEquals(1.0, scale2, 0.0);

            final var scaleTransformation = new MetricTransformation3D(s);

            final var scaledCamera1 = scaleTransformation.transformAndReturnNew(estMetricCam1);
            final var scaledCamera2 = scaleTransformation.transformAndReturnNew(estMetricCam2);
            final var scaledCamera3 = scaleTransformation.transformAndReturnNew(estMetricCam3);

            final var scaledReconstructionPoints3D = scaleTransformation.transformPointsAndReturnNew(
                    metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();
            scaledCamera3.decompose();

            final var scaledCenter1 = scaledCamera1.getCameraCenter();
            final var scaledCenter2 = scaledCamera2.getCameraCenter();
            final var scaledCenter3 = scaledCamera3.getCameraCenter();

            final var scaledIntrinsic1 = scaledCamera1.getIntrinsicParameters();
            final var scaledIntrinsic2 = scaledCamera2.getIntrinsicParameters();
            final var scaledIntrinsic3 = scaledCamera3.getIntrinsicParameters();

            final var scaledRotation1 = scaledCamera1.getCameraRotation();
            final var scaledRotation2 = scaledCamera2.getCameraRotation();
            final var scaledRotation3 = scaledCamera3.getCameraRotation();

            final var scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

            // check cameras are correct
            if (Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(center1.equals(scaledCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR));
            if (!center3.equals(scaledCenter3, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center3.equals(scaledCenter3, LARGE_ABSOLUTE_ERROR));

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic3.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(scaledRotation3.asInhomogeneousMatrix().equals(rotation3.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = start; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(scaledReconstructionPoints3D.get(i - start), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(scaledReconstructionPoints3D.get(i - start), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            for (var i = 0; i < numPoints2; i++) {
                if (!points3D2.get(i).equals(scaledReconstructionPoints3D.get(i + numPoints1 - start),
                        LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D2.get(i).equals(scaledReconstructionPoints3D.get(i + numPoints1 - start),
                        LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsEssentialEPnPDAQThreeViews() throws InvalidPairOfCamerasException, AlgebraException,
            CameraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getInitialCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPoint,
                    principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);
            configuration.setUseEPnPForAdditionalCamerasEstimation(true);
            configuration.setUseUPnPForAdditionalCamerasEstimation(false);
            configuration.setUseDAQForAdditionalCamerasIntrinics(true);
            configuration.setUseDIACForAdditionalCamerasIntrinsics(false);

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var alphaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation1 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            final var cameraSeparation2 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation1,
                    center1.getInhomY() + cameraSeparation1,
                    center1.getInhomZ() + cameraSeparation1);
            final var center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);
            final var rotation3 = new MatrixRotation3D(alphaEuler3, betaEuler3, gammaEuler3);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);
            final var camera3 = new PinholeCamera(intrinsic, rotation3, center3);

            final var fundamentalMatrix1 = new FundamentalMatrix(camera1, camera2);
            final var fundamentalMatrix2 = new FundamentalMatrix(camera2, camera3);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of 1st two cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final var decomposer = new SingularValueDecomposer(planesIntersectionMatrix);
            decomposer.decompose();
            final var v = decomposer.getV();
            final var centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final var numPoints1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPoints2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var start = randomizer.nextInt(0, numPoints1 - MIN_TRACKED_POINTS);

            InhomogeneousPoint3D point3D;
            final var points3D1 = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            Point2D projectedPoint3;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            final var projectedPoints3 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            for (var i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front1 || !front2);
                points3D1.add(point3D);

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);

                projectedPoint3 = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3);
                projectedPoints3.add(projectedPoint3);
            }

            final var points3D2 = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint2b;
            Point2D projectedPoint3b;
            final var projectedPoints2b = new ArrayList<Point2D>();
            final var projectedPoints3b = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front2);
                points3D2.add(point3D);

                // here 3D point is in front of both cameras

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);

                projectedPoint3b = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3b);
                projectedPoints3b.add(projectedPoint3b);
            }

            final var listener = new SparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final SparseReconstructor reconstructor) {
                    return viewCount < 3;
                }

                @Override
                public void onRequestSamples(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final List<Sample2D> currentViewNewlySpawnedSamples) {

                    previousViewTrackedSamples.clear();
                    currentViewTrackedSamples.clear();
                    currentViewNewlySpawnedSamples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    } else if (estimatedFundamentalMatrix == null) {
                        // second view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        // spawned samples
                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewNewlySpawnedSamples.add(sample);
                        }
                    } else {
                        // third view
                        for (var i = start; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = start; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints3.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints3b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final SparseReconstructor reconstructor, final List<Sample2D> allPreviousViewSamples,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final int previousViewId, final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    var numCameras = 0;
                    if (estimatedMetricCamera1 != null && (estimatedMetricCamera1.getViewId() == previousViewId
                            || estimatedMetricCamera1.getViewId() == currentViewId)) {
                        numCameras++;
                    }
                    if (estimatedMetricCamera2 != null && (estimatedMetricCamera2.getViewId() == previousViewId
                            || estimatedMetricCamera2.getViewId() == currentViewId)) {
                        numCameras++;
                    }

                    EstimatedCamera[] estimatedCameras = null;
                    if (numCameras > 0) {
                        estimatedCameras = new EstimatedCamera[numCameras];

                        var pos = 0;
                        if (estimatedMetricCamera1 != null && (estimatedMetricCamera1.getViewId() == previousViewId
                                || estimatedMetricCamera1.getViewId() == currentViewId)) {
                            estimatedCameras[pos] = estimatedMetricCamera1;
                            pos++;
                        }
                        if (estimatedMetricCamera2 != null && (estimatedMetricCamera2.getViewId() == previousViewId
                                || estimatedMetricCamera2.getViewId() == currentViewId)) {
                            estimatedCameras[pos] = estimatedMetricCamera2;
                        }
                    }

                    final var allPreviousPoints = new ArrayList<Point2D>();
                    for (final var sample : allPreviousViewSamples) {
                        allPreviousPoints.add(sample.getPoint());
                    }
                    final var tree = new KDTree2D(allPreviousPoints);

                    // search previous view tracked samples within tree
                    final var numTrackedSamples = previousViewTrackedSamples.size();
                    Point2D point;
                    Point2D nearestPoint;
                    int nearestIndex;
                    MatchedSamples match;
                    for (var i = 0; i < numTrackedSamples; i++) {
                        final var previousSample = previousViewTrackedSamples.get(i);
                        point = previousSample.getPoint();
                        nearestIndex = tree.nearestIndex(point);
                        nearestPoint = allPreviousPoints.get(nearestIndex);
                        final var nearestSample = allPreviousViewSamples.get(nearestIndex);

                        if (point.distanceTo(nearestPoint) > NEAREST_THRESHOLD) {
                            continue;
                        }

                        final var currentSample = currentViewTrackedSamples.get(i);

                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                previousSample, currentSample
                        });
                        match.setViewIds(new int[]{previousViewId, currentViewId});

                        match.setReconstructedPoint(nearestSample.getReconstructedPoint());

                        if (estimatedCameras != null) {
                            match.setCameras(estimatedCameras);
                        }

                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final SparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    if (SparseReconstructorTest.this.estimatedFundamentalMatrix == null) {
                        SparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                    } else if (estimatedFundamentalMatrix2 == null) {
                        estimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                    }
                }

                @Override
                public void onMetricCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    if (estimatedMetricCamera2 == null) {
                        estimatedMetricCamera1 = previousCamera;
                        estimatedMetricCamera2 = currentCamera;
                    } else if (estimatedMetricCamera3 == null) {
                        estimatedMetricCamera2 = previousCamera;
                        estimatedMetricCamera3 = currentCamera;
                    }
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final double scale, final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    if (estimatedEuclideanCamera2 == null) {
                        estimatedEuclideanCamera1 = previousCamera;
                        estimatedEuclideanCamera2 = currentCamera;
                        SparseReconstructorTest.this.scale = scale;
                    } else if (estimatedEuclideanCamera3 == null) {
                        estimatedEuclideanCamera2 = previousCamera;
                        estimatedEuclideanCamera3 = currentCamera;
                        scale2 = scale;
                    }
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    if (euclideanReconstructedPoints == null) {
                        SparseReconstructorTest.this.scale = scale;
                    } else {
                        scale2 = scale;
                    }

                    euclideanReconstructedPoints = points;
                }

                @Override
                public void onStart(final SparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final SparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final SparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final SparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(started);
            assertTrue(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(estimatedMetricCamera3, reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(estimatedMetricCamera2, reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera3, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(metricReconstructedPoints, reconstructor.getActiveMetricReconstructedPoints());
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getActiveEuclideanReconstructedPoints());
            assertEquals(scale2, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrices are correct
            fundamentalMatrix1.normalize();
            estimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            fundamentalMatrix2.normalize();
            estimatedFundamentalMatrix2.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix1.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundamentalMatrix1.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix1.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundamentalMatrix1.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR));
            if (!fundamentalMatrix2.getInternalMatrix().equals(
                    estimatedFundamentalMatrix2.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundamentalMatrix2.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix2.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix2.getInternalMatrix().equals(
                    estimatedFundamentalMatrix2.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundamentalMatrix2.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix2.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR));


            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final var estMetricCam1 = this.estimatedMetricCamera1.getCamera();
            final var estMetricCam2 = this.estimatedMetricCamera2.getCamera();
            final var estMetricCam3 = this.estimatedMetricCamera3.getCamera();
            assertSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);
            assertSame(this.estimatedMetricCamera3, estimatedEuclideanCamera3);

            estMetricCam1.decompose();
            estMetricCam2.decompose();
            estMetricCam3.decompose();

            assertSame(metricReconstructedPoints, euclideanReconstructedPoints);

            final var numReconstructedPoints = numPoints1 - start + numPoints2;

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numReconstructedPoints; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of at least 1st two cameras
            for (var i = 0; i < numReconstructedPoints; i++) {
                final var p = metricReconstructedPoints3D.get(i);
                assertTrue(estMetricCam1.isPointInFrontOfCamera(p));
                assertTrue(estMetricCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estMetricCam1.getCameraCenter();
            final var estimatedCenter2 = estMetricCam2.getCameraCenter();

            // transform points and cameras to account for scale change
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);
            final var s = baseline / estimatedBaseline;
            assertEquals(1.0, this.scale, 0.0);
            assertEquals(1.0, scale2, 0.0);

            final var scaleTransformation = new MetricTransformation3D(s);

            final var scaledCamera1 = scaleTransformation.transformAndReturnNew(estMetricCam1);
            final var scaledCamera2 = scaleTransformation.transformAndReturnNew(estMetricCam2);
            final var scaledCamera3 = scaleTransformation.transformAndReturnNew(estMetricCam3);

            final var scaledReconstructionPoints3D = scaleTransformation.transformPointsAndReturnNew(
                    metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();
            scaledCamera3.decompose();

            final var scaledCenter1 = scaledCamera1.getCameraCenter();
            final var scaledCenter2 = scaledCamera2.getCameraCenter();
            final var scaledCenter3 = scaledCamera3.getCameraCenter();

            final var scaledIntrinsic1 = scaledCamera1.getIntrinsicParameters();
            final var scaledIntrinsic2 = scaledCamera2.getIntrinsicParameters();
            final var scaledIntrinsic3 = scaledCamera3.getIntrinsicParameters();

            final var scaledRotation1 = scaledCamera1.getCameraRotation();
            final var scaledRotation2 = scaledCamera2.getCameraRotation();
            final var scaledRotation3 = scaledCamera3.getCameraRotation();

            final var scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

            // check cameras are correct
            if (Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(center1.equals(scaledCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR));
            if (!center3.equals(scaledCenter3, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center3.equals(scaledCenter3, LARGE_ABSOLUTE_ERROR));

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            if (Math.abs(scaledIntrinsic3.getHorizontalFocalLength()
                    - intrinsic.getHorizontalFocalLength()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(scaledIntrinsic3.getVerticalFocalLength()
                    - intrinsic.getVerticalFocalLength()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(scaledRotation3.asInhomogeneousMatrix().equals(rotation3.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = start; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(scaledReconstructionPoints3D.get(i - start), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(scaledReconstructionPoints3D.get(i - start), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            for (int i = 0; i < numPoints2; i++) {
                if (!points3D2.get(i).equals(scaledReconstructionPoints3D.get(i + numPoints1 - start),
                        LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D2.get(i).equals(scaledReconstructionPoints3D.get(i + numPoints1 - start),
                        LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsEssentialEPnPDIACThreeViews() throws InvalidPairOfCamerasException, AlgebraException,
            CameraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            reset();

            final var configuration = new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_DIAC, MAX_FOCAL_LENGTH_DIAC);
            final var aspectRatio = configuration.getInitialCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);
            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);
            configuration.setUseEPnPForAdditionalCamerasEstimation(true);
            configuration.setUseUPnPForAdditionalCamerasEstimation(false);
            configuration.setUseDAQForAdditionalCamerasIntrinics(false);
            configuration.setUseDIACForAdditionalCamerasIntrinsics(true);

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var alphaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation1 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_DIAC, MAX_CAMERA_SEPARATION_DIAC);
            final var cameraSeparation2 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_DIAC, MAX_CAMERA_SEPARATION_DIAC);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation1,
                    center1.getInhomY() + cameraSeparation1,
                    center1.getInhomZ() + cameraSeparation1);
            final var center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);
            final var rotation3 = new MatrixRotation3D(alphaEuler3, betaEuler3, gammaEuler3);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);
            final var camera3 = new PinholeCamera(intrinsic, rotation3, center3);

            final var fundamentalMatrix1 = new FundamentalMatrix(camera1, camera2);
            final var fundamentalMatrix2 = new FundamentalMatrix(camera2, camera3);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of 1st two cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final var decomposer = new SingularValueDecomposer(planesIntersectionMatrix);
            decomposer.decompose();
            final var v = decomposer.getV();
            final var centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final var numPoints1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPoints2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var start = randomizer.nextInt(0, numPoints1 - MIN_TRACKED_POINTS);

            InhomogeneousPoint3D point3D;
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            Point2D projectedPoint3;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            final var projectedPoints3 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            var maxTriesReached = false;
            for (var i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        maxTriesReached = true;
                        break;
                    }
                    numTry++;
                } while (!front1 || !front2);

                if (maxTriesReached) {
                    break;
                }

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);

                projectedPoint3 = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3);
                projectedPoints3.add(projectedPoint3);
            }

            if (maxTriesReached) {
                continue;
            }

            Point2D projectedPoint2b;
            Point2D projectedPoint3b;
            final var projectedPoints2b = new ArrayList<Point2D>();
            final var projectedPoints3b = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        maxTriesReached = true;
                        break;
                    }
                    numTry++;
                } while (!front2);

                if (maxTriesReached) {
                    break;
                }

                // here 3D point is in front of both cameras

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);

                projectedPoint3b = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3b);
                projectedPoints3b.add(projectedPoint3b);
            }

            if (maxTriesReached) {
                continue;
            }


            final var listener = new SparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final SparseReconstructor reconstructor) {
                    return viewCount < 3;
                }

                @Override
                public void onRequestSamples(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final List<Sample2D> currentViewNewlySpawnedSamples) {

                    previousViewTrackedSamples.clear();
                    currentViewTrackedSamples.clear();
                    currentViewNewlySpawnedSamples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    } else if (estimatedFundamentalMatrix == null) {
                        // second view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        // spawned samples
                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewNewlySpawnedSamples.add(sample);
                        }
                    } else {
                        // third view
                        for (var i = start; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = start; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints3.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints3b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final SparseReconstructor reconstructor, final List<Sample2D> allPreviousViewSamples,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final int previousViewId, final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    var numCameras = 0;
                    if (estimatedMetricCamera1 != null && (estimatedMetricCamera1.getViewId() == previousViewId
                            || estimatedMetricCamera1.getViewId() == currentViewId)) {
                        numCameras++;
                    }
                    if (estimatedMetricCamera2 != null && (estimatedMetricCamera2.getViewId() == previousViewId
                            || estimatedMetricCamera2.getViewId() == currentViewId)) {
                        numCameras++;
                    }

                    EstimatedCamera[] estimatedCameras = null;
                    if (numCameras > 0) {
                        estimatedCameras = new EstimatedCamera[numCameras];

                        var pos = 0;
                        if (estimatedMetricCamera1 != null && (estimatedMetricCamera1.getViewId() == previousViewId
                                || estimatedMetricCamera1.getViewId() == currentViewId)) {
                            estimatedCameras[pos] = estimatedMetricCamera1;
                            pos++;
                        }
                        if (estimatedMetricCamera2 != null && (estimatedMetricCamera2.getViewId() == previousViewId
                                || estimatedMetricCamera2.getViewId() == currentViewId)) {
                            estimatedCameras[pos] = estimatedMetricCamera2;
                        }
                    }

                    final var allPreviousPoints = new ArrayList<Point2D>();
                    for (final var sample : allPreviousViewSamples) {
                        allPreviousPoints.add(sample.getPoint());
                    }
                    final var tree = new KDTree2D(allPreviousPoints);

                    // search previous view tracked samples within tree
                    var numTrackedSamples = previousViewTrackedSamples.size();
                    Point2D point;
                    Point2D nearestPoint;
                    int nearestIndex;
                    MatchedSamples match;
                    for (var i = 0; i < numTrackedSamples; i++) {
                        final var previousSample = previousViewTrackedSamples.get(i);
                        point = previousSample.getPoint();
                        nearestIndex = tree.nearestIndex(point);
                        nearestPoint = allPreviousPoints.get(nearestIndex);
                        final var nearestSample = allPreviousViewSamples.get(nearestIndex);

                        if (point.distanceTo(nearestPoint) > NEAREST_THRESHOLD) {
                            continue;
                        }

                        final var currentSample = currentViewTrackedSamples.get(i);

                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                previousSample, currentSample
                        });
                        match.setViewIds(new int[]{previousViewId, currentViewId});

                        match.setReconstructedPoint(nearestSample.getReconstructedPoint());

                        if (estimatedCameras != null) {
                            match.setCameras(estimatedCameras);
                        }

                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final SparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    if (SparseReconstructorTest.this.estimatedFundamentalMatrix == null) {
                        SparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                    } else if (estimatedFundamentalMatrix2 == null) {
                        estimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                    }
                }

                @Override
                public void onMetricCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    if (estimatedMetricCamera2 == null) {
                        estimatedMetricCamera1 = previousCamera;
                        estimatedMetricCamera2 = currentCamera;
                    } else if (estimatedMetricCamera3 == null) {
                        estimatedMetricCamera2 = previousCamera;
                        estimatedMetricCamera3 = currentCamera;
                    }
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final double scale, final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    if (estimatedEuclideanCamera2 == null) {
                        estimatedEuclideanCamera1 = previousCamera;
                        estimatedEuclideanCamera2 = currentCamera;
                        SparseReconstructorTest.this.scale = scale;
                    } else if (estimatedEuclideanCamera3 == null) {
                        estimatedEuclideanCamera2 = previousCamera;
                        estimatedEuclideanCamera3 = currentCamera;
                        scale2 = scale;
                    }
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    if (euclideanReconstructedPoints == null) {
                        SparseReconstructorTest.this.scale = scale;
                    } else {
                        scale2 = scale;
                    }

                    euclideanReconstructedPoints = points;
                }

                @Override
                public void onStart(final SparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final SparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final SparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final SparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(started);
            assertTrue(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            if (estimatedEuclideanCamera3 == null) {
                continue;
            }
            assertSame(estimatedMetricCamera3, reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(estimatedMetricCamera2, reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera3, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(metricReconstructedPoints, reconstructor.getActiveMetricReconstructedPoints());
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getActiveEuclideanReconstructedPoints());
            assertEquals(scale2, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrices are correct
            fundamentalMatrix1.normalize();
            estimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            fundamentalMatrix2.normalize();
            estimatedFundamentalMatrix2.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix1.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundamentalMatrix1.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix1.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundamentalMatrix1.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR));
            if (!fundamentalMatrix2.getInternalMatrix().equals(
                    estimatedFundamentalMatrix2.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundamentalMatrix2.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix2.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix2.getInternalMatrix().equals(
                    estimatedFundamentalMatrix2.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundamentalMatrix2.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix2.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR));


            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final var estMetricCam1 = this.estimatedMetricCamera1.getCamera();
            final var estMetricCam2 = this.estimatedMetricCamera2.getCamera();
            if (estimatedMetricCamera3 == null) {
                continue;
            }
            final PinholeCamera estMetricCam3 = this.estimatedMetricCamera3.getCamera();
            assertSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);
            assertSame(this.estimatedMetricCamera3, estimatedEuclideanCamera3);

            estMetricCam1.decompose();
            estMetricCam2.decompose();
            estMetricCam3.decompose();

            assertSame(metricReconstructedPoints, euclideanReconstructedPoints);

            final var numReconstructedPoints = numPoints1 - start + numPoints2;

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numReconstructedPoints; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of at least 1st two cameras
            var failedIter = false;
            for (var i = 0; i < numReconstructedPoints; i++) {
                final var p = metricReconstructedPoints3D.get(i);
                if (!estMetricCam1.isPointInFrontOfCamera(p)
                        || !estMetricCam2.isPointInFrontOfCamera(p)) {
                    failedIter = true;
                    break;
                }
                assertTrue(estMetricCam1.isPointInFrontOfCamera(p));
                assertTrue(estMetricCam2.isPointInFrontOfCamera(p));
            }

            if (failedIter) {
                continue;
            }

            final var estimatedCenter1 = estMetricCam1.getCameraCenter();
            final var estimatedCenter2 = estMetricCam2.getCameraCenter();

            // transform points and cameras to account for scale change
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);
            final var s = baseline / estimatedBaseline;
            assertEquals(1.0, this.scale, 0.0);
            assertEquals(1.0, scale2, 0.0);

            final var scaleTransformation = new MetricTransformation3D(s);

            final var scaledCamera1 = scaleTransformation.transformAndReturnNew(estMetricCam1);
            final var scaledCamera2 = scaleTransformation.transformAndReturnNew(estMetricCam2);
            final var scaledCamera3 = scaleTransformation.transformAndReturnNew(estMetricCam3);

            scaledCamera1.decompose();
            scaledCamera2.decompose();
            scaledCamera3.decompose();

            final var scaledCenter1 = scaledCamera1.getCameraCenter();
            final var scaledCenter2 = scaledCamera2.getCameraCenter();

            final var scaledIntrinsic1 = scaledCamera1.getIntrinsicParameters();
            final var scaledIntrinsic2 = scaledCamera2.getIntrinsicParameters();

            final var scaledRotation1 = scaledCamera1.getCameraRotation();
            final var scaledRotation2 = scaledCamera2.getCameraRotation();

            final var scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

            // check cameras are correct
            if (Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(center1.equals(scaledCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR));

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsEssentialUPnPThreeViews() throws InvalidPairOfCamerasException, AlgebraException,
            CameraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getInitialCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPoint,
                    principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);
            configuration.setUseEPnPForAdditionalCamerasEstimation(false);
            configuration.setUseUPnPForAdditionalCamerasEstimation(true);
            configuration.setUseDAQForAdditionalCamerasIntrinics(false);
            configuration.setUseDIACForAdditionalCamerasIntrinsics(false);

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var alphaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation1 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            final var cameraSeparation2 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation1,
                    center1.getInhomY() + cameraSeparation1,
                    center1.getInhomZ() + cameraSeparation1);
            final var center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);
            final var rotation3 = new MatrixRotation3D(alphaEuler3, betaEuler3, gammaEuler3);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);
            final var camera3 = new PinholeCamera(intrinsic, rotation3, center3);

            final var fundamentalMatrix1 = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of 1st two cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final var decomposer = new SingularValueDecomposer(planesIntersectionMatrix);
            decomposer.decompose();
            final var v = decomposer.getV();
            final var centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final var numPoints1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPoints2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var start = randomizer.nextInt(0, numPoints1 - MIN_TRACKED_POINTS);

            InhomogeneousPoint3D point3D;
            final var points3D1 = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            Point2D projectedPoint3;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            final var projectedPoints3 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            for (var i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front1 || !front2);
                points3D1.add(point3D);

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);

                projectedPoint3 = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3);
                projectedPoints3.add(projectedPoint3);
            }

            final var points3D2 = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint2b;
            Point2D projectedPoint3b;
            final var projectedPoints2b = new ArrayList<Point2D>();
            final var projectedPoints3b = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front2);
                points3D2.add(point3D);

                // here 3D point is in front of both cameras

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);

                projectedPoint3b = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3b);
                projectedPoints3b.add(projectedPoint3b);
            }

            final var listener = new SparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final SparseReconstructor reconstructor) {
                    return viewCount < 3;
                }

                @Override
                public void onRequestSamples(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final List<Sample2D> currentViewNewlySpawnedSamples) {

                    previousViewTrackedSamples.clear();
                    currentViewTrackedSamples.clear();
                    currentViewNewlySpawnedSamples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    } else if (estimatedFundamentalMatrix == null) {
                        // second view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        // spawned samples
                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewNewlySpawnedSamples.add(sample);
                        }
                    } else {
                        // third view
                        for (var i = start; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = start; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints3.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints3b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final SparseReconstructor reconstructor, final List<Sample2D> allPreviousViewSamples,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final int previousViewId, final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    var numCameras = 0;
                    if (estimatedMetricCamera1 != null && (estimatedMetricCamera1.getViewId() == previousViewId
                            || estimatedMetricCamera1.getViewId() == currentViewId)) {
                        numCameras++;
                    }
                    if (estimatedMetricCamera2 != null && (estimatedMetricCamera2.getViewId() == previousViewId
                            || estimatedMetricCamera2.getViewId() == currentViewId)) {
                        numCameras++;
                    }

                    EstimatedCamera[] estimatedCameras = null;
                    if (numCameras > 0) {
                        estimatedCameras = new EstimatedCamera[numCameras];

                        var pos = 0;
                        if (estimatedMetricCamera1 != null && (estimatedMetricCamera1.getViewId() == previousViewId
                                || estimatedMetricCamera1.getViewId() == currentViewId)) {
                            estimatedCameras[pos] = estimatedMetricCamera1;
                            pos++;
                        }
                        if (estimatedMetricCamera2 != null && (estimatedMetricCamera2.getViewId() == previousViewId
                                || estimatedMetricCamera2.getViewId() == currentViewId)) {
                            estimatedCameras[pos] = estimatedMetricCamera2;
                        }
                    }

                    final var allPreviousPoints = new ArrayList<Point2D>();
                    for (final var sample : allPreviousViewSamples) {
                        allPreviousPoints.add(sample.getPoint());
                    }
                    final var tree = new KDTree2D(allPreviousPoints);

                    // search previous view tracked samples within tree
                    final var numTrackedSamples = previousViewTrackedSamples.size();
                    Point2D point;
                    Point2D nearestPoint;
                    int nearestIndex;
                    MatchedSamples match;
                    for (var i = 0; i < numTrackedSamples; i++) {
                        final var previousSample = previousViewTrackedSamples.get(i);
                        point = previousSample.getPoint();
                        nearestIndex = tree.nearestIndex(point);
                        nearestPoint = allPreviousPoints.get(nearestIndex);
                        final var nearestSample = allPreviousViewSamples.get(nearestIndex);

                        if (point.distanceTo(nearestPoint) > NEAREST_THRESHOLD) {
                            continue;
                        }

                        final var currentSample = currentViewTrackedSamples.get(i);

                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                previousSample, currentSample
                        });
                        match.setViewIds(new int[]{previousViewId, currentViewId});

                        match.setReconstructedPoint(nearestSample.getReconstructedPoint());

                        if (estimatedCameras != null) {
                            match.setCameras(estimatedCameras);
                        }

                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final SparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    if (SparseReconstructorTest.this.estimatedFundamentalMatrix == null) {
                        SparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                    } else if (estimatedFundamentalMatrix2 == null) {
                        estimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                    }
                }

                @Override
                public void onMetricCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    if (estimatedMetricCamera2 == null) {
                        estimatedMetricCamera1 = previousCamera;
                        estimatedMetricCamera2 = currentCamera;
                    } else if (estimatedMetricCamera3 == null) {
                        estimatedMetricCamera2 = previousCamera;
                        estimatedMetricCamera3 = currentCamera;
                    }
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final double scale, final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    if (estimatedEuclideanCamera2 == null) {
                        estimatedEuclideanCamera1 = previousCamera;
                        estimatedEuclideanCamera2 = currentCamera;
                        SparseReconstructorTest.this.scale = scale;
                    } else if (estimatedEuclideanCamera3 == null) {
                        estimatedEuclideanCamera2 = previousCamera;
                        estimatedEuclideanCamera3 = currentCamera;
                        scale2 = scale;
                    }
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    if (euclideanReconstructedPoints == null) {
                        SparseReconstructorTest.this.scale = scale;
                    } else {
                        scale2 = scale;
                    }

                    euclideanReconstructedPoints = points;
                }

                @Override
                public void onStart(final SparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final SparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final SparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final SparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(started);
            assertTrue(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(estimatedMetricCamera3, reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(estimatedMetricCamera2, reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera3, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(metricReconstructedPoints, reconstructor.getActiveMetricReconstructedPoints());
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getActiveEuclideanReconstructedPoints());
            assertEquals(scale2, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrices are correct
            fundamentalMatrix1.normalize();
            estimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            assertNull(estimatedFundamentalMatrix2);

            // matrices are equal up to scale
            if (!fundamentalMatrix1.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix1.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                            estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix1.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix1.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                            estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final var estMetricCam1 = this.estimatedMetricCamera1.getCamera();
            final var estMetricCam2 = this.estimatedMetricCamera2.getCamera();
            final var estMetricCam3 = this.estimatedMetricCamera3.getCamera();
            assertSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);
            assertSame(this.estimatedMetricCamera3, estimatedEuclideanCamera3);

            estMetricCam1.decompose();
            estMetricCam2.decompose();
            estMetricCam3.decompose();

            assertSame(metricReconstructedPoints, euclideanReconstructedPoints);

            final var numReconstructedPoints = numPoints1 - start + numPoints2;

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numReconstructedPoints; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of at least 1st two cameras
            for (var i = 0; i < numReconstructedPoints; i++) {
                final var p = metricReconstructedPoints3D.get(i);
                assertTrue(estMetricCam1.isPointInFrontOfCamera(p));
                assertTrue(estMetricCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estMetricCam1.getCameraCenter();
            final var estimatedCenter2 = estMetricCam2.getCameraCenter();

            // transform points and cameras to account for scale change
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);
            final var s = baseline / estimatedBaseline;
            assertEquals(1.0, this.scale, 0.0);

            final var scaleTransformation = new MetricTransformation3D(s);

            final var scaledCamera1 = scaleTransformation.transformAndReturnNew(estMetricCam1);
            final var scaledCamera2 = scaleTransformation.transformAndReturnNew(estMetricCam2);
            final var scaledCamera3 = scaleTransformation.transformAndReturnNew(estMetricCam3);

            final var scaledReconstructionPoints3D = scaleTransformation.transformPointsAndReturnNew(
                    metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();
            scaledCamera3.decompose();

            final var scaledCenter1 = scaledCamera1.getCameraCenter();
            final var scaledCenter2 = scaledCamera2.getCameraCenter();
            final var scaledCenter3 = scaledCamera3.getCameraCenter();

            final var scaledIntrinsic1 = scaledCamera1.getIntrinsicParameters();
            final var scaledIntrinsic2 = scaledCamera2.getIntrinsicParameters();
            final var scaledIntrinsic3 = scaledCamera3.getIntrinsicParameters();

            final var scaledRotation1 = scaledCamera1.getCameraRotation();
            final var scaledRotation2 = scaledCamera2.getCameraRotation();
            final var scaledRotation3 = scaledCamera3.getCameraRotation();

            final var scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

            // check cameras are correct
            if (Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(center1.equals(scaledCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR));
            if (!center3.equals(scaledCenter3, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center3.equals(scaledCenter3, LARGE_ABSOLUTE_ERROR));

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            if (Math.abs(scaledIntrinsic3.getHorizontalFocalLength()
                    - intrinsic.getHorizontalFocalLength()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(scaledIntrinsic3.getVerticalFocalLength()
                    - intrinsic.getVerticalFocalLength()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(scaledRotation3.asInhomogeneousMatrix().equals(rotation3.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = start; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(scaledReconstructionPoints3D.get(i - start), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(scaledReconstructionPoints3D.get(i - start), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            for (var i = 0; i < numPoints2; i++) {
                if (!points3D2.get(i).equals(scaledReconstructionPoints3D.get(i + numPoints1 - start),
                        LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D2.get(i).equals(scaledReconstructionPoints3D.get(i + numPoints1 - start),
                        LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsEssentialDLTThreeViews() throws InvalidPairOfCamerasException, AlgebraException,
            CameraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getInitialCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPoint,
                    principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);
            configuration.setUseEPnPForAdditionalCamerasEstimation(false);
            configuration.setUseUPnPForAdditionalCamerasEstimation(false);
            configuration.setUseDAQForAdditionalCamerasIntrinics(false);
            configuration.setUseDIACForAdditionalCamerasIntrinsics(false);

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var alphaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation1 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            final var cameraSeparation2 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation1,
                    center1.getInhomY() + cameraSeparation1,
                    center1.getInhomZ() + cameraSeparation1);
            final var center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);
            final var rotation3 = new MatrixRotation3D(alphaEuler3, betaEuler3, gammaEuler3);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);
            final var camera3 = new PinholeCamera(intrinsic, rotation3, center3);

            final var fundamentalMatrix1 = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of 1st two cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3, horizontalPlane2.getD());

            final var decomposer = new SingularValueDecomposer(planesIntersectionMatrix);
            decomposer.decompose();
            final var v = decomposer.getV();
            final var centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final var numPoints1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPoints2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var start = randomizer.nextInt(0, numPoints1 - MIN_TRACKED_POINTS);

            InhomogeneousPoint3D point3D;
            final var points3D1 = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            Point2D projectedPoint3;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            final var projectedPoints3 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            for (var i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front1 || !front2);
                points3D1.add(point3D);

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);

                projectedPoint3 = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3);
                projectedPoints3.add(projectedPoint3);
            }

            final var points3D2 = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint2b;
            Point2D projectedPoint3b;
            final var projectedPoints2b = new ArrayList<Point2D>();
            final var projectedPoints3b = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY,
                            centralCommonPoint.getInhomZ() + lambdaZ);

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front2);
                points3D2.add(point3D);

                // here 3D point is in front of both cameras

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);

                projectedPoint3b = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3b);
                projectedPoints3b.add(projectedPoint3b);
            }

            final var listener = new SparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final SparseReconstructor reconstructor) {
                    return viewCount < 3;
                }

                @Override
                public void onRequestSamples(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final List<Sample2D> currentViewNewlySpawnedSamples) {

                    previousViewTrackedSamples.clear();
                    currentViewTrackedSamples.clear();
                    currentViewNewlySpawnedSamples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    } else if (estimatedFundamentalMatrix == null) {
                        // second view
                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        // spawned samples
                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewNewlySpawnedSamples.add(sample);
                        }
                    } else {
                        // third view
                        for (var i = start; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2b.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = start; i < numPoints1; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints3.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints3b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final SparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final SparseReconstructor reconstructor, final List<Sample2D> allPreviousViewSamples,
                        final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples,
                        final int previousViewId, final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    var numCameras = 0;
                    if (estimatedMetricCamera1 != null && (estimatedMetricCamera1.getViewId() == previousViewId
                            || estimatedMetricCamera1.getViewId() == currentViewId)) {
                        numCameras++;
                    }
                    if (estimatedMetricCamera2 != null && (estimatedMetricCamera2.getViewId() == previousViewId
                            || estimatedMetricCamera2.getViewId() == currentViewId)) {
                        numCameras++;
                    }

                    EstimatedCamera[] estimatedCameras = null;
                    if (numCameras > 0) {
                        estimatedCameras = new EstimatedCamera[numCameras];

                        var pos = 0;
                        if (estimatedMetricCamera1 != null && (estimatedMetricCamera1.getViewId() == previousViewId
                                || estimatedMetricCamera1.getViewId() == currentViewId)) {
                            estimatedCameras[pos] = estimatedMetricCamera1;
                            pos++;
                        }
                        if (estimatedMetricCamera2 != null && (estimatedMetricCamera2.getViewId() == previousViewId
                                || estimatedMetricCamera2.getViewId() == currentViewId)) {
                            estimatedCameras[pos] = estimatedMetricCamera2;
                        }
                    }

                    final var allPreviousPoints = new ArrayList<Point2D>();
                    for (final var sample : allPreviousViewSamples) {
                        allPreviousPoints.add(sample.getPoint());
                    }
                    final var tree = new KDTree2D(allPreviousPoints);

                    // search previous view tracked samples within tree
                    final var numTrackedSamples = previousViewTrackedSamples.size();
                    Point2D point;
                    Point2D nearestPoint;
                    int nearestIndex;
                    MatchedSamples match;
                    for (var i = 0; i < numTrackedSamples; i++) {
                        final var previousSample = previousViewTrackedSamples.get(i);
                        point = previousSample.getPoint();
                        nearestIndex = tree.nearestIndex(point);
                        nearestPoint = allPreviousPoints.get(nearestIndex);
                        final var nearestSample = allPreviousViewSamples.get(nearestIndex);

                        if (point.distanceTo(nearestPoint) > NEAREST_THRESHOLD) {
                            continue;
                        }

                        final var currentSample = currentViewTrackedSamples.get(i);

                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                previousSample, currentSample
                        });
                        match.setViewIds(new int[]{previousViewId, currentViewId});

                        match.setReconstructedPoint(nearestSample.getReconstructedPoint());

                        if (estimatedCameras != null) {
                            match.setCameras(estimatedCameras);
                        }

                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final SparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    if (SparseReconstructorTest.this.estimatedFundamentalMatrix == null) {
                        SparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                    } else if (estimatedFundamentalMatrix2 == null) {
                        estimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                    }
                }

                @Override
                public void onMetricCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    if (estimatedMetricCamera2 == null) {
                        estimatedMetricCamera1 = previousCamera;
                        estimatedMetricCamera2 = currentCamera;
                    } else if (estimatedMetricCamera3 == null) {
                        estimatedMetricCamera2 = previousCamera;
                        estimatedMetricCamera3 = currentCamera;
                    }
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final SparseReconstructor reconstructor, final int previousViewId, final int currentViewId,
                        final double scale, final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    if (estimatedEuclideanCamera2 == null) {
                        estimatedEuclideanCamera1 = previousCamera;
                        estimatedEuclideanCamera2 = currentCamera;
                        SparseReconstructorTest.this.scale = scale;
                    } else if (estimatedEuclideanCamera3 == null) {
                        estimatedEuclideanCamera2 = previousCamera;
                        estimatedEuclideanCamera3 = currentCamera;
                        scale2 = scale;
                    }
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final SparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    if (euclideanReconstructedPoints == null) {
                        SparseReconstructorTest.this.scale = scale;
                    } else {
                        scale2 = scale;
                    }

                    euclideanReconstructedPoints = points;
                }

                @Override
                public void onStart(final SparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final SparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final SparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final SparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(started);
            assertTrue(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(estimatedMetricCamera3, reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(estimatedMetricCamera2, reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera3, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(metricReconstructedPoints, reconstructor.getActiveMetricReconstructedPoints());
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getActiveEuclideanReconstructedPoints());
            assertEquals(scale2, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrices are correct
            fundamentalMatrix1.normalize();
            estimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            assertNull(estimatedFundamentalMatrix2);

            // matrices are equal up to scale
            if (!fundamentalMatrix1.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix1.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix1.getInternalMatrix().equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundamentalMatrix1.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final var estMetricCam1 = this.estimatedMetricCamera1.getCamera();
            final var estMetricCam2 = this.estimatedMetricCamera2.getCamera();
            final var estMetricCam3 = this.estimatedMetricCamera3.getCamera();
            assertSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);
            assertSame(this.estimatedMetricCamera3, estimatedEuclideanCamera3);

            estMetricCam1.decompose();
            estMetricCam2.decompose();
            estMetricCam3.decompose();

            assertSame(metricReconstructedPoints, euclideanReconstructedPoints);

            final var numReconstructedPoints = numPoints1 - start + numPoints2;

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numReconstructedPoints; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of at least 1st two cameras
            for (var i = 0; i < numReconstructedPoints; i++) {
                final var p = metricReconstructedPoints3D.get(i);
                assertTrue(estMetricCam1.isPointInFrontOfCamera(p));
                assertTrue(estMetricCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estMetricCam1.getCameraCenter();
            final var estimatedCenter2 = estMetricCam2.getCameraCenter();

            // transform points and cameras to account for scale change
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);
            final var s = baseline / estimatedBaseline;
            assertEquals(1.0, this.scale, 0.0);

            final var scaleTransformation = new MetricTransformation3D(s);

            final var scaledCamera1 = scaleTransformation.transformAndReturnNew(estMetricCam1);
            final var scaledCamera2 = scaleTransformation.transformAndReturnNew(estMetricCam2);
            final var scaledCamera3 = scaleTransformation.transformAndReturnNew(estMetricCam3);

            final var scaledReconstructionPoints3D = scaleTransformation.transformPointsAndReturnNew(
                    metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();
            scaledCamera3.decompose();

            final var scaledCenter1 = scaledCamera1.getCameraCenter();
            final var scaledCenter2 = scaledCamera2.getCameraCenter();
            final var scaledCenter3 = scaledCamera3.getCameraCenter();

            final var scaledIntrinsic1 = scaledCamera1.getIntrinsicParameters();
            final var scaledIntrinsic2 = scaledCamera2.getIntrinsicParameters();
            final var scaledIntrinsic3 = scaledCamera3.getIntrinsicParameters();

            final var scaledRotation1 = scaledCamera1.getCameraRotation();
            final var scaledRotation2 = scaledCamera2.getCameraRotation();
            final var scaledRotation3 = scaledCamera3.getCameraRotation();

            final var scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

            // check cameras are correct
            if (Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(center1.equals(scaledCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(scaledCenter2, LARGE_ABSOLUTE_ERROR));
            if (!center3.equals(scaledCenter3, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center3.equals(scaledCenter3, LARGE_ABSOLUTE_ERROR));

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            if (Math.abs(scaledIntrinsic3.getHorizontalFocalLength()
                    - intrinsic.getHorizontalFocalLength()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(scaledIntrinsic3.getVerticalFocalLength()
                    - intrinsic.getVerticalFocalLength()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(scaledIntrinsic3.getSkewness() - intrinsic.getSkewness()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            if (Math.abs(scaledIntrinsic3.getHorizontalPrincipalPoint()
                    - intrinsic.getHorizontalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            if (Math.abs(scaledIntrinsic3.getVerticalPrincipalPoint()
                    - intrinsic.getVerticalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(scaledRotation3.asInhomogeneousMatrix().equals(rotation3.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = start; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(scaledReconstructionPoints3D.get(i - start), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(scaledReconstructionPoints3D.get(i - start), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            for (var i = 0; i < numPoints2; i++) {
                if (!points3D2.get(i).equals(scaledReconstructionPoints3D.get(i + numPoints1 - start),
                        LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D2.get(i).equals(scaledReconstructionPoints3D.get(i + numPoints1 - start),
                        LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private void reset() {
        viewCount = 0;
        estimatedFundamentalMatrix = estimatedFundamentalMatrix2 = null;
        estimatedMetricCamera1 = estimatedMetricCamera2 = estimatedMetricCamera3 = null;
        estimatedEuclideanCamera1 = estimatedEuclideanCamera2 = estimatedEuclideanCamera3 = null;
        metricReconstructedPoints = null;
        euclideanReconstructedPoints = null;
        started = finished = failed = cancelled = false;
        scale = 0.0;
    }
}
