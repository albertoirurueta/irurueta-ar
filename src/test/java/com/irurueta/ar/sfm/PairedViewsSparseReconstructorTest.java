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
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class PairedViewsSparseReconstructorTest {

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

    private int viewCount = 0;
    private EstimatedFundamentalMatrix estimatedFundamentalMatrix;
    private EstimatedFundamentalMatrix estimatedFundamentalMatrix2;
    private EstimatedCamera estimatedEuclideanCamera1;
    private EstimatedCamera estimatedEuclideanCamera2;
    private EstimatedCamera estimatedEuclideanCamera2b;
    private EstimatedCamera estimatedEuclideanCamera3;
    private List<ReconstructedPoint3D> euclideanReconstructedPoints;
    private List<ReconstructedPoint3D> euclideanReconstructedPoints2;

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
        estimatedEuclideanCamera1 = estimatedEuclideanCamera2 = estimatedEuclideanCamera3 = null;
        euclideanReconstructedPoints = null;
        started = finished = failed = cancelled = false;
    }

    @Test
    void testConstructor() {
        assertEquals(2, PairedViewsSparseReconstructor.MIN_NUMBER_OF_VIEWS);

        final var configuration = new PairedViewsSparseReconstructorConfiguration();
        final var listener = new PairedViewsSparseReconstructorListener() {
            @Override
            public double onBaselineRequested(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                    final EstimatedCamera metricCamera1, final EstimatedCamera metricCamera2) {
                return 1.0;
            }

            @Override
            public boolean hasMoreViewsAvailable(final PairedViewsSparseReconstructor reconstructor) {
                return false;
            }

            @Override
            public void onRequestSamplesForCurrentViewPair(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                    final List<Sample2D> samples1, final List<Sample2D> samples2) {
                // no action needed
            }

            @Override
            public void onSamplesAccepted(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                    final List<Sample2D> samples1, final List<Sample2D> samples2) {
                // no action needed
            }

            @Override
            public void onSamplesRejected(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                    final List<Sample2D> samples1, final List<Sample2D> samples2) {
                // no action needed
            }

            @Override
            public void onRequestMatches(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                    final List<Sample2D> samples1, final List<Sample2D> samples2, final List<MatchedSamples> matches) {
                // no action needed
            }

            @Override
            public void onFundamentalMatrixEstimated(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                    final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                // no action needed
            }

            @Override
            public void onEuclideanCameraPairEstimated(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                    final double scale, final EstimatedCamera camera1, final EstimatedCamera camera2) {
                // no action needed
            }

            @Override
            public void onEuclideanReconstructedPointsEstimated(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                    final double scale, final List<ReconstructedPoint3D> points) {
                // no action needed
            }

            @Override
            public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                return null;
            }

            @Override
            public void onStart(final PairedViewsSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onFinish(final PairedViewsSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                // no action needed
            }
        };

        // constructor with listener
        var reconstructor = new PairedViewsSparseReconstructor(listener);

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
        assertNull(reconstructor.getMetricReconstructedPoints());
        assertNull(reconstructor.getEuclideanReconstructedPoints());
        assertEquals(BaseSparseReconstructor.DEFAULT_SCALE, reconstructor.getCurrentScale(), 0.0);
        assertNull(reconstructor.getPreviousViewSamples());
        assertNull(reconstructor.getCurrentViewSamples());
        assertTrue(reconstructor.isFirstViewPair());
        assertFalse(reconstructor.isAdditionalViewPair());

        // constructor with configuration and listener
        reconstructor = new PairedViewsSparseReconstructor(configuration, listener);

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
        assertNull(reconstructor.getMetricReconstructedPoints());
        assertNull(reconstructor.getEuclideanReconstructedPoints());
        assertEquals(BaseSparseReconstructor.DEFAULT_SCALE, reconstructor.getCurrentScale(), 0.0);
        assertNull(reconstructor.getPreviousViewSamples());
        assertNull(reconstructor.getCurrentViewSamples());
        assertTrue(reconstructor.isFirstViewPair());
        assertFalse(reconstructor.isAdditionalViewPair());
    }

    @Test
    void testGeneralPointsEssentialTwoViews() throws InvalidPairOfCamerasException, AlgebraException, CameraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getPairedCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPoint,
                    principalPoint, skewness);
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
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis planes of
            // both cameras
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

            final var numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final var points3D = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            for (var i = 0; i < numPoints; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY, centralCommonPoint.getInhomZ() + lambdaZ);

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front1 || !front2);
                points3D.add(point3D);

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            final var listener = new PairedViewsSparseReconstructorListener() {

                @Override
                public double onBaselineRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final EstimatedCamera metricCamera1, final EstimatedCamera metricCamera2) {
                    return center1.distanceTo(center2);
                }

                @Override
                public boolean hasMoreViewsAvailable(final PairedViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentViewPair(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {

                    samples1.clear();
                    samples2.clear();

                    Sample2D sample1;
                    Sample2D sample2;
                    for (var i = 0; i < numPoints; i++) {
                        sample1 = new Sample2D();
                        sample1.setPoint(projectedPoints1.get(i));
                        sample1.setViewId(viewId1);
                        samples1.add(sample1);

                        sample2 = new Sample2D();
                        sample2.setPoint(projectedPoints2.get(i));
                        sample2.setViewId(viewId2);
                        samples2.add(sample2);
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onSamplesRejected(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onRequestMatches(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                samples1.get(i), samples2.get(i)
                        });
                        match.setViewIds(new int[]{viewId1, viewId2});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    PairedViewsSparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onEuclideanCameraPairEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final EstimatedCamera camera1, final EstimatedCamera camera2) {
                    estimatedEuclideanCamera1 = camera1;
                    estimatedEuclideanCamera2 = camera2;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                    return intrinsic;
                }

                @Override
                public void onStart(final PairedViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final PairedViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new PairedViewsSparseReconstructor(configuration, listener);

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
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.isAdditionalViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final var estEuclideanCam1 = this.estimatedEuclideanCamera1.getCamera();
            final var estEuclideanCam2 = this.estimatedEuclideanCamera2.getCamera();

            estEuclideanCam1.decompose();
            estEuclideanCam2.decompose();

            final var euclideanReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(euclideanReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints; i++) {
                final var p = euclideanReconstructedPoints3D.get(i);
                assertTrue(estEuclideanCam1.isPointInFrontOfCamera(p));
                assertTrue(estEuclideanCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estEuclideanCam1.getCameraCenter();
            final var estimatedCenter2 = estEuclideanCam2.getCameraCenter();

            // check scale
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);

            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);
            assertEquals(scale, baseline, LARGE_ABSOLUTE_ERROR);

            // check cameras
            final var estimatedIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var estimatedIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();

            final var estimatedRotation1 = estEuclideanCam1.getCameraRotation();
            final var estimatedRotation2 = estEuclideanCam2.getCameraRotation();

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR));

            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPoints; i++) {
                if (!points3D.get(i).equals(euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
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
            final var configuration = new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_DIAC, MAX_FOCAL_LENGTH_DIAC);
            final var aspectRatio = configuration.getPairedCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);
            configuration.setPairedCamerasAspectRatio(aspectRatio);

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

            // 1st find an approximate central point by intersecting the axis planes of
            // both cameras
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

            final var numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final var points3D = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            var maxTriesReached = false;
            for (var i = 0; i < numPoints; i++) {
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

                points3D.add(point3D);

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

            final var listener = new PairedViewsSparseReconstructorListener() {
                @Override
                public double onBaselineRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedCamera metricCamera1, final EstimatedCamera metricCamera2) {
                    return center1.distanceTo(center2);
                }

                @Override
                public boolean hasMoreViewsAvailable(final PairedViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentViewPair(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {

                    samples1.clear();
                    samples2.clear();

                    Sample2D sample1;
                    Sample2D sample2;
                    for (var i = 0; i < numPoints; i++) {
                        sample1 = new Sample2D();
                        sample1.setPoint(projectedPoints1.get(i));
                        sample1.setViewId(viewId1);
                        samples1.add(sample1);

                        sample2 = new Sample2D();
                        sample2.setPoint(projectedPoints2.get(i));
                        sample2.setViewId(viewId2);
                        samples2.add(sample2);
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onSamplesRejected(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onRequestMatches(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                samples1.get(i), samples2.get(i)
                        });
                        match.setViewIds(new int[]{viewId1, viewId2});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    PairedViewsSparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onEuclideanCameraPairEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final EstimatedCamera camera1, final EstimatedCamera camera2) {
                    estimatedEuclideanCamera1 = camera1;
                    estimatedEuclideanCamera2 = camera2;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                    return intrinsic;
                }

                @Override
                public void onStart(final PairedViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final PairedViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new PairedViewsSparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            if (!finished || failed) {
                continue;
            }

            // check correctness
            assertTrue(started);
            assertFalse(cancelled);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.isAdditionalViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final var estEuclideanCam1 = this.estimatedEuclideanCamera1.getCamera();
            final var estEuclideanCam2 = this.estimatedEuclideanCamera2.getCamera();

            estEuclideanCam1.decompose();
            estEuclideanCam2.decompose();

            final var euclideanReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(euclideanReconstructedPoints.get(i).getPoint());
            }

            // check that most of the points are in front of both cameras
            var valid = 0;
            var invalid = 0;
            for (var i = 0; i < numPoints; i++) {
                if (euclideanReconstructedPoints.get(i).isInlier()) {
                    final var p = euclideanReconstructedPoints3D.get(i);
                    assertTrue(estEuclideanCam1.isPointInFrontOfCamera(p));
                    assertTrue(estEuclideanCam2.isPointInFrontOfCamera(p));
                    valid++;
                } else {
                    invalid++;
                }
            }

            assertTrue(valid >= invalid);

            final var estimatedCenter1 = estEuclideanCam1.getCameraCenter();
            final var estimatedCenter2 = estEuclideanCam2.getCameraCenter();

            // check scale
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);

            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);
            assertEquals(scale, baseline, LARGE_ABSOLUTE_ERROR);

            // check cameras
            final var estimatedIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var estimatedIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();

            final var estimatedRotation1 = estEuclideanCam1.getCameraRotation();
            final var estimatedRotation2 = estEuclideanCam2.getCameraRotation();

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR));

            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPoints; i++) {
                if (!points3D.get(i).equals(
                        euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
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
            final var configuration = new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getPairedCamerasAspectRatio();
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

            // 1st find an approximate central point by intersecting the axis planes of
            // both cameras
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

            final var numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final var points3D = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean leftFront;
            boolean rightFront;
            var maxTriesReached = false;
            for (var i = 0; i < numPoints; i++) {
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

                points3D.add(point3D);

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

            final var listener = new PairedViewsSparseReconstructorListener() {

                @Override
                public double onBaselineRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedCamera metricCamera1, final EstimatedCamera metricCamera2) {
                    return center1.distanceTo(center2);
                }

                @Override
                public boolean hasMoreViewsAvailable(final PairedViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentViewPair(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {

                    samples1.clear();
                    samples2.clear();

                    Sample2D sample1;
                    Sample2D sample2;
                    for (var i = 0; i < numPoints; i++) {
                        sample1 = new Sample2D();
                        sample1.setPoint(projectedPoints1.get(i));
                        sample1.setViewId(viewId1);
                        samples1.add(sample1);

                        sample2 = new Sample2D();
                        sample2.setPoint(projectedPoints2.get(i));
                        sample2.setViewId(viewId2);
                        samples2.add(sample2);
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onSamplesRejected(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onRequestMatches(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                samples1.get(i), samples2.get(i)
                        });
                        match.setViewIds(new int[]{viewId1, viewId2});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    PairedViewsSparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onEuclideanCameraPairEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final EstimatedCamera camera1, final EstimatedCamera camera2) {
                    estimatedEuclideanCamera1 = camera1;
                    estimatedEuclideanCamera2 = camera2;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                    return intrinsic;
                }

                @Override
                public void onStart(final PairedViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final PairedViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new PairedViewsSparseReconstructor(configuration, listener);

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
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.isAdditionalViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final var estEuclideanCam1 = this.estimatedEuclideanCamera1.getCamera();
            final var estEuclideanCam2 = this.estimatedEuclideanCamera2.getCamera();

            estEuclideanCam1.decompose();
            estEuclideanCam2.decompose();

            final var euclideanReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(euclideanReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints; i++) {
                final var p = euclideanReconstructedPoints3D.get(i);
                assertTrue(estEuclideanCam1.isPointInFrontOfCamera(p));
                assertTrue(estEuclideanCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estEuclideanCam1.getCameraCenter();
            final var estimatedCenter2 = estEuclideanCam2.getCameraCenter();

            // check scale
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);

            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);
            assertEquals(scale, baseline, LARGE_ABSOLUTE_ERROR);

            // check cameras
            final var estimatedIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var estimatedIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();

            final var estimatedRotation1 = estEuclideanCam1.getCameraRotation();
            final var estimatedRotation2 = estEuclideanCam2.getCameraRotation();

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR));

            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getSkewness(), intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getSkewness(), intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPoints; i++) {
                if (!points3D.get(i).equals(euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
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
            final var configuration = new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getPairedCamerasAspectRatio();
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

            // 1st find an approximate central point by intersecting the axis planes of
            // both cameras
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

            final var numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final var points3D = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean leftFront;
            boolean rightFront;
            var maxTriesReached = false;
            for (var i = 0; i < numPoints; i++) {
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

                points3D.add(point3D);

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

            final var listener = new PairedViewsSparseReconstructorListener() {

                @Override
                public double onBaselineRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedCamera metricCamera1, final EstimatedCamera metricCamera2) {
                    return center1.distanceTo(center2);
                }

                @Override
                public boolean hasMoreViewsAvailable(final PairedViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentViewPair(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {

                    samples1.clear();
                    samples2.clear();

                    Sample2D sample1;
                    Sample2D sample2;
                    for (var i = 0; i < numPoints; i++) {
                        sample1 = new Sample2D();
                        sample1.setPoint(projectedPoints1.get(i));
                        sample1.setViewId(viewId1);
                        samples1.add(sample1);

                        sample2 = new Sample2D();
                        sample2.setPoint(projectedPoints2.get(i));
                        sample2.setViewId(viewId2);
                        samples2.add(sample2);
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onSamplesRejected(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onRequestMatches(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                samples1.get(i), samples2.get(i)
                        });
                        match.setViewIds(new int[]{viewId1, viewId2});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    PairedViewsSparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onEuclideanCameraPairEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final EstimatedCamera camera1, final EstimatedCamera camera2) {
                    estimatedEuclideanCamera1 = camera1;
                    estimatedEuclideanCamera2 = camera2;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                    return intrinsic;
                }

                @Override
                public void onStart(final PairedViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final PairedViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new PairedViewsSparseReconstructor(configuration, listener);

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
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.isAdditionalViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final var estEuclideanCam1 = this.estimatedEuclideanCamera1.getCamera();
            final var estEuclideanCam2 = this.estimatedEuclideanCamera2.getCamera();

            estEuclideanCam1.decompose();
            estEuclideanCam2.decompose();

            final var euclideanReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(euclideanReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints; i++) {
                final var p = euclideanReconstructedPoints3D.get(i);
                assertTrue(estEuclideanCam1.isPointInFrontOfCamera(p));
                assertTrue(estEuclideanCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estEuclideanCam1.getCameraCenter();
            final var estimatedCenter2 = estEuclideanCam2.getCameraCenter();

            // check scale
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);

            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);
            assertEquals(scale, baseline, LARGE_ABSOLUTE_ERROR);

            // check cameras
            final var estimatedIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var estimatedIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();

            final var estimatedRotation1 = estEuclideanCam1.getCameraRotation();
            final var estimatedRotation2 = estEuclideanCam2.getCameraRotation();

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR));

            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getSkewness(), intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getSkewness(), intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPoints; i++) {
                if (!points3D.get(i).equals(euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
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
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getPairedCamerasAspectRatio();
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

            // 1st find an approximate central point by intersecting the axis planes of
            // both cameras
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

            final var numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean leftFront;
            boolean rightFront;
            var maxTriesReached = false;
            for (var i = 0; i < numPoints; i++) {
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

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            final var listener = new PairedViewsSparseReconstructorListener() {

                @Override
                public double onBaselineRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedCamera metricCamera1, final EstimatedCamera metricCamera2) {
                    return center1.distanceTo(center2);
                }

                @Override
                public boolean hasMoreViewsAvailable(final PairedViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentViewPair(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {

                    samples1.clear();
                    samples2.clear();

                    Sample2D sample1;
                    Sample2D sample2;
                    for (var i = 0; i < numPoints; i++) {
                        sample1 = new Sample2D();
                        sample1.setPoint(projectedPoints1.get(i));
                        sample1.setViewId(viewId1);
                        samples1.add(sample1);

                        sample2 = new Sample2D();
                        sample2.setPoint(projectedPoints2.get(i));
                        sample2.setViewId(viewId2);
                        samples2.add(sample2);
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onSamplesRejected(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onRequestMatches(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                samples1.get(i), samples2.get(i)
                        });
                        match.setViewIds(new int[]{viewId1, viewId2});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    PairedViewsSparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onEuclideanCameraPairEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final EstimatedCamera camera1, final EstimatedCamera camera2) {
                    estimatedEuclideanCamera1 = camera1;
                    estimatedEuclideanCamera2 = camera2;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                    return intrinsic;
                }

                @Override
                public void onStart(final PairedViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final PairedViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new PairedViewsSparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertFalse(reconstructor.isFinished());

            try {
                reconstructor.start();
            } catch (final IndexOutOfBoundsException e) {
                continue;
            }

            // check correctness
            assertTrue(started);
            assertTrue(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.isAdditionalViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final var estEuclideanCam1 = this.estimatedEuclideanCamera1.getCamera();
            final var estEuclideanCam2 = this.estimatedEuclideanCamera2.getCamera();

            estEuclideanCam1.decompose();
            estEuclideanCam2.decompose();

            // check cameras
            final var estimatedIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var estimatedIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();

            if (Math.abs(estimatedIntrinsic1.getHorizontalFocalLength()
                    - intrinsic.getHorizontalFocalLength()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);

            if (Math.abs(estimatedIntrinsic1.getVerticalFocalLength()
                    - intrinsic.getVerticalFocalLength()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getSkewness(), intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getSkewness(), intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
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
            final var configuration = new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getPairedCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPointX = 0.0;
            final var principalPointY = 0.0;

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

            final var numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            final var points3D = new ArrayList<HomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            for (var i = 0; i < numPoints; i++) {
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
                points3D.add(point3D);

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            final var listener = new PairedViewsSparseReconstructorListener() {

                @Override
                public double onBaselineRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedCamera metricCamera1, final EstimatedCamera metricCamera2) {
                    return center1.distanceTo(center2);
                }

                @Override
                public boolean hasMoreViewsAvailable(final PairedViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentViewPair(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {

                    samples1.clear();
                    samples2.clear();

                    Sample2D sample1;
                    Sample2D sample2;
                    for (var i = 0; i < numPoints; i++) {
                        sample1 = new Sample2D();
                        sample1.setPoint(projectedPoints1.get(i));
                        sample1.setViewId(viewId1);
                        samples1.add(sample1);

                        sample2 = new Sample2D();
                        sample2.setPoint(projectedPoints2.get(i));
                        sample2.setViewId(viewId2);
                        samples2.add(sample2);
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onSamplesRejected(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onRequestMatches(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                samples1.get(i), samples2.get(i)
                        });
                        match.setViewIds(new int[]{viewId1, viewId2});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    PairedViewsSparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onEuclideanCameraPairEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final EstimatedCamera camera1, final EstimatedCamera camera2) {
                    estimatedEuclideanCamera1 = camera1;
                    estimatedEuclideanCamera2 = camera2;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                    return intrinsic;
                }

                @Override
                public void onStart(final PairedViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final PairedViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new PairedViewsSparseReconstructor(configuration, listener);

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
            if (!reconstructor.isAdditionalViewPair()) {
                continue;
            }
            assertTrue(reconstructor.isAdditionalViewPair());
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            if (reconstructor.getCurrentMetricEstimatedCamera() == null) {
                continue;
            }
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final var estEuclideanCam1 = this.estimatedEuclideanCamera1.getCamera();
            final var estEuclideanCam2 = this.estimatedEuclideanCamera2.getCamera();

            estEuclideanCam1.decompose();
            estEuclideanCam2.decompose();

            final var euclideanReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(euclideanReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints; i++) {
                final var p = euclideanReconstructedPoints3D.get(i);
                assertTrue(estEuclideanCam1.isPointInFrontOfCamera(p));
                assertTrue(estEuclideanCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estEuclideanCam1.getCameraCenter();
            final var estimatedCenter2 = estEuclideanCam2.getCameraCenter();

            // check scale
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);

            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);
            assertEquals(scale, baseline, LARGE_ABSOLUTE_ERROR);

            // check cameras
            final var estimatedIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var estimatedIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();

            final var estimatedRotation1 = estEuclideanCam1.getCameraRotation();
            final var estimatedRotation2 = estEuclideanCam2.getCameraRotation();

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR));

            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPoints; i++) {
                if (!points3D.get(i).equals(euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
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
            final var configuration = new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);
            configuration.setIntrinsicParametersKnown(true);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getPairedCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);
            configuration.setPairedCamerasAspectRatio(aspectRatio);

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

            final var numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            for (var i = 0; i < numPoints; i++) {
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

            final var listener = new PairedViewsSparseReconstructorListener() {

                @Override
                public double onBaselineRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedCamera metricCamera1, final EstimatedCamera metricCamera2) {
                    return center1.distanceTo(center2);
                }

                @Override
                public boolean hasMoreViewsAvailable(final PairedViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentViewPair(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {

                    samples1.clear();
                    samples2.clear();

                    Sample2D sample1;
                    Sample2D sample2;
                    for (var i = 0; i < numPoints; i++) {
                        sample1 = new Sample2D();
                        sample1.setPoint(projectedPoints1.get(i));
                        sample1.setViewId(viewId1);
                        samples1.add(sample1);

                        sample2 = new Sample2D();
                        sample2.setPoint(projectedPoints2.get(i));
                        sample2.setViewId(viewId2);
                        samples2.add(sample2);
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onSamplesRejected(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onRequestMatches(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                samples1.get(i), samples2.get(i)
                        });
                        match.setViewIds(new int[]{viewId1, viewId2});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    PairedViewsSparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onEuclideanCameraPairEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final EstimatedCamera camera1, final EstimatedCamera camera2) {
                    estimatedEuclideanCamera1 = camera1;
                    estimatedEuclideanCamera2 = camera2;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                    return intrinsic;
                }

                @Override
                public void onStart(final PairedViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final PairedViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new PairedViewsSparseReconstructor(configuration, listener);

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
            if (!reconstructor.isAdditionalViewPair()) {
                continue;
            }
            assertTrue(reconstructor.isAdditionalViewPair());
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            if (reconstructor.getCurrentMetricEstimatedCamera() == null) {
                continue;
            }
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final var estEuclideanCam1 = this.estimatedEuclideanCamera1.getCamera();
            final var estEuclideanCam2 = this.estimatedEuclideanCamera2.getCamera();

            estEuclideanCam1.decompose();
            estEuclideanCam2.decompose();

            final var euclideanReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(euclideanReconstructedPoints.get(i).getPoint());
            }

            // check that most of the points are in front of both cameras
            var valid = 0;
            var invalid = 0;
            for (var i = 0; i < numPoints; i++) {
                if (euclideanReconstructedPoints.get(i).isInlier()) {
                    final var p = euclideanReconstructedPoints3D.get(i);
                    assertTrue(estEuclideanCam1.isPointInFrontOfCamera(p));
                    assertTrue(estEuclideanCam2.isPointInFrontOfCamera(p));
                    valid++;
                } else {
                    invalid++;
                }
            }

            if (valid < invalid) {
                continue;
            }

            final var estimatedCenter1 = estEuclideanCam1.getCameraCenter();
            final var estimatedCenter2 = estEuclideanCam2.getCameraCenter();

            // check scale
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);

            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);
            assertEquals(scale, baseline, LARGE_ABSOLUTE_ERROR);

            // check cameras
            final var estimatedRotation1 = estEuclideanCam1.getCameraRotation();

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
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
            final var configuration = new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getPairedCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPointX = 0.0;
            final var principalPointY = 0.0;

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

            final var numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            final var points3D = new ArrayList<HomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            for (var i = 0; i < numPoints; i++) {
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
                points3D.add(point3D);

                // here 3D point is in front of both cameras

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            final var listener = new PairedViewsSparseReconstructorListener() {

                @Override
                public double onBaselineRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedCamera metricCamera1, final EstimatedCamera metricCamera2) {
                    return center1.distanceTo(center2);
                }

                @Override
                public boolean hasMoreViewsAvailable(final PairedViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentViewPair(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {

                    samples1.clear();
                    samples2.clear();

                    Sample2D sample1;
                    Sample2D sample2;
                    for (var i = 0; i < numPoints; i++) {
                        sample1 = new Sample2D();
                        sample1.setPoint(projectedPoints1.get(i));
                        sample1.setViewId(viewId1);
                        samples1.add(sample1);

                        sample2 = new Sample2D();
                        sample2.setPoint(projectedPoints2.get(i));
                        sample2.setViewId(viewId2);
                        samples2.add(sample2);
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onSamplesRejected(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onRequestMatches(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                samples1.get(i), samples2.get(i)
                        });
                        match.setViewIds(new int[]{viewId1, viewId2});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    PairedViewsSparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onEuclideanCameraPairEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final EstimatedCamera camera1, final EstimatedCamera camera2) {
                    estimatedEuclideanCamera1 = camera1;
                    estimatedEuclideanCamera2 = camera2;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                    return intrinsic;
                }

                @Override
                public void onStart(final PairedViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final PairedViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new PairedViewsSparseReconstructor(configuration, listener);

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
            if (!reconstructor.isAdditionalViewPair()) {
                continue;
            }
            assertTrue(reconstructor.isAdditionalViewPair());
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            if (reconstructor.getCurrentMetricEstimatedCamera() == null) {
                continue;
            }
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final var estEuclideanCam1 = this.estimatedEuclideanCamera1.getCamera();
            final var estEuclideanCam2 = this.estimatedEuclideanCamera2.getCamera();

            estEuclideanCam1.decompose();
            estEuclideanCam2.decompose();

            final var euclideanReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(euclideanReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints; i++) {
                final var p = euclideanReconstructedPoints3D.get(i);
                assertTrue(estEuclideanCam1.isPointInFrontOfCamera(p));
                assertTrue(estEuclideanCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estEuclideanCam1.getCameraCenter();
            final var estimatedCenter2 = estEuclideanCam2.getCameraCenter();

            // check scale
            final var baseline = center1.distanceTo(center2);
            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);

            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);
            assertEquals(scale, baseline, LARGE_ABSOLUTE_ERROR);

            // check cameras
            final var estimatedRotation1 = estEuclideanCam1.getCameraRotation();

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPoints; i++) {
                if (!points3D.get(i).equals(euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
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
            final var configuration = new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getPairedCamerasAspectRatio();
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

            final var numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            var failedIter = false;
            for (var i = 0; i < numPoints; i++) {
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

            final var listener = new PairedViewsSparseReconstructorListener() {

                @Override
                public double onBaselineRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedCamera metricCamera1, final EstimatedCamera metricCamera2) {
                    return center1.distanceTo(center2);
                }

                @Override
                public boolean hasMoreViewsAvailable(final PairedViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentViewPair(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {

                    samples1.clear();
                    samples2.clear();

                    Sample2D sample1;
                    Sample2D sample2;
                    for (var i = 0; i < numPoints; i++) {
                        sample1 = new Sample2D();
                        sample1.setPoint(projectedPoints1.get(i));
                        sample1.setViewId(viewId1);
                        samples1.add(sample1);

                        sample2 = new Sample2D();
                        sample2.setPoint(projectedPoints2.get(i));
                        sample2.setViewId(viewId2);
                        samples2.add(sample2);
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onSamplesRejected(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onRequestMatches(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                samples1.get(i), samples2.get(i)
                        });
                        match.setViewIds(new int[]{viewId1, viewId2});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    PairedViewsSparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onEuclideanCameraPairEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final EstimatedCamera camera1, final EstimatedCamera camera2) {
                    estimatedEuclideanCamera1 = camera1;
                    estimatedEuclideanCamera2 = camera2;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                    return intrinsic;
                }

                @Override
                public void onStart(final PairedViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final PairedViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                    PairedViewsSparseReconstructorTest.this.failed = true;
                }
            };

            final var reconstructor = new PairedViewsSparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(this.failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(started);
            assertTrue(finished);
            assertFalse(cancelled);
            assertFalse(this.failed);
            assertTrue(reconstructor.isFinished());
            if (!reconstructor.isAdditionalViewPair()) {
                continue;
            }
            assertTrue(reconstructor.isAdditionalViewPair());
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            if (reconstructor.getCurrentMetricEstimatedCamera() == null) {
                continue;
            }
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getEuclideanReconstructedPoints());
            assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final var estEuclideanCam1 = this.estimatedEuclideanCamera1.getCamera();
            final var estEuclideanCam2 = this.estimatedEuclideanCamera2.getCamera();

            estEuclideanCam1.decompose();
            estEuclideanCam2.decompose();

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsEssentialThreeViews() throws InvalidPairOfCamerasException, AlgebraException, CameraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

            final var randomizer = new UniformRandomizer();
            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
            final var aspectRatio = configuration.getPairedCamerasAspectRatio();
            final var skewness = 0.0;
            final var principalPoint = 0.0;

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPoint,
                    principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

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

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);
            final var fundamentalMatrix2 = new FundamentalMatrix(camera2, camera3);

            // create 3D points laying in front of all cameras

            // 1st find an approximate central point by intersecting the axis planes of
            // all cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var horizontalPlane3 = camera3.getHorizontalAxisPlane();
            final var verticalPlane3 = camera3.getVerticalAxisPlane();
            final var planesIntersectionMatrixPair1 = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            final var planesIntersectionMatrixPair2 = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrixPair1.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrixPair1.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrixPair1.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrixPair1.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrixPair1.setElementAt(1, 0, horizontalPlane1.getA());
            planesIntersectionMatrixPair1.setElementAt(1, 1, horizontalPlane1.getB());
            planesIntersectionMatrixPair1.setElementAt(1, 2, horizontalPlane1.getC());
            planesIntersectionMatrixPair1.setElementAt(1, 3, horizontalPlane1.getD());

            planesIntersectionMatrixPair1.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrixPair1.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrixPair1.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrixPair1.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrixPair1.setElementAt(3, 0, horizontalPlane2.getA());
            planesIntersectionMatrixPair1.setElementAt(3, 1, horizontalPlane2.getB());
            planesIntersectionMatrixPair1.setElementAt(3, 2, horizontalPlane2.getC());
            planesIntersectionMatrixPair1.setElementAt(3, 3, horizontalPlane2.getD());

            planesIntersectionMatrixPair2.setElementAt(0, 0, verticalPlane2.getA());
            planesIntersectionMatrixPair2.setElementAt(0, 1, verticalPlane2.getB());
            planesIntersectionMatrixPair2.setElementAt(0, 2, verticalPlane2.getC());
            planesIntersectionMatrixPair2.setElementAt(0, 3, verticalPlane2.getD());

            planesIntersectionMatrixPair2.setElementAt(1, 0, horizontalPlane2.getA());
            planesIntersectionMatrixPair2.setElementAt(1, 1, horizontalPlane2.getB());
            planesIntersectionMatrixPair2.setElementAt(1, 2, horizontalPlane2.getC());
            planesIntersectionMatrixPair2.setElementAt(1, 3, horizontalPlane2.getD());

            planesIntersectionMatrixPair2.setElementAt(2, 0, verticalPlane3.getA());
            planesIntersectionMatrixPair2.setElementAt(2, 1, verticalPlane3.getB());
            planesIntersectionMatrixPair2.setElementAt(2, 2, verticalPlane3.getC());
            planesIntersectionMatrixPair2.setElementAt(2, 3, verticalPlane3.getD());

            planesIntersectionMatrixPair2.setElementAt(3, 0, horizontalPlane3.getA());
            planesIntersectionMatrixPair2.setElementAt(3, 1, horizontalPlane3.getB());
            planesIntersectionMatrixPair2.setElementAt(3, 2, horizontalPlane3.getC());
            planesIntersectionMatrixPair2.setElementAt(3, 2, horizontalPlane3.getD());

            final var decomposerPair1 = new SingularValueDecomposer(planesIntersectionMatrixPair1);
            decomposerPair1.decompose();
            final var vPair1 = decomposerPair1.getV();

            final var decomposerPair2 = new SingularValueDecomposer(planesIntersectionMatrixPair2);
            decomposerPair2.decompose();
            final var vPair2 = decomposerPair2.getV();

            final var centralCommonPointPair1 = new HomogeneousPoint3D(
                    vPair1.getElementAt(0, 3),
                    vPair1.getElementAt(1, 3),
                    vPair1.getElementAt(2, 3),
                    vPair1.getElementAt(3, 3));

            final var centralCommonPointPair2 = new HomogeneousPoint3D(
                    vPair2.getElementAt(0, 3),
                    vPair2.getElementAt(1, 3),
                    vPair2.getElementAt(2, 3),
                    vPair2.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final var numPointsPair1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPointsPair2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final var points3DPair1 = new ArrayList<InhomogeneousPoint3D>();
            final var points3DPair2 = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            Point2D projectedPoint3;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2a = new ArrayList<Point2D>();
            final var projectedPoints2b = new ArrayList<Point2D>();
            final var projectedPoints3 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            boolean front3;
            for (var i = 0; i < numPointsPair1; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            centralCommonPointPair1.getInhomX() + lambdaX,
                            centralCommonPointPair1.getInhomY() + lambdaY,
                            centralCommonPointPair1.getInhomZ() + lambdaZ);

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    front3 = camera3.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front1 || !front2 || !front3);

                // here 3D point is in front of 1st pair of cameras

                points3DPair1.add(point3D);

                // project 3D point into 1st pair of cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2a.add(projectedPoint2);
            }

            for (var i = 0; i < numPointsPair2; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            center2.getInhomX() + centralCommonPointPair2.getInhomX() + lambdaX,
                            center2.getInhomY() + centralCommonPointPair2.getInhomY() + lambdaY,
                            center2.getInhomZ() + centralCommonPointPair2.getInhomZ() + lambdaZ);

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    front3 = camera3.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front1 || !front2 || !front3);

                // here 3D point is in front of 2nd pair of cameras

                points3DPair2.add(point3D);

                // project 3D point into 2nd pair of cameras
                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2b.add(projectedPoint2);

                projectedPoint3 = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3);
                projectedPoints3.add(projectedPoint3);
            }

            final var listener = new PairedViewsSparseReconstructorListener() {

                @Override
                public double onBaselineRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedCamera metricCamera1, final EstimatedCamera metricCamera2) {
                    final var vCount = reconstructor.getViewCount();
                    if (vCount == 0) {
                        return center1.distanceTo(center2);
                    } else if (vCount == 2) {
                        return center2.distanceTo(center3);
                    }

                    return 1.0;
                }

                @Override
                public boolean hasMoreViewsAvailable(final PairedViewsSparseReconstructor reconstructor) {
                    // 3 views = 2 view pairs (2 images * 2 views --> 4 view counts)
                    return viewCount < 4;
                }

                @Override
                public void onRequestSamplesForCurrentViewPair(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {

                    samples1.clear();
                    samples2.clear();

                    final var vCount = reconstructor.getViewCount();

                    Sample2D sample1;
                    Sample2D sample2;
                    if (vCount == 0) {
                        // first view pair
                        for (var i = 0; i < numPointsPair1; i++) {
                            sample1 = new Sample2D();
                            sample1.setPoint(projectedPoints1.get(i));
                            sample1.setViewId(viewId1);
                            samples1.add(sample1);

                            sample2 = new Sample2D();
                            sample2.setPoint(projectedPoints2a.get(i));
                            sample2.setViewId(viewId2);
                            samples2.add(sample2);
                        }

                    } else if (vCount == 2) {
                        // second view pair
                        for (var i = 0; i < numPointsPair2; i++) {
                            sample1 = new Sample2D();
                            sample1.setPoint(projectedPoints2b.get(i));
                            sample1.setViewId(viewId1);
                            samples1.add(sample1);

                            sample2 = new Sample2D();
                            sample2.setPoint(projectedPoints3.get(i));
                            sample2.setViewId(viewId2);
                            samples2.add(sample2);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onSamplesRejected(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onRequestMatches(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final List<Sample2D> samples1, final List<Sample2D> samples2, final List<MatchedSamples> matches) {
                    matches.clear();

                    final var vCount = reconstructor.getViewCount();
                    final int numPoints;
                    if (vCount == 0) {
                        // first view pair
                        numPoints = numPointsPair1;
                    } else {
                        // second view pair
                        numPoints = numPointsPair2;
                    }

                    MatchedSamples match;
                    for (var i = 0; i < numPoints; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                samples1.get(i), samples2.get(i)
                        });
                        match.setViewIds(new int[]{viewId1, viewId2});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {

                    final var vCount = reconstructor.getViewCount();
                    if (vCount == 0) {
                        PairedViewsSparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                    } else if (vCount == 2) {
                        estimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                    }
                }

                @Override
                public void onEuclideanCameraPairEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final EstimatedCamera camera1, final EstimatedCamera camera2) {

                    final var vCount = reconstructor.getViewCount();
                    if (vCount == 0) {
                        estimatedEuclideanCamera1 = camera1;
                        estimatedEuclideanCamera2 = camera2;
                        PairedViewsSparseReconstructorTest.this.scale = scale;
                    } else if (vCount == 2) {
                        estimatedEuclideanCamera2b = camera1;
                        estimatedEuclideanCamera3 = camera2;
                        scale2 = scale;
                    }
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                        final double scale, final List<ReconstructedPoint3D> points) {

                    final var vCount = reconstructor.getViewCount();
                    if (vCount == 0) {
                        euclideanReconstructedPoints = points;
                        PairedViewsSparseReconstructorTest.this.scale = scale;
                    } else if (vCount == 2) {
                        euclideanReconstructedPoints2 = points;
                        scale2 = scale;
                    }
                }

                @Override
                public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                        final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                    return intrinsic;
                }

                @Override
                public void onStart(final PairedViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final PairedViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new PairedViewsSparseReconstructor(configuration, listener);

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
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.isAdditionalViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix2, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera3, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera2b, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints2, reconstructor.getEuclideanReconstructedPoints());
            assertEquals(scale2, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

            // check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            fundamentalMatrix2.normalize();
            estimatedFundamentalMatrix.getFundamentalMatrix().normalize();
            estimatedFundamentalMatrix2.getFundamentalMatrix().normalize();

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

            // check that reconstructed points are in Euclidean stratum (up to a certain scale)
            final var estEuclideanCam1 = this.estimatedEuclideanCamera1.getCamera();
            final var estEuclideanCam2 = this.estimatedEuclideanCamera2.getCamera();
            final var estEuclideanCam2b = this.estimatedEuclideanCamera2b.getCamera();
            final var estEuclideanCam3 = this.estimatedEuclideanCamera3.getCamera();

            estEuclideanCam1.decompose();
            estEuclideanCam2.decompose();
            estEuclideanCam2b.decompose();
            estEuclideanCam3.decompose();

            final var euclideanReconstructedPoints3DPair1 = new ArrayList<Point3D>();
            for (var i = 0; i < numPointsPair1; i++) {
                euclideanReconstructedPoints3DPair1.add(euclideanReconstructedPoints.get(i).getPoint());
            }

            final var euclideanReconstructedPoints3DPair2 = new ArrayList<Point3D>();
            for (var i = 0; i < numPointsPair2; i++) {
                euclideanReconstructedPoints3DPair2.add(euclideanReconstructedPoints2.get(i).getPoint());
            }

            // check that most points are in front of all cameras
            var numValidPoints = 0;
            var numInvalidPoints = 0;
            for (var i = 0; i < numPointsPair1; i++) {
                final var p = euclideanReconstructedPoints3DPair1.get(i);
                if (estEuclideanCam1.isPointInFrontOfCamera(p)
                        && estEuclideanCam2.isPointInFrontOfCamera(p)
                        && estEuclideanCam3.isPointInFrontOfCamera(p)) {

                    assertTrue(estEuclideanCam1.isPointInFrontOfCamera(p));
                    assertTrue(estEuclideanCam2.isPointInFrontOfCamera(p));
                    assertTrue(estEuclideanCam3.isPointInFrontOfCamera(p));

                    numValidPoints++;
                } else {
                    numInvalidPoints++;
                }
            }

            assertTrue(numValidPoints > numInvalidPoints);

            numValidPoints = 0;
            numInvalidPoints = 0;
            for (var i = 0; i < numPointsPair2; i++) {
                final var p = euclideanReconstructedPoints3DPair2.get(i);
                if (estEuclideanCam1.isPointInFrontOfCamera(p)
                        && estEuclideanCam2.isPointInFrontOfCamera(p)
                        && estEuclideanCam2b.isPointInFrontOfCamera(p)
                        && estEuclideanCam3.isPointInFrontOfCamera(p)) {

                    assertTrue(estEuclideanCam1.isPointInFrontOfCamera(p));
                    assertTrue(estEuclideanCam2.isPointInFrontOfCamera(p));
                    assertTrue(estEuclideanCam2b.isPointInFrontOfCamera(p));
                    assertTrue(estEuclideanCam3.isPointInFrontOfCamera(p));

                    numValidPoints++;
                } else {
                    numInvalidPoints++;
                }
            }

            assertTrue(numValidPoints > numInvalidPoints);

            final var estimatedCenter1 = estEuclideanCam1.getCameraCenter();
            final var estimatedCenter2 = estEuclideanCam2.getCameraCenter();
            final var estimatedCenter2b = estEuclideanCam2b.getCameraCenter();
            final var estimatedCenter3 = estEuclideanCam3.getCameraCenter();

            // check scale
            final var baseline1 = center1.distanceTo(center2);
            final var estimatedBaseline1 = estimatedCenter1.distanceTo(estimatedCenter2);
            final var baseline2 = center2.distanceTo(center3);
            final var estimatedBaseline2 = estimatedCenter2.distanceTo(estimatedCenter3);

            if (Math.abs(estimatedBaseline1 - baseline1) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline1, baseline1, LARGE_ABSOLUTE_ERROR);
            assertEquals(scale, baseline1, LARGE_ABSOLUTE_ERROR);

            if (Math.abs(estimatedBaseline2 - baseline2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline2, baseline2, LARGE_ABSOLUTE_ERROR);
            assertEquals(scale2, baseline2, LARGE_ABSOLUTE_ERROR);

            // check cameras
            final var estimatedIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var estimatedIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();
            final var estimatedIntrinsic2b = estEuclideanCam2b.getIntrinsicParameters();
            final var estimatedIntrinsic3 = estEuclideanCam3.getIntrinsicParameters();

            final var estimatedRotation1 = estEuclideanCam1.getCameraRotation();
            final var estimatedRotation2 = estEuclideanCam2.getCameraRotation();
            final var estimatedRotation2b = estEuclideanCam2b.getCameraRotation();
            final var estimatedRotation3 = estEuclideanCam3.getCameraRotation();

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR));

            if (!estimatedCenter2.equals(estimatedCenter2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimatedCenter2.equals(estimatedCenter2b, ABSOLUTE_ERROR));

            if (!center3.equals(estimatedCenter3, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center3.equals(estimatedCenter3, LARGE_ABSOLUTE_ERROR));

            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2b.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2b.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2b.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2b.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2b.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic3.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic3.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic3.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic3.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic3.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(estimatedRotation2b.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(estimatedRotation3.asInhomogeneousMatrix().equals(rotation3.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPointsPair1; i++) {
                if (!points3DPair1.get(i).equals(euclideanReconstructedPoints3DPair1.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3DPair1.get(i).equals(euclideanReconstructedPoints3DPair1.get(i),
                        LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            for (var i = 0; i < numPointsPair2; i++) {
                if (!points3DPair2.get(i).equals(euclideanReconstructedPoints3DPair2.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3DPair2.get(i).equals(euclideanReconstructedPoints3DPair2.get(i),
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
    void testCancel() throws GeometryException, AlgebraException {
        final var configuration = new PairedViewsSparseReconstructorConfiguration();
        configuration.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        configuration.setIntrinsicParametersKnown(true);

        final var randomizer = new UniformRandomizer();
        final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL, MAX_FOCAL_LENGTH_ESSENTIAL);
        final var aspectRatio = configuration.getPairedCamerasAspectRatio();
        final var skewness = 0.0;
        final var principalPoint = 0.0;

        final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPoint,
                principalPoint, skewness);
        intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

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

        // create 3D points laying in front of all cameras

        // 1st find an approximate central point by intersecting the axis planes of
        // all cameras
        final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
        final var verticalPlane1 = camera1.getVerticalAxisPlane();
        final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
        final var verticalPlane2 = camera2.getVerticalAxisPlane();
        final var horizontalPlane3 = camera3.getHorizontalAxisPlane();
        final var verticalPlane3 = camera3.getVerticalAxisPlane();
        final var planesIntersectionMatrixPair1 = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
        final var planesIntersectionMatrixPair2 = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
        planesIntersectionMatrixPair1.setElementAt(0, 0, verticalPlane1.getA());
        planesIntersectionMatrixPair1.setElementAt(0, 1, verticalPlane1.getB());
        planesIntersectionMatrixPair1.setElementAt(0, 2, verticalPlane1.getC());
        planesIntersectionMatrixPair1.setElementAt(0, 3, verticalPlane1.getD());

        planesIntersectionMatrixPair1.setElementAt(1, 0, horizontalPlane1.getA());
        planesIntersectionMatrixPair1.setElementAt(1, 1, horizontalPlane1.getB());
        planesIntersectionMatrixPair1.setElementAt(1, 2, horizontalPlane1.getC());
        planesIntersectionMatrixPair1.setElementAt(1, 3, horizontalPlane1.getD());

        planesIntersectionMatrixPair1.setElementAt(2, 0, verticalPlane2.getA());
        planesIntersectionMatrixPair1.setElementAt(2, 1, verticalPlane2.getB());
        planesIntersectionMatrixPair1.setElementAt(2, 2, verticalPlane2.getC());
        planesIntersectionMatrixPair1.setElementAt(2, 3, verticalPlane2.getD());

        planesIntersectionMatrixPair1.setElementAt(3, 0, horizontalPlane2.getA());
        planesIntersectionMatrixPair1.setElementAt(3, 1, horizontalPlane2.getB());
        planesIntersectionMatrixPair1.setElementAt(3, 2, horizontalPlane2.getC());
        planesIntersectionMatrixPair1.setElementAt(3, 3, horizontalPlane2.getD());

        planesIntersectionMatrixPair2.setElementAt(0, 0, verticalPlane2.getA());
        planesIntersectionMatrixPair2.setElementAt(0, 1, verticalPlane2.getB());
        planesIntersectionMatrixPair2.setElementAt(0, 2, verticalPlane2.getC());
        planesIntersectionMatrixPair2.setElementAt(0, 3, verticalPlane2.getD());

        planesIntersectionMatrixPair2.setElementAt(1, 0, horizontalPlane2.getA());
        planesIntersectionMatrixPair2.setElementAt(1, 1, horizontalPlane2.getB());
        planesIntersectionMatrixPair2.setElementAt(1, 2, horizontalPlane2.getC());
        planesIntersectionMatrixPair2.setElementAt(1, 3, horizontalPlane2.getD());

        planesIntersectionMatrixPair2.setElementAt(2, 0, verticalPlane3.getA());
        planesIntersectionMatrixPair2.setElementAt(2, 1, verticalPlane3.getB());
        planesIntersectionMatrixPair2.setElementAt(2, 2, verticalPlane3.getC());
        planesIntersectionMatrixPair2.setElementAt(2, 3, verticalPlane3.getD());

        planesIntersectionMatrixPair2.setElementAt(3, 0, horizontalPlane3.getA());
        planesIntersectionMatrixPair2.setElementAt(3, 1, horizontalPlane3.getB());
        planesIntersectionMatrixPair2.setElementAt(3, 2, horizontalPlane3.getC());
        planesIntersectionMatrixPair2.setElementAt(3, 2, horizontalPlane3.getD());

        final var decomposerPair1 = new SingularValueDecomposer(planesIntersectionMatrixPair1);
        decomposerPair1.decompose();
        final var vPair1 = decomposerPair1.getV();

        final var decomposerPair2 = new SingularValueDecomposer(planesIntersectionMatrixPair2);
        decomposerPair2.decompose();
        final var vPair2 = decomposerPair2.getV();

        final var centralCommonPointPair1 = new HomogeneousPoint3D(
                vPair1.getElementAt(0, 3),
                vPair1.getElementAt(1, 3),
                vPair1.getElementAt(2, 3),
                vPair1.getElementAt(3, 3));

        final var centralCommonPointPair2 = new HomogeneousPoint3D(
                vPair2.getElementAt(0, 3),
                vPair2.getElementAt(1, 3),
                vPair2.getElementAt(2, 3),
                vPair2.getElementAt(3, 3));

        double lambdaX;
        double lambdaY;
        double lambdaZ;

        final var numPointsPair1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
        final var numPointsPair2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

        InhomogeneousPoint3D point3D;
        Point2D projectedPoint1;
        Point2D projectedPoint2;
        Point2D projectedPoint3;
        final var projectedPoints1 = new ArrayList<Point2D>();
        final var projectedPoints2a = new ArrayList<Point2D>();
        final var projectedPoints2b = new ArrayList<Point2D>();
        final var projectedPoints3 = new ArrayList<Point2D>();
        boolean front1;
        boolean front2;
        boolean front3;
        for (var i = 0; i < numPointsPair1; i++) {
            // generate points and ensure they lie in front of both cameras
            var numTry = 0;
            do {
                lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                point3D = new InhomogeneousPoint3D(
                        centralCommonPointPair1.getInhomX() + lambdaX,
                        centralCommonPointPair1.getInhomY() + lambdaY,
                        centralCommonPointPair1.getInhomZ() + lambdaZ);

                front1 = camera1.isPointInFrontOfCamera(point3D);
                front2 = camera2.isPointInFrontOfCamera(point3D);
                front3 = camera3.isPointInFrontOfCamera(point3D);
                if (numTry > MAX_TRIES) {
                    fail("max tries reached");
                }
                numTry++;
            } while (!front1 || !front2 || !front3);

            // here 3D point is in front of 1st pair of cameras

            // project 3D point into 1st pair of cameras
            projectedPoint1 = new InhomogeneousPoint2D();
            camera1.project(point3D, projectedPoint1);
            projectedPoints1.add(projectedPoint1);

            projectedPoint2 = new InhomogeneousPoint2D();
            camera2.project(point3D, projectedPoint2);
            projectedPoints2a.add(projectedPoint2);
        }

        for (var i = 0; i < numPointsPair2; i++) {
            // generate points and ensure they lie in front of both cameras
            var numTry = 0;
            do {
                lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                point3D = new InhomogeneousPoint3D(
                        center2.getInhomX() + centralCommonPointPair2.getInhomX() + lambdaX,
                        center2.getInhomY() + centralCommonPointPair2.getInhomY() + lambdaY,
                        center2.getInhomZ() + centralCommonPointPair2.getInhomZ() + lambdaZ);

                front1 = camera1.isPointInFrontOfCamera(point3D);
                front2 = camera2.isPointInFrontOfCamera(point3D);
                front3 = camera3.isPointInFrontOfCamera(point3D);
                if (numTry > MAX_TRIES) {
                    fail("max tries reached");
                }
                numTry++;
            } while (!front1 || !front2 || !front3);

            // here 3D point is in front of 2nd pair of cameras

            // project 3D point into 2nd pair of cameras
            projectedPoint2 = new InhomogeneousPoint2D();
            camera2.project(point3D, projectedPoint2);
            projectedPoints2b.add(projectedPoint2);

            projectedPoint3 = new InhomogeneousPoint2D();
            camera3.project(point3D, projectedPoint3);
            projectedPoints3.add(projectedPoint3);
        }

        final var listener = new PairedViewsSparseReconstructorListener() {

            @Override
            public double onBaselineRequested(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                    final EstimatedCamera metricCamera1, final EstimatedCamera metricCamera2) {
                final var vCount = reconstructor.getViewCount();
                if (vCount == 0) {
                    return center1.distanceTo(center2);
                } else if (vCount == 2) {
                    return center2.distanceTo(center3);
                }

                return 1.0;
            }

            @Override
            public boolean hasMoreViewsAvailable(final PairedViewsSparseReconstructor reconstructor) {
                reconstructor.cancel();
                return viewCount < 4; //3 views = 2 view pairs (2 images * 2 views --> 4 view counts)
            }

            @Override
            public void onRequestSamplesForCurrentViewPair(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                    final List<Sample2D> samples1, final List<Sample2D> samples2) {

                samples1.clear();
                samples2.clear();

                final var vCount = reconstructor.getViewCount();

                Sample2D sample1;
                Sample2D sample2;
                if (vCount == 0) {
                    // first view pair
                    for (var i = 0; i < numPointsPair1; i++) {
                        sample1 = new Sample2D();
                        sample1.setPoint(projectedPoints1.get(i));
                        sample1.setViewId(viewId1);
                        samples1.add(sample1);

                        sample2 = new Sample2D();
                        sample2.setPoint(projectedPoints2a.get(i));
                        sample2.setViewId(viewId2);
                        samples2.add(sample2);
                    }

                } else if (vCount == 2) {
                    // second view pair
                    for (var i = 0; i < numPointsPair2; i++) {
                        sample1 = new Sample2D();
                        sample1.setPoint(projectedPoints2b.get(i));
                        sample1.setViewId(viewId1);
                        samples1.add(sample1);

                        sample2 = new Sample2D();
                        sample2.setPoint(projectedPoints3.get(i));
                        sample2.setViewId(viewId2);
                        samples2.add(sample2);
                    }
                }
            }

            @Override
            public void onSamplesAccepted(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                    final List<Sample2D> samples1, final List<Sample2D> samples2) {
                viewCount += 2;
            }

            @Override
            public void onSamplesRejected(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                    final List<Sample2D> samples1, final List<Sample2D> samples2) {
                viewCount += 2;
            }

            @Override
            public void onRequestMatches(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                    final List<Sample2D> samples1, final List<Sample2D> samples2, final List<MatchedSamples> matches) {
                matches.clear();

                final var vCount = reconstructor.getViewCount();
                final int numPoints;
                if (vCount == 0) {
                    // first view pair
                    numPoints = numPointsPair1;
                } else {
                    // second view pair
                    numPoints = numPointsPair2;
                }

                MatchedSamples match;
                for (var i = 0; i < numPoints; i++) {
                    match = new MatchedSamples();
                    match.setSamples(new Sample2D[]{
                            samples1.get(i), samples2.get(i)
                    });
                    match.setViewIds(new int[]{viewId1, viewId2});
                    matches.add(match);
                }
            }

            @Override
            public void onFundamentalMatrixEstimated(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                    final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {

                final var vCount = reconstructor.getViewCount();
                if (vCount == 0) {
                    PairedViewsSparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                } else if (vCount == 2) {
                    estimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                }
            }

            @Override
            public void onEuclideanCameraPairEstimated(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                    final double scale, final EstimatedCamera camera1, final EstimatedCamera camera2) {

                final var vCount = reconstructor.getViewCount();
                if (vCount == 0) {
                    estimatedEuclideanCamera1 = camera1;
                    estimatedEuclideanCamera2 = camera2;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                } else if (vCount == 2) {
                    estimatedEuclideanCamera2b = camera1;
                    estimatedEuclideanCamera3 = camera2;
                    scale2 = scale;
                }
            }

            @Override
            public void onEuclideanReconstructedPointsEstimated(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId1, final int viewId2,
                    final double scale, final List<ReconstructedPoint3D> points) {

                final var vCount = reconstructor.getViewCount();
                if (vCount == 0) {
                    euclideanReconstructedPoints = points;
                    PairedViewsSparseReconstructorTest.this.scale = scale;
                } else if (vCount == 2) {
                    euclideanReconstructedPoints2 = points;
                    scale2 = scale;
                }
            }

            @Override
            public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                    final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                return intrinsic;
            }

            @Override
            public void onStart(final PairedViewsSparseReconstructor reconstructor) {
                started = true;
            }

            @Override
            public void onFinish(final PairedViewsSparseReconstructor reconstructor) {
                finished = true;
            }

            @Override
            public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                cancelled = true;
            }

            @Override
            public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                failed = true;
            }
        };

        final var reconstructor = new PairedViewsSparseReconstructor(configuration, listener);

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
        assertFalse(finished);
        assertTrue(cancelled);
        assertFalse(failed);
        assertFalse(reconstructor.isFinished());
        assertFalse(reconstructor.isFirstViewPair());
        assertTrue(reconstructor.isAdditionalViewPair());
        assertTrue(reconstructor.getViewCount() > 0);
        assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
        assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
        assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
        assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
        assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
        assertSame(estimatedEuclideanCamera2, reconstructor.getCurrentEuclideanEstimatedCamera());
        assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
        assertSame(estimatedEuclideanCamera1, reconstructor.getPreviousEuclideanEstimatedCamera());
        assertNotNull(reconstructor.getMetricReconstructedPoints());
        assertNotNull(reconstructor.getEuclideanReconstructedPoints());
        assertSame(euclideanReconstructedPoints, reconstructor.getEuclideanReconstructedPoints());
        assertEquals(scale, reconstructor.getCurrentScale(), 0.0);
        assertNotNull(reconstructor.getPreviousViewSamples());
        assertNotNull(reconstructor.getCurrentViewSamples());
    }

    private void reset() {
        viewCount = 0;
        estimatedFundamentalMatrix = estimatedFundamentalMatrix2 = null;
        estimatedEuclideanCamera1 = estimatedEuclideanCamera2 = estimatedEuclideanCamera3 = null;
        euclideanReconstructedPoints = null;
        started = finished = failed = cancelled = false;
        scale = 0.0;
    }
}
