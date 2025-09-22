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
import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.LockedException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.NotAvailableException;
import com.irurueta.algebra.NotReadyException;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.WrongSizeException;
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

class KnownBaselineTwoViewsSparseReconstructorTest {

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
    private EstimatedCamera estimatedCamera1;
    private EstimatedCamera estimatedCamera2;
    private List<ReconstructedPoint3D> reconstructedPoints;

    private boolean started;
    private boolean finished;
    private boolean failed;
    private boolean cancelled;

    @BeforeEach
    void setUp() {
        viewCount = 0;
        estimatedFundamentalMatrix = null;
        estimatedCamera1 = estimatedCamera2 = null;
        reconstructedPoints = null;
        started = finished = failed = cancelled = false;
    }

    @Test
    void testConstructor() {
        final var configuration = new KnownBaselineTwoViewsSparseReconstructorConfiguration();
        final var listener = new KnownBaselineTwoViewsSparseReconstructorListener() {

            @Override
            public boolean hasMoreViewsAvailable(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                return false;
            }

            @Override
            public void onRequestSamplesForCurrentView(
                    final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                    final int viewId, final List<Sample2D> samples) {
                // no action needed
            }

            @Override
            public void onSamplesAccepted(
                    final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                    final int viewId, final List<Sample2D> samples) {
                // no action needed
            }

            @Override
            public void onSamplesRejected(
                    final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                    final int viewId, final List<Sample2D> samples) {
                // no action needed
            }

            @Override
            public void onRequestMatches(
                    final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                    final List<Sample2D> samples1, final List<Sample2D> samples2,
                    final int viewId1, final int viewId2, final List<MatchedSamples> matches) {
                // no action needed
            }

            @Override
            public void onFundamentalMatrixEstimated(
                    final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                    final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                // no action needed
            }

            @Override
            public void onCamerasEstimated(
                    final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                    final int viewId1, final int viewId2, final EstimatedCamera camera1,
                    final EstimatedCamera camera2) {
                // no action needed
            }

            @Override
            public void onReconstructedPointsEstimated(
                    final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                    final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                // no action needed
            }

            @Override
            public void onStart(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onFinish(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onCancel(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onFail(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                // no action needed
            }
        };

        // constructor with listener
        var reconstructor = new KnownBaselineTwoViewsSparseReconstructor(listener);

        // check default values
        assertNotNull(reconstructor.getConfiguration());
        assertSame(listener, reconstructor.getListener());
        assertFalse(reconstructor.isRunning());
        assertFalse(reconstructor.isCancelled());
        assertFalse(reconstructor.hasFailed());
        assertFalse(reconstructor.isFinished());
        assertEquals(0, reconstructor.getViewCount());
        assertNull(reconstructor.getEstimatedFundamentalMatrix());
        assertNull(reconstructor.getEstimatedCamera1());
        assertNull(reconstructor.getEstimatedCamera2());
        assertNull(reconstructor.getReconstructedPoints());

        // constructor with configuration and listener
        reconstructor = new KnownBaselineTwoViewsSparseReconstructor(configuration, listener);

        // check default values
        assertSame(configuration, reconstructor.getConfiguration());
        assertSame(listener, reconstructor.getListener());
        assertFalse(reconstructor.isRunning());
        assertFalse(reconstructor.isCancelled());
        assertFalse(reconstructor.hasFailed());
        assertEquals(0, reconstructor.getViewCount());
        assertNull(reconstructor.getEstimatedFundamentalMatrix());
        assertNull(reconstructor.getEstimatedCamera1());
        assertNull(reconstructor.getEstimatedCamera2());
        assertNull(reconstructor.getReconstructedPoints());
    }

    @Test
    void testGeneralPointsEssential() throws InvalidPairOfCamerasException, WrongSizeException, NotReadyException,
            LockedException, DecomposerException, NotAvailableException, CameraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new KnownBaselineTwoViewsSparseReconstructorConfiguration();
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

            final var baseline = center1.distanceTo(center2);
            configuration.setBaseline(baseline);

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

            final var listener = new KnownBaselineTwoViewsSparseReconstructorListener() {

                @Override
                public boolean hasMoreViewsAvailable(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {

                    samples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    // no action needed
                }

                @Override
                public void onRequestMatches(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final int viewId1, final int viewId2,
                        final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{samples1.get(i), samples2.get(i)});
                        match.setViewIds(new int[]{viewId1, viewId2});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    KnownBaselineTwoViewsSparseReconstructorTest.this.estimatedFundamentalMatrix =
                            estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId1, final int viewId2, final EstimatedCamera camera1,
                        final EstimatedCamera camera2) {
                    estimatedCamera1 = camera1;
                    estimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    reconstructedPoints = points;
                }

                @Override
                public void onStart(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new KnownBaselineTwoViewsSparseReconstructor(configuration, listener);

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
                    estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(),
                    ABSOLUTE_ERROR) || fundamentalMatrix.getInternalMatrix()
                    .multiplyByScalarAndReturnNew(-1).equals(
                            estimatedFundamentalMatrix.getFundamentalMatrix().getInternalMatrix(),
                            ABSOLUTE_ERROR));

            // check that reconstructed points are in a Euclidean stratum (with
            // correct scale)
            final var estCam1 = this.estimatedCamera1.getCamera();
            final var estCam2 = this.estimatedCamera2.getCamera();

            estCam1.decompose();
            estCam2.decompose();

            final var reconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints; i++) {
                reconstructedPoints3D.add(reconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints; i++) {
                final var p = reconstructedPoints3D.get(i);
                assertTrue(estCam1.isPointInFrontOfCamera(p));
                assertTrue(estCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estCam1.getCameraCenter();
            final var estimatedCenter2 = estCam2.getCameraCenter();

            final var estimatedIntrinsic1 = estCam1.getIntrinsicParameters();
            final var estimatedIntrinsic2 = estCam2.getIntrinsicParameters();

            final var estimatedRotation1 = estCam1.getCameraRotation();
            final var estimatedRotation2 = estCam2.getCameraRotation();

            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);

            // check cameras are correct
            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);

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
                if (!points3D.get(i).equals(reconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(reconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            // cancel
            assertFalse(reconstructor.isCancelled());
            reconstructor.cancel();

            assertTrue(reconstructor.isCancelled());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsDIAC() throws InvalidPairOfCamerasException, CameraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException,
            AlgebraException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new KnownBaselineTwoViewsSparseReconstructorConfiguration();
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

            final var baseline = center1.distanceTo(center2);
            configuration.setBaseline(baseline);

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
            HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
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

            final var listener = new KnownBaselineTwoViewsSparseReconstructorListener() {

                @Override
                public boolean hasMoreViewsAvailable(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {

                    samples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    // no action needed
                }

                @Override
                public void onRequestMatches(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final int viewId1, final int viewId2, final List<MatchedSamples> matches) {
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
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    KnownBaselineTwoViewsSparseReconstructorTest.this.estimatedFundamentalMatrix =
                            estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId1, final int viewId2, final EstimatedCamera camera1,
                        final EstimatedCamera camera2) {
                    estimatedCamera1 = camera1;
                    estimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    reconstructedPoints = points;
                }

                @Override
                public void onStart(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new KnownBaselineTwoViewsSparseReconstructor(configuration, listener);

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
            final var estCam1 = this.estimatedCamera1.getCamera();
            final var estCam2 = this.estimatedCamera2.getCamera();

            estCam1.decompose();
            estCam2.decompose();

            final var reconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints; i++) {
                reconstructedPoints3D.add(reconstructedPoints.get(i).getPoint());
            }

            // check that most of the points are in front of both cameras
            var valid = 0;
            var invalid = 0;
            for (var i = 0; i < numPoints; i++) {
                if (reconstructedPoints.get(i).isInlier()) {
                    final var p = reconstructedPoints3D.get(i);
                    assertTrue(estCam1.isPointInFrontOfCamera(p));
                    assertTrue(estCam2.isPointInFrontOfCamera(p));
                    valid++;
                } else {
                    invalid++;
                }
            }

            if (valid < invalid) {
                continue;
            }

            final var estimatedCenter1 = estCam1.getCameraCenter();
            final var estimatedCenter2 = estCam2.getCameraCenter();

            final var estimatedIntrinsic1 = estCam1.getIntrinsicParameters();
            final var estimatedIntrinsic2 = estCam2.getIntrinsicParameters();

            final var estimatedRotation1 = estCam1.getCameraRotation();
            final var estimatedRotation2 = estCam2.getCameraRotation();

            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);

            // check cameras are correct
            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR));

            if (Math.abs(estimatedIntrinsic1.getHorizontalFocalLength()
                    - intrinsic.getHorizontalFocalLength()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            if (Math.abs(estimatedIntrinsic1.getVerticalFocalLength()
                    - intrinsic.getVerticalFocalLength()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            if (Math.abs(estimatedIntrinsic1.getSkewness() - intrinsic.getSkewness()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            if (Math.abs(estimatedIntrinsic1.getHorizontalPrincipalPoint()
                    - intrinsic.getHorizontalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            if (Math.abs(estimatedIntrinsic1.getVerticalPrincipalPoint()
                    - intrinsic.getVerticalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            if (Math.abs(estimatedIntrinsic2.getHorizontalFocalLength()
                    - intrinsic.getHorizontalFocalLength()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            if (Math.abs(estimatedIntrinsic2.getVerticalFocalLength()
                    - intrinsic.getVerticalFocalLength()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            if (Math.abs(estimatedIntrinsic2.getSkewness() - intrinsic.getSkewness()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            if (Math.abs(estimatedIntrinsic2.getHorizontalPrincipalPoint()
                    - intrinsic.getHorizontalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            if (Math.abs(estimatedIntrinsic2.getVerticalPrincipalPoint()
                    - intrinsic.getVerticalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPoints; i++) {
                if (!points3D.get(i).equals(reconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(reconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            // cancel
            assertFalse(reconstructor.isCancelled());
            reconstructor.cancel();

            assertTrue(reconstructor.isCancelled());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsDAQAndEssentialZeroPrincipalPoint() throws InvalidPairOfCamerasException, CameraException,
            AlgebraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new KnownBaselineTwoViewsSparseReconstructorConfiguration();
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

            final var baseline = center1.distanceTo(center2);
            configuration.setBaseline(baseline);

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

                // here world point is in front of both cameras

                // project world point into both cameras
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

            final var listener = new KnownBaselineTwoViewsSparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {

                    samples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    // no action needed
                }

                @Override
                public void onRequestMatches(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final int viewId1, final int viewId2,
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
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    KnownBaselineTwoViewsSparseReconstructorTest.this.estimatedFundamentalMatrix =
                            estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId1, final int viewId2, final EstimatedCamera camera1,
                        final EstimatedCamera camera2) {
                    estimatedCamera1 = camera1;
                    estimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    reconstructedPoints = points;
                }

                @Override
                public void onStart(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new KnownBaselineTwoViewsSparseReconstructor(configuration, listener);

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
            final var estCam1 = this.estimatedCamera1.getCamera();
            final var estCam2 = this.estimatedCamera2.getCamera();

            estCam1.decompose();
            estCam2.decompose();

            final var reconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints; i++) {
                reconstructedPoints3D.add(reconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints; i++) {
                final var p = reconstructedPoints3D.get(i);
                assertTrue(estCam1.isPointInFrontOfCamera(p));
                assertTrue(estCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estCam1.getCameraCenter();
            final var estimatedCenter2 = estCam2.getCameraCenter();

            final var estimatedIntrinsic1 = estCam1.getIntrinsicParameters();
            final var estimatedIntrinsic2 = estCam2.getIntrinsicParameters();

            final var estimatedRotation1 = estCam1.getCameraRotation();
            final var estimatedRotation2 = estCam2.getCameraRotation();

            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);

            // check cameras are correct
            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);

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
                if (!points3D.get(i).equals(reconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(reconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            // cancel
            assertFalse(reconstructor.isCancelled());
            reconstructor.cancel();

            assertTrue(reconstructor.isCancelled());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsDAQAndEssential() throws InvalidPairOfCamerasException, CameraException, AlgebraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new KnownBaselineTwoViewsSparseReconstructorConfiguration();
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

            final var baseline = center1.distanceTo(center2);
            configuration.setBaseline(baseline);

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

                // here world point is in front of both cameras

                // project world point into both cameras
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

            final var listener = new KnownBaselineTwoViewsSparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {

                    samples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    // no action needed
                }

                @Override
                public void onRequestMatches(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final int viewId1, final int viewId2, final List<MatchedSamples> matches) {
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
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    KnownBaselineTwoViewsSparseReconstructorTest.this.estimatedFundamentalMatrix =
                            estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId1, final int viewId2, final EstimatedCamera camera1,
                        final EstimatedCamera camera2) {
                    estimatedCamera1 = camera1;
                    estimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    reconstructedPoints = points;
                }

                @Override
                public void onStart(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new KnownBaselineTwoViewsSparseReconstructor(configuration, listener);

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
            final var estCam1 = this.estimatedCamera1.getCamera();
            final var estCam2 = this.estimatedCamera2.getCamera();

            estCam1.decompose();
            estCam2.decompose();

            final var reconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints; i++) {
                reconstructedPoints3D.add(reconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints; i++) {
                final var p = reconstructedPoints3D.get(i);
                assertTrue(estCam1.isPointInFrontOfCamera(p));
                assertTrue(estCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estCam1.getCameraCenter();
            final var estimatedCenter2 = estCam2.getCameraCenter();

            final var estimatedIntrinsic1 = estCam1.getIntrinsicParameters();
            final var estimatedIntrinsic2 = estCam2.getIntrinsicParameters();

            final var estimatedRotation1 = estCam1.getCameraRotation();
            final var estimatedRotation2 = estCam2.getCameraRotation();

            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);

            // check cameras are correct
            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);

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
                if (!points3D.get(i).equals(reconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(reconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            // cancel
            assertFalse(reconstructor.isCancelled());
            reconstructor.cancel();

            assertTrue(reconstructor.isCancelled());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsDAQ() throws InvalidPairOfCamerasException, CameraException, RobustEstimatorException,
            AlgebraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException, com.irurueta.geometry.estimators.LockedException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new KnownBaselineTwoViewsSparseReconstructorConfiguration();
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

            final var baseline = center1.distanceTo(center2);
            configuration.setBaseline(baseline);

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

            final var numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final var points3D = new ArrayList<Point3D>();
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

                // here world point is in front of both cameras

                // project world point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            final var listener = new KnownBaselineTwoViewsSparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {

                    samples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    // no action needed
                }

                @Override
                public void onRequestMatches(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final int viewId1, final int viewId2, final List<MatchedSamples> matches) {
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
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    KnownBaselineTwoViewsSparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId1, final int viewId2, final EstimatedCamera camera1,
                        final EstimatedCamera camera2) {
                    estimatedCamera1 = camera1;
                    estimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    reconstructedPoints = points;
                }

                @Override
                public void onStart(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            if (maxTriesReached) {
                continue;
            }

            final var reconstructor = new KnownBaselineTwoViewsSparseReconstructor(configuration, listener);

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
            // certain scale, rotation and translation)
            final var estCam1 = this.estimatedCamera1.getCamera();
            final var estCam2 = this.estimatedCamera2.getCamera();

            final var reconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < points3D.size(); i++) {
                reconstructedPoints3D.add(reconstructedPoints.get(i).getPoint());
            }

            final var transformationEstimator = MetricTransformation3DRobustEstimator.create(reconstructedPoints3D,
                    points3D, RobustEstimatorMethod.LMEDS);

            final var transformation = transformationEstimator.estimate();

            final var transformedCamera1 = transformation.transformAndReturnNew(estCam1);
            final var transformedCamera2 = transformation.transformAndReturnNew(estCam2);

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

            // cancel
            assertFalse(reconstructor.isCancelled());
            reconstructor.cancel();

            assertTrue(reconstructor.isCancelled());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testPlanarPointsEssential() throws InvalidPairOfCamerasException, CameraException, AlgebraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new KnownBaselineTwoViewsSparseReconstructorConfiguration();
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

            final var baseline = center1.distanceTo(center2);
            configuration.setBaseline(baseline);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras and laying in
            // a plane

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

            final var listener = new KnownBaselineTwoViewsSparseReconstructorListener() {

                @Override
                public boolean hasMoreViewsAvailable(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> samples) {

                    samples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final int viewId1, final int viewId2, final List<MatchedSamples> matches) {
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
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    KnownBaselineTwoViewsSparseReconstructorTest.this.estimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId1, final int viewId2, final EstimatedCamera camera1,
                        final EstimatedCamera camera2) {
                    estimatedCamera1 = camera1;
                    estimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches,
                        final List<ReconstructedPoint3D> points) {
                    reconstructedPoints = points;
                }

                @Override
                public void onStart(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new KnownBaselineTwoViewsSparseReconstructor(configuration, listener);

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
            final var estCam1 = this.estimatedCamera1.getCamera();
            final var estCam2 = this.estimatedCamera2.getCamera();

            estCam1.decompose();
            estCam2.decompose();

            final var reconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints; i++) {
                reconstructedPoints3D.add(reconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints; i++) {
                final var p = reconstructedPoints3D.get(i);
                assertTrue(estCam1.isPointInFrontOfCamera(p));
                assertTrue(estCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estCam1.getCameraCenter();
            final var estimatedCenter2 = estCam2.getCameraCenter();

            final var estimatedIntrinsic1 = estCam1.getIntrinsicParameters();
            final var estimatedIntrinsic2 = estCam2.getIntrinsicParameters();

            final var estimatedRotation1 = estCam1.getCameraRotation();
            final var estimatedRotation2 = estCam2.getCameraRotation();

            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);

            // check cameras are correct
            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);

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

            if (!estimatedRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPoints; i++) {
                if (!points3D.get(i).equals(reconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(reconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            // cancel
            assertFalse(reconstructor.isCancelled());
            reconstructor.cancel();

            assertTrue(reconstructor.isCancelled());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testPlanarPointsDIAC() throws InvalidPairOfCamerasException, CameraException, AlgebraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new KnownBaselineTwoViewsSparseReconstructorConfiguration();
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
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var baseline = center1.distanceTo(center2);
            configuration.setBaseline(baseline);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras and laying in
            // a plane

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

            final var listener = new KnownBaselineTwoViewsSparseReconstructorListener() {

                @Override
                public boolean hasMoreViewsAvailable(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        int viewId, List<Sample2D> samples) {

                    samples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final int viewId1, final int viewId2, final List<MatchedSamples> matches) {
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
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    KnownBaselineTwoViewsSparseReconstructorTest.this.estimatedFundamentalMatrix =
                            estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId1, final int viewId2, final EstimatedCamera camera1,
                        final EstimatedCamera camera2) {
                    estimatedCamera1 = camera1;
                    estimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    reconstructedPoints = points;
                }

                @Override
                public void onStart(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new KnownBaselineTwoViewsSparseReconstructor(configuration, listener);

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

            // check that estimated fundamental matrix is correct
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
            final var estCam1 = this.estimatedCamera1.getCamera();
            final var estCam2 = this.estimatedCamera2.getCamera();

            estCam1.decompose();
            estCam2.decompose();

            final var reconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints; i++) {
                reconstructedPoints3D.add(reconstructedPoints.get(i).getPoint());
            }

            // check that most of the points are in front of both cameras
            var valid = 0;
            var invalid = 0;
            for (var i = 0; i < numPoints; i++) {
                if (reconstructedPoints.get(i).isInlier()) {
                    final var p = reconstructedPoints3D.get(i);
                    assertTrue(estCam1.isPointInFrontOfCamera(p));
                    assertTrue(estCam2.isPointInFrontOfCamera(p));
                    valid++;
                } else {
                    invalid++;
                }
            }

            if (valid < invalid) {
                continue;
            }

            final var estimatedCenter1 = estCam1.getCameraCenter();
            final var estimatedCenter2 = estCam2.getCameraCenter();

            final var estimatedRotation1 = estCam1.getCameraRotation();

            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);

            // check cameras are correct
            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // cancel
            assertFalse(reconstructor.isCancelled());
            reconstructor.cancel();

            assertTrue(reconstructor.isCancelled());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testPlanarPointsDAQAndEssentialZeroPrincipalPoint() throws InvalidPairOfCamerasException, CameraException,
            AlgebraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new KnownBaselineTwoViewsSparseReconstructorConfiguration();
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

            final var baseline = center1.distanceTo(center2);
            configuration.setBaseline(baseline);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras and laying in
            // a plane

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

            final var listener = new KnownBaselineTwoViewsSparseReconstructorListener() {

                @Override
                public boolean hasMoreViewsAvailable(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        int viewId, final List<Sample2D> samples) {

                    samples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final int viewId1, final int viewId2, final List<MatchedSamples> matches) {
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
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    KnownBaselineTwoViewsSparseReconstructorTest.this.estimatedFundamentalMatrix =
                            estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId1, final int viewId2, final EstimatedCamera camera1,
                        final EstimatedCamera camera2) {
                    estimatedCamera1 = camera1;
                    estimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    reconstructedPoints = points;
                }

                @Override
                public void onStart(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new KnownBaselineTwoViewsSparseReconstructor(configuration, listener);

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

            // check that estimated fundamental matrix is correct
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
            final var estCam1 = this.estimatedCamera1.getCamera();
            final var estCam2 = this.estimatedCamera2.getCamera();

            estCam1.decompose();
            estCam2.decompose();

            final var reconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints; i++) {
                reconstructedPoints3D.add(reconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            var allInFront = true;
            for (var i = 0; i < numPoints; i++) {
                final var p = reconstructedPoints3D.get(i);
                if (!estCam1.isPointInFrontOfCamera(p) || !estCam2.isPointInFrontOfCamera(p)) {
                    allInFront = false;
                    break;
                }
                assertTrue(estCam1.isPointInFrontOfCamera(p));
                assertTrue(estCam2.isPointInFrontOfCamera(p));
            }

            if (!allInFront) {
                continue;
            }

            final var estimatedCenter1 = estCam1.getCameraCenter();
            final var estimatedCenter2 = estCam2.getCameraCenter();

            final var estimatedRotation1 = estCam1.getCameraRotation();

            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);

            // check cameras are correct
            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPoints; i++) {
                if (!points3D.get(i).equals(reconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(reconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            // cancel
            assertFalse(reconstructor.isCancelled());
            reconstructor.cancel();

            assertTrue(reconstructor.isCancelled());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testPlanarPointsDAQAndEssential() throws InvalidPairOfCamerasException, CameraException, AlgebraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new KnownBaselineTwoViewsSparseReconstructorConfiguration();
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

            final var baseline = center1.distanceTo(center2);
            configuration.setBaseline(baseline);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras and laying in
            // a plane

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

            if (failedIter) {
                continue;
            }

            final var listener = new KnownBaselineTwoViewsSparseReconstructorListener() {

                @Override
                public boolean hasMoreViewsAvailable(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {

                    samples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final int viewId1, final int viewId2, final List<MatchedSamples> matches) {
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
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    KnownBaselineTwoViewsSparseReconstructorTest.this.estimatedFundamentalMatrix =
                            estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId1, final int viewId2, final EstimatedCamera camera1,
                        final EstimatedCamera camera2) {
                    estimatedCamera1 = camera1;
                    estimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    reconstructedPoints = points;
                }

                @Override
                public void onStart(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    KnownBaselineTwoViewsSparseReconstructorTest.this.failed = true;
                }
            };

            final var reconstructor = new KnownBaselineTwoViewsSparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(this.failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            if (!finished) {
                continue;
            }
            assertTrue(started);
            assertFalse(cancelled);
            assertFalse(this.failed);
            assertTrue(reconstructor.isFinished());

            // check that estimated fundamental matrix is correct
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
            final var estCam1 = this.estimatedCamera1.getCamera();
            final var estCam2 = this.estimatedCamera2.getCamera();

            estCam1.decompose();
            estCam2.decompose();

            final var reconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints; i++) {
                reconstructedPoints3D.add(reconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints; i++) {
                final var p = reconstructedPoints3D.get(i);
                assertTrue(estCam1.isPointInFrontOfCamera(p));
                assertTrue(estCam2.isPointInFrontOfCamera(p));
            }

            final var estimatedCenter1 = estCam1.getCameraCenter();
            final var estimatedCenter2 = estCam2.getCameraCenter();

            final var estimatedRotation1 = estCam1.getCameraRotation();

            final var estimatedBaseline = estimatedCenter1.distanceTo(estimatedCenter2);

            // check cameras are correct
            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct
            var validPoints = true;
            for (var i = 0; i < numPoints; i++) {
                if (!points3D.get(i).equals(reconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(reconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            // cancel
            assertFalse(reconstructor.isCancelled());
            reconstructor.cancel();

            assertTrue(reconstructor.isCancelled());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testPlanarPointsDAQ() throws InvalidPairOfCamerasException, CameraException, AlgebraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var configuration = new KnownBaselineTwoViewsSparseReconstructorConfiguration();
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

            final var baseline = center1.distanceTo(center2);
            configuration.setBaseline(baseline);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras and laying in
            // a plane

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

            final var listener = new KnownBaselineTwoViewsSparseReconstructorListener() {

                @Override
                public boolean hasMoreViewsAvailable(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {

                    samples.clear();

                    Sample2D sample;
                    if (viewCount == 0) {
                        // first view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    } else {
                        // second view
                        for (var i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId, final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final int viewId1, final int viewId2, final List<MatchedSamples> matches) {
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
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    KnownBaselineTwoViewsSparseReconstructorTest.this.estimatedFundamentalMatrix =
                            estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final int viewId1, final int viewId2, final EstimatedCamera camera1,
                        final EstimatedCamera camera2) {
                    estimatedCamera1 = camera1;
                    estimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        final KnownBaselineTwoViewsSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    reconstructedPoints = points;
                }

                @Override
                public void onStart(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final KnownBaselineTwoViewsSparseReconstructor reconstructor) {
                    KnownBaselineTwoViewsSparseReconstructorTest.this.failed = true;
                }
            };

            final var reconstructor = new KnownBaselineTwoViewsSparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(this.failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            if (!finished) {
                continue;
            }
            assertTrue(started);
            assertFalse(cancelled);
            assertFalse(this.failed);
            assertTrue(reconstructor.isFinished());

            // check that estimated fundamental matrix is correct
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
            final var estCam1 = this.estimatedCamera1.getCamera();
            final var estCam2 = this.estimatedCamera2.getCamera();

            estCam1.decompose();
            estCam2.decompose();

            // cancel
            assertFalse(reconstructor.isCancelled());
            reconstructor.cancel();

            assertTrue(reconstructor.isCancelled());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private void reset() {
        viewCount = 0;
        estimatedFundamentalMatrix = null;
        estimatedCamera1 = estimatedCamera2 = null;
        reconstructedPoints = null;
        started = finished = cancelled = failed = false;
    }
}
