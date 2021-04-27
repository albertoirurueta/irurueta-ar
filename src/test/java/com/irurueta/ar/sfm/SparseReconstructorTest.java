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
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class SparseReconstructorTest {

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

    private int mViewCount = 0;
    private EstimatedFundamentalMatrix mEstimatedFundamentalMatrix;
    private EstimatedFundamentalMatrix mEstimatedFundamentalMatrix2;
    private EstimatedCamera mEstimatedMetricCamera1;
    private EstimatedCamera mEstimatedMetricCamera2;
    private EstimatedCamera mEstimatedMetricCamera3;
    private EstimatedCamera mEstimatedEuclideanCamera1;
    private EstimatedCamera mEstimatedEuclideanCamera2;
    private EstimatedCamera mEstimatedEuclideanCamera3;
    private List<ReconstructedPoint3D> mMetricReconstructedPoints;
    private List<ReconstructedPoint3D> mEuclideanReconstructedPoints;

    private double mScale;
    private double mScale2;

    private boolean mStarted;
    private boolean mFinished;
    private boolean mFailed;
    private boolean mCancelled;

    @Before
    public void setUp() {
        mViewCount = 0;
        mEstimatedFundamentalMatrix = mEstimatedFundamentalMatrix2 = null;
        mEstimatedMetricCamera1 = mEstimatedMetricCamera2 =
                mEstimatedMetricCamera3 = null;
        mEstimatedEuclideanCamera1 = mEstimatedEuclideanCamera2
                = mEstimatedEuclideanCamera3 = null;
        mMetricReconstructedPoints = null;
        mEuclideanReconstructedPoints = null;
        mStarted = mFinished = mFailed = mCancelled = false;
    }

    @Test
    public void testConstructor() {
        assertEquals(SparseReconstructor.MIN_NUMBER_OF_VIEWS, 2);

        final SparseReconstructorConfiguration configuration =
                new SparseReconstructorConfiguration();
        final SparseReconstructorListener listener =
                new SparseReconstructorListener() {
                    @Override
                    public boolean hasMoreViewsAvailable(
                            final SparseReconstructor reconstructor) {
                        return false;
                    }

                    @Override
                    public void onRequestSamples(
                            final SparseReconstructor reconstructor,
                            final int previousViewId, final int currentViewId,
                            final List<Sample2D> previousViewTrackedSamples,
                            final List<Sample2D> currentViewTrackedSamples,
                            final List<Sample2D> currentViewNewlySpawnedSamples) {
                    }

                    @Override
                    public void onSamplesAccepted(
                            final SparseReconstructor reconstructor, final int viewId,
                            final List<Sample2D> previousViewTrackedSamples,
                            final List<Sample2D> currentViewTrackedSamples) {
                    }

                    @Override
                    public void onSamplesRejected(
                            final SparseReconstructor reconstructor, final int viewId,
                            final List<Sample2D> previousViewTrackedSamples,
                            final List<Sample2D> currentViewTrackedSamples) {
                    }

                    @Override
                    public void onRequestMatches(
                            final SparseReconstructor reconstructor,
                            final List<Sample2D> allPreviousViewSamples,
                            final List<Sample2D> previousViewTrackedSamples,
                            final List<Sample2D> currentViewTrackedSamples,
                            final int previousViewId, final int currentViewId,
                            final List<MatchedSamples> matches) {
                    }

                    @Override
                    public void onFundamentalMatrixEstimated(
                            final SparseReconstructor reconstructor,
                            final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    }

                    @Override
                    public void onMetricCameraEstimated(
                            final SparseReconstructor reconstructor, final int previousViewId,
                            final int currentViewId, final EstimatedCamera previousCamera,
                            final EstimatedCamera currentCamera) {
                    }

                    @Override
                    public void onMetricReconstructedPointsEstimated(
                            final SparseReconstructor reconstructor,
                            final List<MatchedSamples> matches,
                            final List<ReconstructedPoint3D> points) {
                    }

                    @Override
                    public void onEuclideanCameraEstimated(
                            final SparseReconstructor reconstructor, final int previousViewId,
                            final int currentViewId, final double scale,
                            final EstimatedCamera previousCamera,
                            final EstimatedCamera currentCamera) {
                    }

                    @Override
                    public void onEuclideanReconstructedPointsEstimated(
                            final SparseReconstructor reconstructor, final double scale,
                            final List<ReconstructedPoint3D> points) {
                    }

                    @Override
                    public void onStart(
                            final SparseReconstructor reconstructor) {
                    }

                    @Override
                    public void onFinish(
                            final SparseReconstructor reconstructor) {
                    }

                    @Override
                    public void onCancel(
                            final SparseReconstructor reconstructor) {
                    }

                    @Override
                    public void onFail(
                            final SparseReconstructor reconstructor) {
                    }
                };

        // constructor with listener
        SparseReconstructor reconstructor = new SparseReconstructor(listener);

        // check default values
        assertNotNull(reconstructor.getConfiguration());
        assertSame(reconstructor.getListener(), listener);
        assertFalse(reconstructor.isRunning());
        assertFalse(reconstructor.isCancelled());
        assertFalse(reconstructor.hasFailed());
        assertFalse(reconstructor.isFinished());
        assertEquals(reconstructor.getViewCount(), 0);
        assertNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
        assertNull(reconstructor.getCurrentMetricEstimatedCamera());
        assertNull(reconstructor.getPreviousMetricEstimatedCamera());
        assertNull(reconstructor.getCurrentEuclideanEstimatedCamera());
        assertNull(reconstructor.getPreviousEuclideanEstimatedCamera());
        assertNull(reconstructor.getActiveMetricReconstructedPoints());
        assertNull(reconstructor.getActiveEuclideanReconstructedPoints());
        assertEquals(reconstructor.getCurrentScale(), BaseSparseReconstructor.DEFAULT_SCALE, 0.0);
        assertNull(reconstructor.getPreviousViewTrackedSamples());
        assertNull(reconstructor.getCurrentViewTrackedSamples());
        assertNull(reconstructor.getCurrentViewNewlySpawnedSamples());
        assertTrue(reconstructor.isFirstView());
        assertFalse(reconstructor.isSecondView());
        assertFalse(reconstructor.isAdditionalView());

        // constructor with configuration and listener
        reconstructor = new SparseReconstructor(configuration, listener);

        // check default values
        assertSame(reconstructor.getConfiguration(), configuration);
        assertSame(reconstructor.getListener(), listener);
        assertFalse(reconstructor.isRunning());
        assertFalse(reconstructor.isCancelled());
        assertFalse(reconstructor.hasFailed());
        assertFalse(reconstructor.isFinished());
        assertEquals(reconstructor.getViewCount(), 0);
        assertNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
        assertNull(reconstructor.getCurrentMetricEstimatedCamera());
        assertNull(reconstructor.getPreviousMetricEstimatedCamera());
        assertNull(reconstructor.getCurrentEuclideanEstimatedCamera());
        assertNull(reconstructor.getPreviousEuclideanEstimatedCamera());
        assertNull(reconstructor.getActiveMetricReconstructedPoints());
        assertNull(reconstructor.getActiveEuclideanReconstructedPoints());
        assertEquals(reconstructor.getCurrentScale(), BaseSparseReconstructor.DEFAULT_SCALE, 0.0);
        assertNull(reconstructor.getPreviousViewTrackedSamples());
        assertNull(reconstructor.getCurrentViewTrackedSamples());
        assertNull(reconstructor.getCurrentViewNewlySpawnedSamples());
        assertTrue(reconstructor.isFirstView());
        assertFalse(reconstructor.isSecondView());
        assertFalse(reconstructor.isAdditionalView());
    }

    @Test
    public void testGeneralPointsEssentialTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final SparseReconstructorConfiguration configuration =
                    new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getInitialCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);

            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final List<InhomogeneousPoint3D> points3D1 = new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1;
            boolean front2;
            for (int i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front1);
                //noinspection ConstantConditions
                assertTrue(front2);

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            Point2D projectedPoint2b;
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
                // generate new spawned point in front of camera 2
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

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

                // check that 3D point is in front of camera 2
                //noinspection ConstantConditions
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            final SparseReconstructorListener listener =
                    new SparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                final SparseReconstructor reconstructor) {
                            return mViewCount < 2;
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
                            if (mViewCount == 0) {
                                // first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else {
                                // second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                // spawned samples
                                for (int i = 0; i < numPoints2; i++) {
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
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                final SparseReconstructor reconstructor, final int viewId,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(
                                final SparseReconstructor reconstructor,
                                final List<Sample2D> allPreviousViewSamples,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples,
                                final int previousViewId, final int currentViewId,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints1; i++) {
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
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedMetricCamera1 = previousCamera;
                            mEstimatedMetricCamera2 = currentCamera;
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final List<MatchedSamples> matches,
                                final List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedEuclideanCamera1 = previousCamera;
                            mEstimatedEuclideanCamera2 = currentCamera;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final double scale, final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public void onStart(
                                final SparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final SparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final SparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final SparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final SparseReconstructor reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera1);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            final PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            assertSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();

            assertSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            final List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (int i = 0; i < numPoints1; i++) {
                final Point3D p = metricReconstructedPoints3D.get(i);
                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));
            }

            final Point3D estimatedCenter1 = estimatedMetricCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedMetricCamera2.getCameraCenter();

            // transform points and cameras to account for scale change
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            final double scale = baseline / estimatedBaseline;
            assertEquals(mScale, 1.0, 0.0);

            final MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);

            final PinholeCamera scaledCamera1 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera1);
            final PinholeCamera scaledCamera2 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera2);

            final List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();

            final Point3D scaledCenter1 = scaledCamera1.getCameraCenter();
            final Point3D scaledCenter2 = scaledCamera2.getCameraCenter();

            final PinholeCameraIntrinsicParameters scaledIntrinsic1 =
                    scaledCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters scaledIntrinsic2 =
                    scaledCamera2.getIntrinsicParameters();

            final Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
            final Rotation3D scaledRotation2 = scaledCamera2.getCameraRotation();

            final double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

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

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i),
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
    public void testGeneralPointsDIACTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final SparseReconstructorConfiguration configuration =
                    new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_DIAC,
                    MAX_FOCAL_LENGTH_DIAC);
            final double aspectRatio = configuration.getInitialCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPointX = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);
            final double principalPointY = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, principalPointX, principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);
            configuration.setInitialCamerasAspectRatio(aspectRatio);

            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_DIAC, MAX_CAMERA_SEPARATION_DIAC);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final List<InhomogeneousPoint3D> points3D1 =
                    new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1;
            boolean front2;
            boolean maxTriesReached = false;
            for (int i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_DIAC,
                            MAX_LAMBDA_DIAC);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_DIAC,
                            MAX_LAMBDA_DIAC);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_DIAC,
                            MAX_LAMBDA_DIAC);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front1);
                //noinspection ConstantConditions
                assertTrue(front2);

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
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
                // generate new spawned point in front of camera 2
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_DIAC,
                            MAX_LAMBDA_DIAC);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_DIAC,
                            MAX_LAMBDA_DIAC);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_DIAC,
                            MAX_LAMBDA_DIAC);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            final SparseReconstructorListener listener =
                    new SparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                final SparseReconstructor reconstructor) {
                            return mViewCount < 2;
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
                            if (mViewCount == 0) {
                                // first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else {
                                // second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                // spawned samples
                                for (int i = 0; i < numPoints2; i++) {
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
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                final SparseReconstructor reconstructor, final int viewId,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(
                                final SparseReconstructor reconstructor,
                                final List<Sample2D> allPreviousViewSamples,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples,
                                final int previousViewId, final int currentViewId,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints1; i++) {
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
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedMetricCamera1 = previousCamera;
                            mEstimatedMetricCamera2 = currentCamera;
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedEuclideanCamera1 = previousCamera;
                            mEstimatedEuclideanCamera2 = currentCamera;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final double scale, final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public void onStart(
                                final SparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final SparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final SparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final SparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final SparseReconstructor reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            if (!mFinished) {
                continue;
            }

            // check correctness
            assertTrue(mStarted);
            //noinspection ConstantConditions
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera1);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            final PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            assertSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();

            assertSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            final List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
            }

            // check that most of the points are in front of both cameras
            int valid = 0;
            int invalid = 0;
            for (int i = 0; i < numPoints1; i++) {
                if (mMetricReconstructedPoints.get(i).isInlier()) {
                    final Point3D p = metricReconstructedPoints3D.get(i);
                    assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                    assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));
                    valid++;
                } else {
                    invalid++;
                }
            }

            assertTrue(valid >= invalid);

            final Point3D estimatedCenter1 = estimatedMetricCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedMetricCamera2.getCameraCenter();

            // transform points and cameras to account for scale change
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            final double scale = baseline / estimatedBaseline;
            assertEquals(mScale, 1.0, 0.0);

            final MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);

            final PinholeCamera scaledCamera1 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera1);
            final PinholeCamera scaledCamera2 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera2);

            final List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();

            final Point3D scaledCenter1 = new InhomogeneousPoint3D(
                    scaledCamera1.getCameraCenter());
            final Point3D scaledCenter2 = new InhomogeneousPoint3D(
                    scaledCamera2.getCameraCenter());

            final PinholeCameraIntrinsicParameters scaledIntrinsic1 =
                    scaledCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters scaledIntrinsic2 =
                    scaledCamera2.getIntrinsicParameters();

            final Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
            final Rotation3D scaledRotation2 = scaledCamera2.getCameraRotation();

            final double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

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

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i),
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
    public void testGeneralPointsDAQAndEssentialZeroPrincipalPointTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final SparseReconstructorConfiguration configuration =
                    new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getInitialCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPointX = 0.0;
            final double principalPointY = 0.0;

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, principalPointX, principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final List<InhomogeneousPoint3D> points3D1 = new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean leftFront;
            boolean rightFront;
            boolean maxTriesReached = false;
            for (int i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(leftFront);
                //noinspection ConstantConditions
                assertTrue(rightFront);

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            Point2D projectedPoint2b;
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(rightFront);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            if (maxTriesReached) {
                continue;
            }

            final SparseReconstructorListener listener =
                    new SparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                final SparseReconstructor reconstructor) {
                            return mViewCount < 2;
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
                            if (mViewCount == 0) {
                                // first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else {
                                // second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                // spawned samples
                                for (int i = 0; i < numPoints2; i++) {
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
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                final SparseReconstructor reconstructor, final int viewId,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(
                                final SparseReconstructor reconstructor,
                                final List<Sample2D> allPreviousViewSamples,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples,
                                final int previousViewId, final int currentViewId,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints1; i++) {
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
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedMetricCamera1 = previousCamera;
                            mEstimatedMetricCamera2 = currentCamera;
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedEuclideanCamera1 = previousCamera;
                            mEstimatedEuclideanCamera2 = currentCamera;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final double scale, final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public void onStart(
                                final SparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final SparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final SparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final SparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final SparseReconstructor reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera1);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            final PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            assertSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();

            assertSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            final List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (int i = 0; i < numPoints1; i++) {
                final Point3D p = metricReconstructedPoints3D.get(i);
                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));
            }

            final Point3D estimatedCenter1 = estimatedMetricCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedMetricCamera2.getCameraCenter();

            // transform points and cameras to account for scale change
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            final double scale = baseline / estimatedBaseline;
            assertEquals(mScale, 1.0, 0.0);

            final MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);

            final PinholeCamera scaledCamera1 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera1);
            final PinholeCamera scaledCamera2 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera2);

            final List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();

            final Point3D scaledCenter1 = new InhomogeneousPoint3D(
                    scaledCamera1.getCameraCenter());
            final Point3D scaledCenter2 = new InhomogeneousPoint3D(
                    scaledCamera2.getCameraCenter());

            final PinholeCameraIntrinsicParameters scaledIntrinsic1 =
                    scaledCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters scaledIntrinsic2 =
                    scaledCamera2.getIntrinsicParameters();

            final Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
            final Rotation3D scaledRotation2 = scaledCamera2.getCameraRotation();

            final double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

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

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i),
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
    public void testGeneralPointsDAQAndEssentialTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final SparseReconstructorConfiguration configuration =
                    new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getInitialCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPointX = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_ESSENTIAL,
                    MAX_PRINCIPAL_POINT_ESSENTIAL);
            final double principalPointY = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_ESSENTIAL,
                    MAX_PRINCIPAL_POINT_ESSENTIAL);

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, principalPointX, principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final List<InhomogeneousPoint3D> points3D1 = new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean leftFront;
            boolean rightFront;
            boolean maxTriesReached = false;
            for (int i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(leftFront);
                //noinspection ConstantConditions
                assertTrue(rightFront);

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
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(rightFront);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            if (maxTriesReached) {
                continue;
            }

            final SparseReconstructorListener listener =
                    new SparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                final SparseReconstructor reconstructor) {
                            return mViewCount < 2;
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
                            if (mViewCount == 0) {
                                // first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else {
                                // second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                // spawned samples
                                for (int i = 0; i < numPoints2; i++) {
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
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                final SparseReconstructor reconstructor, final int viewId,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(
                                final SparseReconstructor reconstructor,
                                final List<Sample2D> allPreviousViewSamples,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples,
                                final int previousViewId, final int currentViewId,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints1; i++) {
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
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedMetricCamera1 = previousCamera;
                            mEstimatedMetricCamera2 = currentCamera;
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedEuclideanCamera1 = previousCamera;
                            mEstimatedEuclideanCamera2 = currentCamera;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final double scale, final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public void onStart(
                                final SparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final SparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final SparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final SparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final SparseReconstructor reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera1);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            final PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            assertSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();

            assertSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            final List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (int i = 0; i < numPoints1; i++) {
                final Point3D p = metricReconstructedPoints3D.get(i);
                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));
            }

            final Point3D estimatedCenter1 = estimatedMetricCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedMetricCamera2.getCameraCenter();

            // transform points and cameras to account for scale change
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            final double scale = baseline / estimatedBaseline;
            assertEquals(mScale, 1.0, 0.0);

            final MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);

            final PinholeCamera scaledCamera1 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera1);
            final PinholeCamera scaledCamera2 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera2);

            final List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();

            final Point3D scaledCenter1 = new InhomogeneousPoint3D(
                    scaledCamera1.getCameraCenter());
            final Point3D scaledCenter2 = new InhomogeneousPoint3D(
                    scaledCamera2.getCameraCenter());

            final PinholeCameraIntrinsicParameters scaledIntrinsic1 =
                    scaledCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters scaledIntrinsic2 =
                    scaledCamera2.getIntrinsicParameters();

            final Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
            final Rotation3D scaledRotation2 = scaledCamera2.getCameraRotation();

            final double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

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

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i),
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
    public void testGeneralPointsDAQTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException, com.irurueta.geometry.estimators.LockedException,
            RobustEstimatorException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final SparseReconstructorConfiguration configuration =
                    new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getInitialCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPointX = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_ESSENTIAL,
                    MAX_PRINCIPAL_POINT_ESSENTIAL);
            final double principalPointY = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_ESSENTIAL,
                    MAX_PRINCIPAL_POINT_ESSENTIAL);

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, principalPointX, principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final List<Point3D> points3D1 = new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean leftFront;
            boolean rightFront;
            boolean maxTriesReached = false;
            for (int i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(leftFront);
                //noinspection ConstantConditions
                assertTrue(rightFront);

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            Point2D projectedPoint2b;
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL,
                            MAX_LAMBDA_ESSENTIAL);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(rightFront);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            final SparseReconstructorListener listener =
                    new SparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                final SparseReconstructor reconstructor) {
                            return mViewCount < 2;
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
                            if (mViewCount == 0) {
                                // first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else {
                                // second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                // spawned samples
                                for (int i = 0; i < numPoints2; i++) {
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
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                final SparseReconstructor reconstructor, final int viewId,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(
                                final SparseReconstructor reconstructor,
                                final List<Sample2D> allPreviousViewSamples,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples,
                                final int previousViewId, final int currentViewId,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints1; i++) {
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
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedMetricCamera1 = previousCamera;
                            mEstimatedMetricCamera2 = currentCamera;
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedEuclideanCamera1 = previousCamera;
                            mEstimatedEuclideanCamera2 = currentCamera;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final double scale, final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public void onStart(
                                final SparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final SparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final SparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final SparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            if (maxTriesReached) {
                continue;
            }

            final SparseReconstructor reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera1);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            final PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            assertSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);

            assertSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            final List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
            }

            final MetricTransformation3DRobustEstimator transformationEstimator =
                    MetricTransformation3DRobustEstimator.create(
                            metricReconstructedPoints3D, points3D1,
                            RobustEstimatorMethod.LMedS);

            final MetricTransformation3D transformation =
                    transformationEstimator.estimate();

            final PinholeCamera transformedCamera1 =
                    transformation.transformAndReturnNew(estimatedMetricCamera1);
            final PinholeCamera transformedCamera2 =
                    transformation.transformAndReturnNew(estimatedMetricCamera2);

            // check cameras intrinsics are correct (rotation, center and points
            // might contain large errors and for that reason we do not checked)
            transformedCamera1.decompose();
            transformedCamera2.decompose();

            final PinholeCameraIntrinsicParameters transformedIntrinsic1 =
                    transformedCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters transformedIntrinsic2 =
                    transformedCamera2.getIntrinsicParameters();

            assertEquals(transformedIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(transformedIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(transformedIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(transformedIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(transformedIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            assertEquals(transformedIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(transformedIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(transformedIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(transformedIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(transformedIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testPlanarPointsEssentialTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final SparseReconstructorConfiguration configuration =
                    new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getInitialCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPointX = 0.0;
            final double principalPointY = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, principalPointX, principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);

            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);

            // create 3D points laying in front of both cameras and laying in a plane

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            final double[] principalAxis1 = camera1.getPrincipalAxisArray();
            final double[] principalAxis2 = camera2.getPrincipalAxisArray();
            final double[] avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2),
                    0.5);

            final Plane plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();

            final double planeA = plane.getA();
            final double planeB = plane.getB();
            final double planeC = plane.getC();
            final double planeD = plane.getD();

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            final List<HomogeneousPoint3D> points3D1 = new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1;
            boolean front2;
            for (int i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final double homW = 1.0;
                    final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR,
                                MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) /
                                planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR,
                                MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) /
                                planeA;
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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front1);
                //noinspection ConstantConditions
                assertTrue(front2);

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            Point2D projectedPoint2b;
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final double homW = 1.0;
                    final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR,
                                MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) /
                                planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR,
                                MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) /
                                planeA;
                    }

                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                    assertTrue(plane.isLocus(point3D));

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front2);

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            final SparseReconstructorListener listener =
                    new SparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                final SparseReconstructor reconstructor) {
                            return mViewCount < 2;
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
                            if (mViewCount == 0) {
                                // first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else {
                                // second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                // spawned samples
                                for (int i = 0; i < numPoints2; i++) {
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
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                final SparseReconstructor reconstructor, final int viewId,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(
                                final SparseReconstructor reconstructor,
                                final List<Sample2D> allPreviousViewSamples,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples,
                                final int previousViewId, final int currentViewId,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints1; i++) {
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
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedMetricCamera1 = previousCamera;
                            mEstimatedMetricCamera2 = currentCamera;
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final List<MatchedSamples> matches,
                                final List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedEuclideanCamera1 = previousCamera;
                            mEstimatedEuclideanCamera2 = currentCamera;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final double scale, final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public void onStart(
                                final SparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final SparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final SparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final SparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final SparseReconstructor reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            if (!mFinished) {
                continue;
            }
            assertTrue(mStarted);
            //noinspection ConstantConditions
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertTrue(reconstructor.getViewCount() > 0);
            if (reconstructor.getCurrentMetricEstimatedCamera() == null) {
                continue;
            }
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera1);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
            if (mEstimatedFundamentalMatrix == null || mEstimatedFundamentalMatrix.getFundamentalMatrix() == null) {
                continue;
            }

            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            final PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            assertSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);

            assertSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();

            final List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (int i = 0; i < numPoints1; i++) {
                final Point3D p = metricReconstructedPoints3D.get(i);
                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));
            }

            final Point3D estimatedCenter1 = estimatedMetricCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedMetricCamera2.getCameraCenter();

            // transform points and cameras to account for scale change
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            final double scale = baseline / estimatedBaseline;
            assertEquals(mScale, 1.0, 0.0);


            final MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);

            final PinholeCamera scaledCamera1 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera1);
            final PinholeCamera scaledCamera2 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera2);

            final List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();

            final Point3D scaledCenter1 = scaledCamera1.getCameraCenter();
            final Point3D scaledCenter2 = scaledCamera2.getCameraCenter();

            final PinholeCameraIntrinsicParameters scaledIntrinsic1 =
                    scaledCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters scaledIntrinsic2 =
                    scaledCamera2.getIntrinsicParameters();

            final Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
            final Rotation3D scaledRotation2 = scaledCamera2.getCameraRotation();

            final double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

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

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            if (!scaledRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), LARGE_ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i),
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
    public void testPlanarPointsDIACTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final SparseReconstructorConfiguration configuration =
                    new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getInitialCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPointX = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);
            final double principalPointY = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, principalPointX, principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);
            configuration.setInitialCamerasAspectRatio(aspectRatio);

            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);

            // create 3D points laying in front of both cameras and laying in a plane

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            final double[] principalAxis1 = camera1.getPrincipalAxisArray();
            final double[] principalAxis2 = camera2.getPrincipalAxisArray();
            final double[] avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2),
                    0.5);

            final Plane plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();

            final double planeA = plane.getA();
            final double planeB = plane.getB();
            final double planeC = plane.getC();
            final double planeD = plane.getD();

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1;
            boolean front2;
            for (int i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final double homW = 1.0;
                    final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR,
                                MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) /
                                planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR,
                                MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) /
                                planeA;
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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front1);
                //noinspection ConstantConditions
                assertTrue(front2);

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            Point2D projectedPoint2b;
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final double homW = 1.0;
                    final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR,
                                MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) /
                                planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR,
                                MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) /
                                planeA;
                    }

                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                    assertTrue(plane.isLocus(point3D));

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front2);

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            final SparseReconstructorListener listener =
                    new SparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                final SparseReconstructor reconstructor) {
                            return mViewCount < 2;
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
                            if (mViewCount == 0) {
                                // first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else {
                                // second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                // spawned samples
                                for (int i = 0; i < numPoints2; i++) {
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
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                final SparseReconstructor reconstructor, final int viewId,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(
                                final SparseReconstructor reconstructor,
                                final List<Sample2D> allPreviousViewSamples,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples,
                                final int previousViewId, final int currentViewId,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints1; i++) {
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
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedMetricCamera1 = previousCamera;
                            mEstimatedMetricCamera2 = currentCamera;
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedEuclideanCamera1 = previousCamera;
                            mEstimatedEuclideanCamera2 = currentCamera;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final double scale, final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public void onStart(
                                final SparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final SparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final SparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final SparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final SparseReconstructor reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            if (!mFinished || mFailed) {
                continue;
            }
            assertTrue(mStarted);
            //noinspection ConstantConditions
            assertTrue(mFinished);
            assertFalse(mCancelled);
            //noinspection ConstantConditions
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertTrue(reconstructor.getViewCount() > 0);
            if (reconstructor.getCurrentMetricEstimatedCamera() == null) {
                continue;
            }
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera1);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());


            // check that estimated fundamental matrix is correct
            if (mEstimatedFundamentalMatrix == null || mEstimatedFundamentalMatrix.getFundamentalMatrix() == null) {
                continue;
            }

            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), LARGE_ABSOLUTE_ERROR) &&
                    !fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), LARGE_ABSOLUTE_ERROR) ||
                    fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), LARGE_ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            final PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            assertSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);

            assertSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();

            final List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
            }

            // check that most of the points are in front of both cameras
            int valid = 0, invalid = 0;
            for (int i = 0; i < numPoints1; i++) {
                if (mMetricReconstructedPoints.get(i).isInlier()) {
                    final Point3D p = metricReconstructedPoints3D.get(i);
                    assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                    assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));
                    valid++;
                } else {
                    invalid++;
                }
            }

            if (valid < invalid) {
                continue;
            }

            final Point3D estimatedCenter1 = estimatedMetricCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedMetricCamera2.getCameraCenter();

            // transform points and cameras to account for scale change
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            final double scale = baseline / estimatedBaseline;
            assertEquals(mScale, 1.0, 0.0);

            final MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);

            final PinholeCamera scaledCamera1 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera1);
            final PinholeCamera scaledCamera2 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera2);

            scaledCamera1.decompose();
            scaledCamera2.decompose();

            final Point3D scaledCenter1 = new InhomogeneousPoint3D(
                    scaledCamera1.getCameraCenter());
            final Point3D scaledCenter2 = new InhomogeneousPoint3D(
                    scaledCamera2.getCameraCenter());

            final Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();

            final double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

            // check cameras are correct
            if (Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testPlanarPointsDAQAndEssentialZeroPrincipalPointTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final SparseReconstructorConfiguration configuration =
                    new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getInitialCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPointX = 0.0;
            final double principalPointY = 0.0;

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, principalPointX, principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);

            // create 3D points laying in front of both cameras and laying in a plane

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            final double[] principalAxis1 = camera1.getPrincipalAxisArray();
            final double[] principalAxis2 = camera2.getPrincipalAxisArray();
            final double[] avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2),
                    0.5);

            final Plane plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();

            final double planeA = plane.getA();
            final double planeB = plane.getB();
            final double planeC = plane.getC();
            final double planeD = plane.getD();

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            final List<HomogeneousPoint3D> points3D1 = new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1;
            boolean front2;
            for (int i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final double homW = 1.0;
                    final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR,
                                MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) /
                                planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR,
                                MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) /
                                planeA;
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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front1);
                //noinspection ConstantConditions
                assertTrue(front2);

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            Point2D projectedPoint2b;
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final double homW = 1.0;
                    final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR,
                                MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) /
                                planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR,
                                MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) /
                                planeA;
                    }

                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                    assertTrue(plane.isLocus(point3D));

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front2);

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            final SparseReconstructorListener listener =
                    new SparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                final SparseReconstructor reconstructor) {
                            return mViewCount < 2;
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
                            if (mViewCount == 0) {
                                // first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else {
                                // second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                // spawned samples
                                for (int i = 0; i < numPoints2; i++) {
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
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                final SparseReconstructor reconstructor, final int viewId,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(
                                final SparseReconstructor reconstructor,
                                final List<Sample2D> allPreviousViewSamples,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples,
                                final int previousViewId, final int currentViewId,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints1; i++) {
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
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedMetricCamera1 = previousCamera;
                            mEstimatedMetricCamera2 = currentCamera;
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedEuclideanCamera1 = previousCamera;
                            mEstimatedEuclideanCamera2 = currentCamera;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final double scale, final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public void onStart(
                                final SparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final SparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final SparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final SparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final SparseReconstructor reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            if (!mFinished || mFailed) {
                continue;
            }
            assertTrue(mStarted);
            //noinspection ConstantConditions
            assertTrue(mFinished);
            assertFalse(mCancelled);
            //noinspection ConstantConditions
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertTrue(reconstructor.getViewCount() > 0);
            if (reconstructor.getCurrentMetricEstimatedCamera() == null) {
                continue;
            }
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera1);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
            if (mEstimatedFundamentalMatrix == null || mEstimatedFundamentalMatrix.getFundamentalMatrix() == null) {
                continue;
            }

            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), LARGE_ABSOLUTE_ERROR) &&
                    !fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), LARGE_ABSOLUTE_ERROR) ||
                    fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), LARGE_ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            final PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            assertSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);

            assertSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();

            if (mMetricReconstructedPoints.size() < numPoints1) {
                continue;
            }

            final List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (int i = 0; i < numPoints1; i++) {
                final Point3D p = metricReconstructedPoints3D.get(i);
                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));
            }

            final Point3D estimatedCenter1 = estimatedMetricCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedMetricCamera2.getCameraCenter();

            // transform points and cameras to account for scale change
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            final double scale = baseline / estimatedBaseline;

            final MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);

            final PinholeCamera scaledCamera1 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera1);
            final PinholeCamera scaledCamera2 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera2);

            final List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();

            final Point3D scaledCenter1 = scaledCamera1.getCameraCenter();
            final Point3D scaledCenter2 = scaledCamera2.getCameraCenter();

            final Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();

            final double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

            // check cameras are correct
            if (Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);

            assertTrue(center1.equals(scaledCenter1, LARGE_ABSOLUTE_ERROR));

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i),
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
    public void testPlanarPointsDAQTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final SparseReconstructorConfiguration configuration =
                    new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getInitialCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPointX = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_ESSENTIAL,
                    MAX_PRINCIPAL_POINT_ESSENTIAL);
            final double principalPointY = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_ESSENTIAL,
                    MAX_PRINCIPAL_POINT_ESSENTIAL);

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, principalPointX, principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);

            // create 3D points laying in front of both cameras and laying in a plane

            // 1st find an approximate central point by intersecting the axis
            // planes of both cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            final double[] principalAxis1 = camera1.getPrincipalAxisArray();
            final double[] principalAxis2 = camera2.getPrincipalAxisArray();
            final double[] avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2),
                    0.5);

            final Plane plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();

            final double planeA = plane.getA();
            final double planeB = plane.getB();
            final double planeC = plane.getC();
            final double planeD = plane.getD();

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1;
            boolean front2;
            boolean failed = false;
            for (int i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final double homW = 1.0;
                    final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR,
                                MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) /
                                planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR,
                                MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) /
                                planeA;
                    }

                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                    assertTrue(plane.isLocus(point3D));

                    front1 = camera1.isPointInFrontOfCamera(point3D);
                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        failed = true;
                        break;
                    }
                    numTry++;
                } while (!front1 || !front2);

                if (failed) {
                    break;
                }

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front1);
                //noinspection ConstantConditions
                assertTrue(front2);

                // project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            if (failed) {
                continue;
            }

            Point2D projectedPoint2b;
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    // get a random point belonging to the plane
                    // a*x + b*y + c*z + d*w = 0
                    // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    final double homX;
                    final double homY;
                    final double homW = 1.0;
                    final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                        homX = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR,
                                MAX_RANDOM_VALUE_PLANAR);
                        homY = -(planeA * homX + planeC * homZ + planeD * homW) /
                                planeB;
                    } else {
                        homY = randomizer.nextDouble(MIN_RANDOM_VALUE_PLANAR,
                                MAX_RANDOM_VALUE_PLANAR);
                        homX = -(planeB * homY + planeC * homZ + planeD * homW) /
                                planeA;
                    }

                    point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                    assertTrue(plane.isLocus(point3D));

                    front2 = camera2.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        failed = true;
                        break;
                    }
                    numTry++;
                } while (!front2);

                if (failed) {
                    break;
                }

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            if (failed) {
                continue;
            }

            final SparseReconstructorListener listener =
                    new SparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                final SparseReconstructor reconstructor) {
                            return mViewCount < 2;
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
                            if (mViewCount == 0) {
                                // first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else {
                                // second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                // spawned samples
                                for (int i = 0; i < numPoints2; i++) {
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
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                final SparseReconstructor reconstructor, final int viewId,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(
                                final SparseReconstructor reconstructor,
                                final List<Sample2D> allPreviousViewSamples,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples,
                                final int previousViewId, final int currentViewId,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints1; i++) {
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
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedMetricCamera1 = previousCamera;
                            mEstimatedMetricCamera2 = currentCamera;
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            mEstimatedEuclideanCamera1 = previousCamera;
                            mEstimatedEuclideanCamera2 = currentCamera;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final double scale, final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public void onStart(
                                final SparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final SparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final SparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final SparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final SparseReconstructor reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            if (!mFinished || mFailed) {
                continue;
            }
            assertTrue(mStarted);
            //noinspection ConstantConditions
            assertTrue(mFinished);
            assertFalse(mCancelled);
            //noinspection ConstantConditions
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertTrue(reconstructor.getViewCount() > 0);
            if (reconstructor.getCurrentMetricEstimatedCamera() == null) {
                continue;
            }
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera1);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
            if (mEstimatedFundamentalMatrix == null || mEstimatedFundamentalMatrix.getFundamentalMatrix() == null) {
                continue;
            }

            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), LARGE_ABSOLUTE_ERROR) &&
                    !fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), LARGE_ABSOLUTE_ERROR) ||
                    fundamentalMatrix.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), LARGE_ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            final PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            assertSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);

            assertSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGeneralPointsEssentialEPnPAdditionalIntrinsicThreeViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final SparseReconstructorConfiguration configuration =
                    new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getInitialCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);
            configuration.setAdditionalCamerasIntrinsics(intrinsic);
            configuration.setUseEPnPForAdditionalCamerasEstimation(true);
            configuration.setUseUPnPForAdditionalCamerasEstimation(false);
            configuration.setUseDAQForAdditionalCamerasIntrinics(false);
            configuration.setUseDIACForAdditionalCamerasIntrinsics(false);

            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double alphaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation1 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            final double cameraSeparation2 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation1,
                    center1.getInhomY() + cameraSeparation1,
                    center1.getInhomZ() + cameraSeparation1);
            final Point3D center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            final MatrixRotation3D rotation3 = new MatrixRotation3D(alphaEuler3,
                    betaEuler3, gammaEuler3);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
            final PinholeCamera camera3 = new PinholeCamera(intrinsic, rotation3,
                    center3);

            final FundamentalMatrix fundamentalMatrix1 = new FundamentalMatrix(
                    camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of 1st two cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int start = randomizer.nextInt(0,
                    numPoints1 - MIN_TRACKED_POINTS);

            InhomogeneousPoint3D point3D;
            final List<InhomogeneousPoint3D> points3D1 = new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            Point2D projectedPoint3;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            final List<Point2D> projectedPoints3 = new ArrayList<>();
            boolean front1;
            boolean front2;
            for (int i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front1);
                //noinspection ConstantConditions
                assertTrue(front2);

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

            final List<InhomogeneousPoint3D> points3D2 = new ArrayList<>();
            Point2D projectedPoint2b;
            Point2D projectedPoint3b;
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            final List<Point2D> projectedPoints3b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);

                projectedPoint3b = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3b);
                projectedPoints3b.add(projectedPoint3b);
            }

            final SparseReconstructorListener listener =
                    new SparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                final SparseReconstructor reconstructor) {
                            return mViewCount < 3;
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
                            if (mViewCount == 0) {
                                // first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else if (mEstimatedFundamentalMatrix == null) {
                                // second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                // spawned samples
                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewNewlySpawnedSamples.add(sample);
                                }
                            } else {
                                // third view
                                for (int i = start; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = start; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints3.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints2; i++) {
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
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                final SparseReconstructor reconstructor, final int viewId,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(
                                final SparseReconstructor reconstructor,
                                final List<Sample2D> allPreviousViewSamples,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples,
                                final int previousViewId, final int currentViewId,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            int numCameras = 0;
                            if (mEstimatedMetricCamera1 != null &&
                                    (mEstimatedMetricCamera1.getViewId() == previousViewId ||
                                            mEstimatedMetricCamera1.getViewId() == currentViewId)) {
                                numCameras++;
                            }
                            if (mEstimatedMetricCamera2 != null &&
                                    (mEstimatedMetricCamera2.getViewId() == previousViewId ||
                                            mEstimatedMetricCamera2.getViewId() == currentViewId)) {
                                numCameras++;
                            }

                            EstimatedCamera[] estimatedCameras = null;
                            if (numCameras > 0) {
                                estimatedCameras = new EstimatedCamera[numCameras];

                                int pos = 0;
                                if (mEstimatedMetricCamera1 != null &&
                                        (mEstimatedMetricCamera1.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera1.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera1;
                                    pos++;
                                }
                                if (mEstimatedMetricCamera2 != null &&
                                        (mEstimatedMetricCamera2.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera2.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera2;
                                }
                            }

                            final List<Point2D> allPreviousPoints = new ArrayList<>();
                            for (final Sample2D sample : allPreviousViewSamples) {
                                allPreviousPoints.add(sample.getPoint());
                            }
                            final KDTree2D tree = new KDTree2D(allPreviousPoints);

                            // search previous view tracked samples within tree
                            final int numTrackedSamples = previousViewTrackedSamples.size();
                            Point2D point;
                            Point2D nearestPoint;
                            int nearestIndex;
                            MatchedSamples match;
                            for (int i = 0; i < numTrackedSamples; i++) {
                                final Sample2D previousSample = previousViewTrackedSamples.get(i);
                                point = previousSample.getPoint();
                                nearestIndex = tree.nearestIndex(point);
                                nearestPoint = allPreviousPoints.get(nearestIndex);
                                final Sample2D nearestSample = allPreviousViewSamples.get(nearestIndex);

                                if (point.distanceTo(nearestPoint) > NEAREST_THRESHOLD) {
                                    continue;
                                }

                                final Sample2D currentSample = currentViewTrackedSamples.get(i);

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
                            if (mEstimatedFundamentalMatrix == null) {
                                mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                            } else if (mEstimatedFundamentalMatrix2 == null) {
                                mEstimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                            }
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            if (mEstimatedMetricCamera2 == null) {
                                mEstimatedMetricCamera1 = previousCamera;
                                mEstimatedMetricCamera2 = currentCamera;
                            } else if (mEstimatedMetricCamera3 == null) {
                                mEstimatedMetricCamera2 = previousCamera;
                                mEstimatedMetricCamera3 = currentCamera;
                            }
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            if (mEstimatedEuclideanCamera2 == null) {
                                mEstimatedEuclideanCamera1 = previousCamera;
                                mEstimatedEuclideanCamera2 = currentCamera;
                                mScale = scale;
                            } else if (mEstimatedEuclideanCamera3 == null) {
                                mEstimatedEuclideanCamera2 = previousCamera;
                                mEstimatedEuclideanCamera3 = currentCamera;
                                mScale2 = scale;
                            }
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final double scale, final List<ReconstructedPoint3D> points) {
                            if (mEuclideanReconstructedPoints == null) {
                                mScale = scale;
                            } else {
                                mScale2 = scale;
                            }

                            mEuclideanReconstructedPoints = points;
                        }

                        @Override
                        public void onStart(
                                final SparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final SparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final SparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final SparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final SparseReconstructor reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera3);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera3);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale2, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrices are correct
            fundamentalMatrix1.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            assertNull(mEstimatedFundamentalMatrix2);

            // matrices are equal up to scale
            if (!fundamentalMatrix1.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix1.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix1.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix1.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            final PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            final PinholeCamera estimatedMetricCamera3 = mEstimatedMetricCamera3.getCamera();
            assertSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);
            assertSame(mEstimatedMetricCamera3, mEstimatedEuclideanCamera3);

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();
            estimatedMetricCamera3.decompose();

            assertSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            final int numReconstructedPoints = numPoints1 - start + numPoints2;
            if (mMetricReconstructedPoints.size() != numReconstructedPoints) {
                continue;
            }

            final List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numReconstructedPoints; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of at least 2nd camera
            for (int i = 0; i < numReconstructedPoints; i++) {
                final Point3D p = metricReconstructedPoints3D.get(i);
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));
            }

            final Point3D estimatedCenter1 = estimatedMetricCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedMetricCamera2.getCameraCenter();

            // transform points and cameras to account for scale change
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            final double scale = baseline / estimatedBaseline;
            assertEquals(mScale, 1.0, 0.0);
            assertEquals(mScale2, 1.0, 0.0);

            final MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);

            final PinholeCamera scaledCamera1 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera1);
            final PinholeCamera scaledCamera2 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera2);
            final PinholeCamera scaledCamera3 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera3);

            final List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();
            scaledCamera3.decompose();

            final Point3D scaledCenter1 = scaledCamera1.getCameraCenter();
            final Point3D scaledCenter2 = scaledCamera2.getCameraCenter();
            final Point3D scaledCenter3 = scaledCamera3.getCameraCenter();

            final PinholeCameraIntrinsicParameters scaledIntrinsic1 =
                    scaledCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters scaledIntrinsic2 =
                    scaledCamera2.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters scaledIntrinsic3 =
                    scaledCamera3.getIntrinsicParameters();

            final Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
            final Rotation3D scaledRotation2 = scaledCamera2.getCameraRotation();
            final Rotation3D scaledRotation3 = scaledCamera3.getCameraRotation();

            final double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

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

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic3.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(scaledRotation3.asInhomogeneousMatrix().equals(
                    rotation3.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = start; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i - start), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i - start),
                        LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            for (int i = 0; i < numPoints2; i++) {
                if (!points3D2.get(i).equals(
                        scaledReconstructionPoints3D.get(i + numPoints1 - start),
                        LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D2.get(i).equals(
                        scaledReconstructionPoints3D.get(i + numPoints1 - start),
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
    public void testGeneralPointsEssentialEPnPDAQThreeViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final SparseReconstructorConfiguration configuration =
                    new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getInitialCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);
            configuration.setUseEPnPForAdditionalCamerasEstimation(true);
            configuration.setUseUPnPForAdditionalCamerasEstimation(false);
            configuration.setUseDAQForAdditionalCamerasIntrinics(true);
            configuration.setUseDIACForAdditionalCamerasIntrinsics(false);

            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double alphaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation1 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            final double cameraSeparation2 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation1,
                    center1.getInhomY() + cameraSeparation1,
                    center1.getInhomZ() + cameraSeparation1);
            final Point3D center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            final MatrixRotation3D rotation3 = new MatrixRotation3D(alphaEuler3,
                    betaEuler3, gammaEuler3);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
            final PinholeCamera camera3 = new PinholeCamera(intrinsic, rotation3,
                    center3);

            final FundamentalMatrix fundamentalMatrix1 = new FundamentalMatrix(
                    camera1, camera2);
            final FundamentalMatrix fundamentalMatrix2 = new FundamentalMatrix(
                    camera2, camera3);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of 1st two cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int start = randomizer.nextInt(0,
                    numPoints1 - MIN_TRACKED_POINTS);

            InhomogeneousPoint3D point3D;
            final List<InhomogeneousPoint3D> points3D1 = new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            Point2D projectedPoint3;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            final List<Point2D> projectedPoints3 = new ArrayList<>();
            boolean front1;
            boolean front2;
            for (int i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front1);
                //noinspection ConstantConditions
                assertTrue(front2);

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

            final List<InhomogeneousPoint3D> points3D2 = new ArrayList<>();
            Point2D projectedPoint2b;
            Point2D projectedPoint3b;
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            final List<Point2D> projectedPoints3b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);

                projectedPoint3b = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3b);
                projectedPoints3b.add(projectedPoint3b);
            }

            final SparseReconstructorListener listener =
                    new SparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                final SparseReconstructor reconstructor) {
                            return mViewCount < 3;
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
                            if (mViewCount == 0) {
                                // first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else if (mEstimatedFundamentalMatrix == null) {
                                // second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                // spawned samples
                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewNewlySpawnedSamples.add(sample);
                                }
                            } else {
                                // third view
                                for (int i = start; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = start; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints3.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints2; i++) {
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
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                final SparseReconstructor reconstructor, final int viewId,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(
                                final SparseReconstructor reconstructor,
                                final List<Sample2D> allPreviousViewSamples,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples,
                                final int previousViewId, final int currentViewId,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            int numCameras = 0;
                            if (mEstimatedMetricCamera1 != null &&
                                    (mEstimatedMetricCamera1.getViewId() == previousViewId ||
                                            mEstimatedMetricCamera1.getViewId() == currentViewId)) {
                                numCameras++;
                            }
                            if (mEstimatedMetricCamera2 != null &&
                                    (mEstimatedMetricCamera2.getViewId() == previousViewId ||
                                            mEstimatedMetricCamera2.getViewId() == currentViewId)) {
                                numCameras++;
                            }

                            EstimatedCamera[] estimatedCameras = null;
                            if (numCameras > 0) {
                                estimatedCameras = new EstimatedCamera[numCameras];

                                int pos = 0;
                                if (mEstimatedMetricCamera1 != null &&
                                        (mEstimatedMetricCamera1.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera1.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera1;
                                    pos++;
                                }
                                if (mEstimatedMetricCamera2 != null &&
                                        (mEstimatedMetricCamera2.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera2.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera2;
                                }
                            }

                            final List<Point2D> allPreviousPoints = new ArrayList<>();
                            for (final Sample2D sample : allPreviousViewSamples) {
                                allPreviousPoints.add(sample.getPoint());
                            }
                            final KDTree2D tree = new KDTree2D(allPreviousPoints);

                            // search previous view tracked samples within tree
                            final int numTrackedSamples = previousViewTrackedSamples.size();
                            Point2D point;
                            Point2D nearestPoint;
                            int nearestIndex;
                            MatchedSamples match;
                            for (int i = 0; i < numTrackedSamples; i++) {
                                final Sample2D previousSample = previousViewTrackedSamples.get(i);
                                point = previousSample.getPoint();
                                nearestIndex = tree.nearestIndex(point);
                                nearestPoint = allPreviousPoints.get(nearestIndex);
                                final Sample2D nearestSample = allPreviousViewSamples.get(nearestIndex);

                                if (point.distanceTo(nearestPoint) > NEAREST_THRESHOLD) {
                                    continue;
                                }

                                final Sample2D currentSample = currentViewTrackedSamples.get(i);

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
                            if (mEstimatedFundamentalMatrix == null) {
                                mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                            } else if (mEstimatedFundamentalMatrix2 == null) {
                                mEstimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                            }
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            if (mEstimatedMetricCamera2 == null) {
                                mEstimatedMetricCamera1 = previousCamera;
                                mEstimatedMetricCamera2 = currentCamera;
                            } else if (mEstimatedMetricCamera3 == null) {
                                mEstimatedMetricCamera2 = previousCamera;
                                mEstimatedMetricCamera3 = currentCamera;
                            }
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            if (mEstimatedEuclideanCamera2 == null) {
                                mEstimatedEuclideanCamera1 = previousCamera;
                                mEstimatedEuclideanCamera2 = currentCamera;
                                mScale = scale;
                            } else if (mEstimatedEuclideanCamera3 == null) {
                                mEstimatedEuclideanCamera2 = previousCamera;
                                mEstimatedEuclideanCamera3 = currentCamera;
                                mScale2 = scale;
                            }
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final double scale, final List<ReconstructedPoint3D> points) {
                            if (mEuclideanReconstructedPoints == null) {
                                mScale = scale;
                            } else {
                                mScale2 = scale;
                            }

                            mEuclideanReconstructedPoints = points;
                        }

                        @Override
                        public void onStart(
                                final SparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final SparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final SparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final SparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final SparseReconstructor reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera3);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera3);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale2, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrices are correct
            fundamentalMatrix1.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            fundamentalMatrix2.normalize();
            mEstimatedFundamentalMatrix2.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix1.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix1.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix1.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix1.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));
            if (!fundamentalMatrix2.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix2.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix2.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix2.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix2.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix2.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix2.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix2.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));


            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            final PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            final PinholeCamera estimatedMetricCamera3 = mEstimatedMetricCamera3.getCamera();
            assertSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);
            assertSame(mEstimatedMetricCamera3, mEstimatedEuclideanCamera3);

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();
            estimatedMetricCamera3.decompose();

            assertSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            final int numReconstructedPoints = numPoints1 - start + numPoints2;

            final List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numReconstructedPoints; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of at least 1st two cameras
            for (int i = 0; i < numReconstructedPoints; i++) {
                final Point3D p = metricReconstructedPoints3D.get(i);
                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));
            }

            final Point3D estimatedCenter1 = estimatedMetricCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedMetricCamera2.getCameraCenter();

            // transform points and cameras to account for scale change
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            final double scale = baseline / estimatedBaseline;
            assertEquals(mScale, 1.0, 0.0);
            assertEquals(mScale2, 1.0, 0.0);

            final MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);

            final PinholeCamera scaledCamera1 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera1);
            final PinholeCamera scaledCamera2 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera2);
            final PinholeCamera scaledCamera3 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera3);

            final List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();
            scaledCamera3.decompose();

            final Point3D scaledCenter1 = scaledCamera1.getCameraCenter();
            final Point3D scaledCenter2 = scaledCamera2.getCameraCenter();
            final Point3D scaledCenter3 = scaledCamera3.getCameraCenter();

            final PinholeCameraIntrinsicParameters scaledIntrinsic1 =
                    scaledCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters scaledIntrinsic2 =
                    scaledCamera2.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters scaledIntrinsic3 =
                    scaledCamera3.getIntrinsicParameters();

            final Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
            final Rotation3D scaledRotation2 = scaledCamera2.getCameraRotation();
            final Rotation3D scaledRotation3 = scaledCamera3.getCameraRotation();

            final double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

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

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            if (Math.abs(scaledIntrinsic3.getHorizontalFocalLength() -
                    intrinsic.getHorizontalFocalLength()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            if (Math.abs(scaledIntrinsic3.getVerticalFocalLength() -
                    intrinsic.getVerticalFocalLength()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(scaledRotation3.asInhomogeneousMatrix().equals(
                    rotation3.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = start; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i - start), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i - start),
                        LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            for (int i = 0; i < numPoints2; i++) {
                if (!points3D2.get(i).equals(
                        scaledReconstructionPoints3D.get(i + numPoints1 - start),
                        LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D2.get(i).equals(
                        scaledReconstructionPoints3D.get(i + numPoints1 - start),
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
    public void testGeneralPointsEssentialEPnPDIACThreeViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            reset();

            final SparseReconstructorConfiguration configuration =
                    new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_DIAC,
                    MAX_FOCAL_LENGTH_DIAC);
            final double aspectRatio = configuration.getInitialCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPointX = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);
            final double principalPointY = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPointX, principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);
            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);
            configuration.setUseEPnPForAdditionalCamerasEstimation(true);
            configuration.setUseUPnPForAdditionalCamerasEstimation(false);
            configuration.setUseDAQForAdditionalCamerasIntrinics(false);
            configuration.setUseDIACForAdditionalCamerasIntrinsics(true);

            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double alphaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation1 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_DIAC,
                    MAX_CAMERA_SEPARATION_DIAC);
            final double cameraSeparation2 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_DIAC,
                    MAX_CAMERA_SEPARATION_DIAC);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation1,
                    center1.getInhomY() + cameraSeparation1,
                    center1.getInhomZ() + cameraSeparation1);
            final Point3D center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            final MatrixRotation3D rotation3 = new MatrixRotation3D(alphaEuler3,
                    betaEuler3, gammaEuler3);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
            final PinholeCamera camera3 = new PinholeCamera(intrinsic, rotation3,
                    center3);

            final FundamentalMatrix fundamentalMatrix1 = new FundamentalMatrix(
                    camera1, camera2);
            final FundamentalMatrix fundamentalMatrix2 = new FundamentalMatrix(
                    camera2, camera3);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of 1st two cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int start = randomizer.nextInt(0,
                    numPoints1 - MIN_TRACKED_POINTS);

            InhomogeneousPoint3D point3D;
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            Point2D projectedPoint3;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            final List<Point2D> projectedPoints3 = new ArrayList<>();
            boolean front1;
            boolean front2;
            boolean maxTriesReached = false;
            for (int i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front1);
                //noinspection ConstantConditions
                assertTrue(front2);

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
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            final List<Point2D> projectedPoints3b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_DIAC, MAX_LAMBDA_DIAC);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front2);

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


            final SparseReconstructorListener listener =
                    new SparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                final SparseReconstructor reconstructor) {
                            return mViewCount < 3;
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
                            if (mViewCount == 0) {
                                // first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else if (mEstimatedFundamentalMatrix == null) {
                                // second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                // spawned samples
                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewNewlySpawnedSamples.add(sample);
                                }
                            } else {
                                // third view
                                for (int i = start; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = start; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints3.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints2; i++) {
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
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                final SparseReconstructor reconstructor, final int viewId,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(
                                final SparseReconstructor reconstructor,
                                final List<Sample2D> allPreviousViewSamples,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples,
                                final int previousViewId, final int currentViewId,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            int numCameras = 0;
                            if (mEstimatedMetricCamera1 != null &&
                                    (mEstimatedMetricCamera1.getViewId() == previousViewId ||
                                            mEstimatedMetricCamera1.getViewId() == currentViewId)) {
                                numCameras++;
                            }
                            if (mEstimatedMetricCamera2 != null &&
                                    (mEstimatedMetricCamera2.getViewId() == previousViewId ||
                                            mEstimatedMetricCamera2.getViewId() == currentViewId)) {
                                numCameras++;
                            }

                            EstimatedCamera[] estimatedCameras = null;
                            if (numCameras > 0) {
                                estimatedCameras = new EstimatedCamera[numCameras];

                                int pos = 0;
                                if (mEstimatedMetricCamera1 != null &&
                                        (mEstimatedMetricCamera1.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera1.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera1;
                                    pos++;
                                }
                                if (mEstimatedMetricCamera2 != null &&
                                        (mEstimatedMetricCamera2.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera2.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera2;
                                }
                            }

                            final List<Point2D> allPreviousPoints = new ArrayList<>();
                            for (final Sample2D sample : allPreviousViewSamples) {
                                allPreviousPoints.add(sample.getPoint());
                            }
                            final KDTree2D tree = new KDTree2D(allPreviousPoints);

                            // search previous view tracked samples within tree
                            int numTrackedSamples = previousViewTrackedSamples.size();
                            Point2D point;
                            Point2D nearestPoint;
                            int nearestIndex;
                            MatchedSamples match;
                            for (int i = 0; i < numTrackedSamples; i++) {
                                final Sample2D previousSample = previousViewTrackedSamples.get(i);
                                point = previousSample.getPoint();
                                nearestIndex = tree.nearestIndex(point);
                                nearestPoint = allPreviousPoints.get(nearestIndex);
                                final Sample2D nearestSample = allPreviousViewSamples.get(nearestIndex);

                                if (point.distanceTo(nearestPoint) > NEAREST_THRESHOLD) {
                                    continue;
                                }

                                final Sample2D currentSample = currentViewTrackedSamples.get(i);

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
                            if (mEstimatedFundamentalMatrix == null) {
                                mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                            } else if (mEstimatedFundamentalMatrix2 == null) {
                                mEstimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                            }
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            if (mEstimatedMetricCamera2 == null) {
                                mEstimatedMetricCamera1 = previousCamera;
                                mEstimatedMetricCamera2 = currentCamera;
                            } else if (mEstimatedMetricCamera3 == null) {
                                mEstimatedMetricCamera2 = previousCamera;
                                mEstimatedMetricCamera3 = currentCamera;
                            }
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            if (mEstimatedEuclideanCamera2 == null) {
                                mEstimatedEuclideanCamera1 = previousCamera;
                                mEstimatedEuclideanCamera2 = currentCamera;
                                mScale = scale;
                            } else if (mEstimatedEuclideanCamera3 == null) {
                                mEstimatedEuclideanCamera2 = previousCamera;
                                mEstimatedEuclideanCamera3 = currentCamera;
                                mScale2 = scale;
                            }
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final double scale, final List<ReconstructedPoint3D> points) {
                            if (mEuclideanReconstructedPoints == null) {
                                mScale = scale;
                            } else {
                                mScale2 = scale;
                            }

                            mEuclideanReconstructedPoints = points;
                        }

                        @Override
                        public void onStart(
                                final SparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final SparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final SparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final SparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final SparseReconstructor reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            if (mEstimatedEuclideanCamera3 == null) {
                continue;
            }
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera3);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera3);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale2, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrices are correct
            fundamentalMatrix1.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            fundamentalMatrix2.normalize();
            mEstimatedFundamentalMatrix2.getFundamentalMatrix().normalize();

            // matrices are equal up to scale
            if (!fundamentalMatrix1.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix1.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix1.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix1.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));
            if (!fundamentalMatrix2.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix2.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix2.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix2.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix2.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix2.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix2.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix2.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));


            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            final PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            if (mEstimatedMetricCamera3 == null) {
                continue;
            }
            final PinholeCamera estimatedMetricCamera3 = mEstimatedMetricCamera3.getCamera();
            assertSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);
            assertSame(mEstimatedMetricCamera3, mEstimatedEuclideanCamera3);

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();
            estimatedMetricCamera3.decompose();

            assertSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            final int numReconstructedPoints = numPoints1 - start + numPoints2;

            final List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numReconstructedPoints; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of at least 1st two cameras
            boolean failed = false;
            for (int i = 0; i < numReconstructedPoints; i++) {
                final Point3D p = metricReconstructedPoints3D.get(i);
                if (!estimatedMetricCamera1.isPointInFrontOfCamera(p) ||
                        !estimatedMetricCamera2.isPointInFrontOfCamera(p)) {
                    failed = true;
                    break;
                }
                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));
            }

            if (failed) {
                continue;
            }

            final Point3D estimatedCenter1 = estimatedMetricCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedMetricCamera2.getCameraCenter();

            // transform points and cameras to account for scale change
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            final double scale = baseline / estimatedBaseline;
            assertEquals(mScale, 1.0, 0.0);
            assertEquals(mScale2, 1.0, 0.0);

            final MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);

            final PinholeCamera scaledCamera1 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera1);
            final PinholeCamera scaledCamera2 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera2);
            final PinholeCamera scaledCamera3 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera3);

            scaledCamera1.decompose();
            scaledCamera2.decompose();
            scaledCamera3.decompose();

            final Point3D scaledCenter1 = scaledCamera1.getCameraCenter();
            final Point3D scaledCenter2 = scaledCamera2.getCameraCenter();

            final PinholeCameraIntrinsicParameters scaledIntrinsic1 =
                    scaledCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters scaledIntrinsic2 =
                    scaledCamera2.getIntrinsicParameters();

            final Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
            final Rotation3D scaledRotation2 = scaledCamera2.getCameraRotation();

            final double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

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

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGeneralPointsEssentialUPnPThreeViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final SparseReconstructorConfiguration configuration =
                    new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getInitialCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);
            configuration.setUseEPnPForAdditionalCamerasEstimation(false);
            configuration.setUseUPnPForAdditionalCamerasEstimation(true);
            configuration.setUseDAQForAdditionalCamerasIntrinics(false);
            configuration.setUseDIACForAdditionalCamerasIntrinsics(false);

            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double alphaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation1 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            final double cameraSeparation2 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation1,
                    center1.getInhomY() + cameraSeparation1,
                    center1.getInhomZ() + cameraSeparation1);
            final Point3D center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            final MatrixRotation3D rotation3 = new MatrixRotation3D(alphaEuler3,
                    betaEuler3, gammaEuler3);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
            final PinholeCamera camera3 = new PinholeCamera(intrinsic, rotation3,
                    center3);

            final FundamentalMatrix fundamentalMatrix1 = new FundamentalMatrix(
                    camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of 1st two cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int start = randomizer.nextInt(0,
                    numPoints1 - MIN_TRACKED_POINTS);

            InhomogeneousPoint3D point3D;
            final List<InhomogeneousPoint3D> points3D1 = new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            Point2D projectedPoint3;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            final List<Point2D> projectedPoints3 = new ArrayList<>();
            boolean front1;
            boolean front2;
            for (int i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front1);
                //noinspection ConstantConditions
                assertTrue(front2);

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

            final List<InhomogeneousPoint3D> points3D2 = new ArrayList<>();
            Point2D projectedPoint2b;
            Point2D projectedPoint3b;
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            final List<Point2D> projectedPoints3b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);

                projectedPoint3b = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3b);
                projectedPoints3b.add(projectedPoint3b);
            }

            final SparseReconstructorListener listener =
                    new SparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                final SparseReconstructor reconstructor) {
                            return mViewCount < 3;
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
                            if (mViewCount == 0) {
                                // first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else if (mEstimatedFundamentalMatrix == null) {
                                // second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                // spawned samples
                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewNewlySpawnedSamples.add(sample);
                                }
                            } else {
                                // third view
                                for (int i = start; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = start; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints3.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints2; i++) {
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
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                final SparseReconstructor reconstructor, final int viewId,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(
                                final SparseReconstructor reconstructor,
                                final List<Sample2D> allPreviousViewSamples,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples,
                                final int previousViewId, final int currentViewId,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            int numCameras = 0;
                            if (mEstimatedMetricCamera1 != null &&
                                    (mEstimatedMetricCamera1.getViewId() == previousViewId ||
                                            mEstimatedMetricCamera1.getViewId() == currentViewId)) {
                                numCameras++;
                            }
                            if (mEstimatedMetricCamera2 != null &&
                                    (mEstimatedMetricCamera2.getViewId() == previousViewId ||
                                            mEstimatedMetricCamera2.getViewId() == currentViewId)) {
                                numCameras++;
                            }

                            EstimatedCamera[] estimatedCameras = null;
                            if (numCameras > 0) {
                                estimatedCameras = new EstimatedCamera[numCameras];

                                int pos = 0;
                                if (mEstimatedMetricCamera1 != null &&
                                        (mEstimatedMetricCamera1.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera1.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera1;
                                    pos++;
                                }
                                if (mEstimatedMetricCamera2 != null &&
                                        (mEstimatedMetricCamera2.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera2.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera2;
                                }
                            }

                            final List<Point2D> allPreviousPoints = new ArrayList<>();
                            for (final Sample2D sample : allPreviousViewSamples) {
                                allPreviousPoints.add(sample.getPoint());
                            }
                            final KDTree2D tree = new KDTree2D(allPreviousPoints);

                            // search previous view tracked samples within tree
                            final int numTrackedSamples = previousViewTrackedSamples.size();
                            Point2D point;
                            Point2D nearestPoint;
                            int nearestIndex;
                            MatchedSamples match;
                            for (int i = 0; i < numTrackedSamples; i++) {
                                final Sample2D previousSample = previousViewTrackedSamples.get(i);
                                point = previousSample.getPoint();
                                nearestIndex = tree.nearestIndex(point);
                                nearestPoint = allPreviousPoints.get(nearestIndex);
                                final Sample2D nearestSample = allPreviousViewSamples.get(nearestIndex);

                                if (point.distanceTo(nearestPoint) > NEAREST_THRESHOLD) {
                                    continue;
                                }

                                final Sample2D currentSample = currentViewTrackedSamples.get(i);

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
                            if (mEstimatedFundamentalMatrix == null) {
                                mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                            } else if (mEstimatedFundamentalMatrix2 == null) {
                                mEstimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                            }
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            if (mEstimatedMetricCamera2 == null) {
                                mEstimatedMetricCamera1 = previousCamera;
                                mEstimatedMetricCamera2 = currentCamera;
                            } else if (mEstimatedMetricCamera3 == null) {
                                mEstimatedMetricCamera2 = previousCamera;
                                mEstimatedMetricCamera3 = currentCamera;
                            }
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            if (mEstimatedEuclideanCamera2 == null) {
                                mEstimatedEuclideanCamera1 = previousCamera;
                                mEstimatedEuclideanCamera2 = currentCamera;
                                mScale = scale;
                            } else if (mEstimatedEuclideanCamera3 == null) {
                                mEstimatedEuclideanCamera2 = previousCamera;
                                mEstimatedEuclideanCamera3 = currentCamera;
                                mScale2 = scale;
                            }
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final double scale, final List<ReconstructedPoint3D> points) {
                            if (mEuclideanReconstructedPoints == null) {
                                mScale = scale;
                            } else {
                                mScale2 = scale;
                            }

                            mEuclideanReconstructedPoints = points;
                        }

                        @Override
                        public void onStart(
                                final SparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final SparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final SparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final SparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final SparseReconstructor reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera3);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera3);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale2, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrices are correct
            fundamentalMatrix1.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            assertNull(mEstimatedFundamentalMatrix2);

            // matrices are equal up to scale
            if (!fundamentalMatrix1.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix1.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix1.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix1.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            final PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            final PinholeCamera estimatedMetricCamera3 = mEstimatedMetricCamera3.getCamera();
            assertSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);
            assertSame(mEstimatedMetricCamera3, mEstimatedEuclideanCamera3);

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();
            estimatedMetricCamera3.decompose();

            assertSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            final int numReconstructedPoints = numPoints1 - start + numPoints2;

            final List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numReconstructedPoints; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of at least 1st two cameras
            for (int i = 0; i < numReconstructedPoints; i++) {
                final Point3D p = metricReconstructedPoints3D.get(i);
                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));
            }

            final Point3D estimatedCenter1 = estimatedMetricCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedMetricCamera2.getCameraCenter();

            // transform points and cameras to account for scale change
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            final double scale = baseline / estimatedBaseline;
            assertEquals(mScale, 1.0, 0.0);

            final MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);

            final PinholeCamera scaledCamera1 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera1);
            final PinholeCamera scaledCamera2 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera2);
            final PinholeCamera scaledCamera3 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera3);

            final List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();
            scaledCamera3.decompose();

            final Point3D scaledCenter1 = scaledCamera1.getCameraCenter();
            final Point3D scaledCenter2 = scaledCamera2.getCameraCenter();
            final Point3D scaledCenter3 = scaledCamera3.getCameraCenter();

            final PinholeCameraIntrinsicParameters scaledIntrinsic1 =
                    scaledCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters scaledIntrinsic2 =
                    scaledCamera2.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters scaledIntrinsic3 =
                    scaledCamera3.getIntrinsicParameters();

            final Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
            final Rotation3D scaledRotation2 = scaledCamera2.getCameraRotation();
            final Rotation3D scaledRotation3 = scaledCamera3.getCameraRotation();

            final double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

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

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            if (Math.abs(scaledIntrinsic3.getHorizontalFocalLength() -
                    intrinsic.getHorizontalFocalLength()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            if (Math.abs(scaledIntrinsic3.getVerticalFocalLength() -
                    intrinsic.getVerticalFocalLength()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic3.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(scaledRotation3.asInhomogeneousMatrix().equals(
                    rotation3.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = start; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i - start), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i - start),
                        LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            for (int i = 0; i < numPoints2; i++) {
                if (!points3D2.get(i).equals(
                        scaledReconstructionPoints3D.get(i + numPoints1 - start),
                        LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D2.get(i).equals(
                        scaledReconstructionPoints3D.get(i + numPoints1 - start),
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
    public void testGeneralPointsEssentialDLTThreeViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final SparseReconstructorConfiguration configuration =
                    new SparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getInitialCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);
            configuration.setUseEPnPForAdditionalCamerasEstimation(false);
            configuration.setUseUPnPForAdditionalCamerasEstimation(false);
            configuration.setUseDAQForAdditionalCamerasIntrinics(false);
            configuration.setUseDIACForAdditionalCamerasIntrinsics(false);

            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double alphaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation1 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            final double cameraSeparation2 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation1,
                    center1.getInhomY() + cameraSeparation1,
                    center1.getInhomZ() + cameraSeparation1);
            final Point3D center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            final MatrixRotation3D rotation3 = new MatrixRotation3D(alphaEuler3,
                    betaEuler3, gammaEuler3);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
            final PinholeCamera camera3 = new PinholeCamera(intrinsic, rotation3,
                    center3);

            final FundamentalMatrix fundamentalMatrix1 = new FundamentalMatrix(
                    camera1, camera2);

            // create 3D points laying in front of both cameras

            // 1st find an approximate central point by intersecting the axis
            // planes of 1st two cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrix = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrix.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrix.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrix.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrix.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrix.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrix.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrix.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrix.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrix.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrix.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrix.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrix.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrix.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrix.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrix.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrix.setElementAt(3, 3,
                    horizontalPlane2.getD());

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            final Matrix v = decomposer.getV();
            final HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final int numPoints1 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int numPoints2 = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);
            final int start = randomizer.nextInt(0,
                    numPoints1 - MIN_TRACKED_POINTS);

            InhomogeneousPoint3D point3D;
            final List<InhomogeneousPoint3D> points3D1 = new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            Point2D projectedPoint3;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            final List<Point2D> projectedPoints3 = new ArrayList<>();
            boolean front1;
            boolean front2;
            for (int i = 0; i < numPoints1; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front1);
                //noinspection ConstantConditions
                assertTrue(front2);

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

            final List<InhomogeneousPoint3D> points3D2 = new ArrayList<>();
            Point2D projectedPoint2b;
            Point2D projectedPoint3b;
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            final List<Point2D> projectedPoints3b = new ArrayList<>();
            for (int i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

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

                // check that 3D point is in front of both cameras
                //noinspection ConstantConditions
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);

                projectedPoint3b = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3b);
                projectedPoints3b.add(projectedPoint3b);
            }

            final SparseReconstructorListener listener =
                    new SparseReconstructorListener() {
                        @Override
                        public boolean hasMoreViewsAvailable(
                                final SparseReconstructor reconstructor) {
                            return mViewCount < 3;
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
                            if (mViewCount == 0) {
                                // first view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }
                            } else if (mEstimatedFundamentalMatrix == null) {
                                // second view
                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints1.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                // spawned samples
                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewNewlySpawnedSamples.add(sample);
                                }
                            } else {
                                // third view
                                for (int i = start; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints2; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints2b.get(i));
                                    sample.setViewId(previousViewId);
                                    previousViewTrackedSamples.add(sample);
                                }

                                for (int i = start; i < numPoints1; i++) {
                                    sample = new Sample2D();
                                    sample.setPoint(projectedPoints3.get(i));
                                    sample.setViewId(currentViewId);
                                    currentViewTrackedSamples.add(sample);
                                }

                                for (int i = 0; i < numPoints2; i++) {
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
                            mViewCount++;
                        }

                        @Override
                        public void onSamplesRejected(
                                final SparseReconstructor reconstructor, final int viewId,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples) {
                            mViewCount++;
                        }

                        @Override
                        public void onRequestMatches(
                                final SparseReconstructor reconstructor,
                                final List<Sample2D> allPreviousViewSamples,
                                final List<Sample2D> previousViewTrackedSamples,
                                final List<Sample2D> currentViewTrackedSamples,
                                final int previousViewId, final int currentViewId,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            int numCameras = 0;
                            if (mEstimatedMetricCamera1 != null &&
                                    (mEstimatedMetricCamera1.getViewId() == previousViewId ||
                                            mEstimatedMetricCamera1.getViewId() == currentViewId)) {
                                numCameras++;
                            }
                            if (mEstimatedMetricCamera2 != null &&
                                    (mEstimatedMetricCamera2.getViewId() == previousViewId ||
                                            mEstimatedMetricCamera2.getViewId() == currentViewId)) {
                                numCameras++;
                            }

                            EstimatedCamera[] estimatedCameras = null;
                            if (numCameras > 0) {
                                estimatedCameras = new EstimatedCamera[numCameras];

                                int pos = 0;
                                if (mEstimatedMetricCamera1 != null &&
                                        (mEstimatedMetricCamera1.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera1.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera1;
                                    pos++;
                                }
                                if (mEstimatedMetricCamera2 != null &&
                                        (mEstimatedMetricCamera2.getViewId() == previousViewId ||
                                                mEstimatedMetricCamera2.getViewId() == currentViewId)) {
                                    estimatedCameras[pos] = mEstimatedMetricCamera2;
                                }
                            }

                            final List<Point2D> allPreviousPoints = new ArrayList<>();
                            for (final Sample2D sample : allPreviousViewSamples) {
                                allPreviousPoints.add(sample.getPoint());
                            }
                            final KDTree2D tree = new KDTree2D(allPreviousPoints);

                            // search previous view tracked samples within tree
                            final int numTrackedSamples = previousViewTrackedSamples.size();
                            Point2D point;
                            Point2D nearestPoint;
                            int nearestIndex;
                            MatchedSamples match;
                            for (int i = 0; i < numTrackedSamples; i++) {
                                final Sample2D previousSample = previousViewTrackedSamples.get(i);
                                point = previousSample.getPoint();
                                nearestIndex = tree.nearestIndex(point);
                                nearestPoint = allPreviousPoints.get(nearestIndex);
                                final Sample2D nearestSample = allPreviousViewSamples.get(nearestIndex);

                                if (point.distanceTo(nearestPoint) > NEAREST_THRESHOLD) {
                                    continue;
                                }

                                final Sample2D currentSample = currentViewTrackedSamples.get(i);

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
                            if (mEstimatedFundamentalMatrix == null) {
                                mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                            } else if (mEstimatedFundamentalMatrix2 == null) {
                                mEstimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                            }
                        }

                        @Override
                        public void onMetricCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            if (mEstimatedMetricCamera2 == null) {
                                mEstimatedMetricCamera1 = previousCamera;
                                mEstimatedMetricCamera2 = currentCamera;
                            } else if (mEstimatedMetricCamera3 == null) {
                                mEstimatedMetricCamera2 = previousCamera;
                                mEstimatedMetricCamera3 = currentCamera;
                            }
                        }

                        @Override
                        public void onMetricReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                            mMetricReconstructedPoints = points;
                        }

                        @Override
                        public void onEuclideanCameraEstimated(
                                final SparseReconstructor reconstructor, final int previousViewId,
                                final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                                final EstimatedCamera currentCamera) {
                            if (mEstimatedEuclideanCamera2 == null) {
                                mEstimatedEuclideanCamera1 = previousCamera;
                                mEstimatedEuclideanCamera2 = currentCamera;
                                mScale = scale;
                            } else if (mEstimatedEuclideanCamera3 == null) {
                                mEstimatedEuclideanCamera2 = previousCamera;
                                mEstimatedEuclideanCamera3 = currentCamera;
                                mScale2 = scale;
                            }
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final SparseReconstructor reconstructor,
                                final double scale, final List<ReconstructedPoint3D> points) {
                            if (mEuclideanReconstructedPoints == null) {
                                mScale = scale;
                            } else {
                                mScale2 = scale;
                            }

                            mEuclideanReconstructedPoints = points;
                        }

                        @Override
                        public void onStart(
                                final SparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final SparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final SparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final SparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final SparseReconstructor reconstructor = new SparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            // check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertSame(reconstructor.getCurrentMetricEstimatedCamera(), mEstimatedMetricCamera3);
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(reconstructor.getPreviousMetricEstimatedCamera(), mEstimatedMetricCamera2);
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera3);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), mMetricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale2, 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrices are correct
            fundamentalMatrix1.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            assertNull(mEstimatedFundamentalMatrix2);

            // matrices are equal up to scale
            if (!fundamentalMatrix1.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix1.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix1.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix1.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));

            // check that reconstructed points are in a metric stratum (up to a
            // certain scale)
            final PinholeCamera estimatedMetricCamera1 = mEstimatedMetricCamera1.getCamera();
            final PinholeCamera estimatedMetricCamera2 = mEstimatedMetricCamera2.getCamera();
            final PinholeCamera estimatedMetricCamera3 = mEstimatedMetricCamera3.getCamera();
            assertSame(mEstimatedMetricCamera1, mEstimatedEuclideanCamera1);
            assertSame(mEstimatedMetricCamera2, mEstimatedEuclideanCamera2);
            assertSame(mEstimatedMetricCamera3, mEstimatedEuclideanCamera3);

            estimatedMetricCamera1.decompose();
            estimatedMetricCamera2.decompose();
            estimatedMetricCamera3.decompose();

            assertSame(mMetricReconstructedPoints, mEuclideanReconstructedPoints);

            final int numReconstructedPoints = numPoints1 - start + numPoints2;

            final List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numReconstructedPoints; i++) {
                metricReconstructedPoints3D.add(
                        mMetricReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of at least 1st two cameras
            for (int i = 0; i < numReconstructedPoints; i++) {
                final Point3D p = metricReconstructedPoints3D.get(i);
                assertTrue(estimatedMetricCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedMetricCamera2.isPointInFrontOfCamera(p));
            }

            final Point3D estimatedCenter1 = estimatedMetricCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedMetricCamera2.getCameraCenter();

            // transform points and cameras to account for scale change
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            final double scale = baseline / estimatedBaseline;
            assertEquals(mScale, 1.0, 0.0);

            final MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);

            final PinholeCamera scaledCamera1 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera1);
            final PinholeCamera scaledCamera2 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera2);
            final PinholeCamera scaledCamera3 =
                    scaleTransformation.transformAndReturnNew(estimatedMetricCamera3);

            final List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(metricReconstructedPoints3D);

            scaledCamera1.decompose();
            scaledCamera2.decompose();
            scaledCamera3.decompose();

            final Point3D scaledCenter1 = scaledCamera1.getCameraCenter();
            final Point3D scaledCenter2 = scaledCamera2.getCameraCenter();
            final Point3D scaledCenter3 = scaledCamera3.getCameraCenter();

            final PinholeCameraIntrinsicParameters scaledIntrinsic1 =
                    scaledCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters scaledIntrinsic2 =
                    scaledCamera2.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters scaledIntrinsic3 =
                    scaledCamera3.getIntrinsicParameters();

            final Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
            final Rotation3D scaledRotation2 = scaledCamera2.getCameraRotation();
            final Rotation3D scaledRotation3 = scaledCamera3.getCameraRotation();

            final double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);

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

            assertEquals(scaledIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(scaledIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(scaledIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            if (Math.abs(scaledIntrinsic3.getHorizontalFocalLength() -
                    intrinsic.getHorizontalFocalLength()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            if (Math.abs(scaledIntrinsic3.getVerticalFocalLength() -
                    intrinsic.getVerticalFocalLength()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            if (Math.abs(scaledIntrinsic3.getSkewness() -
                    intrinsic.getSkewness()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            if (Math.abs(scaledIntrinsic3.getHorizontalPrincipalPoint() -
                    intrinsic.getHorizontalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            if (Math.abs(scaledIntrinsic3.getVerticalPrincipalPoint() -
                    intrinsic.getVerticalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledIntrinsic3.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(scaledRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(scaledRotation3.asInhomogeneousMatrix().equals(
                    rotation3.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = start; i < numPoints1; i++) {
                if (!points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i - start), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D1.get(i).equals(
                        scaledReconstructionPoints3D.get(i - start),
                        LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            for (int i = 0; i < numPoints2; i++) {
                if (!points3D2.get(i).equals(
                        scaledReconstructionPoints3D.get(i + numPoints1 - start),
                        LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D2.get(i).equals(
                        scaledReconstructionPoints3D.get(i + numPoints1 - start),
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
        mViewCount = 0;
        mEstimatedFundamentalMatrix = mEstimatedFundamentalMatrix2 = null;
        mEstimatedMetricCamera1 = mEstimatedMetricCamera2 =
                mEstimatedMetricCamera3 = null;
        mEstimatedEuclideanCamera1 = mEstimatedEuclideanCamera2
                = mEstimatedEuclideanCamera3 = null;
        mMetricReconstructedPoints = null;
        mEuclideanReconstructedPoints = null;
        mStarted = mFinished = mFailed = mCancelled = false;
        mScale = 0.0;
    }
}
