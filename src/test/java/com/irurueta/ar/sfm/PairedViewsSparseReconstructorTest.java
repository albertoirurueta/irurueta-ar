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
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PairedViewsSparseReconstructorTest {

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

    private int mViewCount = 0;
    private EstimatedFundamentalMatrix mEstimatedFundamentalMatrix;
    private EstimatedFundamentalMatrix mEstimatedFundamentalMatrix2;
    private EstimatedCamera mEstimatedEuclideanCamera1;
    private EstimatedCamera mEstimatedEuclideanCamera2;
    private EstimatedCamera mEstimatedEuclideanCamera2b;
    private EstimatedCamera mEstimatedEuclideanCamera3;
    private List<ReconstructedPoint3D> mEuclideanReconstructedPoints;
    private List<ReconstructedPoint3D> mEuclideanReconstructedPoints2;

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
        mEstimatedEuclideanCamera1 = mEstimatedEuclideanCamera2
                = mEstimatedEuclideanCamera3 = null;
        mEuclideanReconstructedPoints = null;
        mStarted = mFinished = mFailed = mCancelled = false;
    }

    @Test
    public void testConstructor() {
        assertEquals(PairedViewsSparseReconstructor.MIN_NUMBER_OF_VIEWS, 2);

        final PairedViewsSparseReconstructorConfiguration configuration =
                new PairedViewsSparseReconstructorConfiguration();
        final PairedViewsSparseReconstructorListener listener =
                new PairedViewsSparseReconstructorListener() {
                    @Override
                    public double onBaselineRequested(
                            final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                            final int viewId2, final EstimatedCamera metricCamera1,
                            final EstimatedCamera metricCamera2) {
                        return 1.0;
                    }

                    @Override
                    public boolean hasMoreViewsAvailable(
                            final PairedViewsSparseReconstructor reconstructor) {
                        return false;
                    }

                    @Override
                    public void onRequestSamplesForCurrentViewPair(
                            final PairedViewsSparseReconstructor reconstructor,
                            final int viewId1, final int viewId2, final List<Sample2D> samples1,
                            final List<Sample2D> samples2) {
                    }

                    @Override
                    public void onSamplesAccepted(
                            final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                            final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    }

                    @Override
                    public void onSamplesRejected(
                            final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                            final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    }

                    @Override
                    public void onRequestMatches(
                            final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                            final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
                            final List<MatchedSamples> matches) {
                    }

                    @Override
                    public void onFundamentalMatrixEstimated(
                            final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                            final int viewId2, final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    }

                    @Override
                    public void onEuclideanCameraPairEstimated(
                            final PairedViewsSparseReconstructor reconstructor,
                            final int viewId1, final int viewId2, final double scale,
                            final EstimatedCamera camera1, final EstimatedCamera camera2) {
                    }

                    @Override
                    public void onEuclideanReconstructedPointsEstimated(
                            final PairedViewsSparseReconstructor reconstructor,
                            final int viewId1, final int viewId2, final double scale,
                            final List<ReconstructedPoint3D> points) {
                    }

                    @Override
                    public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                            final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                        return null;
                    }

                    @Override
                    public void onStart(final PairedViewsSparseReconstructor reconstructor) {
                    }

                    @Override
                    public void onFinish(final PairedViewsSparseReconstructor reconstructor) {
                    }

                    @Override
                    public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                    }

                    @Override
                    public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                    }
                };

        // constructor with listener
        PairedViewsSparseReconstructor reconstructor = new PairedViewsSparseReconstructor(listener);

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
        assertNull(reconstructor.getMetricReconstructedPoints());
        assertNull(reconstructor.getEuclideanReconstructedPoints());
        assertEquals(reconstructor.getCurrentScale(), BaseSparseReconstructor.DEFAULT_SCALE, 0.0);
        assertNull(reconstructor.getPreviousViewSamples());
        assertNull(reconstructor.getCurrentViewSamples());
        assertTrue(reconstructor.isFirstViewPair());
        assertFalse(reconstructor.isAdditionalViewPair());

        // constructor with configuration and listener
        reconstructor = new PairedViewsSparseReconstructor(configuration, listener);

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
        assertNull(reconstructor.getMetricReconstructedPoints());
        assertNull(reconstructor.getEuclideanReconstructedPoints());
        assertEquals(reconstructor.getCurrentScale(), BaseSparseReconstructor.DEFAULT_SCALE, 0.0);
        assertNull(reconstructor.getPreviousViewSamples());
        assertNull(reconstructor.getCurrentViewSamples());
        assertTrue(reconstructor.isFirstViewPair());
        assertFalse(reconstructor.isAdditionalViewPair());
    }

    @Test
    public void testGeneralPointsEssentialTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final PairedViewsSparseReconstructorConfiguration configuration =
                    new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getPairedCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
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

            // 1st find an approximate central point by intersecting the axis planes of
            // both cameras
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

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final List<InhomogeneousPoint3D> points3D =
                    new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1;
            boolean front2;
            for (int i = 0; i < numPoints; i++) {
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
                points3D.add(point3D);

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

            final PairedViewsSparseReconstructorListener listener =
                    new PairedViewsSparseReconstructorListener() {

                        @Override
                        public double onBaselineRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final EstimatedCamera metricCamera1,
                                final EstimatedCamera metricCamera2) {
                            return center1.distanceTo(center2);
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(
                                final PairedViewsSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final List<Sample2D> samples1,
                                final List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            Sample2D sample1;
                            Sample2D sample2;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final double scale, final EstimatedCamera camera1,
                                final EstimatedCamera camera2) {
                            mEstimatedEuclideanCamera1 = camera1;
                            mEstimatedEuclideanCamera2 = camera2;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final double scale,
                                final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final PairedViewsSparseReconstructor reconstructor = new PairedViewsSparseReconstructor(
                    configuration, listener);


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
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.isAdditionalViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(reconstructor.getEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            final PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();

            final List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                Point3D p = euclideanReconstructedPoints3D.get(i);
                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(p));
            }

            final Point3D estimatedCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedEuclideanCamera2.getCameraCenter();

            // check scale
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);

            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);
            assertEquals(mScale, baseline, LARGE_ABSOLUTE_ERROR);

            // check cameras
            final PinholeCameraIntrinsicParameters estimatedIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters estimatedIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();

            final Rotation3D estimatedRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            final Rotation3D estimatedRotation2 = estimatedEuclideanCamera2.getCameraRotation();

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR));

            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {
                if (!points3D.get(i).equals(
                        euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(
                        euclideanReconstructedPoints3D.get(i),
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
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final PairedViewsSparseReconstructorConfiguration configuration =
                    new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_DIAC,
                    MAX_FOCAL_LENGTH_DIAC);
            final double aspectRatio = configuration.getPairedCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPointX = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);
            final double principalPointY = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPointX, principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);
            configuration.setPairedCamerasAspectRatio(aspectRatio);

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
                    MIN_CAMERA_SEPARATION_DIAC,
                    MAX_CAMERA_SEPARATION_DIAC);

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

            // 1st find an approximate central point by intersecting the axis planes of
            // both cameras
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

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final List<InhomogeneousPoint3D> points3D =
                    new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1;
            boolean front2;
            boolean maxTriesReached = false;
            for (int i = 0; i < numPoints; i++) {
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

                points3D.add(point3D);

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

            final PairedViewsSparseReconstructorListener listener =
                    new PairedViewsSparseReconstructorListener() {
                        @Override
                        public double onBaselineRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final EstimatedCamera metricCamera1,
                                final EstimatedCamera metricCamera2) {
                            return center1.distanceTo(center2);
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(
                                final PairedViewsSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final List<Sample2D> samples1,
                                final List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            Sample2D sample1;
                            Sample2D sample2;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2,
                                final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final double scale,
                                final EstimatedCamera camera1, final EstimatedCamera camera2) {
                            mEstimatedEuclideanCamera1 = camera1;
                            mEstimatedEuclideanCamera2 = camera2;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final double scale,
                                final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final PairedViewsSparseReconstructor reconstructor = new PairedViewsSparseReconstructor(
                    configuration, listener);


            // check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            if (!mFinished || mFailed) {
                continue;
            }

            // check correctness
            assertTrue(mStarted);
            //noinspection ConstantConditions
            assertTrue(mFinished);
            assertFalse(mCancelled);
            //noinspection ConstantConditions
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.isAdditionalViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(reconstructor.getEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            final PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();

            final List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            // check that most of the points are in front of both cameras
            int valid = 0;
            int invalid = 0;
            for (int i = 0; i < numPoints; i++) {
                if (mEuclideanReconstructedPoints.get(i).isInlier()) {
                    final Point3D p = euclideanReconstructedPoints3D.get(i);
                    assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(p));
                    assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(p));
                    valid++;
                } else {
                    invalid++;
                }
            }

            assertTrue(valid >= invalid);

            final Point3D estimatedCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedEuclideanCamera2.getCameraCenter();

            // check scale
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);

            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);
            assertEquals(mScale, baseline, LARGE_ABSOLUTE_ERROR);

            // check cameras
            final PinholeCameraIntrinsicParameters estimatedIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters estimatedIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();

            final Rotation3D estimatedRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            final Rotation3D estimatedRotation2 = estimatedEuclideanCamera2.getCameraRotation();

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR));

            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {
                if (!points3D.get(i).equals(
                        euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(
                        euclideanReconstructedPoints3D.get(i),
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
            final PairedViewsSparseReconstructorConfiguration configuration =
                    new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getPairedCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPointX = 0.0;
            final double principalPointY = 0.0;

            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPointX, principalPointY, skewness);
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

            // 1st find an approximate central point by intersecting the axis planes of
            // both cameras
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

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final List<InhomogeneousPoint3D> points3D =
                    new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean leftFront;
            boolean rightFront;
            boolean maxTriesReached = false;
            for (int i = 0; i < numPoints; i++) {
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

                points3D.add(point3D);

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

            final PairedViewsSparseReconstructorListener listener =
                    new PairedViewsSparseReconstructorListener() {

                        @Override
                        public double onBaselineRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final EstimatedCamera metricCamera1,
                                final EstimatedCamera metricCamera2) {
                            return center1.distanceTo(center2);
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(
                                final PairedViewsSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final List<Sample2D> samples1,
                                final List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            Sample2D sample1;
                            Sample2D sample2;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2,
                                final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final double scale,
                                final EstimatedCamera camera1, final EstimatedCamera camera2) {
                            mEstimatedEuclideanCamera1 = camera1;
                            mEstimatedEuclideanCamera2 = camera2;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final double scale, final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final PairedViewsSparseReconstructor reconstructor = new PairedViewsSparseReconstructor(
                    configuration, listener);


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
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.isAdditionalViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(reconstructor.getEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            final PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();

            final List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                final Point3D p = euclideanReconstructedPoints3D.get(i);
                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(p));
            }

            final Point3D estimatedCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedEuclideanCamera2.getCameraCenter();

            // check scale
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);

            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);
            assertEquals(mScale, baseline, LARGE_ABSOLUTE_ERROR);

            // check cameras
            final PinholeCameraIntrinsicParameters estimatedIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters estimatedIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();

            final Rotation3D estimatedRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            final Rotation3D estimatedRotation2 = estimatedEuclideanCamera2.getCameraRotation();

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR));

            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), LARGE_ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), LARGE_ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {
                if (!points3D.get(i).equals(
                        euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(
                        euclideanReconstructedPoints3D.get(i),
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
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final PairedViewsSparseReconstructorConfiguration configuration =
                    new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getPairedCamerasAspectRatio();
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
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPointX, principalPointY, skewness);
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

            // 1st find an approximate central point by intersecting the axis planes of
            // both cameras
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

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final List<InhomogeneousPoint3D> points3D =
                    new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean leftFront;
            boolean rightFront;
            boolean maxTriesReached = false;
            for (int i = 0; i < numPoints; i++) {
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

                points3D.add(point3D);

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

            final PairedViewsSparseReconstructorListener listener =
                    new PairedViewsSparseReconstructorListener() {

                        @Override
                        public double onBaselineRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final EstimatedCamera metricCamera1,
                                final EstimatedCamera metricCamera2) {
                            return center1.distanceTo(center2);
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(
                                final PairedViewsSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final List<Sample2D> samples1,
                                final List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            Sample2D sample1;
                            Sample2D sample2;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2,
                                final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final double scale,
                                final EstimatedCamera camera1, final EstimatedCamera camera2) {
                            mEstimatedEuclideanCamera1 = camera1;
                            mEstimatedEuclideanCamera2 = camera2;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final double scale, final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final PairedViewsSparseReconstructor reconstructor = new PairedViewsSparseReconstructor(
                    configuration, listener);

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
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.isAdditionalViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(reconstructor.getEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            final PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();

            final List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                final Point3D p = euclideanReconstructedPoints3D.get(i);
                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(p));
            }

            final Point3D estimatedCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedEuclideanCamera2.getCameraCenter();

            // check scale
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);

            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);
            assertEquals(mScale, baseline, LARGE_ABSOLUTE_ERROR);

            // check cameras
            final PinholeCameraIntrinsicParameters estimatedIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters estimatedIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();

            final Rotation3D estimatedRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            final Rotation3D estimatedRotation2 = estimatedEuclideanCamera2.getCameraRotation();

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR));

            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), LARGE_ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), LARGE_ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {
                if (!points3D.get(i).equals(
                        euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(
                        euclideanReconstructedPoints3D.get(i),
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
            CameraException, com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final PairedViewsSparseReconstructorConfiguration configuration =
                    new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getPairedCamerasAspectRatio();
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
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPointX, principalPointY, skewness);
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

            // 1st find an approximate central point by intersecting the axis planes of
            // both cameras
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

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean leftFront;
            boolean rightFront;
            boolean maxTriesReached = false;
            for (int i = 0; i < numPoints; i++) {
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

            final PairedViewsSparseReconstructorListener listener =
                    new PairedViewsSparseReconstructorListener() {

                        @Override
                        public double onBaselineRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final EstimatedCamera metricCamera1,
                                final EstimatedCamera metricCamera2) {
                            return center1.distanceTo(center2);
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(
                                final PairedViewsSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final List<Sample2D> samples1,
                                final List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            Sample2D sample1;
                            Sample2D sample2;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final double scale, final EstimatedCamera camera1,
                                final EstimatedCamera camera2) {
                            mEstimatedEuclideanCamera1 = camera1;
                            mEstimatedEuclideanCamera2 = camera2;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final double scale, final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final PairedViewsSparseReconstructor reconstructor = new PairedViewsSparseReconstructor(
                    configuration, listener);

            // check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            try {
              reconstructor.start();
            } catch (final IndexOutOfBoundsException e) {
              continue;
            }

            // check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.isAdditionalViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(reconstructor.getEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            final PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();

            // check cameras
            final PinholeCameraIntrinsicParameters estimatedIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters estimatedIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();

            if (Math.abs(estimatedIntrinsic1.getHorizontalFocalLength() - intrinsic.getHorizontalFocalLength()) >
                    LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);

            if (Math.abs(estimatedIntrinsic1.getVerticalFocalLength() - intrinsic.getVerticalFocalLength()) >
                    LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(),
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
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final PairedViewsSparseReconstructorConfiguration configuration =
                    new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getPairedCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPointX = 0.0;
            final double principalPointY = 0.0;

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

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            final List<HomogeneousPoint3D> points3D =
                    new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1;
            boolean front2;
            for (int i = 0; i < numPoints; i++) {
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
                points3D.add(point3D);

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

            final PairedViewsSparseReconstructorListener listener =
                    new PairedViewsSparseReconstructorListener() {

                        @Override
                        public double onBaselineRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final EstimatedCamera metricCamera1,
                                final EstimatedCamera metricCamera2) {
                            return center1.distanceTo(center2);
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(
                                final PairedViewsSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final List<Sample2D> samples1,
                                final List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            Sample2D sample1;
                            Sample2D sample2;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2,
                                final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final double scale,
                                final EstimatedCamera camera1, final EstimatedCamera camera2) {
                            mEstimatedEuclideanCamera1 = camera1;
                            mEstimatedEuclideanCamera2 = camera2;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final double scale, final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(final PairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(final PairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final PairedViewsSparseReconstructor reconstructor = new PairedViewsSparseReconstructor(
                    configuration, listener);

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
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(reconstructor.getEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            final PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();

            final List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                final Point3D p = euclideanReconstructedPoints3D.get(i);
                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(p));
            }

            final Point3D estimatedCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedEuclideanCamera2.getCameraCenter();

            // check scale
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);

            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);
            assertEquals(mScale, baseline, LARGE_ABSOLUTE_ERROR);

            // check cameras
            final PinholeCameraIntrinsicParameters estimatedIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters estimatedIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();

            final Rotation3D estimatedRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            final Rotation3D estimatedRotation2 = estimatedEuclideanCamera2.getCameraRotation();

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(center2.equals(estimatedCenter2, LARGE_ABSOLUTE_ERROR));

            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {
                if (!points3D.get(i).equals(
                        euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(
                        euclideanReconstructedPoints3D.get(i),
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
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final PairedViewsSparseReconstructorConfiguration configuration =
                    new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);
            configuration.setIntrinsicParametersKnown(true);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getPairedCamerasAspectRatio();
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
            configuration.setPairedCamerasAspectRatio(aspectRatio);

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

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1;
            boolean front2;
            for (int i = 0; i < numPoints; i++) {
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

            final PairedViewsSparseReconstructorListener listener =
                    new PairedViewsSparseReconstructorListener() {

                        @Override
                        public double onBaselineRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final EstimatedCamera metricCamera1,
                                final EstimatedCamera metricCamera2) {
                            return center1.distanceTo(center2);
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(
                                final PairedViewsSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final List<Sample2D> samples1,
                                final List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            Sample2D sample1;
                            Sample2D sample2;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final double scale,
                                final EstimatedCamera camera1, final EstimatedCamera camera2) {
                            mEstimatedEuclideanCamera1 = camera1;
                            mEstimatedEuclideanCamera2 = camera2;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final double scale, final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(final PairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(final PairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final PairedViewsSparseReconstructor reconstructor = new PairedViewsSparseReconstructor(
                    configuration, listener);

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
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(reconstructor.getEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            final PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();

            final List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            // check that most of the points are in front of both cameras
            int valid = 0;
            int invalid = 0;
            for (int i = 0; i < numPoints; i++) {
                if (mEuclideanReconstructedPoints.get(i).isInlier()) {
                    final Point3D p = euclideanReconstructedPoints3D.get(i);
                    assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(p));
                    assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(p));
                    valid++;
                } else {
                    invalid++;
                }
            }

            if (valid < invalid) {
                continue;
            }

            final Point3D estimatedCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedEuclideanCamera2.getCameraCenter();

            // check scale
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);

            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);
            assertEquals(mScale, baseline, LARGE_ABSOLUTE_ERROR);

            // check cameras
            final Rotation3D estimatedRotation1 = estimatedEuclideanCamera1.getCameraRotation();

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(
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
            final PairedViewsSparseReconstructorConfiguration configuration =
                    new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getPairedCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPointX = 0.0;
            final double principalPointY = 0.0;

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

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            final List<HomogeneousPoint3D> points3D =
                    new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1;
            boolean front2;
            for (int i = 0; i < numPoints; i++) {
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
                points3D.add(point3D);

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

            final PairedViewsSparseReconstructorListener listener =
                    new PairedViewsSparseReconstructorListener() {

                        @Override
                        public double onBaselineRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final EstimatedCamera metricCamera1,
                                final EstimatedCamera metricCamera2) {
                            return center1.distanceTo(center2);
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(
                                final PairedViewsSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            Sample2D sample1;
                            Sample2D sample2;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2,
                                final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final double scale,
                                final EstimatedCamera camera1, final EstimatedCamera camera2) {
                            mEstimatedEuclideanCamera1 = camera1;
                            mEstimatedEuclideanCamera2 = camera2;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final double scale, final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final PairedViewsSparseReconstructor reconstructor = new PairedViewsSparseReconstructor(
                    configuration, listener);

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
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(reconstructor.getEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            final PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();

            final List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                final Point3D p = euclideanReconstructedPoints3D.get(i);
                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(p));
            }

            final Point3D estimatedCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedEuclideanCamera2.getCameraCenter();

            // check scale
            final double baseline = center1.distanceTo(center2);
            final double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);

            if (Math.abs(estimatedBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, LARGE_ABSOLUTE_ERROR);
            assertEquals(mScale, baseline, LARGE_ABSOLUTE_ERROR);

            // check cameras
            final Rotation3D estimatedRotation1 = estimatedEuclideanCamera1.getCameraRotation();

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {
                if (!points3D.get(i).equals(
                        euclideanReconstructedPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(
                        euclideanReconstructedPoints3D.get(i),
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
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final PairedViewsSparseReconstructorConfiguration configuration =
                    new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getPairedCamerasAspectRatio();
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

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            HomogeneousPoint3D point3D;
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1;
            boolean front2;
            boolean failed = false;
            for (int i = 0; i < numPoints; i++) {
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

            final PairedViewsSparseReconstructorListener listener =
                    new PairedViewsSparseReconstructorListener() {

                        @Override
                        public double onBaselineRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final EstimatedCamera metricCamera1,
                                final EstimatedCamera metricCamera2) {
                            return center1.distanceTo(center2);
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(
                                final PairedViewsSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final List<Sample2D> samples1,
                                final List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            Sample2D sample1;
                            Sample2D sample2;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            MatchedSamples match;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2,
                                final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final double scale,
                                final EstimatedCamera camera1, final EstimatedCamera camera2) {
                            mEstimatedEuclideanCamera1 = camera1;
                            mEstimatedEuclideanCamera2 = camera2;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final double scale, final List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final PairedViewsSparseReconstructor reconstructor = new PairedViewsSparseReconstructor(
                    configuration, listener);

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
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(reconstructor.getEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

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

            final PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            final PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGeneralPointsEssentialThreeViews()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final PairedViewsSparseReconstructorConfiguration configuration =
                    new PairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            final double aspectRatio = configuration.getPairedCamerasAspectRatio();
            final double skewness = 0.0;
            final double principalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
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

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);
            final FundamentalMatrix fundamentalMatrix2 = new FundamentalMatrix(
                    camera2, camera3);

            // create 3D points laying in front of all cameras

            // 1st find an approximate central point by intersecting the axis planes of
            // all cameras
            final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            final Plane horizontalPlane3 = camera3.getHorizontalAxisPlane();
            final Plane verticalPlane3 = camera3.getVerticalAxisPlane();
            final Matrix planesIntersectionMatrixPair1 = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            final Matrix planesIntersectionMatrixPair2 = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            planesIntersectionMatrixPair1.setElementAt(0, 0, verticalPlane1.getA());
            planesIntersectionMatrixPair1.setElementAt(0, 1, verticalPlane1.getB());
            planesIntersectionMatrixPair1.setElementAt(0, 2, verticalPlane1.getC());
            planesIntersectionMatrixPair1.setElementAt(0, 3, verticalPlane1.getD());

            planesIntersectionMatrixPair1.setElementAt(1, 0,
                    horizontalPlane1.getA());
            planesIntersectionMatrixPair1.setElementAt(1, 1,
                    horizontalPlane1.getB());
            planesIntersectionMatrixPair1.setElementAt(1, 2,
                    horizontalPlane1.getC());
            planesIntersectionMatrixPair1.setElementAt(1, 3,
                    horizontalPlane1.getD());

            planesIntersectionMatrixPair1.setElementAt(2, 0, verticalPlane2.getA());
            planesIntersectionMatrixPair1.setElementAt(2, 1, verticalPlane2.getB());
            planesIntersectionMatrixPair1.setElementAt(2, 2, verticalPlane2.getC());
            planesIntersectionMatrixPair1.setElementAt(2, 3, verticalPlane2.getD());

            planesIntersectionMatrixPair1.setElementAt(3, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrixPair1.setElementAt(3, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrixPair1.setElementAt(3, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrixPair1.setElementAt(3, 3,
                    horizontalPlane2.getD());

            planesIntersectionMatrixPair2.setElementAt(0, 0, verticalPlane2.getA());
            planesIntersectionMatrixPair2.setElementAt(0, 1, verticalPlane2.getB());
            planesIntersectionMatrixPair2.setElementAt(0, 2, verticalPlane2.getC());
            planesIntersectionMatrixPair2.setElementAt(0, 3, verticalPlane2.getD());

            planesIntersectionMatrixPair2.setElementAt(1, 0,
                    horizontalPlane2.getA());
            planesIntersectionMatrixPair2.setElementAt(1, 1,
                    horizontalPlane2.getB());
            planesIntersectionMatrixPair2.setElementAt(1, 2,
                    horizontalPlane2.getC());
            planesIntersectionMatrixPair2.setElementAt(1, 3,
                    horizontalPlane2.getD());

            planesIntersectionMatrixPair2.setElementAt(2, 0, verticalPlane3.getA());
            planesIntersectionMatrixPair2.setElementAt(2, 1, verticalPlane3.getB());
            planesIntersectionMatrixPair2.setElementAt(2, 2, verticalPlane3.getC());
            planesIntersectionMatrixPair2.setElementAt(2, 3, verticalPlane3.getD());

            planesIntersectionMatrixPair2.setElementAt(3, 0,
                    horizontalPlane3.getA());
            planesIntersectionMatrixPair2.setElementAt(3, 1,
                    horizontalPlane3.getB());
            planesIntersectionMatrixPair2.setElementAt(3, 2,
                    horizontalPlane3.getC());
            planesIntersectionMatrixPair2.setElementAt(3, 2,
                    horizontalPlane3.getD());

            final SingularValueDecomposer decomposerPair1 = new SingularValueDecomposer(
                    planesIntersectionMatrixPair1);
            decomposerPair1.decompose();
            final Matrix vPair1 = decomposerPair1.getV();

            final SingularValueDecomposer decomposerPair2 = new SingularValueDecomposer(
                    planesIntersectionMatrixPair2);
            decomposerPair2.decompose();
            final Matrix vPair2 = decomposerPair2.getV();

            final HomogeneousPoint3D centralCommonPointPair1 = new HomogeneousPoint3D(
                    vPair1.getElementAt(0, 3),
                    vPair1.getElementAt(1, 3),
                    vPair1.getElementAt(2, 3),
                    vPair1.getElementAt(3, 3));

            final HomogeneousPoint3D centralCommonPointPair2 = new HomogeneousPoint3D(
                    vPair2.getElementAt(0, 3),
                    vPair2.getElementAt(1, 3),
                    vPair2.getElementAt(2, 3),
                    vPair2.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final int numPointsPair1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final int numPointsPair2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final List<InhomogeneousPoint3D> points3DPair1 = new ArrayList<>();
            final List<InhomogeneousPoint3D> points3DPair2 = new ArrayList<>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            Point2D projectedPoint3;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2a = new ArrayList<>();
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            final List<Point2D> projectedPoints3 = new ArrayList<>();
            boolean front1;
            boolean front2;
            boolean front3;
            for (int i = 0; i < numPointsPair1; i++) {
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

                // check that 3D point is in front of 1st pair of cameras
                //noinspection ConstantConditions
                assertTrue(front1);
                //noinspection ConstantConditions
                assertTrue(front2);
                //noinspection ConstantConditions
                assertTrue(front3);

                points3DPair1.add(point3D);

                // project 3D point into 1st pair of cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2a.add(projectedPoint2);
            }

            for (int i = 0; i < numPointsPair2; i++) {
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

                // check that 3D point is in front of 2nd pair of cameras
                //noinspection ConstantConditions
                assertTrue(front1);
                //noinspection ConstantConditions
                assertTrue(front2);
                //noinspection ConstantConditions
                assertTrue(front3);

                points3DPair2.add(point3D);

                // project 3D point into 2nd pair of cameras
                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2b.add(projectedPoint2);

                projectedPoint3 = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3);
                projectedPoints3.add(projectedPoint3);
            }

            final PairedViewsSparseReconstructorListener listener =
                    new PairedViewsSparseReconstructorListener() {

                        @Override
                        public double onBaselineRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final EstimatedCamera metricCamera1,
                                final EstimatedCamera metricCamera2) {
                            final int viewCount = reconstructor.getViewCount();
                            if (viewCount == 0) {
                                return center1.distanceTo(center2);
                            } else if (viewCount == 2) {
                                return center2.distanceTo(center3);
                            }

                            return 1.0;
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(
                                final PairedViewsSparseReconstructor reconstructor) {
                            // 3 views = 2 view pairs (2 images * 2 views --> 4 view counts)
                            return mViewCount < 4;
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final List<Sample2D> samples1,
                                final List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            final int viewCount = reconstructor.getViewCount();

                            Sample2D sample1;
                            Sample2D sample2;
                            if (viewCount == 0) {
                                // first view pair
                                for (int i = 0; i < numPointsPair1; i++) {
                                    sample1 = new Sample2D();
                                    sample1.setPoint(projectedPoints1.get(i));
                                    sample1.setViewId(viewId1);
                                    samples1.add(sample1);

                                    sample2 = new Sample2D();
                                    sample2.setPoint(projectedPoints2a.get(i));
                                    sample2.setViewId(viewId2);
                                    samples2.add(sample2);
                                }

                            } else if (viewCount == 2) {
                                // second view pair
                                for (int i = 0; i < numPointsPair2; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
                                final List<MatchedSamples> matches) {
                            matches.clear();

                            final int viewCount = reconstructor.getViewCount();
                            final int numPoints;
                            if (viewCount == 0) {
                                // first view pair
                                numPoints = numPointsPair1;
                            } else {
                                // second view pair
                                numPoints = numPointsPair2;
                            }

                            MatchedSamples match;
                            for (int i = 0; i < numPoints; i++) {
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
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2,
                                final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {

                            final int viewCount = reconstructor.getViewCount();
                            if (viewCount == 0) {
                                mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                            } else if (viewCount == 2) {
                                mEstimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                            }
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(
                                final PairedViewsSparseReconstructor reconstructor,
                                final int viewId1, final int viewId2, final double scale,
                                final EstimatedCamera camera1, final EstimatedCamera camera2) {

                            final int viewCount = reconstructor.getViewCount();
                            if (viewCount == 0) {
                                mEstimatedEuclideanCamera1 = camera1;
                                mEstimatedEuclideanCamera2 = camera2;
                                mScale = scale;
                            } else if (viewCount == 2) {
                                mEstimatedEuclideanCamera2b = camera1;
                                mEstimatedEuclideanCamera3 = camera2;
                                mScale2 = scale;
                            }
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                                final int viewId2, final double scale, final List<ReconstructedPoint3D> points) {

                            final int viewCount = reconstructor.getViewCount();
                            if (viewCount == 0) {
                                mEuclideanReconstructedPoints = points;
                                mScale = scale;
                            } else if (viewCount == 2) {
                                mEuclideanReconstructedPoints2 = points;
                                mScale2 = scale;
                            }
                        }

                        @Override
                        public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                                final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(
                                final PairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            final PairedViewsSparseReconstructor reconstructor = new PairedViewsSparseReconstructor(
                    configuration, listener);

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
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.isAdditionalViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix2);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera3);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2b);
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(reconstructor.getEuclideanReconstructedPoints(), mEuclideanReconstructedPoints2);
            assertEquals(reconstructor.getCurrentScale(), mScale2, 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

            // check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            fundamentalMatrix2.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();
            mEstimatedFundamentalMatrix2.getFundamentalMatrix().normalize();

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

            // check that reconstructed points are in euclidean stratum (up to a certain scale)
            final PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            final PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();
            final PinholeCamera estimatedEuclideanCamera2b = mEstimatedEuclideanCamera2b.getCamera();
            final PinholeCamera estimatedEuclideanCamera3 = mEstimatedEuclideanCamera3.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();
            estimatedEuclideanCamera2b.decompose();
            estimatedEuclideanCamera3.decompose();

            final List<Point3D> euclideanReconstructedPoints3DPair1 = new ArrayList<>();
            for (int i = 0; i < numPointsPair1; i++) {
                euclideanReconstructedPoints3DPair1.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            final List<Point3D> euclideanReconstructedPoints3DPair2 = new ArrayList<>();
            for (int i = 0; i < numPointsPair2; i++) {
                euclideanReconstructedPoints3DPair2.add(
                        mEuclideanReconstructedPoints2.get(i).getPoint());
            }

            // check that most points are in front of all cameras
            int numValidPoints = 0;
            int numInvalidPoints = 0;
            for (int i = 0; i < numPointsPair1; i++) {
                final Point3D p = euclideanReconstructedPoints3DPair1.get(i);
                if (estimatedEuclideanCamera1.isPointInFrontOfCamera(p) &&
                        estimatedEuclideanCamera2.isPointInFrontOfCamera(p) &&
                        estimatedEuclideanCamera3.isPointInFrontOfCamera(p)) {

                    assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(p));
                    assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(p));
                    assertTrue(estimatedEuclideanCamera3.isPointInFrontOfCamera(p));

                    numValidPoints++;
                } else {
                    numInvalidPoints++;
                }
            }

            assertTrue(numValidPoints > numInvalidPoints);

            numValidPoints = 0;
            numInvalidPoints = 0;
            for (int i = 0; i < numPointsPair2; i++) {
                final Point3D p = euclideanReconstructedPoints3DPair2.get(i);
                if (estimatedEuclideanCamera1.isPointInFrontOfCamera(p) &&
                        estimatedEuclideanCamera2.isPointInFrontOfCamera(p) &&
                        estimatedEuclideanCamera2b.isPointInFrontOfCamera(p) &&
                        estimatedEuclideanCamera3.isPointInFrontOfCamera(p)) {

                    assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(p));
                    assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(p));
                    assertTrue(estimatedEuclideanCamera2b.isPointInFrontOfCamera(p));
                    assertTrue(estimatedEuclideanCamera3.isPointInFrontOfCamera(p));

                    numValidPoints++;
                } else {
                    numInvalidPoints++;
                }
            }

            assertTrue(numValidPoints > numInvalidPoints);

            final Point3D estimatedCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            final Point3D estimatedCenter2 = estimatedEuclideanCamera2.getCameraCenter();
            final Point3D estimatedCenter2b = estimatedEuclideanCamera2b.getCameraCenter();
            final Point3D estimatedCenter3 = estimatedEuclideanCamera3.getCameraCenter();

            // check scale
            final double baseline1 = center1.distanceTo(center2);
            final double estimatedBaseline1 = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            final double baseline2 = center2.distanceTo(center3);
            final double estimatedBaseline2 = estimatedCenter2.distanceTo(
                    estimatedCenter3);

            if (Math.abs(estimatedBaseline1 - baseline1) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline1, baseline1, LARGE_ABSOLUTE_ERROR);
            assertEquals(mScale, baseline1, LARGE_ABSOLUTE_ERROR);

            if (Math.abs(estimatedBaseline2 - baseline2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(estimatedBaseline2, baseline2, LARGE_ABSOLUTE_ERROR);
            assertEquals(mScale2, baseline2, LARGE_ABSOLUTE_ERROR);

            // check cameras
            final PinholeCameraIntrinsicParameters estimatedIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters estimatedIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters estimatedIntrinsic2b =
                    estimatedEuclideanCamera2b.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters estimatedIntrinsic3 =
                    estimatedEuclideanCamera3.getIntrinsicParameters();

            final Rotation3D estimatedRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            final Rotation3D estimatedRotation2 = estimatedEuclideanCamera2.getCameraRotation();
            final Rotation3D estimatedRotation2b = estimatedEuclideanCamera2b.getCameraRotation();
            final Rotation3D estimatedRotation3 = estimatedEuclideanCamera3.getCameraRotation();

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

            assertEquals(estimatedIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic2b.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2b.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2b.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2b.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic2b.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(estimatedIntrinsic3.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic3.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic3.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic3.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(estimatedIntrinsic3.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(estimatedRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(estimatedRotation2.asInhomogeneousMatrix().equals(
                    estimatedRotation2b.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(estimatedRotation3.asInhomogeneousMatrix().equals(
                    rotation3.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            // check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPointsPair1; i++) {
                if (!points3DPair1.get(i).equals(
                        euclideanReconstructedPoints3DPair1.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3DPair1.get(i).equals(
                        euclideanReconstructedPoints3DPair1.get(i),
                        LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            for (int i = 0; i < numPointsPair2; i++) {
                if (!points3DPair2.get(i).equals(
                        euclideanReconstructedPoints3DPair2.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3DPair2.get(i).equals(
                        euclideanReconstructedPoints3DPair2.get(i),
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
    public void testCancel() throws GeometryException, AlgebraException {
        final PairedViewsSparseReconstructorConfiguration configuration =
                new PairedViewsSparseReconstructorConfiguration();
        configuration.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        configuration.setIntrinsicParametersKnown(true);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                MAX_FOCAL_LENGTH_ESSENTIAL);
        final double aspectRatio = configuration.getPairedCamerasAspectRatio();
        final double skewness = 0.0;
        final double principalPoint = 0.0;

        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                        principalPoint, principalPoint, skewness);
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

        // create 3D points laying in front of all cameras

        // 1st find an approximate central point by intersecting the axis planes of
        // all cameras
        final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
        final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
        final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
        final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
        final Plane horizontalPlane3 = camera3.getHorizontalAxisPlane();
        final Plane verticalPlane3 = camera3.getVerticalAxisPlane();
        final Matrix planesIntersectionMatrixPair1 = new Matrix(
                Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
        final Matrix planesIntersectionMatrixPair2 = new Matrix(
                Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
        planesIntersectionMatrixPair1.setElementAt(0, 0, verticalPlane1.getA());
        planesIntersectionMatrixPair1.setElementAt(0, 1, verticalPlane1.getB());
        planesIntersectionMatrixPair1.setElementAt(0, 2, verticalPlane1.getC());
        planesIntersectionMatrixPair1.setElementAt(0, 3, verticalPlane1.getD());

        planesIntersectionMatrixPair1.setElementAt(1, 0,
                horizontalPlane1.getA());
        planesIntersectionMatrixPair1.setElementAt(1, 1,
                horizontalPlane1.getB());
        planesIntersectionMatrixPair1.setElementAt(1, 2,
                horizontalPlane1.getC());
        planesIntersectionMatrixPair1.setElementAt(1, 3,
                horizontalPlane1.getD());

        planesIntersectionMatrixPair1.setElementAt(2, 0, verticalPlane2.getA());
        planesIntersectionMatrixPair1.setElementAt(2, 1, verticalPlane2.getB());
        planesIntersectionMatrixPair1.setElementAt(2, 2, verticalPlane2.getC());
        planesIntersectionMatrixPair1.setElementAt(2, 3, verticalPlane2.getD());

        planesIntersectionMatrixPair1.setElementAt(3, 0,
                horizontalPlane2.getA());
        planesIntersectionMatrixPair1.setElementAt(3, 1,
                horizontalPlane2.getB());
        planesIntersectionMatrixPair1.setElementAt(3, 2,
                horizontalPlane2.getC());
        planesIntersectionMatrixPair1.setElementAt(3, 3,
                horizontalPlane2.getD());

        planesIntersectionMatrixPair2.setElementAt(0, 0, verticalPlane2.getA());
        planesIntersectionMatrixPair2.setElementAt(0, 1, verticalPlane2.getB());
        planesIntersectionMatrixPair2.setElementAt(0, 2, verticalPlane2.getC());
        planesIntersectionMatrixPair2.setElementAt(0, 3, verticalPlane2.getD());

        planesIntersectionMatrixPair2.setElementAt(1, 0,
                horizontalPlane2.getA());
        planesIntersectionMatrixPair2.setElementAt(1, 1,
                horizontalPlane2.getB());
        planesIntersectionMatrixPair2.setElementAt(1, 2,
                horizontalPlane2.getC());
        planesIntersectionMatrixPair2.setElementAt(1, 3,
                horizontalPlane2.getD());

        planesIntersectionMatrixPair2.setElementAt(2, 0, verticalPlane3.getA());
        planesIntersectionMatrixPair2.setElementAt(2, 1, verticalPlane3.getB());
        planesIntersectionMatrixPair2.setElementAt(2, 2, verticalPlane3.getC());
        planesIntersectionMatrixPair2.setElementAt(2, 3, verticalPlane3.getD());

        planesIntersectionMatrixPair2.setElementAt(3, 0,
                horizontalPlane3.getA());
        planesIntersectionMatrixPair2.setElementAt(3, 1,
                horizontalPlane3.getB());
        planesIntersectionMatrixPair2.setElementAt(3, 2,
                horizontalPlane3.getC());
        planesIntersectionMatrixPair2.setElementAt(3, 2,
                horizontalPlane3.getD());

        final SingularValueDecomposer decomposerPair1 = new SingularValueDecomposer(
                planesIntersectionMatrixPair1);
        decomposerPair1.decompose();
        final Matrix vPair1 = decomposerPair1.getV();

        final SingularValueDecomposer decomposerPair2 = new SingularValueDecomposer(
                planesIntersectionMatrixPair2);
        decomposerPair2.decompose();
        final Matrix vPair2 = decomposerPair2.getV();

        final HomogeneousPoint3D centralCommonPointPair1 = new HomogeneousPoint3D(
                vPair1.getElementAt(0, 3),
                vPair1.getElementAt(1, 3),
                vPair1.getElementAt(2, 3),
                vPair1.getElementAt(3, 3));

        final HomogeneousPoint3D centralCommonPointPair2 = new HomogeneousPoint3D(
                vPair2.getElementAt(0, 3),
                vPair2.getElementAt(1, 3),
                vPair2.getElementAt(2, 3),
                vPair2.getElementAt(3, 3));

        double lambdaX;
        double lambdaY;
        double lambdaZ;

        final int numPointsPair1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
        final int numPointsPair2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

        InhomogeneousPoint3D point3D;
        Point2D projectedPoint1;
        Point2D projectedPoint2;
        Point2D projectedPoint3;
        final List<Point2D> projectedPoints1 = new ArrayList<>();
        final List<Point2D> projectedPoints2a = new ArrayList<>();
        final List<Point2D> projectedPoints2b = new ArrayList<>();
        final List<Point2D> projectedPoints3 = new ArrayList<>();
        boolean front1;
        boolean front2;
        boolean front3;
        for (int i = 0; i < numPointsPair1; i++) {
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

            // check that 3D point is in front of 1st pair of cameras
            //noinspection ConstantConditions
            assertTrue(front1);
            //noinspection ConstantConditions
            assertTrue(front2);
            //noinspection ConstantConditions
            assertTrue(front3);

            // project 3D point into 1st pair of cameras
            projectedPoint1 = new InhomogeneousPoint2D();
            camera1.project(point3D, projectedPoint1);
            projectedPoints1.add(projectedPoint1);

            projectedPoint2 = new InhomogeneousPoint2D();
            camera2.project(point3D, projectedPoint2);
            projectedPoints2a.add(projectedPoint2);
        }

        for (int i = 0; i < numPointsPair2; i++) {
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

            // check that 3D point is in front of 2nd pair of cameras
            //noinspection ConstantConditions
            assertTrue(front1);
            //noinspection ConstantConditions
            assertTrue(front2);
            //noinspection ConstantConditions
            assertTrue(front3);

            // project 3D point into 2nd pair of cameras
            projectedPoint2 = new InhomogeneousPoint2D();
            camera2.project(point3D, projectedPoint2);
            projectedPoints2b.add(projectedPoint2);

            projectedPoint3 = new InhomogeneousPoint2D();
            camera3.project(point3D, projectedPoint3);
            projectedPoints3.add(projectedPoint3);
        }

        final PairedViewsSparseReconstructorListener listener =
                new PairedViewsSparseReconstructorListener() {

                    @Override
                    public double onBaselineRequested(
                            final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                            final int viewId2, final EstimatedCamera metricCamera1,
                            final EstimatedCamera metricCamera2) {
                        final int viewCount = reconstructor.getViewCount();
                        if (viewCount == 0) {
                            return center1.distanceTo(center2);
                        } else if (viewCount == 2) {
                            return center2.distanceTo(center3);
                        }

                        return 1.0;
                    }

                    @Override
                    public boolean hasMoreViewsAvailable(
                            final PairedViewsSparseReconstructor reconstructor) {
                        reconstructor.cancel();
                        return mViewCount < 4; //3 views = 2 view pairs (2 images * 2 views --> 4 view counts)
                    }

                    @Override
                    public void onRequestSamplesForCurrentViewPair(
                            final PairedViewsSparseReconstructor reconstructor,
                            final int viewId1, final int viewId2, final List<Sample2D> samples1,
                            final List<Sample2D> samples2) {

                        samples1.clear();
                        samples2.clear();

                        final int viewCount = reconstructor.getViewCount();

                        Sample2D sample1;
                        Sample2D sample2;
                        if (viewCount == 0) {
                            // first view pair
                            for (int i = 0; i < numPointsPair1; i++) {
                                sample1 = new Sample2D();
                                sample1.setPoint(projectedPoints1.get(i));
                                sample1.setViewId(viewId1);
                                samples1.add(sample1);

                                sample2 = new Sample2D();
                                sample2.setPoint(projectedPoints2a.get(i));
                                sample2.setViewId(viewId2);
                                samples2.add(sample2);
                            }

                        } else if (viewCount == 2) {
                            // second view pair
                            for (int i = 0; i < numPointsPair2; i++) {
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
                            final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                            final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                        mViewCount += 2;
                    }

                    @Override
                    public void onSamplesRejected(
                            final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                            final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                        mViewCount += 2;
                    }

                    @Override
                    public void onRequestMatches(
                            final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                            final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
                            final List<MatchedSamples> matches) {
                        matches.clear();

                        final int viewCount = reconstructor.getViewCount();
                        final int numPoints;
                        if (viewCount == 0) {
                            // first view pair
                            numPoints = numPointsPair1;
                        } else {
                            // second view pair
                            numPoints = numPointsPair2;
                        }

                        MatchedSamples match;
                        for (int i = 0; i < numPoints; i++) {
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
                            final PairedViewsSparseReconstructor reconstructor,
                            final int viewId1, final int viewId2,
                            final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {

                        final int viewCount = reconstructor.getViewCount();
                        if (viewCount == 0) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        } else if (viewCount == 2) {
                            mEstimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                        }
                    }

                    @Override
                    public void onEuclideanCameraPairEstimated(
                            final PairedViewsSparseReconstructor reconstructor,
                            final int viewId1, final int viewId2, final double scale,
                            final EstimatedCamera camera1, final EstimatedCamera camera2) {

                        final int viewCount = reconstructor.getViewCount();
                        if (viewCount == 0) {
                            mEstimatedEuclideanCamera1 = camera1;
                            mEstimatedEuclideanCamera2 = camera2;
                            mScale = scale;
                        } else if (viewCount == 2) {
                            mEstimatedEuclideanCamera2b = camera1;
                            mEstimatedEuclideanCamera3 = camera2;
                            mScale2 = scale;
                        }
                    }

                    @Override
                    public void onEuclideanReconstructedPointsEstimated(
                            final PairedViewsSparseReconstructor reconstructor, final int viewId1,
                            final int viewId2, final double scale, final List<ReconstructedPoint3D> points) {

                        final int viewCount = reconstructor.getViewCount();
                        if (viewCount == 0) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        } else if (viewCount == 2) {
                            mEuclideanReconstructedPoints2 = points;
                            mScale2 = scale;
                        }
                    }

                    @Override
                    public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                            final PairedViewsSparseReconstructor reconstructor, final int viewId) {
                        return intrinsic;
                    }

                    @Override
                    public void onStart(final PairedViewsSparseReconstructor reconstructor) {
                        mStarted = true;
                    }

                    @Override
                    public void onFinish(final PairedViewsSparseReconstructor reconstructor) {
                        mFinished = true;
                    }

                    @Override
                    public void onCancel(final PairedViewsSparseReconstructor reconstructor) {
                        mCancelled = true;
                    }

                    @Override
                    public void onFail(final PairedViewsSparseReconstructor reconstructor) {
                        mFailed = true;
                    }
                };

        final PairedViewsSparseReconstructor reconstructor = new PairedViewsSparseReconstructor(
                configuration, listener);

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
        assertFalse(mFinished);
        assertTrue(mCancelled);
        assertFalse(mFailed);
        assertFalse(reconstructor.isFinished());
        assertFalse(reconstructor.isFirstViewPair());
        assertTrue(reconstructor.isAdditionalViewPair());
        assertTrue(reconstructor.getViewCount() > 0);
        assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
        assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix);
        assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
        assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
        assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
        assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera2);
        assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
        assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera1);
        assertNotNull(reconstructor.getMetricReconstructedPoints());
        assertNotNull(reconstructor.getEuclideanReconstructedPoints());
        assertSame(reconstructor.getEuclideanReconstructedPoints(), mEuclideanReconstructedPoints);
        assertEquals(reconstructor.getCurrentScale(), mScale, 0.0);
        assertNotNull(reconstructor.getPreviousViewSamples());
        assertNotNull(reconstructor.getCurrentViewSamples());
    }
    
    private void reset() {
        mViewCount = 0;
        mEstimatedFundamentalMatrix = mEstimatedFundamentalMatrix2 = null;
        mEstimatedEuclideanCamera1 = mEstimatedEuclideanCamera2
                = mEstimatedEuclideanCamera3 = null;
        mEuclideanReconstructedPoints = null;
        mStarted = mFinished = mFailed = mCancelled = false;
        mScale = 0.0;
    }
}
