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
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidPairOfCamerasException;
import com.irurueta.ar.slam.AbsoluteOrientationSlamCalibrator;
import com.irurueta.ar.slam.AbsoluteOrientationSlamEstimator;
import com.irurueta.geometry.*;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.jupiter.api.Assertions.*;

class AbsoluteOrientationSlamSparseReconstructorTest {

    private static final double MIN_FOCAL_LENGTH_ESSENTIAL = 750.0;
    private static final double MAX_FOCAL_LENGTH_ESSENTIAL = 1500.0;

    private static final double MIN_ANGLE_DEGREES = -30.0;
    private static final double MAX_ANGLE_DEGREES = -15.0;

    private static final double MIN_CAMERA_SEPARATION_ESSENTIAL = 500.0;
    private static final double MAX_CAMERA_SEPARATION_ESSENTIAL = 1000.0;

    private static final int MIN_NUM_POINTS = 25;
    private static final int MAX_NUM_POINTS = 50;

    private static final double MIN_LAMBDA_ESSENTIAL = -1000.0;
    private static final double MAX_LAMBDA_ESSENTIAL = 1000.0;

    private static final int TIMES = 500;
    private static final int MAX_TRIES = 5000;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;

    private static final int MIN_TRACKED_POINTS = 10;
    private static final double NEAREST_THRESHOLD = 1e-6;

    // 5% of relative error in scale estimation
    private static final double RELATIVE_ERROR = 0.05;

    private static final int MAX_CALIBRATION_SAMPLES = 10000;

    // conversion from milliseconds to nanoseconds
    private static final int MILLIS_TO_NANOS = 1000000;

    // time between samples expressed in nanoseconds (a typical sensor in Android
    // delivers a sample every 20ms)
    private static final int DELTA_NANOS = 20000000; // 0.02 seconds

    private static final float MIN_CALIBRATION_OFFSET = -1e-4f;
    private static final float MAX_CALIBRATION_OFFSET = 1e-4f;

    private static final double ACCELERATION_NOISE_STANDARD_DEVIATION = 1e-4;
    private static final double ANGULAR_SPEED_NOISE_STANDARD_DEVIATION = 1e-4;

    private static final int N_SENSOR_SAMPLES = 50;

    private static final Logger LOGGER = Logger.getLogger(
            AbsoluteOrientationSlamSparseReconstructorTest.class.getSimpleName());

    private int viewCount = 0;
    private EstimatedFundamentalMatrix estimatedFundamentalMatrix;
    private EstimatedFundamentalMatrix estimatedFundamentalMatrix2;
    private EstimatedFundamentalMatrix estimatedFundamentalMatrix3;
    private EstimatedCamera estimatedMetricCamera1;
    private EstimatedCamera estimatedMetricCamera2;
    private EstimatedCamera estimatedMetricCamera3;
    private EstimatedCamera estimatedMetricCamera4;
    private EstimatedCamera estimatedEuclideanCamera1;
    private EstimatedCamera estimatedEuclideanCamera2;
    private EstimatedCamera estimatedEuclideanCamera3;
    private EstimatedCamera estimatedEuclideanCamera4;
    private List<ReconstructedPoint3D> metricReconstructedPoints;
    private List<ReconstructedPoint3D> euclideanReconstructedPoints;

    private double scale;
    private double scale2;
    private double scale3;

    private boolean started;
    private boolean finished;
    private boolean failed;
    private boolean cancelled;

    private long timestamp;

    private int slamDataAvailable;
    private int slamCameraEstimated;

    private PinholeCamera slamCamera;
    private Matrix slamCovariance;

    @BeforeEach
    void setUp() {
        viewCount = 0;
        estimatedFundamentalMatrix = estimatedFundamentalMatrix2 = estimatedFundamentalMatrix3 = null;
        estimatedMetricCamera1 = estimatedMetricCamera2 = estimatedMetricCamera3 = estimatedMetricCamera4 = null;
        estimatedEuclideanCamera1 = estimatedEuclideanCamera2 = estimatedEuclideanCamera3 = estimatedEuclideanCamera4 =
                null;
        metricReconstructedPoints = null;
        euclideanReconstructedPoints = null;
        started = finished = failed = cancelled = false;
        timestamp = 0;
        slamDataAvailable = 0;
        slamCameraEstimated = 0;
        slamCamera = null;
        slamCovariance = null;
    }

    @Test
    void testConstructor() {
        assertEquals(2, AbsoluteOrientationSlamSparseReconstructor.MIN_NUMBER_OF_VIEWS);

        final var configuration = new AbsoluteOrientationSlamSparseReconstructorConfiguration();
        final var listener = new AbsoluteOrientationSlamSparseReconstructorListener() {
            @Override
            public void onSlamDataAvailable(
                    final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                    final double positionX, final double positionY, final double positionZ,
                    final double velocityX, final double velocityY, final double velocityZ,
                    final double accelerationX, final double accelerationY,
                    final double accelerationZ, final double quaternionA,
                    final double quaternionB, final double quaternionC,
                    final double quaternionD, final double angularSpeedX,
                    final double angularSpeedY, final double angularSpeedZ,
                    final Matrix covariance) {
                // no action needed
            }

            @Override
            public void onSlamCameraEstimated(
                    final AbsoluteOrientationSlamSparseReconstructor reconstructor, final PinholeCamera camera) {
                // no action needed
            }

            @Override
            public boolean hasMoreViewsAvailable(
                    final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                return false;
            }

            @Override
            public void onRequestSamples(
                    final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                    final int previousViewId, final int currentViewId,
                    final List<Sample2D> previousViewTrackedSamples,
                    final List<Sample2D> currentViewTrackedSamples,
                    final List<Sample2D> currentViewNewlySpawnedSamples) {
                // no action needed
            }

            @Override
            public void onSamplesAccepted(
                    final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int viewId,
                    final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples) {
                // no action needed
            }

            @Override
            public void onSamplesRejected(
                    final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int viewId,
                    final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples) {
                // no action needed
            }

            @Override
            public void onRequestMatches(
                    final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                    final List<Sample2D> allPreviousViewSamples, final List<Sample2D> previousViewTrackedSamples,
                    final List<Sample2D> currentViewTrackedSamples, final int previousViewId, final int currentViewId,
                    final List<MatchedSamples> matches) {
                // no action needed
            }

            @Override
            public void onFundamentalMatrixEstimated(
                    final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                    final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                // no action needed
            }

            @Override
            public void onMetricCameraEstimated(
                    final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int previousViewId,
                    final int currentViewId, final EstimatedCamera previousCamera,
                    final EstimatedCamera currentCamera) {
                // no action needed
            }

            @Override
            public void onMetricReconstructedPointsEstimated(
                    final AbsoluteOrientationSlamSparseReconstructor reconstructor, final List<MatchedSamples> matches,
                    final List<ReconstructedPoint3D> points) {
                // no action needed
            }

            @Override
            public void onEuclideanCameraEstimated(
                    final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int previousViewId,
                    final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                    final EstimatedCamera currentCamera) {
                // no action needed
            }

            @Override
            public void onEuclideanReconstructedPointsEstimated(
                    final AbsoluteOrientationSlamSparseReconstructor reconstructor, final double scale,
                    final List<ReconstructedPoint3D> points) {
                // no action needed
            }

            @Override
            public void onStart(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onFinish(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onCancel(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onFail(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                // no action needed
            }
        };

        // constructor with listener
        var reconstructor = new AbsoluteOrientationSlamSparseReconstructor(listener);

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
        reconstructor = new AbsoluteOrientationSlamSparseReconstructor(configuration, listener);

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
    void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithoutNoiseTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException,
            RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var configuration = new AbsoluteOrientationSlamSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final var accelerationOffsetX = 0.0f;
            final var accelerationOffsetY = 0.0f;
            final var accelerationOffsetZ = 0.0f;

            final var angularOffsetX = 0.0f;
            final var angularOffsetY = 0.0f;
            final var angularOffsetZ = 0.0f;

            final var calibrator = createFinishedCalibrator(accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final var calibrationData = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

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

            final var alphaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);
            final var axisRotation2 = new AxisRotation3D(rotation1.inverseRotationAndReturnNew().combineAndReturnNew(
                    rotation2));

            final var axisX = axisRotation2.getAxisX();
            final var axisY = axisRotation2.getAxisY();
            final var axisZ = axisRotation2.getAxisZ();
            final var angle = axisRotation2.getRotationAngle();

            final var diffRotation = new AxisRotation3D(axisX, axisY, axisZ, angle / N_SENSOR_SAMPLES);
            final var diffQuaternion = new Quaternion(diffRotation);

            // angular speeds (roll, pitch, yaw) on x, y, z axes
            final var angularSpeeds = diffQuaternion.toEulerAngles();
            final var angularSpeedX = angularSpeeds[0];
            final var angularSpeedY = angularSpeeds[1];
            final var angularSpeedZ = angularSpeeds[2];
            final var diffRotation2 = new Quaternion(angularSpeedX, angularSpeedY, angularSpeedZ);

            // number of samples (50 samples * 0.02 s/sample = 1 second)
            final var rotation2b = new MatrixRotation3D(rotation1);
            final var rotation2c = new MatrixRotation3D(rotation1);
            for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation2b.combine(diffRotation);
                rotation2c.combine(diffRotation2);
            }

            // check that rotations created by composing sensor samples are
            // equal to the original one
            assertTrue(rotation2.equals(rotation2b, ABSOLUTE_ERROR));
            assertTrue(rotation2.equals(rotation2c, ABSOLUTE_ERROR));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var rotationTransformation = new EuclideanTransformation3D(rotation1);
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            center1 = rotationTransformation.transformAndReturnNew(center1);
            center2 = rotationTransformation.transformAndReturnNew(center2);

            final var baseline = center1.distanceTo(center2);

            final double accelerationX;
            final double accelerationY;
            final double accelerationZ;

            // s = 0.5*a*t^2 --> a = 2*s/t^2
            // assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ = 2 * cameraSeparation;

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
                assertTrue(front1);
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

                points3D1.add(point3D);

                // check that 3D point is in front of both cameras
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            if (maxTriesReached) {
                continue;
            }

            final var listener = new AbsoluteOrientationSlamSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final double positionX, final double positionY, final double positionZ,
                        final double velocityX, final double velocityY, final double velocityZ,
                        final double accelerationX, final double accelerationY,
                        final double accelerationZ, final double quaternionA,
                        final double quaternionB, final double quaternionC,
                        final double quaternionD, final double angularSpeedX,
                        final double angularSpeedY, final double angularSpeedZ,
                        final Matrix covariance) {
                    slamDataAvailable++;
                    slamCovariance = covariance;
                }

                @Override
                public void onSlamCameraEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final PinholeCamera camera) {
                    slamCameraEstimated++;
                    slamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamples(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
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

                        // assume the following accelerator and gyroscope samples
                        // are obtained during a period of 1 second between 1st
                        // and 2nd view (50 samples * 0.02 s/sample = 1 second)
                        timestamp = 0;
                        final var orientation = new Quaternion(rotation1);
                        for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                            reconstructor.updateAccelerometerSample(timestamp, (float) accelerationX,
                                    (float) accelerationY, (float) accelerationZ);
                            reconstructor.updateGyroscopeSample(timestamp, (float) angularSpeedX,
                                    (float) angularSpeedY, (float) angularSpeedZ);
                            reconstructor.updateOrientationSample(timestamp, orientation);
                            // update orientation
                            orientation.combine(diffQuaternion);
                            timestamp += DELTA_NANOS;
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final List<Sample2D> allPreviousViewSamples, final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples, final int previousViewId,
                        final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints1; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{previousViewTrackedSamples.get(i),
                                currentViewTrackedSamples.get(i)});
                        match.setViewIds(new int[]{previousViewId, currentViewId});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    AbsoluteOrientationSlamSparseReconstructorTest.this.estimatedFundamentalMatrix =
                            estimatedFundamentalMatrix;
                }

                @Override
                public void onMetricCameraEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final int previousViewId, final int currentViewId, final EstimatedCamera previousCamera,
                        final EstimatedCamera currentCamera) {
                    estimatedMetricCamera1 = previousCamera;
                    estimatedMetricCamera2 = currentCamera;
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int previousViewId,
                        final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                        final EstimatedCamera currentCamera) {
                    estimatedEuclideanCamera1 = previousCamera;
                    estimatedEuclideanCamera2 = currentCamera;
                    AbsoluteOrientationSlamSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    AbsoluteOrientationSlamSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onStart(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new AbsoluteOrientationSlamSparseReconstructor(configuration, listener);

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
            assertTrue(slamDataAvailable > 0);
            assertTrue(slamCameraEstimated > 0);
            assertNotNull(slamCamera);
            assertNotNull(slamCovariance);
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
            assertNotSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertNotSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);

            final var estEuclideanCam1 = this.estimatedEuclideanCamera1.getCamera();
            final var estEuclideanCam2 = this.estimatedEuclideanCamera2.getCamera();

            estMetricCam1.decompose();
            estMetricCam2.decompose();

            estEuclideanCam1.decompose();
            estEuclideanCam2.decompose();

            assertNotSame(metricReconstructedPoints, euclideanReconstructedPoints);

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            final var euclideanReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
                euclideanReconstructedPoints3D.add(euclideanReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints1; i++) {
                final var p = metricReconstructedPoints3D.get(i);
                final var pe = euclideanReconstructedPoints3D.get(i);

                assertTrue(estMetricCam1.isPointInFrontOfCamera(p));
                assertTrue(estMetricCam2.isPointInFrontOfCamera(p));

                assertTrue(estEuclideanCam1.isPointInFrontOfCamera(pe));
                assertTrue(estEuclideanCam2.isPointInFrontOfCamera(pe));
            }

            final var euclideanCenter1 = estEuclideanCam1.getCameraCenter();
            final var euclideanCenter2 = estEuclideanCam2.getCameraCenter();

            final var euclideanIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var euclideanIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();

            final var euclideanRotation1 = estEuclideanCam1.getCameraRotation();
            final var euclideanRotation2 = estEuclideanCam2.getCameraRotation();

            final var estimatedBaseline = euclideanCenter1.distanceTo(euclideanCenter2);

            // check cameras are correct
            final var maxBaseline = Math.max(estimatedBaseline, baseline);
            final var absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);

            assertTrue(center1.equals(euclideanCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(euclideanCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2, absoluteScaleError));

            assertEquals(euclideanIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(euclideanRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct (after scale correction)

            // check that scale error is less than 5%
            assertTrue(Math.abs(baseline / scale - 1.0) < RELATIVE_ERROR);

            final var scaleAndOrientationTransformation = new MetricTransformation3D(scale);
            scaleAndOrientationTransformation.setRotation(rotation1.inverseRotationAndReturnNew());

            var numValidPoints = 0;
            double scaleX;
            double scaleY;
            double scaleZ;
            for (var i = 0; i < numPoints1; i++) {
                final var point = points3D1.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3D.get(i);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleAndOrientationTransformation.transform(metricReconstructedPoints3D.get(i), rescaledPoint);

                // Euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                // check that scale error is less than 5%
                assertEquals(scaleX, baseline / scale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale, LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR || Math.abs(scaleY - 1.0) > RELATIVE_ERROR
                        || Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(rescaledPoint.getInhomX() * baseline / scale,
                        rescaledPoint.getInhomY() * baseline / scale,
                        rescaledPoint.getInhomZ() * baseline / scale);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                // check Euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                // check that scale error is less than 5%
                assertEquals(scaleX, baseline / scale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale, LARGE_ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            if (numValidPoints == 0) {
                continue;
            }

            final double scaleRelativeError = Math.abs(baseline / scale - 1.0);
            LOGGER.log(Level.INFO, "Baseline relative error without noise: {0,number,0.000%}",
                    scaleRelativeError);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithNoiseTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException,
            RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var offsetRandomizer = new UniformRandomizer();
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var configuration = new AbsoluteOrientationSlamSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            var accelerationOffsetX = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            var accelerationOffsetY = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            var accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            var angularOffsetX = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            var angularOffsetY = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            var angularOffsetZ = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final var calibrator = createFinishedCalibrator(accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final var calibrationData = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

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

            final var alphaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);
            final var axisRotation2 = new AxisRotation3D(rotation1.inverseRotationAndReturnNew().combineAndReturnNew(
                    rotation2));

            final var axisX = axisRotation2.getAxisX();
            final var axisY = axisRotation2.getAxisY();
            final var axisZ = axisRotation2.getAxisZ();
            final var angle = axisRotation2.getRotationAngle();

            final var diffRotation = new AxisRotation3D(axisX, axisY, axisZ, angle / N_SENSOR_SAMPLES);
            final var diffQuaternion = new Quaternion(diffRotation);

            // angular speeds (roll, pitch, yaw) on x, y, z axes
            final var angularSpeeds = diffQuaternion.toEulerAngles();
            final var angularSpeedX = angularSpeeds[0];
            final var angularSpeedY = angularSpeeds[1];
            final var angularSpeedZ = angularSpeeds[2];
            final var diffRotation2 = new Quaternion(angularSpeedX, angularSpeedY, angularSpeedZ);

            // number of samples (50 samples * 0.02 s/sample = 1 second)
            final var rotation2b = new MatrixRotation3D(rotation1);
            final var rotation2c = new MatrixRotation3D(rotation1);
            for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation2b.combine(diffRotation);
                rotation2c.combine(diffRotation2);
            }

            // check that rotations created by composing sensor samples are
            // equal to the original one
            if (!rotation2.equals(rotation2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(rotation2.equals(rotation2b, ABSOLUTE_ERROR));
            if (!rotation2.equals(rotation2c, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(rotation2.equals(rotation2c, ABSOLUTE_ERROR));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var rotationTransformation = new EuclideanTransformation3D(rotation1);
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            center1 = rotationTransformation.transformAndReturnNew(center1);
            center2 = rotationTransformation.transformAndReturnNew(center2);

            final var baseline = center1.distanceTo(center2);

            final double accelerationX;
            final double accelerationY;
            final double accelerationZ;

            // s = 0.5*a*t^2 --> a = 2*s/t^2
            // assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ = 2 * cameraSeparation;

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
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY, centralCommonPoint.getInhomZ() + lambdaZ);

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
                assertTrue(front1);
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
            final var projectedPoints2b = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY, centralCommonPoint.getInhomZ() + lambdaZ);

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
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            if (maxTriesReached) {
                continue;
            }

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            final var listener = new AbsoluteOrientationSlamSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final double positionX, final double positionY, final double positionZ,
                        final double velocityX, final double velocityY, final double velocityZ,
                        final double accelerationX, final double accelerationY, final double accelerationZ,
                        final double quaternionA, final double quaternionB, final double quaternionC,
                        final double quaternionD, final double angularSpeedX, final double angularSpeedY,
                        final double angularSpeedZ, final Matrix covariance) {
                    slamDataAvailable++;
                    slamCovariance = covariance;
                }

                @Override
                public void onSlamCameraEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final PinholeCamera camera) {
                    slamCameraEstimated++;
                    slamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamples(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int previousViewId,
                        final int currentViewId, final List<Sample2D> previousViewTrackedSamples,
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

                        // assume the following accelerator and gyroscope samples
                        // are obtained during a period of 1 second between 1st
                        // and 2nd view (50 samples * 0.02 s/sample = 1 second)
                        timestamp = 0;
                        float noiseAccelerationX;
                        float noiseAccelerationY;
                        float noiseAccelerationZ;
                        float noiseAngularSpeedX;
                        float noiseAngularSpeedY;
                        float noiseAngularSpeedZ;

                        float accelerationWithNoiseX;
                        float accelerationWithNoiseY;
                        float accelerationWithNoiseZ;
                        float angularSpeedWithNoiseX;
                        float angularSpeedWithNoiseY;
                        float angularSpeedWithNoiseZ;

                        final var accelerationWithNoise = new float[3];
                        final var angularSpeedWithNoise = new float[3];

                        final var orientation = new Quaternion(rotation1);
                        for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                            noiseAccelerationX = accelerationRandomizer.nextFloat();
                            noiseAccelerationY = accelerationRandomizer.nextFloat();
                            noiseAccelerationZ = accelerationRandomizer.nextFloat();

                            noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                            noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                            noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                            accelerationWithNoiseX = (float) accelerationX + noiseAccelerationX;
                            accelerationWithNoiseY = (float) accelerationY + noiseAccelerationY;
                            accelerationWithNoiseZ = (float) accelerationZ + noiseAccelerationZ;
                            accelerationWithNoise[0] = accelerationWithNoiseX;
                            accelerationWithNoise[1] = accelerationWithNoiseY;
                            accelerationWithNoise[2] = accelerationWithNoiseZ;

                            angularSpeedWithNoiseX = (float) angularSpeedX + noiseAngularSpeedX;
                            angularSpeedWithNoiseY = (float) angularSpeedY + noiseAngularSpeedY;
                            angularSpeedWithNoiseZ = (float) angularSpeedZ + noiseAngularSpeedZ;
                            angularSpeedWithNoise[0] = angularSpeedWithNoiseX;
                            angularSpeedWithNoise[1] = angularSpeedWithNoiseY;
                            angularSpeedWithNoise[2] = angularSpeedWithNoiseZ;

                            reconstructor.updateAccelerometerSample(timestamp, accelerationWithNoise);
                            reconstructor.updateGyroscopeSample(timestamp, angularSpeedWithNoise);
                            reconstructor.updateOrientationSample(timestamp, orientation);
                            // update orientation
                            orientation.combine(diffQuaternion);
                            timestamp += DELTA_NANOS;
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final List<Sample2D> allPreviousViewSamples, final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples, final int previousViewId,
                        final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints1; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{previousViewTrackedSamples.get(i),
                                currentViewTrackedSamples.get(i)});
                        match.setViewIds(new int[]{previousViewId, currentViewId});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    AbsoluteOrientationSlamSparseReconstructorTest.this.estimatedFundamentalMatrix =
                            estimatedFundamentalMatrix;
                }

                @Override
                public void onMetricCameraEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int previousViewId,
                        final int currentViewId, final EstimatedCamera previousCamera,
                        final EstimatedCamera currentCamera) {
                    estimatedMetricCamera1 = previousCamera;
                    estimatedMetricCamera2 = currentCamera;
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int previousViewId,
                        final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                        final EstimatedCamera currentCamera) {
                    estimatedEuclideanCamera1 = previousCamera;
                    estimatedEuclideanCamera2 = currentCamera;
                    AbsoluteOrientationSlamSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    AbsoluteOrientationSlamSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onStart(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new AbsoluteOrientationSlamSparseReconstructor(configuration, listener);

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
            assertTrue(slamDataAvailable > 0);
            assertTrue(slamCameraEstimated > 0);
            assertNotNull(slamCamera);
            assertNotNull(slamCovariance);
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
            assertNotSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertNotSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);

            final var estEuclideanCam1 = this.estimatedEuclideanCamera1.getCamera();
            final var estEuclideanCam2 = this.estimatedEuclideanCamera2.getCamera();

            estMetricCam1.decompose();
            estMetricCam2.decompose();

            estEuclideanCam1.decompose();
            estEuclideanCam2.decompose();

            assertNotSame(metricReconstructedPoints, euclideanReconstructedPoints);

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            final var euclideanReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numPoints1; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
                euclideanReconstructedPoints3D.add(euclideanReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numPoints1; i++) {
                final var p = metricReconstructedPoints3D.get(i);
                final var pe = euclideanReconstructedPoints3D.get(i);

                assertTrue(estMetricCam1.isPointInFrontOfCamera(p));
                assertTrue(estMetricCam2.isPointInFrontOfCamera(p));

                assertTrue(estEuclideanCam1.isPointInFrontOfCamera(pe));
                assertTrue(estEuclideanCam2.isPointInFrontOfCamera(pe));
            }

            final var euclideanCenter1 = estEuclideanCam1.getCameraCenter();
            final var euclideanCenter2 = estEuclideanCam2.getCameraCenter();

            final var euclideanIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var euclideanIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();

            final var euclideanRotation1 = estEuclideanCam1.getCameraRotation();
            final var euclideanRotation2 = estEuclideanCam2.getCameraRotation();

            final var estimatedBaseline = euclideanCenter1.distanceTo(euclideanCenter2);

            // check cameras are correct
            final var maxBaseline = Math.max(estimatedBaseline, baseline);
            final var absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);

            assertTrue(center1.equals(euclideanCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(euclideanCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2, absoluteScaleError));

            assertEquals(euclideanIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(euclideanRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct (after scale correction)

            // check that scale error is less than 5%
            assertTrue(Math.abs(baseline / scale - 1.0) < RELATIVE_ERROR);

            final var scaleAndOrientationTransformation = new MetricTransformation3D(scale);
            scaleAndOrientationTransformation.setRotation(rotation1.inverseRotationAndReturnNew());

            var numValidPoints = 0;
            double scaleX;
            double scaleY;
            double scaleZ;
            for (var i = 0; i < numPoints1; i++) {
                final var point = points3D1.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3D.get(i);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleAndOrientationTransformation.transform(metricReconstructedPoints3D.get(i), rescaledPoint);

                // Euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                // check that scale error is less than 5%
                assertEquals(scaleX, baseline / scale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale, LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR || Math.abs(scaleY - 1.0) > RELATIVE_ERROR
                        || Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(rescaledPoint.getInhomX() * baseline / scale,
                        rescaledPoint.getInhomY() * baseline / scale,
                        rescaledPoint.getInhomZ() * baseline / scale);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                // check Euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                // check that scale error is less than 5%
                assertEquals(scaleX, baseline / scale, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale, ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            if (numValidPoints == 0) {
                continue;
            }

            final double scaleRelativeError = Math.abs(baseline / scale - 1.0);
            LOGGER.log(Level.INFO, "Baseline relative error without noise: {0,number,0.000%}", scaleRelativeError);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithoutNoiseThreeViews()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException,
            RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var configuration = new AbsoluteOrientationSlamSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final var accelerationOffsetX = 0.0f;
            final var accelerationOffsetY = 0.0f;
            final var accelerationOffsetZ = 0.0f;

            final var angularOffsetX = 0.0f;
            final var angularOffsetY = 0.0f;
            final var angularOffsetZ = 0.0f;

            final var calibrator = createFinishedCalibrator(accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final var calibrationData = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

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

            final var alphaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var alphaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);
            final var axisRotation2 = new AxisRotation3D(rotation1.inverseRotationAndReturnNew().combineAndReturnNew(
                    rotation2));

            final var rotation3 = new MatrixRotation3D(alphaEuler3, betaEuler3, gammaEuler3);

            final var axis2X = axisRotation2.getAxisX();
            final var axis2Y = axisRotation2.getAxisY();
            final var axis2Z = axisRotation2.getAxisZ();
            final var angle2 = axisRotation2.getRotationAngle();

            var diffRotation = new AxisRotation3D(axis2X, axis2Y, axis2Z, angle2 / N_SENSOR_SAMPLES);
            var diffQuaternion = new Quaternion(diffRotation);

            // angular speeds (roll, pitch, yaw) on x, y, z axes
            var angularSpeeds = diffQuaternion.toEulerAngles();
            final var angularSpeed2X = angularSpeeds[0];
            final var angularSpeed2Y = angularSpeeds[1];
            final var angularSpeed2Z = angularSpeeds[2];
            final var diffRotation2 = new Quaternion(angularSpeed2X, angularSpeed2Y, angularSpeed2Z);

            // number of samples (50 samples * 0.02 s/sample = 1 second)
            final var rotation2b = new MatrixRotation3D(rotation1);
            final var rotation2c = new MatrixRotation3D(rotation1);
            for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation2b.combine(diffRotation);
                rotation2c.combine(diffRotation2);
            }

            // check that rotations created by composing sensor samples are
            // equal to the original one
            assertTrue(rotation2.equals(rotation2b, ABSOLUTE_ERROR));
            assertTrue(rotation2.equals(rotation2c, ABSOLUTE_ERROR));

            final var accumDiffRotation = rotation2.inverseRotationAndReturnNew().combineAndReturnNew(rotation3)
                    .toAxisRotation();
            final var axis3X = accumDiffRotation.getAxisX();
            final var axis3Y = accumDiffRotation.getAxisY();
            final var axis3Z = accumDiffRotation.getAxisZ();
            final var angle3 = accumDiffRotation.getRotationAngle();

            diffRotation = new AxisRotation3D(axis3X, axis3Y, axis3Z, angle3 / N_SENSOR_SAMPLES);
            diffQuaternion = new Quaternion(diffRotation);

            // angular speeds (roll, pitch, yaw) on x, y, z axes
            angularSpeeds = diffQuaternion.toEulerAngles();
            final var angularSpeed3X = angularSpeeds[0];
            final var angularSpeed3Y = angularSpeeds[1];
            final var angularSpeed3Z = angularSpeeds[2];
            final var diffRotation3 = new Quaternion(angularSpeed3X, angularSpeed3Y, angularSpeed3Z);

            // number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            // previously sampled rotation
            final var rotation3b = new MatrixRotation3D(rotation2b);
            final var rotation3c = new MatrixRotation3D(rotation2c);
            for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation3b.combine(diffRotation);
                rotation3c.combine(diffRotation3);
            }

            // check that rotations created by composing sensor samples are equal
            // to the original one
            assertTrue(rotation3.equals(rotation3b, ABSOLUTE_ERROR));
            assertTrue(rotation3.equals(rotation3c, ABSOLUTE_ERROR));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            final var cameraSeparation2 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var rotationTransformation = new EuclideanTransformation3D(rotation1);
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            Point3D center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);
            center1 = rotationTransformation.transformAndReturnNew(center1);
            center2 = rotationTransformation.transformAndReturnNew(center2);
            center3 = rotationTransformation.transformAndReturnNew(center3);

            final var baseline = center1.distanceTo(center2);
            final var baseline2 = center2.distanceTo(center3);

            final double accelerationX;
            final double accelerationY;
            final double accelerationZ;
            final double accelerationX2;
            final double accelerationY2;
            final double accelerationZ2;

            // s = 0.5*a*t^2 --> a = 2*s/t^2
            // assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ = 2 * cameraSeparation;
            accelerationX2 = accelerationY2 = accelerationZ2 = 2 * cameraSeparation2;

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);
            final var camera3 = new PinholeCamera(intrinsic, rotation3, center3);

            final var fundamentalMatrix1 = new FundamentalMatrix(camera1, camera2);

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
            var maxTriesReached = false;
            for (var i = 0; i < numPoints1; i++) {
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
                assertTrue(front1);
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

                    point3D = new InhomogeneousPoint3D(centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY, centralCommonPoint.getInhomZ() + lambdaZ);

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

                points3D2.add(point3D);

                // check that 3D point is in front of both cameras
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

            final var listener = new AbsoluteOrientationSlamSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final double positionX, final double positionY, final double positionZ,
                        final double velocityX, final double velocityY, final double velocityZ,
                        final double accelerationX, final double accelerationY, final double accelerationZ,
                        final double quaternionA, final double quaternionB, final double quaternionC,
                        final double quaternionD, final double angularSpeedX, final double angularSpeedY,
                        final double angularSpeedZ, final Matrix covariance) {
                    slamDataAvailable++;
                    slamCovariance = covariance;
                }

                @Override
                public void onSlamCameraEstimated(final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                                                  final PinholeCamera camera) {
                    slamCameraEstimated++;
                    slamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    return viewCount < 3;
                }

                @Override
                public void onRequestSamples(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int previousViewId,
                        final int currentViewId, final List<Sample2D> previousViewTrackedSamples,
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

                        // assume the following accelerator and gyroscope samples
                        // are obtained during a period of 1 second between 1st
                        // and 2nd view (50 samples * 0.02 s/sample = 1 second)
                        timestamp = 0;
                        final var orientation = new Quaternion(rotation1);
                        for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                            reconstructor.updateAccelerometerSample(timestamp, (float) accelerationX,
                                    (float) accelerationY, (float) accelerationZ);
                            reconstructor.updateGyroscopeSample(timestamp, (float) angularSpeed2X,
                                    (float) angularSpeed2Y, (float) angularSpeed2Z);
                            reconstructor.updateOrientationSample(timestamp, orientation);
                            // update orientation
                            orientation.combine(diffRotation2);
                            timestamp += DELTA_NANOS;
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

                        // assume the following accelerator and gyroscope samples
                        // are obtained during a period of 1 second between 2nd
                        // and 3rd view (50 samples * 0.02 s/sample = 1 second)
                        final var orientation = new Quaternion(rotation2);
                        for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                            reconstructor.updateAccelerometerSample(timestamp, (float) accelerationX2,
                                    (float) accelerationY2, (float) accelerationZ2);
                            reconstructor.updateGyroscopeSample(timestamp, (float) angularSpeed3X,
                                    (float) angularSpeed3Y, (float) angularSpeed3Z);
                            reconstructor.updateOrientationSample(timestamp, orientation);
                            // update orientation
                            orientation.combine(diffRotation3);
                            timestamp += DELTA_NANOS;
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final List<Sample2D> allPreviousViewSamples, final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples, final int previousViewId,
                        final int currentViewId, final List<MatchedSamples> matches) {
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
                        match.setSamples(new Sample2D[]{previousSample, currentSample});
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
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    if (AbsoluteOrientationSlamSparseReconstructorTest.this.estimatedFundamentalMatrix == null) {
                        AbsoluteOrientationSlamSparseReconstructorTest.this.estimatedFundamentalMatrix =
                                estimatedFundamentalMatrix;
                    } else if (estimatedFundamentalMatrix2 == null) {
                        estimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                    }
                }

                @Override
                public void onMetricCameraEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int previousViewId,
                        final int currentViewId, final EstimatedCamera previousCamera,
                        final EstimatedCamera currentCamera) {
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
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int previousViewId,
                        final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                        final EstimatedCamera currentCamera) {
                    if (estimatedEuclideanCamera2 == null) {
                        estimatedEuclideanCamera1 = previousCamera;
                        estimatedEuclideanCamera2 = currentCamera;
                        AbsoluteOrientationSlamSparseReconstructorTest.this.scale = scale;
                    } else if (estimatedEuclideanCamera3 == null) {
                        estimatedEuclideanCamera2 = previousCamera;
                        estimatedEuclideanCamera3 = currentCamera;
                        scale2 = scale;
                    }
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    if (euclideanReconstructedPoints == null) {
                        AbsoluteOrientationSlamSparseReconstructorTest.this.scale = scale;
                    } else {
                        scale2 = scale;
                    }

                    euclideanReconstructedPoints = points;
                }

                @Override
                public void onStart(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new AbsoluteOrientationSlamSparseReconstructor(configuration, listener);

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
            assertTrue(slamDataAvailable > 0);
            assertTrue(slamCameraEstimated > 0);
            assertNotNull(slamCamera);
            assertNotNull(slamCovariance);
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            if (estimatedMetricCamera3 == null) {
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

            // check that estimated fundamental matrix is correct
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
            assertNotSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertNotSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);
            assertNotSame(this.estimatedMetricCamera3, estimatedEuclideanCamera3);

            final var estEuclideanCam1 = this.estimatedEuclideanCamera1.getCamera();
            final var estEuclideanCam2 = this.estimatedEuclideanCamera2.getCamera();
            final var estEuclideanCam3 = this.estimatedEuclideanCamera3.getCamera();

            estMetricCam1.decompose();
            estMetricCam2.decompose();
            estMetricCam3.decompose();

            estEuclideanCam1.decompose();
            estEuclideanCam2.decompose();
            estEuclideanCam3.decompose();

            assertNotSame(metricReconstructedPoints, euclideanReconstructedPoints);

            final var numReconstructedPoints = numPoints1 - start + numPoints2;

            if (metricReconstructedPoints.size() != numReconstructedPoints) {
                continue;
            }

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            final var euclideanReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numReconstructedPoints; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
                euclideanReconstructedPoints3D.add(euclideanReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numReconstructedPoints; i++) {
                final var p = metricReconstructedPoints3D.get(i);
                final var pe = euclideanReconstructedPoints3D.get(i);

                assertTrue(estMetricCam1.isPointInFrontOfCamera(p));
                assertTrue(estMetricCam2.isPointInFrontOfCamera(p));

                assertTrue(estEuclideanCam1.isPointInFrontOfCamera(pe));
                assertTrue(estEuclideanCam2.isPointInFrontOfCamera(pe));
            }

            final var euclideanCenter1 = estEuclideanCam1.getCameraCenter();
            final var euclideanCenter2 = estEuclideanCam2.getCameraCenter();
            final var euclideanCenter3 = estEuclideanCam3.getCameraCenter();

            final var euclideanIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var euclideanIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();
            final var euclideanIntrinsic3 = estEuclideanCam3.getIntrinsicParameters();

            final var euclideanRotation1 = estEuclideanCam1.getCameraRotation();
            final var euclideanRotation2 = estEuclideanCam2.getCameraRotation();
            final var euclideanRotation3 = estEuclideanCam3.getCameraRotation();

            final var estimatedBaseline = euclideanCenter1.distanceTo(euclideanCenter2);
            final var estimatedBaseline2 = euclideanCenter2.distanceTo(euclideanCenter3);

            // check cameras are correct
            final var maxBaseline = Math.max(estimatedBaseline, baseline);
            final var absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);

            final var maxBaseline2 = Math.max(estimatedBaseline2, baseline2);
            final var absoluteScaleError2 = RELATIVE_ERROR * maxBaseline2;
            if (Math.abs(estimatedBaseline2 - baseline2) > absoluteScaleError2) {
                continue;
            }
            assertEquals(estimatedBaseline2, baseline2, absoluteScaleError2);

            assertTrue(center1.equals(euclideanCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(euclideanCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2, absoluteScaleError));
            if (!center3.equals(euclideanCenter3, absoluteScaleError2)) {
                continue;
            }
            assertTrue(center3.equals(euclideanCenter3, absoluteScaleError2));

            assertEquals(euclideanIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic3.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            if (!euclideanRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(euclideanRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            if (!euclideanRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(euclideanRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            if (!euclideanRotation3.asInhomogeneousMatrix().equals(rotation3.asInhomogeneousMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(euclideanRotation3.asInhomogeneousMatrix().equals(rotation3.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct (up to 5% scale error)

            // check that scale error is less than 5%
            assertTrue(Math.abs(baseline / scale - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline / scale2 - 1.0) < RELATIVE_ERROR);

            final var scaleAndOrientationTransformation = new MetricTransformation3D(scale2);
            scaleAndOrientationTransformation.setRotation(rotation1.inverseRotationAndReturnNew());

            var numValidPoints = 0;
            double scaleX;
            double scaleY;
            double scaleZ;
            for (var i = start; i < numPoints1; i++) {
                final var point = points3D1.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3D.get(i - start);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleAndOrientationTransformation.transform(metricReconstructedPoints3D.get(i - start), rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                // check that scale error is less than 5%
                assertEquals(scaleX, baseline / scale2, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale2, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale2, ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR || Math.abs(scaleY - 1.0) > RELATIVE_ERROR
                        || Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(rescaledPoint.getInhomX() * baseline / scale2,
                        rescaledPoint.getInhomY() * baseline / scale2,
                        rescaledPoint.getInhomZ() * baseline / scale2);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                // check Euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                // check that scale error is less than 5%
                assertEquals(scaleX, baseline / scale2, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale2, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale2, ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            for (var i = 0; i < numPoints2; i++) {
                final var point = points3D2.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3D.get(i + numPoints1 - start);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleAndOrientationTransformation.transform(metricReconstructedPoints3D.get(i + numPoints1 - start),
                        rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                // check that scale error is less than 5%
                assertEquals(scaleX, baseline / scale2, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale2, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale2, ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR || Math.abs(scaleY - 1.0) > RELATIVE_ERROR
                        || Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(rescaledPoint.getInhomX() * baseline / scale2,
                        rescaledPoint.getInhomY() * baseline / scale2,
                        rescaledPoint.getInhomZ() * baseline / scale2);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                // check Euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                // check that scale error is less than 5%
                assertEquals(scaleX, baseline / scale2, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale2, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale2, ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            if (numValidPoints == 0) {
                continue;
            }

            var scaleRelativeError = Math.abs(baseline / scale2 - 1.0);
            LOGGER.log(Level.INFO, "Baseline relative error without noise: {0,number,0.000%}", scaleRelativeError);

            // check scales
            final var maxScale = Math.max(scale, scale2);
            scaleRelativeError = RELATIVE_ERROR * maxScale;
            if (Math.abs(scale - scale2) > scaleRelativeError) {
                continue;
            }
            assertEquals(scale, scale2, scaleRelativeError);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithNoiseThreeViews()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException,
            RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var offsetRandomizer = new UniformRandomizer();
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var configuration = new AbsoluteOrientationSlamSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final var accelerationOffsetX = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var accelerationOffsetY = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final var angularOffsetX = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var angularOffsetY = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var angularOffsetZ = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final var calibrator = createFinishedCalibrator(accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final var calibrationData = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

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

            final var alphaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var alphaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);
            final var axisRotation2 = new AxisRotation3D(rotation1.inverseRotationAndReturnNew().combineAndReturnNew(
                    rotation2));

            final var rotation3 = new MatrixRotation3D(alphaEuler3, betaEuler3, gammaEuler3);

            final var axis2X = axisRotation2.getAxisX();
            final var axis2Y = axisRotation2.getAxisY();
            final var axis2Z = axisRotation2.getAxisZ();
            final var angle2 = axisRotation2.getRotationAngle();

            var diffRotation = new AxisRotation3D(axis2X, axis2Y, axis2Z, angle2 / N_SENSOR_SAMPLES);
            var diffQuaternion = new Quaternion(diffRotation);

            // angular speeds (roll, pitch, yaw) on x, y, z axes
            var angularSpeeds = diffQuaternion.toEulerAngles();
            final var angularSpeed2X = angularSpeeds[0];
            final var angularSpeed2Y = angularSpeeds[1];
            final var angularSpeed2Z = angularSpeeds[2];
            final var diffRotation2 = new Quaternion(angularSpeed2X, angularSpeed2Y, angularSpeed2Z);

            // number of samples (50 samples * 0.02 s/sample = 1 second)
            final var rotation2b = new MatrixRotation3D(rotation1);
            final var rotation2c = new MatrixRotation3D(rotation1);
            for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation2b.combine(diffRotation);
                rotation2c.combine(diffRotation2);
            }

            // check that rotations created by composing sensor samples are
            // equal to the original one
            assertTrue(rotation2.equals(rotation2b, ABSOLUTE_ERROR));
            assertTrue(rotation2.equals(rotation2c, ABSOLUTE_ERROR));

            final var accumDiffRotation = rotation2.inverseRotationAndReturnNew().combineAndReturnNew(rotation3)
                    .toAxisRotation();
            final var axis3X = accumDiffRotation.getAxisX();
            final var axis3Y = accumDiffRotation.getAxisY();
            final var axis3Z = accumDiffRotation.getAxisZ();
            final var angle3 = accumDiffRotation.getRotationAngle();

            diffRotation = new AxisRotation3D(axis3X, axis3Y, axis3Z, angle3 / N_SENSOR_SAMPLES);
            diffQuaternion = new Quaternion(diffRotation);

            // angular speeds (roll, pitch, yaw) on x, y, z axes
            angularSpeeds = diffQuaternion.toEulerAngles();
            final var angularSpeed3X = angularSpeeds[0];
            final var angularSpeed3Y = angularSpeeds[1];
            final var angularSpeed3Z = angularSpeeds[2];
            final var diffRotation3 = new Quaternion(angularSpeed3X, angularSpeed3Y, angularSpeed3Z);

            // number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            // previously sampled rotation
            final var rotation3b = new MatrixRotation3D(rotation2b);
            final var rotation3c = new MatrixRotation3D(rotation2c);
            for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation3b.combine(diffRotation);
                rotation3c.combine(diffRotation3);
            }

            // check that rotations created by composing sensor samples are equal
            // to the original one
            assertTrue(rotation3.equals(rotation3b, ABSOLUTE_ERROR));
            assertTrue(rotation3.equals(rotation3c, ABSOLUTE_ERROR));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            final var cameraSeparation2 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var rotationTransformation = new EuclideanTransformation3D(rotation1);
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            Point3D center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);
            center1 = rotationTransformation.transformAndReturnNew(center1);
            center2 = rotationTransformation.transformAndReturnNew(center2);
            center3 = rotationTransformation.transformAndReturnNew(center3);

            final var baseline = center1.distanceTo(center2);
            final var baseline2 = center2.distanceTo(center3);

            final double accelerationX;
            final double accelerationY;
            final double accelerationZ;
            final double accelerationX2;
            final double accelerationY2;
            final double accelerationZ2;

            // s = 0.5*a*t^2 --> a = 2*s/t^2
            // assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ = 2 * cameraSeparation;
            accelerationX2 = accelerationY2 = accelerationZ2 = 2 * cameraSeparation2;

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);
            final var camera3 = new PinholeCamera(intrinsic, rotation3, center3);

            final var fundamentalMatrix1 = new FundamentalMatrix(camera1, camera2);

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
                assertTrue(front1);
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

                    point3D = new InhomogeneousPoint3D(centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY, centralCommonPoint.getInhomZ() + lambdaZ);

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

                points3D2.add(point3D);

                // check that 3D point is in front of both cameras
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

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            final var listener = new AbsoluteOrientationSlamSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final double positionX, final double positionY, final double positionZ,
                        final double velocityX, final double velocityY, final double velocityZ,
                        final double accelerationX, final double accelerationY, final double accelerationZ,
                        final double quaternionA, final double quaternionB, final double quaternionC,
                        final double quaternionD, final double angularSpeedX, final double angularSpeedY,
                        final double angularSpeedZ, final Matrix covariance) {
                    slamDataAvailable++;
                    slamCovariance = covariance;
                }

                @Override
                public void onSlamCameraEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final PinholeCamera camera) {
                    slamCameraEstimated++;
                    slamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    return viewCount < 3;
                }

                @Override
                public void onRequestSamples(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int previousViewId,
                        final int currentViewId, final List<Sample2D> previousViewTrackedSamples,
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

                        // assume the following accelerator and gyroscope samples
                        // are obtained during a period of 1 second between 1st
                        // and 2nd view (50 samples * 0.02 s/sample = 1 second)
                        timestamp = 0;
                        float noiseAccelerationX;
                        float noiseAccelerationY;
                        float noiseAccelerationZ;
                        float noiseAngularSpeedX;
                        float noiseAngularSpeedY;
                        float noiseAngularSpeedZ;

                        float accelerationWithNoiseX;
                        float accelerationWithNoiseY;
                        float accelerationWithNoiseZ;
                        float angularSpeedWithNoiseX;
                        float angularSpeedWithNoiseY;
                        float angularSpeedWithNoiseZ;

                        final var accelerationWithNoise = new float[3];
                        final var angularSpeedWithNoise = new float[3];

                        final var orientation = new Quaternion(rotation1);
                        for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                            noiseAccelerationX = accelerationRandomizer.nextFloat();
                            noiseAccelerationY = accelerationRandomizer.nextFloat();
                            noiseAccelerationZ = accelerationRandomizer.nextFloat();

                            noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                            noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                            noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                            accelerationWithNoiseX = (float) accelerationX + noiseAccelerationX;
                            accelerationWithNoiseY = (float) accelerationY + noiseAccelerationY;
                            accelerationWithNoiseZ = (float) accelerationZ + noiseAccelerationZ;
                            accelerationWithNoise[0] = accelerationWithNoiseX;
                            accelerationWithNoise[1] = accelerationWithNoiseY;
                            accelerationWithNoise[2] = accelerationWithNoiseZ;

                            angularSpeedWithNoiseX = (float) angularSpeed2X + noiseAngularSpeedX;
                            angularSpeedWithNoiseY = (float) angularSpeed2Y + noiseAngularSpeedY;
                            angularSpeedWithNoiseZ = (float) angularSpeed2Z + noiseAngularSpeedZ;
                            angularSpeedWithNoise[0] = angularSpeedWithNoiseX;
                            angularSpeedWithNoise[1] = angularSpeedWithNoiseY;
                            angularSpeedWithNoise[2] = angularSpeedWithNoiseZ;

                            reconstructor.updateAccelerometerSample(timestamp, accelerationWithNoise);
                            reconstructor.updateGyroscopeSample(timestamp, angularSpeedWithNoise);
                            reconstructor.updateOrientationSample(timestamp, orientation);
                            // update orientation
                            orientation.combine(diffRotation2);
                            timestamp += DELTA_NANOS;
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

                        // assume the following accelerator and gyroscope samples
                        // are obtained during a period of 1 second between 2nd
                        // and 3rd view (50 samples * 0.02 s/sample = 1 second)
                        float noiseAccelerationX;
                        float noiseAccelerationY;
                        float noiseAccelerationZ;
                        float noiseAngularSpeedX;
                        float noiseAngularSpeedY;
                        float noiseAngularSpeedZ;

                        float accelerationWithNoiseX;
                        float accelerationWithNoiseY;
                        float accelerationWithNoiseZ;
                        float angularSpeedWithNoiseX;
                        float angularSpeedWithNoiseY;
                        float angularSpeedWithNoiseZ;

                        final var accelerationWithNoise = new float[3];
                        final var angularSpeedWithNoise = new float[3];

                        final var orientation = new Quaternion(rotation2);
                        for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                            noiseAccelerationX = accelerationRandomizer.nextFloat();
                            noiseAccelerationY = accelerationRandomizer.nextFloat();
                            noiseAccelerationZ = accelerationRandomizer.nextFloat();

                            noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                            noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                            noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                            accelerationWithNoiseX = (float) accelerationX2 + noiseAccelerationX;
                            accelerationWithNoiseY = (float) accelerationY2 + noiseAccelerationY;
                            accelerationWithNoiseZ = (float) accelerationZ2 + noiseAccelerationZ;
                            accelerationWithNoise[0] = accelerationWithNoiseX;
                            accelerationWithNoise[1] = accelerationWithNoiseY;
                            accelerationWithNoise[2] = accelerationWithNoiseZ;

                            angularSpeedWithNoiseX = (float) angularSpeed3X + noiseAngularSpeedX;
                            angularSpeedWithNoiseY = (float) angularSpeed3Y + noiseAngularSpeedY;
                            angularSpeedWithNoiseZ = (float) angularSpeed3Z + noiseAngularSpeedZ;
                            angularSpeedWithNoise[0] = angularSpeedWithNoiseX;
                            angularSpeedWithNoise[1] = angularSpeedWithNoiseY;
                            angularSpeedWithNoise[2] = angularSpeedWithNoiseZ;

                            reconstructor.updateAccelerometerSample(timestamp, accelerationWithNoise);
                            reconstructor.updateGyroscopeSample(timestamp, angularSpeedWithNoise);
                            reconstructor.updateOrientationSample(timestamp, orientation);
                            // update orientation
                            orientation.combine(diffRotation3);
                            timestamp += DELTA_NANOS;
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final List<Sample2D> allPreviousViewSamples, final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples, final int previousViewId,
                        final int currentViewId, final List<MatchedSamples> matches) {
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
                        match.setSamples(new Sample2D[]{previousSample, currentSample});
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
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    if (AbsoluteOrientationSlamSparseReconstructorTest.this.estimatedFundamentalMatrix == null) {
                        AbsoluteOrientationSlamSparseReconstructorTest.this.estimatedFundamentalMatrix =
                                estimatedFundamentalMatrix;
                    } else if (estimatedFundamentalMatrix2 == null) {
                        estimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                    }
                }

                @Override
                public void onMetricCameraEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int previousViewId,
                        final int currentViewId, final EstimatedCamera previousCamera,
                        final EstimatedCamera currentCamera) {
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
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int previousViewId,
                        final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                        final EstimatedCamera currentCamera) {
                    if (estimatedEuclideanCamera2 == null) {
                        estimatedEuclideanCamera1 = previousCamera;
                        estimatedEuclideanCamera2 = currentCamera;
                        AbsoluteOrientationSlamSparseReconstructorTest.this.scale = scale;
                    } else if (estimatedEuclideanCamera3 == null) {
                        estimatedEuclideanCamera2 = previousCamera;
                        estimatedEuclideanCamera3 = currentCamera;
                        scale2 = scale;
                    }
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    if (euclideanReconstructedPoints == null) {
                        AbsoluteOrientationSlamSparseReconstructorTest.this.scale = scale;
                    } else {
                        scale2 = scale;
                    }

                    euclideanReconstructedPoints = points;
                }

                @Override
                public void onStart(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new AbsoluteOrientationSlamSparseReconstructor(configuration, listener);

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
            assertTrue(slamDataAvailable > 0);
            assertTrue(slamCameraEstimated > 0);
            assertNotNull(slamCamera);
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            if (reconstructor.getCurrentMetricEstimatedCamera() != estimatedMetricCamera3) {
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

            // check that estimated fundamental matrix is correct
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
            assertNotSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertNotSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);
            assertNotSame(this.estimatedMetricCamera3, estimatedEuclideanCamera3);

            final var estEuclideanCam1 = this.estimatedEuclideanCamera1.getCamera();
            final var estEuclideanCam2 = this.estimatedEuclideanCamera2.getCamera();
            final var estEuclideanCam3 = this.estimatedEuclideanCamera3.getCamera();

            estMetricCam1.decompose();
            estMetricCam2.decompose();
            estMetricCam3.decompose();

            estEuclideanCam1.decompose();
            estEuclideanCam2.decompose();
            estEuclideanCam3.decompose();

            assertNotSame(metricReconstructedPoints, euclideanReconstructedPoints);

            final var numReconstructedPoints = numPoints1 - start + numPoints2;

            if (metricReconstructedPoints.size() != numReconstructedPoints) {
                continue;
            }

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            final var euclideanReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numReconstructedPoints; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
                euclideanReconstructedPoints3D.add(euclideanReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            for (var i = 0; i < numReconstructedPoints; i++) {
                final var p = metricReconstructedPoints3D.get(i);
                final var pe = euclideanReconstructedPoints3D.get(i);

                assertTrue(estMetricCam1.isPointInFrontOfCamera(p));
                assertTrue(estMetricCam2.isPointInFrontOfCamera(p));

                assertTrue(estEuclideanCam1.isPointInFrontOfCamera(pe));
                assertTrue(estEuclideanCam2.isPointInFrontOfCamera(pe));
            }

            final var euclideanCenter1 = estEuclideanCam1.getCameraCenter();
            final var euclideanCenter2 = estEuclideanCam2.getCameraCenter();
            final var euclideanCenter3 = estEuclideanCam3.getCameraCenter();

            final var euclideanIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var euclideanIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();
            final var euclideanIntrinsic3 = estEuclideanCam3.getIntrinsicParameters();

            final var euclideanRotation1 = estEuclideanCam1.getCameraRotation();
            final var euclideanRotation2 = estEuclideanCam2.getCameraRotation();
            final var euclideanRotation3 = estEuclideanCam3.getCameraRotation();

            final var estimatedBaseline = euclideanCenter1.distanceTo(euclideanCenter2);
            final var estimatedBaseline2 = euclideanCenter2.distanceTo(euclideanCenter3);

            // check cameras are correct
            final var maxBaseline = Math.max(estimatedBaseline, baseline);
            final var absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);

            final var maxBaseline2 = Math.max(estimatedBaseline2, baseline2);
            final var absoluteScaleError2 = RELATIVE_ERROR * maxBaseline2;
            if (Math.abs(estimatedBaseline2 - baseline2) > absoluteScaleError2) {
                continue;
            }
            assertEquals(estimatedBaseline2, baseline2, absoluteScaleError2);

            assertTrue(center1.equals(euclideanCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(euclideanCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2, absoluteScaleError));
            if (!center3.equals(euclideanCenter3, absoluteScaleError2)) {
                continue;
            }
            assertTrue(center3.equals(euclideanCenter3, absoluteScaleError2));

            assertEquals(euclideanIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic3.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(euclideanRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(euclideanRotation3.asInhomogeneousMatrix().equals(rotation3.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct (after scale correction)

            // check that scale error is less than 5%
            assertTrue(Math.abs(baseline / scale - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline / scale2 - 1.0) < RELATIVE_ERROR);

            final var scaleAndOrientationTransformation = new MetricTransformation3D(scale2);
            scaleAndOrientationTransformation.setRotation(rotation1.inverseRotationAndReturnNew());

            var numValidPoints = 0;
            double scaleX;
            double scaleY;
            double scaleZ;
            for (var i = start; i < numPoints1; i++) {
                final var point = points3D1.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3D.get(i - start);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleAndOrientationTransformation.transform(metricReconstructedPoints3D.get(i - start), rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                // check that scale error is less than 5%
                assertEquals(scaleX, baseline / scale2, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale2, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale2, LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR || Math.abs(scaleY - 1.0) > RELATIVE_ERROR
                        || Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(rescaledPoint.getInhomX() * baseline / scale2,
                        rescaledPoint.getInhomY() * baseline / scale2,
                        rescaledPoint.getInhomZ() * baseline / scale2);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                // check Euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                // check that scale error is less than 5%
                assertEquals(scaleX, baseline / scale2, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale2, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale2, ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            for (var i = 0; i < numPoints2; i++) {
                final var point = points3D2.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3D.get(i + numPoints1 - start);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleAndOrientationTransformation.transform(metricReconstructedPoints3D.get(i + numPoints1 - start),
                        rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                // check that scale error is less than 5%
                assertEquals(scaleX, baseline / scale2, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale2, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale2, LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR || Math.abs(scaleY - 1.0) > RELATIVE_ERROR
                        || Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(rescaledPoint.getInhomX() * baseline / scale2,
                        rescaledPoint.getInhomY() * baseline / scale2,
                        rescaledPoint.getInhomZ() * baseline / scale2);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                // check Euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                // check that scale error is less than 5%
                assertEquals(scaleX, baseline / scale2, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale2, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale2, ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            if (numValidPoints == 0) {
                continue;
            }

            double scaleRelativeError = Math.abs(baseline / scale2 - 1.0);
            LOGGER.log(Level.INFO, "Baseline relative error without noise: {0,number,0.000%}", scaleRelativeError);

            // check scales
            final var maxScale = Math.max(scale, scale2);
            scaleRelativeError = RELATIVE_ERROR * maxScale;
            if (Math.abs(scale - scale2) > scaleRelativeError) {
                continue;
            }
            assertEquals(scale, scale2, scaleRelativeError);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithoutNoiseFourViews()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException,
            RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var configuration = new AbsoluteOrientationSlamSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);

            final var accelerationOffsetX = 0.0f;
            final var accelerationOffsetY = 0.0f;
            final var accelerationOffsetZ = 0.0f;

            final var angularOffsetX = 0.0f;
            final var angularOffsetY = 0.0f;
            final var angularOffsetZ = 0.0f;

            final var calibrator = createFinishedCalibrator(accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final var calibrationData = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

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

            final var alphaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var alphaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var alphaEuler4 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler4 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler4 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);
            final var axisRotation2 = new AxisRotation3D(rotation1.inverseRotationAndReturnNew().combineAndReturnNew(
                    rotation2));

            final var rotation3 = new MatrixRotation3D(alphaEuler3, betaEuler3, gammaEuler3);
            final var rotation4 = new MatrixRotation3D(alphaEuler4, betaEuler4, gammaEuler4);

            final var axis2X = axisRotation2.getAxisX();
            final var axis2Y = axisRotation2.getAxisY();
            final var axis2Z = axisRotation2.getAxisZ();
            final var angle2 = axisRotation2.getRotationAngle();

            var diffRotation = new AxisRotation3D(axis2X, axis2Y, axis2Z, angle2 / N_SENSOR_SAMPLES);
            var diffQuaternion = new Quaternion(diffRotation);

            // angular speeds (roll, pitch, yaw) on x, y, z axes
            var angularSpeeds = diffQuaternion.toEulerAngles();
            final var angularSpeed2X = angularSpeeds[0];
            final var angularSpeed2Y = angularSpeeds[1];
            final var angularSpeed2Z = angularSpeeds[2];
            final var diffRotation2 = new Quaternion(angularSpeed2X, angularSpeed2Y, angularSpeed2Z);

            // number of samples (50 samples * 0.02 s/sample = 1 second)
            final var rotation2b = new MatrixRotation3D(rotation1);
            final var rotation2c = new MatrixRotation3D(rotation1);
            for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation2b.combine(diffRotation);
                rotation2c.combine(diffRotation2);
            }

            // check that rotations created by composing sensor samples are
            // equal to the original one
            assertTrue(rotation2.equals(rotation2b, ABSOLUTE_ERROR));
            assertTrue(rotation2.equals(rotation2c, ABSOLUTE_ERROR));

            var accumDiffRotation = rotation2.inverseRotationAndReturnNew().combineAndReturnNew(rotation3)
                    .toAxisRotation();
            final var axis3X = accumDiffRotation.getAxisX();
            final var axis3Y = accumDiffRotation.getAxisY();
            final var axis3Z = accumDiffRotation.getAxisZ();
            final var angle3 = accumDiffRotation.getRotationAngle();

            diffRotation = new AxisRotation3D(axis3X, axis3Y, axis3Z, angle3 / N_SENSOR_SAMPLES);
            diffQuaternion = new Quaternion(diffRotation);

            // angular speeds (roll, pitch, yaw) on x, y, z axes
            angularSpeeds = diffQuaternion.toEulerAngles();
            final var angularSpeed3X = angularSpeeds[0];
            final var angularSpeed3Y = angularSpeeds[1];
            final var angularSpeed3Z = angularSpeeds[2];
            final var diffRotation3 = new Quaternion(angularSpeed3X, angularSpeed3Y, angularSpeed3Z);

            // number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            // previously sampled rotation
            final var rotation3b = new MatrixRotation3D(rotation2b);
            final var rotation3c = new MatrixRotation3D(rotation2c);
            for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation3b.combine(diffRotation);
                rotation3c.combine(diffRotation3);
            }

            // check that rotations created by composing sensor samples are equal
            // to the original one
            assertTrue(rotation3.equals(rotation3b, ABSOLUTE_ERROR));
            assertTrue(rotation3.equals(rotation3c, ABSOLUTE_ERROR));

            accumDiffRotation = rotation3.inverseRotationAndReturnNew().combineAndReturnNew(rotation4).toAxisRotation();
            final var axis4X = accumDiffRotation.getAxisX();
            final var axis4Y = accumDiffRotation.getAxisY();
            final var axis4Z = accumDiffRotation.getAxisZ();
            final var angle4 = accumDiffRotation.getRotationAngle();

            diffRotation = new AxisRotation3D(axis4X, axis4Y, axis4Z, angle4 / N_SENSOR_SAMPLES);
            diffQuaternion = new Quaternion(diffRotation);

            // angular speeds (roll, pitch, yaw) on x, y, z axes
            angularSpeeds = diffQuaternion.toEulerAngles();
            final var angularSpeed4X = angularSpeeds[0];
            final var angularSpeed4Y = angularSpeeds[1];
            final var angularSpeed4Z = angularSpeeds[2];
            final var diffRotation4 = new Quaternion(angularSpeed4X, angularSpeed4Y, angularSpeed4Z);

            // number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            // previously sampled rotation
            final var rotation4b = new MatrixRotation3D(rotation3b);
            final var rotation4c = new MatrixRotation3D(rotation3c);
            for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation4b.combine(diffRotation);
                rotation4c.combine(diffRotation4);
            }

            // check that rotations created by composing sensor samples are equal
            // to the original one
            assertTrue(rotation4.equals(rotation4b, ABSOLUTE_ERROR));
            assertTrue(rotation4.equals(rotation4c, ABSOLUTE_ERROR));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            final var cameraSeparation2 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            final var cameraSeparation3 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var rotationTransformation = new EuclideanTransformation3D(rotation1);
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            Point3D center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);
            Point3D center4 = new InhomogeneousPoint3D(
                    center3.getInhomX() + cameraSeparation3,
                    center3.getInhomY() + cameraSeparation3,
                    center3.getInhomZ() + cameraSeparation3);
            center1 = rotationTransformation.transformAndReturnNew(center1);
            center2 = rotationTransformation.transformAndReturnNew(center2);
            center3 = rotationTransformation.transformAndReturnNew(center3);
            center4 = rotationTransformation.transformAndReturnNew(center4);

            final var baseline = center1.distanceTo(center2);
            final var baseline2 = center2.distanceTo(center3);
            final var baseline3 = center3.distanceTo(center4);

            final double accelerationX;
            final double accelerationY;
            final double accelerationZ;
            final double accelerationX2;
            final double accelerationY2;
            final double accelerationZ2;
            final double accelerationX3;
            final double accelerationY3;
            final double accelerationZ3;

            // s = 0.5*a*t^2 --> a = 2*s/t^2
            // assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ = 2 * cameraSeparation;
            accelerationX2 = accelerationY2 = accelerationZ2 = 2 * cameraSeparation2;
            accelerationX3 = accelerationY3 = accelerationZ3 = 2 * cameraSeparation3;

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);
            final var camera3 = new PinholeCamera(intrinsic, rotation3, center3);
            final var camera4 = new PinholeCamera(intrinsic, rotation4, center4);

            final var fundamentalMatrix1 = new FundamentalMatrix(camera1, camera2);

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
            final var numPoints3 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPoints4 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var start1 = randomizer.nextInt(0, numPoints1 - MIN_TRACKED_POINTS);
            final var start2 = randomizer.nextInt(0, numPoints2 - MIN_TRACKED_POINTS);

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
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY, centralCommonPoint.getInhomZ() + lambdaZ);

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
                assertTrue(front1);
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

            final var points3D2 = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint2b;
            Point2D projectedPoint3b;
            Point2D projectedPoint4;
            final var projectedPoints2b = new ArrayList<Point2D>();
            final var projectedPoints3b = new ArrayList<Point2D>();
            final var projectedPoints4 = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints2; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY, centralCommonPoint.getInhomZ() + lambdaZ);

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

                points3D2.add(point3D);

                // check that 3D point is in front of both cameras
                assertTrue(front2);

                // project 3D point into second pair
                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);

                projectedPoint3b = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3b);
                projectedPoints3b.add(projectedPoint3b);

                projectedPoint4 = new InhomogeneousPoint2D();
                camera4.project(point3D, projectedPoint4);
                projectedPoints4.add(projectedPoint4);
            }

            if (maxTriesReached) {
                continue;
            }

            final var points3D3 = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint3c;
            Point2D projectedPoint4b;
            final var projectedPoints3c = new ArrayList<Point2D>();
            final var projectedPoints4b = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints3; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY, centralCommonPoint.getInhomZ() + lambdaZ);

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

                points3D3.add(point3D);

                // check that 3D point is in front of both cameras
                assertTrue(front2);

                // project 3D point into second pair
                projectedPoint3c = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3c);
                projectedPoints3c.add(projectedPoint3c);

                projectedPoint4b = new InhomogeneousPoint2D();
                camera4.project(point3D, projectedPoint4b);
                projectedPoints4b.add(projectedPoint4b);
            }

            if (maxTriesReached) {
                continue;
            }

            Point2D projectedPoint4c;
            final var projectedPoints4c = new ArrayList<Point2D>();
            for (var i = 0; i < numPoints4; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(centralCommonPoint.getInhomX() + lambdaX,
                            centralCommonPoint.getInhomY() + lambdaY, centralCommonPoint.getInhomZ() + lambdaZ);

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
                assertTrue(front2);

                // project 3D point into second pair
                projectedPoint4c = new InhomogeneousPoint2D();
                camera4.project(point3D, projectedPoint4c);
                projectedPoints4c.add(projectedPoint4c);
            }

            if (maxTriesReached) {
                continue;
            }

            final var listener = new AbsoluteOrientationSlamSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final double positionX, final double positionY, final double positionZ,
                        final double velocityX, final double velocityY, final double velocityZ,
                        final double accelerationX, final double accelerationY, final double accelerationZ,
                        final double quaternionA, final double quaternionB, final double quaternionC,
                        final double quaternionD, final double angularSpeedX, final double angularSpeedY,
                        final double angularSpeedZ, final Matrix covariance) {
                    slamDataAvailable++;
                    slamCovariance = covariance;
                }

                @Override
                public void onSlamCameraEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final PinholeCamera camera) {
                    slamCameraEstimated++;
                    slamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    return viewCount < 4;
                }

                @Override
                public void onRequestSamples(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int previousViewId,
                        final int currentViewId, final List<Sample2D> previousViewTrackedSamples,
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
                    } else if (viewCount == 1) {
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

                        // assume the following accelerator and gyroscope samples
                        // are obtained during a period of 1 second between 1st
                        // and 2nd view (50 samples * 0.02 s/sample = 1 second)
                        timestamp = 0;
                        final var orientation = new Quaternion(rotation1);
                        for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                            reconstructor.updateAccelerometerSample(timestamp, (float) accelerationX,
                                    (float) accelerationY, (float) accelerationZ);
                            reconstructor.updateGyroscopeSample(timestamp, (float) angularSpeed2X,
                                    (float) angularSpeed2Y, (float) angularSpeed2Z);
                            reconstructor.updateOrientationSample(timestamp, orientation);
                            // update orientation
                            orientation.combine(diffRotation2);
                            timestamp += DELTA_NANOS;
                        }

                    } else if (viewCount == 2) {
                        // third view
                        for (var i = start1; i < numPoints1; i++) {
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

                        for (var i = start1; i < numPoints1; i++) {
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

                        // spawned samples
                        for (var i = 0; i < numPoints3; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints3c.get(i));
                            sample.setViewId(currentViewId);
                            currentViewNewlySpawnedSamples.add(sample);
                        }

                        // assume the following accelerator and gyroscope samples
                        // are obtained during a period of 1 second between 2nd
                        // and 3rd view (50 samples * 0.02 s/sample = 1 second)
                        final var orientation = new Quaternion(rotation2);
                        for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                            reconstructor.updateAccelerometerSample(timestamp, (float) accelerationX2,
                                    (float) accelerationY2, (float) accelerationZ2);
                            reconstructor.updateGyroscopeSample(timestamp, (float) angularSpeed3X,
                                    (float) angularSpeed3Y, (float) angularSpeed3Z);
                            reconstructor.updateOrientationSample(timestamp, orientation);
                            // update orientation
                            orientation.combine(diffRotation3);
                            timestamp += DELTA_NANOS;
                        }

                    } else {
                        // 4th view
                        for (var i = start2; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints3b.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints3; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints3c.get(i));
                            sample.setViewId(previousViewId);
                            previousViewTrackedSamples.add(sample);
                        }

                        for (var i = start2; i < numPoints2; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints4.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        for (var i = 0; i < numPoints3; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints4b.get(i));
                            sample.setViewId(currentViewId);
                            currentViewTrackedSamples.add(sample);
                        }

                        // spawned samples
                        for (var i = 0; i < numPoints4; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints4c.get(i));
                            sample.setViewId(currentViewId);
                            currentViewNewlySpawnedSamples.add(sample);
                        }

                        // assume the following accelerator and gyroscope samples
                        // are obtained during a period of 1 second between 2nd
                        // and 3rd view (50 samples * 0.02 s/sample = 1 second)
                        final var orientation = new Quaternion(rotation3);
                        for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                            reconstructor.updateAccelerometerSample(timestamp, (float) accelerationX3,
                                    (float) accelerationY3, (float) accelerationZ3);
                            reconstructor.updateGyroscopeSample(timestamp, (float) angularSpeed4X,
                                    (float) angularSpeed4Y, (float) angularSpeed4Z);
                            reconstructor.updateOrientationSample(timestamp, orientation);
                            // update orientation
                            orientation.combine(diffRotation4);
                            timestamp += DELTA_NANOS;
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final List<Sample2D> allPreviousViewSamples, final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples, final int previousViewId,
                        final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    int numCameras = 0;
                    if (estimatedMetricCamera1 != null && (estimatedMetricCamera1.getViewId() == previousViewId
                            || estimatedMetricCamera1.getViewId() == currentViewId)) {
                        numCameras++;
                    }
                    if (estimatedMetricCamera2 != null && (estimatedMetricCamera2.getViewId() == previousViewId
                            || estimatedMetricCamera2.getViewId() == currentViewId)) {
                        numCameras++;
                    }
                    if (estimatedMetricCamera3 != null && (estimatedMetricCamera3.getViewId() == previousViewId
                            || estimatedMetricCamera3.getViewId() == currentViewId)) {
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
                        if (estimatedMetricCamera3 != null && (estimatedMetricCamera3.getViewId() == previousViewId
                                || estimatedMetricCamera3.getViewId() == currentViewId)) {
                            estimatedCameras[pos] = estimatedMetricCamera3;
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
                        match.setSamples(new Sample2D[]{previousSample, currentSample});
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
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    if (AbsoluteOrientationSlamSparseReconstructorTest.this.estimatedFundamentalMatrix == null) {
                        AbsoluteOrientationSlamSparseReconstructorTest.this.estimatedFundamentalMatrix =
                                estimatedFundamentalMatrix;
                    } else if (estimatedFundamentalMatrix2 == null) {
                        estimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                    } else if (estimatedFundamentalMatrix3 == null) {
                        estimatedFundamentalMatrix3 = estimatedFundamentalMatrix;
                    }
                }

                @Override
                public void onMetricCameraEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int previousViewId,
                        final int currentViewId, final EstimatedCamera previousCamera,
                        final EstimatedCamera currentCamera) {
                    if (estimatedMetricCamera2 == null) {
                        estimatedMetricCamera1 = previousCamera;
                        estimatedMetricCamera2 = currentCamera;
                    } else if (estimatedMetricCamera3 == null) {
                        estimatedMetricCamera2 = previousCamera;
                        estimatedMetricCamera3 = currentCamera;
                    } else if (estimatedMetricCamera4 == null) {
                        estimatedMetricCamera3 = previousCamera;
                        estimatedMetricCamera4 = currentCamera;
                    }
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final int previousViewId,
                        final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                        final EstimatedCamera currentCamera) {
                    if (estimatedEuclideanCamera2 == null) {
                        estimatedEuclideanCamera1 = previousCamera;
                        estimatedEuclideanCamera2 = currentCamera;
                        AbsoluteOrientationSlamSparseReconstructorTest.this.scale = scale;
                    } else if (estimatedEuclideanCamera3 == null) {
                        estimatedEuclideanCamera2 = previousCamera;
                        estimatedEuclideanCamera3 = currentCamera;
                        scale2 = scale;
                    } else if (estimatedEuclideanCamera4 == null) {
                        estimatedEuclideanCamera3 = previousCamera;
                        estimatedEuclideanCamera4 = currentCamera;
                        scale3 = scale;
                    }
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final AbsoluteOrientationSlamSparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    if (euclideanReconstructedPoints == null) {
                        AbsoluteOrientationSlamSparseReconstructorTest.this.scale = scale;
                    } else if (estimatedMetricCamera4 == null) {
                        scale2 = scale;
                    } else {
                        scale3 = scale;
                    }

                    euclideanReconstructedPoints = points;
                }

                @Override
                public void onStart(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final AbsoluteOrientationSlamSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new AbsoluteOrientationSlamSparseReconstructor(configuration, listener);

            // check initial values
            reset();
            assertFalse(started);
            assertFalse(finished);
            assertFalse(cancelled);
            assertFalse(failed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            if (failed) {
                continue;
            }

            // check correctness
            assertTrue(started);
            assertTrue(finished);
            assertFalse(cancelled);
            assertTrue(reconstructor.isFinished());
            assertTrue(slamDataAvailable > 0);
            assertTrue(slamCameraEstimated > 0);
            assertNotNull(slamCamera);
            assertFalse(reconstructor.isFirstView());
            assertFalse(reconstructor.isSecondView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.isAdditionalView());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            if (estimatedMetricCamera4 != reconstructor.getCurrentMetricEstimatedCamera()) {
                continue;
            }
            assertSame(estimatedMetricCamera4, reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(estimatedMetricCamera3, reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera4, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera3, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(metricReconstructedPoints, reconstructor.getActiveMetricReconstructedPoints());
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints, reconstructor.getActiveEuclideanReconstructedPoints());
            assertEquals(scale3, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewTrackedSamples());
            assertNotNull(reconstructor.getCurrentViewNewlySpawnedSamples());

            // check that estimated fundamental matrix is correct
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
            final var estMetricCam4 = this.estimatedMetricCamera4.getCamera();
            assertNotSame(this.estimatedMetricCamera1, estimatedEuclideanCamera1);
            assertNotSame(this.estimatedMetricCamera2, estimatedEuclideanCamera2);
            assertNotSame(this.estimatedMetricCamera3, estimatedEuclideanCamera3);
            assertNotSame(this.estimatedMetricCamera4, estimatedEuclideanCamera4);

            final var estEuclideanCam1 = this.estimatedEuclideanCamera1.getCamera();
            final var estEuclideanCam2 = this.estimatedEuclideanCamera2.getCamera();
            final var estEuclideanCam3 = this.estimatedEuclideanCamera3.getCamera();
            final var estEuclideanCam4 = this.estimatedEuclideanCamera4.getCamera();

            estMetricCam1.decompose();
            estMetricCam2.decompose();
            estMetricCam3.decompose();
            estMetricCam4.decompose();

            estEuclideanCam1.decompose();
            estEuclideanCam2.decompose();
            estEuclideanCam3.decompose();
            estEuclideanCam4.decompose();

            assertNotSame(metricReconstructedPoints, euclideanReconstructedPoints);

            final var numReconstructedPoints = numPoints2 - start2 + numPoints3;

            if (metricReconstructedPoints.size() != numReconstructedPoints) {
                continue;
            }

            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            final var euclideanReconstructedPoints3D = new ArrayList<Point3D>();
            for (var i = 0; i < numReconstructedPoints; i++) {
                metricReconstructedPoints3D.add(metricReconstructedPoints.get(i).getPoint());
                euclideanReconstructedPoints3D.add(euclideanReconstructedPoints.get(i).getPoint());
            }

            // check that all points are in front of both cameras
            var failedIter = false;
            for (var i = 0; i < numReconstructedPoints; i++) {
                final var p = metricReconstructedPoints3D.get(i);
                final var pe = euclideanReconstructedPoints3D.get(i);

                if (!estMetricCam1.isPointInFrontOfCamera(p)) {
                    failedIter = true;
                    break;
                }
                assertTrue(estMetricCam1.isPointInFrontOfCamera(p));

                if (!estMetricCam2.isPointInFrontOfCamera(p)) {
                    failedIter = true;
                    break;
                }
                assertTrue(estMetricCam2.isPointInFrontOfCamera(p));

                if (!estEuclideanCam1.isPointInFrontOfCamera(pe)) {
                    failedIter = true;
                    break;
                }
                assertTrue(estEuclideanCam1.isPointInFrontOfCamera(pe));

                if (!estEuclideanCam2.isPointInFrontOfCamera(pe)) {
                    failedIter = true;
                    break;
                }
                assertTrue(estEuclideanCam2.isPointInFrontOfCamera(pe));
            }

            if (failedIter) {
                continue;
            }

            final var metricCenter1 = estMetricCam1.getCameraCenter();
            final var metricCenter2 = estMetricCam2.getCameraCenter();
            final var metricCenter3 = estMetricCam3.getCameraCenter();
            final var metricCenter4 = estMetricCam4.getCameraCenter();

            final var euclideanCenter1 = estEuclideanCam1.getCameraCenter();
            final var euclideanCenter2 = estEuclideanCam2.getCameraCenter();
            final var euclideanCenter3 = estEuclideanCam3.getCameraCenter();
            final var euclideanCenter4 = estEuclideanCam4.getCameraCenter();

            final var euclideanIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var euclideanIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();
            final var euclideanIntrinsic3 = estEuclideanCam3.getIntrinsicParameters();
            final var euclideanIntrinsic4 = estEuclideanCam4.getIntrinsicParameters();

            final var euclideanRotation1 = estEuclideanCam1.getCameraRotation();
            final var euclideanRotation2 = estEuclideanCam2.getCameraRotation();
            final var euclideanRotation3 = estEuclideanCam3.getCameraRotation();
            final var euclideanRotation4 = estEuclideanCam4.getCameraRotation();

            final var estimatedBaseline = euclideanCenter1.distanceTo(euclideanCenter2);
            final var estimatedBaseline2 = euclideanCenter2.distanceTo(euclideanCenter3);
            final var estimatedBaseline3 = euclideanCenter3.distanceTo(euclideanCenter4);

            // check cameras are correct
            final var maxBaseline = Math.max(estimatedBaseline, baseline);
            final var absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);

            final var maxBaseline2 = Math.max(estimatedBaseline2, baseline2);
            final var absoluteScaleError2 = RELATIVE_ERROR * maxBaseline2;
            if (Math.abs(estimatedBaseline2 - baseline2) > absoluteScaleError2) {
                continue;
            }
            assertEquals(estimatedBaseline2, baseline2, absoluteScaleError2);

            final var maxBaseline3 = Math.max(estimatedBaseline3, baseline3);
            final var absoluteScaleError3 = RELATIVE_ERROR * maxBaseline3;
            if (Math.abs(estimatedBaseline3 - baseline3) > absoluteScaleError3) {
                continue;
            }
            assertEquals(estimatedBaseline3, baseline3, absoluteScaleError3);

            var scaleAndOrientationTransformation = new MetricTransformation3D(baseline);
            scaleAndOrientationTransformation.setRotation(rotation1.inverseRotationAndReturnNew());
            final var rescaledCenter1 = scaleAndOrientationTransformation.transformAndReturnNew(metricCenter1);
            final var rescaledCenter2 = scaleAndOrientationTransformation.transformAndReturnNew(metricCenter2);
            final var rescaledCenter3 = scaleAndOrientationTransformation.transformAndReturnNew(metricCenter3);
            final var rescaledCenter4 = scaleAndOrientationTransformation.transformAndReturnNew(metricCenter4);

            assertTrue(center1.equals(rescaledCenter1, LARGE_ABSOLUTE_ERROR));
            assertTrue(center2.equals(rescaledCenter2, LARGE_ABSOLUTE_ERROR));
            assertTrue(center3.equals(rescaledCenter3, LARGE_ABSOLUTE_ERROR));
            assertTrue(center4.equals(rescaledCenter4, LARGE_ABSOLUTE_ERROR));

            assertTrue(center1.equals(euclideanCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(euclideanCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2, absoluteScaleError));
            if (!center3.equals(euclideanCenter3, absoluteScaleError2)) {
                continue;
            }
            assertTrue(center3.equals(euclideanCenter3, absoluteScaleError2));
            if (!center4.equals(euclideanCenter4, 3 * absoluteScaleError3)) {
                continue;
            }
            assertTrue(center4.equals(euclideanCenter4, 3 * absoluteScaleError3));

            assertEquals(euclideanIntrinsic1.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic3.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic4.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic4.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic4.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic4.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic4.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);

            assertTrue(euclideanRotation1.asInhomogeneousMatrix().equals(rotation1.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(euclideanRotation3.asInhomogeneousMatrix().equals(rotation3.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(euclideanRotation4.asInhomogeneousMatrix().equals(rotation4.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct (after scale correction)

            // check that scale error is less than 5%
            assertTrue(Math.abs(baseline / scale - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline / scale2 - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline / scale3 - 1.0) < RELATIVE_ERROR);

            scaleAndOrientationTransformation = new MetricTransformation3D(scale3);
            scaleAndOrientationTransformation.setRotation(rotation1.inverseRotationAndReturnNew());

            var numValidPoints = 0;
            double scaleX;
            double scaleY;
            double scaleZ;
            for (var i = start2; i < numPoints2; i++) {
                final var point = points3D2.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3D.get(i - start2);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleAndOrientationTransformation.transform(metricReconstructedPoints3D.get(i - start2), rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                // check that scale error is less than 5%
                assertEquals(scaleX, baseline / scale3, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale3, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale3, ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR || Math.abs(scaleY - 1.0) > RELATIVE_ERROR
                        || Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(rescaledPoint.getInhomX() * baseline / scale3,
                        rescaledPoint.getInhomY() * baseline / scale3,
                        rescaledPoint.getInhomZ() * baseline / scale3);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                // check Euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                // check that scale error is less than 5%
                assertEquals(scaleX, baseline / scale3, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale3, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale3, ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            for (var i = 0; i < numPoints3; i++) {
                final var point = points3D3.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3D.get(i + numPoints2 - start2);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleAndOrientationTransformation.transform(metricReconstructedPoints3D.get(i + numPoints2 - start2),
                        rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                // check that scale error is less than 5%
                assertEquals(scaleX, baseline / scale3, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale3, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale3, ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR || Math.abs(scaleY - 1.0) > RELATIVE_ERROR
                        || Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(rescaledPoint.getInhomX() * baseline / scale3,
                        rescaledPoint.getInhomY() * baseline / scale3,
                        rescaledPoint.getInhomZ() * baseline / scale3);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                // check Euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                // check that scale error is less than 5%
                assertEquals(scaleX, baseline / scale3, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale3, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale3, ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            if (numValidPoints == 0) {
                continue;
            }

            var scaleRelativeError = Math.abs(baseline / scale3 - 1.0);
            LOGGER.log(Level.INFO, "Baseline relative error without noise: {0,number,0.000%}", scaleRelativeError);

            // check scales
            final var maxScale = Math.max(Math.max(scale, scale2), scale3);
            scaleRelativeError = RELATIVE_ERROR * maxScale;
            if (Math.abs(scale - scale2) > scaleRelativeError) {
                continue;
            }
            assertEquals(scale, scale2, scaleRelativeError);
            if (Math.abs(scale - scale3) > scaleRelativeError) {
                continue;
            }
            assertEquals(scale, scale3, scaleRelativeError);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private void reset() {
        viewCount = 0;
        estimatedFundamentalMatrix = estimatedFundamentalMatrix2 = estimatedFundamentalMatrix3 = null;
        estimatedMetricCamera1 = estimatedMetricCamera2 = estimatedMetricCamera3 = estimatedMetricCamera4 = null;
        estimatedEuclideanCamera1 = estimatedEuclideanCamera2 = estimatedEuclideanCamera3 = estimatedEuclideanCamera4 =
                null;
        metricReconstructedPoints = null;
        euclideanReconstructedPoints = null;
        started = finished = failed = cancelled = false;
        scale = 0.0;
        timestamp = 0;
        slamDataAvailable = 0;
        slamCameraEstimated = 0;
        slamCamera = null;
        slamCovariance = null;
    }

    private static AbsoluteOrientationSlamCalibrator createFinishedCalibrator(
            final float accelerationOffsetX, final float accelerationOffsetY, final float accelerationOffsetZ,
            final float angularOffsetX, final float angularOffsetY, final float angularOffsetZ,
            final GaussianRandomizer noiseRandomizer) {
        final var calibrator = AbsoluteOrientationSlamEstimator.createCalibrator();
        calibrator.setConvergenceThreshold(ABSOLUTE_ERROR);
        calibrator.setMaxNumSamples(MAX_CALIBRATION_SAMPLES);

        var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;

        float accelerationNoiseX;
        float accelerationNoiseY;
        float accelerationNoiseZ;
        float angularNoiseX;
        float angularNoiseY;
        float angularNoiseZ;

        double accelerationX;
        double accelerationY;
        double accelerationZ;
        double angularX;
        double angularY;
        double angularZ;
        final var orientation = new Quaternion();

        for (var i = 0; i < MAX_CALIBRATION_SAMPLES; i++) {
            accelerationNoiseX = noiseRandomizer.nextFloat();
            accelerationNoiseY = noiseRandomizer.nextFloat();
            accelerationNoiseZ = noiseRandomizer.nextFloat();

            angularNoiseX = noiseRandomizer.nextFloat();
            angularNoiseY = noiseRandomizer.nextFloat();
            angularNoiseZ = noiseRandomizer.nextFloat();

            accelerationX = accelerationOffsetX + accelerationNoiseX;
            accelerationY = accelerationOffsetY + accelerationNoiseY;
            accelerationZ = accelerationOffsetZ + accelerationNoiseZ;

            angularX = angularOffsetX + angularNoiseX;
            angularY = angularOffsetY + angularNoiseY;
            angularZ = angularOffsetZ + angularNoiseZ;

            calibrator.updateAccelerometerSample(timestamp, (float) accelerationX, (float) accelerationY,
                    (float) accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float) angularX, (float) angularY, (float) angularZ);
            calibrator.updateOrientationSample(timestamp, orientation);

            if (calibrator.isFinished()) {
                break;
            }

            timestamp += DELTA_NANOS;
        }

        return calibrator;
    }
}
