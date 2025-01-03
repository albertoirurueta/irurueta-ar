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
import com.irurueta.ar.slam.ConstantVelocityModelSlamCalibrator;
import com.irurueta.ar.slam.ConstantVelocityModelSlamEstimator;
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

class ConstantVelocityModelSlamSparseReconstructorTest {

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
    private static final double RELATIVE_ERROR = 0.1;

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
            ConstantVelocityModelSlamSparseReconstructorTest.class.getSimpleName());

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

    private long timestamp;

    private int slamDataAvailable;
    private int slamCameraEstimated;

    private PinholeCamera slamCamera;
    private Matrix slamCovariance;

    @BeforeEach
    void setUp() {
        viewCount = 0;
        estimatedFundamentalMatrix = estimatedFundamentalMatrix2 = null;
        estimatedMetricCamera1 = estimatedMetricCamera2 = estimatedMetricCamera3 = null;
        estimatedEuclideanCamera1 = estimatedEuclideanCamera2 = estimatedEuclideanCamera3 = null;
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
        assertEquals(2, ConstantVelocityModelSlamSparseReconstructor.MIN_NUMBER_OF_VIEWS);

        final var configuration = new ConstantVelocityModelSlamSparseReconstructorConfiguration();
        final var listener = new ConstantVelocityModelSlamSparseReconstructorListener() {
            @Override
            public void onSlamDataAvailable(
                    final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                    final double positionX, final double positionY, final double positionZ,
                    final double velocityX, final double velocityY, final double velocityZ,
                    final double accelerationX, final double accelerationY,
                    final double accelerationZ, final double quaternionA,
                    final double quaternionB, final double quaternionC, final double quaternionD,
                    final double angularSpeedX, final double angularSpeedY,
                    final double angularSpeedZ, final Matrix covariance) {
                // no action needed
            }

            @Override
            public void onSlamCameraEstimated(
                    final ConstantVelocityModelSlamSparseReconstructor reconstructor, final PinholeCamera camera) {
                // no action needed
            }

            @Override
            public boolean hasMoreViewsAvailable(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                return false;
            }

            @Override
            public void onRequestSamples(
                    final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                    final int previousViewId, final int currentViewId,
                    final List<Sample2D> previousViewTrackedSamples,
                    final List<Sample2D> currentViewTrackedSamples,
                    final List<Sample2D> currentViewNewlySpawnedSamples) {
                // no action needed
            }

            @Override
            public void onSamplesAccepted(
                    final ConstantVelocityModelSlamSparseReconstructor reconstructor, final int viewId,
                    final List<Sample2D> previousViewTrackedSamples,
                    final List<Sample2D> currentViewTrackedSamples) {
                // no action needed
            }

            @Override
            public void onSamplesRejected(
                    final ConstantVelocityModelSlamSparseReconstructor reconstructor, final int viewId,
                    final List<Sample2D> previousViewTrackedSamples, final List<Sample2D> currentViewTrackedSamples) {
                // no action needed
            }

            @Override
            public void onRequestMatches(
                    final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                    final List<Sample2D> allPreviousViewSamples,
                    final List<Sample2D> previousViewTrackedSamples,
                    final List<Sample2D> currentViewTrackedSamples, final int previousViewId,
                    final int currentViewId, final List<MatchedSamples> matches) {
                // no action needed
            }

            @Override
            public void onFundamentalMatrixEstimated(
                    final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                    final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                // no action needed
            }

            @Override
            public void onMetricCameraEstimated(
                    final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                    final int previousViewId, final int currentViewId,
                    final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                // no action needed
            }

            @Override
            public void onMetricReconstructedPointsEstimated(
                    final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                    final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                // no action needed
            }

            @Override
            public void onEuclideanCameraEstimated(
                    final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                    final int previousViewId, final int currentViewId, final double scale,
                    final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                // no action needed
            }

            @Override
            public void onEuclideanReconstructedPointsEstimated(
                    final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                    final double scale, final List<ReconstructedPoint3D> points) {
                // no action needed
            }

            @Override
            public void onStart(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onFinish(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onCancel(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onFail(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                // no action needed
            }
        };

        // constructor with listener
        var reconstructor = new ConstantVelocityModelSlamSparseReconstructor(listener);

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
        reconstructor = new ConstantVelocityModelSlamSparseReconstructor(configuration, listener);

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

            final var configuration = new ConstantVelocityModelSlamSparseReconstructorConfiguration();
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

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);
            final var axisRotation2 = new AxisRotation3D(rotation2);

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

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

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
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front1 || !front2);
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
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front2);
                points3D1.add(point3D);

                // check that 3D point is in front of both cameras
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            final var listener = new ConstantVelocityModelSlamSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
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
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final PinholeCamera camera) {
                    slamCameraEstimated++;
                    slamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamples(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final int previousViewId,
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
                        for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                            reconstructor.updateAccelerometerSample(timestamp, (float) accelerationX,
                                    (float) accelerationY, (float) accelerationZ);
                            reconstructor.updateGyroscopeSample(timestamp, (float) angularSpeedX, (float) angularSpeedY,
                                    (float) angularSpeedZ);
                            timestamp += DELTA_NANOS;
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final List<Sample2D> allPreviousViewSamples, final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples, final int previousViewId,
                        final int currentViewId, final List<MatchedSamples> matches) {

                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints1; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                previousViewTrackedSamples.get(i),
                                currentViewTrackedSamples.get(i)
                        });
                        match.setViewIds(new int[]{previousViewId, currentViewId});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    ConstantVelocityModelSlamSparseReconstructorTest.this.estimatedFundamentalMatrix =
                            estimatedFundamentalMatrix;
                }

                @Override
                public void onMetricCameraEstimated(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final int previousViewId, final int currentViewId, final EstimatedCamera previousCamera,
                        final EstimatedCamera currentCamera) {
                    estimatedMetricCamera1 = previousCamera;
                    estimatedMetricCamera2 = currentCamera;
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final int previousViewId, final int currentViewId, final double scale,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedEuclideanCamera1 = previousCamera;
                    estimatedEuclideanCamera2 = currentCamera;
                    ConstantVelocityModelSlamSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    ConstantVelocityModelSlamSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onStart(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new ConstantVelocityModelSlamSparseReconstructor(configuration, listener);

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

            final var scaleTransformation = new MetricTransformation3D(scale);

            var numValidPoints = 0;
            double scaleX;
            double scaleY;
            double scaleZ;
            for (var i = 0; i < numPoints1; i++) {
                final var point = points3D1.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3D.get(i);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleTransformation.transform(metricReconstructedPoints3D.get(i), rescaledPoint);

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
                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / scale,
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

            final var scaleRelativeError = Math.abs(baseline / scale - 1.0);
            LOGGER.log(Level.INFO, "Baseline relative error without noise: {0,number,0.000%}", scaleRelativeError);

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

            final var configuration = new ConstantVelocityModelSlamSparseReconstructorConfiguration();
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

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);
            final var axisRotation2 = new AxisRotation3D(rotation2);

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

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

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
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!front2);

                // check that 3D point is in front of both cameras
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);
            }

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            final var listener = new ConstantVelocityModelSlamSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
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
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final PinholeCamera camera) {
                    slamCameraEstimated++;
                    slamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamples(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final int previousViewId,
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
                            timestamp += DELTA_NANOS;
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final List<Sample2D> allPreviousViewSamples, final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples, final int previousViewId,
                        final int currentViewId, final List<MatchedSamples> matches) {
                    matches.clear();

                    MatchedSamples match;
                    for (var i = 0; i < numPoints1; i++) {
                        match = new MatchedSamples();
                        match.setSamples(new Sample2D[]{
                                previousViewTrackedSamples.get(i),
                                currentViewTrackedSamples.get(i)
                        });
                        match.setViewIds(new int[]{previousViewId, currentViewId});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    ConstantVelocityModelSlamSparseReconstructorTest.this.estimatedFundamentalMatrix =
                            estimatedFundamentalMatrix;
                }

                @Override
                public void onMetricCameraEstimated(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final int previousViewId, final int currentViewId, final EstimatedCamera previousCamera,
                        final EstimatedCamera currentCamera) {
                    estimatedMetricCamera1 = previousCamera;
                    estimatedMetricCamera2 = currentCamera;
                }

                @Override
                public void onMetricReconstructedPointsEstimated(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final int previousViewId, final int currentViewId, final double scale,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    estimatedEuclideanCamera1 = previousCamera;
                    estimatedEuclideanCamera2 = currentCamera;
                    ConstantVelocityModelSlamSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    ConstantVelocityModelSlamSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onStart(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new ConstantVelocityModelSlamSparseReconstructor(configuration, listener);

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

            final var scaleTransformation = new MetricTransformation3D(scale);

            var numValidPoints = 0;
            double scaleX;
            double scaleY;
            double scaleZ;
            for (var i = 0; i < numPoints1; i++) {
                final var point = points3D1.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3D.get(i);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleTransformation.transform(metricReconstructedPoints3D.get(i), rescaledPoint);

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
                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / scale,
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
                if (Math.abs(scaleX - (baseline / scale)) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline / scale, ABSOLUTE_ERROR);
                if (Math.abs(scaleY - (baseline / scale)) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline / scale, ABSOLUTE_ERROR);
                if (Math.abs(scaleZ - (baseline / scale)) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline / scale, ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            if (numValidPoints == 0) {
                continue;
            }

            final var scaleRelativeError = Math.abs(baseline / scale - 1.0);
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

            final var configuration = new ConstantVelocityModelSlamSparseReconstructorConfiguration();
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

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var alphaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);
            final var axisRotation2 = new AxisRotation3D(rotation2);
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
            var diffRotation2 = new Quaternion(angularSpeed2X, angularSpeed2Y, angularSpeed2Z);

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
            diffRotation2 = new Quaternion(angularSpeed3X, angularSpeed3Y, angularSpeed3Z);

            // number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            // previously sampled rotation
            final var rotation3b = new MatrixRotation3D(rotation2b);
            final var rotation3c = new MatrixRotation3D(rotation2c);
            for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation3b.combine(diffRotation);
                rotation3c.combine(diffRotation2);
            }

            // check that rotations created by composing sensor samples are equal
            // to the original one
            assertTrue(rotation3.equals(rotation3b, ABSOLUTE_ERROR));
            assertTrue(rotation3.equals(rotation3c, ABSOLUTE_ERROR));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            final var cameraSeparation2 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            final var center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);

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

                // check that 3D point is in front of both cameras
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);

                projectedPoint3b = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3b);
                projectedPoints3b.add(projectedPoint3b);
            }

            final var listener = new ConstantVelocityModelSlamSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
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
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final PinholeCamera camera) {
                    slamCameraEstimated++;
                    slamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    return viewCount < 3;
                }

                @Override
                public void onRequestSamples(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
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
                        for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                            reconstructor.updateAccelerometerSample(timestamp, (float) accelerationX,
                                    (float) accelerationY, (float) accelerationZ);
                            reconstructor.updateGyroscopeSample(timestamp, (float) angularSpeed2X,
                                    (float) angularSpeed2Y, (float) angularSpeed2Z);
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
                        for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                            reconstructor.updateAccelerometerSample(timestamp, (float) accelerationX2,
                                    (float) accelerationY2, (float) accelerationZ2);
                            reconstructor.updateGyroscopeSample(timestamp, (float) angularSpeed3X,
                                    (float) angularSpeed3Y, (float) angularSpeed3Z);
                            timestamp += DELTA_NANOS;
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final List<Sample2D> allPreviousViewSamples,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples,
                        final int previousViewId, final int currentViewId,
                        final List<MatchedSamples> matches) {
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
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    if (ConstantVelocityModelSlamSparseReconstructorTest.this.estimatedFundamentalMatrix == null) {
                        ConstantVelocityModelSlamSparseReconstructorTest.this.estimatedFundamentalMatrix =
                                estimatedFundamentalMatrix;
                    } else if (estimatedFundamentalMatrix2 == null) {
                        estimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                    }
                }

                @Override
                public void onMetricCameraEstimated(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final int previousViewId, final int currentViewId, final EstimatedCamera previousCamera,
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
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final int previousViewId, final int currentViewId, final double scale,
                        final EstimatedCamera previousCamera, final EstimatedCamera currentCamera) {
                    if (estimatedEuclideanCamera2 == null) {
                        estimatedEuclideanCamera1 = previousCamera;
                        estimatedEuclideanCamera2 = currentCamera;
                        ConstantVelocityModelSlamSparseReconstructorTest.this.scale = scale;
                    } else if (estimatedEuclideanCamera3 == null) {
                        estimatedEuclideanCamera2 = previousCamera;
                        estimatedEuclideanCamera3 = currentCamera;
                        scale2 = scale;
                    }
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    if (euclideanReconstructedPoints == null) {
                        ConstantVelocityModelSlamSparseReconstructorTest.this.scale = scale;
                    } else {
                        scale2 = scale;
                    }

                    euclideanReconstructedPoints = points;
                }

                @Override
                public void onStart(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new ConstantVelocityModelSlamSparseReconstructor(configuration, listener);

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

            // check that points are correct (up to 5% scale error)

            // check that scale error is less than 5%
            assertTrue(Math.abs(baseline / scale - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline / scale2 - 1.0) < RELATIVE_ERROR);
            final var scaleTransformation = new MetricTransformation3D(scale2);

            var numValidPoints = 0;
            double scaleX;
            double scaleY;
            double scaleZ;
            for (var i = start; i < numPoints1; i++) {
                final var point = points3D1.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3D.get(i - start);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleTransformation.transform(metricReconstructedPoints3D.get(i - start), rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                // check that scale error is less than 5%
                if (Math.abs(scaleX - baseline / scale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline / scale2, ABSOLUTE_ERROR);

                if (Math.abs(scaleY - baseline / scale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline / scale2, ABSOLUTE_ERROR);

                if (Math.abs(scaleZ - baseline / scale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline / scale2, ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR || Math.abs(scaleY - 1.0) > RELATIVE_ERROR
                        || Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / scale2,
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
                scaleTransformation.transform(metricReconstructedPoints3D.get(i + numPoints1 - start), rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                // check that scale error is less than 5%
                if (Math.abs(scaleX - baseline / scale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline / scale2, ABSOLUTE_ERROR);

                if (Math.abs(scaleY - baseline / scale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline / scale2, ABSOLUTE_ERROR);

                if (Math.abs(scaleZ - baseline / scale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline / scale2, ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR || Math.abs(scaleY - 1.0) > RELATIVE_ERROR
                        || Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / scale2,
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

            final var configuration = new ConstantVelocityModelSlamSparseReconstructorConfiguration();
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

            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var alphaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler3 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);
            final var axisRotation2 = new AxisRotation3D(rotation2);
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
            var diffRotation2 = new Quaternion(angularSpeed2X, angularSpeed2Y, angularSpeed2Z);

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
            diffRotation2 = new Quaternion(angularSpeed3X, angularSpeed3Y, angularSpeed3Z);

            // number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            // previously sampled rotation
            final var rotation3b = new MatrixRotation3D(rotation2b);
            final var rotation3c = new MatrixRotation3D(rotation2c);
            for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation3b.combine(diffRotation);
                rotation3c.combine(diffRotation2);
            }

            // check that rotations created by composing sensor samples are equal
            // to the original one
            assertTrue(rotation3.equals(rotation3b, ABSOLUTE_ERROR));
            assertTrue(rotation3.equals(rotation3c, ABSOLUTE_ERROR));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            final var cameraSeparation2 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            final var center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);

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

                // check that 3D point is in front of both cameras
                assertTrue(front2);

                projectedPoint2b = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2b);
                projectedPoints2b.add(projectedPoint2b);

                projectedPoint3b = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3b);
                projectedPoints3b.add(projectedPoint3b);
            }

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            final var listener = new ConstantVelocityModelSlamSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
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
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final PinholeCamera camera) {
                    slamCameraEstimated++;
                    slamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    return viewCount < 3;
                }

                @Override
                public void onRequestSamples(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final int previousViewId,
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
                            timestamp += DELTA_NANOS;
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> previousViewTrackedSamples,
                        final List<Sample2D> currentViewTrackedSamples) {
                    viewCount++;
                }

                @Override
                public void onRequestMatches(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
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
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    if (ConstantVelocityModelSlamSparseReconstructorTest.this.estimatedFundamentalMatrix == null) {
                        ConstantVelocityModelSlamSparseReconstructorTest.this.estimatedFundamentalMatrix =
                                estimatedFundamentalMatrix;
                    } else if (estimatedFundamentalMatrix2 == null) {
                        estimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                    }
                }

                @Override
                public void onMetricCameraEstimated(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final int previousViewId,
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
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    metricReconstructedPoints = points;
                }

                @Override
                public void onEuclideanCameraEstimated(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final int previousViewId,
                        final int currentViewId, final double scale, final EstimatedCamera previousCamera,
                        final EstimatedCamera currentCamera) {
                    if (estimatedEuclideanCamera2 == null) {
                        estimatedEuclideanCamera1 = previousCamera;
                        estimatedEuclideanCamera2 = currentCamera;
                        ConstantVelocityModelSlamSparseReconstructorTest.this.scale = scale;
                    } else if (estimatedEuclideanCamera3 == null) {
                        estimatedEuclideanCamera2 = previousCamera;
                        estimatedEuclideanCamera3 = currentCamera;
                        scale2 = scale;
                    }
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final ConstantVelocityModelSlamSparseReconstructor reconstructor, final double scale,
                        final List<ReconstructedPoint3D> points) {
                    if (euclideanReconstructedPoints == null) {
                        ConstantVelocityModelSlamSparseReconstructorTest.this.scale = scale;
                    } else {
                        scale2 = scale;
                    }

                    euclideanReconstructedPoints = points;
                }

                @Override
                public void onStart(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final ConstantVelocityModelSlamSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new ConstantVelocityModelSlamSparseReconstructor(configuration, listener);

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
            assertSame(estimatedMetricCamera3, reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertSame(estimatedMetricCamera2, reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), estimatedEuclideanCamera3);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), estimatedEuclideanCamera2);
            assertNotNull(reconstructor.getActiveMetricReconstructedPoints());
            assertSame(reconstructor.getActiveMetricReconstructedPoints(), metricReconstructedPoints);
            assertNotNull(reconstructor.getActiveEuclideanReconstructedPoints());
            assertSame(reconstructor.getActiveEuclideanReconstructedPoints(), euclideanReconstructedPoints);
            assertEquals(reconstructor.getCurrentScale(), scale2, 0.0);
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
            final var scaleTransformation = new MetricTransformation3D(scale2);

            var numValidPoints = 0;
            double scaleX;
            double scaleY;
            double scaleZ;
            for (var i = start; i < numPoints1; i++) {
                final var point = points3D1.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3D.get(i - start);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleTransformation.transform(metricReconstructedPoints3D.get(i - start), rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                // check that scale error is less than 5%
                if (Math.abs(scaleX - baseline / scale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                if (Math.abs(scaleY - baseline / scale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                if (Math.abs(scaleZ - baseline / scale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline / scale2, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale2, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale2, ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR || Math.abs(scaleY - 1.0) > RELATIVE_ERROR
                        || Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / scale2,
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
                scaleTransformation.transform(metricReconstructedPoints3D.get(i + numPoints1 - start), rescaledPoint);

                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                // check that scale error is less than 5%
                if (Math.abs(scaleX - baseline / scale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                if (Math.abs(scaleY - baseline / scale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                if (Math.abs(scaleZ - baseline / scale2) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline / scale2, ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / scale2, ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / scale2, ABSOLUTE_ERROR);
                if (Math.abs(scaleX - 1.0) > RELATIVE_ERROR || Math.abs(scaleY - 1.0) > RELATIVE_ERROR
                        || Math.abs(scaleZ - 1.0) > RELATIVE_ERROR) {
                    continue;
                }
                rescaledPoint.setInhomogeneousCoordinates(
                        rescaledPoint.getInhomX() * baseline / scale2,
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

    private void reset() {
        viewCount = 0;
        estimatedFundamentalMatrix = estimatedFundamentalMatrix2 = null;
        estimatedMetricCamera1 = estimatedMetricCamera2 = estimatedMetricCamera3 = null;
        estimatedEuclideanCamera1 = estimatedEuclideanCamera2 = estimatedEuclideanCamera3 = null;
        metricReconstructedPoints = null;
        euclideanReconstructedPoints = null;
        started = finished = failed = cancelled = false;
        scale = 0.0;
        timestamp = 0;
        slamCamera = null;
        slamCovariance = null;
    }

    private static ConstantVelocityModelSlamCalibrator createFinishedCalibrator(
            final float accelerationOffsetX, final float accelerationOffsetY,
            final float accelerationOffsetZ, final float angularOffsetX, final float angularOffsetY,
            final float angularOffsetZ, final GaussianRandomizer noiseRandomizer) {
        final var calibrator = ConstantVelocityModelSlamEstimator.createCalibrator();
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

            if (calibrator.isFinished()) {
                break;
            }

            timestamp += DELTA_NANOS;
        }

        return calibrator;
    }
}
