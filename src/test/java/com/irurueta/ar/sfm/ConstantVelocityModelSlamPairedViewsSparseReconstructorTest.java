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
import com.irurueta.ar.slam.ConstantVelocityModelSlamCalibrationData;
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

class ConstantVelocityModelSlamPairedViewsSparseReconstructorTest {

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
    private static final int MAX_TRIES = 20000;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;

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
            ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.class.getSimpleName());

    private int viewCount = 0;
    private EstimatedFundamentalMatrix estimatedFundamentalMatrix;
    private EstimatedFundamentalMatrix estimatedFundamentalMatrix2;
    private EstimatedFundamentalMatrix estimatedFundamentalMatrix3;
    private EstimatedCamera estimatedEuclideanCamera1;
    private EstimatedCamera estimatedEuclideanCamera2;
    private EstimatedCamera estimatedEuclideanCamera2b;
    private EstimatedCamera estimatedEuclideanCamera3;
    private EstimatedCamera estimatedEuclideanCamera3b;
    private EstimatedCamera estimatedEuclideanCamera4;
    private List<ReconstructedPoint3D> euclideanReconstructedPoints;
    private List<ReconstructedPoint3D> euclideanReconstructedPoints2;
    private List<ReconstructedPoint3D> euclideanReconstructedPoints3;

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
        estimatedEuclideanCamera1 = estimatedEuclideanCamera2 = estimatedEuclideanCamera3 = estimatedEuclideanCamera4 =
                null;
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
        assertEquals(2, ConstantVelocityModelSlamPairedViewsSparseReconstructor.MIN_NUMBER_OF_VIEWS);

        final var configuration = new ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();
        final var listener = new ConstantVelocityModelSlamPairedViewsSparseReconstructorListener() {
            @Override
            public void onSlamDataAvailable(
                    final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                    final double positionX, final double positionY, final double positionZ,
                    final double velocityX, final double velocityY, final double velocityZ,
                    final double accelerationX, final double accelerationY, final double accelerationZ,
                    final double quaternionA, final double quaternionB, final double quaternionC,
                    final double quaternionD, final double angularSpeedX, final double angularSpeedY,
                    final double angularSpeedZ, final Matrix covariance) {
                // no action needed
            }

            @Override
            public void onSlamCameraEstimated(
                    final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                    final PinholeCamera camera) {
                // no action needed
            }

            @Override
            public boolean hasMoreViewsAvailable(
                    final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                return false;
            }

            @Override
            public void onRequestSamplesForCurrentViewPair(
                    final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                    final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                // no action needed
            }

            @Override
            public void onSamplesAccepted(
                    final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                    final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                // no action needed
            }

            @Override
            public void onSamplesRejected(
                    final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                    final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                // no action needed
            }

            @Override
            public void onRequestMatches(
                    final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                    final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
                    final List<MatchedSamples> matches) {
                // no action needed
            }

            @Override
            public void onFundamentalMatrixEstimated(
                    final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                    final int viewId2, final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                // no action needed
            }

            @Override
            public void onEuclideanCameraPairEstimated(
                    final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                    final int viewId2, final double scale, final EstimatedCamera camera1,
                    final EstimatedCamera camera2) {
                // no action needed
            }

            @Override
            public void onEuclideanReconstructedPointsEstimated(
                    final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                    final int viewId2, final double scale, final List<ReconstructedPoint3D> points) {
                // no action needed
            }

            @Override
            public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                    final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId) {
                return null;
            }

            @Override
            public void onStart(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onFinish(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onCancel(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onFail(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                // no action needed
            }
        };

        var reconstructor = new ConstantVelocityModelSlamPairedViewsSparseReconstructor(listener);

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
        reconstructor = new ConstantVelocityModelSlamPairedViewsSparseReconstructor(configuration, listener);

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
    void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithoutNoiseTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException, RotationException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var configuration = new ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

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

            final var listener = new ConstantVelocityModelSlamPairedViewsSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
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
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                        final PinholeCamera camera) {
                    slamCameraEstimated++;
                    slamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentViewPair(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                        final int viewId1, final int viewId2, final List<Sample2D> samples1,
                        final List<Sample2D> samples2) {

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

                    // assume the following accelerator and gyroscope samples
                    // are obtained during a period of 1 second between 1st
                    // and 2nd view (50 samples * 0.02 s/sample = 1 second)
                    timestamp = 0;
                    for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                        reconstructor.updateAccelerometerSample(timestamp, (float) accelerationX, (float) accelerationY,
                                (float) accelerationZ);
                        reconstructor.updateGyroscopeSample(timestamp, (float) angularSpeedX, (float) angularSpeedY,
                                (float) angularSpeedZ);
                        timestamp += DELTA_NANOS;
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                        final int viewId1, final int viewId2, final List<Sample2D> samples1,
                        final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onSamplesRejected(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onRequestMatches(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
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
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.this.estimatedFundamentalMatrix =
                            estimatedFundamentalMatrix;
                }

                @Override
                public void onEuclideanCameraPairEstimated(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final double scale, final EstimatedCamera camera1,
                        final EstimatedCamera camera2) {
                    estimatedEuclideanCamera1 = camera1;
                    estimatedEuclideanCamera2 = camera2;
                    ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final double scale, final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId) {
                    return intrinsic;
                }

                @Override
                public void onStart(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new ConstantVelocityModelSlamPairedViewsSparseReconstructor(configuration,
                    listener);

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

            final var euclideanCenter1 = estEuclideanCam1.getCameraCenter();
            final var euclideanCenter2 = estEuclideanCam2.getCameraCenter();

            final var euclideanIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var euclideanIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();

            final var euclideanRotation1 = estEuclideanCam1.getCameraRotation();
            final var euclideanRotation2 = estEuclideanCam2.getCameraRotation();

            // check scale
            final var estimatedBaseline = euclideanCenter1.distanceTo(euclideanCenter2);

            // check cameras are correct
            final var maxBaseline = Math.max(estimatedBaseline, baseline);
            final var absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);
            assertEquals(scale, estimatedBaseline, 5 * LARGE_ABSOLUTE_ERROR);

            // check cameras
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

            final var scaleTransformation = new MetricTransformation3D(baseline / scale);

            var numValidPoints = 0;
            double scaleX;
            double scaleY;
            double scaleZ;
            for (var i = 0; i < numPoints; i++) {
                final var point = points3D.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3D.get(i);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleTransformation.transform(euclideanPoint, rescaledPoint);

                // Euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                // check that scale error is less than 5%
                if (Math.abs(scaleX - 1.0) > LARGE_ABSOLUTE_ERROR || Math.abs(scaleY - 1.0) > LARGE_ABSOLUTE_ERROR
                        || Math.abs(scaleZ - 1.0) > LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(1.0, scaleX, LARGE_ABSOLUTE_ERROR);
                assertEquals(1.0, scaleY, LARGE_ABSOLUTE_ERROR);
                assertEquals(1.0, scaleZ, LARGE_ABSOLUTE_ERROR);
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
            LOGGER.log(Level.INFO, "Baseline relative error without noise: {0,number,0.000%}", scaleRelativeError);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithNoiseTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException, RotationException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var offsetRandomizer = new UniformRandomizer();
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var configuration = new ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

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

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            final var listener = new ConstantVelocityModelSlamPairedViewsSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
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
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                        final PinholeCamera camera) {
                    slamCameraEstimated++;
                    slamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentViewPair(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {

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

                @Override
                public void onSamplesAccepted(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onSamplesRejected(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onRequestMatches(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
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
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.this.estimatedFundamentalMatrix =
                            estimatedFundamentalMatrix;
                }

                @Override
                public void onEuclideanCameraPairEstimated(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final double scale, final EstimatedCamera camera1,
                        final EstimatedCamera camera2) {
                    estimatedEuclideanCamera1 = camera1;
                    estimatedEuclideanCamera2 = camera2;
                    ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final double scale, final List<ReconstructedPoint3D> points) {
                    euclideanReconstructedPoints = points;
                    ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.this.scale = scale;
                }

                @Override
                public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId) {
                    return intrinsic;
                }

                @Override
                public void onStart(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new ConstantVelocityModelSlamPairedViewsSparseReconstructor(configuration,
                    listener);

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

            final var euclideanCenter1 = estEuclideanCam1.getCameraCenter();
            final var euclideanCenter2 = estEuclideanCam2.getCameraCenter();

            final var euclideanIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var euclideanIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();

            final var euclideanRotation1 = estEuclideanCam1.getCameraRotation();
            final var euclideanRotation2 = estEuclideanCam2.getCameraRotation();

            // check scale
            final var estimatedBaseline = euclideanCenter1.distanceTo(euclideanCenter2);

            // check cameras are correct
            final var maxBaseline = Math.max(estimatedBaseline, baseline);
            final var absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);
            assertEquals(scale, estimatedBaseline, 2 * LARGE_ABSOLUTE_ERROR);

            // check cameras
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

            final var scaleTransformation = new MetricTransformation3D(baseline / scale);

            var numValidPoints = 0;
            double scaleX;
            double scaleY;
            double scaleZ;
            for (var i = 0; i < numPoints; i++) {
                final var point = points3D.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3D.get(i);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleTransformation.transform(euclideanPoint, rescaledPoint);

                // Euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                // check that scale error is less than 5%
                if (Math.abs(scaleX - 1.0) > LARGE_ABSOLUTE_ERROR || Math.abs(scaleY - 1.0) > LARGE_ABSOLUTE_ERROR
                        || Math.abs(scaleZ - 1.0) > LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(1.0, scaleX, LARGE_ABSOLUTE_ERROR);
                assertEquals(1.0, scaleY, LARGE_ABSOLUTE_ERROR);
                assertEquals(1.0, scaleZ, LARGE_ABSOLUTE_ERROR);
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
    void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithoutNoiseThreeViews()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException, RotationException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var configuration = new ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

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

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);
            final var axisRotation2 = new AxisRotation3D(rotation2);
            final var rotation3 = new MatrixRotation3D(alphaEuler3, betaEuler3, gammaEuler3);

            final var axis2X = axisRotation2.getAxisX();
            final var axis2Y = axisRotation2.getAxisY();
            final var axis2Z = axisRotation2.getAxisZ();
            final var angle2 = axisRotation2.getRotationAngle();

            var diffRotation = new AxisRotation3D(axis2X, axis2Y, axis2Z, angle2 / N_SENSOR_SAMPLES);
            Quaternion diffQuaternion = new Quaternion(diffRotation);

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

                    point3D = new InhomogeneousPoint3D(centralCommonPointPair1.getInhomX() + lambdaX,
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
                assertTrue(front1);
                assertTrue(front2);
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

                // check that 3D point is in front of 2nd pair of cameras
                assertTrue(front1);
                assertTrue(front2);
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

            final var listener = new ConstantVelocityModelSlamPairedViewsSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
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
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                        final PinholeCamera camera) {
                    slamCameraEstimated++;
                    slamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    // 3 views = 2 view pairs (2 images * 2 views --> 4 view counts)
                    return viewCount < 4;
                }

                @Override
                public void onRequestSamplesForCurrentViewPair(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {

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
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onSamplesRejected(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onRequestMatches(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final List<MatchedSamples> matches) {
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
                        match.setSamples(new Sample2D[]{samples1.get(i), samples2.get(i)});
                        match.setViewIds(new int[]{viewId1, viewId2});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    final var vCount = reconstructor.getViewCount();
                    if (vCount == 0) {
                        ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.this.estimatedFundamentalMatrix =
                                estimatedFundamentalMatrix;
                    } else if (vCount == 2) {
                        estimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                    }
                }

                @Override
                public void onEuclideanCameraPairEstimated(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final double scale, final EstimatedCamera camera1,
                        final EstimatedCamera camera2) {

                    final var vCount = reconstructor.getViewCount();
                    if (vCount == 0) {
                        estimatedEuclideanCamera1 = camera1;
                        estimatedEuclideanCamera2 = camera2;
                        ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.this.scale = scale;
                    } else if (vCount == 2) {
                        estimatedEuclideanCamera2b = camera1;
                        estimatedEuclideanCamera3 = camera2;
                        scale2 = scale;
                    }
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final double scale, final List<ReconstructedPoint3D> points) {

                    final var vCount = reconstructor.getViewCount();
                    if (vCount == 0) {
                        euclideanReconstructedPoints = points;
                        ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.this.scale = scale;
                    } else if (vCount == 2) {
                        euclideanReconstructedPoints2 = points;
                        scale2 = scale;
                    }
                }

                @Override
                public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId) {
                    return intrinsic;
                }

                @Override
                public void onStart(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new ConstantVelocityModelSlamPairedViewsSparseReconstructor(configuration,
                    listener);

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

            final var euclideanCenter1 = estEuclideanCam1.getCameraCenter();
            final var euclideanCenter2 = estEuclideanCam2.getCameraCenter();
            final var euclideanCenter2b = estEuclideanCam2b.getCameraCenter();
            final var euclideanCenter3 = estEuclideanCam3.getCameraCenter();

            final var euclideanIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var euclideanIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();
            final var euclideanIntrinsic2b = estEuclideanCam2b.getIntrinsicParameters();
            final var euclideanIntrinsic3 = estEuclideanCam3.getIntrinsicParameters();

            final var euclideanRotation1 = estEuclideanCam1.getCameraRotation();
            final var euclideanRotation2 = estEuclideanCam2.getCameraRotation();
            final var euclideanRotation2b = estEuclideanCam2b.getCameraRotation();
            final var euclideanRotation3 = estEuclideanCam3.getCameraRotation();

            // check scale
            final var euclideanBaseline = euclideanCenter1.distanceTo(euclideanCenter2);
            final var euclideanBaseline2 = euclideanCenter2b.distanceTo(euclideanCenter3);

            // check cameras are correct
            final var maxBaseline = Math.max(euclideanBaseline, baseline);
            final var absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(euclideanBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(euclideanBaseline, baseline, absoluteScaleError);
            assertEquals(scale, euclideanBaseline, 10 * LARGE_ABSOLUTE_ERROR);

            final var maxBaseline2 = Math.max(euclideanBaseline2, baseline2);
            final var absoluteScaleError2 = RELATIVE_ERROR * maxBaseline2;
            if (Math.abs(euclideanBaseline2 - baseline2) > absoluteScaleError2) {
                continue;
            }
            assertEquals(euclideanBaseline2, baseline2, absoluteScaleError2);
            assertEquals(scale2, euclideanBaseline2, 10 * LARGE_ABSOLUTE_ERROR);

            // check cameras
            assertTrue(center1.equals(euclideanCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(euclideanCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2, absoluteScaleError));
            if (!center2.equals(euclideanCenter2b, absoluteScaleError2)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2b, absoluteScaleError2));
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

            assertEquals(euclideanIntrinsic2b.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
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
            assertTrue(euclideanRotation2b.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(euclideanRotation3.asInhomogeneousMatrix().equals(rotation3.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct (after scale correction)

            // check that scale error is less than 5%
            assertTrue(Math.abs(baseline / scale - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline2 / scale2 - 1.0) < RELATIVE_ERROR);

            final MetricTransformation3D scaleTransformation = new MetricTransformation3D(baseline / scale);
            final MetricTransformation3D scaleTransformation2 = new MetricTransformation3D(baseline2 / scale2);

            numValidPoints = 0;
            double scaleX;
            double scaleY;
            double scaleZ;
            for (var i = 0; i < numPointsPair1; i++) {
                final var point = points3DPair1.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3DPair1.get(i);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleTransformation.transform(euclideanPoint, rescaledPoint);

                // Euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

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

                numValidPoints++;
            }

            if (numValidPoints == 0) {
                continue;
            }

            numValidPoints = 0;
            for (var i = 0; i < numPointsPair2; i++) {
                final var point = points3DPair2.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3DPair2.get(i);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleTransformation2.transform(euclideanPoint, rescaledPoint);

                // Euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                // check Euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                // check that scale error is less than 5%
                if (Math.abs(scaleX - baseline2 / scale2) > 5 * LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline2 / scale2, 5 * LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleY - baseline2 / scale2) > 5 * LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline2 / scale2, 5 * LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleZ - baseline2 / scale2) > 5 * LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline2 / scale2, 5 * LARGE_ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);

                numValidPoints++;
            }

            if (numValidPoints == 0) {
                continue;
            }


            final var scaleRelativeError = Math.abs(baseline / scale - 1.0);
            final var scaleRelativeError2 = Math.abs(baseline2 / scale2 - 1.0);
            LOGGER.log(Level.INFO, "Baseline relative error without noise 1: {0,number,0.000%}",
                    scaleRelativeError);
            LOGGER.log(Level.INFO, "Baseline relative error without noise 2: {0,number,0.000%}",
                    scaleRelativeError2);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithNoiseThreeViews()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException, RotationException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var offsetRandomizer = new UniformRandomizer();
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var configuration = new ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

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

                    point3D = new InhomogeneousPoint3D(centralCommonPointPair1.getInhomX() + lambdaX,
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
                assertTrue(front1);
                assertTrue(front2);
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

                // check that 3D point is in front of 2nd pair of cameras
                assertTrue(front1);
                assertTrue(front2);
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

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            final var listener = new ConstantVelocityModelSlamPairedViewsSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
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
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                        final PinholeCamera camera) {
                    slamCameraEstimated++;
                    slamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    // 3 views = 2 view pairs (2 images * 2 views --> 4 view counts)
                    return viewCount < 4;
                }

                @Override
                public void onRequestSamplesForCurrentViewPair(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {

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
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onSamplesRejected(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onRequestMatches(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final List<MatchedSamples> matches) {
                    matches.clear();

                    final var vCount = reconstructor.getViewCount();
                    int numPoints;
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
                        match.setSamples(new Sample2D[]{samples1.get(i), samples2.get(i)});
                        match.setViewIds(new int[]{viewId1, viewId2});
                        matches.add(match);
                    }
                }

                @Override
                public void onFundamentalMatrixEstimated(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    final var vCount = reconstructor.getViewCount();
                    if (vCount == 0) {
                        ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.this.estimatedFundamentalMatrix =
                                estimatedFundamentalMatrix;
                    } else if (vCount == 2) {
                        estimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                    }
                }

                @Override
                public void onEuclideanCameraPairEstimated(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final double scale, final EstimatedCamera camera1,
                        final EstimatedCamera camera2) {

                    final var vCount = reconstructor.getViewCount();
                    if (vCount == 0) {
                        estimatedEuclideanCamera1 = camera1;
                        estimatedEuclideanCamera2 = camera2;
                        ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.this.scale = scale;
                    } else if (vCount == 2) {
                        estimatedEuclideanCamera2b = camera1;
                        estimatedEuclideanCamera3 = camera2;
                        scale2 = scale;
                    }
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final double scale, final List<ReconstructedPoint3D> points) {

                    final var vCount = reconstructor.getViewCount();
                    if (vCount == 0) {
                        euclideanReconstructedPoints = points;
                        ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.this.scale = scale;
                    } else if (vCount == 2) {
                        euclideanReconstructedPoints2 = points;
                        scale2 = scale;
                    }
                }

                @Override
                public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId) {
                    return intrinsic;
                }

                @Override
                public void onStart(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new ConstantVelocityModelSlamPairedViewsSparseReconstructor(configuration,
                    listener);

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

            final var euclideanCenter1 = estEuclideanCam1.getCameraCenter();
            final var euclideanCenter2 = estEuclideanCam2.getCameraCenter();
            final var euclideanCenter2b = estEuclideanCam2b.getCameraCenter();
            final var euclideanCenter3 = estEuclideanCam3.getCameraCenter();

            final var euclideanIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var euclideanIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();
            final var euclideanIntrinsic2b = estEuclideanCam2b.getIntrinsicParameters();
            final var euclideanIntrinsic3 = estEuclideanCam3.getIntrinsicParameters();

            final var euclideanRotation1 = estEuclideanCam1.getCameraRotation();
            final var euclideanRotation2 = estEuclideanCam2.getCameraRotation();
            final var euclideanRotation2b = estEuclideanCam2b.getCameraRotation();
            final var euclideanRotation3 = estEuclideanCam3.getCameraRotation();

            // check scale
            final var euclideanBaseline = euclideanCenter1.distanceTo(euclideanCenter2);
            final var euclideanBaseline2 = euclideanCenter2b.distanceTo(euclideanCenter3);

            // check cameras are correct
            final var maxBaseline = Math.max(euclideanBaseline, baseline);
            final var absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(euclideanBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(euclideanBaseline, baseline, absoluteScaleError);
            assertEquals(scale, euclideanBaseline, 10 * LARGE_ABSOLUTE_ERROR);

            final var maxBaseline2 = Math.max(euclideanBaseline2, baseline2);
            final var absoluteScaleError2 = RELATIVE_ERROR * maxBaseline2;
            if (Math.abs(euclideanBaseline2 - baseline2) > absoluteScaleError2) {
                continue;
            }
            assertEquals(euclideanBaseline2, baseline2, absoluteScaleError2);
            assertEquals(scale2, euclideanBaseline2, 10 * LARGE_ABSOLUTE_ERROR);

            // check cameras
            assertTrue(center1.equals(euclideanCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(euclideanCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2, absoluteScaleError));
            if (!center2.equals(euclideanCenter2b, absoluteScaleError2)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2b, absoluteScaleError2));
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

            assertEquals(euclideanIntrinsic2b.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
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
            assertTrue(euclideanRotation2b.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(euclideanRotation3.asInhomogeneousMatrix().equals(rotation3.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct (after scale correction)

            // check that scale error is less than 5%
            assertTrue(Math.abs(baseline / scale - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline2 / scale2 - 1.0) < RELATIVE_ERROR);

            final var scaleTransformation = new MetricTransformation3D(baseline / scale);
            final var scaleTransformation2 = new MetricTransformation3D(baseline2 / scale2);

            numValidPoints = 0;
            double scaleX;
            double scaleY;
            double scaleZ;
            for (var i = 0; i < numPointsPair1; i++) {
                final var point = points3DPair1.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3DPair1.get(i);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleTransformation.transform(euclideanPoint, rescaledPoint);

                // Euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

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

                numValidPoints++;
            }

            if (numValidPoints == 0) {
                continue;
            }

            numValidPoints = 0;
            for (var i = 0; i < numPointsPair2; i++) {
                final var point = points3DPair2.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3DPair2.get(i);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleTransformation2.transform(euclideanPoint, rescaledPoint);

                // Euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                // check Euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                // check that scale error is less than 5%
                if (Math.abs(scaleX - baseline2 / scale2) > 5 * LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline2 / scale2, 5 * LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleY - baseline2 / scale2) > 5 * LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline2 / scale2, 5 * LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleZ - baseline2 / scale2) > 5 * LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline2 / scale2, 5 * LARGE_ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);

                numValidPoints++;
            }

            if (numValidPoints == 0) {
                continue;
            }

            final var scaleRelativeError = Math.abs(baseline / scale - 1.0);
            final var scaleRelativeError2 = Math.abs(baseline2 / scale2 - 1.0);
            LOGGER.log(Level.INFO, "Baseline relative error without noise 1: {0,number,0.000%}",
                    scaleRelativeError);
            LOGGER.log(Level.INFO, "Baseline relative error without noise 2: {0,number,0.000%}",
                    scaleRelativeError2);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithoutNoiseFourViews()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException, RotationException,
            com.irurueta.geometry.estimators.NotReadyException, com.irurueta.geometry.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < 2 * TIMES; t++) {
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var configuration = new ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

            final var accelerationOffsetX = 0.0f;
            final var accelerationOffsetY = 0.0f;
            final var accelerationOffsetZ = 0.0f;

            final var angularOffsetX = 0.0f;
            final var angularOffsetY = 0.0f;
            final var angularOffsetZ = 0.0f;

            final var calibrator = createFinishedCalibrator(accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final ConstantVelocityModelSlamCalibrationData calibrationData = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

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
            final var alphaEuler4 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler4 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler4 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);
            final var axisRotation2 = new AxisRotation3D(rotation2);
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

            var accumDiffRotation = rotation2.inverseRotationAndReturnNew().combineAndReturnNew(rotation3)
                    .toAxisRotation();
            final var axis3X = accumDiffRotation.getAxisX();
            final var axis3Y = accumDiffRotation.getAxisY();
            final var axis3Z = accumDiffRotation.getAxisZ();
            final var angle3 = accumDiffRotation.getRotationAngle();

            diffRotation = new AxisRotation3D(axis3X, axis3Y, axis3Z, angle3 / N_SENSOR_SAMPLES);
            diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
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
            diffRotation2 = new Quaternion(angularSpeed4X, angularSpeed4Y, angularSpeed4Z);

            // number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            // previously sampled rotation
            final var rotation4b = new MatrixRotation3D(rotation3b);
            final var rotation4c = new MatrixRotation3D(rotation3c);
            for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation4b.combine(diffRotation);
                rotation4c.combine(diffRotation2);
            }

            // check that rotations created by composing sensor samples are equal
            // to the original one
            if (!rotation4.equals(rotation4b, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!rotation4.equals(rotation4c, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(rotation4.equals(rotation4b, ABSOLUTE_ERROR));
            assertTrue(rotation4.equals(rotation4c, ABSOLUTE_ERROR));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            final var cameraSeparation2 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            final var cameraSeparation3 = randomizer.nextDouble(MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);
            final var center3 = new InhomogeneousPoint3D(center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2, center2.getInhomZ() + cameraSeparation2);
            final var center4 = new InhomogeneousPoint3D(center3.getInhomX() + cameraSeparation3,
                    center3.getInhomY() + cameraSeparation3, center3.getInhomZ() + cameraSeparation3);

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

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);
            final var fundamentalMatrix2 = new FundamentalMatrix(camera2, camera3);
            final var fundamentalMatrix3 = new FundamentalMatrix(camera3, camera4);

            // create 3D points laying in front of all cameras

            // 1st find an approximate central point by intersecting the axis planes of
            // all cameras
            final var horizontalPlane1 = camera1.getHorizontalAxisPlane();
            final var verticalPlane1 = camera1.getVerticalAxisPlane();
            final var horizontalPlane2 = camera2.getHorizontalAxisPlane();
            final var verticalPlane2 = camera2.getVerticalAxisPlane();
            final var horizontalPlane3 = camera3.getHorizontalAxisPlane();
            final var verticalPlane3 = camera3.getVerticalAxisPlane();
            final var horizontalPlane4 = camera4.getHorizontalAxisPlane();
            final var verticalPlane4 = camera4.getVerticalAxisPlane();
            final var planesIntersectionMatrixPair1 = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            final var planesIntersectionMatrixPair2 = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            final var planesIntersectionMatrixPair3 = new Matrix(Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
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

            planesIntersectionMatrixPair3.setElementAt(0, 0, verticalPlane3.getA());
            planesIntersectionMatrixPair3.setElementAt(0, 1, verticalPlane3.getB());
            planesIntersectionMatrixPair3.setElementAt(0, 2, verticalPlane3.getC());
            planesIntersectionMatrixPair3.setElementAt(0, 3, verticalPlane3.getD());

            planesIntersectionMatrixPair3.setElementAt(1, 0, horizontalPlane3.getA());
            planesIntersectionMatrixPair3.setElementAt(1, 1, horizontalPlane3.getB());
            planesIntersectionMatrixPair3.setElementAt(1, 2, horizontalPlane3.getC());
            planesIntersectionMatrixPair3.setElementAt(1, 3, horizontalPlane3.getD());

            planesIntersectionMatrixPair3.setElementAt(2, 0, verticalPlane4.getA());
            planesIntersectionMatrixPair3.setElementAt(2, 1, verticalPlane4.getB());
            planesIntersectionMatrixPair3.setElementAt(2, 2, verticalPlane4.getC());
            planesIntersectionMatrixPair3.setElementAt(2, 3, verticalPlane4.getD());

            planesIntersectionMatrixPair3.setElementAt(3, 0, horizontalPlane4.getA());
            planesIntersectionMatrixPair3.setElementAt(3, 1, horizontalPlane4.getB());
            planesIntersectionMatrixPair3.setElementAt(3, 2, horizontalPlane4.getC());
            planesIntersectionMatrixPair3.setElementAt(3, 3, horizontalPlane4.getD());

            final var decomposerPair1 = new SingularValueDecomposer(planesIntersectionMatrixPair1);
            decomposerPair1.decompose();
            final var vPair1 = decomposerPair1.getV();

            final var decomposerPair2 = new SingularValueDecomposer(planesIntersectionMatrixPair2);
            decomposerPair2.decompose();
            final var vPair2 = decomposerPair2.getV();

            final var decomposerPair3 = new SingularValueDecomposer(planesIntersectionMatrixPair3);
            decomposerPair3.decompose();
            final var vPair3 = decomposerPair3.getV();

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

            final var centralCommonPointPair3 = new HomogeneousPoint3D(
                    vPair3.getElementAt(0, 3),
                    vPair3.getElementAt(1, 3),
                    vPair3.getElementAt(2, 3),
                    vPair3.getElementAt(3, 3));

            double lambdaX;
            double lambdaY;
            double lambdaZ;

            final var numPointsPair1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPointsPair2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final var numPointsPair3 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            final var points3DPair1 = new ArrayList<InhomogeneousPoint3D>();
            final var points3DPair2 = new ArrayList<InhomogeneousPoint3D>();
            final var points3DPair3 = new ArrayList<InhomogeneousPoint3D>();
            Point2D projectedPoint1;
            Point2D projectedPoint2;
            Point2D projectedPoint3;
            Point2D projectedPoint4;
            final var projectedPoints1 = new ArrayList<Point2D>();
            final var projectedPoints2a = new ArrayList<Point2D>();
            final var projectedPoints2b = new ArrayList<Point2D>();
            final var projectedPoints3 = new ArrayList<Point2D>();
            final var projectedPoints3b = new ArrayList<Point2D>();
            final var projectedPoints4 = new ArrayList<Point2D>();
            boolean front1;
            boolean front2;
            boolean front3;
            boolean front4;
            for (var i = 0; i < numPointsPair1; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(centralCommonPointPair1.getInhomX() + lambdaX,
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
                assertTrue(front1);
                assertTrue(front2);
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

                // check that 3D point is in front of 2nd pair of cameras
                assertTrue(front1);
                assertTrue(front2);
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

            var failedIter = false;
            for (var i = 0; i < numPointsPair3; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            center3.getInhomX() + centralCommonPointPair3.getInhomX() + lambdaX,
                            center3.getInhomY() + centralCommonPointPair3.getInhomY() + lambdaY,
                            center3.getInhomZ() + centralCommonPointPair2.getInhomZ() + lambdaZ);

                    front3 = camera3.isPointInFrontOfCamera(point3D);
                    front4 = camera4.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        failedIter = true;
                        break;
                    }
                    numTry++;
                } while (!front3 || !front4);

                if (failedIter) {
                    break;
                }

                // check that 3D point is in front of 2nd pair of cameras
                assertTrue(front3);
                assertTrue(front4);

                points3DPair3.add(point3D);

                // project 3D point into 2nd pair of cameras
                projectedPoint3 = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3);
                projectedPoints3b.add(projectedPoint3);

                projectedPoint4 = new InhomogeneousPoint2D();
                camera4.project(point3D, projectedPoint4);
                projectedPoints4.add(projectedPoint4);
            }

            if (failedIter) {
                continue;
            }

            final var listener = new ConstantVelocityModelSlamPairedViewsSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
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
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                        final PinholeCamera camera) {
                    slamCameraEstimated++;
                    slamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    // 4 views = 3 view pairs (2 images * 3 views --> 6 view counts)
                    return viewCount < 6;
                }

                @Override
                public void onRequestSamplesForCurrentViewPair(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {

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
                    } else if (vCount == 4) {
                        // third view pair
                        for (var i = 0; i < numPointsPair3; i++) {
                            sample1 = new Sample2D();
                            sample1.setPoint(projectedPoints3b.get(i));
                            sample1.setViewId(viewId1);
                            samples1.add(sample1);

                            sample2 = new Sample2D();
                            sample2.setPoint(projectedPoints4.get(i));
                            sample2.setViewId(viewId2);
                            samples2.add(sample2);
                        }

                        // assume the following accelerator and gyroscope samples
                        // are obtained during a period of 1 second between 3rd
                        // and 4th view (50 samples * 0.02 s/sample = 1 second)
                        for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                            reconstructor.updateAccelerometerSample(timestamp, (float) accelerationX3,
                                    (float) accelerationY3, (float) accelerationZ3);
                            reconstructor.updateGyroscopeSample(timestamp, (float) angularSpeed4X,
                                    (float) angularSpeed4Y, (float) angularSpeed4Z);
                            timestamp += DELTA_NANOS;
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onSamplesRejected(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2) {
                    viewCount += 2;
                }

                @Override
                public void onRequestMatches(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final List<Sample2D> samples1, final List<Sample2D> samples2,
                        final List<MatchedSamples> matches) {
                    matches.clear();

                    final var vCount = reconstructor.getViewCount();
                    int numPoints;
                    if (vCount == 0) {
                        // first view pair
                        numPoints = numPointsPair1;
                    } else if (vCount == 2) {
                        // second view pair
                        numPoints = numPointsPair2;
                    } else {
                        // third view pair
                        numPoints = numPointsPair3;
                    }

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
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    final var vCount = reconstructor.getViewCount();
                    if (vCount == 0) {
                        ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.this.estimatedFundamentalMatrix =
                                estimatedFundamentalMatrix;
                    } else if (vCount == 2) {
                        estimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                    } else if (vCount == 4) {
                        estimatedFundamentalMatrix3 = estimatedFundamentalMatrix;
                    }
                }

                @Override
                public void onEuclideanCameraPairEstimated(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final double scale, final EstimatedCamera camera1,
                        final EstimatedCamera camera2) {

                    final var vCount = reconstructor.getViewCount();
                    if (vCount == 0) {
                        estimatedEuclideanCamera1 = camera1;
                        estimatedEuclideanCamera2 = camera2;
                        ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.this.scale = scale;
                    } else if (vCount == 2) {
                        estimatedEuclideanCamera2b = camera1;
                        estimatedEuclideanCamera3 = camera2;
                        scale2 = scale;
                    } else if (vCount == 4) {
                        estimatedEuclideanCamera3b = camera1;
                        estimatedEuclideanCamera4 = camera2;
                    }
                }

                @Override
                public void onEuclideanReconstructedPointsEstimated(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final double scale, final List<ReconstructedPoint3D> points) {

                    final var vCount = reconstructor.getViewCount();
                    if (vCount == 0) {
                        euclideanReconstructedPoints = points;
                        ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.this.scale = scale;
                    } else if (vCount == 2) {
                        euclideanReconstructedPoints2 = points;
                        scale2 = scale;
                    } else if (vCount == 4) {
                        euclideanReconstructedPoints3 = points;
                        scale3 = scale;
                    }
                }

                @Override
                public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                        final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, final int viewId) {
                    return intrinsic;
                }

                @Override
                public void onStart(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                    ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.this.failed = true;
                }
            };

            final var reconstructor = new ConstantVelocityModelSlamPairedViewsSparseReconstructor(configuration,
                    listener);

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
            assertTrue(slamDataAvailable > 0);
            assertTrue(slamCameraEstimated > 0);
            assertNotNull(slamCamera);
            assertNotNull(slamCovariance);
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.isAdditionalViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(estimatedFundamentalMatrix3, reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera4, reconstructor.getCurrentEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(estimatedEuclideanCamera3b, reconstructor.getPreviousEuclideanEstimatedCamera());
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(euclideanReconstructedPoints3, reconstructor.getEuclideanReconstructedPoints());
            assertEquals(scale3, reconstructor.getCurrentScale(), 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

            // check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            fundamentalMatrix2.normalize();
            fundamentalMatrix3.normalize();
            estimatedFundamentalMatrix.getFundamentalMatrix().normalize();
            estimatedFundamentalMatrix2.getFundamentalMatrix().normalize();
            estimatedFundamentalMatrix3.getFundamentalMatrix().normalize();

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
            if (!fundamentalMatrix3.getInternalMatrix().equals(
                    estimatedFundamentalMatrix3.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundamentalMatrix3.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                            estimatedFundamentalMatrix3.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix3.getInternalMatrix().equals(
                    estimatedFundamentalMatrix3.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundamentalMatrix3.getInternalMatrix().multiplyByScalarAndReturnNew(-1).equals(
                            estimatedFundamentalMatrix3.getFundamentalMatrix().getInternalMatrix(), ABSOLUTE_ERROR));

            var estEuclideanCam1 = this.estimatedEuclideanCamera1.getCamera();
            var estEuclideanCam2 = this.estimatedEuclideanCamera2.getCamera();
            var estEuclideanCam2b = this.estimatedEuclideanCamera2b.getCamera();
            var estEuclideanCam3 = this.estimatedEuclideanCamera3.getCamera();
            var estEuclideanCam3b = this.estimatedEuclideanCamera3b.getCamera();
            var estEuclideanCam4 = this.estimatedEuclideanCamera4.getCamera();

            estEuclideanCam1.decompose();
            estEuclideanCam2.decompose();
            estEuclideanCam2b.decompose();
            estEuclideanCam3.decompose();
            estEuclideanCam3b.decompose();
            estEuclideanCam4.decompose();

            final var euclideanReconstructedPoints3DPair1 = new ArrayList<Point3D>();
            for (var i = 0; i < numPointsPair1; i++) {
                euclideanReconstructedPoints3DPair1.add(euclideanReconstructedPoints.get(i).getPoint());
            }

            final var euclideanReconstructedPoints3DPair2 = new ArrayList<Point3D>();
            for (var i = 0; i < numPointsPair2; i++) {
                euclideanReconstructedPoints3DPair2.add(euclideanReconstructedPoints2.get(i).getPoint());
            }

            final var euclideanReconstructedPoints3DPair3 = new ArrayList<Point3D>();
            for (var i = 0; i < numPointsPair3; i++) {
                euclideanReconstructedPoints3DPair3.add(euclideanReconstructedPoints3.get(i).getPoint());
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

            numValidPoints = 0;
            numInvalidPoints = 0;
            for (var i = 0; i < numPointsPair3; i++) {
                final var p = euclideanReconstructedPoints3DPair3.get(i);
                if (estEuclideanCam3b.isPointInFrontOfCamera(p)
                        && estEuclideanCam4.isPointInFrontOfCamera(p)) {

                    assertTrue(estEuclideanCam3b.isPointInFrontOfCamera(p));
                    assertTrue(estEuclideanCam4.isPointInFrontOfCamera(p));

                    numValidPoints++;
                } else {
                    numInvalidPoints++;
                }
            }

            assertTrue(numValidPoints > numInvalidPoints);

            final var euclideanCenter1 = estEuclideanCam1.getCameraCenter();
            final var euclideanCenter2 = estEuclideanCam2.getCameraCenter();
            final var euclideanCenter2b = estEuclideanCam2b.getCameraCenter();
            final var euclideanCenter3 = estEuclideanCam3.getCameraCenter();
            final var euclideanCenter3b = estEuclideanCam3b.getCameraCenter();
            final var euclideanCenter4 = estEuclideanCam4.getCameraCenter();

            final var euclideanIntrinsic1 = estEuclideanCam1.getIntrinsicParameters();
            final var euclideanIntrinsic2 = estEuclideanCam2.getIntrinsicParameters();
            final var euclideanIntrinsic2b = estEuclideanCam2b.getIntrinsicParameters();
            final var euclideanIntrinsic3 = estEuclideanCam3.getIntrinsicParameters();
            final var euclideanIntrinsic3b = estEuclideanCam3b.getIntrinsicParameters();
            final var euclideanIntrinsic4 = estEuclideanCam4.getIntrinsicParameters();

            final var euclideanRotation1 = estEuclideanCam1.getCameraRotation();
            final var euclideanRotation2 = estEuclideanCam2.getCameraRotation();
            final var euclideanRotation2b = estEuclideanCam2b.getCameraRotation();
            final var euclideanRotation3 = estEuclideanCam3.getCameraRotation();
            final var euclideanRotation3b = estEuclideanCam3b.getCameraRotation();
            final var euclideanRotation4 = estEuclideanCam4.getCameraRotation();

            // check scale
            final var euclideanBaseline = euclideanCenter1.distanceTo(euclideanCenter2);
            final var euclideanBaseline2 = euclideanCenter2b.distanceTo(euclideanCenter3);
            final var euclideanBaseline3 = euclideanCenter3b.distanceTo(euclideanCenter4);

            // check cameras are correct
            final var maxBaseline = Math.max(euclideanBaseline, baseline);
            final var absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(euclideanBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(euclideanBaseline, baseline, absoluteScaleError);
            assertEquals(scale, euclideanBaseline, 10 * LARGE_ABSOLUTE_ERROR);

            final var maxBaseline2 = Math.max(euclideanBaseline2, baseline2);
            final var absoluteScaleError2 = RELATIVE_ERROR * maxBaseline2;
            if (Math.abs(euclideanBaseline2 - baseline2) > absoluteScaleError2) {
                continue;
            }
            assertEquals(euclideanBaseline2, baseline2, absoluteScaleError2);
            assertEquals(scale2, euclideanBaseline2, 10 * LARGE_ABSOLUTE_ERROR);

            final var maxBaseline3 = Math.max(euclideanBaseline3, baseline3);
            final var absoluteScaleError3 = RELATIVE_ERROR * maxBaseline3;
            if (Math.abs(euclideanBaseline3 - baseline3) > absoluteScaleError3) {
                continue;
            }
            assertEquals(euclideanBaseline3, baseline3, absoluteScaleError3);
            assertEquals(scale3, euclideanBaseline3, 20 * LARGE_ABSOLUTE_ERROR);

            // check cameras
            assertTrue(center1.equals(euclideanCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(euclideanCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2, absoluteScaleError));
            if (!center2.equals(euclideanCenter2b, absoluteScaleError2)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2b, absoluteScaleError2));
            if (!center3.equals(euclideanCenter3, absoluteScaleError2)) {
                continue;
            }
            assertTrue(center3.equals(euclideanCenter3, absoluteScaleError2));
            if (!center3.equals(euclideanCenter3b, absoluteScaleError3)) {
                continue;
            }
            assertTrue(center3.equals(euclideanCenter3b, absoluteScaleError3));
            if (!center4.equals(euclideanCenter4, absoluteScaleError3)) {
                continue;
            }
            assertTrue(center4.equals(euclideanCenter4, absoluteScaleError3));

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

            assertEquals(euclideanIntrinsic2b.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
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

            assertEquals(euclideanIntrinsic3b.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3b.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3b.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3b.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3b.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(),
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
            assertTrue(euclideanRotation2b.asInhomogeneousMatrix().equals(rotation2.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(euclideanRotation3.asInhomogeneousMatrix().equals(rotation3.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(euclideanRotation3b.asInhomogeneousMatrix().equals(rotation3.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));
            assertTrue(euclideanRotation4.asInhomogeneousMatrix().equals(rotation4.asInhomogeneousMatrix(),
                    ABSOLUTE_ERROR));

            // check that points are correct (after scale correction)

            // check that scale error is less than 5%
            assertTrue(Math.abs(baseline / scale - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline2 / scale2 - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline3 / scale3 - 1.0) < RELATIVE_ERROR);

            final var scaleTransformation = new MetricTransformation3D(baseline / scale);
            final var scaleTransformation2 = new MetricTransformation3D(baseline2 / scale2);
            final var scaleTransformation3 = new MetricTransformation3D(baseline3 / scale3);

            numValidPoints = 0;
            double scaleX;
            double scaleY;
            double scaleZ;
            for (var i = 0; i < numPointsPair1; i++) {
                final var point = points3DPair1.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3DPair1.get(i);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleTransformation.transform(euclideanPoint, rescaledPoint);

                // Euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

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

                numValidPoints++;
            }

            if (numValidPoints == 0) {
                continue;
            }

            numValidPoints = 0;
            for (var i = 0; i < numPointsPair2; i++) {
                final var point = points3DPair2.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3DPair2.get(i);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleTransformation2.transform(euclideanPoint, rescaledPoint);

                // Euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                // check Euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                // check that scale error is less than 5%
                if (Math.abs(scaleX - baseline2 / scale2) > 5 * LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline2 / scale2, 5 * LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleY - baseline2 / scale2) > 5 * LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline2 / scale2, 5 * LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleZ - baseline2 / scale2) > 5 * LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline2 / scale2, 5 * LARGE_ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);

                numValidPoints++;
            }

            if (numValidPoints == 0) {
                continue;
            }

            numValidPoints = 0;
            for (var i = 0; i < numPointsPair3; i++) {
                final var point = points3DPair3.get(i);
                final var euclideanPoint = euclideanReconstructedPoints3DPair3.get(i);

                // check metric points
                final var rescaledPoint = Point3D.create();
                scaleTransformation3.transform(euclideanPoint, rescaledPoint);

                // Euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                // check Euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                // check that scale error is less than 5%
                if (Math.abs(scaleX - baseline3 / scale3) > 5 * LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline3 / scale3, 5 * LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleY - baseline3 / scale3) > 5 * LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline3 / scale3, 5 * LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleZ - baseline3 / scale3) > 5 * LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline3 / scale3, 5 * LARGE_ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);

                numValidPoints++;
            }

            if (numValidPoints == 0) {
                continue;
            }

            final var scaleRelativeError = Math.abs(baseline / scale - 1.0);
            final var scaleRelativeError2 = Math.abs(baseline2 / scale2 - 1.0);
            final var scaleRelativeError3 = Math.abs(baseline3 / scale3 - 1.0);
            LOGGER.log(Level.INFO, "Baseline relative error without noise 1: {0,number,0.000%}",
                    scaleRelativeError);
            LOGGER.log(Level.INFO, "Baseline relative error without noise 2: {0,number,0.000%}",
                    scaleRelativeError2);
            LOGGER.log(Level.INFO, "Baseline relative error without noise 2: {0,number,0.000%}",
                    scaleRelativeError3);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private void reset() {
        viewCount = 0;
        estimatedFundamentalMatrix = estimatedFundamentalMatrix2 = estimatedFundamentalMatrix3 = null;
        estimatedEuclideanCamera1 = estimatedEuclideanCamera2 = estimatedEuclideanCamera3 = estimatedEuclideanCamera4 =
                null;
        euclideanReconstructedPoints = null;
        started = finished = failed = cancelled = false;
        scale = 0.0;
        timestamp = 0;
        slamDataAvailable = 0;
        slamCameraEstimated = 0;
        slamCamera = null;
    }

    private static ConstantVelocityModelSlamCalibrator createFinishedCalibrator(
            final float accelerationOffsetX, final float accelerationOffsetY, final float accelerationOffsetZ,
            final float angularOffsetX, final float angularOffsetY, final float angularOffsetZ,
            final GaussianRandomizer noiseRandomizer) {
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
