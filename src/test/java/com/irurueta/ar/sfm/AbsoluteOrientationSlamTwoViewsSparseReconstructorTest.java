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
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.jupiter.api.Assertions.*;

class AbsoluteOrientationSlamTwoViewsSparseReconstructorTest {

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

    // 5% of relative error in scale estimation
    private static final double RELATIVE_ERROR = 0.05;

    private static final int MAX_CALIBRATION_SAMPLES = 10000;

    // conversion from milliseconds to nanoseconds
    private static final int MILLIS_TO_NANOS = 1000000;

    // time between samples expressed in nanoseconds (a typical sensor in Android
    // delivers a sample every 20ms)
    private static final int DELTA_NANOS = 20000000; //0.02 seconds

    private static final float MIN_CALIBRATION_OFFSET = -1e-4f;
    private static final float MAX_CALIBRATION_OFFSET = 1e-4f;

    private static final double ACCELERATION_NOISE_STANDARD_DEVIATION = 1e-4;
    private static final double ANGULAR_SPEED_NOISE_STANDARD_DEVIATION = 1e-4;

    private static final int N_SENSOR_SAMPLES = 50;

    private static final Logger LOGGER = Logger.getLogger(
            AbsoluteOrientationSlamTwoViewsSparseReconstructorTest.class.getSimpleName());

    private int viewCount = 0;
    private EstimatedFundamentalMatrix estimatedFundamentalMatrix;
    private EstimatedCamera estimatedCamera1;
    private EstimatedCamera estimatedCamera2;
    private List<ReconstructedPoint3D> reconstructedPoints;

    private boolean started;
    private boolean finished;
    private boolean failed;
    private boolean cancelled;

    private int slamDataAvailable;
    private int slamCameraEstimated;

    private PinholeCamera slamCamera;
    private Matrix slamCovariance;

    @BeforeEach
    void setUp() {
        viewCount = 0;
        estimatedFundamentalMatrix = null;
        estimatedCamera1 = estimatedCamera2 = null;
        reconstructedPoints = null;
        started = finished = failed = cancelled = false;
        slamDataAvailable = 0;
        slamCameraEstimated = 0;
        slamCamera = null;
        slamCovariance = null;
    }

    @Test
    void testConstructor() {
        final var configuration = new AbsoluteOrientationSlamTwoViewsSparseReconstructorConfiguration();
        final var listener = new AbsoluteOrientationSlamTwoViewsSparseReconstructorListener() {
            @Override
            public void onSlamDataAvailable(
                    final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor,
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
                    final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor,
                    final PinholeCamera camera) {
                // no action needed
            }

            @Override
            public boolean hasMoreViewsAvailable(
                    final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor) {
                return false;
            }

            @Override
            public void onRequestSamplesForCurrentView(
                    final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor, final int viewId,
                    final List<Sample2D> samples) {
                // no action needed
            }

            @Override
            public void onSamplesAccepted(
                    final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor, final int viewId,
                    final List<Sample2D> samples) {
                // no action needed
            }

            @Override
            public void onSamplesRejected(
                    final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor, final int viewId,
                    final List<Sample2D> samples) {
                // no action needed
            }

            @Override
            public void onRequestMatches(
                    final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor,
                    final List<Sample2D> samples1, final List<Sample2D> samples2, final int viewId1, final int viewId2,
                    final List<MatchedSamples> matches) {
                // no action needed
            }

            @Override
            public void onFundamentalMatrixEstimated(
                    final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor,
                    final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                // no action needed
            }

            @Override
            public void onCamerasEstimated(
                    final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor, final int viewId1,
                    final int viewId2, final EstimatedCamera camera1, final EstimatedCamera camera2) {
                // no action needed
            }

            @Override
            public void onReconstructedPointsEstimated(
                    final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor,
                    final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                // no action needed
            }

            @Override
            public void onStart(final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onFinish(final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onCancel(final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor) {
                // no action needed
            }

            @Override
            public void onFail(final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor) {
                // no action needed
            }
        };

        // constructor with listener
        var reconstructor = new AbsoluteOrientationSlamTwoViewsSparseReconstructor(listener);

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
        reconstructor = new AbsoluteOrientationSlamTwoViewsSparseReconstructor(configuration, listener);

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
    void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithoutNoise()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException, RotationException,
            NotReadyException, NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var configuration = new AbsoluteOrientationSlamTwoViewsSparseReconstructorConfiguration();
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
            assertSame(configuration, configuration.setCalibrationData(calibrationData));

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
            final var axisDiffRotation = new AxisRotation3D(rotation1.inverseRotationAndReturnNew().combineAndReturnNew(
                    rotation2));

            final var axisX = axisDiffRotation.getAxisX();
            final var axisY = axisDiffRotation.getAxisY();
            final var axisZ = axisDiffRotation.getAxisZ();
            final var angle = axisDiffRotation.getRotationAngle();

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
            Point3D center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);
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

            if (maxTriesReached) {
                continue;
            }

            final var listener = new AbsoluteOrientationSlamTwoViewsSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor,
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
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor,
                        final PinholeCamera camera) {
                    slamCameraEstimated++;
                    slamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor, final int viewId,
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

                        // assume the following accelerator and gyroscope samples
                        // are obtained during a period of 1 second between 1st
                        // and 2nd view (50 samples * 0.02 s/sample = 1 second)
                        var timestamp = 0;
                        final var orientation = new Quaternion(rotation1);
                        for (var s = 0; s < N_SENSOR_SAMPLES; s++) {
                            reconstructor.updateAccelerometerSample(timestamp, (float) accelerationX,
                                    (float) accelerationY, (float) accelerationZ);
                            reconstructor.updateGyroscopeSample(timestamp, (float) angularSpeedX, (float) angularSpeedY,
                                    (float) angularSpeedZ);
                            reconstructor.updateOrientationSample(timestamp, orientation);
                            // update orientation
                            orientation.combine(diffQuaternion);
                            timestamp += DELTA_NANOS;
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
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> samples) {
                    // no action needed
                }

                @Override
                public void onRequestMatches(
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor,
                        final List<Sample2D> samples1, final List<Sample2D> samples2, final int viewId1,
                        final int viewId2, final List<MatchedSamples> matches) {
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
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    AbsoluteOrientationSlamTwoViewsSparseReconstructorTest.this.estimatedFundamentalMatrix =
                            estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor,
                        final int viewId1, final int viewId2, final EstimatedCamera camera1,
                        final EstimatedCamera camera2) {
                    estimatedCamera1 = camera1;
                    estimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    reconstructedPoints = points;
                }

                @Override
                public void onStart(final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new AbsoluteOrientationSlamTwoViewsSparseReconstructor(configuration, listener);

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
            final var maxBaseline = Math.max(estimatedBaseline, baseline);
            final var minBaseline = Math.min(estimatedBaseline, baseline);
            final var absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(estimatedCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(estimatedCenter2, absoluteScaleError));

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

            // check that points are correct (after scale correction)
            final var scale = baseline / estimatedBaseline;
            final var scaleTransformation = new MetricTransformation3D(scale);

            var validPoints = true;
            for (var i = 0; i < numPoints; i++) {
                final var rescaledPoint = Point3D.create();
                scaleTransformation.transform(reconstructedPoints3D.get(i), rescaledPoint);
                if (!points3D.get(i).equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            final var scaleRelativeError = 1.0 - minBaseline / maxBaseline;
            LOGGER.log(Level.INFO, "Baseline relative error without noise: {0,number,0.000%}", scaleRelativeError);

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
    void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithNoise() throws InvalidPairOfCamerasException,
            AlgebraException, CameraException, RotationException, NotReadyException, NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var offsetRandomizer = new UniformRandomizer();
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var configuration = new AbsoluteOrientationSlamTwoViewsSparseReconstructorConfiguration();
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
            assertSame(configuration, configuration.setCalibrationData(calibrationData));

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
            final var axisDiffRotation = new AxisRotation3D(rotation1.inverseRotationAndReturnNew().combineAndReturnNew(
                    rotation2));

            final var axisX = axisDiffRotation.getAxisX();
            final var axisY = axisDiffRotation.getAxisY();
            final var axisZ = axisDiffRotation.getAxisZ();
            final var angle = axisDiffRotation.getRotationAngle();

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

            if (maxTriesReached) {
                continue;
            }

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            final var listener = new AbsoluteOrientationSlamTwoViewsSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor,
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
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor,
                        final PinholeCamera camera) {
                    slamCameraEstimated++;
                    slamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor) {
                    return viewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor, final int viewId,
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

                        // assume the following accelerator and gyroscope samples
                        // are obtained during a period of 1 second between 1st
                        // and 2nd view (50 samples * 0.02 s/sample = 1 second)
                        var timestamp = 0L;
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
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> samples) {
                    viewCount++;
                }

                @Override
                public void onSamplesRejected(
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor, final int viewId,
                        final List<Sample2D> samples) {
                    // no action needed
                }

                @Override
                public void onRequestMatches(
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor,
                        final List<Sample2D> samples1, final List<Sample2D> samples2, final int viewId1,
                        final int viewId2, final List<MatchedSamples> matches) {
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
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor,
                        final EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    AbsoluteOrientationSlamTwoViewsSparseReconstructorTest.this.estimatedFundamentalMatrix =
                            estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor, final int viewId1,
                        final int viewId2, final EstimatedCamera camera1, final EstimatedCamera camera2) {
                    estimatedCamera1 = camera1;
                    estimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor,
                        final List<MatchedSamples> matches, final List<ReconstructedPoint3D> points) {
                    reconstructedPoints = points;
                }

                @Override
                public void onStart(final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor) {
                    started = true;
                }

                @Override
                public void onFinish(final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor) {
                    finished = true;
                }

                @Override
                public void onCancel(final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor) {
                    cancelled = true;
                }

                @Override
                public void onFail(final AbsoluteOrientationSlamTwoViewsSparseReconstructor reconstructor) {
                    failed = true;
                }
            };

            final var reconstructor = new AbsoluteOrientationSlamTwoViewsSparseReconstructor(configuration, listener);

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
            final var maxBaseline = Math.max(estimatedBaseline, baseline);
            final var minBaseline = Math.min(estimatedBaseline, baseline);
            final var absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);

            assertTrue(center1.equals(estimatedCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(estimatedCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(estimatedCenter2, absoluteScaleError));

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

            // check that points are correct (after scale correction)
            final var scale = baseline / estimatedBaseline;
            final var scaleTransformation = new MetricTransformation3D(scale);

            var validPoints = true;
            for (var i = 0; i < numPoints; i++) {
                final var rescaledPoint = Point3D.create();
                scaleTransformation.transform(reconstructedPoints3D.get(i), rescaledPoint);
                if (!points3D.get(i).equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            final var scaleRelativeError = 1.0 - minBaseline / maxBaseline;
            LOGGER.log(Level.INFO, "Baseline relative error with noise: {0,number,0.000%}", scaleRelativeError);

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
