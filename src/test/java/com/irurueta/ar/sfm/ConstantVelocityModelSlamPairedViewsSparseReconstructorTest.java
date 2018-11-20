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
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class ConstantVelocityModelSlamPairedViewsSparseReconstructorTest {

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

    //5% of relative error in scale estimation
    private static final double RELATIVE_ERROR = 0.1;

    private static final int MAX_CALIBRATION_SAMPLES = 10000;

    //conversion from milliseconds to nanoseconds
    private static final int MILLIS_TO_NANOS = 1000000;

    //time between samples expressed in nanoseconds (a typical sensor in Android
    //delivers a sample every 20ms)
    private static final int DELTA_NANOS = 20000000; //0.02 seconds

    private static final float MIN_CALIBRATION_OFFSET = -1e-4f;
    private static final float MAX_CALIBRATION_OFFSET = 1e-4f;

    private static final double ACCELERATION_NOISE_STANDARD_DEVIATION = 1e-4;
    private static final double ANGULAR_SPEED_NOISE_STANDARD_DEVIATION = 1e-4;

    private static final int N_SENSOR_SAMPLES = 50;

    private static final Logger LOGGER = Logger.getLogger(
            ConstantVelocityModelSlamPairedViewsSparseReconstructorTest.class.getSimpleName());

    private int mViewCount = 0;
    private EstimatedFundamentalMatrix mEstimatedFundamentalMatrix;
    private EstimatedFundamentalMatrix mEstimatedFundamentalMatrix2;
    private EstimatedFundamentalMatrix mEstimatedFundamentalMatrix3;
    private EstimatedCamera mEstimatedEuclideanCamera1;
    private EstimatedCamera mEstimatedEuclideanCamera2;
    private EstimatedCamera mEstimatedEuclideanCamera2b;
    private EstimatedCamera mEstimatedEuclideanCamera3;
    private EstimatedCamera mEstimatedEuclideanCamera3b;
    private EstimatedCamera mEstimatedEuclideanCamera4;
    private List<ReconstructedPoint3D> mEuclideanReconstructedPoints;
    private List<ReconstructedPoint3D> mEuclideanReconstructedPoints2;
    private List<ReconstructedPoint3D> mEuclideanReconstructedPoints3;

    private double mScale;
    private double mScale2;
    private double mScale3;

    private boolean mStarted;
    private boolean mFinished;
    private boolean mFailed;
    private boolean mCancelled;

    private long mTimestamp;

    private int mSlamDataAvailable;
    private int mSlamCameraEstimated;

    private PinholeCamera mSlamCamera;
    private Matrix mSlamCovariance;

    public ConstantVelocityModelSlamPairedViewsSparseReconstructorTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() {
        mViewCount = 0;
        mEstimatedFundamentalMatrix = mEstimatedFundamentalMatrix2 =
                mEstimatedFundamentalMatrix3 = null;
        mEstimatedEuclideanCamera1 = mEstimatedEuclideanCamera2 =
                mEstimatedEuclideanCamera3 = mEstimatedEuclideanCamera4 = null;
        mEuclideanReconstructedPoints = null;
        mStarted = mFinished = mFailed = mCancelled = false;
        mTimestamp = 0;
        mSlamDataAvailable = 0;
        mSlamCameraEstimated = 0;
        mSlamCamera = null;
        mSlamCovariance = null;
    }

    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        assertEquals(
                ConstantVelocityModelSlamPairedViewsSparseReconstructor.MIN_NUMBER_OF_VIEWS,
                2);

        ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration configuration =
                new ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();
        ConstantVelocityModelSlamPairedViewsSparseReconstructorListener listener =
                new ConstantVelocityModelSlamPairedViewsSparseReconstructorListener() {
                    @Override
                    public void onSlamDataAvailable(
                            ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, double positionX,
                            double positionY, double positionZ, double velocityX, double velocityY, double velocityZ,
                            double accelerationX, double accelerationY, double accelerationZ, double quaternionA,
                            double quaternionB, double quaternionC, double quaternionD, double angularSpeedX,
                            double angularSpeedY, double angularSpeedZ, Matrix covariance) { }

                    @Override
                    public void onSlamCameraEstimated(
                            ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                            PinholeCamera camera) { }

                    @Override
                    public boolean hasMoreViewsAvailable(
                            ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                        return false;
                    }

                    @Override
                    public void onRequestSamplesForCurrentViewPair(
                            ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                            int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) { }

                    @Override
                    public void onSamplesAccepted(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                            int viewId1, int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) { }

                    @Override
                    public void onSamplesRejected(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                            int viewId1, int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) { }

                    @Override
                    public void onRequestMatches(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                            int viewId1, int viewId2, List<Sample2D> samples1, List<Sample2D> samples2,
                            List<MatchedSamples> matches) { }

                    @Override
                    public void onFundamentalMatrixEstimated(
                            ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                            int viewId2, EstimatedFundamentalMatrix estimatedFundamentalMatrix) { }

                    @Override
                    public void onEuclideanCameraPairEstimated(
                            ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                            int viewId2, double scale, EstimatedCamera camera1, EstimatedCamera camera2) { }

                    @Override
                    public void onEuclideanReconstructedPointsEstimated(
                            ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                            int viewId2, double scale, List<ReconstructedPoint3D> points) { }

                    @Override
                    public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                            ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId) {
                        return null;
                    }

                    @Override
                    public void onStart(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) { }

                    @Override
                    public void onFinish(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) { }

                    @Override
                    public void onCancel(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) { }

                    @Override
                    public void onFail(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) { }
                };

        ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor =
                new ConstantVelocityModelSlamPairedViewsSparseReconstructor(listener);

        //check default values
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

        //constructor with configuration and listener
        reconstructor = new ConstantVelocityModelSlamPairedViewsSparseReconstructor(configuration, listener);

        //check default values
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
    public void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithoutNoiseTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException, RotationException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration configuration =
                    new ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

            float accelerationOffsetX = 0.0f;
            float accelerationOffsetY = 0.0f;
            float accelerationOffsetZ = 0.0f;

            float angularOffsetX = 0.0f;
            float angularOffsetY = 0.0f;
            float angularOffsetZ = 0.0f;

            ConstantVelocityModelSlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY,
                    angularOffsetZ, noiseRandomizer);
            ConstantVelocityModelSlamCalibrationData calibrationData
                    = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getPairedCamerasAspectRatio();
            double skewness = 0.0;
            double principalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);


            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            AxisRotation3D axisRotation2 = new AxisRotation3D(rotation2);

            double axisX = axisRotation2.getAxisX();
            double axisY = axisRotation2.getAxisY();
            double axisZ = axisRotation2.getAxisZ();
            double angle = axisRotation2.getRotationAngle();

            AxisRotation3D diffRotation = new AxisRotation3D(axisX, axisY,
                    axisZ, angle / N_SENSOR_SAMPLES);
            Quaternion diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
            double[] angularSpeeds = diffQuaternion.toEulerAngles();
            final double angularSpeedX = angularSpeeds[0];
            final double angularSpeedY = angularSpeeds[1];
            final double angularSpeedZ = angularSpeeds[2];
            Quaternion diffRotation2 = new Quaternion(angularSpeedX,
                    angularSpeedY, angularSpeedZ);

            //number of samples (50 samples * 0.02 s/sample = 1 second)
            MatrixRotation3D rotation2b = new MatrixRotation3D(rotation1);
            MatrixRotation3D rotation2c = new MatrixRotation3D(rotation1);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation2b.combine(diffRotation);
                rotation2c.combine(diffRotation2);
            }

            //check that rotations created by composing sensor samples are
            //equal to the original one
            assertTrue(rotation2.equals(rotation2b, ABSOLUTE_ERROR));
            assertTrue(rotation2.equals(rotation2c, ABSOLUTE_ERROR));

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            double baseline = center1.distanceTo(center2);

            final double accelerationX, accelerationY, accelerationZ;

            //s = 0.5*a*t^2 --> a = 2*s/t^2
            //assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ
                    = 2 * cameraSeparation;

            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);

            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);

            //create 3D points laying in front of both cameras

            //1st find an approximate central point by intersecting the axis
            //planes of both cameras
            Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            Matrix planesIntersectionMatrix = new Matrix(
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

            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            Matrix v = decomposer.getV();
            HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX, lambdaY, lambdaZ;

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3D =
                    new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1, front2;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
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
                } while(!front1 || !front2);
                points3D.add(point3D);

                //check that 3D point is in front of both cameras
                //noinspection all
                assertTrue(front1);
                //noinspection all
                assertTrue(front2);

                //project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            ConstantVelocityModelSlamPairedViewsSparseReconstructorListener listener =
                    new ConstantVelocityModelSlamPairedViewsSparseReconstructorListener() {
                        @Override
                        public void onSlamDataAvailable(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, double positionX,
                                double positionY, double positionZ, double velocityX, double velocityY,
                                double velocityZ, double accelerationX, double accelerationY, double accelerationZ,
                                double quaternionA, double quaternionB, double quaternionC, double quaternionD,
                                double angularSpeedX, double angularSpeedY, double angularSpeedZ, Matrix covariance) {
                            mSlamDataAvailable++;
                            mSlamCovariance = covariance;
                        }

                        @Override
                        public void onSlamCameraEstimated(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                                PinholeCamera camera) {
                            mSlamCameraEstimated++;
                            mSlamCamera = camera;
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            Sample2D sample1, sample2;
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

                            //assume the following accelerator and gyroscope samples
                            //are obtained during a period of 1 second between 1st
                            //and 2nd view (50 samples * 0.02 s/sample = 1 second)
                            mTimestamp = 0;
                            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                reconstructor.updateAccelerometerSample(mTimestamp,
                                        (float) accelerationX, (float) accelerationY,
                                        (float) accelerationZ);
                                reconstructor.updateGyroscopeSample(mTimestamp,
                                        (float) angularSpeedX, (float) angularSpeedY,
                                        (float) angularSpeedZ);
                                mTimestamp += DELTA_NANOS;
                            }
                        }

                        @Override
                        public void onSamplesAccepted(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2,
                                List<MatchedSamples> matches) {

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
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, double scale, EstimatedCamera camera1, EstimatedCamera camera2) {
                            mEstimatedEuclideanCamera1 = camera1;
                            mEstimatedEuclideanCamera2 = camera2;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, double scale, List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor =
                    new ConstantVelocityModelSlamPairedViewsSparseReconstructor(configuration, listener);

            //check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            //check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertTrue(mSlamDataAvailable > 0);
            assertTrue(mSlamCameraEstimated > 0);
            assertNotNull(mSlamCamera);
            assertNotNull(mSlamCovariance);
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

            //check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            //matrices are equal up to scale
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

            PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();

            List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                Point3D p = euclideanReconstructedPoints3D.get(i);
                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(p));
            }

            Point3D euclideanCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            Point3D euclideanCenter2 = estimatedEuclideanCamera2.getCameraCenter();

            PinholeCameraIntrinsicParameters euclideanIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();

            Rotation3D euclideanRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            Rotation3D euclideanRotation2 = estimatedEuclideanCamera2.getCameraRotation();

            //check scale
            double estimatedBaseline = euclideanCenter1.distanceTo(euclideanCenter2);

            //check cameras are correct
            double maxBaseline = Math.max(estimatedBaseline, baseline);
            double absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);
            assertEquals(mScale, estimatedBaseline, 5*LARGE_ABSOLUTE_ERROR);

            //check cameras
            assertTrue(center1.equals(euclideanCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(euclideanCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2, absoluteScaleError));

            assertEquals(euclideanIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(euclideanRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            //check that points are correct (after scale correction)

            //check that scale error is less than 5%
            assertTrue(Math.abs(baseline / mScale - 1.0) < RELATIVE_ERROR);

            MetricTransformation3D scaleTransformation
                    = new MetricTransformation3D(baseline / mScale);

            int numValidPoints = 0;
            double scaleX, scaleY, scaleZ;
            for (int i = 0; i < numPoints; i++) {
                Point3D point = points3D.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3D.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(euclideanPoint, rescaledPoint);

                //euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                //check that scale error is less than 5%
                if (Math.abs(scaleX - 1.0) > LARGE_ABSOLUTE_ERROR ||
                        Math.abs(scaleY - 1.0) > LARGE_ABSOLUTE_ERROR ||
                        Math.abs(scaleZ - 1.0) > LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, 1.0, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, 1.0, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, 1.0, LARGE_ABSOLUTE_ERROR);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                assertEquals(scaleX, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            if (numValidPoints == 0) {
                continue;
            }

            double scaleRelativeError = Math.abs(baseline / mScale - 1.0);
            LOGGER.log(Level.INFO,
                    "Baseline relative error without noise: {0,number,0.000%}",
                    scaleRelativeError);

            numValid++;

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithNoiseTwoViews()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException, RotationException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer offsetRandomizer = new UniformRandomizer(
                    new Random());
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration configuration =
                    new ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

            float accelerationOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            float angularOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            ConstantVelocityModelSlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY,
                    angularOffsetZ, noiseRandomizer);
            ConstantVelocityModelSlamCalibrationData calibrationData
                    = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getPairedCamerasAspectRatio();
            double skewness = 0.0;
            double principalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);


            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            AxisRotation3D axisRotation2 = new AxisRotation3D(rotation2);

            double axisX = axisRotation2.getAxisX();
            double axisY = axisRotation2.getAxisY();
            double axisZ = axisRotation2.getAxisZ();
            double angle = axisRotation2.getRotationAngle();

            AxisRotation3D diffRotation = new AxisRotation3D(axisX, axisY,
                    axisZ, angle / N_SENSOR_SAMPLES);
            Quaternion diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
            double[] angularSpeeds = diffQuaternion.toEulerAngles();
            final double angularSpeedX = angularSpeeds[0];
            final double angularSpeedY = angularSpeeds[1];
            final double angularSpeedZ = angularSpeeds[2];
            Quaternion diffRotation2 = new Quaternion(angularSpeedX,
                    angularSpeedY, angularSpeedZ);

            //number of samples (50 samples * 0.02 s/sample = 1 second)
            MatrixRotation3D rotation2b = new MatrixRotation3D(rotation1);
            MatrixRotation3D rotation2c = new MatrixRotation3D(rotation1);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation2b.combine(diffRotation);
                rotation2c.combine(diffRotation2);
            }

            //check that rotations created by composing sensor samples are
            //equal to the original one
            assertTrue(rotation2.equals(rotation2b, ABSOLUTE_ERROR));
            assertTrue(rotation2.equals(rotation2c, ABSOLUTE_ERROR));

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            double baseline = center1.distanceTo(center2);

            final double accelerationX, accelerationY, accelerationZ;

            //s = 0.5*a*t^2 --> a = 2*s/t^2
            //assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ
                    = 2 * cameraSeparation;

            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);

            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);

            //create 3D points laying in front of both cameras

            //1st find an approximate central point by intersecting the axis
            //planes of both cameras
            Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            Matrix planesIntersectionMatrix = new Matrix(
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

            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    planesIntersectionMatrix);
            decomposer.decompose();
            Matrix v = decomposer.getV();
            HomogeneousPoint3D centralCommonPoint = new HomogeneousPoint3D(
                    v.getElementAt(0, 3),
                    v.getElementAt(1, 3),
                    v.getElementAt(2, 3),
                    v.getElementAt(3, 3));

            double lambdaX, lambdaY, lambdaZ;

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS,
                    MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3D =
                    new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1, front2;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
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
                } while(!front1 || !front2);
                points3D.add(point3D);

                //check that 3D point is in front of both cameras
                //noinspection all
                assertTrue(front1);
                //noinspection all
                assertTrue(front2);

                //project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            ConstantVelocityModelSlamPairedViewsSparseReconstructorListener listener =
                    new ConstantVelocityModelSlamPairedViewsSparseReconstructorListener() {
                        @Override
                        public void onSlamDataAvailable(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, double positionX,
                                double positionY, double positionZ, double velocityX, double velocityY,
                                double velocityZ, double accelerationX, double accelerationY, double accelerationZ,
                                double quaternionA, double quaternionB, double quaternionC, double quaternionD,
                                double angularSpeedX, double angularSpeedY, double angularSpeedZ, Matrix covariance) {
                            mSlamDataAvailable++;
                            mSlamCovariance = covariance;
                        }

                        @Override
                        public void onSlamCameraEstimated(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                                PinholeCamera camera) {
                            mSlamCameraEstimated++;
                            mSlamCamera = camera;
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            return mViewCount < 2;
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            Sample2D sample1, sample2;
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

                            //assume the following accelerator and gyroscope samples
                            //are obtained during a period of 1 second between 1st
                            //and 2nd view (50 samples * 0.02 s/sample = 1 second)
                            mTimestamp = 0;
                            float noiseAccelerationX, noiseAccelerationY,
                                    noiseAccelerationZ;
                            float noiseAngularSpeedX, noiseAngularSpeedY,
                                    noiseAngularSpeedZ;

                            float accelerationWithNoiseX, accelerationWithNoiseY,
                                    accelerationWithNoiseZ;
                            float angularSpeedWithNoiseX, angularSpeedWithNoiseY,
                                    angularSpeedWithNoiseZ;

                            float[] accelerationWithNoise = new float[3];
                            float[] angularSpeedWithNoise = new float[3];
                            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                noiseAccelerationX =
                                        accelerationRandomizer.nextFloat();
                                noiseAccelerationY =
                                        accelerationRandomizer.nextFloat();
                                noiseAccelerationZ =
                                        accelerationRandomizer.nextFloat();

                                noiseAngularSpeedX =
                                        angularSpeedRandomizer.nextFloat();
                                noiseAngularSpeedY =
                                        angularSpeedRandomizer.nextFloat();
                                noiseAngularSpeedZ =
                                        angularSpeedRandomizer.nextFloat();

                                accelerationWithNoiseX = (float)accelerationX +
                                        noiseAccelerationX;
                                accelerationWithNoiseY = (float)accelerationY +
                                        noiseAccelerationY;
                                accelerationWithNoiseZ = (float)accelerationZ +
                                        noiseAccelerationZ;
                                accelerationWithNoise[0] = accelerationWithNoiseX;
                                accelerationWithNoise[1] = accelerationWithNoiseY;
                                accelerationWithNoise[2] = accelerationWithNoiseZ;

                                angularSpeedWithNoiseX = (float)angularSpeedX +
                                        noiseAngularSpeedX;
                                angularSpeedWithNoiseY = (float)angularSpeedY +
                                        noiseAngularSpeedY;
                                angularSpeedWithNoiseZ = (float)angularSpeedZ +
                                        noiseAngularSpeedZ;
                                angularSpeedWithNoise[0] = angularSpeedWithNoiseX;
                                angularSpeedWithNoise[1] = angularSpeedWithNoiseY;
                                angularSpeedWithNoise[2] = angularSpeedWithNoiseZ;

                                reconstructor.updateAccelerometerSample(mTimestamp,
                                        accelerationWithNoise);
                                reconstructor.updateGyroscopeSample(mTimestamp,
                                        angularSpeedWithNoise);
                                mTimestamp += DELTA_NANOS;
                            }
                        }

                        @Override
                        public void onSamplesAccepted(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2,
                                List<MatchedSamples> matches) {

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
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, double scale, EstimatedCamera camera1, EstimatedCamera camera2) {
                            mEstimatedEuclideanCamera1 = camera1;
                            mEstimatedEuclideanCamera2 = camera2;
                            mScale = scale;
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, double scale, List<ReconstructedPoint3D> points) {
                            mEuclideanReconstructedPoints = points;
                            mScale = scale;
                        }

                        @Override
                        public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor =
                    new ConstantVelocityModelSlamPairedViewsSparseReconstructor(configuration, listener);

            //check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            //check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertTrue(mSlamDataAvailable > 0);
            assertTrue(mSlamCameraEstimated > 0);
            assertNotNull(mSlamCamera);
            assertNotNull(mSlamCovariance);
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

            //check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();

            //matrices are equal up to scale
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

            PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();

            List<Point3D> euclideanReconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                euclideanReconstructedPoints3D.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                Point3D p = euclideanReconstructedPoints3D.get(i);
                assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(p));
            }

            Point3D euclideanCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            Point3D euclideanCenter2 = estimatedEuclideanCamera2.getCameraCenter();

            PinholeCameraIntrinsicParameters euclideanIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();

            Rotation3D euclideanRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            Rotation3D euclideanRotation2 = estimatedEuclideanCamera2.getCameraRotation();

            //check scale
            double estimatedBaseline = euclideanCenter1.distanceTo(euclideanCenter2);

            //check cameras are correct
            double maxBaseline = Math.max(estimatedBaseline, baseline);
            double absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(estimatedBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(estimatedBaseline, baseline, absoluteScaleError);
            assertEquals(mScale, estimatedBaseline, 2*LARGE_ABSOLUTE_ERROR);

            //check cameras
            assertTrue(center1.equals(euclideanCenter1, ABSOLUTE_ERROR));
            if (!center2.equals(euclideanCenter2, absoluteScaleError)) {
                continue;
            }
            assertTrue(center2.equals(euclideanCenter2, absoluteScaleError));

            assertEquals(euclideanIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(euclideanRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            //check that points are correct (after scale correction)

            //check that scale error is less than 5%
            assertTrue(Math.abs(baseline / mScale - 1.0) < RELATIVE_ERROR);

            MetricTransformation3D scaleTransformation
                    = new MetricTransformation3D(baseline / mScale);

            int numValidPoints = 0;
            double scaleX, scaleY, scaleZ;
            for (int i = 0; i < numPoints; i++) {
                Point3D point = points3D.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3D.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(euclideanPoint, rescaledPoint);

                //euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                scaleX = point.getInhomX() / rescaledPoint.getInhomX();
                scaleY = point.getInhomY() / rescaledPoint.getInhomY();
                scaleZ = point.getInhomZ() / rescaledPoint.getInhomZ();

                //check that scale error is less than 5%
                if (Math.abs(scaleX - 1.0) > LARGE_ABSOLUTE_ERROR ||
                        Math.abs(scaleY - 1.0) > LARGE_ABSOLUTE_ERROR ||
                        Math.abs(scaleZ - 1.0) > LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, 1.0, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, 1.0, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, 1.0, LARGE_ABSOLUTE_ERROR);
                if (point.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidPoints++;
                }

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                assertEquals(scaleX, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);
            }

            if (numValidPoints == 0) {
                continue;
            }

            double scaleRelativeError = Math.abs(baseline / mScale - 1.0);
            LOGGER.log(Level.INFO,
                    "Baseline relative error without noise: {0,number,0.000%}",
                    scaleRelativeError);

            numValid++;

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithoutNoiseThreeViews()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException, RotationException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration configuration =
                    new ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

            float accelerationOffsetX = 0.0f;
            float accelerationOffsetY = 0.0f;
            float accelerationOffsetZ = 0.0f;

            float angularOffsetX = 0.0f;
            float angularOffsetY = 0.0f;
            float angularOffsetZ = 0.0f;

            ConstantVelocityModelSlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY,
                    angularOffsetZ, noiseRandomizer);
            ConstantVelocityModelSlamCalibrationData calibrationData
                    = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getPairedCamerasAspectRatio();
            double skewness = 0.0;
            double principalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);


            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double alphaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            AxisRotation3D axisRotation2 = new AxisRotation3D(rotation2);
            MatrixRotation3D rotation3 = new MatrixRotation3D(alphaEuler3,
                    betaEuler3, gammaEuler3);

            double axis2X = axisRotation2.getAxisX();
            double axis2Y = axisRotation2.getAxisY();
            double axis2Z = axisRotation2.getAxisZ();
            double angle2 = axisRotation2.getRotationAngle();


            AxisRotation3D diffRotation = new AxisRotation3D(axis2X, axis2Y,
                    axis2Z, angle2 / N_SENSOR_SAMPLES);
            Quaternion diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
            double[] angularSpeeds = diffQuaternion.toEulerAngles();
            final double angularSpeed2X = angularSpeeds[0];
            final double angularSpeed2Y = angularSpeeds[1];
            final double angularSpeed2Z = angularSpeeds[2];
            Quaternion diffRotation2 = new Quaternion(angularSpeed2X,
                    angularSpeed2Y, angularSpeed2Z);

            //number of samples (50 samples * 0.02 s/sample = 1 second)
            MatrixRotation3D rotation2b = new MatrixRotation3D(rotation1);
            MatrixRotation3D rotation2c = new MatrixRotation3D(rotation1);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation2b.combine(diffRotation);
                rotation2c.combine(diffRotation2);
            }

            //check that rotations created by composing sensor samples are
            //equal to the original one
            assertTrue(rotation2.equals(rotation2b, ABSOLUTE_ERROR));
            assertTrue(rotation2.equals(rotation2c, ABSOLUTE_ERROR));

            AxisRotation3D accumDiffRotation = rotation2.inverseRotationAndReturnNew().
                    combineAndReturnNew(rotation3).toAxisRotation();
            double axis3X = accumDiffRotation.getAxisX();
            double axis3Y = accumDiffRotation.getAxisY();
            double axis3Z = accumDiffRotation.getAxisZ();
            double angle3 = accumDiffRotation.getRotationAngle();

            diffRotation = new AxisRotation3D(axis3X, axis3Y, axis3Z, angle3 / N_SENSOR_SAMPLES);
            diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
            angularSpeeds = diffQuaternion.toEulerAngles();
            final double angularSpeed3X = angularSpeeds[0];
            final double angularSpeed3Y = angularSpeeds[1];
            final double angularSpeed3Z = angularSpeeds[2];
            diffRotation2 = new Quaternion(angularSpeed3X, angularSpeed3Y,
                    angularSpeed3Z);

            //number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            //previously sampled rotation
            MatrixRotation3D rotation3b = new MatrixRotation3D(rotation2b);
            MatrixRotation3D rotation3c = new MatrixRotation3D(rotation2c);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation3b.combine(diffRotation);
                rotation3c.combine(diffRotation2);
            }

            //check that rotations created by composing sensor samples are equal
            //to the original one
            assertTrue(rotation3.equals(rotation3b, ABSOLUTE_ERROR));
            assertTrue(rotation3.equals(rotation3c, ABSOLUTE_ERROR));

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            double cameraSeparation2 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);


            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            Point3D center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);

            double baseline = center1.distanceTo(center2);
            double baseline2 = center2.distanceTo(center3);

            final double accelerationX, accelerationY, accelerationZ;
            final double accelerationX2, accelerationY2, accelerationZ2;

            //s = 0.5*a*t^2 --> a = 2*s/t^2
            //assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ
                    = 2 * cameraSeparation;
            accelerationX2 = accelerationY2 = accelerationZ2
                    = 2 * cameraSeparation2;

            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
            PinholeCamera camera3 = new PinholeCamera(intrinsic, rotation3,
                    center3);

            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);
            FundamentalMatrix fundamentalMatrix2 = new FundamentalMatrix(
                    camera2, camera3);

            //create 3D points laying in front of all cameras

            //1st find an approximate central point by intersecting the axis planes of
            //all cameras
            Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            Plane horizontalPlane3 = camera3.getHorizontalAxisPlane();
            Plane verticalPlane3 = camera3.getVerticalAxisPlane();
            Matrix planesIntersectionMatrixPair1 = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            Matrix planesIntersectionMatrixPair2 = new Matrix(
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

            SingularValueDecomposer decomposerPair1 = new SingularValueDecomposer(
                    planesIntersectionMatrixPair1);
            decomposerPair1.decompose();
            Matrix vPair1 = decomposerPair1.getV();

            SingularValueDecomposer decomposerPair2 = new SingularValueDecomposer(
                    planesIntersectionMatrixPair2);
            decomposerPair2.decompose();
            Matrix vPair2 = decomposerPair2.getV();

            HomogeneousPoint3D centralCommonPointPair1 = new HomogeneousPoint3D(
                    vPair1.getElementAt(0, 3),
                    vPair1.getElementAt(1, 3),
                    vPair1.getElementAt(2, 3),
                    vPair1.getElementAt(3, 3));

            HomogeneousPoint3D centralCommonPointPair2 = new HomogeneousPoint3D(
                    vPair2.getElementAt(0, 3),
                    vPair2.getElementAt(1, 3),
                    vPair2.getElementAt(2, 3),
                    vPair2.getElementAt(3, 3));

            double lambdaX, lambdaY, lambdaZ;

            final int numPointsPair1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final int numPointsPair2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3DPair1 = new ArrayList<>();
            List<InhomogeneousPoint3D> points3DPair2 = new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2, projectedPoint3;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2a = new ArrayList<>();
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            final List<Point2D> projectedPoints3 = new ArrayList<>();
            boolean front1, front2, front3;
            for (int i = 0; i < numPointsPair1; i++) {
                //generate points and ensure they lie in front of both cameras
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

                //check that 3D point is in front of 1st pair of cameras
                //noinspection all
                assertTrue(front1);
                //noinspection all
                assertTrue(front2);
                //noinspection all
                assertTrue(front3);

                points3DPair1.add(point3D);

                //project 3D point into 1st pair of cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2a.add(projectedPoint2);
            }

            for (int i = 0; i < numPointsPair2; i++) {
                //generate points and ensure they lie in front of both cameras
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

                //check that 3D point is in front of 2nd pair of cameras
                //noinspection all
                assertTrue(front1);
                //noinspection all
                assertTrue(front2);
                //noinspection all
                assertTrue(front3);

                points3DPair2.add(point3D);

                //project 3D point into 2nd pair of cameras
                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2b.add(projectedPoint2);

                projectedPoint3 = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3);
                projectedPoints3.add(projectedPoint3);
            }

            ConstantVelocityModelSlamPairedViewsSparseReconstructorListener listener =
                    new ConstantVelocityModelSlamPairedViewsSparseReconstructorListener() {
                        @Override
                        public void onSlamDataAvailable(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, double positionX,
                                double positionY, double positionZ, double velocityX, double velocityY,
                                double velocityZ, double accelerationX, double accelerationY, double accelerationZ,
                                double quaternionA, double quaternionB, double quaternionC, double quaternionD,
                                double angularSpeedX, double angularSpeedY, double angularSpeedZ, Matrix covariance) {
                            mSlamDataAvailable++;
                            mSlamCovariance = covariance;
                        }

                        @Override
                        public void onSlamCameraEstimated(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                                PinholeCamera camera) {
                            mSlamCameraEstimated++;
                            mSlamCamera = camera;
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            return mViewCount < 4; //3 views = 2 view pairs (2 images * 2 views --> 4 view counts)
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            int viewCount = reconstructor.getViewCount();

                            Sample2D sample1, sample2;
                            if(viewCount == 0) {
                                //first view pair
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

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 1st
                                //and 2nd view (50 samples * 0.02 s/sample = 1 second)
                                mTimestamp = 0;
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            (float) accelerationX, (float) accelerationY,
                                            (float) accelerationZ);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            (float) angularSpeed2X, (float) angularSpeed2Y,
                                            (float) angularSpeed2Z);
                                    mTimestamp += DELTA_NANOS;
                                }

                            } else if (viewCount == 2){
                                //second view pair
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

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 2nd
                                //and 3rd view (50 samples * 0.02 s/sample = 1 second)
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            (float) accelerationX2, (float) accelerationY2,
                                            (float) accelerationZ2);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            (float) angularSpeed3X, (float) angularSpeed3Y,
                                            (float) angularSpeed3Z);
                                    mTimestamp += DELTA_NANOS;
                                }
                            }
                        }

                        @Override
                        public void onSamplesAccepted(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2,
                                List<MatchedSamples> matches) {
                            matches.clear();

                            int viewCount = reconstructor.getViewCount();
                            int numPoints;
                            if (viewCount == 0) {
                                //first view pair
                                numPoints = numPointsPair1;
                            } else {
                                //second view pair
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
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            int viewCount = reconstructor.getViewCount();
                            if (viewCount == 0) {
                                mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                            } else if (viewCount == 2) {
                                mEstimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                            }
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, double scale, EstimatedCamera camera1, EstimatedCamera camera2) {

                            int viewCount = reconstructor.getViewCount();
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
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, double scale, List<ReconstructedPoint3D> points) {

                            int viewCount = reconstructor.getViewCount();
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
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor =
                    new ConstantVelocityModelSlamPairedViewsSparseReconstructor(configuration, listener);

            //check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            //check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertTrue(mSlamDataAvailable > 0);
            assertTrue(mSlamCameraEstimated > 0);
            assertNotNull(mSlamCamera);
            assertNotNull(mSlamCovariance);
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

            //check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            fundamentalMatrix2.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();
            mEstimatedFundamentalMatrix2.getFundamentalMatrix().normalize();

            //matrices are equal up to scale
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

            PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();
            PinholeCamera estimatedEuclideanCamera2b = mEstimatedEuclideanCamera2b.getCamera();
            PinholeCamera estimatedEuclideanCamera3 = mEstimatedEuclideanCamera3.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();
            estimatedEuclideanCamera2b.decompose();
            estimatedEuclideanCamera3.decompose();

            List<Point3D> euclideanReconstructedPoints3DPair1 = new ArrayList<>();
            for (int i = 0; i < numPointsPair1; i++) {
                euclideanReconstructedPoints3DPair1.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            List<Point3D> euclideanReconstructedPoints3DPair2 = new ArrayList<>();
            for (int i = 0; i < numPointsPair2; i++) {
                euclideanReconstructedPoints3DPair2.add(
                        mEuclideanReconstructedPoints2.get(i).getPoint());
            }

            //check that most points are in front of all cameras
            int numValidPoints = 0, numInvalidPoints = 0;
            for (int i = 0; i < numPointsPair1; i++) {
                Point3D p = euclideanReconstructedPoints3DPair1.get(i);
                if(estimatedEuclideanCamera1.isPointInFrontOfCamera(p) &&
                        estimatedEuclideanCamera2.isPointInFrontOfCamera(p) &&
                        //estimatedEuclideanCamera2b.isPointInFrontOfCamera(p) &&
                        estimatedEuclideanCamera3.isPointInFrontOfCamera(p)) {

                    assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(p));
                    assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(p));
                    //assertTrue(estimatedEuclideanCamera2b.isPointInFrontOfCamera(p));
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
                Point3D p = euclideanReconstructedPoints3DPair2.get(i);
                if(estimatedEuclideanCamera1.isPointInFrontOfCamera(p) &&
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

            Point3D euclideanCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            Point3D euclideanCenter2 = estimatedEuclideanCamera2.getCameraCenter();
            Point3D euclideanCenter2b = estimatedEuclideanCamera2b.getCameraCenter();
            Point3D euclideanCenter3 = estimatedEuclideanCamera3.getCameraCenter();

            PinholeCameraIntrinsicParameters euclideanIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2b =
                    estimatedEuclideanCamera2b.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic3 =
                    estimatedEuclideanCamera3.getIntrinsicParameters();

            Rotation3D euclideanRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            Rotation3D euclideanRotation2 = estimatedEuclideanCamera2.getCameraRotation();
            Rotation3D euclideanRotation2b = estimatedEuclideanCamera2b.getCameraRotation();
            Rotation3D euclideanRotation3 = estimatedEuclideanCamera3.getCameraRotation();

            //check scale
            double euclideanBaseline = euclideanCenter1.distanceTo(euclideanCenter2);
            double euclideanBaseline2 = euclideanCenter2b.distanceTo(euclideanCenter3);

            //check cameras are correct
            double maxBaseline = Math.max(euclideanBaseline, baseline);
            double absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(euclideanBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(euclideanBaseline, baseline, absoluteScaleError);
            assertEquals(mScale, euclideanBaseline, 10*LARGE_ABSOLUTE_ERROR);

            double maxBaseline2 = Math.max(euclideanBaseline2, baseline2);
            double absoluteScaleError2 = RELATIVE_ERROR * maxBaseline2;
            if (Math.abs(euclideanBaseline2 - baseline2) > absoluteScaleError2) {
                continue;
            }
            assertEquals(euclideanBaseline2, baseline2, absoluteScaleError2);
            assertEquals(mScale2, euclideanBaseline2, 10*LARGE_ABSOLUTE_ERROR);


            //check cameras
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

            assertEquals(euclideanIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2b.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic3.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(euclideanRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2b.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation3.asInhomogeneousMatrix().equals(
                    rotation3.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            //check that points are correct (after scale correction)

            //check that scale error is less than 5%
            assertTrue(Math.abs(baseline / mScale - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline2 / mScale2 - 1.0) < RELATIVE_ERROR);

            MetricTransformation3D scaleTransformation
                    = new MetricTransformation3D(baseline / mScale);
            MetricTransformation3D scaleTransformation2
                    = new MetricTransformation3D(baseline2 / mScale2);

            numValidPoints = 0;
            double scaleX, scaleY, scaleZ;
            for (int i = 0; i < numPointsPair1; i++) {
                Point3D point = points3DPair1.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3DPair1.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(euclideanPoint, rescaledPoint);

                //euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                assertEquals(scaleX, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);

                numValidPoints++;
            }

            if (numValidPoints == 0) {
                continue;
            }

            numValidPoints = 0;
            for (int i = 0; i < numPointsPair2; i++) {
                Point3D point = points3DPair2.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3DPair2.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation2.transform(euclideanPoint, rescaledPoint);

                //euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                if (Math.abs(scaleX - baseline2 / mScale2) > 5*LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline2 / mScale2, 5*LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleY - baseline2 / mScale2) > 5*LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline2 / mScale2, 5*LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleZ - baseline2 / mScale2) > 5*LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline2 / mScale2, 5*LARGE_ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);

                numValidPoints++;
            }

            if (numValidPoints == 0) {
                continue;
            }


            double scaleRelativeError = Math.abs(baseline / mScale - 1.0);
            double scaleRelativeError2 = Math.abs(baseline2 / mScale2 - 1.0);
            LOGGER.log(Level.INFO,
                    "Baseline relative error without noise 1: {0,number,0.000%}",
                    scaleRelativeError);
            LOGGER.log(Level.INFO,
                    "Baseline relative error without noise 2: {0,number,0.000%}",
                    scaleRelativeError2);

            numValid++;

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithNoiseThreeViews()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException, RotationException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer offsetRandomizer = new UniformRandomizer(
                    new Random());
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration configuration =
                    new ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

            float accelerationOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            float angularOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            ConstantVelocityModelSlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY,
                    angularOffsetZ, noiseRandomizer);
            ConstantVelocityModelSlamCalibrationData calibrationData
                    = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getPairedCamerasAspectRatio();
            double skewness = 0.0;
            double principalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);


            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double alphaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            AxisRotation3D axisRotation2 = new AxisRotation3D(rotation2);
            MatrixRotation3D rotation3 = new MatrixRotation3D(alphaEuler3,
                    betaEuler3, gammaEuler3);

            double axis2X = axisRotation2.getAxisX();
            double axis2Y = axisRotation2.getAxisY();
            double axis2Z = axisRotation2.getAxisZ();
            double angle2 = axisRotation2.getRotationAngle();


            AxisRotation3D diffRotation = new AxisRotation3D(axis2X, axis2Y,
                    axis2Z, angle2 / N_SENSOR_SAMPLES);
            Quaternion diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
            double[] angularSpeeds = diffQuaternion.toEulerAngles();
            final double angularSpeed2X = angularSpeeds[0];
            final double angularSpeed2Y = angularSpeeds[1];
            final double angularSpeed2Z = angularSpeeds[2];
            Quaternion diffRotation2 = new Quaternion(angularSpeed2X,
                    angularSpeed2Y, angularSpeed2Z);

            //number of samples (50 samples * 0.02 s/sample = 1 second)
            MatrixRotation3D rotation2b = new MatrixRotation3D(rotation1);
            MatrixRotation3D rotation2c = new MatrixRotation3D(rotation1);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation2b.combine(diffRotation);
                rotation2c.combine(diffRotation2);
            }

            //check that rotations created by composing sensor samples are
            //equal to the original one
            assertTrue(rotation2.equals(rotation2b, ABSOLUTE_ERROR));
            assertTrue(rotation2.equals(rotation2c, ABSOLUTE_ERROR));

            AxisRotation3D accumDiffRotation = rotation2.inverseRotationAndReturnNew().
                    combineAndReturnNew(rotation3).toAxisRotation();
            double axis3X = accumDiffRotation.getAxisX();
            double axis3Y = accumDiffRotation.getAxisY();
            double axis3Z = accumDiffRotation.getAxisZ();
            double angle3 = accumDiffRotation.getRotationAngle();

            diffRotation = new AxisRotation3D(axis3X, axis3Y, axis3Z, angle3 / N_SENSOR_SAMPLES);
            diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
            angularSpeeds = diffQuaternion.toEulerAngles();
            final double angularSpeed3X = angularSpeeds[0];
            final double angularSpeed3Y = angularSpeeds[1];
            final double angularSpeed3Z = angularSpeeds[2];
            diffRotation2 = new Quaternion(angularSpeed3X, angularSpeed3Y,
                    angularSpeed3Z);

            //number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            //previously sampled rotation
            MatrixRotation3D rotation3b = new MatrixRotation3D(rotation2b);
            MatrixRotation3D rotation3c = new MatrixRotation3D(rotation2c);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation3b.combine(diffRotation);
                rotation3c.combine(diffRotation2);
            }

            //check that rotations created by composing sensor samples are equal
            //to the original one
            assertTrue(rotation3.equals(rotation3b, ABSOLUTE_ERROR));
            assertTrue(rotation3.equals(rotation3c, ABSOLUTE_ERROR));

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            double cameraSeparation2 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);


            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            Point3D center3 = new InhomogeneousPoint3D(
                    center2.getInhomX() + cameraSeparation2,
                    center2.getInhomY() + cameraSeparation2,
                    center2.getInhomZ() + cameraSeparation2);

            double baseline = center1.distanceTo(center2);
            double baseline2 = center2.distanceTo(center3);

            final double accelerationX, accelerationY, accelerationZ;
            final double accelerationX2, accelerationY2, accelerationZ2;

            //s = 0.5*a*t^2 --> a = 2*s/t^2
            //assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ
                    = 2 * cameraSeparation;
            accelerationX2 = accelerationY2 = accelerationZ2
                    = 2 * cameraSeparation2;

            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
            PinholeCamera camera3 = new PinholeCamera(intrinsic, rotation3,
                    center3);

            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);
            FundamentalMatrix fundamentalMatrix2 = new FundamentalMatrix(
                    camera2, camera3);

            //create 3D points laying in front of all cameras

            //1st find an approximate central point by intersecting the axis planes of
            //all cameras
            Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            Plane horizontalPlane3 = camera3.getHorizontalAxisPlane();
            Plane verticalPlane3 = camera3.getVerticalAxisPlane();
            Matrix planesIntersectionMatrixPair1 = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            Matrix planesIntersectionMatrixPair2 = new Matrix(
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

            SingularValueDecomposer decomposerPair1 = new SingularValueDecomposer(
                    planesIntersectionMatrixPair1);
            decomposerPair1.decompose();
            Matrix vPair1 = decomposerPair1.getV();

            SingularValueDecomposer decomposerPair2 = new SingularValueDecomposer(
                    planesIntersectionMatrixPair2);
            decomposerPair2.decompose();
            Matrix vPair2 = decomposerPair2.getV();

            HomogeneousPoint3D centralCommonPointPair1 = new HomogeneousPoint3D(
                    vPair1.getElementAt(0, 3),
                    vPair1.getElementAt(1, 3),
                    vPair1.getElementAt(2, 3),
                    vPair1.getElementAt(3, 3));

            HomogeneousPoint3D centralCommonPointPair2 = new HomogeneousPoint3D(
                    vPair2.getElementAt(0, 3),
                    vPair2.getElementAt(1, 3),
                    vPair2.getElementAt(2, 3),
                    vPair2.getElementAt(3, 3));

            double lambdaX, lambdaY, lambdaZ;

            final int numPointsPair1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final int numPointsPair2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3DPair1 = new ArrayList<>();
            List<InhomogeneousPoint3D> points3DPair2 = new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2, projectedPoint3;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2a = new ArrayList<>();
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            final List<Point2D> projectedPoints3 = new ArrayList<>();
            boolean front1, front2, front3;
            for (int i = 0; i < numPointsPair1; i++) {
                //generate points and ensure they lie in front of both cameras
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

                //check that 3D point is in front of 1st pair of cameras
                //noinspection all
                assertTrue(front1);
                //noinspection all
                assertTrue(front2);
                //noinspection all
                assertTrue(front3);

                points3DPair1.add(point3D);

                //project 3D point into 1st pair of cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2a.add(projectedPoint2);
            }

            for (int i = 0; i < numPointsPair2; i++) {
                //generate points and ensure they lie in front of both cameras
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

                //check that 3D point is in front of 2nd pair of cameras
                //noinspection all
                assertTrue(front1);
                //noinspection all
                assertTrue(front2);
                //noinspection all
                assertTrue(front3);

                points3DPair2.add(point3D);

                //project 3D point into 2nd pair of cameras
                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2b.add(projectedPoint2);

                projectedPoint3 = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3);
                projectedPoints3.add(projectedPoint3);
            }

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            ConstantVelocityModelSlamPairedViewsSparseReconstructorListener listener =
                    new ConstantVelocityModelSlamPairedViewsSparseReconstructorListener() {
                        @Override
                        public void onSlamDataAvailable(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, double positionX,
                                double positionY, double positionZ, double velocityX, double velocityY,
                                double velocityZ, double accelerationX, double accelerationY, double accelerationZ,
                                double quaternionA, double quaternionB, double quaternionC, double quaternionD,
                                double angularSpeedX, double angularSpeedY, double angularSpeedZ, Matrix covariance) {
                            mSlamDataAvailable++;
                            mSlamCovariance = covariance;
                        }

                        @Override
                        public void onSlamCameraEstimated(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                                PinholeCamera camera) {
                            mSlamCameraEstimated++;
                            mSlamCamera = camera;
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            return mViewCount < 4; //3 views = 2 view pairs (2 images * 2 views --> 4 view counts)
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            int viewCount = reconstructor.getViewCount();

                            Sample2D sample1, sample2;
                            if(viewCount == 0) {
                                //first view pair
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

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 1st
                                //and 2nd view (50 samples * 0.02 s/sample = 1 second)
                                mTimestamp = 0;
                                float noiseAccelerationX, noiseAccelerationY,
                                        noiseAccelerationZ;
                                float noiseAngularSpeedX, noiseAngularSpeedY,
                                        noiseAngularSpeedZ;

                                float accelerationWithNoiseX, accelerationWithNoiseY,
                                        accelerationWithNoiseZ;
                                float angularSpeedWithNoiseX, angularSpeedWithNoiseY,
                                        angularSpeedWithNoiseZ;

                                float[] accelerationWithNoise = new float[3];
                                float[] angularSpeedWithNoise = new float[3];
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    noiseAccelerationX =
                                            accelerationRandomizer.nextFloat();
                                    noiseAccelerationY =
                                            accelerationRandomizer.nextFloat();
                                    noiseAccelerationZ =
                                            accelerationRandomizer.nextFloat();

                                    noiseAngularSpeedX =
                                            angularSpeedRandomizer.nextFloat();
                                    noiseAngularSpeedY =
                                            angularSpeedRandomizer.nextFloat();
                                    noiseAngularSpeedZ =
                                            angularSpeedRandomizer.nextFloat();

                                    accelerationWithNoiseX = (float)accelerationX +
                                            noiseAccelerationX;
                                    accelerationWithNoiseY = (float)accelerationY +
                                            noiseAccelerationY;
                                    accelerationWithNoiseZ = (float)accelerationZ +
                                            noiseAccelerationZ;
                                    accelerationWithNoise[0] = accelerationWithNoiseX;
                                    accelerationWithNoise[1] = accelerationWithNoiseY;
                                    accelerationWithNoise[2] = accelerationWithNoiseZ;

                                    angularSpeedWithNoiseX = (float)angularSpeed2X +
                                            noiseAngularSpeedX;
                                    angularSpeedWithNoiseY = (float)angularSpeed2Y +
                                            noiseAngularSpeedY;
                                    angularSpeedWithNoiseZ = (float)angularSpeed2Z +
                                            noiseAngularSpeedZ;
                                    angularSpeedWithNoise[0] = angularSpeedWithNoiseX;
                                    angularSpeedWithNoise[1] = angularSpeedWithNoiseY;
                                    angularSpeedWithNoise[2] = angularSpeedWithNoiseZ;

                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            accelerationWithNoise);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            angularSpeedWithNoise);
                                    mTimestamp += DELTA_NANOS;
                                }

                            } else if (viewCount == 2){
                                //second view pair
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

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 2nd
                                //and 3rd view (50 samples * 0.02 s/sample = 1 second)
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            (float) accelerationX2, (float) accelerationY2,
                                            (float) accelerationZ2);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            (float) angularSpeed3X, (float) angularSpeed3Y,
                                            (float) angularSpeed3Z);
                                    mTimestamp += DELTA_NANOS;
                                }
                            }
                        }

                        @Override
                        public void onSamplesAccepted(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2,
                                List<MatchedSamples> matches) {
                            matches.clear();

                            int viewCount = reconstructor.getViewCount();
                            int numPoints;
                            if (viewCount == 0) {
                                //first view pair
                                numPoints = numPointsPair1;
                            } else {
                                //second view pair
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
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            int viewCount = reconstructor.getViewCount();
                            if (viewCount == 0) {
                                mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                            } else if (viewCount == 2) {
                                mEstimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                            }
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, double scale, EstimatedCamera camera1, EstimatedCamera camera2) {

                            int viewCount = reconstructor.getViewCount();
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
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, double scale, List<ReconstructedPoint3D> points) {

                            int viewCount = reconstructor.getViewCount();
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
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor =
                    new ConstantVelocityModelSlamPairedViewsSparseReconstructor(configuration, listener);

            //check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            //check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertTrue(mSlamDataAvailable > 0);
            assertTrue(mSlamCameraEstimated > 0);
            assertNotNull(mSlamCamera);
            assertNotNull(mSlamCovariance);
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

            //check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            fundamentalMatrix2.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();
            mEstimatedFundamentalMatrix2.getFundamentalMatrix().normalize();

            //matrices are equal up to scale
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

            PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();
            PinholeCamera estimatedEuclideanCamera2b = mEstimatedEuclideanCamera2b.getCamera();
            PinholeCamera estimatedEuclideanCamera3 = mEstimatedEuclideanCamera3.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();
            estimatedEuclideanCamera2b.decompose();
            estimatedEuclideanCamera3.decompose();

            List<Point3D> euclideanReconstructedPoints3DPair1 = new ArrayList<>();
            for (int i = 0; i < numPointsPair1; i++) {
                euclideanReconstructedPoints3DPair1.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            List<Point3D> euclideanReconstructedPoints3DPair2 = new ArrayList<>();
            for (int i = 0; i < numPointsPair2; i++) {
                euclideanReconstructedPoints3DPair2.add(
                        mEuclideanReconstructedPoints2.get(i).getPoint());
            }

            //check that most points are in front of all cameras
            int numValidPoints = 0, numInvalidPoints = 0;
            for (int i = 0; i < numPointsPair1; i++) {
                Point3D p = euclideanReconstructedPoints3DPair1.get(i);
                if(estimatedEuclideanCamera1.isPointInFrontOfCamera(p) &&
                        estimatedEuclideanCamera2.isPointInFrontOfCamera(p) &&
                        //estimatedEuclideanCamera2b.isPointInFrontOfCamera(p) &&
                        estimatedEuclideanCamera3.isPointInFrontOfCamera(p)) {

                    assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(p));
                    assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(p));
                    //assertTrue(estimatedEuclideanCamera2b.isPointInFrontOfCamera(p));
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
                Point3D p = euclideanReconstructedPoints3DPair2.get(i);
                if(estimatedEuclideanCamera1.isPointInFrontOfCamera(p) &&
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

            Point3D euclideanCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            Point3D euclideanCenter2 = estimatedEuclideanCamera2.getCameraCenter();
            Point3D euclideanCenter2b = estimatedEuclideanCamera2b.getCameraCenter();
            Point3D euclideanCenter3 = estimatedEuclideanCamera3.getCameraCenter();

            PinholeCameraIntrinsicParameters euclideanIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2b =
                    estimatedEuclideanCamera2b.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic3 =
                    estimatedEuclideanCamera3.getIntrinsicParameters();

            Rotation3D euclideanRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            Rotation3D euclideanRotation2 = estimatedEuclideanCamera2.getCameraRotation();
            Rotation3D euclideanRotation2b = estimatedEuclideanCamera2b.getCameraRotation();
            Rotation3D euclideanRotation3 = estimatedEuclideanCamera3.getCameraRotation();

            //check scale
            double euclideanBaseline = euclideanCenter1.distanceTo(euclideanCenter2);
            double euclideanBaseline2 = euclideanCenter2b.distanceTo(euclideanCenter3);

            //check cameras are correct
            double maxBaseline = Math.max(euclideanBaseline, baseline);
            double absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(euclideanBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(euclideanBaseline, baseline, absoluteScaleError);
            assertEquals(mScale, euclideanBaseline, 10*LARGE_ABSOLUTE_ERROR);

            double maxBaseline2 = Math.max(euclideanBaseline2, baseline2);
            double absoluteScaleError2 = RELATIVE_ERROR * maxBaseline2;
            if (Math.abs(euclideanBaseline2 - baseline2) > absoluteScaleError2) {
                continue;
            }
            assertEquals(euclideanBaseline2, baseline2, absoluteScaleError2);
            assertEquals(mScale2, euclideanBaseline2, 10*LARGE_ABSOLUTE_ERROR);


            //check cameras
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

            assertEquals(euclideanIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2b.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic3.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(euclideanRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2b.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation3.asInhomogeneousMatrix().equals(
                    rotation3.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            //check that points are correct (after scale correction)

            //check that scale error is less than 5%
            assertTrue(Math.abs(baseline / mScale - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline2 / mScale2 - 1.0) < RELATIVE_ERROR);

            MetricTransformation3D scaleTransformation
                    = new MetricTransformation3D(baseline / mScale);
            MetricTransformation3D scaleTransformation2
                    = new MetricTransformation3D(baseline2 / mScale2);

            numValidPoints = 0;
            double scaleX, scaleY, scaleZ;
            for (int i = 0; i < numPointsPair1; i++) {
                Point3D point = points3DPair1.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3DPair1.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(euclideanPoint, rescaledPoint);

                //euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                assertEquals(scaleX, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);

                numValidPoints++;
            }

            if (numValidPoints == 0) {
                continue;
            }

            numValidPoints = 0;
            for (int i = 0; i < numPointsPair2; i++) {
                Point3D point = points3DPair2.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3DPair2.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation2.transform(euclideanPoint, rescaledPoint);

                //euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                if (Math.abs(scaleX - baseline2 / mScale2) > 5*LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline2 / mScale2, 5*LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleY - baseline2 / mScale2) > 5*LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline2 / mScale2, 5*LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleZ - baseline2 / mScale2) > 5*LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline2 / mScale2, 5*LARGE_ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);

                numValidPoints++;
            }

            if (numValidPoints == 0) {
                continue;
            }


            double scaleRelativeError = Math.abs(baseline / mScale - 1.0);
            double scaleRelativeError2 = Math.abs(baseline2 / mScale2 - 1.0);
            LOGGER.log(Level.INFO,
                    "Baseline relative error without noise 1: {0,number,0.000%}",
                    scaleRelativeError);
            LOGGER.log(Level.INFO,
                    "Baseline relative error without noise 2: {0,number,0.000%}",
                    scaleRelativeError2);

            numValid++;

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    @Test
    @SuppressWarnings("all")
    public void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithoutNoiseFourViews()
            throws InvalidPairOfCamerasException, AlgebraException, CameraException, RotationException,
            com.irurueta.geometry.estimators.NotReadyException,
            com.irurueta.geometry.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < 2*TIMES; t++) {
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration configuration =
                    new ConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();
            configuration.setPairedCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            configuration.setIntrinsicParametersKnown(true);

            float accelerationOffsetX = 0.0f;
            float accelerationOffsetY = 0.0f;
            float accelerationOffsetZ = 0.0f;

            float angularOffsetX = 0.0f;
            float angularOffsetY = 0.0f;
            float angularOffsetZ = 0.0f;

            ConstantVelocityModelSlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY,
                    accelerationOffsetZ, angularOffsetX, angularOffsetY,
                    angularOffsetZ, noiseRandomizer);
            ConstantVelocityModelSlamCalibrationData calibrationData
                    = calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getPairedCamerasAspectRatio();
            double skewness = 0.0;
            double principalPoint = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                            principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);


            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double alphaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler3 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double alphaEuler4 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler4 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler4 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            AxisRotation3D axisRotation2 = new AxisRotation3D(rotation2);
            MatrixRotation3D rotation3 = new MatrixRotation3D(alphaEuler3,
                    betaEuler3, gammaEuler3);
            MatrixRotation3D rotation4 = new MatrixRotation3D(alphaEuler4,
                    betaEuler4, gammaEuler4);

            double axis2X = axisRotation2.getAxisX();
            double axis2Y = axisRotation2.getAxisY();
            double axis2Z = axisRotation2.getAxisZ();
            double angle2 = axisRotation2.getRotationAngle();


            AxisRotation3D diffRotation = new AxisRotation3D(axis2X, axis2Y,
                    axis2Z, angle2 / N_SENSOR_SAMPLES);
            Quaternion diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
            double[] angularSpeeds = diffQuaternion.toEulerAngles();
            final double angularSpeed2X = angularSpeeds[0];
            final double angularSpeed2Y = angularSpeeds[1];
            final double angularSpeed2Z = angularSpeeds[2];
            Quaternion diffRotation2 = new Quaternion(angularSpeed2X,
                    angularSpeed2Y, angularSpeed2Z);

            //number of samples (50 samples * 0.02 s/sample = 1 second)
            MatrixRotation3D rotation2b = new MatrixRotation3D(rotation1);
            MatrixRotation3D rotation2c = new MatrixRotation3D(rotation1);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation2b.combine(diffRotation);
                rotation2c.combine(diffRotation2);
            }

            //check that rotations created by composing sensor samples are
            //equal to the original one
            assertTrue(rotation2.equals(rotation2b, ABSOLUTE_ERROR));
            assertTrue(rotation2.equals(rotation2c, ABSOLUTE_ERROR));

            AxisRotation3D accumDiffRotation = rotation2.inverseRotationAndReturnNew().
                    combineAndReturnNew(rotation3).toAxisRotation();
            double axis3X = accumDiffRotation.getAxisX();
            double axis3Y = accumDiffRotation.getAxisY();
            double axis3Z = accumDiffRotation.getAxisZ();
            double angle3 = accumDiffRotation.getRotationAngle();

            diffRotation = new AxisRotation3D(axis3X, axis3Y, axis3Z, angle3 / N_SENSOR_SAMPLES);
            diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
            angularSpeeds = diffQuaternion.toEulerAngles();
            final double angularSpeed3X = angularSpeeds[0];
            final double angularSpeed3Y = angularSpeeds[1];
            final double angularSpeed3Z = angularSpeeds[2];
            diffRotation2 = new Quaternion(angularSpeed3X, angularSpeed3Y,
                    angularSpeed3Z);

            //number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            //previously sampled rotation
            MatrixRotation3D rotation3b = new MatrixRotation3D(rotation2b);
            MatrixRotation3D rotation3c = new MatrixRotation3D(rotation2c);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation3b.combine(diffRotation);
                rotation3c.combine(diffRotation2);
            }

            //check that rotations created by composing sensor samples are equal
            //to the original one
            assertTrue(rotation3.equals(rotation3b, ABSOLUTE_ERROR));
            assertTrue(rotation3.equals(rotation3c, ABSOLUTE_ERROR));

            accumDiffRotation = rotation3.inverseRotationAndReturnNew().
                    combineAndReturnNew(rotation4).toAxisRotation();
            double axis4X = accumDiffRotation.getAxisX();
            double axis4Y = accumDiffRotation.getAxisY();
            double axis4Z = accumDiffRotation.getAxisZ();
            double angle4 = accumDiffRotation.getRotationAngle();

            diffRotation = new AxisRotation3D(axis4X, axis4Y, axis4Z, angle4 / N_SENSOR_SAMPLES);
            diffQuaternion = new Quaternion(diffRotation);

            //angular speeds (roll, pitch, yaw) on x, y, z axes
            angularSpeeds = diffQuaternion.toEulerAngles();
            final double angularSpeed4X = angularSpeeds[0];
            final double angularSpeed4Y = angularSpeeds[1];
            final double angularSpeed4Z = angularSpeeds[2];
            diffRotation2 = new Quaternion(angularSpeed4X, angularSpeed4Y,
                    angularSpeed4Z);

            //number of samples (50 samples * 0.02 s/sample = 1 second), starting from
            //previously sampled rotation
            MatrixRotation3D rotation4b = new MatrixRotation3D(rotation3b);
            MatrixRotation3D rotation4c = new MatrixRotation3D(rotation3c);
            for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                rotation4b.combine(diffRotation);
                rotation4c.combine(diffRotation2);
            }

            //check that rotations created by composing sensor samples are equal
            //to the original one
            assertTrue(rotation4.equals(rotation4b, ABSOLUTE_ERROR));
            assertTrue(rotation4.equals(rotation4c, ABSOLUTE_ERROR));

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            double cameraSeparation2 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            double cameraSeparation3 = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL,
                    MAX_CAMERA_SEPARATION_ESSENTIAL);

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

            double baseline = center1.distanceTo(center2);
            double baseline2 = center2.distanceTo(center3);
            double baseline3 = center3.distanceTo(center4);

            final double accelerationX, accelerationY, accelerationZ;
            final double accelerationX2, accelerationY2, accelerationZ2;
            final double accelerationX3, accelerationY3, accelerationZ3;

            //s = 0.5*a*t^2 --> a = 2*s/t^2
            //assuming t = 1 second (50 samples * 0.02 s/sample = 1 second)
            accelerationX = accelerationY = accelerationZ
                    = 2 * cameraSeparation;
            accelerationX2 = accelerationY2 = accelerationZ2
                    = 2 * cameraSeparation2;
            accelerationX3 = accelerationY3 = accelerationZ3
                    = 2 * cameraSeparation3;

            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
            PinholeCamera camera3 = new PinholeCamera(intrinsic, rotation3,
                    center3);
            PinholeCamera camera4 = new PinholeCamera(intrinsic, rotation4,
                    center4);

            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);
            FundamentalMatrix fundamentalMatrix2 = new FundamentalMatrix(
                    camera2, camera3);
            FundamentalMatrix fundamentalMatrix3 = new FundamentalMatrix(
                    camera3, camera4);

            //create 3D points laying in front of all cameras

            //1st find an approximate central point by intersecting the axis planes of
            //all cameras
            Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
            Plane verticalPlane1 = camera1.getVerticalAxisPlane();
            Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
            Plane verticalPlane2 = camera2.getVerticalAxisPlane();
            Plane horizontalPlane3 = camera3.getHorizontalAxisPlane();
            Plane verticalPlane3 = camera3.getVerticalAxisPlane();
            Plane horizontalPlane4 = camera4.getHorizontalAxisPlane();
            Plane verticalPlane4 = camera4.getVerticalAxisPlane();
            Matrix planesIntersectionMatrixPair1 = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            Matrix planesIntersectionMatrixPair2 = new Matrix(
                    Plane.PLANE_NUMBER_PARAMS, Plane.PLANE_NUMBER_PARAMS);
            Matrix planesIntersectionMatrixPair3 = new Matrix(
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



            planesIntersectionMatrixPair3.setElementAt(0, 0, verticalPlane3.getA());
            planesIntersectionMatrixPair3.setElementAt(0, 1, verticalPlane3.getB());
            planesIntersectionMatrixPair3.setElementAt(0, 2, verticalPlane3.getC());
            planesIntersectionMatrixPair3.setElementAt(0, 3, verticalPlane3.getD());

            planesIntersectionMatrixPair3.setElementAt(1, 0,
                    horizontalPlane3.getA());
            planesIntersectionMatrixPair3.setElementAt(1, 1,
                    horizontalPlane3.getB());
            planesIntersectionMatrixPair3.setElementAt(1, 2,
                    horizontalPlane3.getC());
            planesIntersectionMatrixPair3.setElementAt(1, 3,
                    horizontalPlane3.getD());

            planesIntersectionMatrixPair3.setElementAt(2, 0, verticalPlane4.getA());
            planesIntersectionMatrixPair3.setElementAt(2, 1, verticalPlane4.getB());
            planesIntersectionMatrixPair3.setElementAt(2, 2, verticalPlane4.getC());
            planesIntersectionMatrixPair3.setElementAt(2, 3, verticalPlane4.getD());

            planesIntersectionMatrixPair3.setElementAt(3, 0,
                    horizontalPlane4.getA());
            planesIntersectionMatrixPair3.setElementAt(3, 1,
                    horizontalPlane4.getB());
            planesIntersectionMatrixPair3.setElementAt(3, 2,
                    horizontalPlane4.getC());
            planesIntersectionMatrixPair3.setElementAt(3, 3,
                    horizontalPlane4.getD());

            SingularValueDecomposer decomposerPair1 = new SingularValueDecomposer(
                    planesIntersectionMatrixPair1);
            decomposerPair1.decompose();
            Matrix vPair1 = decomposerPair1.getV();

            SingularValueDecomposer decomposerPair2 = new SingularValueDecomposer(
                    planesIntersectionMatrixPair2);
            decomposerPair2.decompose();
            Matrix vPair2 = decomposerPair2.getV();

            SingularValueDecomposer decomposerPair3 = new SingularValueDecomposer(
                    planesIntersectionMatrixPair3);
            decomposerPair3.decompose();
            Matrix vPair3 = decomposerPair3.getV();

            HomogeneousPoint3D centralCommonPointPair1 = new HomogeneousPoint3D(
                    vPair1.getElementAt(0, 3),
                    vPair1.getElementAt(1, 3),
                    vPair1.getElementAt(2, 3),
                    vPair1.getElementAt(3, 3));

            HomogeneousPoint3D centralCommonPointPair2 = new HomogeneousPoint3D(
                    vPair2.getElementAt(0, 3),
                    vPair2.getElementAt(1, 3),
                    vPair2.getElementAt(2, 3),
                    vPair2.getElementAt(3, 3));

            HomogeneousPoint3D centralCommonPointPair3 = new HomogeneousPoint3D(
                    vPair3.getElementAt(0, 3),
                    vPair3.getElementAt(1, 3),
                    vPair3.getElementAt(2, 3),
                    vPair3.getElementAt(3, 3));

            double lambdaX, lambdaY, lambdaZ;

            final int numPointsPair1 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final int numPointsPair2 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);
            final int numPointsPair3 = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

            InhomogeneousPoint3D point3D;
            List<InhomogeneousPoint3D> points3DPair1 = new ArrayList<>();
            List<InhomogeneousPoint3D> points3DPair2 = new ArrayList<>();
            List<InhomogeneousPoint3D> points3DPair3 = new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2, projectedPoint3, projectedPoint4;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2a = new ArrayList<>();
            final List<Point2D> projectedPoints2b = new ArrayList<>();
            final List<Point2D> projectedPoints3 = new ArrayList<>();
            final List<Point2D> projectedPoints3b = new ArrayList<>();
            final List<Point2D> projectedPoints4 = new ArrayList<>();
            boolean front1, front2, front3, front4;
            for (int i = 0; i < numPointsPair1; i++) {
                //generate points and ensure they lie in front of both cameras
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

                //check that 3D point is in front of 1st pair of cameras
                //noinspection all
                assertTrue(front1);
                //noinspection all
                assertTrue(front2);
                //noinspection all
                assertTrue(front3);

                points3DPair1.add(point3D);

                //project 3D point into 1st pair of cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2a.add(projectedPoint2);
            }

            for (int i = 0; i < numPointsPair2; i++) {
                //generate points and ensure they lie in front of both cameras
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

                //check that 3D point is in front of 2nd pair of cameras
                //noinspection all
                assertTrue(front1);
                //noinspection all
                assertTrue(front2);
                //noinspection all
                assertTrue(front3);

                points3DPair2.add(point3D);

                //project 3D point into 2nd pair of cameras
                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2b.add(projectedPoint2);

                projectedPoint3 = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3);
                projectedPoints3.add(projectedPoint3);
            }

            boolean failed = false;
            for (int i = 0; i < numPointsPair3; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {
                    lambdaX = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaY = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);
                    lambdaZ = randomizer.nextDouble(
                            MIN_LAMBDA_ESSENTIAL, MAX_LAMBDA_ESSENTIAL);

                    point3D = new InhomogeneousPoint3D(
                            center3.getInhomX() +
                                    centralCommonPointPair3.getInhomX() + lambdaX,
                            center3.getInhomY() +
                                    centralCommonPointPair3.getInhomY() + lambdaY,
                            center3.getInhomZ() +
                                    centralCommonPointPair2.getInhomZ() + lambdaZ);

                    //front2 = camera2.isPointInFrontOfCamera(point3D);
                    front3 = camera3.isPointInFrontOfCamera(point3D);
                    front4 = camera4.isPointInFrontOfCamera(point3D);
                    if (numTry > MAX_TRIES) {
                        failed = true;
                        break;
                    }
                    numTry++;
                } while (!front3 || !front4);

                if (failed) {
                    break;
                }

                //check that 3D point is in front of 2nd pair of cameras
                //noinspection all
                assertTrue(front3);
                //noinspection all
                assertTrue(front4);

                points3DPair3.add(point3D);

                //project 3D point into 2nd pair of cameras
                projectedPoint3 = new InhomogeneousPoint2D();
                camera3.project(point3D, projectedPoint3);
                projectedPoints3b.add(projectedPoint3);

                projectedPoint4 = new InhomogeneousPoint2D();
                camera4.project(point3D, projectedPoint4);
                projectedPoints4.add(projectedPoint4);
            }

            if (failed) {
                continue;
            }

            ConstantVelocityModelSlamPairedViewsSparseReconstructorListener listener =
                    new ConstantVelocityModelSlamPairedViewsSparseReconstructorListener() {
                        @Override
                        public void onSlamDataAvailable(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, double positionX,
                                double positionY, double positionZ, double velocityX, double velocityY,
                                double velocityZ, double accelerationX, double accelerationY, double accelerationZ,
                                double quaternionA, double quaternionB, double quaternionC, double quaternionD,
                                double angularSpeedX, double angularSpeedY, double angularSpeedZ, Matrix covariance) {
                            mSlamDataAvailable++;
                            mSlamCovariance = covariance;
                        }

                        @Override
                        public void onSlamCameraEstimated(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor,
                                PinholeCamera camera) {
                            mSlamCameraEstimated++;
                            mSlamCamera = camera;
                        }

                        @Override
                        public boolean hasMoreViewsAvailable(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            return mViewCount < 6; //4 views = 3 view pairs (2 images * 3 views --> 6 view counts)
                        }

                        @Override
                        public void onRequestSamplesForCurrentViewPair(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {

                            samples1.clear();
                            samples2.clear();

                            int viewCount = reconstructor.getViewCount();

                            Sample2D sample1, sample2;
                            if(viewCount == 0) {
                                //first view pair
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

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 1st
                                //and 2nd view (50 samples * 0.02 s/sample = 1 second)
                                mTimestamp = 0;
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            (float) accelerationX, (float) accelerationY,
                                            (float) accelerationZ);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            (float) angularSpeed2X, (float) angularSpeed2Y,
                                            (float) angularSpeed2Z);
                                    mTimestamp += DELTA_NANOS;
                                }

                            } else if (viewCount == 2){
                                //second view pair
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

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 2nd
                                //and 3rd view (50 samples * 0.02 s/sample = 1 second)
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            (float) accelerationX2, (float) accelerationY2,
                                            (float) accelerationZ2);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            (float) angularSpeed3X, (float) angularSpeed3Y,
                                            (float) angularSpeed3Z);
                                    mTimestamp += DELTA_NANOS;
                                }
                            } else if (viewCount == 4) {
                                //third view pair
                                for (int i = 0; i < numPointsPair3; i++) {
                                    sample1 = new Sample2D();
                                    sample1.setPoint(projectedPoints3b.get(i));
                                    sample1.setViewId(viewId1);
                                    samples1.add(sample1);

                                    sample2 = new Sample2D();
                                    sample2.setPoint(projectedPoints4.get(i));
                                    sample2.setViewId(viewId2);
                                    samples2.add(sample2);
                                }

                                //assume the following accelerator and gyroscope samples
                                //are obtained during a period of 1 second between 3rd
                                //and 4th view (50 samples * 0.02 s/sample = 1 second)
                                for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                                    reconstructor.updateAccelerometerSample(mTimestamp,
                                            (float) accelerationX3, (float) accelerationY3,
                                            (float) accelerationZ3);
                                    reconstructor.updateGyroscopeSample(mTimestamp,
                                            (float) angularSpeed4X, (float) angularSpeed4Y,
                                            (float) angularSpeed4Z);
                                    mTimestamp += DELTA_NANOS;
                                }
                            }
                        }

                        @Override
                        public void onSamplesAccepted(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onSamplesRejected(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2) {
                            mViewCount += 2;
                        }

                        @Override
                        public void onRequestMatches(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, List<Sample2D> samples1, List<Sample2D> samples2,
                                List<MatchedSamples> matches) {
                            matches.clear();

                            int viewCount = reconstructor.getViewCount();
                            int numPoints;
                            if (viewCount == 0) {
                                //first view pair
                                numPoints = numPointsPair1;
                            } else if (viewCount == 2){
                                //second view pair
                                numPoints = numPointsPair2;
                            } else {
                                //third view pair
                                numPoints = numPointsPair3;
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
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                            int viewCount = reconstructor.getViewCount();
                            if (viewCount == 0) {
                                mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                            } else if (viewCount == 2) {
                                mEstimatedFundamentalMatrix2 = estimatedFundamentalMatrix;
                            } else if (viewCount == 4) {
                                mEstimatedFundamentalMatrix3 = estimatedFundamentalMatrix;
                            }
                        }

                        @Override
                        public void onEuclideanCameraPairEstimated(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, double scale, EstimatedCamera camera1, EstimatedCamera camera2) {

                            int viewCount = reconstructor.getViewCount();
                            if (viewCount == 0) {
                                mEstimatedEuclideanCamera1 = camera1;
                                mEstimatedEuclideanCamera2 = camera2;
                                mScale = scale;
                            } else if (viewCount == 2) {
                                mEstimatedEuclideanCamera2b = camera1;
                                mEstimatedEuclideanCamera3 = camera2;
                                mScale2 = scale;
                            } else if (viewCount == 4) {
                                mEstimatedEuclideanCamera3b = camera1;
                                mEstimatedEuclideanCamera4 = camera2;
                            }
                        }

                        @Override
                        public void onEuclideanReconstructedPointsEstimated(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId1,
                                int viewId2, double scale, List<ReconstructedPoint3D> points) {

                            int viewCount = reconstructor.getViewCount();
                            if (viewCount == 0) {
                                mEuclideanReconstructedPoints = points;
                                mScale = scale;
                            } else if (viewCount == 2) {
                                mEuclideanReconstructedPoints2 = points;
                                mScale2 = scale;
                            } else if (viewCount == 4) {
                                mEuclideanReconstructedPoints3 = points;
                                mScale3 = scale;
                            }
                        }

                        @Override
                        public PinholeCameraIntrinsicParameters onIntrinsicParametersRequested(
                                ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor, int viewId) {
                            return intrinsic;
                        }

                        @Override
                        public void onStart(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mStarted = true;
                        }

                        @Override
                        public void onFinish(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mFinished = true;
                        }

                        @Override
                        public void onCancel(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mCancelled = true;
                        }

                        @Override
                        public void onFail(ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor) {
                            mFailed = true;
                        }
                    };

            ConstantVelocityModelSlamPairedViewsSparseReconstructor reconstructor =
                    new ConstantVelocityModelSlamPairedViewsSparseReconstructor(configuration, listener);

            //check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());

            reconstructor.start();

            //check correctness
            assertTrue(mStarted);
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            assertTrue(mSlamDataAvailable > 0);
            assertTrue(mSlamCameraEstimated > 0);
            assertNotNull(mSlamCamera);
            assertNotNull(mSlamCovariance);
            assertFalse(reconstructor.isFirstViewPair());
            assertTrue(reconstructor.isAdditionalViewPair());
            assertTrue(reconstructor.getViewCount() > 0);
            assertNotNull(reconstructor.getCurrentEstimatedFundamentalMatrix());
            assertSame(reconstructor.getCurrentEstimatedFundamentalMatrix(), mEstimatedFundamentalMatrix3);
            assertNotNull(reconstructor.getCurrentMetricEstimatedCamera());
            assertNotNull(reconstructor.getPreviousMetricEstimatedCamera());
            assertNotNull(reconstructor.getCurrentEuclideanEstimatedCamera());
            assertSame(reconstructor.getCurrentEuclideanEstimatedCamera(), mEstimatedEuclideanCamera4);
            assertNotNull(reconstructor.getPreviousEuclideanEstimatedCamera());
            assertSame(reconstructor.getPreviousEuclideanEstimatedCamera(), mEstimatedEuclideanCamera3b);
            assertNotNull(reconstructor.getMetricReconstructedPoints());
            assertNotNull(reconstructor.getEuclideanReconstructedPoints());
            assertSame(reconstructor.getEuclideanReconstructedPoints(), mEuclideanReconstructedPoints3);
            assertEquals(reconstructor.getCurrentScale(), mScale3, 0.0);
            assertNotNull(reconstructor.getPreviousViewSamples());
            assertNotNull(reconstructor.getCurrentViewSamples());

            //check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            fundamentalMatrix2.normalize();
            fundamentalMatrix3.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();
            mEstimatedFundamentalMatrix2.getFundamentalMatrix().normalize();
            mEstimatedFundamentalMatrix3.getFundamentalMatrix().normalize();

            //matrices are equal up to scale
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
            if (!fundamentalMatrix3.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix3.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) &&
                    !fundamentalMatrix3.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix3.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix3.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix3.getFundamentalMatrix().
                            getInternalMatrix(), ABSOLUTE_ERROR) ||
                    fundamentalMatrix3.getInternalMatrix().
                            multiplyByScalarAndReturnNew(-1).equals(
                            mEstimatedFundamentalMatrix3.getFundamentalMatrix().
                                    getInternalMatrix(), ABSOLUTE_ERROR));

            PinholeCamera estimatedEuclideanCamera1 = mEstimatedEuclideanCamera1.getCamera();
            PinholeCamera estimatedEuclideanCamera2 = mEstimatedEuclideanCamera2.getCamera();
            PinholeCamera estimatedEuclideanCamera2b = mEstimatedEuclideanCamera2b.getCamera();
            PinholeCamera estimatedEuclideanCamera3 = mEstimatedEuclideanCamera3.getCamera();
            PinholeCamera estimatedEuclideanCamera3b = mEstimatedEuclideanCamera3b.getCamera();
            PinholeCamera estimatedEuclideanCamera4 = mEstimatedEuclideanCamera4.getCamera();

            estimatedEuclideanCamera1.decompose();
            estimatedEuclideanCamera2.decompose();
            estimatedEuclideanCamera2b.decompose();
            estimatedEuclideanCamera3.decompose();
            estimatedEuclideanCamera3b.decompose();
            estimatedEuclideanCamera4.decompose();

            List<Point3D> euclideanReconstructedPoints3DPair1 = new ArrayList<>();
            for (int i = 0; i < numPointsPair1; i++) {
                euclideanReconstructedPoints3DPair1.add(
                        mEuclideanReconstructedPoints.get(i).getPoint());
            }

            List<Point3D> euclideanReconstructedPoints3DPair2 = new ArrayList<>();
            for (int i = 0; i < numPointsPair2; i++) {
                euclideanReconstructedPoints3DPair2.add(
                        mEuclideanReconstructedPoints2.get(i).getPoint());
            }

            List<Point3D> euclideanReconstructedPoints3DPair3 = new ArrayList<>();
            for (int i = 0; i < numPointsPair3; i++) {
                euclideanReconstructedPoints3DPair3.add(
                        mEuclideanReconstructedPoints3.get(i).getPoint());
            }

            //check that most points are in front of all cameras
            int numValidPoints = 0, numInvalidPoints = 0;
            for (int i = 0; i < numPointsPair1; i++) {
                Point3D p = euclideanReconstructedPoints3DPair1.get(i);
                if(estimatedEuclideanCamera1.isPointInFrontOfCamera(p) &&
                        estimatedEuclideanCamera2.isPointInFrontOfCamera(p) &&
                        //estimatedEuclideanCamera2b.isPointInFrontOfCamera(p) &&
                        estimatedEuclideanCamera3.isPointInFrontOfCamera(p)) {

                    assertTrue(estimatedEuclideanCamera1.isPointInFrontOfCamera(p));
                    assertTrue(estimatedEuclideanCamera2.isPointInFrontOfCamera(p));
                    //assertTrue(estimatedEuclideanCamera2b.isPointInFrontOfCamera(p));
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
                Point3D p = euclideanReconstructedPoints3DPair2.get(i);
                if(estimatedEuclideanCamera1.isPointInFrontOfCamera(p) &&
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

            numValidPoints = 0;
            numInvalidPoints = 0;
            for (int i = 0; i < numPointsPair3; i++) {
                Point3D p = euclideanReconstructedPoints3DPair3.get(i);
                if(estimatedEuclideanCamera3b.isPointInFrontOfCamera(p) &&
                        estimatedEuclideanCamera4.isPointInFrontOfCamera(p)) {

                    assertTrue(estimatedEuclideanCamera3b.isPointInFrontOfCamera(p));
                    assertTrue(estimatedEuclideanCamera4.isPointInFrontOfCamera(p));

                    numValidPoints++;
                } else {
                    numInvalidPoints++;
                }
            }

            assertTrue(numValidPoints > numInvalidPoints);

            Point3D euclideanCenter1 = estimatedEuclideanCamera1.getCameraCenter();
            Point3D euclideanCenter2 = estimatedEuclideanCamera2.getCameraCenter();
            Point3D euclideanCenter2b = estimatedEuclideanCamera2b.getCameraCenter();
            Point3D euclideanCenter3 = estimatedEuclideanCamera3.getCameraCenter();
            Point3D euclideanCenter3b = estimatedEuclideanCamera3b.getCameraCenter();
            Point3D euclideanCenter4 = estimatedEuclideanCamera4.getCameraCenter();

            PinholeCameraIntrinsicParameters euclideanIntrinsic1 =
                    estimatedEuclideanCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2 =
                    estimatedEuclideanCamera2.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic2b =
                    estimatedEuclideanCamera2b.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic3 =
                    estimatedEuclideanCamera3.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic3b =
                    estimatedEuclideanCamera3b.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters euclideanIntrinsic4 =
                    estimatedEuclideanCamera4.getIntrinsicParameters();

            Rotation3D euclideanRotation1 = estimatedEuclideanCamera1.getCameraRotation();
            Rotation3D euclideanRotation2 = estimatedEuclideanCamera2.getCameraRotation();
            Rotation3D euclideanRotation2b = estimatedEuclideanCamera2b.getCameraRotation();
            Rotation3D euclideanRotation3 = estimatedEuclideanCamera3.getCameraRotation();
            Rotation3D euclideanRotation3b = estimatedEuclideanCamera3b.getCameraRotation();
            Rotation3D euclideanRotation4 = estimatedEuclideanCamera4.getCameraRotation();

            //check scale
            double euclideanBaseline = euclideanCenter1.distanceTo(euclideanCenter2);
            double euclideanBaseline2 = euclideanCenter2b.distanceTo(euclideanCenter3);
            double euclideanBaseline3 = euclideanCenter3b.distanceTo(euclideanCenter4);

            //check cameras are correct
            double maxBaseline = Math.max(euclideanBaseline, baseline);
            double absoluteScaleError = RELATIVE_ERROR * maxBaseline;
            if (Math.abs(euclideanBaseline - baseline) > absoluteScaleError) {
                continue;
            }
            assertEquals(euclideanBaseline, baseline, absoluteScaleError);
            assertEquals(mScale, euclideanBaseline, 10*LARGE_ABSOLUTE_ERROR);

            double maxBaseline2 = Math.max(euclideanBaseline2, baseline2);
            double absoluteScaleError2 = RELATIVE_ERROR * maxBaseline2;
            if (Math.abs(euclideanBaseline2 - baseline2) > absoluteScaleError2) {
                continue;
            }
            assertEquals(euclideanBaseline2, baseline2, absoluteScaleError2);
            assertEquals(mScale2, euclideanBaseline2, 10*LARGE_ABSOLUTE_ERROR);

            double maxBaseline3 = Math.max(euclideanBaseline3, baseline3);
            double absoluteScaleError3 = RELATIVE_ERROR * maxBaseline3;
            if (Math.abs(euclideanBaseline3 - baseline3) > absoluteScaleError3) {
                continue;
            }
            assertEquals(euclideanBaseline3, baseline3, absoluteScaleError3);
            assertEquals(mScale3, euclideanBaseline3, 20*LARGE_ABSOLUTE_ERROR);


            //check cameras
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

            assertEquals(euclideanIntrinsic1.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic1.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic2b.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic2b.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic3.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic3b.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3b.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3b.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3b.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic3b.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(euclideanIntrinsic4.getHorizontalFocalLength(),
                    intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic4.getVerticalFocalLength(),
                    intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic4.getSkewness(),
                    intrinsic.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic4.getHorizontalPrincipalPoint(),
                    intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(euclideanIntrinsic4.getVerticalPrincipalPoint(),
                    intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertTrue(euclideanRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation2b.asInhomogeneousMatrix().equals(
                    rotation2.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation3.asInhomogeneousMatrix().equals(
                    rotation3.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation3b.asInhomogeneousMatrix().equals(
                    rotation3.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
            assertTrue(euclideanRotation4.asInhomogeneousMatrix().equals(
                    rotation4.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            //check that points are correct (after scale correction)

            //check that scale error is less than 5%
            assertTrue(Math.abs(baseline / mScale - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline2 / mScale2 - 1.0) < RELATIVE_ERROR);
            assertTrue(Math.abs(baseline3 / mScale3 - 1.0) < RELATIVE_ERROR);

            MetricTransformation3D scaleTransformation
                    = new MetricTransformation3D(baseline / mScale);
            MetricTransformation3D scaleTransformation2
                    = new MetricTransformation3D(baseline2 / mScale2);
            MetricTransformation3D scaleTransformation3
                    = new MetricTransformation3D(baseline3 / mScale3);

            numValidPoints = 0;
            double scaleX, scaleY, scaleZ;
            for (int i = 0; i < numPointsPair1; i++) {
                Point3D point = points3DPair1.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3DPair1.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(euclideanPoint, rescaledPoint);

                //euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                assertEquals(scaleX, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleY, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertEquals(scaleZ, baseline / mScale, LARGE_ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);

                numValidPoints++;
            }

            if (numValidPoints == 0) {
                continue;
            }

            numValidPoints = 0;
            for (int i = 0; i < numPointsPair2; i++) {
                Point3D point = points3DPair2.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3DPair2.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation2.transform(euclideanPoint, rescaledPoint);

                //euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                if (Math.abs(scaleX - baseline2 / mScale2) > 5*LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline2 / mScale2, 5*LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleY - baseline2 / mScale2) > 5*LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline2 / mScale2, 5*LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleZ - baseline2 / mScale2) > 5*LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline2 / mScale2, 5*LARGE_ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);

                numValidPoints++;
            }

            if (numValidPoints == 0) {
                continue;
            }

            numValidPoints = 0;
            for (int i = 0; i < numPointsPair3; i++) {
                Point3D point = points3DPair3.get(i);
                Point3D euclideanPoint = euclideanReconstructedPoints3DPair3.get(i);

                //check metric points
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation3.transform(euclideanPoint, rescaledPoint);

                //euclidean and rescaled points match
                assertTrue(euclideanPoint.equals(rescaledPoint, LARGE_ABSOLUTE_ERROR));

                //check euclidean points
                scaleX = point.getInhomX() / euclideanPoint.getInhomX();
                scaleY = point.getInhomY() / euclideanPoint.getInhomY();
                scaleZ = point.getInhomZ() / euclideanPoint.getInhomZ();

                //check that scale error is less than 5%
                if (Math.abs(scaleX - baseline3 / mScale3) > 5*LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleX, baseline3 / mScale3, 5*LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleY - baseline3 / mScale3) > 5*LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleY, baseline3 / mScale3, 5*LARGE_ABSOLUTE_ERROR);
                if (Math.abs(scaleZ - baseline3 / mScale3) > 5*LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(scaleZ, baseline3 / mScale3, 5*LARGE_ABSOLUTE_ERROR);
                assertTrue(Math.abs(scaleX - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleY - 1.0) < RELATIVE_ERROR);
                assertTrue(Math.abs(scaleZ - 1.0) < RELATIVE_ERROR);

                numValidPoints++;
            }

            if (numValidPoints == 0) {
                continue;
            }


            double scaleRelativeError = Math.abs(baseline / mScale - 1.0);
            double scaleRelativeError2 = Math.abs(baseline2 / mScale2 - 1.0);
            double scaleRelativeError3 = Math.abs(baseline3 / mScale3 - 1.0);
            LOGGER.log(Level.INFO,
                    "Baseline relative error without noise 1: {0,number,0.000%}",
                    scaleRelativeError);
            LOGGER.log(Level.INFO,
                    "Baseline relative error without noise 2: {0,number,0.000%}",
                    scaleRelativeError2);
            LOGGER.log(Level.INFO,
                    "Baseline relative error without noise 2: {0,number,0.000%}",
                    scaleRelativeError3);

            numValid++;

            if (numValid > 0) {
                break;
            }
        }

        assertTrue(numValid > 0);
    }

    private void reset() {
        mViewCount = 0;
        mEstimatedFundamentalMatrix = mEstimatedFundamentalMatrix2 =
                mEstimatedFundamentalMatrix3 = null;
        mEstimatedEuclideanCamera1 = mEstimatedEuclideanCamera2 =
                mEstimatedEuclideanCamera3 = mEstimatedEuclideanCamera4 = null;
        mEuclideanReconstructedPoints = null;
        mStarted = mFinished = mFailed = mCancelled = false;
        mScale = 0.0;
        mTimestamp = 0;
        mSlamDataAvailable = 0;
        mSlamCameraEstimated = 0;
        mSlamCamera = null;
    }

    private ConstantVelocityModelSlamCalibrator createFinishedCalibrator(float accelerationOffsetX,
            float accelerationOffsetY, float accelerationOffsetZ, float angularOffsetX, float angularOffsetY,
            float angularOffsetZ, GaussianRandomizer noiseRandomizer) {
        ConstantVelocityModelSlamCalibrator calibrator = ConstantVelocityModelSlamEstimator.createCalibrator();
        calibrator.setConvergenceThreshold(ABSOLUTE_ERROR);
        calibrator.setMaxNumSamples(MAX_CALIBRATION_SAMPLES);

        long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;

        float accelerationNoiseX, accelerationNoiseY, accelerationNoiseZ;
        float angularNoiseX, angularNoiseY, angularNoiseZ;

        double accelerationX, accelerationY, accelerationZ;
        double angularX, angularY, angularZ;

        for (int i = 0; i < MAX_CALIBRATION_SAMPLES; i++) {
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

            calibrator.updateAccelerometerSample(timestamp, (float) accelerationX,
                    (float) accelerationY, (float) accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float) angularX, (float) angularY,
                    (float) angularZ);

            if (calibrator.isFinished()) {
                break;
            }

            timestamp += DELTA_NANOS;
        }

        return calibrator;
    }
}
