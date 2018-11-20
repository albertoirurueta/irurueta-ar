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
import com.irurueta.ar.slam.AbsoluteOrientationConstantVelocityModelSlamCalibrationData;
import com.irurueta.ar.slam.AbsoluteOrientationConstantVelocityModelSlamCalibrator;
import com.irurueta.ar.slam.AbsoluteOrientationConstantVelocityModelSlamEstimator;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorTest {
    
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
    
    private int mViewCount = 0;
    private EstimatedFundamentalMatrix mEstimatedFundamentalMatrix;
    private EstimatedCamera mEstimatedCamera1;
    private EstimatedCamera mEstimatedCamera2;
    private List<ReconstructedPoint3D> mReconstructedPoints;

    private boolean mStarted;
    private boolean mFinished;
    private boolean mFailed;
    private boolean mCancelled;

    private int mSlamDataAvailable;
    private int mSlamCameraEstimated;

    private PinholeCamera mSlamCamera;
    private Matrix mSlamCovariance;

    public AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() {
        mViewCount = 0;
        mEstimatedFundamentalMatrix = null;
        mEstimatedCamera1 = mEstimatedCamera2 = null;
        mReconstructedPoints = null;
        mStarted = mFinished = mFailed = mCancelled = false;
        mSlamDataAvailable = 0;
        mSlamCameraEstimated = 0;
        mSlamCamera = null;
        mSlamCovariance = null;
    }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration configuration =
                new AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration();
        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorListener listener =
                new AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorListener() {
            @Override
            public void onSlamDataAvailable(
                    AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor,
                    double positionX, double positionY, double positionZ,
                    double velocityX, double velocityY, double velocityZ,
                    double accelerationX, double accelerationY, double accelerationZ,
                    double quaternionA, double quaternionB, double quaternionC, double quaternionD,
                    double angularSpeedX, double angularSpeedY, double angularSpeedZ, Matrix covariance) { }

            @Override
            public void onSlamCameraEstimated(
                    AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor,
                    PinholeCamera camera) { }

            @Override
            public boolean hasMoreViewsAvailable(
                    AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                return false;
            }

            @Override
            public void onRequestSamplesForCurrentView(
                    AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                    int viewId, List<Sample2D> samples) { }

            @Override
            public void onSamplesAccepted(
                    AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                    int viewId, List<Sample2D> samples) { }

            @Override
            public void onSamplesRejected(
                    AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                    int viewId, List<Sample2D> samples) { }

            @Override
            public void onRequestMatches(
                    AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                    List<Sample2D> samples1, List<Sample2D> samples2, 
                    int viewId1, int viewId2, List<MatchedSamples> matches) { }

            @Override
            public void onFundamentalMatrixEstimated(
                    AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                    EstimatedFundamentalMatrix estimatedFundamentalMatrix) { }

            @Override
            public void onCamerasEstimated(
                    AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                    int viewId1, int viewId2, EstimatedCamera camera1, 
                    EstimatedCamera camera2) { }

            @Override
            public void onReconstructedPointsEstimated(
                    AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                    List<MatchedSamples> matches, 
                    List<ReconstructedPoint3D> points) { }

            @Override
            public void onStart(
                    AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) { }

            @Override
            public void onFinish(
                    AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) { }

            @Override
            public void onCancel(
                    AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) { }

            @Override
            public void onFail(
                    AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) { }
        };
        
        //constructor with listener
        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor =
                new AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor(
                        listener);
        
        //check default values
        assertNotNull(reconstructor.getConfiguration());
        assertSame(reconstructor.getListener(), listener);
        assertFalse(reconstructor.isRunning());
        assertFalse(reconstructor.isCancelled());
        assertFalse(reconstructor.hasFailed());
        assertFalse(reconstructor.isFinished());
        assertEquals(reconstructor.getViewCount(), 0);
        assertNull(reconstructor.getEstimatedFundamentalMatrix());
        assertNull(reconstructor.getEstimatedCamera1());
        assertNull(reconstructor.getEstimatedCamera2());
        assertNull(reconstructor.getReconstructedPoints());

        //constructor with configuration and listener
        reconstructor = new AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor(
                configuration, listener);
        
        //check default values
        assertSame(reconstructor.getConfiguration(), configuration);
        assertSame(reconstructor.getListener(), listener);
        assertFalse(reconstructor.isRunning());
        assertFalse(reconstructor.isCancelled());
        assertFalse(reconstructor.hasFailed());
        assertEquals(reconstructor.getViewCount(), 0);
        assertNull(reconstructor.getEstimatedFundamentalMatrix());
        assertNull(reconstructor.getEstimatedCamera1());
        assertNull(reconstructor.getEstimatedCamera2());
        assertNull(reconstructor.getReconstructedPoints());        
    }
    
    @Test
    public void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithoutNoise()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException, RotationException, NotReadyException,
            NotAvailableException {
        
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
        
            AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration configuration =
                    new AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            
            float accelerationOffsetX = 0.0f;
            float accelerationOffsetY = 0.0f;
            float accelerationOffsetZ = 0.0f;

            float angularOffsetX = 0.0f;
            float angularOffsetY = 0.0f;
            float angularOffsetZ = 0.0f;
            
            AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator = 
                    createFinishedCalibrator(accelerationOffsetX, 
                    accelerationOffsetY, accelerationOffsetZ, angularOffsetX, 
                    angularOffsetY, angularOffsetZ, noiseRandomizer);
            AbsoluteOrientationConstantVelocityModelSlamCalibrationData calibrationData =
                    calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPoint = 0.0;

            PinholeCameraIntrinsicParameters intrinsic
                    = new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);
            
            double alphaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            AxisRotation3D axisDiffRotation = new AxisRotation3D(
                    rotation1.inverseRotationAndReturnNew().
                            combineAndReturnNew(rotation2));

            double axisX = axisDiffRotation.getAxisX();
            double axisY = axisDiffRotation.getAxisY();
            double axisZ = axisDiffRotation.getAxisZ();
            double angle = axisDiffRotation.getRotationAngle();

            AxisRotation3D diffRotation = new AxisRotation3D(axisX, axisY,
                    axisZ, angle / N_SENSOR_SAMPLES);
            final Quaternion diffQuaternion = new Quaternion(diffRotation);
            
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

            EuclideanTransformation3D rotationTransformation =
                    new EuclideanTransformation3D(rotation1);            
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            center1 = rotationTransformation.transformAndReturnNew(center1);
            center2 = rotationTransformation.transformAndReturnNew(center2);

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
            List<InhomogeneousPoint3D> points3D = new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1, front2, maxTriesReached = false;
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
                        maxTriesReached = true;
                        break;
                    }
                    numTry++;
                } while (!front1 || !front2);
                
                if (maxTriesReached) break;
                
                points3D.add(point3D);

                //project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }
            
            if (maxTriesReached) continue;
            
            AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorListener listener =
                    new AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor,
                        double positionX, double positionY, double positionZ,
                        double velocityX, double velocityY, double velocityZ,
                        double accelerationX, double accelerationY, double accelerationZ,
                        double quaternionA, double quaternionB, double quaternionC, double quaternionD,
                        double angularSpeedX, double angularSpeedY, double angularSpeedZ, Matrix covariance) {
                    mSlamDataAvailable++;
                    mSlamCovariance = covariance;
                }

                @Override
                public void onSlamCameraEstimated(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor,
                        PinholeCamera camera) {
                    mSlamCameraEstimated++;
                    mSlamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    return mViewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        int viewId, List<Sample2D> samples) {
                    samples.clear();

                    Sample2D sample;
                    if (mViewCount == 0) {
                        //first view
                        for (int i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }

                        //assume the following accelerator and gyroscope samples
                        //are obtained during a period of 1 second between 1st
                        //and 2nd view (50 samples * 0.02 s/sample = 1 second)
                        long timestamp = 0;
                        Quaternion orientation = new Quaternion(rotation1);
                        for (int s = 0; s < N_SENSOR_SAMPLES; s++) {
                            reconstructor.updateAccelerometerSample(timestamp,
                                    (float) accelerationX, (float) accelerationY,
                                    (float) accelerationZ);
                            reconstructor.updateGyroscopeSample(timestamp,
                                    (float) angularSpeedX, (float) angularSpeedY,
                                    (float) angularSpeedZ);
                            reconstructor.updateOrientationSample(timestamp, 
                                    orientation);
                            //actualizamos orientación
                            orientation.combine(diffQuaternion);                  
                            timestamp += DELTA_NANOS;
                        }

                    } else {
                        //second view
                        for (int i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        int viewId, List<Sample2D> samples) {
                    mViewCount++;
                }

                @Override
                public void onSamplesRejected(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        int viewId, List<Sample2D> samples) { }

                @Override
                public void onRequestMatches(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        List<Sample2D> samples1, List<Sample2D> samples2, 
                        int viewId1, int viewId2, 
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
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        int viewId1, int viewId2, EstimatedCamera camera1, 
                        EstimatedCamera camera2) {
                    mEstimatedCamera1 = camera1;
                    mEstimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        List<MatchedSamples> matches, 
                        List<ReconstructedPoint3D> points) {
                    mReconstructedPoints = points;
                }

                @Override
                public void onStart(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    mStarted = true;
                }

                @Override
                public void onFinish(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    mFinished = true;
                }

                @Override
                public void onCancel(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    mCancelled = true;
                }

                @Override
                public void onFail(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    mFailed = true;
                }
            };
            
            AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor =
                    new AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor(
                            configuration, listener);
            
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

            //check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();
            
            //matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                    getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundamentalMatrix.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1).equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                    getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundamentalMatrix.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1).equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                    getInternalMatrix(), ABSOLUTE_ERROR));
            
            //check that reconstructed points are in a euclidean stratum (with            
            //correct scale)
            PinholeCamera estimatedCamera1 = mEstimatedCamera1.getCamera();
            PinholeCamera estimatedCamera2 = mEstimatedCamera2.getCamera();

            estimatedCamera1.decompose();
            estimatedCamera2.decompose();

            List<Point3D> reconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                reconstructedPoints3D.add(
                        mReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                Point3D p = reconstructedPoints3D.get(i);
                assertTrue(estimatedCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedCamera2.isPointInFrontOfCamera(p));
            }

            Point3D estimatedCenter1 = estimatedCamera1.getCameraCenter();
            Point3D estimatedCenter2 = estimatedCamera2.getCameraCenter();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsic1
                    = estimatedCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters estimatedIntrinsic2
                    = estimatedCamera2.getIntrinsicParameters();

            Rotation3D estimatedRotation1
                    = estimatedCamera1.getCameraRotation();
            Rotation3D estimatedRotation2
                    = estimatedCamera2.getCameraRotation();

            double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);

            //check cameras are correct

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

            //check that points are correct (after scale correction)
            double scale = baseline / estimatedBaseline;
            MetricTransformation3D scaleTransformation
                    = new MetricTransformation3D(scale);

            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(reconstructedPoints3D.get(i),
                        rescaledPoint);
                if (!points3D.get(i).equals(rescaledPoint, 
                        LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(rescaledPoint,
                        LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            numValid++;

            if (numValid > 0) {
                break;
            }            
        }
        
        assertTrue(numValid > 0);
    }

    @Test
    public void testGeneralPointsEssentialWithConstantAccelerationAndRotationWithNoise()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException, RotationException, NotReadyException,
            NotAvailableException {
        
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer offsetRandomizer = new UniformRandomizer(
                    new Random());                                    
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
        
            AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration configuration =
                    new AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
            
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
            
            AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator = 
                    createFinishedCalibrator(accelerationOffsetX, 
                    accelerationOffsetY, accelerationOffsetZ, angularOffsetX, 
                    angularOffsetY, angularOffsetZ, noiseRandomizer);
            AbsoluteOrientationConstantVelocityModelSlamCalibrationData calibrationData =
                    calibrator.getCalibrationData();
            configuration.setCalibrationData(calibrationData);

            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL,
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPoint = 0.0;

            PinholeCameraIntrinsicParameters intrinsic
                    = new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);
            
            double alphaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            AxisRotation3D axisDiffRotation = new AxisRotation3D(
                    rotation1.inverseRotationAndReturnNew().
                            combineAndReturnNew(rotation2));

            double axisX = axisDiffRotation.getAxisX();
            double axisY = axisDiffRotation.getAxisY();
            double axisZ = axisDiffRotation.getAxisZ();
            double angle = axisDiffRotation.getRotationAngle();

            AxisRotation3D diffRotation = new AxisRotation3D(axisX, axisY,
                    axisZ, angle / N_SENSOR_SAMPLES);
            final Quaternion diffQuaternion = new Quaternion(diffRotation);
            
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

            EuclideanTransformation3D rotationTransformation =
                    new EuclideanTransformation3D(rotation1);            
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            center1 = rotationTransformation.transformAndReturnNew(center1);
            center2 = rotationTransformation.transformAndReturnNew(center2);

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
            List<InhomogeneousPoint3D> points3D = new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1, front2, maxTriesReached = false;
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
                        maxTriesReached = true;
                        break;
                    }
                    numTry++;
                } while (!front1 || !front2);
                
                if (maxTriesReached) break;
                
                points3D.add(point3D);

                //project 3D point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }
            
            if (maxTriesReached) continue;            
            
            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            
            AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorListener listener =
                    new AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorListener() {
                @Override
                public void onSlamDataAvailable(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor,
                        double positionX, double positionY, double positionZ,
                        double velocityX, double velocityY, double velocityZ,
                        double accelerationX, double accelerationY, double accelerationZ,
                        double quaternionA, double quaternionB, double quaternionC, double quaternionD,
                        double angularSpeedX, double angularSpeedY, double angularSpeedZ, Matrix covariance) {
                    mSlamDataAvailable++;
                    mSlamCovariance = covariance;
                }

                @Override
                public void onSlamCameraEstimated(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor,
                        PinholeCamera camera) {
                    mSlamCameraEstimated++;
                    mSlamCamera = camera;
                }

                @Override
                public boolean hasMoreViewsAvailable(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    return mViewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        int viewId, List<Sample2D> samples) {
                    samples.clear();

                    Sample2D sample;
                    if (mViewCount == 0) {
                        //first view
                        for (int i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints1.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }

                        //assume the following accelerator and gyroscope samples
                        //are obtained during a period of 1 second between 1st
                        //and 2nd view (50 samples * 0.02 s/sample = 1 second)
                        long timestamp = 0;
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
                        
                        Quaternion orientation = new Quaternion(rotation1);
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
                            
                            reconstructor.updateAccelerometerSample(timestamp,
                                    accelerationWithNoise);
                            reconstructor.updateGyroscopeSample(timestamp,
                                    angularSpeedWithNoise);
                            reconstructor.updateOrientationSample(timestamp, 
                                    orientation);
                            //actualizamos orientación
                            orientation.combine(diffQuaternion);                  
                            timestamp += DELTA_NANOS;
                        }

                    } else {
                        //second view
                        for (int i = 0; i < numPoints; i++) {
                            sample = new Sample2D();
                            sample.setPoint(projectedPoints2.get(i));
                            sample.setViewId(viewId);
                            samples.add(sample);
                        }
                    }
                }

                @Override
                public void onSamplesAccepted(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        int viewId, List<Sample2D> samples) {
                    mViewCount++;
                }

                @Override
                public void onSamplesRejected(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        int viewId, List<Sample2D> samples) { }

                @Override
                public void onRequestMatches(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        List<Sample2D> samples1, List<Sample2D> samples2, 
                        int viewId1, int viewId2, 
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
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        int viewId1, int viewId2, EstimatedCamera camera1, 
                        EstimatedCamera camera2) {
                    mEstimatedCamera1 = camera1;
                    mEstimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor, 
                        List<MatchedSamples> matches, 
                        List<ReconstructedPoint3D> points) {
                    mReconstructedPoints = points;
                }

                @Override
                public void onStart(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    mStarted = true;
                }

                @Override
                public void onFinish(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    mFinished = true;
                }

                @Override
                public void onCancel(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    mCancelled = true;
                }

                @Override
                public void onFail(
                        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor) {
                    mFailed = true;
                }
            };
            
            AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor reconstructor =
                    new AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructor(
                            configuration, listener);
            
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

            //check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();
            
            //matrices are equal up to scale
            if (!fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                    getInternalMatrix(), ABSOLUTE_ERROR)
                    && !fundamentalMatrix.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1).equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                    getInternalMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundamentalMatrix.getInternalMatrix().equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                    getInternalMatrix(), ABSOLUTE_ERROR)
                    || fundamentalMatrix.getInternalMatrix().
                    multiplyByScalarAndReturnNew(-1).equals(
                    mEstimatedFundamentalMatrix.getFundamentalMatrix().
                    getInternalMatrix(), ABSOLUTE_ERROR));
            
            //check that reconstructed points are in a euclidean stratum (with            
            //correct scale)
            PinholeCamera estimatedCamera1 = mEstimatedCamera1.getCamera();
            PinholeCamera estimatedCamera2 = mEstimatedCamera2.getCamera();

            estimatedCamera1.decompose();
            estimatedCamera2.decompose();

            List<Point3D> reconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                reconstructedPoints3D.add(
                        mReconstructedPoints.get(i).getPoint());
            }

            //check that all points are in front of both cameras
            for (int i = 0; i < numPoints; i++) {
                Point3D p = reconstructedPoints3D.get(i);
                assertTrue(estimatedCamera1.isPointInFrontOfCamera(p));
                assertTrue(estimatedCamera2.isPointInFrontOfCamera(p));
            }

            Point3D estimatedCenter1 = estimatedCamera1.getCameraCenter();
            Point3D estimatedCenter2 = estimatedCamera2.getCameraCenter();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsic1
                    = estimatedCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters estimatedIntrinsic2
                    = estimatedCamera2.getIntrinsicParameters();

            Rotation3D estimatedRotation1
                    = estimatedCamera1.getCameraRotation();
            Rotation3D estimatedRotation2
                    = estimatedCamera2.getCameraRotation();

            double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);

            //check cameras are correct

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

            //check that points are correct (after scale correction)
            double scale = baseline / estimatedBaseline;
            MetricTransformation3D scaleTransformation
                    = new MetricTransformation3D(scale);

            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {
                Point3D rescaledPoint = Point3D.create();
                scaleTransformation.transform(reconstructedPoints3D.get(i),
                        rescaledPoint);
                if (!points3D.get(i).equals(rescaledPoint, 
                        LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(rescaledPoint,
                        LARGE_ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            numValid++;

            if (numValid > 0) {
                break;
            }            
        }
        
        assertTrue(numValid > 0);
    }
    
    private void reset() {
        mViewCount = 0;
        mEstimatedFundamentalMatrix = null;
        mEstimatedCamera1 = mEstimatedCamera2 = null;
        mReconstructedPoints = null;
        mStarted = mFinished = mCancelled = mFailed = false;
        mSlamDataAvailable = 0;
        mSlamCameraEstimated = 0;
        mSlamCamera = null;
        mSlamCovariance = null;
    }

    private AbsoluteOrientationConstantVelocityModelSlamCalibrator createFinishedCalibrator(
            float accelerationOffsetX, float accelerationOffsetY, 
            float accelerationOffsetZ, float angularOffsetX, 
            float angularOffsetY, float angularOffsetZ,
            GaussianRandomizer noiseRandomizer) {
        AbsoluteOrientationConstantVelocityModelSlamCalibrator calibrator = 
                AbsoluteOrientationConstantVelocityModelSlamEstimator.createCalibrator();
        calibrator.setConvergenceThreshold(ABSOLUTE_ERROR);
        calibrator.setMaxNumSamples(MAX_CALIBRATION_SAMPLES);
        
        long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
        
        float accelerationNoiseX, accelerationNoiseY, accelerationNoiseZ;
        float angularNoiseX, angularNoiseY, angularNoiseZ;
        
        double accelerationX, accelerationY, accelerationZ;
        double angularX, angularY, angularZ;
        Quaternion orientation = new Quaternion();

        for(int i = 0; i < MAX_CALIBRATION_SAMPLES; i++) {
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
            
            calibrator.updateAccelerometerSample(timestamp, (float)accelerationX, 
                    (float)accelerationY, (float)accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float)angularX, (float)angularY, 
                    (float)angularZ);
            calibrator.updateOrientationSample(timestamp, orientation);            
            
            if(calibrator.isFinished()) break;
            
            timestamp += DELTA_NANOS;
        }
        
        return calibrator;
    }            
}
