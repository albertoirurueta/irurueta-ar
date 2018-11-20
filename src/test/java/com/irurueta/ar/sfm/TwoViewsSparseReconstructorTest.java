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
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.MetricTransformation3DRobustEstimator;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class TwoViewsSparseReconstructorTest {
    
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
    private EstimatedCamera mEstimatedCamera1;
    private EstimatedCamera mEstimatedCamera2;
    private List<ReconstructedPoint3D> mReconstructedPoints;
    
    private boolean mStarted;
    private boolean mFinished;
    private boolean mFailed;
    private boolean mCancelled;
    
    public TwoViewsSparseReconstructorTest() { }
    
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
    }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        TwoViewsSparseReconstructorConfiguration configuration = 
                new TwoViewsSparseReconstructorConfiguration();
        TwoViewsSparseReconstructorListener listener =
                new TwoViewsSparseReconstructorListener() {
                    
            @Override
            public boolean hasMoreViewsAvailable(
                    TwoViewsSparseReconstructor reconstructor) {
                return false;
            }

            @Override
            public void onRequestSamplesForCurrentView(
                    TwoViewsSparseReconstructor reconstructor, int viewId, 
                    List<Sample2D> samples) { }

            @Override
            public void onSamplesAccepted(
                    TwoViewsSparseReconstructor reconstructor, int viewId, 
                    List<Sample2D> samples) { }

            @Override
            public void onSamplesRejected(
                    TwoViewsSparseReconstructor reconstructor, int viewId, 
                    List<Sample2D> samples) { }

            @Override
            public void onRequestMatches(
                    TwoViewsSparseReconstructor reconstructor, 
                    List<Sample2D> samples1, List<Sample2D> samples2, 
                    int viewId1, int viewId2, List<MatchedSamples> matches) { }

            @Override
            public void onFundamentalMatrixEstimated(
                    TwoViewsSparseReconstructor reconstructor, 
                    EstimatedFundamentalMatrix estimatedFundamentalMatrix) { }

            @Override
            public void onCamerasEstimated(
                    TwoViewsSparseReconstructor reconstructor, int viewId1, 
                    int viewId2, EstimatedCamera camera1, 
                    EstimatedCamera camera2) { }

            @Override
            public void onReconstructedPointsEstimated(
                    TwoViewsSparseReconstructor reconstructor, 
                    List<MatchedSamples> matches, 
                    List<ReconstructedPoint3D> points) { }

            @Override
            public void onStart(TwoViewsSparseReconstructor reconstructor) { }

            @Override
            public void onFinish(TwoViewsSparseReconstructor reconstructor) { }

            @Override
            public void onCancel(TwoViewsSparseReconstructor reconstructor) { }

            @Override
            public void onFail(TwoViewsSparseReconstructor reconstructor) { }
        };
                
        //constructor with listener
        TwoViewsSparseReconstructor reconstructor = 
                new TwoViewsSparseReconstructor(listener);
        
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
        reconstructor = new TwoViewsSparseReconstructor(configuration, 
                listener);
        
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
    public void testGeneralPointsEssential() 
            throws InvalidPairOfCamerasException, CameraException,
            NotReadyException, NotAvailableException, AlgebraException {
        
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            TwoViewsSparseReconstructorConfiguration configuration =
                    new TwoViewsSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL, 
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPoint = 0.0;
        
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, 
                    focalLength, principalPoint, principalPoint, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);
        
            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);
        
            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL, 
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
        
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
        
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, 
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, 
                    betaEuler2, gammaEuler2);
        
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
        
            TwoViewsSparseReconstructorListener listener =
                    new TwoViewsSparseReconstructorListener() {
                                        
                @Override
                public boolean hasMoreViewsAvailable(
                        TwoViewsSparseReconstructor reconstructor) {
                    return mViewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) {
                
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
                        TwoViewsSparseReconstructor reconstructor, int viewId,
                        List<Sample2D> samples) {
                    mViewCount++;
                }

                @Override
                public void onSamplesRejected(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) { }

                @Override
                public void onRequestMatches(
                        TwoViewsSparseReconstructor reconstructor, 
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
                        TwoViewsSparseReconstructor reconstructor, 
                        EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        TwoViewsSparseReconstructor reconstructor, int viewId1, 
                        int viewId2, EstimatedCamera camera1, 
                        EstimatedCamera camera2) {
                    mEstimatedCamera1 = camera1;
                    mEstimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        TwoViewsSparseReconstructor reconstructor, 
                        List<MatchedSamples> matches, 
                        List<ReconstructedPoint3D> points) {
                    mReconstructedPoints = points;
                }

                @Override
                public void onStart(TwoViewsSparseReconstructor reconstructor) {
                    mStarted = true;
                }

                @Override
                public void onFinish(
                        TwoViewsSparseReconstructor reconstructor) {
                    mFinished = true;
                }

                @Override
                public void onCancel(
                        TwoViewsSparseReconstructor reconstructor) {
                    mCancelled = true;
                }

                @Override
                public void onFail(TwoViewsSparseReconstructor reconstructor) {
                    mFailed = true;
                }
            };

            TwoViewsSparseReconstructor reconstructor = 
                    new TwoViewsSparseReconstructor(configuration, listener);
        
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
        
            //check that reconstructed points are in a metric stratum (up to a
            //certain scale)
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

            //transform points and cameras to account for scale change
            double baseline = center1.distanceTo(center2);
            double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            double scale = baseline / estimatedBaseline;
        
            MetricTransformation3D scaleTransformation = 
                    new MetricTransformation3D(scale);
        
            PinholeCamera scaledCamera1 = 
                    scaleTransformation.transformAndReturnNew(estimatedCamera1);
            PinholeCamera scaledCamera2 = 
                    scaleTransformation.transformAndReturnNew(estimatedCamera2);
        
            List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(reconstructedPoints3D);
        
            scaledCamera1.decompose();
            scaledCamera2.decompose();
        
            Point3D scaledCenter1 = scaledCamera1.getCameraCenter();
            Point3D scaledCenter2 = scaledCamera2.getCameraCenter();
        
            PinholeCameraIntrinsicParameters scaledIntrinsic1 = 
                    scaledCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters scaledIntrinsic2 =
                    scaledCamera2.getIntrinsicParameters();
        
            Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
            Rotation3D scaledRotation2 = scaledCamera2.getCameraRotation();
        
            double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);
        
            //check cameras are correct
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
        
            //check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {                
                if (!points3D.get(i).equals(
                        scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(
                        scaledReconstructionPoints3D.get(i), 
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
    public void testGeneralPointsDIAC() throws InvalidPairOfCamerasException,
            CameraException, NotReadyException, NotAvailableException, AlgebraException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            TwoViewsSparseReconstructorConfiguration configuration =
                    new TwoViewsSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);
            
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH_DIAC, 
                    MAX_FOCAL_LENGTH_DIAC);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPointX = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);
            double principalPointY = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                    focalLength, principalPointX, principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);
            
            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);
            configuration.setInitialCamerasAspectRatio(aspectRatio);
            
            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_DIAC, MAX_CAMERA_SEPARATION_DIAC);
            
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            
            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
            
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, 
                    camera2);
            
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
            boolean maxTriesReached = false;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
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
            
            if (maxTriesReached) {
                continue;
            }
            
            TwoViewsSparseReconstructorListener listener =
                    new TwoViewsSparseReconstructorListener() {
                        
                @Override
                public boolean hasMoreViewsAvailable(
                        TwoViewsSparseReconstructor reconstructor) {
                    return mViewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) {
                    
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
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) {
                    mViewCount++;
                }

                @Override
                public void onSamplesRejected(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) { }

                @Override
                public void onRequestMatches(
                        TwoViewsSparseReconstructor reconstructor, 
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
                        TwoViewsSparseReconstructor reconstructor, 
                        EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        TwoViewsSparseReconstructor reconstructor, int viewId1, 
                        int viewId2, EstimatedCamera camera1, 
                        EstimatedCamera camera2) {
                    mEstimatedCamera1 = camera1;
                    mEstimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        TwoViewsSparseReconstructor reconstructor, 
                        List<MatchedSamples> matches, 
                        List<ReconstructedPoint3D> points) {
                    mReconstructedPoints = points;
                }

                @Override
                public void onStart(TwoViewsSparseReconstructor reconstructor) {
                    mStarted = true;
                }

                @Override
                public void onFinish(
                        TwoViewsSparseReconstructor reconstructor) {
                    mFinished = true;
                }

                @Override
                public void onCancel(
                        TwoViewsSparseReconstructor reconstructor) {
                    mCancelled = true;
                }

                @Override
                public void onFail(TwoViewsSparseReconstructor reconstructor) {
                    mFailed = true;
                }
            };
            
            TwoViewsSparseReconstructor reconstructor =
                    new TwoViewsSparseReconstructor(configuration, listener);
            
            //check initial values
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
            
            //check correctness
            assertTrue(mStarted);
            //noinspection all
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
            
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
            
            //check that reconstructed points are in a metric stratum (up to a
            //certain scale)
            PinholeCamera estimatedCamera1 = mEstimatedCamera1.getCamera();
            PinholeCamera estimatedCamera2 = mEstimatedCamera2.getCamera();
            
            estimatedCamera1.decompose();
            estimatedCamera2.decompose();
            
            List<Point3D> reconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                reconstructedPoints3D.add(
                        mReconstructedPoints.get(i).getPoint());
            }
            
            //check that most of the points are in front of both cameras
            int valid = 0, invalid = 0;
            for (int i = 0; i < numPoints; i++) {                
                if (mReconstructedPoints.get(i).isInlier()) {
                    Point3D p = reconstructedPoints3D.get(i);
                    assertTrue(estimatedCamera1.isPointInFrontOfCamera(p));
                    assertTrue(estimatedCamera2.isPointInFrontOfCamera(p));
                    valid++;
                } else {
                    invalid++;
                }
            }
            
            assertTrue(valid >= invalid);
            
            Point3D estimatedCenter1 = estimatedCamera1.getCameraCenter();
            Point3D estimatedCenter2 = estimatedCamera2.getCameraCenter();
            
            //transform points and cameras to account for scale change
            double baseline = center1.distanceTo(center2);
            double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            double scale = baseline / estimatedBaseline;
            
            MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);
            
            PinholeCamera scaledCamera1 =
                    scaleTransformation.transformAndReturnNew(estimatedCamera1);
            PinholeCamera scaledCamera2 =
                    scaleTransformation.transformAndReturnNew(estimatedCamera2);
            
            List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(reconstructedPoints3D);
            
            scaledCamera1.decompose();
            scaledCamera2.decompose();
            
            Point3D scaledCenter1 = new InhomogeneousPoint3D(
                    scaledCamera1.getCameraCenter());
            Point3D scaledCenter2 = new InhomogeneousPoint3D(
                    scaledCamera2.getCameraCenter());
            
            PinholeCameraIntrinsicParameters scaledIntrinsic1 =
                    scaledCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters scaledIntrinsic2 =
                    scaledCamera2.getIntrinsicParameters();
            
            Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
            Rotation3D scaledRotation2 = scaledCamera2.getCameraRotation();
            
            double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);
            
            //check cameras are correct
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
        
            //check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {                
                if (!points3D.get(i).equals(
                        scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(
                        scaledReconstructionPoints3D.get(i), 
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
    public void testGeneralPointsDAQAndEssentialZeroPrincipalPoint() 
            throws InvalidPairOfCamerasException,
            CameraException, NotReadyException, NotAvailableException, AlgebraException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            TwoViewsSparseReconstructorConfiguration configuration =
                    new TwoViewsSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.
                            DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);
            
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL, 
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPointX = 0.0;
            double principalPointY = 0.0;
            
            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, 
                    focalLength, principalPointX, principalPointY, skewness);
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
            
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL, 
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            
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
            boolean leftFront, rightFront;
            boolean maxTriesReached = false;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
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
                
                //check that world point is in front of both cameras
                //noinspection all
                assertTrue(leftFront);
                //noinspection all
                assertTrue(rightFront);
                
                //project world point into both cameras
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
            
            TwoViewsSparseReconstructorListener listener =
                    new TwoViewsSparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(
                        TwoViewsSparseReconstructor reconstructor) {
                    return mViewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) {
                    
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
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) {
                    mViewCount++;
                }

                @Override
                public void onSamplesRejected(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) { }

                @Override
                public void onRequestMatches(
                        TwoViewsSparseReconstructor reconstructor, 
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
                        TwoViewsSparseReconstructor reconstructor, 
                        EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        TwoViewsSparseReconstructor reconstructor, int viewId1, 
                        int viewId2, EstimatedCamera camera1, 
                        EstimatedCamera camera2) {
                    mEstimatedCamera1 = camera1;
                    mEstimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        TwoViewsSparseReconstructor reconstructor, 
                        List<MatchedSamples> matches, 
                        List<ReconstructedPoint3D> points) {
                    mReconstructedPoints = points;
                }

                @Override
                public void onStart(TwoViewsSparseReconstructor reconstructor) {
                    mStarted = true;
                }

                @Override
                public void onFinish(
                        TwoViewsSparseReconstructor reconstructor) {
                    mFinished = true;
                }

                @Override
                public void onCancel(
                        TwoViewsSparseReconstructor reconstructor) {
                    mCancelled = true;
                }

                @Override
                public void onFail(TwoViewsSparseReconstructor reconstructor) {
                    mFailed = true;
                }
            };
            
            TwoViewsSparseReconstructor reconstructor =
                    new TwoViewsSparseReconstructor(configuration, listener);
            
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
            
            //check that reconstructed points are in a metric stratum (up to a
            //certain scale)
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

            //transform points and cameras to account for scale change
            double baseline = center1.distanceTo(center2);
            double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            double scale = baseline / estimatedBaseline;
        
            MetricTransformation3D scaleTransformation = 
                    new MetricTransformation3D(scale);
        
            PinholeCamera scaledCamera1 = 
                    scaleTransformation.transformAndReturnNew(estimatedCamera1);
            PinholeCamera scaledCamera2 = 
                    scaleTransformation.transformAndReturnNew(estimatedCamera2);
        
            List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(reconstructedPoints3D);
        
            scaledCamera1.decompose();
            scaledCamera2.decompose();
        
            Point3D scaledCenter1 = scaledCamera1.getCameraCenter();
            Point3D scaledCenter2 = scaledCamera2.getCameraCenter();
            
            PinholeCameraIntrinsicParameters scaledIntrinsic1 = 
                    scaledCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters scaledIntrinsic2 =
                    scaledCamera2.getIntrinsicParameters();
        
            Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
            Rotation3D scaledRotation2 = scaledCamera2.getCameraRotation();
        
            double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);
        
            //check cameras are correct
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
        
            //check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {                
                if (!points3D.get(i).equals(
                        scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(
                        scaledReconstructionPoints3D.get(i), 
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
    public void testGeneralPointsDAQAndEssential() 
            throws InvalidPairOfCamerasException,
            CameraException, NotReadyException, NotAvailableException, AlgebraException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            TwoViewsSparseReconstructorConfiguration configuration =
                    new TwoViewsSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.
                            DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);
            
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL, 
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPointX = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_ESSENTIAL, 
                    MAX_PRINCIPAL_POINT_ESSENTIAL);
            double principalPointY = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_ESSENTIAL, 
                    MAX_PRINCIPAL_POINT_ESSENTIAL);
            
            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, 
                    focalLength, principalPointX, principalPointY, skewness);
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
            
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL, 
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
            
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            
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
            boolean leftFront, rightFront;
            boolean maxTriesReached = false;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
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
                } while(!leftFront || !rightFront);
                
                if (maxTriesReached) {
                    break;
                }
                
                points3D.add(point3D);
                
                //check that world point is in front of both cameras
                //noinspection all
                assertTrue(leftFront);
                //noinspection all
                assertTrue(rightFront);
                
                //project world point into both cameras
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
            
            TwoViewsSparseReconstructorListener listener =
                    new TwoViewsSparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(
                        TwoViewsSparseReconstructor reconstructor) {
                    return mViewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) {
                    
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
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) {
                    mViewCount++;
                }

                @Override
                public void onSamplesRejected(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) { }

                @Override
                public void onRequestMatches(
                        TwoViewsSparseReconstructor reconstructor, 
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
                        TwoViewsSparseReconstructor reconstructor, 
                        EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        TwoViewsSparseReconstructor reconstructor, int viewId1, 
                        int viewId2, EstimatedCamera camera1, 
                        EstimatedCamera camera2) {
                    mEstimatedCamera1 = camera1;
                    mEstimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        TwoViewsSparseReconstructor reconstructor, 
                        List<MatchedSamples> matches, 
                        List<ReconstructedPoint3D> points) {
                    mReconstructedPoints = points;
                }

                @Override
                public void onStart(TwoViewsSparseReconstructor reconstructor) {
                    mStarted = true;
                }

                @Override
                public void onFinish(
                        TwoViewsSparseReconstructor reconstructor) {
                    mFinished = true;
                }

                @Override
                public void onCancel(
                        TwoViewsSparseReconstructor reconstructor) {
                    mCancelled = true;
                }

                @Override
                public void onFail(TwoViewsSparseReconstructor reconstructor) {
                    mFailed = true;
                }
            };
            
            TwoViewsSparseReconstructor reconstructor =
                    new TwoViewsSparseReconstructor(configuration, listener);
            
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
            
            //check that reconstructed points are in a metric stratum (up to a
            //certain scale)
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

            //transform points and cameras to account for scale change
            double baseline = center1.distanceTo(center2);
            double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            double scale = baseline / estimatedBaseline;
        
            MetricTransformation3D scaleTransformation = 
                    new MetricTransformation3D(scale);
        
            PinholeCamera scaledCamera1 = 
                    scaleTransformation.transformAndReturnNew(estimatedCamera1);
            PinholeCamera scaledCamera2 = 
                    scaleTransformation.transformAndReturnNew(estimatedCamera2);
        
            List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(reconstructedPoints3D);
        
            scaledCamera1.decompose();
            scaledCamera2.decompose();
        
            Point3D scaledCenter1 = scaledCamera1.getCameraCenter();
            Point3D scaledCenter2 = scaledCamera2.getCameraCenter();
            
            PinholeCameraIntrinsicParameters scaledIntrinsic1 = 
                    scaledCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters scaledIntrinsic2 =
                    scaledCamera2.getIntrinsicParameters();
        
            Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
            Rotation3D scaledRotation2 = scaledCamera2.getCameraRotation();
        
            double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);
        
            //check cameras are correct
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
        
            //check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {                
                if (!points3D.get(i).equals(
                        scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(
                        scaledReconstructionPoints3D.get(i), 
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
    public void testGeneralPointsDAQ() throws InvalidPairOfCamerasException,
            CameraException, NotReadyException, NotAvailableException, LockedException,
            RobustEstimatorException, AlgebraException {
        
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            TwoViewsSparseReconstructorConfiguration configuration =
                    new TwoViewsSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);
            
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL, 
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPointX = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_ESSENTIAL, 
                    MAX_PRINCIPAL_POINT_ESSENTIAL);
            double principalPointY = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_ESSENTIAL, 
                    MAX_PRINCIPAL_POINT_ESSENTIAL);
            
            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, 
                    focalLength, principalPointX, principalPointY, skewness);
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
            
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL, 
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
            
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);            
            
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            
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
            List<Point3D> points3D = new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean leftFront, rightFront;
            boolean maxTriesReached = false;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
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
                
                //check that world point is in front of both cameras
                //noinspection all
                assertTrue(leftFront);
                //noinspection all
                assertTrue(rightFront);
                
                //project world point into both cameras
                projectedPoint1 = new InhomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoints1.add(projectedPoint1);
                
                projectedPoint2 = new InhomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoints2.add(projectedPoint2);
            }
            
            TwoViewsSparseReconstructorListener listener =
                    new TwoViewsSparseReconstructorListener() {
                @Override
                public boolean hasMoreViewsAvailable(
                        TwoViewsSparseReconstructor reconstructor) {
                    return mViewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) {
                    
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
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) {
                    mViewCount++;
                }

                @Override
                public void onSamplesRejected(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) { }

                @Override
                public void onRequestMatches(
                        TwoViewsSparseReconstructor reconstructor, 
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
                        TwoViewsSparseReconstructor reconstructor, 
                        EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        TwoViewsSparseReconstructor reconstructor, int viewId1, 
                        int viewId2, EstimatedCamera camera1, 
                        EstimatedCamera camera2) {
                    mEstimatedCamera1 = camera1;
                    mEstimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        TwoViewsSparseReconstructor reconstructor, 
                        List<MatchedSamples> matches, 
                        List<ReconstructedPoint3D> points) {
                    mReconstructedPoints = points;
                }

                @Override
                public void onStart(TwoViewsSparseReconstructor reconstructor) {
                    mStarted = true;
                }

                @Override
                public void onFinish(
                        TwoViewsSparseReconstructor reconstructor) {
                    mFinished = true;
                }

                @Override
                public void onCancel(
                        TwoViewsSparseReconstructor reconstructor) {
                    mCancelled = true;
                }

                @Override
                public void onFail(TwoViewsSparseReconstructor reconstructor) {
                    mFailed = true;
                }
            };
            
            if (maxTriesReached) {
                continue;
            }
            
            TwoViewsSparseReconstructor reconstructor =
                    new TwoViewsSparseReconstructor(configuration, listener);
            
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

            //check that reconstructed points are in a metric stratum (up to a
            //certain scale, rotation and translation)
            PinholeCamera estimatedCamera1 = mEstimatedCamera1.getCamera();
            PinholeCamera estimatedCamera2 = mEstimatedCamera2.getCamera();
            
            List<Point3D> reconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < points3D.size(); i++) {
                reconstructedPoints3D.add(mReconstructedPoints.get(i).
                        getPoint());
            }        
                                                
            MetricTransformation3DRobustEstimator transformationEstimator =
                    MetricTransformation3DRobustEstimator.create(
                            reconstructedPoints3D, points3D,
                            RobustEstimatorMethod.LMedS);
            
            MetricTransformation3D transformation = 
                    transformationEstimator.estimate();
            
            PinholeCamera transformedCamera1 = 
                    transformation.transformAndReturnNew(estimatedCamera1);
            PinholeCamera transformedCamera2 =
                    transformation.transformAndReturnNew(estimatedCamera2);
            
            //check cameras intrinsics are correct (rotation, center and points
            //might contain large errors and for that reason we do not checked)
            transformedCamera1.decompose();
            transformedCamera2.decompose();
            
            PinholeCameraIntrinsicParameters transformedIntrinsic1 =
                    transformedCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters transformedIntrinsic2 =
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
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }
        
    @Test
    public void testPlanarPointsEssential() 
            throws InvalidPairOfCamerasException,
            CameraException, NotReadyException, NotAvailableException, AlgebraException {
        
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            TwoViewsSparseReconstructorConfiguration configuration =
                    new TwoViewsSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX);
        
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL, 
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPointX = 0.0;
            double principalPointY = 0.0;
                    
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, 
                    focalLength, principalPointX, principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);
            
            configuration.setInitialIntrinsic1(intrinsic);
            configuration.setInitialIntrinsic2(intrinsic);
            
            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL, 
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
        
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
        
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, 
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, 
                    betaEuler2, gammaEuler2);
        
            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
        
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);
        
            //create 3D points laying in front of both cameras and laying in
            //a plane
        
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
            
            double[] principalAxis1 = camera1.getPrincipalAxisArray();
            double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double[] avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2), 
                    0.5);
            
            Plane plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();
        
            double planeA = plane.getA();
            double planeB = plane.getB();
            double planeC = plane.getC();
            double planeD = plane.getD();
            
            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS, 
                    MAX_NUM_POINTS);
        
            HomogeneousPoint3D point3D;
            List<HomogeneousPoint3D> points3D = 
                    new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1, front2;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {                
                    //get a random point belonging to the plane
                    //a*x + b*y + c*z + d*w = 0
                    //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    double homX, homY;
                    double homW = 1.0;
                    double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
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
        
            TwoViewsSparseReconstructorListener listener =
                    new TwoViewsSparseReconstructorListener() {
                                        
                @Override
                public boolean hasMoreViewsAvailable(
                        TwoViewsSparseReconstructor reconstructor) {
                    return mViewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) {
                
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
                        TwoViewsSparseReconstructor reconstructor, int viewId,
                        List<Sample2D> samples) {
                    mViewCount++;
                }

                @Override
                public void onSamplesRejected(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) { 
                    mViewCount++;
                }

                @Override
                public void onRequestMatches(
                        TwoViewsSparseReconstructor reconstructor, 
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
                        TwoViewsSparseReconstructor reconstructor, 
                        EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        TwoViewsSparseReconstructor reconstructor, int viewId1, 
                        int viewId2, EstimatedCamera camera1, 
                        EstimatedCamera camera2) {
                    mEstimatedCamera1 = camera1;
                    mEstimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        TwoViewsSparseReconstructor reconstructor, 
                        List<MatchedSamples> matches, 
                        List<ReconstructedPoint3D> points) {
                    mReconstructedPoints = points;
                }

                @Override
                public void onStart(TwoViewsSparseReconstructor reconstructor) {
                    mStarted = true;
                }

                @Override
                public void onFinish(
                        TwoViewsSparseReconstructor reconstructor) {
                    mFinished = true;
                }

                @Override
                public void onCancel(
                        TwoViewsSparseReconstructor reconstructor) {
                    mCancelled = true;
                }

                @Override
                public void onFail(TwoViewsSparseReconstructor reconstructor) {
                    mFailed = true;
                }
            };

            TwoViewsSparseReconstructor reconstructor = 
                    new TwoViewsSparseReconstructor(configuration, listener);
        
            //check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());
        
            reconstructor.start();
        
            //check correctness
            if (!mFinished) {
                continue;
            }
            assertTrue(mStarted);
            //noinspection all
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
        
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
        
            //check that reconstructed points are in a metric stratum (up to a
            //certain scale)
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

            //transform points and cameras to account for scale change
            double baseline = center1.distanceTo(center2);
            double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            double scale = baseline / estimatedBaseline;
        
            MetricTransformation3D scaleTransformation = 
                    new MetricTransformation3D(scale);
        
            PinholeCamera scaledCamera1 = 
                    scaleTransformation.transformAndReturnNew(estimatedCamera1);
            PinholeCamera scaledCamera2 = 
                    scaleTransformation.transformAndReturnNew(estimatedCamera2);
        
            List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(reconstructedPoints3D);
        
            scaledCamera1.decompose();
            scaledCamera2.decompose();
        
            Point3D scaledCenter1 = scaledCamera1.getCameraCenter();
            Point3D scaledCenter2 = scaledCamera2.getCameraCenter();
        
            PinholeCameraIntrinsicParameters scaledIntrinsic1 = 
                    scaledCamera1.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters scaledIntrinsic2 =
                    scaledCamera2.getIntrinsicParameters();
        
            Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
            Rotation3D scaledRotation2 = scaledCamera2.getCameraRotation();
        
            double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);
        
            //check cameras are correct
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
            
            //check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {                
                if (!points3D.get(i).equals(
                        scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(
                        scaledReconstructionPoints3D.get(i), 
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
    public void testPlanarPointsDIAC() throws InvalidPairOfCamerasException,
            CameraException, NotReadyException, NotAvailableException, AlgebraException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            TwoViewsSparseReconstructorConfiguration configuration =
                    new TwoViewsSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);
        
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL, 
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPointX = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);
            double principalPointY = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_DIAC, MAX_PRINCIPAL_POINT_DIAC);
                    
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, 
                    focalLength, principalPointX, principalPointY, skewness);
            intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);
            
            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);
            configuration.setInitialCamerasAspectRatio(aspectRatio);
            
            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL, 
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
        
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
        
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, 
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, 
                    betaEuler2, gammaEuler2);
        
            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
        
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);
        
            //create 3D points laying in front of both cameras and laying in
            //a plane
        
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
            
            double[] principalAxis1 = camera1.getPrincipalAxisArray();
            double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double[] avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2), 
                    0.5);
            
            Plane plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();
        
            double planeA = plane.getA();
            double planeB = plane.getB();
            double planeC = plane.getC();
            double planeD = plane.getD();
            
            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS, 
                    MAX_NUM_POINTS);
        
            HomogeneousPoint3D point3D;
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1, front2;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {                
                    //get a random point belonging to the plane
                    //a*x + b*y + c*z + d*w = 0
                    //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    double homX, homY;
                    double homW = 1.0;
                    double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
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
        
            TwoViewsSparseReconstructorListener listener =
                    new TwoViewsSparseReconstructorListener() {
                                        
                @Override
                public boolean hasMoreViewsAvailable(
                        TwoViewsSparseReconstructor reconstructor) {
                    return mViewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) {
                
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
                        TwoViewsSparseReconstructor reconstructor, int viewId,
                        List<Sample2D> samples) {
                    mViewCount++;
                }

                @Override
                public void onSamplesRejected(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) { 
                    mViewCount++;
                }

                @Override
                public void onRequestMatches(
                        TwoViewsSparseReconstructor reconstructor, 
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
                        TwoViewsSparseReconstructor reconstructor, 
                        EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        TwoViewsSparseReconstructor reconstructor, int viewId1, 
                        int viewId2, EstimatedCamera camera1, 
                        EstimatedCamera camera2) {
                    mEstimatedCamera1 = camera1;
                    mEstimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        TwoViewsSparseReconstructor reconstructor, 
                        List<MatchedSamples> matches, 
                        List<ReconstructedPoint3D> points) {
                    mReconstructedPoints = points;
                }

                @Override
                public void onStart(TwoViewsSparseReconstructor reconstructor) {
                    mStarted = true;
                }

                @Override
                public void onFinish(
                        TwoViewsSparseReconstructor reconstructor) {
                    mFinished = true;
                }

                @Override
                public void onCancel(
                        TwoViewsSparseReconstructor reconstructor) {
                    mCancelled = true;
                }

                @Override
                public void onFail(TwoViewsSparseReconstructor reconstructor) {
                    mFailed = true;
                }
            };

            TwoViewsSparseReconstructor reconstructor = 
                    new TwoViewsSparseReconstructor(configuration, listener);
        
            //check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());
        
            reconstructor.start();
        
            //check correctness
            if (!mFinished) {
                continue;
            }
            assertTrue(mStarted);
            //noinspection all
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
        
            //check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();
        
            //matrices are equal up to scale
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
        
            //check that reconstructed points are in a metric stratum (up to a
            //certain scale)
            PinholeCamera estimatedCamera1 = mEstimatedCamera1.getCamera();
            PinholeCamera estimatedCamera2 = mEstimatedCamera2.getCamera();
        
            estimatedCamera1.decompose();
            estimatedCamera2.decompose();
        
            List<Point3D> reconstructedPoints3D = new ArrayList<>();
            for (int i = 0; i < numPoints; i++) {
                reconstructedPoints3D.add(
                        mReconstructedPoints.get(i).getPoint());
            }
        
            //check that most of the points are in front of both cameras
            int valid = 0, invalid = 0;
            for (int i = 0; i < numPoints; i++) {
                if (mReconstructedPoints.get(i).isInlier()) {
                    Point3D p = reconstructedPoints3D.get(i);
                    assertTrue(estimatedCamera1.isPointInFrontOfCamera(p));
                    assertTrue(estimatedCamera2.isPointInFrontOfCamera(p));
                    valid++;
                } else {
                    invalid++;
                }
            }   

            if (valid < invalid) {
                continue;
            }
            //noinspection all
            assertTrue(valid >= invalid);
        
            Point3D estimatedCenter1 = estimatedCamera1.getCameraCenter();
            Point3D estimatedCenter2 = estimatedCamera2.getCameraCenter();

            //transform points and cameras to account for scale change
            double baseline = center1.distanceTo(center2);
            double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            double scale = baseline / estimatedBaseline;
        
            MetricTransformation3D scaleTransformation = 
                    new MetricTransformation3D(scale);
        
            PinholeCamera scaledCamera1 = 
                    scaleTransformation.transformAndReturnNew(estimatedCamera1);
            PinholeCamera scaledCamera2 = 
                    scaleTransformation.transformAndReturnNew(estimatedCamera2);
                
            scaledCamera1.decompose();
            scaledCamera2.decompose();
        
            Point3D scaledCenter1 = new InhomogeneousPoint3D(
                    scaledCamera1.getCameraCenter());
            Point3D scaledCenter2 = new InhomogeneousPoint3D(
                    scaledCamera2.getCameraCenter());
                
            Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
        
            double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);
        
            //check cameras are correct
            if(Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);
                
            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
                        
            numValid++;
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);       
    }
    
    @Test
    public void testPlanarPointsDAQAndEssentialZeroPrincipalPoint() 
            throws InvalidPairOfCamerasException,
            CameraException, NotReadyException, NotAvailableException, AlgebraException {
        
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            TwoViewsSparseReconstructorConfiguration configuration =
                    new TwoViewsSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.
                            DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);
        
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL, 
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPointX = 0.0;
            double principalPointY = 0.0;
                    
            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, 
                    focalLength, principalPointX, principalPointY, skewness);
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
                
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL, 
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
        
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
        
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, 
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, 
                    betaEuler2, gammaEuler2);
        
            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
        
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);
        
            //create 3D points laying in front of both cameras and laying in
            //a plane
        
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
            
            double[] principalAxis1 = camera1.getPrincipalAxisArray();
            double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double[] avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2), 
                    0.5);
            
            Plane plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();
        
            double planeA = plane.getA();
            double planeB = plane.getB();
            double planeC = plane.getC();
            double planeD = plane.getD();
            
            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS, 
                    MAX_NUM_POINTS);
        
            HomogeneousPoint3D point3D;
            List<HomogeneousPoint3D> points3D = 
                    new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1, front2;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {                
                    //get a random point belonging to the plane
                    //a*x + b*y + c*z + d*w = 0
                    //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    double homX, homY;
                    double homW = 1.0;
                    double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
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
        
            TwoViewsSparseReconstructorListener listener =
                    new TwoViewsSparseReconstructorListener() {
                                        
                @Override
                public boolean hasMoreViewsAvailable(
                        TwoViewsSparseReconstructor reconstructor) {
                    return mViewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) {
                
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
                        TwoViewsSparseReconstructor reconstructor, int viewId,
                        List<Sample2D> samples) {
                    mViewCount++;
                }

                @Override
                public void onSamplesRejected(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) { 
                    mViewCount++;
                }

                @Override
                public void onRequestMatches(
                        TwoViewsSparseReconstructor reconstructor, 
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
                        TwoViewsSparseReconstructor reconstructor, 
                        EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        TwoViewsSparseReconstructor reconstructor, int viewId1, 
                        int viewId2, EstimatedCamera camera1, 
                        EstimatedCamera camera2) {
                    mEstimatedCamera1 = camera1;
                    mEstimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        TwoViewsSparseReconstructor reconstructor, 
                        List<MatchedSamples> matches, 
                        List<ReconstructedPoint3D> points) {
                    mReconstructedPoints = points;
                }

                @Override
                public void onStart(TwoViewsSparseReconstructor reconstructor) {
                    mStarted = true;
                }

                @Override
                public void onFinish(
                        TwoViewsSparseReconstructor reconstructor) {
                    mFinished = true;
                }

                @Override
                public void onCancel(
                        TwoViewsSparseReconstructor reconstructor) {
                    mCancelled = true;
                }

                @Override
                public void onFail(TwoViewsSparseReconstructor reconstructor) {
                    mFailed = true;
                }
            };

            TwoViewsSparseReconstructor reconstructor = 
                    new TwoViewsSparseReconstructor(configuration, listener);
        
            //check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());
        
            reconstructor.start();
        
            //check correctness
            if (!mFinished) {
                continue;
            }
            assertTrue(mStarted);
            //noinspection all
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
        
            //check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();
        
            //matrices are equal up to scale
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
        
            //check that reconstructed points are in a metric stratum (up to a
            //certain scale)
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

            //transform points and cameras to account for scale change
            double baseline = center1.distanceTo(center2);
            double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            double scale = baseline / estimatedBaseline;
        
            MetricTransformation3D scaleTransformation = 
                    new MetricTransformation3D(scale);
        
            PinholeCamera scaledCamera1 = 
                    scaleTransformation.transformAndReturnNew(estimatedCamera1);
            PinholeCamera scaledCamera2 = 
                    scaleTransformation.transformAndReturnNew(estimatedCamera2);
        
            List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(reconstructedPoints3D);
        
            scaledCamera1.decompose();
            scaledCamera2.decompose();
        
            Point3D scaledCenter1 = scaledCamera1.getCameraCenter();
            Point3D scaledCenter2 = scaledCamera2.getCameraCenter();
                
            Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
        
            double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);
        
            //check cameras are correct
            if (Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);
        
            assertTrue(center1.equals(scaledCenter1, LARGE_ABSOLUTE_ERROR));        
                    
            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
                        
            //check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {                
                if (!points3D.get(i).equals(
                        scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(
                        scaledReconstructionPoints3D.get(i), 
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
    public void testPlanarPointsDAQAndEssential() 
            throws InvalidPairOfCamerasException,
            CameraException, NotReadyException, NotAvailableException, AlgebraException {
        
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            TwoViewsSparseReconstructorConfiguration configuration =
                    new TwoViewsSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.
                            DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX);
        
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL, 
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPointX = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_ESSENTIAL, 
                    MAX_PRINCIPAL_POINT_ESSENTIAL);
            double principalPointY = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_ESSENTIAL, 
                    MAX_PRINCIPAL_POINT_ESSENTIAL);
                    
            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);
            
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, 
                    focalLength, principalPointX, principalPointY, skewness);
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
                
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL, 
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
        
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
        
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, 
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, 
                    betaEuler2, gammaEuler2);
        
            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
        
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);
        
            //create 3D points laying in front of both cameras and laying in
            //a plane
        
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
            
            double[] principalAxis1 = camera1.getPrincipalAxisArray();
            double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double[] avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2), 
                    0.5);
            
            Plane plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();
        
            double planeA = plane.getA();
            double planeB = plane.getB();
            double planeC = plane.getC();
            double planeD = plane.getD();
            
            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS, 
                    MAX_NUM_POINTS);
        
            HomogeneousPoint3D point3D;
            List<HomogeneousPoint3D> points3D = new ArrayList<>();
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1, front2, failed = false;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {                
                    //get a random point belonging to the plane
                    //a*x + b*y + c*z + d*w = 0
                    //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    double homX, homY;
                    double homW = 1.0;
                    double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
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

            if (failed) {
                continue;
            }
        
            TwoViewsSparseReconstructorListener listener =
                    new TwoViewsSparseReconstructorListener() {
                                        
                @Override
                public boolean hasMoreViewsAvailable(
                        TwoViewsSparseReconstructor reconstructor) {
                    return mViewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) {
                
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
                        TwoViewsSparseReconstructor reconstructor, int viewId,
                        List<Sample2D> samples) {
                    mViewCount++;
                }

                @Override
                public void onSamplesRejected(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) { 
                    mViewCount++;
                }

                @Override
                public void onRequestMatches(
                        TwoViewsSparseReconstructor reconstructor, 
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
                        TwoViewsSparseReconstructor reconstructor, 
                        EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        TwoViewsSparseReconstructor reconstructor, int viewId1, 
                        int viewId2, EstimatedCamera camera1, 
                        EstimatedCamera camera2) {
                    mEstimatedCamera1 = camera1;
                    mEstimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        TwoViewsSparseReconstructor reconstructor, 
                        List<MatchedSamples> matches, 
                        List<ReconstructedPoint3D> points) {
                    mReconstructedPoints = points;
                }

                @Override
                public void onStart(TwoViewsSparseReconstructor reconstructor) {
                    mStarted = true;
                }

                @Override
                public void onFinish(
                        TwoViewsSparseReconstructor reconstructor) {
                    mFinished = true;
                }

                @Override
                public void onCancel(
                        TwoViewsSparseReconstructor reconstructor) {
                    mCancelled = true;
                }

                @Override
                public void onFail(TwoViewsSparseReconstructor reconstructor) {
                    mFailed = true;
                }
            };

            TwoViewsSparseReconstructor reconstructor = 
                    new TwoViewsSparseReconstructor(configuration, listener);
        
            //check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());
        
            reconstructor.start();
        
            //check correctness
            if (!mFinished) {
                continue;
            }
            assertTrue(mStarted);
            //noinspection all
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
        
            //check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();
        
            //matrices are equal up to scale
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
        
            //check that reconstructed points are in a metric stratum (up to a
            //certain scale)
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

            //transform points and cameras to account for scale change
            double baseline = center1.distanceTo(center2);
            double estimatedBaseline = estimatedCenter1.distanceTo(
                    estimatedCenter2);
            double scale = baseline / estimatedBaseline;
        
            MetricTransformation3D scaleTransformation = 
                    new MetricTransformation3D(scale);
        
            PinholeCamera scaledCamera1 = 
                    scaleTransformation.transformAndReturnNew(estimatedCamera1);
            PinholeCamera scaledCamera2 = 
                    scaleTransformation.transformAndReturnNew(estimatedCamera2);
        
            List<Point3D> scaledReconstructionPoints3D = scaleTransformation.
                    transformPointsAndReturnNew(reconstructedPoints3D);
        
            scaledCamera1.decompose();
            scaledCamera2.decompose();
        
            Point3D scaledCenter1 = scaledCamera1.getCameraCenter();
            Point3D scaledCenter2 = scaledCamera2.getCameraCenter();
                
            Rotation3D scaledRotation1 = scaledCamera1.getCameraRotation();
        
            double scaledBaseline = scaledCenter1.distanceTo(scaledCenter2);
        
            //check cameras are correct
            if (Math.abs(scaledBaseline - baseline) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(scaledBaseline, baseline, LARGE_ABSOLUTE_ERROR);        
                    
            assertTrue(scaledRotation1.asInhomogeneousMatrix().equals(
                    rotation1.asInhomogeneousMatrix(), ABSOLUTE_ERROR));
                        
            //check that points are correct
            boolean validPoints = true;
            for (int i = 0; i < numPoints; i++) {                
                if (!points3D.get(i).equals(
                        scaledReconstructionPoints3D.get(i), LARGE_ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(points3D.get(i).equals(
                        scaledReconstructionPoints3D.get(i), 
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
    public void testPlanarPointsDAQ() 
            throws InvalidPairOfCamerasException,
            CameraException, NotReadyException, NotAvailableException, AlgebraException {
        
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            TwoViewsSparseReconstructorConfiguration configuration =
                    new TwoViewsSparseReconstructorConfiguration();
            configuration.setInitialCamerasEstimatorMethod(
                    InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);
        
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double focalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH_ESSENTIAL, 
                    MAX_FOCAL_LENGTH_ESSENTIAL);
            double aspectRatio = configuration.getInitialCamerasAspectRatio();
            double skewness = 0.0;
            double principalPointX = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_ESSENTIAL, 
                    MAX_PRINCIPAL_POINT_ESSENTIAL);
            double principalPointY = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT_ESSENTIAL, 
                    MAX_PRINCIPAL_POINT_ESSENTIAL);
            
            configuration.setPrincipalPointX(principalPointX);
            configuration.setPrincipalPointY(principalPointY);            
                    
            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength, 
                    focalLength, principalPointX, principalPointY, skewness);
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
        
        
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION_ESSENTIAL, 
                    MAX_CAMERA_SEPARATION_ESSENTIAL);
        
            Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);
        
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, 
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, 
                    betaEuler2, gammaEuler2);
        
            PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);
        
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    camera1, camera2);
        
            //create 3D points laying in front of both cameras and laying in
            //a plane
        
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
            
            double[] principalAxis1 = camera1.getPrincipalAxisArray();
            double[] principalAxis2 = camera2.getPrincipalAxisArray();
            double[] avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2), 
                    0.5);
            
            Plane plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();
        
            double planeA = plane.getA();
            double planeB = plane.getB();
            double planeC = plane.getC();
            double planeD = plane.getD();
            
            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS, 
                    MAX_NUM_POINTS);
        
            HomogeneousPoint3D point3D;
            Point2D projectedPoint1, projectedPoint2;
            final List<Point2D> projectedPoints1 = new ArrayList<>();
            final List<Point2D> projectedPoints2 = new ArrayList<>();
            boolean front1, front2;
            boolean failed = false;
            for (int i = 0; i < numPoints; i++) {
                //generate points and ensure they lie in front of both cameras
                int numTry = 0;
                do {                
                    //get a random point belonging to the plane
                    //a*x + b*y + c*z + d*w = 0
                    //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                    double homX, homY;
                    double homW = 1.0;
                    double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
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

            if (failed) {
                continue;
            }
        
            TwoViewsSparseReconstructorListener listener =
                    new TwoViewsSparseReconstructorListener() {
                                        
                @Override
                public boolean hasMoreViewsAvailable(
                        TwoViewsSparseReconstructor reconstructor) {
                    return mViewCount < 2;
                }

                @Override
                public void onRequestSamplesForCurrentView(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) {
                
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
                        TwoViewsSparseReconstructor reconstructor, int viewId,
                        List<Sample2D> samples) {
                    mViewCount++;
                }

                @Override
                public void onSamplesRejected(
                        TwoViewsSparseReconstructor reconstructor, int viewId, 
                        List<Sample2D> samples) { 
                    mViewCount++;
                }

                @Override
                public void onRequestMatches(
                        TwoViewsSparseReconstructor reconstructor, 
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
                        TwoViewsSparseReconstructor reconstructor, 
                        EstimatedFundamentalMatrix estimatedFundamentalMatrix) {
                    mEstimatedFundamentalMatrix = estimatedFundamentalMatrix;
                }

                @Override
                public void onCamerasEstimated(
                        TwoViewsSparseReconstructor reconstructor, int viewId1, 
                        int viewId2, EstimatedCamera camera1, 
                        EstimatedCamera camera2) {
                    mEstimatedCamera1 = camera1;
                    mEstimatedCamera2 = camera2;
                }

                @Override
                public void onReconstructedPointsEstimated(
                        TwoViewsSparseReconstructor reconstructor, 
                        List<MatchedSamples> matches, 
                        List<ReconstructedPoint3D> points) {
                    mReconstructedPoints = points;
                }

                @Override
                public void onStart(TwoViewsSparseReconstructor reconstructor) {
                    mStarted = true;
                }

                @Override
                public void onFinish(
                        TwoViewsSparseReconstructor reconstructor) {
                    mFinished = true;
                }

                @Override
                public void onCancel(
                        TwoViewsSparseReconstructor reconstructor) {
                    mCancelled = true;
                }

                @Override
                public void onFail(TwoViewsSparseReconstructor reconstructor) {
                    mFailed = true;
                }
            };

            TwoViewsSparseReconstructor reconstructor = 
                    new TwoViewsSparseReconstructor(configuration, listener);
        
            //check initial values
            reset();
            assertFalse(mStarted);
            assertFalse(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertFalse(reconstructor.isFinished());
        
            reconstructor.start();
        
            //check correctness
            if (!mFinished) {
                continue;
            }
            assertTrue(mStarted);
            //noinspection all
            assertTrue(mFinished);
            assertFalse(mCancelled);
            assertFalse(mFailed);
            assertTrue(reconstructor.isFinished());
        
            //check that estimated fundamental matrix is correct
            fundamentalMatrix.normalize();
            mEstimatedFundamentalMatrix.getFundamentalMatrix().normalize();
        
            //matrices are equal up to scale
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
        
            //check that reconstructed points are in a metric stratum (up to a
            //certain scale)
            PinholeCamera estimatedCamera1 = mEstimatedCamera1.getCamera();
            PinholeCamera estimatedCamera2 = mEstimatedCamera2.getCamera();
        
            estimatedCamera1.decompose();
            estimatedCamera2.decompose();
        
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
    }
}
