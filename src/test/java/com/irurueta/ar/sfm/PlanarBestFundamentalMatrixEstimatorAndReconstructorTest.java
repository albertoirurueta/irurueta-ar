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
import com.irurueta.ar.epipolar.Corrector;
import com.irurueta.ar.epipolar.CorrectorType;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidPairOfCamerasException;
import com.irurueta.ar.epipolar.estimators.FundamentalMatrixEstimatorException;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.PointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PlanarBestFundamentalMatrixEstimatorAndReconstructorTest implements
        PlanarBestFundamentalMatrixEstimatorAndReconstructorListener {

    private static final double MIN_FOCAL_LENGTH = 750.0;
    private static final double MAX_FOCAL_LENGTH = 1500.0;

    private static final double MIN_ANGLE_DEGREES = -30.0;
    private static final double MAX_ANGLE_DEGREES = -15.0;

    private static final double MIN_CAMERA_SEPARATION = 500.0;
    private static final double MAX_CAMERA_SEPARATION = 1000.0;

    private static final int MIN_NUM_POINTS = 25;
    private static final int MAX_NUM_POINTS = 50;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_RANDOM_VALUE_PLANAR = -1500.0;
    private static final double MAX_RANDOM_VALUE_PLANAR = 1500.0;

    private static final int TIMES = 500;
    private static final int MAX_TRIES = 5000;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;

    private int mEstimateStart;
    private int mEstimateEnd;

    @Test
    public void testConstructor() {
        // empty constructor
        PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        // check default values
        assertNull(er.getLeftPoints());
        assertNull(er.getRightPoints());
        assertNull(er.getLeftIntrinsics());
        assertNull(er.getRightIntrinsics());
        assertNotNull(er.getHomographyEstimator());
        assertEquals(Corrector.DEFAULT_TYPE, er.getEssentialCameraEstimatorCorrectorType());
        assertEquals(er.getHomographyEstimator().getConfidence(), er.getHomographyConfidence(), 0.0);
        assertEquals(er.getHomographyEstimator().getMaxIterations(), er.getHomographyMaxIterations());
        assertEquals(er.getHomographyEstimator().isResultRefined(), er.isHomographyRefined());
        assertEquals(er.getHomographyEstimator().isCovarianceKept(), er.isHomographyCovarianceKept());
        assertNull(er.getHomographyCovariance());
        assertEquals(er.getHomographyEstimator().getMethod(), er.getHomographyMethod());
        assertNull(er.getQualityScores());
        assertNull(er.getListener());
        assertFalse(er.isLocked());
        assertFalse(er.isReady());
        assertNull(er.getFundamentalMatrix());
        assertNull(er.getTriangulatedPoints());
        assertNull(er.getValidTriangulatedPoints());
        assertNull(er.getEstimatedLeftCamera());
        assertNull(er.getEstimatedRightCamera());

        // constructor with points and intrinsics
        final List<Point2D> leftPoints = new ArrayList<>();
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());

        final List<Point2D> rightPoints = new ArrayList<>();
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());

        final PinholeCameraIntrinsicParameters leftIntrinsics = new PinholeCameraIntrinsicParameters();
        final PinholeCameraIntrinsicParameters rightIntrinsics = new PinholeCameraIntrinsicParameters();

        er = new PlanarBestFundamentalMatrixEstimatorAndReconstructor(
                leftPoints, rightPoints, leftIntrinsics, rightIntrinsics);

        // check correctness
        assertSame(leftPoints, er.getLeftPoints());
        assertSame(rightPoints, er.getRightPoints());
        assertSame(leftIntrinsics, er.getLeftIntrinsics());
        assertSame(rightIntrinsics, er.getRightIntrinsics());
        assertNotNull(er.getHomographyEstimator());
        assertEquals(Corrector.DEFAULT_TYPE, er.getEssentialCameraEstimatorCorrectorType());
        assertEquals(er.getHomographyEstimator().getConfidence(), er.getHomographyConfidence(), 0.0);
        assertEquals(er.getHomographyEstimator().getMaxIterations(), er.getHomographyMaxIterations());
        assertEquals(er.getHomographyEstimator().isResultRefined(), er.isHomographyRefined());
        assertEquals(er.getHomographyEstimator().isCovarianceKept(), er.isHomographyCovarianceKept());
        assertNull(er.getHomographyCovariance());
        assertEquals(er.getHomographyEstimator().getMethod(), er.getHomographyMethod());
        assertNotNull(er.getQualityScores());
        assertNull(er.getListener());
        assertFalse(er.isLocked());
        assertTrue(er.isReady());
        assertNull(er.getFundamentalMatrix());
        assertNull(er.getTriangulatedPoints());
        assertNull(er.getValidTriangulatedPoints());
        assertNull(er.getEstimatedLeftCamera());
        assertNull(er.getEstimatedRightCamera());

        // Force IllegalArgumentException
        final List<Point2D> wrong = new ArrayList<>();

        er = null;
        try {
            er = new PlanarBestFundamentalMatrixEstimatorAndReconstructor(wrong,
                    rightPoints, leftIntrinsics, rightIntrinsics);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            er = new PlanarBestFundamentalMatrixEstimatorAndReconstructor(
                    leftPoints, wrong, leftIntrinsics, rightIntrinsics);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            er = new PlanarBestFundamentalMatrixEstimatorAndReconstructor(wrong,
                    wrong, leftIntrinsics, rightIntrinsics);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(er);

        // constructor with points, intrinsics and listener
        er = new PlanarBestFundamentalMatrixEstimatorAndReconstructor(
                leftPoints, rightPoints, leftIntrinsics, rightIntrinsics, this);

        // check correctness
        assertSame(leftPoints, er.getLeftPoints());
        assertSame(rightPoints, er.getRightPoints());
        assertSame(leftIntrinsics, er.getLeftIntrinsics());
        assertSame(rightIntrinsics, er.getRightIntrinsics());
        assertNotNull(er.getHomographyEstimator());
        assertEquals(Corrector.DEFAULT_TYPE, er.getEssentialCameraEstimatorCorrectorType());
        assertEquals(er.getHomographyEstimator().getConfidence(), er.getHomographyConfidence(), 0.0);
        assertEquals(er.getHomographyEstimator().getMaxIterations(), er.getHomographyMaxIterations());
        assertEquals(er.getHomographyEstimator().isResultRefined(), er.isHomographyRefined());
        assertEquals(er.getHomographyEstimator().isCovarianceKept(), er.isHomographyCovarianceKept());
        assertNull(er.getHomographyCovariance());
        assertEquals(er.getHomographyEstimator().getMethod(), er.getHomographyMethod());
        assertNotNull(er.getQualityScores());
        assertSame(this, er.getListener());
        assertFalse(er.isLocked());
        assertTrue(er.isReady());
        assertNull(er.getFundamentalMatrix());
        assertNull(er.getTriangulatedPoints());
        assertNull(er.getValidTriangulatedPoints());
        assertNull(er.getEstimatedLeftCamera());
        assertNull(er.getEstimatedRightCamera());

        // Force IllegalArgumentException
        er = null;
        try {
            er = new PlanarBestFundamentalMatrixEstimatorAndReconstructor(wrong,
                    rightPoints, leftIntrinsics, rightIntrinsics, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            er = new PlanarBestFundamentalMatrixEstimatorAndReconstructor(
                    leftPoints, wrong, leftIntrinsics, rightIntrinsics, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            er = new PlanarBestFundamentalMatrixEstimatorAndReconstructor(wrong,
                    wrong, leftIntrinsics, rightIntrinsics, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(er);
    }

    @Test
    public void testGetSetLeftPoints() throws LockedException {
        final PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        // check default value
        assertNull(er.getLeftPoints());

        // set new value
        final List<Point2D> leftPoints = new ArrayList<>();
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());

        er.setLeftPoints(leftPoints);

        // check correctness
        assertSame(leftPoints, er.getLeftPoints());

        // Force IllegalArgumentException
        final List<Point2D> wrong = new ArrayList<>();
        try {
            er.setLeftPoints(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetRightPoints() throws LockedException {
        final PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        // check default value
        assertNull(er.getRightPoints());

        // set new value
        final List<Point2D> rightPoints = new ArrayList<>();
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());

        er.setRightPoints(rightPoints);

        // check correctness
        assertSame(rightPoints, er.getRightPoints());

        // Force IllegalArgumentException
        final List<Point2D> wrong = new ArrayList<>();
        try {
            er.setRightPoints(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSetLeftAndRightPoints() throws LockedException {
        final PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        // check default values
        assertNull(er.getLeftPoints());
        assertNull(er.getRightPoints());

        // set new value
        final List<Point2D> leftPoints = new ArrayList<>();
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());
        leftPoints.add(Point2D.create());

        final List<Point2D> rightPoints = new ArrayList<>();
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());
        rightPoints.add(Point2D.create());

        er.setLeftAndRightPoints(leftPoints, rightPoints);

        // check correctness
        assertSame(leftPoints, er.getLeftPoints());
        assertSame(rightPoints, er.getRightPoints());
        assertSame(leftPoints, er.getHomographyEstimator().getInputPoints());
        assertSame(rightPoints, er.getHomographyEstimator().getOutputPoints());

        // Force IllegalArgumentException
        final List<Point2D> wrong = new ArrayList<>();
        try {
            er.setLeftAndRightPoints(wrong, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            er.setLeftAndRightPoints(leftPoints, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            er.setLeftAndRightPoints(wrong, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        rightPoints.add(Point2D.create());
        try {
            er.setLeftAndRightPoints(leftPoints, rightPoints);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetLeftIntrinsics() throws LockedException {
        final PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        // initial value
        assertNull(er.getLeftIntrinsics());

        // set new value
        final PinholeCameraIntrinsicParameters leftIntrinsics = new PinholeCameraIntrinsicParameters();
        er.setLeftIntrinsics(leftIntrinsics);

        // check correctness
        assertSame(leftIntrinsics, er.getLeftIntrinsics());
    }

    @Test
    public void testGetSetRightIntrinsics() throws LockedException {
        final PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        // initial value
        assertNull(er.getRightIntrinsics());

        // set new value
        final PinholeCameraIntrinsicParameters rightIntrinsics = new PinholeCameraIntrinsicParameters();
        er.setRightIntrinsics(rightIntrinsics);

        // check correctness
        assertSame(rightIntrinsics, er.getRightIntrinsics());
    }

    @Test
    public void testGetSetHomographyEstimator() throws LockedException {
        final PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        // initial value
        assertNotNull(er.getHomographyEstimator());

        // set new value
        final PointCorrespondenceProjectiveTransformation2DRobustEstimator homographyEstimator =
                PointCorrespondenceProjectiveTransformation2DRobustEstimator.create();
        er.setHomographyEstimator(homographyEstimator);

        // check correctness
        assertSame(homographyEstimator, er.getHomographyEstimator());

        // Force NullPointerException
        try {
            er.setHomographyEstimator(null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
    }

    @Test
    public void testGetSetEssentialCameraEstimatorCorrectorType()
            throws LockedException {
        final PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        // check initial value
        assertEquals(Corrector.DEFAULT_TYPE, er.getEssentialCameraEstimatorCorrectorType());

        // set new value
        er.setEssentialCameraEstimatorCorrectorType(CorrectorType.GOLD_STANDARD);

        // check correctness
        assertEquals(CorrectorType.GOLD_STANDARD, er.getEssentialCameraEstimatorCorrectorType());
    }

    @Test
    public void testGetSetHomographyConfidence() throws LockedException {
        final PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        // check default value
        assertEquals(er.getHomographyEstimator().getConfidence(), er.getHomographyConfidence(), 0.0);

        // set new value
        er.setHomographyConfidence(0.5);

        // check correctness
        assertEquals(0.5, er.getHomographyConfidence(), 0.0);
        assertEquals(0.5, er.getHomographyEstimator().getConfidence(), 0.0);
    }

    @Test
    public void testGetSetHomographyMaxIterations() throws LockedException {
        final PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        // check default value
        assertEquals(er.getHomographyEstimator().getMaxIterations(), er.getHomographyMaxIterations());

        // set new value
        er.setHomographyMaxIterations(10);

        // check correctness
        assertEquals(10, er.getHomographyMaxIterations());
        assertEquals(10, er.getHomographyEstimator().getMaxIterations());
    }

    @Test
    public void testIsSetHomographyRefined() throws LockedException {
        final PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        // check default value
        final boolean refined = er.isHomographyRefined();
        assertEquals(er.getHomographyEstimator().isResultRefined(), er.isHomographyRefined());

        // set new value
        er.setHomographyRefined(!refined);

        // check correctness
        assertEquals(!refined, er.isHomographyRefined());
        assertEquals(!refined, er.getHomographyEstimator().isResultRefined());
    }

    @Test
    public void testIsSetHomographyCovarianceKept() throws LockedException {
        final PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        // check default value
        final boolean covarianceKept = er.isHomographyCovarianceKept();
        assertEquals(er.getHomographyEstimator().isCovarianceKept(), er.isHomographyCovarianceKept());

        // set new value
        er.setHomographyCovarianceKept(!covarianceKept);

        // check correctness
        assertEquals(!covarianceKept, er.isHomographyCovarianceKept());
        assertEquals(!covarianceKept, er.getHomographyEstimator().isCovarianceKept());
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        // check default value
        assertNull(er.getQualityScores());

        // set new estimator
        final PointCorrespondenceProjectiveTransformation2DRobustEstimator homographyEstimator =
                PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(
                        RobustEstimatorMethod.PROSAC);
        er.setHomographyEstimator(homographyEstimator);
        assertNull(er.getQualityScores());

        // set new value
        final double[] qualityScores = new double[
                PlanarBestFundamentalMatrixEstimatorAndReconstructor.MINIMUM_SIZE];
        er.setQualityScores(qualityScores);

        // check correctness
        assertSame(qualityScores, er.getQualityScores());

        // Force IllegalArgumentException
        final double[] wrong = new double[1];
        try {
            er.setQualityScores(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() {
        final PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                new PlanarBestFundamentalMatrixEstimatorAndReconstructor();

        // check default value
        assertNull(er.getListener());

        // set new value
        er.setListener(this);

        // check correctness
        assertSame(this, er.getListener());
    }

    @Test
    public void testEstimateAndReconstruct()
            throws InvalidPairOfCamerasException, AlgebraException,
            CameraException, LockedException, NotReadyException,
            NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double focalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double focalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double principalPointX = 0.0;
            final double principalPointY = 0.0;

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(focalLength1, focalLength1,
                            principalPointX, principalPointY, skewness);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(focalLength2, focalLength2,
                            principalPointX, principalPointY, skewness);

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
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // create 3D points laying in front of both cameras and laying in
            // a plane

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

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(planesIntersectionMatrix);
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
                    ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2), 0.5);

            final Plane plane = new Plane(centralCommonPoint, avgPrincipalAxis);
            plane.normalize();

            final double planeA = plane.getA();
            final double planeB = plane.getB();
            final double planeC = plane.getC();
            final double planeD = plane.getD();

            final int numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

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
                    final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

                // check that 3D point is in front of both cameras
                assertTrue(front1);
                assertTrue(front2);

                // project 3D point into both cameras
                projectedPoint1 = new HomogeneousPoint2D();
                camera1.project(point3D, projectedPoint1);
                projectedPoint1.normalize();

                projectedPoints1.add(projectedPoint1);

                projectedPoint2 = new HomogeneousPoint2D();
                camera2.project(point3D, projectedPoint2);
                projectedPoint2.normalize();

                projectedPoints2.add(projectedPoint2);
            }

            final PlanarBestFundamentalMatrixEstimatorAndReconstructor er =
                    new PlanarBestFundamentalMatrixEstimatorAndReconstructor(
                            projectedPoints1, projectedPoints2, intrinsic1, intrinsic2, this);

            reset();
            assertEquals(0, mEstimateStart);
            assertEquals(0, mEstimateEnd);
            assertFalse(er.isLocked());
            assertTrue(er.isReady());

            try {
                er.estimateAndReconstruct();
            } catch (final FundamentalMatrixEstimatorException e) {
                continue;
            }

            // check correctness
            assertEquals(1, mEstimateStart);
            assertEquals(1, mEstimateEnd);
            assertFalse(er.isLocked());

            // check correctness of homography
            boolean failed = false;
            InhomogeneousPoint2D point1;
            InhomogeneousPoint2D point2;
            InhomogeneousPoint2D point2b;
            for (int i = 0; i < numPoints; i++) {
                point1 = new InhomogeneousPoint2D(projectedPoints1.get(i));
                point2 = new InhomogeneousPoint2D(projectedPoints2.get(i));

                point2b = new InhomogeneousPoint2D();
                er.getHomography().transform(point1, point2b);

                if (!point2.equals(point2b, LARGE_ABSOLUTE_ERROR)) {
                    failed = true;
                    break;
                }
                assertTrue(point2.equals(point2b, LARGE_ABSOLUTE_ERROR));
            }

            if (failed) {
                continue;
            }

            // check that estimated fundamental matrix is equal up to scale to
            // the original one
            if (!areEqualUpToScale(fundamentalMatrix, er.getFundamentalMatrix())) {
                continue;
            }
            assertTrue(areEqualUpToScale(fundamentalMatrix, er.getFundamentalMatrix()));

            // check that triangulated points lie in front of both estimated
            // cameras
            // NOTE: points and cameras are reconstructed up to scale respect to
            // original ones
            final PinholeCamera camera1b = er.getEstimatedLeftCamera();
            final PinholeCamera camera2b = er.getEstimatedRightCamera();

            final List<Point3D> triangulatedPoints = er.getTriangulatedPoints();
            final BitSet validTriangulatedPoints = er.getValidTriangulatedPoints();
            for (int i = 0; i < validTriangulatedPoints.length(); i++) {
                final Point3D point = triangulatedPoints.get(i);

                assertTrue(validTriangulatedPoints.get(i));

                assertTrue(camera1b.isPointInFrontOfCamera(point));
                assertTrue(camera2b.isPointInFrontOfCamera(point));
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(
            final PlanarBestFundamentalMatrixEstimatorAndReconstructor estimatorAndReconstructor) {
        mEstimateStart++;
        checkLocked(estimatorAndReconstructor);
    }

    @Override
    public void onEstimateEnd(
            final PlanarBestFundamentalMatrixEstimatorAndReconstructor estimatorAndReconstructor) {
        mEstimateEnd++;
        checkLocked(estimatorAndReconstructor);
    }

    private static boolean areEqualUpToScale(
            final FundamentalMatrix fundamentalMatrix1,
            final FundamentalMatrix fundamentalMatrix2) throws NotAvailableException, NotReadyException {

        // normalize to increase accuracy
        fundamentalMatrix1.normalize();
        fundamentalMatrix2.normalize();

        final Matrix f1 = fundamentalMatrix1.getInternalMatrix();
        final Matrix f2a = fundamentalMatrix2.getInternalMatrix();
        final Matrix f2b = f2a.multiplyByScalarAndReturnNew(-1.0);

        return f1.equals(f2a, ABSOLUTE_ERROR) || f1.equals(f2b, ABSOLUTE_ERROR);
    }

    private void reset() {
        mEstimateStart = mEstimateEnd = 0;
    }

    private void checkLocked(
            final PlanarBestFundamentalMatrixEstimatorAndReconstructor estimatorAndReconstructor) {

        try {
            estimatorAndReconstructor.setLeftPoints(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimatorAndReconstructor.setRightPoints(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimatorAndReconstructor.setLeftAndRightPoints(null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimatorAndReconstructor.setLeftIntrinsics(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimatorAndReconstructor.setRightIntrinsics(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimatorAndReconstructor.setHomographyEstimator(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimatorAndReconstructor.setEssentialCameraEstimatorCorrectorType(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimatorAndReconstructor.setHomographyConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimatorAndReconstructor.setHomographyMaxIterations(10);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimatorAndReconstructor.setHomographyRefined(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimatorAndReconstructor.setHomographyCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimatorAndReconstructor.setQualityScores(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        assertTrue(estimatorAndReconstructor.isLocked());
    }
}
