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
import com.irurueta.ar.calibration.DualImageOfAbsoluteConic;
import com.irurueta.ar.calibration.estimators.KruppaDualImageOfAbsoluteConicEstimator;
import com.irurueta.ar.calibration.estimators.KruppaDualImageOfAbsoluteConicEstimatorException;
import com.irurueta.ar.epipolar.Corrector;
import com.irurueta.ar.epipolar.CorrectorType;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidFundamentalMatrixException;
import com.irurueta.ar.epipolar.InvalidPairOfCamerasException;
import com.irurueta.geometry.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.BitSet;

import static org.junit.jupiter.api.Assertions.*;

class DualImageOfAbsoluteConicInitialCamerasEstimatorTest implements InitialCamerasEstimatorListener {

    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;

    private static final double MIN_ANGLE_DEGREES_KRUPPA = -90.0;
    private static final double MAX_ANGLE_DEGREES_KRUPPA = -30.0;

    private static final double MIN_PRINCIPAL_POINT = 10.0;
    private static final double MAX_PRINCIPAL_POINT = 20.0;

    private static final double MIN_CAMERA_SEPARATION = 5.0;
    private static final double MAX_CAMERA_SEPARATION = 10.0;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-3;

    private static final int MIN_POINTS = 50;
    private static final int MAX_POINTS = 100;

    private static final double MIN_LAMBDA = 100.0;
    private static final double MAX_LAMBDA = 500.0;

    private static final int TIMES = 50;
    private static final int MAX_TRIES = 2000;

    @Test
    void testConstructor() {
        var estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator();

        // check default values
        assertNull(estimator.getFundamentalMatrix());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC, estimator.getMethod());
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        final var fundamentalMatrix = new FundamentalMatrix();
        estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator(fundamentalMatrix);

        // check default values
        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC, estimator.getMethod());
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        final var leftPoints = new ArrayList<Point2D>();
        leftPoints.add(Point2D.create());
        final var rightPoints = new ArrayList<Point2D>();
        rightPoints.add(Point2D.create());
        estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator(leftPoints, rightPoints);

        // check default values
        assertNull(estimator.getFundamentalMatrix());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC, estimator.getMethod());
        assertFalse(estimator.isReady());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new DualImageOfAbsoluteConicInitialCamerasEstimator(leftPoints, null));
        assertThrows(IllegalArgumentException.class,
                () -> new DualImageOfAbsoluteConicInitialCamerasEstimator(null, rightPoints));

        estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator(fundamentalMatrix, leftPoints, rightPoints);

        // check default values
        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC, estimator.getMethod());
        assertTrue(estimator.isReady());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new DualImageOfAbsoluteConicInitialCamerasEstimator(
                fundamentalMatrix, leftPoints, null));
        assertThrows(IllegalArgumentException.class, () -> new DualImageOfAbsoluteConicInitialCamerasEstimator(
                fundamentalMatrix, null, rightPoints));

        estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator(this);

        // check default values
        assertNull(estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC, estimator.getMethod());
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator(fundamentalMatrix, this);

        // check default values
        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC, estimator.getMethod());
        assertFalse(estimator.isReady());
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator(leftPoints, rightPoints, this);

        // check default values
        assertNull(estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC, estimator.getMethod());
        assertFalse(estimator.isReady());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new DualImageOfAbsoluteConicInitialCamerasEstimator(
                leftPoints, null, this));
        assertThrows(IllegalArgumentException.class, () -> new DualImageOfAbsoluteConicInitialCamerasEstimator(
                null, rightPoints, this));

        estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator(fundamentalMatrix, leftPoints, rightPoints,
                this);

        // check default values
        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
        assertSame(this, estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC, estimator.getMethod());
        assertTrue(estimator.isReady());
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
        assertEquals(EssentialMatrixInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
        assertNull(estimator.getTriangulatedPoints());
        assertNull(estimator.getValidTriangulatedPoints());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new DualImageOfAbsoluteConicInitialCamerasEstimator(
                fundamentalMatrix, leftPoints, null, this));
        assertThrows(IllegalArgumentException.class, () -> new DualImageOfAbsoluteConicInitialCamerasEstimator(
                fundamentalMatrix, null, rightPoints, this));
    }

    @Test
    void testGetSetFundamentalMatrix() throws com.irurueta.geometry.estimators.LockedException {
        final var estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator();

        // check default value
        assertNull(estimator.getFundamentalMatrix());

        // set new value
        final var fundamentalMatrix = new FundamentalMatrix();
        estimator.setFundamentalMatrix(fundamentalMatrix);

        // check correctness
        assertSame(fundamentalMatrix, estimator.getFundamentalMatrix());
    }

    @Test
    void testGetSetListener() {
        final var estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetAspectRatio() throws com.irurueta.geometry.estimators.LockedException {
        final var estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator();

        // check default value
        assertEquals(1.0, estimator.getAspectRatio(), 0.0);

        // set new value
        estimator.setAspectRatio(-1.0);

        // check correctness
        assertEquals(-1.0, estimator.getAspectRatio(), 0.0);
    }

    @Test
    void testGetSetPrincipalPointX() throws com.irurueta.geometry.estimators.LockedException {
        final var estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator();

        // check default value
        assertEquals(0.0, estimator.getPrincipalPointX(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        estimator.setPrincipalPointX(principalPointX);

        // check correctness
        assertEquals(principalPointX, estimator.getPrincipalPointX(), 0.0);
    }

    @Test
    void testGetSetPrincipalPointY() throws com.irurueta.geometry.estimators.LockedException {
        final var estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator();

        // check default value
        assertEquals(0.0, estimator.getPrincipalPointY(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        estimator.setPrincipalPointY(principalPointY);

        // check correctness
        assertEquals(principalPointY, estimator.getPrincipalPointY(), 0.0);
    }

    @Test
    void testGetSetPrincipalPoint() throws com.irurueta.geometry.estimators.LockedException {
        final var estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator();

        // check default value
        assertEquals(0.0, estimator.getPrincipalPointX(), 0.0);
        assertEquals(0.0, estimator.getPrincipalPointY(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        estimator.setPrincipalPoint(principalPointX, principalPointY);

        // check correctness
        assertEquals(principalPointX, estimator.getPrincipalPointX(), 0.0);
        assertEquals(principalPointY, estimator.getPrincipalPointY(), 0.0);
    }

    @Test
    void testGetSetLeftPoints() throws com.irurueta.geometry.estimators.LockedException {
        final var estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator();

        // check default value
        assertNull(estimator.getLeftPoints());

        // set new value
        final var leftPoints = new ArrayList<Point2D>();
        estimator.setLeftPoints(leftPoints);

        // check correctness
        assertSame(leftPoints, estimator.getLeftPoints());
    }

    @Test
    void testGetSetRightPoints() throws com.irurueta.geometry.estimators.LockedException {
        final var estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator();

        // check default value
        assertNull(estimator.getRightPoints());

        // set new value
        final var rightPoints = new ArrayList<Point2D>();
        estimator.setRightPoints(rightPoints);

        // check correctness
        assertSame(rightPoints, estimator.getRightPoints());
    }

    @Test
    void testSetLeftAndRightPoints() throws com.irurueta.geometry.estimators.LockedException {
        final var estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator();

        // check default values
        assertNull(estimator.getLeftPoints());
        assertNull(estimator.getRightPoints());

        // set new values
        final var leftPoints = new ArrayList<Point2D>();
        leftPoints.add(Point2D.create());
        final var rightPoints = new ArrayList<Point2D>();
        rightPoints.add(Point2D.create());
        estimator.setLeftAndRightPoints(leftPoints, rightPoints);

        // check correctness
        assertSame(leftPoints, estimator.getLeftPoints());
        assertSame(rightPoints, estimator.getRightPoints());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setLeftAndRightPoints(leftPoints, null));
        assertThrows(IllegalArgumentException.class, () -> estimator.setLeftAndRightPoints(null, rightPoints));

        final var wrongPoints = new ArrayList<Point2D>();
        assertThrows(IllegalArgumentException.class, () -> estimator.setLeftAndRightPoints(leftPoints, wrongPoints));
    }

    @Test
    void testGetSetCorrectorType() throws com.irurueta.geometry.estimators.LockedException {
        final var estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator();

        // check default value
        assertEquals(Corrector.DEFAULT_TYPE, estimator.getCorrectorType());

        // set new value
        estimator.setCorrectorType(CorrectorType.GOLD_STANDARD);

        // check correctness
        assertEquals(CorrectorType.GOLD_STANDARD, estimator.getCorrectorType());
    }

    @Test
    void testAreSetPointsTriangulated() throws com.irurueta.geometry.estimators.LockedException {
        final var estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator();

        // check default value
        assertEquals(DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());

        // set new value
        estimator.setPointsTriangulated(!DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS);

        // check correctness
        assertEquals(!DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_TRIANGULATE_POINTS,
                estimator.arePointsTriangulated());
    }

    @Test
    void testAreValidTriangulatedPointsMarked() throws com.irurueta.geometry.estimators.LockedException {
        final var estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator();

        // check default value
        assertEquals(DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());

        // set new value
        estimator.setValidTriangulatedPointsMarked(
                !DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS);

        // check correctness
        assertEquals(!DualImageOfAbsoluteConicInitialCamerasEstimator.DEFAULT_MARK_VALID_TRIANGULATED_POINTS,
                estimator.areValidTriangulatedPointsMarked());
    }

    @Test
    void testEstimate() throws InvalidPairOfCamerasException, com.irurueta.geometry.estimators.LockedException,
            com.irurueta.geometry.estimators.NotReadyException, InvalidPinholeCameraIntrinsicParametersException,
            CameraException, InitialCamerasEstimationFailedException, com.irurueta.geometry.NotAvailableException,
            InvalidFundamentalMatrixException, AlgebraException {
        var numValidTimes = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));

            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var aspectRatio = 1.0;
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            final var diacEstimator = new KruppaDualImageOfAbsoluteConicEstimator(fundamentalMatrix);
            diacEstimator.setPrincipalPointX(principalPointX);
            diacEstimator.setPrincipalPointY(principalPointY);
            diacEstimator.setFocalDistanceAspectRatioKnown(true);
            diacEstimator.setFocalDistanceAspectRatio(aspectRatio);

            DualImageOfAbsoluteConic diac;
            try {
                diac = diacEstimator.estimate();
            } catch (final KruppaDualImageOfAbsoluteConicEstimatorException e) {
                continue;
            }
            final var intrinsic2 = diac.getIntrinsicParameters();

            if (Math.abs(intrinsic2.getHorizontalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(intrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);

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

            final var principalAxis1 = camera1.getPrincipalAxisArray();
            final var principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1;
            double lambda2;

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            InhomogeneousPoint3D worldPoint;
            final var worldPoints = new ArrayList<InhomogeneousPoint3D>();
            Point2D leftPoint;
            Point2D rightPoint;
            final var leftPoints = new ArrayList<Point2D>();
            final var rightPoints = new ArrayList<Point2D>();
            boolean leftFront;
            boolean rightFront;
            var failed = false;
            for (var i = 0; i < nPoints; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambda1 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);
                    lambda2 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);

                    worldPoint = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + principalAxis1[0] * lambda1
                                    + principalAxis2[0] * lambda2,
                            centralCommonPoint.getInhomY() + principalAxis1[1] * lambda1
                                    + principalAxis2[1] * lambda2,
                            centralCommonPoint.getInhomZ() + principalAxis1[2] * lambda1
                                    + principalAxis2[2] * lambda2);
                    leftFront = camera1.isPointInFrontOfCamera(worldPoint);
                    rightFront = camera2.isPointInFrontOfCamera(worldPoint);
                    if (numTry > MAX_TRIES) {
                        failed = true;
                        break;
                    }
                    numTry++;
                } while (!leftFront || !rightFront);

                if (failed) {
                    break;
                }

                worldPoints.add(worldPoint);

                // check that world point is in front of both cameras
                assertTrue(leftFront);
                assertTrue(rightFront);

                // project world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);

                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }

            if (failed) {
                continue;
            }

            var estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator(fundamentalMatrix, leftPoints,
                    rightPoints, this);
            estimator.setPrincipalPointX(principalPointX);
            estimator.setPrincipalPointY(principalPointY);
            estimator.setPointsTriangulated(true);
            estimator.setValidTriangulatedPointsMarked(true);

            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedLeftCamera());
            assertNull(estimator.getEstimatedRightCamera());
            assertNull(estimator.getTriangulatedPoints());
            assertNull(estimator.getValidTriangulatedPoints());

            estimator.estimate();

            final var camera1b = estimator.getEstimatedLeftCamera();
            final var camera2b = estimator.getEstimatedRightCamera();
            final var triangulatedPoints = estimator.getTriangulatedPoints();
            final var validTriangulatedPoints = estimator.getValidTriangulatedPoints();

            camera1b.decompose();
            camera2b.decompose();

            final var intrinsic1b = camera1b.getIntrinsicParameters();
            final var intrinsic2b = camera2b.getIntrinsicParameters();

            final var rotation1b = camera1b.getCameraRotation();
            final var rotation2b = camera2b.getCameraRotation();

            final var center1b = camera1b.getCameraCenter();
            final var center2b = camera2b.getCameraCenter();

            assertEquals(focalLength, intrinsic1b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic1b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic1b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic1b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(focalLength, intrinsic2b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic2b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic2b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic2b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            final var diffRotation = rotation2b.combineAndReturnNew(rotation1b.inverseRotationAndReturnNew());

            assertTrue(rotation1b.asInhomogeneousMatrix().equals(Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            assertNotNull(center1b);
            assertNotNull(center2b);

            // compute scale factor
            final var distanceA = center1.distanceTo(center2);
            final var distanceB = center1b.distanceTo(center2b);
            final var scaleFactor = distanceB / distanceA;
            final var invScaleFactor = distanceA / distanceB;

            // NOTE: distance between estimated cameras is always normalized
            assertEquals(1.0, distanceB, ABSOLUTE_ERROR);

            final var scaleTransformation = new MetricTransformation3D(scaleFactor);
            final var invScaleTransformation = scaleTransformation.inverseAndReturnNew();
            final var invScaleTransformation2 = new MetricTransformation3D(invScaleFactor);

            assertTrue(invScaleTransformation.asMatrix().equals(invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));

            // check that estimated cameras generate the same input fundamental
            // matrix
            final var fundamentalMatrixB = new FundamentalMatrix(camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar
            // geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final var epipole1 = camera1.project(center2);
            final var epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final var epipole1a = fundamentalMatrix.getLeftEpipole();
            final var epipole2a = fundamentalMatrix.getRightEpipole();

            final var epipole1b = fundamentalMatrixB.getLeftEpipole();
            final var epipole2b = fundamentalMatrixB.getRightEpipole();

            if (epipole1.distanceTo(epipole1a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1.distanceTo(epipole1a), ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2.distanceTo(epipole2a), ABSOLUTE_ERROR);

            if (epipole1.distanceTo(epipole1b) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1.distanceTo(epipole1b), ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2b) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2.distanceTo(epipole2b), LARGE_ABSOLUTE_ERROR);

            // generate epipolar lines
            Point3D scaledWorldPoint;
            Point3D triangulatedPoint;
            Point3D scaledTriangulatedPoint;
            var numValid1a = 0;
            var numValid2a = 0;
            var numValid1b = 0;
            var numValid2b = 0;
            var numValidEqual = 0;
            for (var i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);

                triangulatedPoint = triangulatedPoints.get(i);
                assertTrue(validTriangulatedPoints.get(i));

                final var line1a = fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                final var line2a = fundamentalMatrix.getRightEpipolarLine(leftPoint);

                final var line1b = fundamentalMatrixB.getLeftEpipolarLine(rightPoint);
                final var line2b = fundamentalMatrixB.getRightEpipolarLine(leftPoint);

                // check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));

                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));

                // back-project epipolar lines for each pair of cameras and check
                // that each pair of lines correspond to the same epipolar plane
                final var epipolarPlane1a = camera1.backProject(line1a);
                final var epipolarPlane2a = camera2.backProject(line2a);

                final var epipolarPlane1b = camera1b.backProject(line1b);
                final var epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

                // check that 3D point and both camera centers for each pair of
                // cameras belong to their corresponding epipolar plane
                if (epipolarPlane1a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid1a++;
                }
                assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));

                if (epipolarPlane2a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid2a++;
                }
                assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

                // notice that since estimated cameras have an arbitrary scale,
                // original world point doesn't need to lie on epipolar plane
                // because first a scale transformation needs to be done
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(worldPoint);
                if (epipolarPlane1a.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid1b++;
                }
                assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));

                if (epipolarPlane2b.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid2b++;
                }
                assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR));

                // recover scale in triangulated point
                scaledTriangulatedPoint = invScaleTransformation.transformAndReturnNew(triangulatedPoint);

                // check that triangulated point after recovering scale matches
                // original point
                if (worldPoint.equals(scaledTriangulatedPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidEqual++;
                }
            }

            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 && numValid2b > 0 && numValidEqual > 0) {
                numValidTimes++;
            }

            // recover scale of cameras by undoing their transformations
            final var camera1c = invScaleTransformation.transformAndReturnNew(camera1b);
            final var camera2c = invScaleTransformation.transformAndReturnNew(camera2b);

            // check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();

            final var camera1Matrix = camera1.getInternalMatrix();
            final var camera1cMatrix = camera1c.getInternalMatrix();
            if (!camera1Matrix.equals(camera1cMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera1Matrix.equals(camera1cMatrix, LARGE_ABSOLUTE_ERROR));

            final var camera2Matrix = camera2.getInternalMatrix();
            final var camera2cMatrix = camera2c.getInternalMatrix();
            if (camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR));

            // force NotReadyException
            estimator = new DualImageOfAbsoluteConicInitialCamerasEstimator();

            assertFalse(estimator.isReady());

            assertThrows(com.irurueta.geometry.estimators.NotReadyException.class, estimator::estimate);

            break;
        }

        assertTrue(numValidTimes > 0);
    }

    @Test
    void testGenerateInitialMetricCamerasUsingDIAC1() throws InvalidPairOfCamerasException, CameraException,
            InitialCamerasEstimationFailedException, com.irurueta.geometry.NotAvailableException,
            com.irurueta.geometry.estimators.NotReadyException, InvalidFundamentalMatrixException, AlgebraException,
            com.irurueta.geometry.estimators.LockedException, InvalidPinholeCameraIntrinsicParametersException {
        var numValidTimes = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));

            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var aspectRatio = 1.0;
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            final var diacEstimator = new KruppaDualImageOfAbsoluteConicEstimator(fundamentalMatrix);
            diacEstimator.setPrincipalPointX(principalPointX);
            diacEstimator.setPrincipalPointY(principalPointY);
            diacEstimator.setFocalDistanceAspectRatioKnown(true);
            diacEstimator.setFocalDistanceAspectRatio(aspectRatio);

            DualImageOfAbsoluteConic diac;
            try {
                diac = diacEstimator.estimate();
            } catch (final KruppaDualImageOfAbsoluteConicEstimatorException e) {
                continue;
            }
            final var intrinsic2 = diac.getIntrinsicParameters();

            if (Math.abs(intrinsic2.getHorizontalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(intrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);

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

            final var principalAxis1 = camera1.getPrincipalAxisArray();
            final var principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1;
            double lambda2;

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            InhomogeneousPoint3D worldPoint;
            final var worldPoints = new ArrayList<InhomogeneousPoint3D>();
            Point2D leftPoint;
            Point2D rightPoint;
            final var leftPoints = new ArrayList<Point2D>();
            final var rightPoints = new ArrayList<Point2D>();
            boolean leftFront;
            boolean rightFront;
            var failed = false;
            for (var i = 0; i < nPoints; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambda1 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);
                    lambda2 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);

                    worldPoint = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + principalAxis1[0] * lambda1
                                    + principalAxis2[0] * lambda2,
                            centralCommonPoint.getInhomY() + principalAxis1[1] * lambda1
                                    + principalAxis2[1] * lambda2,
                            centralCommonPoint.getInhomZ() + principalAxis1[2] * lambda1
                                    + principalAxis2[2] * lambda2);
                    leftFront = camera1.isPointInFrontOfCamera(worldPoint);
                    rightFront = camera2.isPointInFrontOfCamera(worldPoint);
                    if (numTry > MAX_TRIES) {
                        failed = true;
                        break;
                    }
                    numTry++;
                } while (!leftFront || !rightFront);

                if (failed) {
                    break;
                }

                worldPoints.add(worldPoint);

                // check that world point is in front of both cameras
                assertTrue(leftFront);
                assertTrue(rightFront);

                // project world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);

                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }

            if (failed) {
                continue;
            }

            final var camera1b = new PinholeCamera();
            final var camera2b = new PinholeCamera();
            final var numValid = DualImageOfAbsoluteConicInitialCamerasEstimator.generateInitialMetricCamerasUsingDIAC(
                    fundamentalMatrix, principalPointX, principalPointY, leftPoints, rightPoints, camera1b, camera2b);

            // check correctness
            assertEquals(numValid, nPoints);

            camera1b.decompose();
            camera2b.decompose();

            final var intrinsic1b = camera1b.getIntrinsicParameters();
            final var intrinsic2b = camera2b.getIntrinsicParameters();

            final var rotation1b = camera1b.getCameraRotation();
            final var rotation2b = camera2b.getCameraRotation();

            final var center1b = camera1b.getCameraCenter();
            final var center2b = camera2b.getCameraCenter();

            assertEquals(focalLength, intrinsic1b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic1b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic1b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic1b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(focalLength, intrinsic2b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic2b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic2b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic2b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            final var diffRotation = rotation2b.combineAndReturnNew(rotation1b.inverseRotationAndReturnNew());

            assertTrue(rotation1b.asInhomogeneousMatrix().equals(Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            assertNotNull(center1b);
            assertNotNull(center2b);

            // compute scale factor
            final var distanceA = center1.distanceTo(center2);
            final var distanceB = center1b.distanceTo(center2b);
            final var scaleFactor = distanceB / distanceA;
            final var invScaleFactor = distanceA / distanceB;

            // NOTE: distance between estimated cameras is always normalized
            assertEquals(1.0, distanceB, ABSOLUTE_ERROR);

            final var scaleTransformation = new MetricTransformation3D(scaleFactor);
            final var invScaleTransformation = scaleTransformation.inverseAndReturnNew();
            final var invScaleTransformation2 = new MetricTransformation3D(invScaleFactor);

            assertTrue(invScaleTransformation.asMatrix().equals(invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));


            // check that estimated cameras generate the same input fundamental
            // matrix
            final var fundamentalMatrixB = new FundamentalMatrix(camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar
            // geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final var epipole1 = camera1.project(center2);
            final var epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final var epipole1a = fundamentalMatrix.getLeftEpipole();
            final var epipole2a = fundamentalMatrix.getRightEpipole();

            final var epipole1b = fundamentalMatrixB.getLeftEpipole();
            final var epipole2b = fundamentalMatrixB.getRightEpipole();

            if (epipole1.distanceTo(epipole1a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1.distanceTo(epipole1a), ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2.distanceTo(epipole2a), ABSOLUTE_ERROR);

            if (epipole1.distanceTo(epipole1b) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1.distanceTo(epipole1b), ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2b) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2.distanceTo(epipole2b), LARGE_ABSOLUTE_ERROR);

            // generate epipolar lines
            Point3D scaledWorldPoint;
            var numValid1a = 0;
            var numValid2a = 0;
            var numValid1b = 0;
            var numValid2b = 0;
            for (var i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);

                final var line1a = fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                final var line2a = fundamentalMatrix.getRightEpipolarLine(leftPoint);

                final var line1b = fundamentalMatrixB.getLeftEpipolarLine(rightPoint);
                final var line2b = fundamentalMatrixB.getRightEpipolarLine(leftPoint);

                // check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));

                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));

                // back-project epipolar lines for each pair of cameras and check
                // that each pair of lines correspond to the same epipolar plane
                final var epipolarPlane1a = camera1.backProject(line1a);
                final var epipolarPlane2a = camera2.backProject(line2a);

                final var epipolarPlane1b = camera1b.backProject(line1b);
                final var epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

                // check that 3D point and both camera centers for each pair of
                // cameras belong to their corresponding epipolar plane
                if (epipolarPlane1a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid1a++;
                }
                assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));

                if (epipolarPlane2a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid2a++;
                }
                assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

                // notice that since estimated cameras have an arbitrary scale,
                // original world point doesn't need to lie on epipolar plane
                // because first a scale transformation needs to be done
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(worldPoint);
                if (epipolarPlane1a.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid1b++;
                }
                assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));

                if (epipolarPlane2b.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid2b++;
                }
                assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR));
            }

            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 && numValid2b > 0) {
                numValidTimes++;
            }

            // recover scale of cameras by undoing their transformations
            final var camera1c = invScaleTransformation.transformAndReturnNew(camera1b);
            final var camera2c = invScaleTransformation.transformAndReturnNew(camera2b);

            // check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();

            final var camera1Matrix = camera1.getInternalMatrix();
            final var camera1cMatrix = camera1c.getInternalMatrix();
            if (!camera1Matrix.equals(camera1cMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera1Matrix.equals(camera1cMatrix, LARGE_ABSOLUTE_ERROR));

            final var camera2Matrix = camera2.getInternalMatrix();
            final var camera2cMatrix = camera2c.getInternalMatrix();
            if (!camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR));

            if (numValidTimes > 0) {
                break;
            }
        }

        assertTrue(numValidTimes > 0);
    }

    @Test
    void testGenerateInitialMetricCamerasUsingDIAC2() throws InvalidPairOfCamerasException, CameraException,
            InitialCamerasEstimationFailedException, com.irurueta.geometry.NotAvailableException,
            com.irurueta.geometry.estimators.NotReadyException, InvalidFundamentalMatrixException, AlgebraException,
            com.irurueta.geometry.estimators.LockedException, InvalidPinholeCameraIntrinsicParametersException {
        var numValidTimes = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));

            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var aspectRatio = 1.0;
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            final var diacEstimator = new KruppaDualImageOfAbsoluteConicEstimator(fundamentalMatrix);
            diacEstimator.setPrincipalPointX(principalPointX);
            diacEstimator.setPrincipalPointY(principalPointY);
            diacEstimator.setFocalDistanceAspectRatioKnown(true);
            diacEstimator.setFocalDistanceAspectRatio(aspectRatio);

            DualImageOfAbsoluteConic diac;
            try {
                diac = diacEstimator.estimate();
            } catch (final KruppaDualImageOfAbsoluteConicEstimatorException e) {
                continue;
            }
            final var intrinsic2 = diac.getIntrinsicParameters();

            if (Math.abs(intrinsic2.getHorizontalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(intrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);

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

            final var principalAxis1 = camera1.getPrincipalAxisArray();
            final var principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1;
            double lambda2;

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            InhomogeneousPoint3D worldPoint;
            final var worldPoints = new ArrayList<InhomogeneousPoint3D>();
            Point2D leftPoint;
            Point2D rightPoint;
            final var leftPoints = new ArrayList<Point2D>();
            final var rightPoints = new ArrayList<Point2D>();
            boolean leftFront;
            boolean rightFront;
            var failed = false;
            for (var i = 0; i < nPoints; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambda1 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);
                    lambda2 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);

                    worldPoint = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + principalAxis1[0] * lambda1
                                    + principalAxis2[0] * lambda2,
                            centralCommonPoint.getInhomY() + principalAxis1[1] * lambda1
                                    + principalAxis2[1] * lambda2,
                            centralCommonPoint.getInhomZ() + principalAxis1[2] * lambda1
                                    + principalAxis2[2] * lambda2);
                    leftFront = camera1.isPointInFrontOfCamera(worldPoint);
                    rightFront = camera2.isPointInFrontOfCamera(worldPoint);
                    if (numTry > MAX_TRIES) {
                        failed = true;
                        break;
                    }
                    numTry++;
                } while (!leftFront || !rightFront);
                if (failed) {
                    break;
                }
                worldPoints.add(worldPoint);

                // check that world point is in front of both cameras
                assertTrue(leftFront);
                assertTrue(rightFront);

                // project world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);

                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }

            if (failed) {
                continue;
            }

            final var camera1b = new PinholeCamera();
            final var camera2b = new PinholeCamera();
            final var numValid = DualImageOfAbsoluteConicInitialCamerasEstimator.generateInitialMetricCamerasUsingDIAC(
                    fundamentalMatrix, principalPointX, principalPointY, leftPoints, rightPoints,
                    CorrectorType.GOLD_STANDARD, camera1b, camera2b);

            // check correctness
            assertEquals(numValid, nPoints);

            camera1b.decompose();
            camera2b.decompose();

            final var intrinsic1b = camera1b.getIntrinsicParameters();
            final var intrinsic2b = camera2b.getIntrinsicParameters();

            final var rotation1b = camera1b.getCameraRotation();
            final var rotation2b = camera2b.getCameraRotation();

            final var center1b = camera1b.getCameraCenter();
            final var center2b = camera2b.getCameraCenter();

            assertEquals(focalLength, intrinsic1b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic1b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic1b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic1b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(focalLength, intrinsic2b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic2b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic2b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic2b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            final var diffRotation = rotation2b.combineAndReturnNew(rotation1b.inverseRotationAndReturnNew());

            assertTrue(rotation1b.asInhomogeneousMatrix().equals(Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            assertNotNull(center1b);
            assertNotNull(center2b);

            // compute scale factor
            final var distanceA = center1.distanceTo(center2);
            final var distanceB = center1b.distanceTo(center2b);
            final var scaleFactor = distanceB / distanceA;
            final var invScaleFactor = distanceA / distanceB;

            // NOTE: distance between estimated cameras is always normalized
            assertEquals(1.0, distanceB, ABSOLUTE_ERROR);

            final var scaleTransformation = new MetricTransformation3D(scaleFactor);
            final var invScaleTransformation = scaleTransformation.inverseAndReturnNew();
            final var invScaleTransformation2 = new MetricTransformation3D(invScaleFactor);

            assertTrue(invScaleTransformation.asMatrix().equals(invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));

            // check that estimated cameras generate the same input fundamental
            // matrix
            final var fundamentalMatrixB = new FundamentalMatrix(camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar
            // geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final var epipole1 = camera1.project(center2);
            final var epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final var epipole1a = fundamentalMatrix.getLeftEpipole();
            final var epipole2a = fundamentalMatrix.getRightEpipole();

            final var epipole1b = fundamentalMatrixB.getLeftEpipole();
            final var epipole2b = fundamentalMatrixB.getRightEpipole();

            if (epipole1.distanceTo(epipole1a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1.distanceTo(epipole1a), ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2.distanceTo(epipole2a), ABSOLUTE_ERROR);

            if (epipole1.distanceTo(epipole1b) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1.distanceTo(epipole1b), ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2b) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2.distanceTo(epipole2b), LARGE_ABSOLUTE_ERROR);

            // generate epipolar lines
            Point3D scaledWorldPoint;
            var numValid1a = 0;
            var numValid2a = 0;
            var numValid1b = 0;
            var numValid2b = 0;
            for (var i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);

                final var line1a = fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                final var line2a = fundamentalMatrix.getRightEpipolarLine(leftPoint);

                final var line1b = fundamentalMatrixB.getLeftEpipolarLine(rightPoint);
                final var line2b = fundamentalMatrixB.getRightEpipolarLine(leftPoint);

                // check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));

                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));

                // back-project epipolar lines for each pair of cameras and check
                // that each pair of lines correspond to the same epipolar plane
                final var epipolarPlane1a = camera1.backProject(line1a);
                final var epipolarPlane2a = camera2.backProject(line2a);

                final var epipolarPlane1b = camera1b.backProject(line1b);
                final var epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

                // check that 3D point and both camera centers for each pair of
                // cameras belong to their corresponding epipolar plane
                if (epipolarPlane1a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid1a++;
                }
                assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));

                if (epipolarPlane2a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid2a++;
                }
                assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

                // notice that since estimated cameras have an arbitrary scale,
                // original world point doesn't need to lie on epipolar plane
                // because first a scale transformation needs to be done
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(worldPoint);
                if (epipolarPlane1a.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid1b++;
                }
                assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));

                if (epipolarPlane2b.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid2b++;
                }
                assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR));
            }

            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 && numValid2b > 0) {
                numValidTimes++;
            }

            // recover scale of cameras by undoing their transformations
            final var camera1c = invScaleTransformation.transformAndReturnNew(camera1b);
            final var camera2c = invScaleTransformation.transformAndReturnNew(camera2b);

            // check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();

            final var camera1Matrix = camera1.getInternalMatrix();
            final var camera1cMatrix = camera1c.getInternalMatrix();
            if (!camera1Matrix.equals(camera1cMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera1Matrix.equals(camera1cMatrix, LARGE_ABSOLUTE_ERROR));

            final var camera2Matrix = camera2.getInternalMatrix();
            final var camera2cMatrix = camera2c.getInternalMatrix();
            if (!camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR));

            if (numValidTimes > 0) {
                break;
            }
        }

        assertTrue(numValidTimes > 0);
    }

    @Test
    void testGenerateInitialMetricCamerasUsingDIAC3() throws InvalidPairOfCamerasException, CameraException,
            InitialCamerasEstimationFailedException, com.irurueta.geometry.NotAvailableException,
            com.irurueta.geometry.estimators.NotReadyException, InvalidFundamentalMatrixException, AlgebraException,
            com.irurueta.geometry.estimators.LockedException, InvalidPinholeCameraIntrinsicParametersException {
        var numValidTimes = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));

            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var aspectRatio = 1.0;
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            final var diacEstimator = new KruppaDualImageOfAbsoluteConicEstimator(fundamentalMatrix);
            diacEstimator.setPrincipalPointX(principalPointX);
            diacEstimator.setPrincipalPointY(principalPointY);
            diacEstimator.setFocalDistanceAspectRatioKnown(true);
            diacEstimator.setFocalDistanceAspectRatio(aspectRatio);

            final DualImageOfAbsoluteConic diac;
            try {
                diac = diacEstimator.estimate();
            } catch (final KruppaDualImageOfAbsoluteConicEstimatorException e) {
                continue;
            }
            final var intrinsic2 = diac.getIntrinsicParameters();

            if (Math.abs(intrinsic2.getHorizontalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(intrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);

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

            final var principalAxis1 = camera1.getPrincipalAxisArray();
            final var principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1;
            double lambda2;

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            InhomogeneousPoint3D worldPoint;
            final var worldPoints = new ArrayList<InhomogeneousPoint3D>();
            Point2D leftPoint;
            Point2D rightPoint;
            final var leftPoints = new ArrayList<Point2D>();
            final var rightPoints = new ArrayList<Point2D>();
            boolean leftFront;
            boolean rightFront;
            var failed = false;
            for (var i = 0; i < nPoints; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambda1 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);
                    lambda2 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);

                    worldPoint = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + principalAxis1[0] * lambda1
                                    + principalAxis2[0] * lambda2,
                            centralCommonPoint.getInhomY() + principalAxis1[1] * lambda1
                                    + principalAxis2[1] * lambda2,
                            centralCommonPoint.getInhomZ() + principalAxis1[2] * lambda1
                                    + principalAxis2[2] * lambda2);
                    leftFront = camera1.isPointInFrontOfCamera(worldPoint);
                    rightFront = camera2.isPointInFrontOfCamera(worldPoint);
                    if (numTry > MAX_TRIES) {
                        failed = true;
                        break;
                    }
                    numTry++;
                } while (!leftFront || !rightFront);

                if (failed) {
                    break;
                }

                worldPoints.add(worldPoint);

                // check that world point is in front of both cameras
                assertTrue(leftFront);
                assertTrue(rightFront);

                // project world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);

                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }

            if (failed) {
                continue;
            }

            final var camera1b = new PinholeCamera();
            final var camera2b = new PinholeCamera();
            final var triangulatedPoints = new ArrayList<Point3D>();
            final var validTriangulatedPoints = new BitSet(nPoints);
            final var numValid = DualImageOfAbsoluteConicInitialCamerasEstimator.generateInitialMetricCamerasUsingDIAC(
                    fundamentalMatrix, principalPointX, principalPointY, leftPoints, rightPoints, camera1b, camera2b,
                    triangulatedPoints, validTriangulatedPoints);

            // check correctness
            assertEquals(numValid, nPoints);

            camera1b.decompose();
            camera2b.decompose();

            final var intrinsic1b = camera1b.getIntrinsicParameters();
            final var intrinsic2b = camera2b.getIntrinsicParameters();

            final var rotation1b = camera1b.getCameraRotation();
            final var rotation2b = camera2b.getCameraRotation();

            final var center1b = camera1b.getCameraCenter();
            final var center2b = camera2b.getCameraCenter();

            assertEquals(intrinsic1b.getHorizontalFocalLength(), focalLength, ABSOLUTE_ERROR);
            assertEquals(intrinsic1b.getVerticalFocalLength(), focalLength, ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic1b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic1b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(focalLength, intrinsic2b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic2b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic2b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic2b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            final var diffRotation = rotation2b.combineAndReturnNew(rotation1b.inverseRotationAndReturnNew());

            assertTrue(rotation1b.asInhomogeneousMatrix().equals(Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            assertNotNull(center1b);
            assertNotNull(center2b);

            // compute scale factor
            final var distanceA = center1.distanceTo(center2);
            final var distanceB = center1b.distanceTo(center2b);
            final var scaleFactor = distanceB / distanceA;
            final var invScaleFactor = distanceA / distanceB;

            // NOTE: distance between estimated cameras is always normalized
            assertEquals(1.0, distanceB, ABSOLUTE_ERROR);

            final var scaleTransformation = new MetricTransformation3D(scaleFactor);
            final var invScaleTransformation = scaleTransformation.inverseAndReturnNew();
            final var invScaleTransformation2 = new MetricTransformation3D(invScaleFactor);

            assertTrue(invScaleTransformation.asMatrix().equals(invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));

            // check that estimated cameras generate the same input fundamental
            // matrix
            final var fundamentalMatrixB = new FundamentalMatrix(camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar
            // geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final var epipole1 = camera1.project(center2);
            final var epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final var epipole1a = fundamentalMatrix.getLeftEpipole();
            final var epipole2a = fundamentalMatrix.getRightEpipole();

            final var epipole1b = fundamentalMatrixB.getLeftEpipole();
            final var epipole2b = fundamentalMatrixB.getRightEpipole();

            if (epipole1.distanceTo(epipole1a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1.distanceTo(epipole1a), ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2.distanceTo(epipole2a), ABSOLUTE_ERROR);

            if (epipole1.distanceTo(epipole1b) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1.distanceTo(epipole1b), ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2b) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2.distanceTo(epipole2b), LARGE_ABSOLUTE_ERROR);

            // generate epipolar lines
            Point3D scaledWorldPoint;
            Point3D triangulatedPoint;
            Point3D scaledTriangulatedPoint;
            var numValid1a = 0;
            var numValid2a = 0;
            var numValid1b = 0;
            var numValid2b = 0;
            var numValidEqual = 0;
            for (var i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);

                triangulatedPoint = triangulatedPoints.get(i);
                assertTrue(validTriangulatedPoints.get(i));

                final var line1a = fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                final var line2a = fundamentalMatrix.getRightEpipolarLine(leftPoint);

                final var line1b = fundamentalMatrixB.getLeftEpipolarLine(rightPoint);
                final var line2b = fundamentalMatrixB.getRightEpipolarLine(leftPoint);

                // check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));

                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));

                // back-project epipolar lines for each pair of cameras and check
                // that each pair of lines correspond to the same epipolar plane
                final var epipolarPlane1a = camera1.backProject(line1a);
                final var epipolarPlane2a = camera2.backProject(line2a);

                final var epipolarPlane1b = camera1b.backProject(line1b);
                final var epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

                // check that 3D point and both camera centers for each pair of
                // cameras belong to their corresponding epipolar plane
                if (epipolarPlane1a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid1a++;
                }
                assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));

                if (epipolarPlane2a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid2a++;
                }
                assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

                // notice that since estimated cameras have an arbitrary scale,
                // original world point doesn't need to lie on epipolar plane
                // because first a scale transformation needs to be done
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(worldPoint);
                if (epipolarPlane1a.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid1b++;
                }
                assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));

                if (epipolarPlane2b.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid2b++;
                }
                assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR));

                // recover scale in triangulated point
                scaledTriangulatedPoint = invScaleTransformation.transformAndReturnNew(triangulatedPoint);

                // check that triangulated point after recovering scale matches
                // original point
                if (worldPoint.equals(scaledTriangulatedPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidEqual++;
                }
            }

            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 && numValid2b > 0 && numValidEqual > 0) {
                numValidTimes++;
            }

            // recover scale of cameras by undoing their transformations
            final var camera1c = invScaleTransformation.transformAndReturnNew(camera1b);
            final var camera2c = invScaleTransformation.transformAndReturnNew(camera2b);

            // check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();

            final var camera1Matrix = camera1.getInternalMatrix();
            final var camera1cMatrix = camera1c.getInternalMatrix();
            if (camera1Matrix.equals(camera1cMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera1Matrix.equals(camera1cMatrix, LARGE_ABSOLUTE_ERROR));

            final var camera2Matrix = camera2.getInternalMatrix();
            final var camera2cMatrix = camera2c.getInternalMatrix();
            if (camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR));

            if (numValidTimes > 0) {
                break;
            }
        }

        assertTrue(numValidTimes > 0);
    }

    @Test
    void testGenerateInitialMetricCamerasUsingDIAC4() throws InvalidPairOfCamerasException, CameraException,
            InitialCamerasEstimationFailedException, com.irurueta.geometry.NotAvailableException,
            com.irurueta.geometry.estimators.NotReadyException, InvalidFundamentalMatrixException, AlgebraException,
            com.irurueta.geometry.estimators.LockedException, InvalidPinholeCameraIntrinsicParametersException {
        var numValidTimes = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));

            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var aspectRatio = 1.0;
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            final var diacEstimator = new KruppaDualImageOfAbsoluteConicEstimator(fundamentalMatrix);
            diacEstimator.setPrincipalPointX(principalPointX);
            diacEstimator.setPrincipalPointY(principalPointY);
            diacEstimator.setFocalDistanceAspectRatioKnown(true);
            diacEstimator.setFocalDistanceAspectRatio(aspectRatio);

            final DualImageOfAbsoluteConic diac;
            try {
                diac = diacEstimator.estimate();
            } catch (final KruppaDualImageOfAbsoluteConicEstimatorException e) {
                continue;
            }
            final var intrinsic2 = diac.getIntrinsicParameters();

            if (Math.abs(intrinsic2.getHorizontalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(intrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);

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

            final var principalAxis1 = camera1.getPrincipalAxisArray();
            final var principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1;
            double lambda2;

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            InhomogeneousPoint3D worldPoint;
            final var worldPoints = new ArrayList<InhomogeneousPoint3D>();
            Point2D leftPoint;
            Point2D rightPoint;
            final var leftPoints = new ArrayList<Point2D>();
            final var rightPoints = new ArrayList<Point2D>();
            boolean leftFront;
            boolean rightFront;
            var failed = false;
            for (var i = 0; i < nPoints; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambda1 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);
                    lambda2 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);

                    worldPoint = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + principalAxis1[0] * lambda1
                                    + principalAxis2[0] * lambda2,
                            centralCommonPoint.getInhomY() + principalAxis1[1] * lambda1
                                    + principalAxis2[1] * lambda2,
                            centralCommonPoint.getInhomZ() + principalAxis1[2] * lambda1
                                    + principalAxis2[2] * lambda2);
                    leftFront = camera1.isPointInFrontOfCamera(worldPoint);
                    rightFront = camera2.isPointInFrontOfCamera(worldPoint);
                    if (numTry > MAX_TRIES) {
                        failed = true;
                        break;
                    }
                    numTry++;
                } while (!leftFront || !rightFront);

                if (failed) {
                    break;
                }
                worldPoints.add(worldPoint);

                // check that world point is in front of both cameras
                assertTrue(leftFront);
                assertTrue(rightFront);

                // project world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);

                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }

            if (failed) {
                continue;
            }

            final var camera1b = new PinholeCamera();
            final var camera2b = new PinholeCamera();
            final var triangulatedPoints = new ArrayList<Point3D>();
            final var validTriangulatedPoints = new BitSet(nPoints);
            final var numValid = DualImageOfAbsoluteConicInitialCamerasEstimator.generateInitialMetricCamerasUsingDIAC(
                    fundamentalMatrix, principalPointX, principalPointY, leftPoints, rightPoints,
                    CorrectorType.SAMPSON_CORRECTOR, camera1b, camera2b, triangulatedPoints, validTriangulatedPoints);

            // check correctness
            assertEquals(numValid, nPoints);

            camera1b.decompose();
            camera2b.decompose();

            final var intrinsic1b = camera1b.getIntrinsicParameters();
            final var intrinsic2b = camera2b.getIntrinsicParameters();

            final var rotation1b = camera1b.getCameraRotation();
            final var rotation2b = camera2b.getCameraRotation();

            final var center1b = camera1b.getCameraCenter();
            final var center2b = camera2b.getCameraCenter();

            assertEquals(focalLength, intrinsic1b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic1b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic1b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic1b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(focalLength, intrinsic2b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic2b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic2b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic2b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            final var diffRotation = rotation2b.combineAndReturnNew(rotation1b.inverseRotationAndReturnNew());

            assertTrue(rotation1b.asInhomogeneousMatrix().equals(Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            assertNotNull(center1b);
            assertNotNull(center2b);

            // compute scale factor
            final var distanceA = center1.distanceTo(center2);
            final var distanceB = center1b.distanceTo(center2b);
            final var scaleFactor = distanceB / distanceA;
            final var invScaleFactor = distanceA / distanceB;

            // NOTE: distance between estimated cameras is always normalized
            assertEquals(1.0, distanceB, ABSOLUTE_ERROR);

            final var scaleTransformation = new MetricTransformation3D(scaleFactor);
            final var invScaleTransformation = scaleTransformation.inverseAndReturnNew();
            final var invScaleTransformation2 = new MetricTransformation3D(invScaleFactor);

            assertTrue(invScaleTransformation.asMatrix().equals(invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));

            // check that estimated cameras generate the same input fundamental
            // matrix
            final var fundamentalMatrixB = new FundamentalMatrix(camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar
            // geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final var epipole1 = camera1.project(center2);
            final var epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final var epipole1a = fundamentalMatrix.getLeftEpipole();
            final var epipole2a = fundamentalMatrix.getRightEpipole();

            final var epipole1b = fundamentalMatrixB.getLeftEpipole();
            final var epipole2b = fundamentalMatrixB.getRightEpipole();

            if (epipole1.distanceTo(epipole1a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1.distanceTo(epipole1a), ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2.distanceTo(epipole2a), ABSOLUTE_ERROR);

            if (epipole1.distanceTo(epipole1b) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1.distanceTo(epipole1b), ABSOLUTE_ERROR);

            if (epipole2.distanceTo(epipole2b) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2.distanceTo(epipole2b), LARGE_ABSOLUTE_ERROR);

            // generate epipolar lines
            Point3D scaledWorldPoint;
            Point3D triangulatedPoint;
            Point3D scaledTriangulatedPoint;
            var numValid1a = 0;
            var numValid2a = 0;
            var numValid1b = 0;
            var numValid2b = 0;
            var numValidEqual = 0;
            for (var i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);

                triangulatedPoint = triangulatedPoints.get(i);
                assertTrue(validTriangulatedPoints.get(i));

                final var line1a = fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                final var line2a = fundamentalMatrix.getRightEpipolarLine(leftPoint);

                final var line1b = fundamentalMatrixB.getLeftEpipolarLine(rightPoint);
                final var line2b = fundamentalMatrixB.getRightEpipolarLine(leftPoint);

                // check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));

                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));

                // back-project epipolar lines for each pair of cameras and check
                // that each pair of lines correspond to the same epipolar plane
                final var epipolarPlane1a = camera1.backProject(line1a);
                final var epipolarPlane2a = camera2.backProject(line2a);

                final var epipolarPlane1b = camera1b.backProject(line1b);
                final var epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

                // check that 3D point and both camera centers for each pair of
                // cameras belong to their corresponding epipolar plane
                if (epipolarPlane1a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid1a++;
                }
                assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));

                if (epipolarPlane2a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid2a++;
                }
                assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

                // notice that since estimated cameras have an arbitrary scale,
                // original world point doesn't need to lie on epipolar plane
                // because first a scale transformation needs to be done
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(worldPoint);
                if (epipolarPlane1a.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid1b++;
                }
                assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));

                if (epipolarPlane2b.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid2b++;
                }
                assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR));

                // recover scale in triangulated point
                scaledTriangulatedPoint = invScaleTransformation.transformAndReturnNew(triangulatedPoint);

                // check that triangulated point after recovering scale matches
                // original point
                if (worldPoint.equals(scaledTriangulatedPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidEqual++;
                }
            }

            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 && numValid2b > 0 && numValidEqual > 0) {
                numValidTimes++;
            }

            // recover scale of cameras by undoing their transformations
            final var camera1c = invScaleTransformation.transformAndReturnNew(camera1b);
            final var camera2c = invScaleTransformation.transformAndReturnNew(camera2b);

            // check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();

            final var camera1Matrix = camera1.getInternalMatrix();
            final var camera1cMatrix = camera1c.getInternalMatrix();
            if (!camera1Matrix.equals(camera1cMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera1Matrix.equals(camera1cMatrix, LARGE_ABSOLUTE_ERROR));

            final var camera2Matrix = camera2.getInternalMatrix();
            final var camera2cMatrix = camera2c.getInternalMatrix();
            if (!camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR));

            break;
        }

        assertTrue(numValidTimes > 0);
    }

    @Test
    void testGenerateInitialMetricCamerasUsingDIAC5() throws InvalidPairOfCamerasException, CameraException,
            InitialCamerasEstimationFailedException, com.irurueta.geometry.NotAvailableException,
            com.irurueta.geometry.estimators.NotReadyException, InvalidFundamentalMatrixException, AlgebraException,
            com.irurueta.geometry.estimators.LockedException, InvalidPinholeCameraIntrinsicParametersException {
        var numValidTimes = 0;
        var numValidTimes2 = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));

            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var aspectRatio = 1.0;
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            final var diacEstimator = new KruppaDualImageOfAbsoluteConicEstimator(fundamentalMatrix);
            diacEstimator.setPrincipalPointX(principalPointX);
            diacEstimator.setPrincipalPointY(principalPointY);
            diacEstimator.setFocalDistanceAspectRatioKnown(true);
            diacEstimator.setFocalDistanceAspectRatio(aspectRatio);

            final DualImageOfAbsoluteConic diac;
            try {
                diac = diacEstimator.estimate();
            } catch (final KruppaDualImageOfAbsoluteConicEstimatorException e) {
                continue;
            }
            final var intrinsic2 = diac.getIntrinsicParameters();

            if (Math.abs(intrinsic2.getHorizontalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(intrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);

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

            final var principalAxis1 = camera1.getPrincipalAxisArray();
            final var principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1;
            double lambda2;

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            InhomogeneousPoint3D worldPoint;
            final var worldPoints = new ArrayList<InhomogeneousPoint3D>();
            Point2D leftPoint;
            Point2D rightPoint;
            final var leftPoints = new ArrayList<Point2D>();
            final var rightPoints = new ArrayList<Point2D>();
            boolean leftFront;
            boolean rightFront;
            for (var i = 0; i < nPoints; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambda1 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);
                    lambda2 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);

                    worldPoint = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + principalAxis1[0] * lambda1
                                    + principalAxis2[0] * lambda2,
                            centralCommonPoint.getInhomY() + principalAxis1[1] * lambda1
                                    + principalAxis2[1] * lambda2,
                            centralCommonPoint.getInhomZ() + principalAxis1[2] * lambda1
                                    + principalAxis2[2] * lambda2);
                    leftFront = camera1.isPointInFrontOfCamera(worldPoint);
                    rightFront = camera2.isPointInFrontOfCamera(worldPoint);
                    if (numTry > MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!leftFront || !rightFront);
                worldPoints.add(worldPoint);

                // check that world point is in front of both cameras
                assertTrue(leftFront);
                assertTrue(rightFront);

                // project world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);

                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }

            final var camera1b = new PinholeCamera();
            final var camera2b = new PinholeCamera();
            final var numValid = DualImageOfAbsoluteConicInitialCamerasEstimator.generateInitialMetricCamerasUsingDIAC(
                    fundamentalMatrix, principalPointX, principalPointY, aspectRatio, leftPoints, rightPoints, camera1b,
                    camera2b);

            // check correctness
            assertEquals(numValid, nPoints);

            camera1b.decompose();
            camera2b.decompose();

            final var intrinsic1b = camera1b.getIntrinsicParameters();
            final var intrinsic2b = camera2b.getIntrinsicParameters();

            final var rotation1b = camera1b.getCameraRotation();
            final var rotation2b = camera2b.getCameraRotation();

            final var center1b = camera1b.getCameraCenter();
            final var center2b = camera2b.getCameraCenter();

            assertEquals(focalLength, intrinsic1b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic1b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic1b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic1b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(focalLength, intrinsic2b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic2b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic2b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic2b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            final var diffRotation = rotation2b.combineAndReturnNew(rotation1b.inverseRotationAndReturnNew());

            assertTrue(rotation1b.asInhomogeneousMatrix().equals(Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            assertNotNull(center1b);
            assertNotNull(center2b);

            // compute scale factor
            final var distanceA = center1.distanceTo(center2);
            final var distanceB = center1b.distanceTo(center2b);
            final var scaleFactor = distanceB / distanceA;
            final var invScaleFactor = distanceA / distanceB;

            // NOTE: distance between estimated cameras is always normalized
            assertEquals(1.0, distanceB, ABSOLUTE_ERROR);

            final var scaleTransformation = new MetricTransformation3D(scaleFactor);
            final var invScaleTransformation = scaleTransformation.inverseAndReturnNew();
            final var invScaleTransformation2 = new MetricTransformation3D(invScaleFactor);

            assertTrue(invScaleTransformation.asMatrix().equals(invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));

            // check that estimated cameras generate the same input fundamental
            // matrix
            final var fundamentalMatrixB = new FundamentalMatrix(camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar
            // geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final var epipole1 = camera1.project(center2);
            final var epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final var epipole1a = fundamentalMatrix.getLeftEpipole();
            final var epipole2a = fundamentalMatrix.getRightEpipole();

            final var epipole1b = fundamentalMatrixB.getLeftEpipole();
            final var epipole2b = fundamentalMatrixB.getRightEpipole();

            assertEquals(0.0, epipole1.distanceTo(epipole1a), ABSOLUTE_ERROR);
            assertEquals(0.0, epipole2.distanceTo(epipole2a), LARGE_ABSOLUTE_ERROR);

            assertEquals(0.0, epipole1.distanceTo(epipole1b), ABSOLUTE_ERROR);
            assertEquals(0.0, epipole2.distanceTo(epipole2b), VERY_LARGE_ABSOLUTE_ERROR);

            // generate epipolar lines
            Point3D scaledWorldPoint;
            var numValid1a = 0;
            var numValid2a = 0;
            var numValid1b = 0;
            var numValid2b = 0;
            for (var i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);

                final var line1a = fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                final var line2a = fundamentalMatrix.getRightEpipolarLine(leftPoint);

                final var line1b = fundamentalMatrixB.getLeftEpipolarLine(rightPoint);
                final var line2b = fundamentalMatrixB.getRightEpipolarLine(leftPoint);

                // check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));

                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));

                // back-project epipolar lines for each pair of cameras and check
                // that each pair of lines correspond to the same epipolar plane
                final var epipolarPlane1a = camera1.backProject(line1a);
                final var epipolarPlane2a = camera2.backProject(line2a);

                final var epipolarPlane1b = camera1b.backProject(line1b);
                final var epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

                // check that 3D point and both camera centers for each pair of
                // cameras belong to their corresponding epipolar plane
                if (epipolarPlane1a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid1a++;
                }
                assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));

                if (epipolarPlane2a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid2a++;
                }
                assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

                // notice that since estimated cameras have an arbitrary scale,
                // original world point doesn't need to lie on epipolar plane
                // because first a scale transformation needs to be done
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(worldPoint);
                if (epipolarPlane1a.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid1b++;
                }
                assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));

                if (epipolarPlane2b.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid2b++;
                }
                assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR));
            }

            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 && numValid2b > 0) {
                numValidTimes++;
            }

            // recover scale of cameras by undoing their transformations
            final var camera1c = invScaleTransformation.transformAndReturnNew(camera1b);
            final var camera2c = invScaleTransformation.transformAndReturnNew(camera2b);

            // check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();

            final var camera1Matrix = camera1.getInternalMatrix();
            final var camera1cMatrix = camera1c.getInternalMatrix();
            if (!camera1Matrix.equals(camera1cMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera1Matrix.equals(camera1cMatrix, LARGE_ABSOLUTE_ERROR));

            final var camera2Matrix = camera2.getInternalMatrix();
            final var camera2cMatrix = camera2c.getInternalMatrix();
            if (!camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR));

            numValidTimes2++;

            if (numValidTimes > 0 && numValidTimes2 > 0) {
                break;
            }
        }

        assertTrue(numValidTimes > 0);
        assertTrue(numValidTimes2 > 0);
    }

    @Test
    void testGenerateInitialMetricCamerasUsingDIAC6() throws InvalidPairOfCamerasException, CameraException,
            InitialCamerasEstimationFailedException, com.irurueta.geometry.NotAvailableException,
            com.irurueta.geometry.estimators.NotReadyException, InvalidFundamentalMatrixException, AlgebraException,
            com.irurueta.geometry.estimators.LockedException, InvalidPinholeCameraIntrinsicParametersException {
        var numValidTimes = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));

            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var aspectRatio = 1.0;
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            final var diacEstimator = new KruppaDualImageOfAbsoluteConicEstimator(fundamentalMatrix);
            diacEstimator.setPrincipalPointX(principalPointX);
            diacEstimator.setPrincipalPointY(principalPointY);
            diacEstimator.setFocalDistanceAspectRatioKnown(true);
            diacEstimator.setFocalDistanceAspectRatio(aspectRatio);

            final DualImageOfAbsoluteConic diac;
            try {
                diac = diacEstimator.estimate();
            } catch (KruppaDualImageOfAbsoluteConicEstimatorException e) {
                continue;
            }
            final var intrinsic2 = diac.getIntrinsicParameters();

            if (Math.abs(intrinsic2.getHorizontalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(intrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);

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

            final var principalAxis1 = camera1.getPrincipalAxisArray();
            final var principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1;
            double lambda2;

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            InhomogeneousPoint3D worldPoint;
            final var worldPoints = new ArrayList<InhomogeneousPoint3D>();
            Point2D leftPoint;
            Point2D rightPoint;
            final var leftPoints = new ArrayList<Point2D>();
            final var rightPoints = new ArrayList<Point2D>();
            boolean leftFront;
            boolean rightFront;
            var failed = false;
            for (var i = 0; i < nPoints; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambda1 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);
                    lambda2 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);

                    worldPoint = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + principalAxis1[0] * lambda1
                                    + principalAxis2[0] * lambda2,
                            centralCommonPoint.getInhomY() + principalAxis1[1] * lambda1
                                    + principalAxis2[1] * lambda2,
                            centralCommonPoint.getInhomZ() + principalAxis1[2] * lambda1
                                    + principalAxis2[2] * lambda2);
                    leftFront = camera1.isPointInFrontOfCamera(worldPoint);
                    rightFront = camera2.isPointInFrontOfCamera(worldPoint);
                    if (numTry > MAX_TRIES) {
                        failed = true;
                        break;
                    }
                    numTry++;
                } while (!leftFront || !rightFront);

                if (failed) {
                    break;
                }

                worldPoints.add(worldPoint);

                // check that world point is in front of both cameras
                assertTrue(leftFront);
                assertTrue(rightFront);

                // project world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);

                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }

            if (failed) {
                continue;
            }

            final var camera1b = new PinholeCamera();
            final var camera2b = new PinholeCamera();
            final var numValid = DualImageOfAbsoluteConicInitialCamerasEstimator.generateInitialMetricCamerasUsingDIAC(
                    fundamentalMatrix, principalPointX, principalPointY, aspectRatio, leftPoints, rightPoints,
                    CorrectorType.GOLD_STANDARD, camera1b, camera2b);

            // check correctness
            assertEquals(numValid, nPoints);

            camera1b.decompose();
            camera2b.decompose();

            final var intrinsic1b = camera1b.getIntrinsicParameters();
            final var intrinsic2b = camera2b.getIntrinsicParameters();

            final var rotation1b = camera1b.getCameraRotation();
            final var rotation2b = camera2b.getCameraRotation();

            final var center1b = camera1b.getCameraCenter();
            final var center2b = camera2b.getCameraCenter();

            assertEquals(focalLength, intrinsic1b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic1b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic1b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic1b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(focalLength, intrinsic2b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic2b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic2b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic2b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            final var diffRotation = rotation2b.combineAndReturnNew(rotation1b.inverseRotationAndReturnNew());

            assertTrue(rotation1b.asInhomogeneousMatrix().equals(Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            assertNotNull(center1b);
            assertNotNull(center2b);

            // compute scale factor
            final var distanceA = center1.distanceTo(center2);
            final var distanceB = center1b.distanceTo(center2b);
            final var scaleFactor = distanceB / distanceA;
            final var invScaleFactor = distanceA / distanceB;

            // NOTE: distance between estimated cameras is always normalized
            assertEquals(1.0, distanceB, ABSOLUTE_ERROR);

            final var scaleTransformation = new MetricTransformation3D(scaleFactor);
            final var invScaleTransformation = scaleTransformation.inverseAndReturnNew();
            final var invScaleTransformation2 = new MetricTransformation3D(invScaleFactor);

            assertTrue(invScaleTransformation.asMatrix().equals(invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));

            // check that estimated cameras generate the same input fundamental
            // matrix
            final var fundamentalMatrixB = new FundamentalMatrix(camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar
            // geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final var epipole1 = camera1.project(center2);
            final var epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final var epipole1a = fundamentalMatrix.getLeftEpipole();
            final var epipole2a = fundamentalMatrix.getRightEpipole();

            final var epipole1b = fundamentalMatrixB.getLeftEpipole();
            final var epipole2b = fundamentalMatrixB.getRightEpipole();

            assertEquals(0.0, epipole1.distanceTo(epipole1a), ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2.distanceTo(epipole2a), ABSOLUTE_ERROR);

            assertEquals(0.0, epipole1.distanceTo(epipole1b), ABSOLUTE_ERROR);
            assertEquals(0.0, epipole2.distanceTo(epipole2b), 2.0 * LARGE_ABSOLUTE_ERROR);

            // generate epipolar lines
            Point3D scaledWorldPoint;
            var numValid1a = 0;
            var numValid2a = 0;
            var numValid1b = 0;
            var numValid2b = 0;
            for (var i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);

                final var line1a = fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                final var line2a = fundamentalMatrix.getRightEpipolarLine(leftPoint);

                final var line1b = fundamentalMatrixB.getLeftEpipolarLine(rightPoint);
                final var line2b = fundamentalMatrixB.getRightEpipolarLine(leftPoint);

                // check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));

                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));

                // back-project epipolar lines for each pair of cameras and check
                // that each pair of lines correspond to the same epipolar plane
                final var epipolarPlane1a = camera1.backProject(line1a);
                final var epipolarPlane2a = camera2.backProject(line2a);

                final var epipolarPlane1b = camera1b.backProject(line1b);
                final var epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

                // check that 3D point and both camera centers for each pair of
                // cameras belong to their corresponding epipolar plane
                if (epipolarPlane1a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid1a++;
                }
                assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));

                if (epipolarPlane2a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid2a++;
                }
                assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

                // notice that since estimated cameras have an arbitrary scale,
                // original world point doesn't need to lie on epipolar plane
                // because first a scale transformation needs to be done
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(worldPoint);
                if (epipolarPlane1a.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid1b++;
                }
                assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));

                if (epipolarPlane2b.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid2b++;
                }
                assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR));
            }

            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 && numValid2b > 0) {
                numValidTimes++;
            }

            // recover scale of cameras by undoing their transformations
            final var camera1c = invScaleTransformation.transformAndReturnNew(camera1b);
            final var camera2c = invScaleTransformation.transformAndReturnNew(camera2b);

            // check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();

            final var camera1Matrix = camera1.getInternalMatrix();
            final var camera1cMatrix = camera1c.getInternalMatrix();
            if (!camera1Matrix.equals(camera1cMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera1Matrix.equals(camera1cMatrix, LARGE_ABSOLUTE_ERROR));

            final var camera2Matrix = camera2.getInternalMatrix();
            final var camera2cMatrix = camera2c.getInternalMatrix();
            if (!camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR));
        }

        assertTrue(numValidTimes > 0);
    }

    @Test
    void testGenerateInitialMetricCamerasUsingDIAC7() throws InvalidPairOfCamerasException, CameraException,
            InitialCamerasEstimationFailedException, com.irurueta.geometry.NotAvailableException,
            com.irurueta.geometry.estimators.NotReadyException, InvalidFundamentalMatrixException, AlgebraException,
            com.irurueta.geometry.estimators.LockedException, InvalidPinholeCameraIntrinsicParametersException {
        var numValidTimes = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));

            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var aspectRatio = 1.0;
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            final var diacEstimator = new KruppaDualImageOfAbsoluteConicEstimator(fundamentalMatrix);
            diacEstimator.setPrincipalPointX(principalPointX);
            diacEstimator.setPrincipalPointY(principalPointY);
            diacEstimator.setFocalDistanceAspectRatioKnown(true);
            diacEstimator.setFocalDistanceAspectRatio(aspectRatio);

            final DualImageOfAbsoluteConic diac;
            try {
                diac = diacEstimator.estimate();
            } catch (final KruppaDualImageOfAbsoluteConicEstimatorException e) {
                continue;
            }
            final var intrinsic2 = diac.getIntrinsicParameters();

            if (Math.abs(intrinsic2.getHorizontalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(intrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);

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

            final var principalAxis1 = camera1.getPrincipalAxisArray();
            final var principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1;
            double lambda2;

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            InhomogeneousPoint3D worldPoint;
            final var worldPoints = new ArrayList<InhomogeneousPoint3D>();
            Point2D leftPoint;
            Point2D rightPoint;
            final var leftPoints = new ArrayList<Point2D>();
            final var rightPoints = new ArrayList<Point2D>();
            boolean leftFront;
            boolean rightFront;
            for (var i = 0; i < nPoints; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambda1 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);
                    lambda2 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);

                    worldPoint = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + principalAxis1[0] * lambda1
                                    + principalAxis2[0] * lambda2,
                            centralCommonPoint.getInhomY() + principalAxis1[1] * lambda1
                                    + principalAxis2[1] * lambda2,
                            centralCommonPoint.getInhomZ() + principalAxis1[2] * lambda1
                                    + principalAxis2[2] * lambda2);
                    leftFront = camera1.isPointInFrontOfCamera(worldPoint);
                    rightFront = camera2.isPointInFrontOfCamera(worldPoint);
                    if (numTry > 5 * MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!leftFront || !rightFront);
                worldPoints.add(worldPoint);

                // check that world point is in front of both cameras
                assertTrue(leftFront);
                assertTrue(rightFront);

                // project world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);

                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }

            final var camera1b = new PinholeCamera();
            final var camera2b = new PinholeCamera();
            final var triangulatedPoints = new ArrayList<Point3D>();
            final var validTriangulatedPoints = new BitSet(nPoints);
            final var numValid = DualImageOfAbsoluteConicInitialCamerasEstimator.generateInitialMetricCamerasUsingDIAC(
                    fundamentalMatrix, principalPointX, principalPointY, aspectRatio, leftPoints, rightPoints, camera1b,
                    camera2b, triangulatedPoints, validTriangulatedPoints);

            // check correctness
            assertEquals(numValid, nPoints);

            camera1b.decompose();
            camera2b.decompose();

            final var intrinsic1b = camera1b.getIntrinsicParameters();
            final var intrinsic2b = camera2b.getIntrinsicParameters();

            final var rotation1b = camera1b.getCameraRotation();
            final var rotation2b = camera2b.getCameraRotation();

            final var center1b = camera1b.getCameraCenter();
            final var center2b = camera2b.getCameraCenter();

            assertEquals(focalLength, intrinsic1b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic1b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic1b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic1b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(focalLength, intrinsic2b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic2b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic2b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic2b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            final var diffRotation = rotation2b.combineAndReturnNew(rotation1b.inverseRotationAndReturnNew());

            assertTrue(rotation1b.asInhomogeneousMatrix().equals(Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            assertNotNull(center1b);
            assertNotNull(center2b);

            // compute scale factor
            final var distanceA = center1.distanceTo(center2);
            final var distanceB = center1b.distanceTo(center2b);
            final var scaleFactor = distanceB / distanceA;
            final var invScaleFactor = distanceA / distanceB;

            // NOTE: distance between estimated cameras is always normalized
            assertEquals(1.0, distanceB, ABSOLUTE_ERROR);

            final var scaleTransformation = new MetricTransformation3D(scaleFactor);
            final var invScaleTransformation = scaleTransformation.inverseAndReturnNew();
            final var invScaleTransformation2 = new MetricTransformation3D(invScaleFactor);

            assertTrue(invScaleTransformation.asMatrix().equals(invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));

            // check that estimated cameras generate the same input fundamental
            // matrix
            final var fundamentalMatrixB = new FundamentalMatrix(camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar
            // geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final var epipole1 = camera1.project(center2);
            final var epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final var epipole1a = fundamentalMatrix.getLeftEpipole();
            final var epipole2a = fundamentalMatrix.getRightEpipole();

            final var epipole1b = fundamentalMatrixB.getLeftEpipole();
            final var epipole2b = fundamentalMatrixB.getRightEpipole();

            assertEquals(0.0, epipole1.distanceTo(epipole1a), ABSOLUTE_ERROR);
            assertEquals(0.0, epipole2.distanceTo(epipole2a), ABSOLUTE_ERROR);

            assertEquals(0.0, epipole1.distanceTo(epipole1b), ABSOLUTE_ERROR);
            assertEquals(0.0, epipole2.distanceTo(epipole2b), LARGE_ABSOLUTE_ERROR);

            // generate epipolar lines
            Point3D scaledWorldPoint;
            Point3D triangulatedPoint;
            Point3D scaledTriangulatedPoint;
            var numValid1a = 0;
            var numValid2a = 0;
            var numValid1b = 0;
            var numValid2b = 0;
            var numValidEqual = 0;
            for (var i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);

                triangulatedPoint = triangulatedPoints.get(i);
                assertTrue(validTriangulatedPoints.get(i));

                final var line1a = fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                final var line2a = fundamentalMatrix.getRightEpipolarLine(leftPoint);

                final var line1b = fundamentalMatrixB.getLeftEpipolarLine(rightPoint);
                final var line2b = fundamentalMatrixB.getRightEpipolarLine(leftPoint);

                // check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));

                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));

                // back-project epipolar lines for each pair of cameras and check
                // that each pair of lines correspond to the same epipolar plane
                final var epipolarPlane1a = camera1.backProject(line1a);
                final var epipolarPlane2a = camera2.backProject(line2a);

                final var epipolarPlane1b = camera1b.backProject(line1b);
                final var epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

                // check that 3D point and both camera centers for each pair of
                // cameras belong to their corresponding epipolar plane
                if (epipolarPlane1a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid1a++;
                }
                assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));

                if (epipolarPlane2a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid2a++;
                }
                assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

                // notice that since estimated cameras have an arbitrary scale,
                // original world point doesn't need to lie on epipolar plane
                // because first a scale transformation needs to be done
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(worldPoint);
                if (epipolarPlane1a.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid1b++;
                }
                assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));

                if (epipolarPlane2b.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid2b++;
                }
                assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR));

                // recover scale in triangulated point
                scaledTriangulatedPoint = invScaleTransformation.transformAndReturnNew(triangulatedPoint);

                // check that triangulated point after recovering scale matches
                // original point
                if (worldPoint.equals(scaledTriangulatedPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidEqual++;
                }
            }

            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 && numValid2b > 0 && numValidEqual > 0) {
                numValidTimes++;
            }

            // recover scale of cameras by undoing their transformations
            final var camera1c = invScaleTransformation.transformAndReturnNew(camera1b);
            final var camera2c = invScaleTransformation.transformAndReturnNew(camera2b);

            // check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();

            final var camera1Matrix = camera1.getInternalMatrix();
            final var camera1cMatrix = camera1c.getInternalMatrix();
            if (!camera1Matrix.equals(camera1cMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera1Matrix.equals(camera1cMatrix, LARGE_ABSOLUTE_ERROR));

            final var camera2Matrix = camera2.getInternalMatrix();
            final var camera2cMatrix = camera2c.getInternalMatrix();
            if (!camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR));

            if (numValidTimes > 0) {
                break;
            }
        }

        assertTrue(numValidTimes > 0);
    }

    @Test
    void testGenerateInitialMetricCamerasUsingDIAC8() throws InvalidPairOfCamerasException, CameraException,
            InitialCamerasEstimationFailedException, com.irurueta.geometry.NotAvailableException,
            com.irurueta.geometry.estimators.NotReadyException, InvalidFundamentalMatrixException, AlgebraException,
            com.irurueta.geometry.estimators.LockedException, InvalidPinholeCameraIntrinsicParametersException {
        var numValidTimes = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES_KRUPPA,
                    MAX_ANGLE_DEGREES_KRUPPA));

            final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var aspectRatio = 1.0;
            final var skewness = 0.0;
            final var principalPointX = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var principalPointY = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic = new PinholeCameraIntrinsicParameters(focalLength, focalLength, principalPointX,
                    principalPointY, skewness);

            final var camera1 = new PinholeCamera(intrinsic, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic, rotation2, center2);

            final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            final var diacEstimator = new KruppaDualImageOfAbsoluteConicEstimator(fundamentalMatrix);
            diacEstimator.setPrincipalPointX(principalPointX);
            diacEstimator.setPrincipalPointY(principalPointY);
            diacEstimator.setFocalDistanceAspectRatioKnown(true);
            diacEstimator.setFocalDistanceAspectRatio(aspectRatio);

            final DualImageOfAbsoluteConic diac;
            try {
                diac = diacEstimator.estimate();
            } catch (final KruppaDualImageOfAbsoluteConicEstimatorException e) {
                continue;
            }
            final var intrinsic2 = diac.getIntrinsicParameters();

            if (Math.abs(intrinsic2.getHorizontalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }

            assertEquals(intrinsic2.getHorizontalFocalLength(), intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalFocalLength(), intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getHorizontalPrincipalPoint(), intrinsic.getHorizontalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getVerticalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(intrinsic2.getSkewness(), intrinsic.getSkewness(), ABSOLUTE_ERROR);

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

            final var principalAxis1 = camera1.getPrincipalAxisArray();
            final var principalAxis2 = camera2.getPrincipalAxisArray();
            double lambda1;
            double lambda2;

            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

            InhomogeneousPoint3D worldPoint;
            final var worldPoints = new ArrayList<InhomogeneousPoint3D>();
            Point2D leftPoint;
            Point2D rightPoint;
            final var leftPoints = new ArrayList<Point2D>();
            final var rightPoints = new ArrayList<Point2D>();
            boolean leftFront;
            boolean rightFront;
            for (var i = 0; i < nPoints; i++) {
                // generate points and ensure they lie in front of both cameras
                var numTry = 0;
                do {
                    lambda1 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);
                    lambda2 = randomizer.nextDouble(MIN_LAMBDA, MAX_LAMBDA);

                    worldPoint = new InhomogeneousPoint3D(
                            centralCommonPoint.getInhomX() + principalAxis1[0] * lambda1
                                    + principalAxis2[0] * lambda2,
                            centralCommonPoint.getInhomY() + principalAxis1[1] * lambda1
                                    + principalAxis2[1] * lambda2,
                            centralCommonPoint.getInhomZ() + principalAxis1[2] * lambda1
                                    + principalAxis2[2] * lambda2);
                    leftFront = camera1.isPointInFrontOfCamera(worldPoint);
                    rightFront = camera2.isPointInFrontOfCamera(worldPoint);
                    if (numTry > 2 * MAX_TRIES) {
                        fail("max tries reached");
                    }
                    numTry++;
                } while (!leftFront || !rightFront);
                worldPoints.add(worldPoint);

                // check that world point is in front of both cameras
                assertTrue(leftFront);
                assertTrue(rightFront);

                // project world point into both cameras
                leftPoint = camera1.project(worldPoint);
                leftPoints.add(leftPoint);

                rightPoint = camera2.project(worldPoint);
                rightPoints.add(rightPoint);
            }

            final var camera1b = new PinholeCamera();
            final var camera2b = new PinholeCamera();
            final var triangulatedPoints = new ArrayList<Point3D>();
            final var validTriangulatedPoints = new BitSet(nPoints);
            final var numValid = DualImageOfAbsoluteConicInitialCamerasEstimator.generateInitialMetricCamerasUsingDIAC(
                    fundamentalMatrix, principalPointX, principalPointY, aspectRatio, leftPoints, rightPoints,
                    CorrectorType.SAMPSON_CORRECTOR, camera1b, camera2b, triangulatedPoints, validTriangulatedPoints);

            // check correctness
            assertEquals(numValid, nPoints);

            camera1b.decompose();
            camera2b.decompose();

            final var intrinsic1b = camera1b.getIntrinsicParameters();
            final var intrinsic2b = camera2b.getIntrinsicParameters();

            final var rotation1b = camera1b.getCameraRotation();
            final var rotation2b = camera2b.getCameraRotation();

            final var center1b = camera1b.getCameraCenter();
            final var center2b = camera2b.getCameraCenter();

            assertEquals(focalLength, intrinsic1b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic1b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic1b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic1b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic1b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            assertEquals(focalLength, intrinsic2b.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(focalLength, intrinsic2b.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2b.getSkewness(), ABSOLUTE_ERROR);
            assertEquals(principalPointX, intrinsic2b.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(principalPointY, intrinsic2b.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

            final var diffRotation = rotation2b.combineAndReturnNew(rotation1b.inverseRotationAndReturnNew());

            assertTrue(rotation1b.asInhomogeneousMatrix().equals(Matrix.identity(3, 3), ABSOLUTE_ERROR));
            assertTrue(rotation2.asInhomogeneousMatrix().equals(diffRotation.asInhomogeneousMatrix(), ABSOLUTE_ERROR));

            assertNotNull(center1b);
            assertNotNull(center2b);

            // compute scale factor
            final var distanceA = center1.distanceTo(center2);
            final var distanceB = center1b.distanceTo(center2b);
            final var scaleFactor = distanceB / distanceA;
            final var invScaleFactor = distanceA / distanceB;

            // NOTE: distance between estimated cameras is always normalized
            assertEquals(1.0, distanceB, ABSOLUTE_ERROR);

            final var scaleTransformation = new MetricTransformation3D(scaleFactor);
            final var invScaleTransformation = scaleTransformation.inverseAndReturnNew();
            final var invScaleTransformation2 = new MetricTransformation3D(invScaleFactor);

            assertTrue(invScaleTransformation.asMatrix().equals(invScaleTransformation2.asMatrix(), ABSOLUTE_ERROR));


            // check that estimated cameras generate the same input fundamental
            // matrix
            final var fundamentalMatrixB = new FundamentalMatrix(camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar
            // geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final var epipole1 = camera1.project(center2);
            final var epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final var epipole1a = fundamentalMatrix.getLeftEpipole();
            final var epipole2a = fundamentalMatrix.getRightEpipole();

            final var epipole1b = fundamentalMatrixB.getLeftEpipole();
            final var epipole2b = fundamentalMatrixB.getRightEpipole();

            if (epipole1.distanceTo(epipole1a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1.distanceTo(epipole1a), ABSOLUTE_ERROR);

            if (epipole2.distanceTo(epipole2a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2.distanceTo(epipole2a), ABSOLUTE_ERROR);

            if (epipole1.distanceTo(epipole1b) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1.distanceTo(epipole1b), ABSOLUTE_ERROR);

            if (epipole2.distanceTo(epipole2b) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2.distanceTo(epipole2b), LARGE_ABSOLUTE_ERROR);

            // generate epipolar lines
            Point3D scaledWorldPoint;
            Point3D triangulatedPoint;
            Point3D scaledTriangulatedPoint;
            var numValid1a = 0;
            var numValid2a = 0;
            var numValid1b = 0;
            var numValid2b = 0;
            var numValidEqual = 0;
            for (var i = 0; i < nPoints; i++) {
                worldPoint = worldPoints.get(i);
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);

                triangulatedPoint = triangulatedPoints.get(i);
                assertTrue(validTriangulatedPoints.get(i));

                final var line1a = fundamentalMatrix.getLeftEpipolarLine(rightPoint);
                final var line2a = fundamentalMatrix.getRightEpipolarLine(leftPoint);

                final var line1b = fundamentalMatrixB.getLeftEpipolarLine(rightPoint);
                final var line2b = fundamentalMatrixB.getRightEpipolarLine(leftPoint);

                // check that points lie on their corresponding epipolar lines
                assertTrue(line1a.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2a.isLocus(rightPoint, ABSOLUTE_ERROR));

                assertTrue(line1b.isLocus(leftPoint, ABSOLUTE_ERROR));
                assertTrue(line2b.isLocus(rightPoint, ABSOLUTE_ERROR));

                // back-project epipolar lines for each pair of cameras and check
                // that each pair of lines correspond to the same epipolar plane
                final var epipolarPlane1a = camera1.backProject(line1a);
                final var epipolarPlane2a = camera2.backProject(line2a);

                final var epipolarPlane1b = camera1b.backProject(line1b);
                final var epipolarPlane2b = camera2b.backProject(line2b);

                assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

                // check that 3D point and both camera centers for each pair of
                // cameras belong to their corresponding epipolar plane
                if (epipolarPlane1a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid1a++;
                }
                assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));

                if (epipolarPlane2a.isLocus(worldPoint, ABSOLUTE_ERROR)) {
                    numValid2a++;
                }
                assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

                // notice that since estimated cameras have an arbitrary scale,
                // original world point doesn't need to lie on epipolar plane
                // because first a scale transformation needs to be done
                scaledWorldPoint = scaleTransformation.transformAndReturnNew(worldPoint);
                if (epipolarPlane1a.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid1b++;
                }
                assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));

                if (epipolarPlane2b.isLocus(scaledWorldPoint, ABSOLUTE_ERROR)) {
                    numValid2b++;
                }
                assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
                assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR));

                // recover scale in triangulated point
                scaledTriangulatedPoint = invScaleTransformation.transformAndReturnNew(triangulatedPoint);

                // check that triangulated point after recovering scale matches
                // original point
                if (worldPoint.equals(scaledTriangulatedPoint, LARGE_ABSOLUTE_ERROR)) {
                    numValidEqual++;
                }
            }

            if (numValid1a > 0 && numValid2a > 0 && numValid1b > 0 && numValid2b > 0 && numValidEqual > 0) {
                numValidTimes++;
            }

            // recover scale of cameras by undoing their transformations
            final var camera1c = invScaleTransformation.transformAndReturnNew(camera1b);
            final var camera2c = invScaleTransformation.transformAndReturnNew(camera2b);

            // check that now cameras are equal to the original ones
            camera1.normalize();
            camera2.normalize();
            camera1c.normalize();
            camera2c.normalize();

            final var camera1Matrix = camera1.getInternalMatrix();
            final var camera1cMatrix = camera1c.getInternalMatrix();
            assertTrue(camera1Matrix.equals(camera1cMatrix, ABSOLUTE_ERROR));

            final var camera2Matrix = camera2.getInternalMatrix();
            final var camera2cMatrix = camera2c.getInternalMatrix();
            assertTrue(camera2Matrix.equals(camera2cMatrix, LARGE_ABSOLUTE_ERROR));

            if (numValidTimes > 0) {
                break;
            }
        }

        assertTrue(numValidTimes > 0);
    }

    @Override
    public void onStart(final InitialCamerasEstimator estimator) {
        checkLocked((DualImageOfAbsoluteConicInitialCamerasEstimator) estimator);
    }

    @Override
    public void onFinish(final InitialCamerasEstimator estimator, final PinholeCamera estimatedLeftCamera,
                         final PinholeCamera estimatedRightCamera) {
        checkLocked((DualImageOfAbsoluteConicInitialCamerasEstimator) estimator);
    }

    @Override
    public void onFail(final InitialCamerasEstimator estimator, final InitialCamerasEstimationFailedException e) {
        checkLocked((DualImageOfAbsoluteConicInitialCamerasEstimator) estimator);
    }

    private static void checkLocked(final DualImageOfAbsoluteConicInitialCamerasEstimator estimator) {
        assertThrows(com.irurueta.geometry.estimators.LockedException.class, estimator::estimate);
        assertThrows(com.irurueta.geometry.estimators.LockedException.class, () -> estimator.setAspectRatio(-1.0));
        assertThrows(com.irurueta.geometry.estimators.LockedException.class, () -> estimator.setPrincipalPointX(0.0));
        assertThrows(com.irurueta.geometry.estimators.LockedException.class, () -> estimator.setPrincipalPointY(0.0));
        assertThrows(com.irurueta.geometry.estimators.LockedException.class,
                () -> estimator.setPrincipalPoint(0.0, 0.0));
        assertThrows(com.irurueta.geometry.estimators.LockedException.class, () -> estimator.setLeftPoints(null));
        assertThrows(com.irurueta.geometry.estimators.LockedException.class, () -> estimator.setRightPoints(null));
        assertThrows(com.irurueta.geometry.estimators.LockedException.class,
                () -> estimator.setLeftAndRightPoints(null, null));
        assertThrows(com.irurueta.geometry.estimators.LockedException.class, () -> estimator.setCorrectorType(null));
        assertThrows(com.irurueta.geometry.estimators.LockedException.class,
                () -> estimator.setPointsTriangulated(true));
        assertThrows(com.irurueta.geometry.estimators.LockedException.class,
                () -> estimator.setValidTriangulatedPointsMarked(true));
        assertThrows(com.irurueta.geometry.estimators.LockedException.class,
                () -> estimator.setFundamentalMatrix(null));
    }
}
