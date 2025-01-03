/*
 * Copyright (C) 2015 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.ar.epipolar;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.LockedException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.NotReadyException;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.ar.SerializationHelper;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.ProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class FundamentalMatrixTest {

    private static final int FUND_MATRIX_ROWS = 3;
    private static final int FUND_MATRIX_COLS = 3;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-1;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;

    private static final double MIN_SKEWNESS = -1.0;
    private static final double MAX_SKEWNESS = 1.0;

    private static final double MIN_PRINCIPAL_POINT = 0.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;

    private static final double MIN_ANGLE_DEGREES = -30.0;
    private static final double MAX_ANGLE_DEGREES = 30.0;

    private static final double MIN_CAMERA_SEPARATION = 5.0;
    private static final double MAX_CAMERA_SEPARATION = 10.0;

    private static final int MIN_NUM_POINTS = 25;
    private static final int MAX_NUM_POINTS = 50;

    private static final int MAX_TRIES = 5000;

    private static final int TIMES = 100;

    @Test
    void testConstructor() throws GeometryException, AlgebraException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            // test empty constructor
            var fundMatrix = new FundamentalMatrix();
            assertFalse(fundMatrix.isInternalMatrixAvailable());
            assertThrows(NotAvailableException.class, fundMatrix::getInternalMatrix);

            // test constructor with internal matrix

            // create a valid 3x3 rank 2 matrix
            final var a = Matrix.createWithUniformRandomValues(FUND_MATRIX_ROWS, FUND_MATRIX_COLS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);

            final var decomposer = new SingularValueDecomposer(a);

            decomposer.decompose();

            final var u = decomposer.getU();
            final var w = decomposer.getW();
            final var v = decomposer.getV();

            // transpose V
            final var transV = v.transposeAndReturnNew();

            // Set last singular value to zero to enforce rank 2
            w.setElementAt(2, 2, 0.0);

            final var fundamentalInternalMatrix1 = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));

            fundMatrix = new FundamentalMatrix(fundamentalInternalMatrix1);
            assertTrue(fundMatrix.isInternalMatrixAvailable());
            assertEquals(fundamentalInternalMatrix1, fundMatrix.getInternalMatrix());
            assertNotSame(fundamentalInternalMatrix1, fundMatrix.getInternalMatrix());

            // Force InvalidFundamentalMatrixException

            // try with a non 3x3 matrix
            final var fundamentalInternalMatrix2 = new Matrix(FUND_MATRIX_ROWS + 1, FUND_MATRIX_COLS + 1);
            assertThrows(InvalidFundamentalMatrixException.class,
                    () -> new FundamentalMatrix(fundamentalInternalMatrix2));

            // try with a non rank-2 3x3 matrix
            var fundamentalInternalMatrix3 = Matrix.createWithUniformRandomValues(FUND_MATRIX_ROWS, FUND_MATRIX_COLS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            while (Utils.rank(fundamentalInternalMatrix3) == 2) {
                fundamentalInternalMatrix3 = Matrix.createWithUniformRandomValues(FUND_MATRIX_ROWS, FUND_MATRIX_COLS,
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            }

            final var wrongFundamentalInternalMatrix = fundamentalInternalMatrix3;
            assertThrows(InvalidFundamentalMatrixException.class,
                    () -> new FundamentalMatrix(wrongFundamentalInternalMatrix));

            // test constructor by providing two pinhole cameras
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final var skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final var horizontalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var horizontalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var center1 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                    horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
            final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                    horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

            final var camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            fundMatrix = new FundamentalMatrix(camera1, camera2);

            // check correctness by checking generated epipolar geometry

            // compute epipoles
            final var epipole1a = camera1.project(center2);
            final var epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            final var epipole1b = fundMatrix.getLeftEpipole();
            final var epipole2b = fundMatrix.getRightEpipole();

            if (epipole1a.distanceTo(epipole1b) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1a.distanceTo(epipole1b), LARGE_ABSOLUTE_ERROR);
            if (epipole2a.distanceTo(epipole2b) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2a.distanceTo(epipole2b), LARGE_ABSOLUTE_ERROR);

            // generate a random 3D point
            final var point3D = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            // project 3D point with both cameras
            // left view
            final var point2D1 = camera1.project(point3D);
            // right view
            final var point2D2 = camera2.project(point3D);

            // Obtain epipolar line on left view using 2D point on right view
            final var line1 = fundMatrix.getLeftEpipolarLine(point2D2);
            // Obtain epipolar line on right view using 2D point on left view
            final var line2 = fundMatrix.getRightEpipolarLine(point2D1);

            // check that 2D point on left view belongs to left epipolar line
            assertTrue(line1.isLocus(point2D1, ABSOLUTE_ERROR));

            // check that 2D point on right view belongs to right epipolar line
            assertTrue(line2.isLocus(point2D2, ABSOLUTE_ERROR));

            // back-project epipolar lines and check that both produce the same
            // epipolar plane
            final var epipolarPlane1 = camera1.backProject(line1);
            final var epipolarPlane2 = camera2.backProject(line2);

            assertTrue(epipolarPlane1.equals(epipolarPlane2, ABSOLUTE_ERROR));

            // check that projected 3D point and both camera centers belong to
            // epipolar plane
            if (!epipolarPlane1.isLocus(point3D, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1.isLocus(point3D, ABSOLUTE_ERROR));
            if (!epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR));
            if (!epipolarPlane1.isLocus(center2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1.isLocus(center2, ABSOLUTE_ERROR));

            if (!epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR));
            if (!epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR));
            if (!epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));

            // test with homography and right epipole
            final var fundMatrix2 = new FundamentalMatrix();
            final var leftPoints = new ArrayList<Point2D>();
            final var rightPoints = new ArrayList<Point2D>();
            Transformation2D homography = null;
            try {
                homography = generateHomography(camera1, camera2, fundMatrix2, leftPoints, rightPoints);
            } catch (final Exception ignore) {
                // no action needed
            }
            if (homography == null) {
                continue;
            }

            fundMatrix2.computeEpipoles();
            final var leftEpipole2 = fundMatrix2.getLeftEpipole();
            final var rightEpipole2 = fundMatrix2.getRightEpipole();

            fundMatrix = new FundamentalMatrix(homography, rightEpipole2);

            fundMatrix.normalize();
            fundMatrix2.normalize();

            // check correctness
            if (!fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix(), VERY_LARGE_ABSOLUTE_ERROR)
                    && !fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                    .multiplyByScalarAndReturnNew(-1.0), VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundMatrix.getInternalMatrix().equals(
                    fundMatrix2.getInternalMatrix(), VERY_LARGE_ABSOLUTE_ERROR)
                    || fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                    .multiplyByScalarAndReturnNew(-1.0), VERY_LARGE_ABSOLUTE_ERROR));

            fundMatrix.computeEpipoles();
            final var leftEpipole = fundMatrix.getLeftEpipole();
            final var rightEpipole = fundMatrix.getRightEpipole();

            if (!leftEpipole.equals(leftEpipole2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(leftEpipole.equals(leftEpipole2, LARGE_ABSOLUTE_ERROR));

            if (!rightEpipole.equals(rightEpipole2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(rightEpipole.equals(rightEpipole2, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }
        assertTrue(numValid > 0);
    }

    @Test
    void testGetSetInternalMatrixAndAvailability() throws WrongSizeException, NotReadyException, LockedException,
            DecomposerException, com.irurueta.algebra.NotAvailableException, InvalidFundamentalMatrixException,
            NotAvailableException {
        // create a valid 3x3 rank 2 matrix
        final var a = Matrix.createWithUniformRandomValues(FUND_MATRIX_ROWS, FUND_MATRIX_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var u = decomposer.getU();
        final var w = decomposer.getW();
        final var v = decomposer.getV();

        // transpose V
        final var transV = v.transposeAndReturnNew();

        // set last singular value to zero to enforce rank 2
        w.setElementAt(2, 2, 0.0);

        final var fundamentalInternalMatrix1 = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));

        // Instantiate empty fundamental matrix
        final var fundMatrix = new FundamentalMatrix();

        assertFalse(fundMatrix.isInternalMatrixAvailable());
        assertThrows(NotAvailableException.class, fundMatrix::getInternalMatrix);

        // set internal matrix
        fundMatrix.setInternalMatrix(fundamentalInternalMatrix1);

        // Check correctness
        assertTrue(fundMatrix.isInternalMatrixAvailable());
        assertEquals(fundamentalInternalMatrix1, fundMatrix.getInternalMatrix());
        assertNotSame(fundamentalInternalMatrix1, fundMatrix.getInternalMatrix());

        // Force InvalidFundamentalMatrixException

        // try with a non 3x3 matrix
        final var fundamentalInternalMatrix2 = new Matrix(FUND_MATRIX_ROWS + 1, FUND_MATRIX_COLS + 1);
        assertThrows(InvalidFundamentalMatrixException.class,
                () -> fundMatrix.setInternalMatrix(fundamentalInternalMatrix2));

        // try with a non rank-2 3x3 matrix
        var fundamentalInternalMatrix3 = Matrix.createWithUniformRandomValues(FUND_MATRIX_ROWS, FUND_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while (Utils.rank(fundamentalInternalMatrix3) == 2) {
            fundamentalInternalMatrix3 = Matrix.createWithUniformRandomValues(FUND_MATRIX_ROWS, FUND_MATRIX_COLS,
                    MIN_RANDOM_VALUE, MIN_RANDOM_VALUE);
        }

        final var wrongFundamentalInternalMatrix = fundamentalInternalMatrix3;
        assertThrows(InvalidFundamentalMatrixException.class,
                () -> fundMatrix.setInternalMatrix(wrongFundamentalInternalMatrix));
    }

    @Test
    void testSetFromPairOfCameras() throws InvalidPairOfCamerasException,
            com.irurueta.geometry.estimators.NotReadyException, InvalidFundamentalMatrixException,
            NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final var skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final var horizontalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var horizontalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var center1 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                    horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
            final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                    horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

            final var camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            final var fundMatrix = new FundamentalMatrix();

            // check default values
            assertFalse(fundMatrix.isInternalMatrixAvailable());

            fundMatrix.setFromPairOfCameras(camera1, camera2);

            // check correctness by checking generated epipolar geometry

            assertTrue(fundMatrix.isInternalMatrixAvailable());

            // compute epipoles
            final var epipole1a = camera1.project(center2);
            final var epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            final var epipole1b = fundMatrix.getLeftEpipole();
            final var epipole2b = fundMatrix.getRightEpipole();

            if (epipole1a.distanceTo(epipole1b) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1a.distanceTo(epipole1b), LARGE_ABSOLUTE_ERROR);
            if (epipole2a.distanceTo(epipole2b) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2a.distanceTo(epipole2b), LARGE_ABSOLUTE_ERROR);

            // generate a random 3D point
            final var point3D = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            // project 3D point with both cameras
            // left view
            final var point2D1 = camera1.project(point3D);
            // right view
            final var point2D2 = camera2.project(point3D);

            // Obtain epipolar line on left view using 2D point on right view
            final var line1 = fundMatrix.getLeftEpipolarLine(point2D2);
            // Obtain epipolar line on right view using 2D point on left view
            final var line2 = fundMatrix.getRightEpipolarLine(point2D1);

            // check that 2D point on left view belongs to left epipolar line
            assertTrue(line1.isLocus(point2D1, ABSOLUTE_ERROR));

            // check that 2D point on right view belongs to right epipolar line
            assertTrue(line2.isLocus(point2D2, ABSOLUTE_ERROR));

            // back-project epipolar lines and check that both produce the same
            // epipolar plane
            final var epipolarPlane1 = camera1.backProject(line1);
            final var epipolarPlane2 = camera2.backProject(line2);

            assertTrue(epipolarPlane1.equals(epipolarPlane2, ABSOLUTE_ERROR));

            // check that 3D point and both camera centers belong to
            // epipolar plane
            assertTrue(epipolarPlane1.isLocus(point3D, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane1.isLocus(center2, ABSOLUTE_ERROR));

            assertTrue(epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSetFromHomography() throws GeometryException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var fundMatrix2 = new FundamentalMatrix();
            final var leftPoints = new ArrayList<Point2D>();
            final var rightPoints = new ArrayList<Point2D>();
            final var camera1 = new PinholeCamera();
            final var camera2 = new PinholeCamera();
            Transformation2D homography = null;
            try {
                homography = generateHomography(camera1, camera2, fundMatrix2, leftPoints, rightPoints);
            } catch (final Exception ignore) {
                // no action needed
            }
            if (homography == null) {
                continue;
            }

            fundMatrix2.computeEpipoles();
            final var leftEpipole2 = fundMatrix2.getLeftEpipole();
            final var rightEpipole2 = fundMatrix2.getRightEpipole();

            final var fundMatrix = new FundamentalMatrix();
            fundMatrix.setFromHomography(homography, rightEpipole2);

            fundMatrix.normalize();
            fundMatrix2.normalize();

            // check correctness
            if (!fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix(), VERY_LARGE_ABSOLUTE_ERROR)
                    && !fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                    .multiplyByScalarAndReturnNew(-1.0), VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundMatrix.getInternalMatrix().equals(
                    fundMatrix2.getInternalMatrix(), VERY_LARGE_ABSOLUTE_ERROR)
                    || fundMatrix.getInternalMatrix().equals(fundMatrix2.getInternalMatrix()
                    .multiplyByScalarAndReturnNew(-1.0), VERY_LARGE_ABSOLUTE_ERROR));

            fundMatrix.computeEpipoles();
            final var leftEpipole = fundMatrix.getLeftEpipole();
            final var rightEpipole = fundMatrix.getRightEpipole();

            if (!leftEpipole.equals(leftEpipole2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(leftEpipole.equals(leftEpipole2, LARGE_ABSOLUTE_ERROR));

            if (!rightEpipole.equals(rightEpipole2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(rightEpipole.equals(rightEpipole2, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }
        assertTrue(numValid > 0);
    }

    @Test
    void testGetAndComputeEpipolesAndEpipolarLinesAndCheckAvailability() throws InvalidPairOfCamerasException,
            com.irurueta.geometry.estimators.NotReadyException, InvalidFundamentalMatrixException,
            NotAvailableException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            // provide two pinhole cameras
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final var skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final var horizontalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var horizontalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var center1 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                    horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
            final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                    horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

            final var camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            final var fundMatrix = new FundamentalMatrix(camera1, camera2);

            // compute epipoles
            final var epipole1a = camera1.project(center2);
            final var epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            final var epipole1b = fundMatrix.getLeftEpipole();
            final var epipole2b = fundMatrix.getRightEpipole();

            if (epipole1a.distanceTo(epipole1b) > 10.0 * ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1a.distanceTo(epipole1b), 10.0 * ABSOLUTE_ERROR);
            if (epipole2a.distanceTo(epipole2b) > 10.0 * ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2a.distanceTo(epipole2b), 10.0 * ABSOLUTE_ERROR);

            // generate a random 3D point
            final var point3D = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            // project 3D point with both cameras
            // left view
            final var point2D1 = camera1.project(point3D);
            // right view
            final var point2D2 = camera2.project(point3D);

            // Obtain epipolar line on left view using 2D point on right view
            final var line1 = fundMatrix.getLeftEpipolarLine(point2D2);
            // Obtain epipolar line on right view using 2D point on left view
            final var line2 = fundMatrix.getRightEpipolarLine(point2D1);

            final var line1b = new Line2D();
            fundMatrix.leftEpipolarLine(point2D2, line1b);
            final var line2b = new Line2D();
            fundMatrix.rightEpipolarLine(point2D1, line2b);

            // check that 2D point on left view belongs to left epipolar line
            assertTrue(line1.isLocus(point2D1, ABSOLUTE_ERROR));
            assertTrue(line1b.isLocus(point2D1, ABSOLUTE_ERROR));

            // check that both lines are equal
            assertTrue(line1.equals(line1b, ABSOLUTE_ERROR));

            // check that 2D point on right view belongs to right epipolar line
            assertTrue(line2.isLocus(point2D2, ABSOLUTE_ERROR));
            assertTrue(line2b.isLocus(point2D2, ABSOLUTE_ERROR));

            // check that both lines are equal
            assertTrue(line2.equals(line2b, ABSOLUTE_ERROR));

            // back-project epipolar lines and check that both produce the same
            // epipolar plane
            final var epipolarPlane1 = camera1.backProject(line1);
            final var epipolarPlane2 = camera2.backProject(line2);

            assertTrue(epipolarPlane1.equals(epipolarPlane2, ABSOLUTE_ERROR));

            // check that projected 3D point and both camera centers belong to
            // epipolar plane
            if (!epipolarPlane1.isLocus(point3D, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1.isLocus(point3D, ABSOLUTE_ERROR));
            if (!epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1.isLocus(center1, ABSOLUTE_ERROR));
            if (!epipolarPlane1.isLocus(center2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1.isLocus(center2, ABSOLUTE_ERROR));

            if (!epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2.isLocus(point3D, ABSOLUTE_ERROR));
            if (!epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2.isLocus(center1, ABSOLUTE_ERROR));
            if (!epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2.isLocus(center2, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testNormalizeAndIsNormalized() throws InvalidPairOfCamerasException,
            com.irurueta.geometry.estimators.NotReadyException, NotAvailableException {
        // provide two pinhole cameras
        final var randomizer = new UniformRandomizer();
        final var alphaEuler1 = 0.0;
        final var betaEuler1 = 0.0;
        final var gammaEuler1 = 0.0;
        final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final var skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final var skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var horizontalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

        final var center1 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

        final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
        final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

        final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
        final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

        final var camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
        final var camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

        final var fundMatrix = new FundamentalMatrix(camera1, camera2);

        assertFalse(fundMatrix.isNormalized());

        // normalize
        fundMatrix.normalize();

        // check correctness
        assertTrue(fundMatrix.isNormalized());

        // check that internal matrix has Frobenius norm equal to 1
        final var internalMatrix = fundMatrix.getInternalMatrix();

        assertEquals(1.0, Utils.normF(internalMatrix), ABSOLUTE_ERROR);
    }

    @Test
    void testIsValidInternalMatrix() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException {

        // create a valid 3x3 rank 2 matrix
        final var a = Matrix.createWithUniformRandomValues(FUND_MATRIX_ROWS, FUND_MATRIX_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var u = decomposer.getU();
        final var w = decomposer.getW();
        final var v = decomposer.getV();

        // transpose V
        final var transV = v.transposeAndReturnNew();

        // Set last singular value to zero to enforce rank 2
        w.setElementAt(2, 2, 0.0);

        var fundamentalInternalMatrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));

        assertTrue(FundamentalMatrix.isValidInternalMatrix(fundamentalInternalMatrix));

        // try with a non 3x3 matrix
        fundamentalInternalMatrix = new Matrix(FUND_MATRIX_ROWS + 1, FUND_MATRIX_COLS + 1);
        assertFalse(FundamentalMatrix.isValidInternalMatrix(fundamentalInternalMatrix));

        // try with a non rank-2 3x3 matrix
        fundamentalInternalMatrix = Matrix.createWithUniformRandomValues(FUND_MATRIX_ROWS, FUND_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while (Utils.rank(fundamentalInternalMatrix) == 2) {
            fundamentalInternalMatrix = Matrix.createWithUniformRandomValues(FUND_MATRIX_ROWS, FUND_MATRIX_COLS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        }

        assertFalse(FundamentalMatrix.isValidInternalMatrix(fundamentalInternalMatrix));
    }

    @Test
    void testGenerateCamerasInArbitraryProjectiveSpace() throws InvalidPairOfCamerasException,
            InvalidFundamentalMatrixException, com.irurueta.geometry.estimators.NotReadyException,
            NotAvailableException, CameraException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var alphaEuler1 = 0.0;
            final var betaEuler1 = 0.0;
            final var gammaEuler1 = 0.0;
            final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final var skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final var skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final var horizontalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var horizontalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final var verticalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var center1 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                    horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
            final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                    horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

            final var camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
            final var camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

            final var fundMatrix = new FundamentalMatrix();

            // check default values
            assertFalse(fundMatrix.isInternalMatrixAvailable());

            fundMatrix.setFromPairOfCameras(camera1, camera2);
            fundMatrix.normalize();

            // obtain arbitrary pair of cameras
            final var referencePlaneDirectorVectorX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var referencePlaneDirectorVectorY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var referencePlaneDirectorVectorZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var scaleFactor = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
            final var camera1b = new PinholeCamera();
            final var camera2b = new PinholeCamera();
            fundMatrix.generateCamerasInArbitraryProjectiveSpace(camera1b, camera2b, referencePlaneDirectorVectorX,
                    referencePlaneDirectorVectorY, referencePlaneDirectorVectorZ, scaleFactor);

            // compute fundamental matrix for arbitrary cameras
            final var fundMatrix2 = new FundamentalMatrix();

            fundMatrix2.setFromPairOfCameras(camera1b, camera2b);
            fundMatrix2.normalize();

            // obtain arbitrary pair of cameras for plane at infinity as reference
            // plane and unitary scale factor
            final var camera1c = new PinholeCamera();
            final var camera2c = new PinholeCamera();
            fundMatrix.generateCamerasInArbitraryProjectiveSpace(camera1c, camera2c);

            // compute fundamental matrix for arbitrary cameras
            final var fundMatrix3 = new FundamentalMatrix();

            fundMatrix3.setFromPairOfCameras(camera1c, camera2c);
            fundMatrix3.normalize();

            // check correctness by checking generated epipolar geometry

            assertTrue(fundMatrix.isInternalMatrixAvailable());
            assertTrue(fundMatrix2.isInternalMatrixAvailable());
            assertTrue(fundMatrix3.isInternalMatrixAvailable());

            // compute epipoles
            final var epipole1 = camera1.project(center2);
            final var epipole2 = camera2.project(center1);

            fundMatrix.computeEpipoles();
            fundMatrix2.computeEpipoles();
            fundMatrix3.computeEpipoles();

            final var epipole1a = fundMatrix.getLeftEpipole();
            final var epipole2a = fundMatrix.getRightEpipole();

            final var epipole1b = fundMatrix2.getLeftEpipole();
            final var epipole2b = fundMatrix2.getRightEpipole();

            final var epipole1c = fundMatrix3.getLeftEpipole();
            final var epipole2c = fundMatrix3.getRightEpipole();

            if (epipole1.distanceTo(epipole1a) > 5.0 * LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1.distanceTo(epipole1a), 5.0 * LARGE_ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2a) > 5.0 * LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2.distanceTo(epipole2a), 5.0 * LARGE_ABSOLUTE_ERROR);

            if (epipole1.distanceTo(epipole1b) > 5.0 * LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1.distanceTo(epipole1b), 5.0 * LARGE_ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2b) > 5.0 * LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2.distanceTo(epipole2b), 5.0 * LARGE_ABSOLUTE_ERROR);

            if (epipole1.distanceTo(epipole1c) > 5.0 * LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole1.distanceTo(epipole1c), 5.0 * LARGE_ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2c) > 5.0 * LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(0.0, epipole2.distanceTo(epipole2c), 5.0 * LARGE_ABSOLUTE_ERROR);

            // generate a random 3D points
            final var point3Da = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var point3Db = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var point3Dc = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            // project 3D points with each pair of cameras
            final var point2D1a = camera1.project(point3Da);
            final var point2D2a = camera2.project(point3Da);

            final var point2D1b = camera1b.project(point3Db);
            final var point2D2b = camera2b.project(point3Db);

            final var point2D1c = camera1c.project(point3Dc);
            final var point2D2c = camera2c.project(point3Dc);

            // obtain epipolar lines
            final var line1a = fundMatrix.getLeftEpipolarLine(point2D2a);
            final var line2a = fundMatrix.getRightEpipolarLine(point2D1a);

            final var line1b = fundMatrix2.getLeftEpipolarLine(point2D2b);
            final var line2b = fundMatrix2.getRightEpipolarLine(point2D1b);

            final var line1c = fundMatrix3.getLeftEpipolarLine(point2D2c);
            final var line2c = fundMatrix3.getRightEpipolarLine(point2D1c);

            // check that points lie on their corresponding epipolar lines
            if (!line1a.isLocus(point2D1a, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(line1a.isLocus(point2D1a, ABSOLUTE_ERROR));
            if (!line2a.isLocus(point2D2a, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(line2a.isLocus(point2D2a, ABSOLUTE_ERROR));

            if (!line1b.isLocus(point2D1b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(line1b.isLocus(point2D1b, ABSOLUTE_ERROR));
            if (!line2b.isLocus(point2D2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(line2b.isLocus(point2D2b, ABSOLUTE_ERROR));

            if (!line1c.isLocus(point2D1c, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(line1c.isLocus(point2D1c, ABSOLUTE_ERROR));
            if (!line2c.isLocus(point2D2c, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(line2c.isLocus(point2D2c, ABSOLUTE_ERROR));

            // back-project epipolar lines for each pair of cameras and check that
            // each pair of lines correspond to the same epipolar plane
            final var epipolarPlane1a = camera1.backProject(line1a);
            final var epipolarPlane2a = camera2.backProject(line2a);

            final var epipolarPlane1b = camera1b.backProject(line1b);
            final var epipolarPlane2b = camera2b.backProject(line2b);

            final var epipolarPlane1c = camera1c.backProject(line1c);
            final var epipolarPlane2c = camera2c.backProject(line2c);

            assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane1c.equals(epipolarPlane2c, ABSOLUTE_ERROR));

            // check that 3D point and both camera centers for each pair of cameras
            // belong to their corresponding epipolar plane
            assertTrue(epipolarPlane1a.isLocus(point3Da, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));

            assertTrue(epipolarPlane2a.isLocus(point3Da, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

            camera1b.decompose(false, true);
            camera2b.decompose(false, true);
            final var center1b = camera1b.getCameraCenter();
            final var center2b = camera2b.getCameraCenter();

            if (!epipolarPlane1b.isLocus(point3Db, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1b.isLocus(point3Db, ABSOLUTE_ERROR));
            if (!epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
            if (!epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));

            if (!epipolarPlane2b.isLocus(point3Db, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2b.isLocus(point3Db, ABSOLUTE_ERROR));
            if (!epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
            if (!epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR));

            camera1c.decompose(false, true);
            camera2c.decompose(false, true);
            final var center1c = camera1c.getCameraCenter();
            final var center2c = camera2c.getCameraCenter();

            if (!epipolarPlane1c.isLocus(point3Dc, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1c.isLocus(point3Dc, ABSOLUTE_ERROR));
            if (!epipolarPlane1c.isLocus(center1c, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1c.isLocus(center1c, ABSOLUTE_ERROR));
            if (!epipolarPlane1c.isLocus(center2c, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1c.isLocus(center2c, ABSOLUTE_ERROR));

            if (!epipolarPlane2c.isLocus(point3Dc, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2c.isLocus(point3Dc, ABSOLUTE_ERROR));
            if (!epipolarPlane2c.isLocus(center1c, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2c.isLocus(center1c, ABSOLUTE_ERROR));
            if (!epipolarPlane2c.isLocus(center2c, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2c.isLocus(center2c, ABSOLUTE_ERROR));

            numValid++;
            break;
        }
        assertTrue(numValid > 0);
    }

    @Test
    void testSerializeDeserialize() throws GeometryException, IOException, ClassNotFoundException {
        // create fundamental matrix by providing two pinhole cameras
        final var randomizer = new UniformRandomizer();
        final var alphaEuler1 = 0.0;
        final var betaEuler1 = 0.0;
        final var gammaEuler1 = 0.0;
        final var alphaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final var skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final var skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint1 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var horizontalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint2 = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

        final var center1 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

        final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
        final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

        final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
        final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

        final var camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
        final var camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

        final var fundMatrix1 = new FundamentalMatrix(camera1, camera2);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(fundMatrix1);
        final var fundMatrix2 = SerializationHelper.<FundamentalMatrix>deserialize(bytes);

        // check
        assertEquals(fundMatrix1.getInternalMatrix(), fundMatrix2.getInternalMatrix());
    }

    private static Transformation2D generateHomography(final PinholeCamera camera1, final PinholeCamera camera2,
            final FundamentalMatrix fundamentalMatrix, final List<Point2D> projectedPoints1,
            final List<Point2D> projectedPoints2) throws GeometryException, AlgebraException, RobustEstimatorException {

        final var randomizer = new UniformRandomizer();
        final var focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var aspectRatio = 1.0;
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

        final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

        final var center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
        final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

        final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
        final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

        camera1.setIntrinsicAndExtrinsicParameters(intrinsic, rotation1, center1);
        camera2.setIntrinsicAndExtrinsicParameters(intrinsic, rotation2, center2);

        fundamentalMatrix.setFromPairOfCameras(camera1, camera2);

        // create 3D points laying in front of both cameras an in a plane
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
        final var centralCommonPoint = new HomogeneousPoint3D(v.getElementAt(0, 3),
                v.getElementAt(1, 3), v.getElementAt(2, 3), v.getElementAt(3, 3));

        final var principalAxis1 = camera1.getPrincipalAxisArray();
        final var principalAxis2 = camera2.getPrincipalAxisArray();
        final var avgPrincipalAxis = ArrayUtils.multiplyByScalarAndReturnNew(
                ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2), 0.5);

        final var plane = new Plane(centralCommonPoint, avgPrincipalAxis);
        plane.normalize();

        final var planeA = plane.getA();
        final var planeB = plane.getB();
        final var planeC = plane.getC();
        final var planeD = plane.getD();

        final var numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

        HomogeneousPoint3D point3D;
        projectedPoints1.clear();
        projectedPoints2.clear();
        boolean front1;
        boolean front2;
        for (var i = 0; i < numPoints; i++) {
            // generate points and ensure they lie in front of both cameras
            var numTry = 0;
            do {
                // get a random point belonging to the plane
                // a*x + b*y + c*z + d*w = 0
                // y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                final double homX;
                final double homY;
                final var homW = 1.0;
                final var homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homY = -(planeA * homX + planeC * homZ + planeD * homW) / planeB;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                    homX = -(planeB * homY + planeC * homZ + planeD * homW) / planeA;
                }

                point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);

                assertTrue(plane.isLocus(point3D));

                front1 = camera1.isPointInFrontOfCamera(point3D);
                front2 = camera2.isPointInFrontOfCamera(point3D);
                if (numTry > MAX_TRIES) {
                    return null;
                }
                numTry++;
            } while (!front1 || !front2);

            // here 3D point is in front of both cameras

            // project 3D point into both cameras
            projectedPoints1.add(camera1.project(point3D));
            projectedPoints2.add(camera2.project(point3D));
        }

        // estimate homography
        final var homographyEstimator = ProjectiveTransformation2DRobustEstimator.createFromPoints(projectedPoints1,
                projectedPoints2, RobustEstimatorMethod.LMEDS);

        return homographyEstimator.estimate();
    }
}
