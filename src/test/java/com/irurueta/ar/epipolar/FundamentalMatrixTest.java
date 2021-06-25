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
import org.junit.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class FundamentalMatrixTest {

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
    public void testConstructor() throws GeometryException, AlgebraException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            // test empty constructor
            FundamentalMatrix fundMatrix = new FundamentalMatrix();
            assertFalse(fundMatrix.isInternalMatrixAvailable());
            try {
                fundMatrix.getInternalMatrix();
                fail("NotAvailableException expected but not thrown");
            } catch (final NotAvailableException ignore) {
            }

            // test constructor with internal matrix

            // create a valid 3x3 rank 2 matrix
            final Matrix a = Matrix.createWithUniformRandomValues(FUND_MATRIX_ROWS,
                    FUND_MATRIX_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

            decomposer.decompose();

            final Matrix u = decomposer.getU();
            final Matrix w = decomposer.getW();
            final Matrix v = decomposer.getV();

            // transpose V
            final Matrix transV = v.transposeAndReturnNew();

            // Set last singular value to zero to enforce rank 2
            w.setElementAt(2, 2, 0.0);

            Matrix fundamentalInternalMatrix = u.multiplyAndReturnNew(
                    w.multiplyAndReturnNew(transV));

            fundMatrix = new FundamentalMatrix(fundamentalInternalMatrix);
            assertTrue(fundMatrix.isInternalMatrixAvailable());
            assertEquals(fundamentalInternalMatrix, fundMatrix.getInternalMatrix());
            assertNotSame(fundamentalInternalMatrix,
                    fundMatrix.getInternalMatrix());

            // Force InvalidFundamentalMatrixException

            // try with a non 3x3 matrix
            fundamentalInternalMatrix = new Matrix(FUND_MATRIX_ROWS + 1,
                    FUND_MATRIX_COLS + 1);

            fundMatrix = null;
            try {
                fundMatrix = new FundamentalMatrix(fundamentalInternalMatrix);
                fail("InvalidFundamentalMatrixException expected but not thrown");
            } catch (final InvalidFundamentalMatrixException ignore) {
            }
            assertNull(fundMatrix);

            // try with a non rank-2 3x3 matrix
            fundamentalInternalMatrix = Matrix.createWithUniformRandomValues(
                    FUND_MATRIX_ROWS, FUND_MATRIX_COLS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            while (Utils.rank(fundamentalInternalMatrix) == 2) {
                fundamentalInternalMatrix = Matrix.createWithUniformRandomValues(
                        FUND_MATRIX_ROWS, FUND_MATRIX_COLS, MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
            }

            fundMatrix = null;
            try {
                fundMatrix = new FundamentalMatrix(fundamentalInternalMatrix);
                fail("InvalidFundamentalMatrixException expected but not thrown");
            } catch (final InvalidFundamentalMatrixException ignore) {
            }
            assertNull(fundMatrix);

            // test constructor by providing two pinhole cameras
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final double cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION,
                    MAX_CAMERA_SEPARATION);

            final Point3D center1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                    gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                    gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);

            fundMatrix = new FundamentalMatrix(camera1, camera2);

            // check correctness by checking generated epipolar geometry

            // compute epipoles
            final Point2D epipole1a = camera1.project(center2);
            final Point2D epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            final Point2D epipole1b = fundMatrix.getLeftEpipole();
            final Point2D epipole2b = fundMatrix.getRightEpipole();

            if (epipole1a.distanceTo(epipole1b) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole1a.distanceTo(epipole1b), 0.0,
                    LARGE_ABSOLUTE_ERROR);
            if (epipole2a.distanceTo(epipole2b) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole2a.distanceTo(epipole2b), 0.0,
                    LARGE_ABSOLUTE_ERROR);

            // generate a random 3D point
            final Point3D point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            // project 3D point with both cameras
            // left view
            final Point2D point2D1 = camera1.project(point3D);
            // right view
            final Point2D point2D2 = camera2.project(point3D);

            // Obtain epipolar line on left view using 2D point on right view
            final Line2D line1 = fundMatrix.getLeftEpipolarLine(point2D2);
            // Obtain epipolar line on right view using 2D point on left view
            final Line2D line2 = fundMatrix.getRightEpipolarLine(point2D1);

            // check that 2D point on left view belongs to left epipolar line
            assertTrue(line1.isLocus(point2D1, ABSOLUTE_ERROR));

            // check that 2D point on right view belongs to right epipolar line
            assertTrue(line2.isLocus(point2D2, ABSOLUTE_ERROR));

            // back-project epipolar lines and check that both produce the same
            // epipolar plane
            final Plane epipolarPlane1 = camera1.backProject(line1);
            final Plane epipolarPlane2 = camera2.backProject(line2);

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
            final FundamentalMatrix fundMatrix2 = new FundamentalMatrix();
            final List<Point2D> leftPoints = new ArrayList<>();
            final List<Point2D> rightPoints = new ArrayList<>();
            Transformation2D homography = null;
            try {
                homography = generateHomography(camera1, camera2,
                        fundMatrix2, leftPoints, rightPoints);
            } catch (final Exception ignore) {
            }
            if (homography == null) {
                continue;
            }

            fundMatrix2.computeEpipoles();
            final Point2D leftEpipole2 = fundMatrix2.getLeftEpipole();
            final Point2D rightEpipole2 = fundMatrix2.getRightEpipole();

            fundMatrix = new FundamentalMatrix(homography, rightEpipole2);

            fundMatrix.normalize();
            fundMatrix2.normalize();

            // check correctness
            if (!fundMatrix.getInternalMatrix().equals(
                    fundMatrix2.getInternalMatrix(), VERY_LARGE_ABSOLUTE_ERROR) &&
                    !fundMatrix.getInternalMatrix().equals(
                            fundMatrix2.getInternalMatrix().
                                    multiplyByScalarAndReturnNew(-1.0), VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundMatrix.getInternalMatrix().equals(
                    fundMatrix2.getInternalMatrix(), VERY_LARGE_ABSOLUTE_ERROR) ||
                    fundMatrix.getInternalMatrix().equals(
                            fundMatrix2.getInternalMatrix().
                                    multiplyByScalarAndReturnNew(-1.0), VERY_LARGE_ABSOLUTE_ERROR));

            fundMatrix.computeEpipoles();
            final Point2D leftEpipole = fundMatrix.getLeftEpipole();
            final Point2D rightEpipole = fundMatrix.getRightEpipole();

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
    public void testGetSetInternalMatrixAndAvailability()
            throws WrongSizeException, NotReadyException, LockedException,
            DecomposerException, com.irurueta.algebra.NotAvailableException,
            InvalidFundamentalMatrixException, NotAvailableException {
        // create a valid 3x3 rank 2 matrix
        final Matrix a = Matrix.createWithUniformRandomValues(FUND_MATRIX_ROWS,
                FUND_MATRIX_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final SingularValueDecomposer decomposer =
                new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix u = decomposer.getU();
        final Matrix w = decomposer.getW();
        final Matrix v = decomposer.getV();

        // transpose V
        final Matrix transV = v.transposeAndReturnNew();

        // set last singular value to zero to enforce rank 2
        w.setElementAt(2, 2, 0.0);

        Matrix fundamentalInternalMatrix = u.multiplyAndReturnNew(
                w.multiplyAndReturnNew(transV));

        // Instantiate empty fundamental matrix
        final FundamentalMatrix fundMatrix = new FundamentalMatrix();

        assertFalse(fundMatrix.isInternalMatrixAvailable());
        try {
            fundMatrix.getInternalMatrix();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }

        // set internal matrix
        fundMatrix.setInternalMatrix(fundamentalInternalMatrix);

        // Check correctness
        assertTrue(fundMatrix.isInternalMatrixAvailable());
        assertEquals(fundamentalInternalMatrix, fundMatrix.getInternalMatrix());
        assertNotSame(fundamentalInternalMatrix,
                fundMatrix.getInternalMatrix());

        // Force InvalidFundamentalMatrixException

        // try with a non 3x3 matrix
        fundamentalInternalMatrix = new Matrix(FUND_MATRIX_ROWS + 1,
                FUND_MATRIX_COLS + 1);
        try {
            fundMatrix.setInternalMatrix(fundamentalInternalMatrix);
            fail("InvalidFundamentalMatrixException expected but not thrown");
        } catch (final InvalidFundamentalMatrixException ignore) {
        }

        // try with a non rank-2 3x3 matrix
        fundamentalInternalMatrix = Matrix.createWithUniformRandomValues(
                FUND_MATRIX_ROWS, FUND_MATRIX_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        while (Utils.rank(fundamentalInternalMatrix) == 2) {
            fundamentalInternalMatrix = Matrix.createWithUniformRandomValues(
                    FUND_MATRIX_ROWS, FUND_MATRIX_COLS, MIN_RANDOM_VALUE,
                    MIN_RANDOM_VALUE);
        }

        try {
            fundMatrix.setInternalMatrix(fundamentalInternalMatrix);
            fail("InvalidFundamentalMatrixException expected but not thrown");
        } catch (final InvalidFundamentalMatrixException ignore) {
        }
    }

    @Test
    public void testSetFromPairOfCameras() throws InvalidPairOfCamerasException,
            com.irurueta.geometry.estimators.NotReadyException,
            InvalidFundamentalMatrixException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final double cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION,
                    MAX_CAMERA_SEPARATION);

            final Point3D center1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                    gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                    gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);

            final FundamentalMatrix fundMatrix = new FundamentalMatrix();

            // check default values
            assertFalse(fundMatrix.isInternalMatrixAvailable());

            fundMatrix.setFromPairOfCameras(camera1, camera2);

            // check correctness by checking generated epipolar geometry

            assertTrue(fundMatrix.isInternalMatrixAvailable());

            // compute epipoles
            final Point2D epipole1a = camera1.project(center2);
            final Point2D epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            final Point2D epipole1b = fundMatrix.getLeftEpipole();
            final Point2D epipole2b = fundMatrix.getRightEpipole();

            if (epipole1a.distanceTo(epipole1b) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole1a.distanceTo(epipole1b), 0.0, LARGE_ABSOLUTE_ERROR);
            if (epipole2a.distanceTo(epipole2b) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole2a.distanceTo(epipole2b), 0.0, LARGE_ABSOLUTE_ERROR);

            // generate a random 3D point
            final Point3D point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            // project 3D point with both cameras
            // left view
            final Point2D point2D1 = camera1.project(point3D);
            // right view
            final Point2D point2D2 = camera2.project(point3D);

            // Obtain epipolar line on left view using 2D point on right view
            final Line2D line1 = fundMatrix.getLeftEpipolarLine(point2D2);
            // Obtain epipolar line on right view using 2D point on left view
            final Line2D line2 = fundMatrix.getRightEpipolarLine(point2D1);

            // check that 2D point on left view belongs to left epipolar line
            assertTrue(line1.isLocus(point2D1, ABSOLUTE_ERROR));

            // check that 2D point on right view belongs to right epipolar line
            assertTrue(line2.isLocus(point2D2, ABSOLUTE_ERROR));

            // back-project epipolar lines and check that both produce the same
            // epipolar plane
            final Plane epipolarPlane1 = camera1.backProject(line1);
            final Plane epipolarPlane2 = camera2.backProject(line2);

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
    public void testSetFromHomography() throws GeometryException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final FundamentalMatrix fundMatrix2 = new FundamentalMatrix();
            final List<Point2D> leftPoints = new ArrayList<>();
            final List<Point2D> rightPoints = new ArrayList<>();
            final PinholeCamera camera1 = new PinholeCamera();
            final PinholeCamera camera2 = new PinholeCamera();
            Transformation2D homography = null;
            try {
                homography = generateHomography(camera1, camera2,
                        fundMatrix2, leftPoints, rightPoints);
            } catch (final Exception ignore) {
            }
            if (homography == null) {
                continue;
            }

            fundMatrix2.computeEpipoles();
            final Point2D leftEpipole2 = fundMatrix2.getLeftEpipole();
            final Point2D rightEpipole2 = fundMatrix2.getRightEpipole();

            final FundamentalMatrix fundMatrix = new FundamentalMatrix();
            fundMatrix.setFromHomography(homography, rightEpipole2);

            fundMatrix.normalize();
            fundMatrix2.normalize();

            // check correctness
            if (!fundMatrix.getInternalMatrix().equals(
                    fundMatrix2.getInternalMatrix(), VERY_LARGE_ABSOLUTE_ERROR) &&
                    !fundMatrix.getInternalMatrix().equals(
                            fundMatrix2.getInternalMatrix().
                                    multiplyByScalarAndReturnNew(-1.0), VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(fundMatrix.getInternalMatrix().equals(
                    fundMatrix2.getInternalMatrix(), VERY_LARGE_ABSOLUTE_ERROR) ||
                    fundMatrix.getInternalMatrix().equals(
                            fundMatrix2.getInternalMatrix().
                                    multiplyByScalarAndReturnNew(-1.0), VERY_LARGE_ABSOLUTE_ERROR));

            fundMatrix.computeEpipoles();
            final Point2D leftEpipole = fundMatrix.getLeftEpipole();
            final Point2D rightEpipole = fundMatrix.getRightEpipole();

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
    public void testGetAndComputeEpipolesAndEpipolarLinesAndCheckAvailability()
            throws InvalidPairOfCamerasException,
            com.irurueta.geometry.estimators.NotReadyException,
            InvalidFundamentalMatrixException, NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            // provide two pinhole cameras
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS,
                    MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS,
                    MAX_SKEWNESS);

            final double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final Point3D center1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                    gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                    gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);

            final FundamentalMatrix fundMatrix = new FundamentalMatrix(camera1,
                    camera2);

            // compute epipoles
            final Point2D epipole1a = camera1.project(center2);
            final Point2D epipole2a = camera2.project(center1);

            fundMatrix.computeEpipoles();

            final Point2D epipole1b = fundMatrix.getLeftEpipole();
            final Point2D epipole2b = fundMatrix.getRightEpipole();

            if (epipole1a.distanceTo(epipole1b) > 10.0 * ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole1a.distanceTo(epipole1b), 0.0,
                    10.0 * ABSOLUTE_ERROR);
            if (epipole2a.distanceTo(epipole2b) > 10.0 * ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole2a.distanceTo(epipole2b), 0.0, 10.0 * ABSOLUTE_ERROR);

            // generate a random 3D point
            final Point3D point3D = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            // project 3D point with both cameras
            // left view
            final Point2D point2D1 = camera1.project(point3D);
            // right view
            final Point2D point2D2 = camera2.project(point3D);

            // Obtain epipolar line on left view using 2D point on right view
            final Line2D line1 = fundMatrix.getLeftEpipolarLine(point2D2);
            // Obtain epipolar line on right view using 2D point on left view
            final Line2D line2 = fundMatrix.getRightEpipolarLine(point2D1);

            final Line2D line1b = new Line2D();
            fundMatrix.leftEpipolarLine(point2D2, line1b);
            final Line2D line2b = new Line2D();
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
            final Plane epipolarPlane1 = camera1.backProject(line1);
            final Plane epipolarPlane2 = camera2.backProject(line2);

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
    public void testNormalizeAndIsNormalized()
            throws InvalidPairOfCamerasException,
            com.irurueta.geometry.estimators.NotReadyException,
            NotAvailableException {
        // provide two pinhole cameras
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double alphaEuler1 = 0.0;
        final double betaEuler1 = 0.0;
        final double gammaEuler1 = 0.0;
        final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);

        final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final double horizontalPrincipalPoint1 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint1 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final double cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION,
                MAX_CAMERA_SEPARATION);

        final Point3D center1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D center2 = new InhomogeneousPoint3D(
                center1.getInhomX() + cameraSeparation,
                center1.getInhomY() + cameraSeparation,
                center1.getInhomZ() + cameraSeparation);

        final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                gammaEuler1);
        final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                gammaEuler2);

        final PinholeCameraIntrinsicParameters intrinsic1 =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                        verticalFocalLength1, horizontalPrincipalPoint1,
                        verticalPrincipalPoint1, skewness1);
        final PinholeCameraIntrinsicParameters intrinsic2 =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                        verticalFocalLength2, horizontalPrincipalPoint2,
                        verticalPrincipalPoint2, skewness2);

        final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1,
                center1);
        final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                center2);

        final FundamentalMatrix fundMatrix = new FundamentalMatrix(camera1, camera2);

        assertFalse(fundMatrix.isNormalized());

        // normalize
        fundMatrix.normalize();

        // check correctness
        assertTrue(fundMatrix.isNormalized());

        // check that internal matrix has Frobenius norm equal to 1
        final Matrix internalMatrix = fundMatrix.getInternalMatrix();

        assertEquals(Utils.normF(internalMatrix), 1.0, ABSOLUTE_ERROR);
    }

    @Test
    public void testIsValidInternalMatrix() throws WrongSizeException,
            NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException {

        // create a valid 3x3 rank 2 matrix
        final Matrix a = Matrix.createWithUniformRandomValues(FUND_MATRIX_ROWS,
                FUND_MATRIX_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final Matrix u = decomposer.getU();
        final Matrix w = decomposer.getW();
        final Matrix v = decomposer.getV();

        // transpose V
        final Matrix transV = v.transposeAndReturnNew();

        // Set last singular value to zero to enforce rank 2
        w.setElementAt(2, 2, 0.0);

        Matrix fundamentalInternalMatrix = u.multiplyAndReturnNew(
                w.multiplyAndReturnNew(transV));

        assertTrue(FundamentalMatrix.isValidInternalMatrix(
                fundamentalInternalMatrix));

        // try with a non 3x3 matrix
        fundamentalInternalMatrix = new Matrix(FUND_MATRIX_ROWS + 1,
                FUND_MATRIX_COLS + 1);
        assertFalse(FundamentalMatrix.isValidInternalMatrix(
                fundamentalInternalMatrix));

        // try with a non rank-2 3x3 matrix
        fundamentalInternalMatrix = Matrix.createWithUniformRandomValues(
                FUND_MATRIX_ROWS, FUND_MATRIX_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        while (Utils.rank(fundamentalInternalMatrix) == 2) {
            fundamentalInternalMatrix = Matrix.createWithUniformRandomValues(
                    FUND_MATRIX_ROWS, FUND_MATRIX_COLS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
        }

        assertFalse(FundamentalMatrix.isValidInternalMatrix(
                fundamentalInternalMatrix));
    }

    @Test
    public void testGenerateCamerasInArbitraryProjectiveSpace()
            throws InvalidPairOfCamerasException,
            InvalidFundamentalMatrixException,
            com.irurueta.geometry.estimators.NotReadyException,
            NotAvailableException, CameraException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

            final double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final double cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION,
                    MAX_CAMERA_SEPARATION);

            final Point3D center1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                    gammaEuler1);
            final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                    gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);

            final FundamentalMatrix fundMatrix = new FundamentalMatrix();

            // check default values
            assertFalse(fundMatrix.isInternalMatrixAvailable());

            fundMatrix.setFromPairOfCameras(camera1, camera2);
            fundMatrix.normalize();

            // obtain arbitrary pair of cameras
            final double referencePlaneDirectorVectorX = randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double referencePlaneDirectorVectorY = randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double referencePlaneDirectorVectorZ = randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double scaleFactor = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
            final PinholeCamera camera1b = new PinholeCamera();
            final PinholeCamera camera2b = new PinholeCamera();
            fundMatrix.generateCamerasInArbitraryProjectiveSpace(camera1b, camera2b,
                    referencePlaneDirectorVectorX, referencePlaneDirectorVectorY,
                    referencePlaneDirectorVectorZ, scaleFactor);

            // compute fundamental matrix for arbitrary cameras
            final FundamentalMatrix fundMatrix2 = new FundamentalMatrix();

            fundMatrix2.setFromPairOfCameras(camera1b, camera2b);
            fundMatrix2.normalize();

            // obtain arbitrary pair of cameras for plane at infinity as reference
            // plane and unitary scale factor
            final PinholeCamera camera1c = new PinholeCamera();
            final PinholeCamera camera2c = new PinholeCamera();
            fundMatrix.generateCamerasInArbitraryProjectiveSpace(camera1c,
                    camera2c);

            // compute fundamental matrix for arbitrary cameras
            final FundamentalMatrix fundMatrix3 = new FundamentalMatrix();

            fundMatrix3.setFromPairOfCameras(camera1c, camera2c);
            fundMatrix3.normalize();

            // check correctness by checking generated epipolar geometry

            assertTrue(fundMatrix.isInternalMatrixAvailable());
            assertTrue(fundMatrix2.isInternalMatrixAvailable());
            assertTrue(fundMatrix3.isInternalMatrixAvailable());

            // compute epipoles
            final Point2D epipole1 = camera1.project(center2);
            final Point2D epipole2 = camera2.project(center1);

            fundMatrix.computeEpipoles();
            fundMatrix2.computeEpipoles();
            fundMatrix3.computeEpipoles();

            final Point2D epipole1a = fundMatrix.getLeftEpipole();
            final Point2D epipole2a = fundMatrix.getRightEpipole();

            final Point2D epipole1b = fundMatrix2.getLeftEpipole();
            final Point2D epipole2b = fundMatrix2.getRightEpipole();

            final Point2D epipole1c = fundMatrix3.getLeftEpipole();
            final Point2D epipole2c = fundMatrix3.getRightEpipole();

            if (epipole1.distanceTo(epipole1a) > 5.0 * LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole1.distanceTo(epipole1a), 0.0, 5.0 * LARGE_ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2a) > 5.0 * LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole2.distanceTo(epipole2a), 0.0, 5.0 * LARGE_ABSOLUTE_ERROR);

            if (epipole1.distanceTo(epipole1b) > 5.0 * LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole1.distanceTo(epipole1b), 0.0, 5.0 * LARGE_ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2b) > 5.0 * LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole2.distanceTo(epipole2b), 0.0, 5.0 * LARGE_ABSOLUTE_ERROR);

            if (epipole1.distanceTo(epipole1c) > 5.0 * LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole1.distanceTo(epipole1c), 0.0, 5.0 * LARGE_ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2c) > 5.0 * LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole2.distanceTo(epipole2c), 0.0, 5.0 * LARGE_ABSOLUTE_ERROR);

            // generate a random 3D points
            final Point3D point3Da = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point3D point3Db = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point3D point3Dc = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            // project 3D points with each pair of cameras
            final Point2D point2D1a = camera1.project(point3Da);
            final Point2D point2D2a = camera2.project(point3Da);

            final Point2D point2D1b = camera1b.project(point3Db);
            final Point2D point2D2b = camera2b.project(point3Db);

            final Point2D point2D1c = camera1c.project(point3Dc);
            final Point2D point2D2c = camera2c.project(point3Dc);

            // obtain epipolar lines
            final Line2D line1a = fundMatrix.getLeftEpipolarLine(point2D2a);
            final Line2D line2a = fundMatrix.getRightEpipolarLine(point2D1a);

            final Line2D line1b = fundMatrix2.getLeftEpipolarLine(point2D2b);
            final Line2D line2b = fundMatrix2.getRightEpipolarLine(point2D1b);

            final Line2D line1c = fundMatrix3.getLeftEpipolarLine(point2D2c);
            final Line2D line2c = fundMatrix3.getRightEpipolarLine(point2D1c);

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
            final Plane epipolarPlane1a = camera1.backProject(line1a);
            final Plane epipolarPlane2a = camera2.backProject(line2a);

            final Plane epipolarPlane1b = camera1b.backProject(line1b);
            final Plane epipolarPlane2b = camera2b.backProject(line2b);

            final Plane epipolarPlane1c = camera1c.backProject(line1c);
            final Plane epipolarPlane2c = camera2c.backProject(line2c);

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
            final Point3D center1b = camera1b.getCameraCenter();
            final Point3D center2b = camera2b.getCameraCenter();

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
            final Point3D center1c = camera1c.getCameraCenter();
            final Point3D center2c = camera2c.getCameraCenter();

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
    public void testSerializeDeserialize() throws GeometryException, AlgebraException, IOException, ClassNotFoundException {
        // create fundamental matrix by providing two pinhole cameras
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double alphaEuler1 = 0.0;
        final double betaEuler1 = 0.0;
        final double gammaEuler1 = 0.0;
        final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double horizontalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength1 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);

        final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final double horizontalPrincipalPoint1 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint1 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final double cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION,
                MAX_CAMERA_SEPARATION);

        final Point3D center1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D center2 = new InhomogeneousPoint3D(
                center1.getInhomX() + cameraSeparation,
                center1.getInhomY() + cameraSeparation,
                center1.getInhomZ() + cameraSeparation);

        final Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1,
                gammaEuler1);
        final Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                gammaEuler2);

        final PinholeCameraIntrinsicParameters intrinsic1 =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                        verticalFocalLength1, horizontalPrincipalPoint1,
                        verticalPrincipalPoint1, skewness1);
        final PinholeCameraIntrinsicParameters intrinsic2 =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                        verticalFocalLength2, horizontalPrincipalPoint2,
                        verticalPrincipalPoint2, skewness2);

        final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1,
                center1);
        final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                center2);

        final FundamentalMatrix fundMatrix1 = new FundamentalMatrix(camera1, camera2);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(fundMatrix1);
        final FundamentalMatrix fundMatrix2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(fundMatrix1.getInternalMatrix(), fundMatrix2.getInternalMatrix());
    }

    private Transformation2D generateHomography(
            final PinholeCamera camera1,
            final PinholeCamera camera2,
            final FundamentalMatrix fundamentalMatrix,
            final List<Point2D> projectedPoints1,
            final List<Point2D> projectedPoints2) throws GeometryException, AlgebraException,
            RobustEstimatorException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double aspectRatio = 1.0;
        final double skewness = 0.0;
        final double principalPoint = 0.0;

        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(focalLength, focalLength,
                        principalPoint, principalPoint, skewness);
        intrinsic.setAspectRatioKeepingHorizontalFocalLength(aspectRatio);

        final double alphaEuler1 = 0.0;
        final double betaEuler1 = 0.0;
        final double gammaEuler1 = 0.0;
        final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION,
                MAX_CAMERA_SEPARATION);

        final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
        final Point3D center2 = new InhomogeneousPoint3D(
                center1.getInhomX() + cameraSeparation,
                center1.getInhomY() + cameraSeparation,
                center1.getInhomZ() + cameraSeparation);

        final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                betaEuler1, gammaEuler1);
        final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                betaEuler2, gammaEuler2);

        camera1.setIntrinsicAndExtrinsicParameters(intrinsic, rotation1,
                center1);
        camera2.setIntrinsicAndExtrinsicParameters(intrinsic, rotation2,
                center2);

        fundamentalMatrix.setFromPairOfCameras(camera1, camera2);

        // create 3D points laying in front of both cameras an in a plane
        final Plane horizontalPlane1 = camera1.getHorizontalAxisPlane();
        final Plane verticalPlane1 = camera1.getVerticalAxisPlane();
        final Plane horizontalPlane2 = camera2.getHorizontalAxisPlane();
        final Plane verticalPlane2 = camera2.getVerticalAxisPlane();
        final Matrix planesIntersectionMatrix = new Matrix(Plane.PLANE_NUMBER_PARAMS,
                Plane.PLANE_NUMBER_PARAMS);
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

        final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                planesIntersectionMatrix);
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
                ArrayUtils.sumAndReturnNew(principalAxis1, principalAxis2),
                0.5);

        final Plane plane = new Plane(centralCommonPoint, avgPrincipalAxis);
        plane.normalize();

        final double planeA = plane.getA();
        final double planeB = plane.getB();
        final double planeC = plane.getC();
        final double planeD = plane.getD();

        final int numPoints = randomizer.nextInt(MIN_NUM_POINTS, MAX_NUM_POINTS);

        HomogeneousPoint3D point3D;
        projectedPoints1.clear();
        projectedPoints2.clear();
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
                final double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                        MAX_RANDOM_VALUE);
                if (Math.abs(planeB) > ABSOLUTE_ERROR) {
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    homY = -(planeA * homX + planeC * homZ + planeD * homW) /
                            planeB;
                } else {
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                            MAX_RANDOM_VALUE);
                    homX = -(planeB * homY + planeC * homZ + planeD * homW) /
                            planeA;
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

            // check that 3D point is in front of both cameras
            //noinspection ConstantConditions
            assertTrue(front1);
            //noinspection ConstantConditions
            assertTrue(front2);

            // project 3D point into both cameras
            projectedPoints1.add(camera1.project(point3D));
            projectedPoints2.add(camera2.project(point3D));
        }

        // estimate homography
        final ProjectiveTransformation2DRobustEstimator homographyEstimator =
                ProjectiveTransformation2DRobustEstimator.createFromPoints(
                        projectedPoints1, projectedPoints2,
                        RobustEstimatorMethod.LMedS);

        return homographyEstimator.estimate();
    }
}
