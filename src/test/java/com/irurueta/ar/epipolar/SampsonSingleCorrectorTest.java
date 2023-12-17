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

import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.LockedException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.NotAvailableException;
import com.irurueta.algebra.NotReadyException;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class SampsonSingleCorrectorTest {

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double MIN_PROJECTED_ERROR = -1.0;
    private static final double MAX_PROJECTED_ERROR = 1.0;

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

    private static final int TIMES = 100;

    @Test
    public void testConstructor() throws WrongSizeException, NotAvailableException, NotReadyException,
            LockedException, DecomposerException, InvalidFundamentalMatrixException {
        // test constructor without arguments
        SampsonSingleCorrector corrector = new SampsonSingleCorrector();

        // check default values
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertNull(corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());

        // test constructor with fundamental matrix
        final FundamentalMatrix emptyFundamentalMatrix = new FundamentalMatrix();
        corrector = new SampsonSingleCorrector(emptyFundamentalMatrix);

        // check default values
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertSame(emptyFundamentalMatrix, corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());

        // test constructor with left and right points
        final Point2D leftPoint = Point2D.create();
        final Point2D rightPoint = Point2D.create();
        corrector = new SampsonSingleCorrector(leftPoint, rightPoint);

        // check default values
        assertSame(leftPoint, corrector.getLeftPoint());
        assertSame(rightPoint, corrector.getRightPoint());
        assertNull(corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());

        // test constructor with left and right points and a valid fundamental
        // matrix
        FundamentalMatrix fundamentalMatrix;
        int rank;
        do {
            Matrix internalMatrix = Matrix.createWithUniformRandomValues(
                    FundamentalMatrix.FUNDAMENTAL_MATRIX_ROWS,
                    FundamentalMatrix.FUNDAMENTAL_MATRIX_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            // ensure that internal matrix has rank 2
            final SingularValueDecomposer decomposer = new SingularValueDecomposer(internalMatrix);
            decomposer.decompose();

            // if rank is less than 2 we need to
            // pick another random matrix
            rank = decomposer.getRank();

            final Matrix u = decomposer.getU();
            final Matrix w = decomposer.getW();
            final Matrix v = decomposer.getV();
            final Matrix transV = v.transposeAndReturnNew();

            // set last element to 0 to force rank 2
            w.setElementAt(2, 2, 0.0);

            internalMatrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));

            fundamentalMatrix = new FundamentalMatrix(internalMatrix);
        } while (rank < 2);

        corrector = new SampsonSingleCorrector(leftPoint, rightPoint, fundamentalMatrix);

        // check default values
        assertSame(leftPoint, corrector.getLeftPoint());
        assertSame(rightPoint, corrector.getRightPoint());
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertTrue(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());

        // if we use a fundamental matrix without internal matrix defined, then
        // corrector is not ready
        corrector = new SampsonSingleCorrector(leftPoint, rightPoint, emptyFundamentalMatrix);

        // check default values
        assertSame(leftPoint, corrector.getLeftPoint());
        assertSame(rightPoint, corrector.getRightPoint());
        assertSame(emptyFundamentalMatrix, corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());
    }

    @Test
    public void testGetSetPointsAndFundamentalMatrix() {
        final SampsonSingleCorrector corrector = new SampsonSingleCorrector();

        // check default values
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertNull(corrector.getFundamentalMatrix());

        // set new values
        final Point2D leftPoint = Point2D.create();
        final Point2D rightPoint = Point2D.create();
        final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();

        corrector.setPointsAndFundamentalMatrix(leftPoint, rightPoint, fundamentalMatrix);

        // check correctness
        assertSame(leftPoint, corrector.getLeftPoint());
        assertSame(rightPoint, corrector.getRightPoint());
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
    }

    @Test
    public void testGetSetFundamentalMatrix() {
        final SampsonSingleCorrector corrector = new SampsonSingleCorrector();

        // check default value
        assertNull(corrector.getFundamentalMatrix());

        // set new value
        final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
        corrector.setFundamentalMatrix(fundamentalMatrix);

        // check correctness
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
    }

    @Test
    public void testGetSetPoints() {
        final SampsonSingleCorrector corrector = new SampsonSingleCorrector();

        // check default values
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());

        // set new values
        final Point2D leftPoint = Point2D.create();
        final Point2D rightPoint = Point2D.create();

        corrector.setPoints(leftPoint, rightPoint);

        // check correctness
        assertSame(leftPoint, corrector.getLeftPoint());
        assertSame(rightPoint, corrector.getRightPoint());
    }

    @Test
    public void testCorrect() throws InvalidPairOfCamerasException,
            com.irurueta.geometry.estimators.NotReadyException {
        int numImproved = 0;
        for (int t = 0; t < TIMES; t++) {
            // create intrinsic parameters
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

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            // camera centers
            final Point3D cameraCenter1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point3D cameraCenter2 = new InhomogeneousPoint3D(
                    cameraCenter1.getInhomX() + cameraSeparation,
                    cameraCenter1.getInhomY() + cameraSeparation,
                    cameraCenter1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            // create random 3D point to project
            final Point3D pointToProject = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            // create two cameras
            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, cameraCenter1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2, cameraCenter2);

            // project 3D point with both cameras
            final Point2D leftPoint = camera1.project(pointToProject);
            final Point2D rightPoint = camera2.project(pointToProject);

            // add error to projected points
            final double errorLeftX = randomizer.nextDouble(MIN_PROJECTED_ERROR, MAX_PROJECTED_ERROR);
            final double errorLeftY = randomizer.nextDouble(MIN_PROJECTED_ERROR, MAX_PROJECTED_ERROR);
            final double errorRightX = randomizer.nextDouble(MIN_PROJECTED_ERROR, MAX_PROJECTED_ERROR);
            final double errorRightY = randomizer.nextDouble(MIN_PROJECTED_ERROR, MAX_PROJECTED_ERROR);

            final Point2D wrongLeftPoint = new HomogeneousPoint2D(
                    leftPoint.getInhomX() + errorLeftX,
                    leftPoint.getInhomY() + errorLeftY, 1.0);
            final Point2D wrongRightPoint = new HomogeneousPoint2D(
                    rightPoint.getInhomX() + errorRightX,
                    rightPoint.getInhomY() + errorRightY, 1.0);

            // create fundamental matrix for the same pair of cameras used to
            // project point
            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

            // check that points without error belong to epipolar lines
            Line2D rightEpipolarLine = fundamentalMatrix.getRightEpipolarLine(leftPoint);
            Line2D leftEpipolarLine = fundamentalMatrix.getLeftEpipolarLine(rightPoint);
            assertTrue(rightEpipolarLine.isLocus(rightPoint, ABSOLUTE_ERROR));
            assertTrue(leftEpipolarLine.isLocus(leftPoint, ABSOLUTE_ERROR));

            fundamentalMatrix.normalize();

            // use corrector to fix points with error
            final SampsonSingleCorrector corrector = new SampsonSingleCorrector(
                    wrongLeftPoint, wrongRightPoint, fundamentalMatrix);

            assertTrue(corrector.isReady());

            corrector.correct();

            final Point2D correctedLeftPoint = corrector.getLeftCorrectedPoint();
            final Point2D correctedRightPoint = corrector.getRightCorrectedPoint();

            rightEpipolarLine = fundamentalMatrix.getRightEpipolarLine(correctedLeftPoint);
            leftEpipolarLine = fundamentalMatrix.getLeftEpipolarLine(correctedRightPoint);

            final double correctedDistanceLeft = leftEpipolarLine.signedDistance(correctedLeftPoint);
            final double correctedDistanceRight = rightEpipolarLine.signedDistance(correctedRightPoint);

            rightEpipolarLine = fundamentalMatrix.getRightEpipolarLine(wrongLeftPoint);
            leftEpipolarLine = fundamentalMatrix.getLeftEpipolarLine(wrongRightPoint);

            final double wrongDistanceLeft = leftEpipolarLine.signedDistance(wrongLeftPoint);
            final double wrongDistanceRight = rightEpipolarLine.signedDistance(wrongRightPoint);

            // check that corrector has indeed reduced the amount of projection
            // error
            if (Math.abs(correctedDistanceLeft) > Math.abs(wrongDistanceLeft)) {
                continue;
            }
            assertTrue(Math.abs(correctedDistanceLeft) <= Math.abs(wrongDistanceLeft));
            if (Math.abs(correctedDistanceRight) > Math.abs(wrongDistanceRight)) {
                continue;
            }
            assertTrue(Math.abs(correctedDistanceRight) <= Math.abs(wrongDistanceRight));

            numImproved++;
            break;
        }
        assertTrue(numImproved > 0);
    }
}
