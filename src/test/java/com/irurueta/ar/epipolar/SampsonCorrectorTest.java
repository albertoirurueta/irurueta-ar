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
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.NotAvailableException;
import com.irurueta.algebra.NotReadyException;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class SampsonCorrectorTest implements CorrectorListener {

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

    private static final int MIN_POINTS = 100;
    private static final int MAX_POINTS = 500;

    private static final int TIMES = 100;

    private int correctStart;
    private int correctEnd;
    private int correctProgressChange;

    @Test
    void testConstructor() throws WrongSizeException, NotReadyException, DecomposerException, NotAvailableException,
            InvalidFundamentalMatrixException, com.irurueta.algebra.LockedException {
        // test constructor without arguments
        var corrector = new SampsonCorrector();

        // check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());

        // test constructor with fundamental matrix
        final var emptyFundamentalMatrix = new FundamentalMatrix();
        corrector = new SampsonCorrector(emptyFundamentalMatrix);

        // check correctness
        assertSame(emptyFundamentalMatrix, corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());

        // test constructor with left and right points
        final var leftPoints = new ArrayList<Point2D>();
        final var rightPoints = new ArrayList<Point2D>();

        corrector = new SampsonCorrector(leftPoints, rightPoints);

        // check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());

        // Force IllegalArgumentException
        final var badPoints = new ArrayList<Point2D>();
        badPoints.add(Point2D.create());

        assertThrows(IllegalArgumentException.class, () -> new SampsonCorrector(badPoints, rightPoints));
        assertThrows(IllegalArgumentException.class, () -> new SampsonCorrector(leftPoints, badPoints));

        // test constructor with left and right points and fundamental matrix
        corrector = new SampsonCorrector(leftPoints, rightPoints, emptyFundamentalMatrix);

        // check correctness
        assertSame(emptyFundamentalMatrix, corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        // fundamental matrix not defined
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());

        // test constructor with left and right points and a valid fundamental
        // matrix
        FundamentalMatrix fundamentalMatrix;
        int rank;
        do {
            var internalMatrix = Matrix.createWithUniformRandomValues(FundamentalMatrix.FUNDAMENTAL_MATRIX_ROWS,
                    FundamentalMatrix.FUNDAMENTAL_MATRIX_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            // ensure that internal matrix has rank 2
            final var decomposer = new SingularValueDecomposer(internalMatrix);
            decomposer.decompose();

            // if rank is less than 2 we need to
            // pick another random matrix
            rank = decomposer.getRank();

            final var u = decomposer.getU();
            final var w = decomposer.getW();
            final var v = decomposer.getV();
            final var transV = v.transposeAndReturnNew();

            // set last element to 0 to force rank 2
            w.setElementAt(2, 2, 0.0);

            internalMatrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));

            fundamentalMatrix = new FundamentalMatrix(internalMatrix);
        } while (rank < 2);

        corrector = new SampsonCorrector(leftPoints, rightPoints, fundamentalMatrix);

        // check correctness
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertNull(corrector.getListener());
        // fundamental matrix not defined
        assertTrue(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());

        final var finalFundamentalMatrix = fundamentalMatrix;
        assertThrows(IllegalArgumentException.class,
                () -> new SampsonCorrector(badPoints, rightPoints, finalFundamentalMatrix));
        assertThrows(IllegalArgumentException.class,
                () -> new SampsonCorrector(leftPoints, badPoints, finalFundamentalMatrix));

        // test constructor with listener
        corrector = new SampsonCorrector(this);

        // check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(this, corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());

        // test constructor with fundamental matrix and listener
        corrector = new SampsonCorrector(fundamentalMatrix, this);

        // check correctness
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(this, corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());

        // test constructor with left and right points and listener
        corrector = new SampsonCorrector(leftPoints, rightPoints, this);

        // check correctness
        assertNull(corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(this, corrector.getListener());
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());

        assertThrows(IllegalArgumentException.class, () -> new SampsonCorrector(badPoints, rightPoints, this));
        assertThrows(IllegalArgumentException.class, () -> new SampsonCorrector(leftPoints, badPoints, this));

        // test constructor with left and right points, fundamental matrix
        // and listener
        corrector = new SampsonCorrector(leftPoints, rightPoints, emptyFundamentalMatrix, this);

        // check correctness
        assertSame(emptyFundamentalMatrix, corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(this, corrector.getListener());
        // fundamental matrix not defined
        assertFalse(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());

        // test constructor with left and right points, valid fundamental matrix
        // and listener
        corrector = new SampsonCorrector(leftPoints, rightPoints, fundamentalMatrix, this);

        // check correctness
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertNull(corrector.getLeftCorrectedPoints());
        assertNull(corrector.getRightCorrectedPoints());
        assertFalse(corrector.isLocked());
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);
        assertSame(this, corrector.getListener());
        assertTrue(corrector.isReady());
        assertEquals(CorrectorType.SAMPSON_CORRECTOR, corrector.getType());

        assertThrows(IllegalArgumentException.class,
                () -> new SampsonCorrector(badPoints, rightPoints, finalFundamentalMatrix, this));
        assertThrows(IllegalArgumentException.class,
                () -> new SampsonCorrector(leftPoints, badPoints, finalFundamentalMatrix, this));
    }

    @Test
    void testGetSetFundamentalMatrix() throws LockedException {
        final var corrector = new SampsonCorrector();

        // check default value
        assertNull(corrector.getFundamentalMatrix());

        // set new value
        final var fundamentalMatrix = new FundamentalMatrix();
        corrector.setFundamentalMatrix(fundamentalMatrix);

        // check correctness
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());
    }

    @Test
    void testGetSetLeftAndRightPoints() throws LockedException {
        final var corrector = new SampsonCorrector();

        // check default values
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());

        // set new values
        final var leftPoints = new ArrayList<Point2D>();
        final var rightPoints = new ArrayList<Point2D>();

        corrector.setLeftAndRightPoints(leftPoints, rightPoints);

        // check correctness
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());

        // Force IllegalArgumentException
        final var badPoints = new ArrayList<Point2D>();
        badPoints.add(Point2D.create());

        assertThrows(IllegalArgumentException.class, () -> corrector.setLeftAndRightPoints(leftPoints, badPoints));
        assertThrows(IllegalArgumentException.class, () -> corrector.setLeftAndRightPoints(badPoints, rightPoints));
    }

    @Test
    void testGetSetPointsAndFundamentalMatrix() throws LockedException {
        final var corrector = new SampsonCorrector();

        // check default values
        assertNull(corrector.getLeftPoints());
        assertNull(corrector.getRightPoints());
        assertNull(corrector.getFundamentalMatrix());

        // set new values
        final var leftPoints = new ArrayList<Point2D>();
        final var rightPoints = new ArrayList<Point2D>();
        final var fundamentalMatrix = new FundamentalMatrix();

        corrector.setPointsAndFundamentalMatrix(leftPoints, rightPoints, fundamentalMatrix);

        // check correctness
        assertSame(leftPoints, corrector.getLeftPoints());
        assertSame(rightPoints, corrector.getRightPoints());
        assertSame(fundamentalMatrix, corrector.getFundamentalMatrix());

        // Force IllegalArgumentException
        final var badPoints = new ArrayList<Point2D>();
        badPoints.add(Point2D.create());

        assertThrows(IllegalArgumentException.class,
                () -> corrector.setPointsAndFundamentalMatrix(badPoints, rightPoints, fundamentalMatrix));
        assertThrows(IllegalArgumentException.class,
                () -> corrector.setPointsAndFundamentalMatrix(leftPoints, badPoints, fundamentalMatrix));
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var corrector = new SampsonCorrector();

        // check default values
        assertEquals(Corrector.DEFAULT_PROGRESS_DELTA, corrector.getProgressDelta(), 0.0);

        // set new value
        corrector.setProgressDelta(0.5f);

        // check correctness
        assertEquals(0.5, corrector.getProgressDelta(), 0.0);
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var corrector = new SampsonCorrector();

        // check default values
        assertNull(corrector.getListener());

        // set new value
        corrector.setListener(this);

        // check correctness
        assertSame(this, corrector.getListener());
    }

    @Test
    void testAreValidPoints() {
        final var leftPoints = new ArrayList<Point2D>();
        final var rightPoints = new ArrayList<Point2D>();

        assertFalse(SampsonCorrector.areValidPoints(null, null));
        assertFalse(SampsonCorrector.areValidPoints(leftPoints, null));
        assertFalse(SampsonCorrector.areValidPoints(null, rightPoints));
        assertTrue(SampsonCorrector.areValidPoints(leftPoints, rightPoints));
    }

    @Test
    void testCorrect() throws com.irurueta.geometry.estimators.NotReadyException, LockedException {

        var improved = 0;
        var total = 0;
        for (var t = 0; t < TIMES; t++) {
            // create intrinsic parameters
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

            final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                    horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
            final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                    horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

            // camera centers
            final var cameraCenter1 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var cameraCenter2 = new InhomogeneousPoint3D(cameraCenter1.getInhomX() + cameraSeparation,
                    cameraCenter1.getInhomY() + cameraSeparation, cameraCenter1.getInhomZ() + cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            // create random list of 3D points to project
            final var nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            final var pointsToProject = new ArrayList<Point3D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                pointsToProject.add(new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            }
            total += nPoints;

            // create two cameras
            final var camera1 = new PinholeCamera(intrinsic1, rotation1, cameraCenter1);
            final var camera2 = new PinholeCamera(intrinsic2, rotation2, cameraCenter2);

            // project 3D points with both cameras
            final var leftPoints = camera1.project(pointsToProject);
            final var rightPoints = camera2.project(pointsToProject);

            // add error to projected points
            final var wrongLeftPoints = new ArrayList<Point2D>(nPoints);
            final var wrongRightPoints = new ArrayList<Point2D>(nPoints);
            for (var i = 0; i < nPoints; i++) {
                final var errorLeftX = randomizer.nextDouble(MIN_PROJECTED_ERROR, MAX_PROJECTED_ERROR);
                final var errorLeftY = randomizer.nextDouble(MIN_PROJECTED_ERROR, MAX_PROJECTED_ERROR);
                final var errorRightX = randomizer.nextDouble(MIN_PROJECTED_ERROR, MAX_PROJECTED_ERROR);
                final var errorRightY = randomizer.nextDouble(MIN_PROJECTED_ERROR, MAX_PROJECTED_ERROR);

                final var leftPoint = leftPoints.get(i);
                final var rightPoint = rightPoints.get(i);

                final var wrongLeftPoint = new HomogeneousPoint2D(leftPoint.getInhomX() + errorLeftX,
                        leftPoint.getInhomY() + errorLeftY, 1.0);
                final var wrongRightPoint = new HomogeneousPoint2D(rightPoint.getInhomX() + errorRightX,
                        rightPoint.getInhomY() + errorRightY, 1.0);

                wrongLeftPoints.add(wrongLeftPoint);
                wrongRightPoints.add(wrongRightPoint);
            }

            // create fundamental matrix for the same pair of cameras used to
            // project points
            final FundamentalMatrix fundamentalMatrix;
            try {
                fundamentalMatrix = new FundamentalMatrix(camera1, camera2);
            } catch (final InvalidPairOfCamerasException e) {
                continue;
            }

            // check that points without error belong to epipolar lines
            var validPoints = true;
            for (var i = 0; i < nPoints; i++) {
                final var leftPoint = leftPoints.get(i);
                final var rightPoint = rightPoints.get(i);

                final var rightEpipolarLine = fundamentalMatrix.getRightEpipolarLine(leftPoint);
                final var leftEpipolarLine = fundamentalMatrix.getLeftEpipolarLine(rightPoint);

                if (!rightEpipolarLine.isLocus(rightPoint, ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(rightEpipolarLine.isLocus(rightPoint, ABSOLUTE_ERROR));
                if (!leftEpipolarLine.isLocus(leftPoint, ABSOLUTE_ERROR)) {
                    validPoints = false;
                    break;
                }
                assertTrue(leftEpipolarLine.isLocus(leftPoint, ABSOLUTE_ERROR));
            }

            if (!validPoints) {
                continue;
            }

            // use corrector to fix points with error
            final var corrector = new SampsonCorrector(wrongLeftPoints, wrongRightPoints, fundamentalMatrix,
                    this);

            assertTrue(corrector.isReady());
            assertFalse(corrector.isLocked());
            assertEquals(0, correctStart);
            assertEquals(0, correctEnd);
            assertEquals(0, correctProgressChange);

            corrector.correct();

            assertTrue(corrector.isReady());
            assertFalse(corrector.isLocked());
            assertEquals(1, correctStart);
            assertEquals(1, correctEnd);
            assertTrue(correctProgressChange > 0);
            reset();

            final var correctedLeftPoints = corrector.getLeftCorrectedPoints();
            final var correctedRightPoints = corrector.getRightCorrectedPoints();

            // check correctness
            for (var i = 0; i < nPoints; i++) {
                final var correctedLeftPoint = correctedLeftPoints.get(i);
                final var correctedRightPoint = correctedRightPoints.get(i);

                final var wrongLeftPoint = wrongLeftPoints.get(i);
                final var wrongRightPoint = wrongRightPoints.get(i);

                var rightEpipolarLine = fundamentalMatrix.getRightEpipolarLine(correctedLeftPoint);
                var leftEpipolarLine = fundamentalMatrix.getLeftEpipolarLine(correctedRightPoint);

                final var correctedDistanceLeft = leftEpipolarLine.signedDistance(correctedLeftPoint);
                final var correctedDistanceRight = rightEpipolarLine.signedDistance(correctedRightPoint);

                rightEpipolarLine = fundamentalMatrix.getRightEpipolarLine(wrongLeftPoint);
                leftEpipolarLine = fundamentalMatrix.getLeftEpipolarLine(wrongRightPoint);

                final var wrongDistanceLeft = leftEpipolarLine.signedDistance(wrongLeftPoint);
                final var wrongDistanceRight = rightEpipolarLine.signedDistance(wrongRightPoint);

                // check that corrector has indeed reduced the amount of
                // projection error
                if ((Math.abs(correctedDistanceLeft) <= Math.abs(wrongDistanceLeft))
                        && (Math.abs(correctedDistanceRight) <= Math.abs(wrongDistanceRight))) {
                    improved++;
                }
            }
        }

        assertTrue(improved > 3 * total / 4);
    }

    @Override
    public void onCorrectStart(final Corrector corrector) {
        correctStart++;
        checkLocked((SampsonCorrector) corrector);
    }

    @Override
    public void onCorrectEnd(final Corrector corrector) {
        correctEnd++;
        checkLocked((SampsonCorrector) corrector);
    }

    @Override
    public void onCorrectProgressChange(final Corrector corrector, final float progress) {
        correctProgressChange++;
        checkLocked((SampsonCorrector) corrector);
    }

    private void reset() {
        correctStart = correctEnd = correctProgressChange = 0;
    }

    private void checkLocked(final SampsonCorrector corrector) {
        assertThrows(LockedException.class, () -> corrector.setFundamentalMatrix(null));
        assertThrows(LockedException.class, () -> corrector.setLeftAndRightPoints(null, null));
        assertThrows(LockedException.class, () -> corrector.setListener(this));
        assertThrows(LockedException.class,
                () -> corrector.setPointsAndFundamentalMatrix(null, null, null));
        assertThrows(LockedException.class, () -> corrector.setProgressDelta(0.5f));
        assertThrows(LockedException.class, corrector::correct);
        assertTrue(corrector.isLocked());
    }
}
