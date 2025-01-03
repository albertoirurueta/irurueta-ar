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
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class AlgebraicFundamentalMatrixComparatorTest implements FundamentalMatrixComparatorListener {

    private static final double MIN_RANDOM_VALUE = 100.0;
    private static final double MAX_RANDOM_VALUE = 500.0;

    private static final double MIN_FOCAL_LENGTH = 110.0;
    private static final double MAX_FOCAL_LENGTH = 130.0;

    private static final double MIN_SKEWNESS = -0.001;
    private static final double MAX_SKEWNESS = 0.001;

    private static final double MIN_PRINCIPAL_POINT = 90.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;

    private static final double MIN_ANGLE_DEGREES = 10.0;
    private static final double MAX_ANGLE_DEGREES = 15.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private int compareStart;
    private int compareEnd;

    @Test
    void testConstructor() throws AlgebraException, InvalidFundamentalMatrixException {
        // test constructor without arguments
        var comparator = new AlgebraicFundamentalMatrixComparator();

        // check default values
        assertNull(comparator.getGroundTruthFundamentalMatrix());
        assertNull(comparator.getOtherFundamentalMatrix());
        assertNull(comparator.getListener());
        assertFalse(comparator.isLocked());
        assertFalse(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.ALGEBRAIC_COMPARATOR, comparator.getType());

        // test constructor with fundamental matrices
        final var emptyFundamentalMatrix1 = new FundamentalMatrix();
        final var emptyFundamentalMatrix2 = new FundamentalMatrix();

        final var fundamentalMatrix1 = createRandomFundamentalMatrix();
        final var fundamentalMatrix2 = createRandomFundamentalMatrix();

        comparator = new AlgebraicFundamentalMatrixComparator(emptyFundamentalMatrix1, emptyFundamentalMatrix2);

        // check default values
        assertSame(emptyFundamentalMatrix1, comparator.getGroundTruthFundamentalMatrix());
        assertSame(emptyFundamentalMatrix2, comparator.getOtherFundamentalMatrix());
        assertNull(comparator.getListener());
        assertFalse(comparator.isLocked());
        // fundamental matrices are not defined
        assertFalse(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.ALGEBRAIC_COMPARATOR, comparator.getType());

        comparator = new AlgebraicFundamentalMatrixComparator(fundamentalMatrix1, emptyFundamentalMatrix2);

        // check default values
        assertSame(fundamentalMatrix1, comparator.getGroundTruthFundamentalMatrix());
        assertSame(emptyFundamentalMatrix2, comparator.getOtherFundamentalMatrix());
        assertNull(comparator.getListener());
        assertFalse(comparator.isLocked());
        // fundamental matrices are not defined
        assertFalse(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.ALGEBRAIC_COMPARATOR, comparator.getType());

        comparator = new AlgebraicFundamentalMatrixComparator(emptyFundamentalMatrix1, fundamentalMatrix2);

        // check default values
        assertSame(emptyFundamentalMatrix1, comparator.getGroundTruthFundamentalMatrix());
        assertSame(fundamentalMatrix2, comparator.getOtherFundamentalMatrix());
        assertNull(comparator.getListener());
        assertFalse(comparator.isLocked());
        // fundamental matrices are not defined
        assertFalse(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.ALGEBRAIC_COMPARATOR, comparator.getType());

        comparator = new AlgebraicFundamentalMatrixComparator(fundamentalMatrix1, fundamentalMatrix2);

        // check default values
        assertSame(fundamentalMatrix1, comparator.getGroundTruthFundamentalMatrix());
        assertSame(fundamentalMatrix2, comparator.getOtherFundamentalMatrix());
        assertNull(comparator.getListener());
        assertFalse(comparator.isLocked());
        assertTrue(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.ALGEBRAIC_COMPARATOR, comparator.getType());

        // test constructor with listener
        comparator = new AlgebraicFundamentalMatrixComparator(this);

        // check default values
        assertNull(comparator.getGroundTruthFundamentalMatrix());
        assertNull(comparator.getOtherFundamentalMatrix());
        assertSame(this, comparator.getListener());
        assertFalse(comparator.isLocked());
        assertFalse(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.ALGEBRAIC_COMPARATOR, comparator.getType());

        // test constructor with fundamental matrices and listener
        comparator = new AlgebraicFundamentalMatrixComparator(emptyFundamentalMatrix1, emptyFundamentalMatrix2,
                this);

        // check default values
        assertSame(emptyFundamentalMatrix1, comparator.getGroundTruthFundamentalMatrix());
        assertSame(emptyFundamentalMatrix2, comparator.getOtherFundamentalMatrix());
        assertSame(this, comparator.getListener());
        assertFalse(comparator.isLocked());
        // fundamental matrices are not defined
        assertFalse(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.ALGEBRAIC_COMPARATOR, comparator.getType());

        comparator = new AlgebraicFundamentalMatrixComparator(fundamentalMatrix1, emptyFundamentalMatrix2,
                this);

        // check default values
        assertSame(fundamentalMatrix1, comparator.getGroundTruthFundamentalMatrix());
        assertSame(emptyFundamentalMatrix2, comparator.getOtherFundamentalMatrix());
        assertSame(this, comparator.getListener());
        assertFalse(comparator.isLocked());
        // fundamental matrices are not defined
        assertFalse(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.ALGEBRAIC_COMPARATOR, comparator.getType());

        comparator = new AlgebraicFundamentalMatrixComparator(emptyFundamentalMatrix1, fundamentalMatrix2,
                this);

        // check default values
        assertSame(emptyFundamentalMatrix1, comparator.getGroundTruthFundamentalMatrix());
        assertSame(fundamentalMatrix2, comparator.getOtherFundamentalMatrix());
        assertSame(this, comparator.getListener());
        assertFalse(comparator.isLocked());
        // fundamental matrices are not defined
        assertFalse(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.ALGEBRAIC_COMPARATOR, comparator.getType());

        comparator = new AlgebraicFundamentalMatrixComparator(fundamentalMatrix1, fundamentalMatrix2, this);

        // check default values
        assertSame(fundamentalMatrix1, comparator.getGroundTruthFundamentalMatrix());
        assertSame(fundamentalMatrix2, comparator.getOtherFundamentalMatrix());
        assertSame(this, comparator.getListener());
        assertFalse(comparator.isLocked());
        assertTrue(comparator.isReady());
        assertEquals(FundamentalMatrixComparatorType.ALGEBRAIC_COMPARATOR, comparator.getType());
    }

    @Test
    void testGetSetGroundTruthFundamentalMatrix() throws LockedException {
        final var comparator = new AlgebraicFundamentalMatrixComparator();

        // check default value
        assertNull(comparator.getGroundTruthFundamentalMatrix());

        // set new value
        final var fundamentalMatrix = new FundamentalMatrix();
        comparator.setGroundTruthFundamentalMatrix(fundamentalMatrix);

        // check correctness
        assertSame(fundamentalMatrix, comparator.getGroundTruthFundamentalMatrix());
    }

    @Test
    void testGetSetOtherFundamentalMatrix() throws LockedException {
        final var comparator = new AlgebraicFundamentalMatrixComparator();

        // check default value
        assertNull(comparator.getOtherFundamentalMatrix());

        // set new value
        final var fundamentalMatrix = new FundamentalMatrix();
        comparator.setOtherFundamentalMatrix(fundamentalMatrix);

        // check correctness
        assertSame(fundamentalMatrix, comparator.getOtherFundamentalMatrix());
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var comparator = new AlgebraicFundamentalMatrixComparator();

        // check default value
        assertNull(comparator.getListener());

        // set new value
        comparator.setListener(this);

        // check correctness
        assertSame(this, comparator.getListener());
    }

    @Test
    void testCompare() throws InvalidPairOfCamerasException, NotReadyException, LockedException,
            FundamentalMatrixComparatorException {
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

        final var cameraCenter1 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var cameraCenter2 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
        final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

        final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
        final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

        final var camera1 = new PinholeCamera(intrinsic1, rotation1, cameraCenter1);
        final var camera2 = new PinholeCamera(intrinsic2, rotation2, cameraCenter2);

        final var fundamentalMatrix1 = new FundamentalMatrix(camera1, camera2);
        final var fundamentalMatrix2 = new FundamentalMatrix(camera1, camera2);

        final var comparator = new AlgebraicFundamentalMatrixComparator(fundamentalMatrix1, fundamentalMatrix2,
                this);

        // check status
        assertFalse(comparator.isLocked());
        assertTrue(comparator.isReady());
        assertEquals(0, compareStart);
        assertEquals(0, compareEnd);

        // compare
        assertEquals(0.0, comparator.compare(), ABSOLUTE_ERROR);

        // check correctness
        assertTrue(comparator.isReady());
        assertFalse(comparator.isLocked());
        assertEquals(1, compareStart);
        assertEquals(1, compareEnd);
        reset();
    }

    @Override
    public void onCompareStart(final FundamentalMatrixComparator comparator) {
        compareStart++;
        checkLocked((AlgebraicFundamentalMatrixComparator) comparator);
    }

    @Override
    public void onCompareEnd(final FundamentalMatrixComparator comparator) {
        compareEnd++;
        checkLocked((AlgebraicFundamentalMatrixComparator) comparator);
    }

    @Override
    public void onCompareProgressChange(final FundamentalMatrixComparator comparator, final float progress) {
        checkLocked((AlgebraicFundamentalMatrixComparator) comparator);
    }

    private void reset() {
        compareStart = compareEnd = 0;
    }

    private void checkLocked(final AlgebraicFundamentalMatrixComparator comparator) {
        assertTrue(comparator.isLocked());
        assertThrows(LockedException.class, () -> comparator.setGroundTruthFundamentalMatrix(null));
        assertThrows(LockedException.class, () -> comparator.setOtherFundamentalMatrix(null));
        assertThrows(LockedException.class, () -> comparator.setListener(null));
        assertThrows(LockedException.class, comparator::compare);
    }

    private FundamentalMatrix createRandomFundamentalMatrix() throws AlgebraException,
            InvalidFundamentalMatrixException {
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

        return fundamentalMatrix;
    }
}
