/*
 * Copyright (C) 2016 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.ar.calibration;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.Utils;
import com.irurueta.ar.SerializationHelper;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class DualAbsoluteQuadricTest {

    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;

    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;

    private static final double MIN_SKEWNESS = -1.0;
    private static final double MAX_SKEWNESS = 1.0;

    private static final double MIN_PRINCIPAL_POINT = 0.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-6;

    private static final int IAC_ROWS = 3;
    private static final int IAC_COLS = 3;

    private static final int DAQ_ROWS = 4;
    private static final int DAQ_COLS = 4;

    private static final int TIMES = 1000;

    @Test
    public void testConstructor() throws AlgebraException,
            InvalidTransformationException {
        int succeeded = 0;

        try {
            // test empty constructor
            DualAbsoluteQuadric daq = new DualAbsoluteQuadric();

            // check correctness
            assertEquals(1.0, daq.getA(), 0.0);
            assertEquals(1.0, daq.getB(), 0.0);
            assertEquals(1.0, daq.getC(), 0.0);
            assertEquals(0.0, daq.getD(), 0.0);
            assertEquals(0.0, daq.getE(), 0.0);
            assertEquals(0.0, daq.getF(), 0.0);
            assertEquals(0.0, daq.getG(), 0.0);
            assertEquals(0.0, daq.getH(), 0.0);
            assertEquals(0.0, daq.getI(), 0.0);
            assertEquals(0.0, daq.getJ(), 0.0);

            final Matrix m = daq.asMatrix();
            final Matrix m2 = Matrix.identity(DAQ_ROWS, DAQ_COLS);
            m2.setElementAt(3, 3, 0.0);

            assertTrue(m.equals(m2, 0.0));

            // test constructor from parameters
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            daq = new DualAbsoluteQuadric(a, b, c, d, e, f, g, h, i, j);

            // check correctness
            assertEquals(a, daq.getA(), 0.0);
            assertEquals(b, daq.getB(), 0.0);
            assertEquals(c, daq.getC(), 0.0);
            assertEquals(d, daq.getD(), 0.0);

            // test constructor from dual image of absolute conic and plane
            // at infinity
            final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final DualImageOfAbsoluteConic diac = new DualImageOfAbsoluteConic(intrinsic);

            final double planeA = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double planeB = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double planeC = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double planeD = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Plane planeAtInfinity = new Plane(planeA, planeB, planeC, planeD);

            diac.normalize();
            planeAtInfinity.normalize();

            daq = new DualAbsoluteQuadric(diac, planeAtInfinity);

            // check correctness
            Matrix daqMatrix2 = daq.asMatrix();

            final Matrix diacMatrix = diac.asMatrix();
            final Matrix planeAtInfinityDirectorVector =
                    Matrix.newFromArray(planeAtInfinity.getDirectorVector());
            planeAtInfinityDirectorVector.multiplyByScalar(1.0 / planeAtInfinity.getD());

            Matrix daqMatrix = new Matrix(DAQ_ROWS, DAQ_COLS);
            daqMatrix.setSubmatrix(0, 0, 2, 2,
                    diacMatrix);
            daqMatrix.setSubmatrix(0, 3, 2, 3,
                    diacMatrix.multiplyAndReturnNew(planeAtInfinityDirectorVector)
                            .multiplyByScalarAndReturnNew(-1.0));
            daqMatrix.setSubmatrix(3, 0, 3, 2,
                    diacMatrix.multiplyAndReturnNew(planeAtInfinityDirectorVector)
                            .multiplyByScalarAndReturnNew(-1.0).transposeAndReturnNew());
            daqMatrix.setElementAt(3, 3,
                    planeAtInfinityDirectorVector.transposeAndReturnNew().multiplyAndReturnNew(
                            diacMatrix).multiplyAndReturnNew(planeAtInfinityDirectorVector)
                            .getElementAtIndex(0));

            assertTrue(daqMatrix2.equals(daqMatrix, ABSOLUTE_ERROR));

            DualImageOfAbsoluteConic diac2 = daq.getDualImageOfAbsoluteConic();
            assertTrue(diac2.asMatrix().equals(diacMatrix, ABSOLUTE_ERROR));
            Plane planeAtInfinity2 = daq.getPlaneAtInfinity();
            assertTrue(planeAtInfinity.equals(planeAtInfinity2, ABSOLUTE_ERROR));

            // test constructor from dual image of absolute conic
            daq = new DualAbsoluteQuadric(diac);

            // check correctness
            daqMatrix2 = daq.asMatrix();

            final Plane planeAtInfinity3 = new Plane(0.0, 0.0, 0.0, 1.0);

            daqMatrix = new Matrix(DAQ_ROWS, DAQ_COLS);
            daqMatrix.setSubmatrix(0, 0, 2, 2,
                    diacMatrix);

            assertTrue(daqMatrix2.equals(daqMatrix, ABSOLUTE_ERROR));

            diac2 = daq.getDualImageOfAbsoluteConic();
            assertTrue(diac2.asMatrix().equals(diacMatrix, ABSOLUTE_ERROR));
            planeAtInfinity2 = daq.getPlaneAtInfinity();
            assertTrue(planeAtInfinity3.equals(planeAtInfinity2, ABSOLUTE_ERROR));


            // test constructor from plane at infinity
            daq = new DualAbsoluteQuadric(planeAtInfinity);

            // check correctness
            daqMatrix2 = daq.asMatrix();

            daqMatrix = Matrix.identity(DAQ_ROWS, DAQ_COLS);
            daqMatrix.setSubmatrix(0, 3, 2, 3,
                    planeAtInfinityDirectorVector.multiplyByScalarAndReturnNew(-1.0));
            daqMatrix.setSubmatrix(3, 0, 3, 2,
                    planeAtInfinityDirectorVector.multiplyByScalarAndReturnNew(-1.0)
                            .transposeAndReturnNew());
            daqMatrix.setElementAt(3, 3,
                    planeAtInfinityDirectorVector.transposeAndReturnNew().multiplyAndReturnNew(
                            planeAtInfinityDirectorVector).getElementAtIndex(0));

            assertTrue(daqMatrix2.equals(daqMatrix, ABSOLUTE_ERROR));

            diac2 = daq.getDualImageOfAbsoluteConic();
            assertTrue(diac2.asMatrix().equals(Matrix.identity(IAC_ROWS, IAC_COLS), ABSOLUTE_ERROR));
            planeAtInfinity2 = daq.getPlaneAtInfinity();
            assertTrue(planeAtInfinity.equals(planeAtInfinity2, ABSOLUTE_ERROR));

            succeeded++;
        } catch (final CalibrationException ignore) {
        }

        assertTrue(succeeded > 0);

        // initially transformation is the identity
        ProjectiveTransformation3D transformation = new ProjectiveTransformation3D();

        DualAbsoluteQuadric daq = new DualAbsoluteQuadric(transformation);
        daq.normalize();

        // check correctness
        Matrix daqMatrix = Matrix.identity(4, 4);
        daqMatrix.setElementAt(3, 3, 0.0);
        daqMatrix.multiplyByScalar(1.0 / Utils.normF(daqMatrix));

        Matrix daqMatrix2 = daq.asMatrix();
        assertTrue(daqMatrix2.equals(daqMatrix, ABSOLUTE_ERROR));

        // test with random transformation
        Matrix t = Matrix.createWithUniformRandomValues(4, 4,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        transformation = new ProjectiveTransformation3D(t);

        daq = new DualAbsoluteQuadric(transformation);
        daq.normalize();

        // check correctness
        Matrix i = Matrix.identity(4, 4);
        i.setElementAt(3, 3, 0.0);
        daqMatrix = t.multiplyAndReturnNew(i).multiplyAndReturnNew(t.transposeAndReturnNew());
        daqMatrix.multiplyByScalar(1.0 / Utils.normF(daqMatrix));

        daqMatrix2 = daq.asMatrix();
        assertTrue(daqMatrix2.equals(daqMatrix, ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetMetricToProjectiveTransformation()
            throws InvalidTransformationException, AlgebraException {
        final DualAbsoluteQuadric daq = new DualAbsoluteQuadric();

        // check initial value
        ProjectiveTransformation3D transformation = daq.getMetricToProjectiveTransformation();
        transformation.normalize();

        daq.normalize();
        Matrix daqMatrix = daq.asMatrix();

        Matrix t = transformation.asMatrix();
        final Matrix ident = Matrix.identity(4, 4);
        ident.setElementAt(3, 3, 0.0);

        Matrix daqMatrix2 = t.multiplyAndReturnNew(ident).multiplyAndReturnNew(t.transposeAndReturnNew());
        daqMatrix2.multiplyByScalar(1.0 / Utils.normF(daqMatrix2));

        assertTrue(daqMatrix.equals(daqMatrix2, ABSOLUTE_ERROR));

        // set random transformation (from an orthonormal matrix)

        final Matrix m = Matrix.createWithUniformRandomValues(4, 4,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        m.setElementAt(3, 3, 1.0);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();
        t = decomposer.getU();

        final double[] w = decomposer.getSingularValues();
        for (int i = 0; i < 3; i++) {
            final double scalar = Math.sqrt(w[i]);
            for (int j = 0; j < 4; j++) {
                t.setElementAt(j, i, scalar * t.getElementAt(j, i));
            }
        }

        transformation = new ProjectiveTransformation3D(t);

        daq.setMetricToProjectiveTransformation(transformation);
        daq.normalize();

        daqMatrix = daq.asMatrix();

        // check correctness
        final ProjectiveTransformation3D transformation2 = daq.getMetricToProjectiveTransformation();
        final ProjectiveTransformation3D transformation3 = new ProjectiveTransformation3D();
        daq.getMetricToProjectiveTransformation(transformation3);

        transformation2.normalize();
        transformation3.normalize();

        final Matrix t2 = transformation2.asMatrix();
        final Matrix t3 = transformation3.asMatrix();

        daqMatrix2 = t2.multiplyAndReturnNew(ident).multiplyAndReturnNew(t2.transposeAndReturnNew());
        daqMatrix2.multiplyByScalar(1.0 / Utils.normF(daqMatrix2));

        final Matrix daqMatrix3 = t3.multiplyAndReturnNew(ident).multiplyAndReturnNew(
                t3.transposeAndReturnNew());
        daqMatrix3.multiplyByScalar(1.0 / Utils.normF(daqMatrix3));

        assertTrue(daqMatrix.equals(daqMatrix2, ABSOLUTE_ERROR));
        assertTrue(daqMatrix.equals(daqMatrix3, ABSOLUTE_ERROR));
    }

    @Test
    public void testSetDualImageOfAbsoluteConicAndPlaneAtInfinity()
            throws AlgebraException {
        int succeeded = 0;

        try {
            final DualAbsoluteQuadric daq = new DualAbsoluteQuadric();

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final DualImageOfAbsoluteConic diac = new DualImageOfAbsoluteConic(intrinsic);

            final double planeA = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double planeB = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double planeC = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double planeD = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Plane planeAtInfinity = new Plane(planeA, planeB, planeC, planeD);

            diac.normalize();
            planeAtInfinity.normalize();

            daq.setDualImageOfAbsoluteConicAndPlaneAtInfinity(diac, planeAtInfinity);

            // check correctness
            final Matrix daqMatrix2 = daq.asMatrix();

            final Matrix diacMatrix = diac.asMatrix();
            final Matrix planeAtInfinityDirectorVector = Matrix.newFromArray(
                    planeAtInfinity.getDirectorVector());
            planeAtInfinityDirectorVector.multiplyByScalar(1.0 / planeAtInfinity.getD());

            final Matrix daqMatrix = new Matrix(DAQ_ROWS, DAQ_COLS);
            daqMatrix.setSubmatrix(0, 0, 2, 2,
                    diacMatrix);
            daqMatrix.setSubmatrix(0, 3, 2, 3,
                    diacMatrix.multiplyAndReturnNew(planeAtInfinityDirectorVector)
                            .multiplyByScalarAndReturnNew(-1.0));
            daqMatrix.setSubmatrix(3, 0, 3, 2,
                    diacMatrix.multiplyAndReturnNew(planeAtInfinityDirectorVector).
                            multiplyByScalarAndReturnNew(-1.0).transposeAndReturnNew());
            daqMatrix.setElementAt(3, 3, planeAtInfinityDirectorVector.transposeAndReturnNew().
                    multiplyAndReturnNew(diacMatrix).multiplyAndReturnNew(planeAtInfinityDirectorVector)
                    .getElementAtIndex(0));

            assertTrue(daqMatrix2.equals(daqMatrix, ABSOLUTE_ERROR));

            final DualImageOfAbsoluteConic diac2 = daq.getDualImageOfAbsoluteConic();
            assertTrue(diac2.asMatrix().equals(diacMatrix, ABSOLUTE_ERROR));
            final Plane planeAtInfinity2 = daq.getPlaneAtInfinity();
            assertTrue(planeAtInfinity.equals(planeAtInfinity2, ABSOLUTE_ERROR));
            succeeded++;
        } catch (final CalibrationException ignore) {
        }

        assertTrue(succeeded > 0);
    }

    @Test
    public void testGetSetDualImageOfAbsoluteConic() throws AlgebraException {
        int succeeded = 0;

        try {
            final DualAbsoluteQuadric daq = new DualAbsoluteQuadric();

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final DualImageOfAbsoluteConic diac = new DualImageOfAbsoluteConic(intrinsic);
            final Matrix diacMatrix = diac.asMatrix();

            daq.setDualImageOfAbsoluteConic(diac);

            // check correctness
            final Matrix daqMatrix2 = daq.asMatrix();

            final Plane planeAtInfinity3 = new Plane(0.0, 0.0, 0.0, 1.0);

            final Matrix daqMatrix = new Matrix(DAQ_ROWS, DAQ_COLS);
            daqMatrix.setSubmatrix(0, 0, 2, 2,
                    diacMatrix);

            assertTrue(daqMatrix2.equals(daqMatrix, ABSOLUTE_ERROR));

            final DualImageOfAbsoluteConic diac2 = daq.getDualImageOfAbsoluteConic();
            assertTrue(diac2.asMatrix().equals(diacMatrix, ABSOLUTE_ERROR));
            final Plane planeAtInfinity2 = daq.getPlaneAtInfinity();
            assertTrue(planeAtInfinity3.equals(planeAtInfinity2, ABSOLUTE_ERROR));

            succeeded++;
        } catch (final CalibrationException ignore) {
        }

        assertTrue(succeeded > 0);
    }

    @Test
    public void testGetSetPlaneAtInfinity() throws AlgebraException {
        int succeeded = 0;

        try {
            for (int t = 0; t < TIMES; t++) {
                final DualAbsoluteQuadric daq = new DualAbsoluteQuadric();

                final UniformRandomizer randomizer = new UniformRandomizer(new Random());

                final double planeA = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final double planeB = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final double planeC = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                final double planeD = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

                final Plane planeAtInfinity = new Plane(planeA, planeB, planeC, planeD);

                planeAtInfinity.normalize();

                daq.setPlaneAtInfinity(planeAtInfinity);

                // check correctness
                final Matrix daqMatrix2 = daq.asMatrix();

                final Matrix planeAtInfinityDirectorVector = Matrix.newFromArray(
                        planeAtInfinity.getDirectorVector());
                planeAtInfinityDirectorVector.multiplyByScalar(1.0 / planeAtInfinity.getD());

                final Matrix daqMatrix = Matrix.identity(DAQ_ROWS, DAQ_COLS);
                daqMatrix.setSubmatrix(0, 3, 2, 3,
                        planeAtInfinityDirectorVector.multiplyByScalarAndReturnNew(-1.0));
                daqMatrix.setSubmatrix(3, 0, 3, 2,
                        planeAtInfinityDirectorVector.multiplyByScalarAndReturnNew(-1.0)
                                .transposeAndReturnNew());
                daqMatrix.setElementAt(3, 3,
                        planeAtInfinityDirectorVector.transposeAndReturnNew().multiplyAndReturnNew(
                                planeAtInfinityDirectorVector).getElementAtIndex(0));

                if (!daqMatrix2.equals(daqMatrix, LARGE_ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(daqMatrix2.equals(daqMatrix, LARGE_ABSOLUTE_ERROR));

                final DualImageOfAbsoluteConic diac2 = daq.getDualImageOfAbsoluteConic();
                assertTrue(diac2.asMatrix().equals(Matrix.identity(IAC_ROWS, IAC_COLS), ABSOLUTE_ERROR));
                final Plane planeAtInfinity2 = daq.getPlaneAtInfinity();
                assertTrue(planeAtInfinity.equals(planeAtInfinity2, ABSOLUTE_ERROR));

                succeeded++;
                break;
            }
        } catch (final CalibrationException ignore) {
        }

        assertTrue(succeeded > 0);
    }

    @Test
    public void testDeterminant() throws AlgebraException {
        // check that the Dual Absolute Quadric always is singular (determinant
        // is zero)
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        final DualImageOfAbsoluteConic diac = new DualImageOfAbsoluteConic(intrinsic);

        final double planeA = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double planeB = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double planeC = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double planeD = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Plane planeAtInfinity = new Plane(planeA, planeB, planeC, planeD);

        diac.normalize();
        planeAtInfinity.normalize();

        final DualAbsoluteQuadric daq = new DualAbsoluteQuadric(diac, planeAtInfinity);

        final Matrix daqMatrix = daq.asMatrix();

        final double det = Utils.det(daqMatrix);
        assertEquals(0.0, det, ABSOLUTE_ERROR);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        // create from DIAC and plane at infinity
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        final DualImageOfAbsoluteConic diac = new DualImageOfAbsoluteConic(intrinsic);

        final double planeA = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double planeB = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double planeC = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double planeD = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Plane planeAtInfinity = new Plane(planeA, planeB, planeC, planeD);

        diac.normalize();
        planeAtInfinity.normalize();

        final DualAbsoluteQuadric daq1 = new DualAbsoluteQuadric(diac, planeAtInfinity);

        final byte[] bytes = SerializationHelper.serialize(daq1);
        final DualAbsoluteQuadric daq2 = SerializationHelper.deserialize(bytes);

        assertEquals(daq1.asMatrix(), daq2.asMatrix());
    }
}
