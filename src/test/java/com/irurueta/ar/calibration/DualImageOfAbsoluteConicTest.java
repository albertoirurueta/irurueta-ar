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
package com.irurueta.ar.calibration;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.ar.SerializationHelper;
import com.irurueta.geometry.ConicNotAvailableException;
import com.irurueta.geometry.DualQuadric;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.InvalidPinholeCameraIntrinsicParametersException;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.NonSymmetricMatrixException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.*;

public class DualImageOfAbsoluteConicTest {

    private static final int INHOM_3D_COORDS = 3;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;

    private static final double MIN_SKEWNESS = -1.0;
    private static final double MAX_SKEWNESS = 1.0;

    private static final double MIN_PRINCIPAL_POINT = 0.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final int DIAC_ROWS = 3;
    private static final int DIAC_COLS = 3;

    @Test
    public void testConstructor() throws InvalidPinholeCameraIntrinsicParametersException,
            WrongSizeException, NonSymmetricMatrixException {
        // create intrinsic parameters
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        // rotation
        final double alphaEuler = randomizer.nextDouble(
                MIN_ANGLE_DEGREES * Math.PI / 180.0,
                MAX_ANGLE_DEGREES * Math.PI / 180.0);
        final double betaEuler = randomizer.nextDouble(
                MIN_ANGLE_DEGREES * Math.PI / 180.0,
                MAX_ANGLE_DEGREES * Math.PI / 180.0);
        final double gammaEuler = randomizer.nextDouble(
                MIN_ANGLE_DEGREES * Math.PI / 180.0,
                MAX_ANGLE_DEGREES * Math.PI / 180.0);

        final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
        // camera center
        final double[] cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        final PinholeCamera camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

        final DualQuadric daq = DualQuadric.createCanonicalDualAbsoluteQuadric();

        // test constructor from intrinsic parameters
        DualImageOfAbsoluteConic diac = new DualImageOfAbsoluteConic(intrinsic);
        final DualImageOfAbsoluteConic diac2 = new DualImageOfAbsoluteConic(camera, daq);

        diac.normalize();
        diac2.normalize();

        assertEquals(diac.getA(), diac2.getA(), 10.0 * ABSOLUTE_ERROR);
        assertEquals(diac.getB(), diac2.getB(), 10.0 * ABSOLUTE_ERROR);
        assertEquals(diac.getC(), diac2.getC(), 10.0 * ABSOLUTE_ERROR);
        assertEquals(diac.getD(), diac2.getD(), 10.0 * ABSOLUTE_ERROR);
        assertEquals(diac.getE(), diac2.getE(), 10.0 * ABSOLUTE_ERROR);
        assertEquals(diac.getF(), diac2.getF(), 10.0 * ABSOLUTE_ERROR);

        final PinholeCameraIntrinsicParameters intrinsic2 = diac.getIntrinsicParameters();

        assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), ABSOLUTE_ERROR);

        // test constructor from parameters
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        diac = new DualImageOfAbsoluteConic(a, b, c, d, e, f);

        // check correctness
        assertEquals(a, diac.getA(), 0.0);
        assertEquals(b, diac.getB(), 0.0);
        assertEquals(c, diac.getC(), 0.0);
        assertEquals(d, diac.getD(), 0.0);
        assertEquals(e, diac.getE(), 0.0);
        assertEquals(f, diac.getF(), 0.0);

        // test constructor from matrix
        Matrix m = new Matrix(DIAC_ROWS, DIAC_COLS);
        // get random values
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        m.setElementAt(0, 0, a);
        m.setElementAt(0, 1, b);
        m.setElementAt(0, 2, d);
        m.setElementAt(1, 0, b);
        m.setElementAt(1, 1, c);
        m.setElementAt(1, 2, e);
        m.setElementAt(2, 0, d);
        m.setElementAt(2, 1, e);
        m.setElementAt(2, 2, f);

        diac = new DualImageOfAbsoluteConic(m);

        assertEquals(diac.getA(), m.getElementAt(0, 0), 0.0);
        assertEquals(diac.getB(), m.getElementAt(0, 1), 0.0);
        assertEquals(diac.getC(), m.getElementAt(1, 1), 0.0);
        assertEquals(diac.getD(), m.getElementAt(0, 2), 0.0);
        assertEquals(diac.getE(), m.getElementAt(1, 2), 0.0);
        assertEquals(diac.getF(), m.getElementAt(2, 2), 0.0);

        // Constructor using matrix with wrong size exception
        m = new Matrix(DIAC_ROWS, DIAC_COLS + 1);
        diac = null;
        try {
            diac = new DualImageOfAbsoluteConic(m);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(diac);

        // Constructor using non-symmetric matrix
        m = new Matrix(DIAC_ROWS, DIAC_COLS);
        m.setElementAt(0, 0, a);
        m.setElementAt(0, 1, b);
        m.setElementAt(0, 2, d);
        m.setElementAt(1, 0, b + 1.0);
        m.setElementAt(1, 1, c);
        m.setElementAt(1, 2, e + 1.0);
        m.setElementAt(2, 0, d + 1.0);
        m.setElementAt(2, 1, e);
        m.setElementAt(2, 2, f);

        try {
            diac = new DualImageOfAbsoluteConic(m);
            fail("NonSymmetricMatrixException expected but not thrown");
        } catch (final NonSymmetricMatrixException ignore) {
        }
        assertNull(diac);
    }

    @Test
    public void testGetSetIntrinsicParameters()
            throws InvalidPinholeCameraIntrinsicParametersException {
        // create intrinsic parameters
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

        final PinholeCameraIntrinsicParameters intrinsic2 = diac.getIntrinsicParameters();

        assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetConic() throws ConicNotAvailableException,
            InvalidPinholeCameraIntrinsicParametersException {
        // create intrinsic parameters
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

        final ImageOfAbsoluteConic iac = (ImageOfAbsoluteConic) diac.getConic();

        final PinholeCameraIntrinsicParameters intrinsic2 = iac.getIntrinsicParameters();

        assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(),
                ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), ABSOLUTE_ERROR);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {

        // create intrinsic parameters
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

        final DualImageOfAbsoluteConic diac1 = new DualImageOfAbsoluteConic(intrinsic);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(diac1);
        final DualImageOfAbsoluteConic diac2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(diac1.asMatrix(), diac2.asMatrix());
    }
}
