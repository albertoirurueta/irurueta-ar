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
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class DualImageOfAbsoluteConicTest {

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
    void testConstructor() throws InvalidPinholeCameraIntrinsicParametersException, WrongSizeException,
            NonSymmetricMatrixException {
        // create intrinsic parameters
        final var randomizer = new UniformRandomizer();
        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        // rotation
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
        // camera center
        final var cameraCenterArray = new double[INHOM_3D_COORDS];
        randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var cameraCenter = new InhomogeneousPoint3D(cameraCenterArray);

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

        final var daq = DualQuadric.createCanonicalDualAbsoluteQuadric();

        // test constructor from intrinsic parameters
        var diac = new DualImageOfAbsoluteConic(intrinsic);
        final var diac2 = new DualImageOfAbsoluteConic(camera, daq);

        diac.normalize();
        diac2.normalize();

        assertEquals(diac.getA(), diac2.getA(), 10.0 * ABSOLUTE_ERROR);
        assertEquals(diac.getB(), diac2.getB(), 10.0 * ABSOLUTE_ERROR);
        assertEquals(diac.getC(), diac2.getC(), 10.0 * ABSOLUTE_ERROR);
        assertEquals(diac.getD(), diac2.getD(), 10.0 * ABSOLUTE_ERROR);
        assertEquals(diac.getE(), diac2.getE(), 10.0 * ABSOLUTE_ERROR);
        assertEquals(diac.getF(), diac2.getF(), 10.0 * ABSOLUTE_ERROR);

        final var intrinsic2 = diac.getIntrinsicParameters();

        assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), ABSOLUTE_ERROR);

        // test constructor from parameters
        var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        diac = new DualImageOfAbsoluteConic(a, b, c, d, e, f);

        // check correctness
        assertEquals(a, diac.getA(), 0.0);
        assertEquals(b, diac.getB(), 0.0);
        assertEquals(c, diac.getC(), 0.0);
        assertEquals(d, diac.getD(), 0.0);
        assertEquals(e, diac.getE(), 0.0);
        assertEquals(f, diac.getF(), 0.0);

        // test constructor from matrix
        var m = new Matrix(DIAC_ROWS, DIAC_COLS);
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
        final var wrong1 = new Matrix(DIAC_ROWS, DIAC_COLS + 1);
        assertThrows(IllegalArgumentException.class, () -> new DualImageOfAbsoluteConic(wrong1));

        // Constructor using non-symmetric matrix
        final var wrong2 = new Matrix(DIAC_ROWS, DIAC_COLS);
        wrong2.setElementAt(0, 0, a);
        wrong2.setElementAt(0, 1, b);
        wrong2.setElementAt(0, 2, d);
        wrong2.setElementAt(1, 0, b + 1.0);
        wrong2.setElementAt(1, 1, c);
        wrong2.setElementAt(1, 2, e + 1.0);
        wrong2.setElementAt(2, 0, d + 1.0);
        wrong2.setElementAt(2, 1, e);
        wrong2.setElementAt(2, 2, f);

        assertThrows(NonSymmetricMatrixException.class, () -> new DualImageOfAbsoluteConic(wrong2));
    }

    @Test
    void testGetSetIntrinsicParameters() throws InvalidPinholeCameraIntrinsicParametersException {
        // create intrinsic parameters
        final var randomizer = new UniformRandomizer();
        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        final var diac = new DualImageOfAbsoluteConic(intrinsic);

        final var intrinsic2 = diac.getIntrinsicParameters();

        assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetConic() throws ConicNotAvailableException, InvalidPinholeCameraIntrinsicParametersException {
        // create intrinsic parameters
        final var randomizer = new UniformRandomizer();
        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        final var diac = new DualImageOfAbsoluteConic(intrinsic);

        final var iac = (ImageOfAbsoluteConic) diac.getConic();

        final var intrinsic2 = iac.getIntrinsicParameters();

        assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), ABSOLUTE_ERROR);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {

        // create intrinsic parameters
        final var randomizer = new UniformRandomizer();
        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        final var diac1 = new DualImageOfAbsoluteConic(intrinsic);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(diac1);
        final var diac2 = SerializationHelper.<DualImageOfAbsoluteConic>deserialize(bytes);

        // check
        assertEquals(diac1.asMatrix(), diac2.asMatrix());
    }
}
