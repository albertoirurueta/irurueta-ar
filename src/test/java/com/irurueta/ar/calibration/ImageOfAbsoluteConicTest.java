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
import com.irurueta.geometry.DualConicNotAvailableException;
import com.irurueta.geometry.InvalidPinholeCameraIntrinsicParametersException;
import com.irurueta.geometry.NonSymmetricMatrixException;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class ImageOfAbsoluteConicTest {

    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;

    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;

    private static final double MIN_SKEWNESS = -1.0;
    private static final double MAX_SKEWNESS = 1.0;

    private static final double MIN_PRINCIPAL_POINT = 0.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final int IAC_ROWS = 3;
    private static final int IAC_COLS = 3;

    @Test
    void testConstructorAndGetIntrinsicParameters() throws InvalidPinholeCameraIntrinsicParametersException,
            NonSymmetricMatrixException, WrongSizeException {
        // create intrinsic parameters
        final var randomizer = new UniformRandomizer();
        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        // test constructor from intrinsic parameters
        var iac = new ImageOfAbsoluteConic(intrinsic);

        final var intrinsic2 = iac.getIntrinsicParameters();

        assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), ABSOLUTE_ERROR);

        final var intrinsic3 = iac.getIntrinsicParametersCholesky();

        assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic3.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(), intrinsic3.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic3.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic3.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), ABSOLUTE_ERROR);

        // test constructor from parameters
        var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        iac = new ImageOfAbsoluteConic(a, b, c, d, e, f);

        // check correctness
        assertEquals(a, iac.getA(), 0.0);
        assertEquals(b, iac.getB(), 0.0);
        assertEquals(c, iac.getC(), 0.0);
        assertEquals(d, iac.getD(), 0.0);
        assertEquals(e, iac.getE(), 0.0);
        assertEquals(f, iac.getF(), 0.0);

        // test constructor from matrix
        var m = new Matrix(IAC_ROWS, IAC_COLS);
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

        iac = new ImageOfAbsoluteConic(m);

        assertEquals(m.getElementAt(0, 0), iac.getA(),0.0);
        assertEquals(m.getElementAt(0, 1), iac.getB(), 0.0);
        assertEquals(m.getElementAt(1, 1), iac.getC(), 0.0);
        assertEquals(m.getElementAt(0, 2), iac.getD(), 0.0);
        assertEquals(m.getElementAt(1, 2), iac.getE(), 0.0);
        assertEquals(m.getElementAt(2, 2), iac.getF(), 0.0);

        // Constructor using matrix with wrong size exception
        final var wrong1 = new Matrix(IAC_ROWS, IAC_COLS + 1);
        assertThrows(IllegalArgumentException.class, () -> new ImageOfAbsoluteConic(wrong1));

        // Constructor using non-symmetric matrix
        final var wrong2 = new Matrix(IAC_ROWS, IAC_COLS);
        wrong2.setElementAt(0, 0, a);
        wrong2.setElementAt(0, 1, b);
        wrong2.setElementAt(0, 2, d);
        wrong2.setElementAt(1, 0, b + 1.0);
        wrong2.setElementAt(1, 1, c);
        wrong2.setElementAt(1, 2, e + 1.0);
        wrong2.setElementAt(2, 0, d + 1.0);
        wrong2.setElementAt(2, 1, e);
        wrong2.setElementAt(2, 2, f);

        assertThrows(NonSymmetricMatrixException.class, () -> new ImageOfAbsoluteConic(wrong2));
    }

    @Test
    void testGetDualConic() throws InvalidPinholeCameraIntrinsicParametersException, DualConicNotAvailableException {
        // create intrinsic parameters
        final var randomizer = new UniformRandomizer();
        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        final var iac = new ImageOfAbsoluteConic(intrinsic);

        final var diac = (DualImageOfAbsoluteConic) iac.getDualConic();

        final var intrinsic2 = diac.getIntrinsicParameters();

        assertEquals(intrinsic.getHorizontalFocalLength(), intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(), intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(), intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(), intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(), ABSOLUTE_ERROR);
    }

    @Test
    void testSerializeDeserialize() throws InvalidPinholeCameraIntrinsicParametersException, IOException,
            ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        // test constructor from intrinsic parameters
        final var iac1 = new ImageOfAbsoluteConic(intrinsic);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(iac1);
        final var iac2 = SerializationHelper.<ImageOfAbsoluteConic>deserialize(bytes);

        // check
        assertEquals(iac1.asMatrix(), iac2.asMatrix());
    }
}
