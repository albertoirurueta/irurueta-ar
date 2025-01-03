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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.ar.SerializationHelper;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class EstimatedCameraTest {

    private static final double MIN_ANGLE_DEGREES = -30.0;
    private static final double MAX_ANGLE_DEGREES = 30.0;

    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;

    private static final double MIN_SKEWNESS = -1.0;
    private static final double MAX_SKEWNESS = 1.0;

    private static final double MIN_PRINCIPAL_POINT = 0.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    @Test
    void testConstructor() {
        final var ec = new EstimatedCamera();

        // check default values
        assertNull(ec.getId());
        assertEquals(0, ec.getViewId());
        assertNull(ec.getCamera());
        assertEquals(EstimatedCamera.DEFAULT_QUALITY_SCORE, ec.getQualityScore(), 0.0);
        assertNull(ec.getCovariance());
    }

    @Test
    void testGetSetId() {
        final var ec = new EstimatedCamera();

        // check default value
        assertNull(ec.getId());

        // set new value
        ec.setId("id");

        // check correctness
        assertEquals("id", ec.getId());
    }

    @Test
    void testGetSetViewId() {
        final var ec = new EstimatedCamera();

        // check default value
        assertEquals(0, ec.getViewId());

        // set new value
        ec.setViewId(1);

        // check
        assertEquals(1, ec.getViewId());
    }

    @Test
    void testGetSetCamera() {
        final var ec = new EstimatedCamera();

        // check default value
        assertNull(ec.getCamera());

        // set new value
        final var camera = new PinholeCamera();
        ec.setCamera(camera);

        // check correctness
        assertSame(camera, ec.getCamera());
    }

    @Test
    void testGetSetQualityScore() {
        final var ec = new EstimatedCamera();

        // check default value
        assertEquals(EstimatedCamera.DEFAULT_QUALITY_SCORE, ec.getQualityScore(), 0.0);

        // set new value
        ec.setQualityScore(5.0);

        // check correctness
        assertEquals(5.0, ec.getQualityScore(), 0.0);
    }

    @Test
    void testGetSetCovariance() throws WrongSizeException {
        final var ec = new EstimatedCamera();

        // check default value
        assertNull(ec.getCovariance());

        // set new value
        final var cov = new Matrix(12, 12);
        ec.setCovariance(cov);

        // check correctness
        assertSame(cov, ec.getCovariance());
    }

    @Test
    void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final var ec1 = new EstimatedCamera();

        // set new values
        ec1.setId("id");
        ec1.setViewId(1);

        final var randomizer = new UniformRandomizer();
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        final var camera = new PinholeCamera(intrinsic, rotation, center);

        ec1.setCamera(camera);
        ec1.setQualityScore(0.5);

        final var cov = new Matrix(12, 12);
        ec1.setCovariance(cov);

        // check
        assertEquals("id", ec1.getId());
        assertEquals(1, ec1.getViewId());
        assertSame(camera, ec1.getCamera());
        assertEquals(0.5, ec1.getQualityScore(), 0.0);
        assertSame(cov, ec1.getCovariance());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(ec1);
        final var ec2 = SerializationHelper.<EstimatedCamera>deserialize(bytes);

        // check
        assertEquals(ec1.getId(), ec2.getId());
        assertEquals(ec1.getViewId(), ec2.getViewId());
        assertEquals(ec1.getCamera().getInternalMatrix(), ec2.getCamera().getInternalMatrix());
        assertEquals(ec1.getQualityScore(), ec2.getQualityScore(), 0.0);
        assertEquals(ec1.getCovariance(), ec2.getCovariance());
    }
}
