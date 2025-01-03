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
package com.irurueta.ar.sfm;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.ar.SerializationHelper;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidPairOfCamerasException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.BitSet;

import static org.junit.jupiter.api.Assertions.*;

class EstimatedFundamentalMatrixTest {

    private static final double MIN_ANGLE_DEGREES = -30.0;
    private static final double MAX_ANGLE_DEGREES = 30.0;

    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;

    private static final double MIN_SKEWNESS = -1.0;
    private static final double MAX_SKEWNESS = 1.0;

    private static final double MIN_PRINCIPAL_POINT = 0.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;

    private static final double MIN_CAMERA_SEPARATION = 5.0;
    private static final double MAX_CAMERA_SEPARATION = 10.0;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    @Test
    void testConstructor() {
        final var efm = new EstimatedFundamentalMatrix();

        // check default values
        assertNull(efm.getId());
        assertNull(efm.getFundamentalMatrix());
        assertEquals(EstimatedFundamentalMatrix.DEFAULT_QUALITY_SCORE, efm.getQualityScore(), 0.0);
        assertNull(efm.getCovariance());
        assertEquals(0, efm.getViewId1());
        assertEquals(0, efm.getViewId2());
        assertNull(efm.getInliers());
        assertNull(efm.getLeftSamples());
        assertNull(efm.getRightSamples());
    }

    @Test
    void testGetSetId() {
        final var efm = new EstimatedFundamentalMatrix();

        // check default value
        assertNull(efm.getId());

        // set new value
        efm.setId("id");

        // check correctness
        assertEquals("id", efm.getId());
    }

    @Test
    void testGetSetFundamentalMatrix() {
        final var efm = new EstimatedFundamentalMatrix();

        // check default value
        assertNull(efm.getFundamentalMatrix());

        // set new value
        final var f = new FundamentalMatrix();
        efm.setFundamentalMatrix(f);

        // check correctness
        assertSame(f, efm.getFundamentalMatrix());
    }

    @Test
    void testGetSetQualityScore() {
        final var efm = new EstimatedFundamentalMatrix();

        // check default value
        assertEquals(EstimatedFundamentalMatrix.DEFAULT_QUALITY_SCORE, efm.getQualityScore(), 0.0);

        // set new value
        efm.setQualityScore(5.0);

        // check correctness
        assertEquals(5.0, efm.getQualityScore(), 0.0);
    }

    @Test
    void testGetSetCovariance() throws WrongSizeException {
        final var efm = new EstimatedFundamentalMatrix();

        // check default value
        assertNull(efm.getCovariance());

        // set new value
        final var cov = new Matrix(9, 9);
        efm.setCovariance(cov);

        // check correctness
        assertSame(cov, efm.getCovariance());
    }

    @Test
    void testGetSetViewId1() {
        final var efm = new EstimatedFundamentalMatrix();

        // check default value
        assertEquals(0, efm.getViewId1());

        // set new value
        efm.setViewId1(5);

        // check correctness
        assertEquals(5, efm.getViewId1());
    }

    @Test
    void testGetSetViewId2() {
        final var efm = new EstimatedFundamentalMatrix();

        // check default value
        assertEquals(0, efm.getViewId2());

        // set new value
        efm.setViewId2(10);

        // check correctness
        assertEquals(10, efm.getViewId2());
    }

    @Test
    void testGetSetInliers() {
        final var efm = new EstimatedFundamentalMatrix();

        // check default value
        assertNull(efm.getInliers());

        // set new value
        final var inliers = new BitSet();
        efm.setInliers(inliers);

        // check correctness
        assertSame(inliers, efm.getInliers());
    }

    @Test
    void testGetSetLeftSamples() {
        final var efm = new EstimatedFundamentalMatrix();

        // check default value
        assertNull(efm.getLeftSamples());

        // set new value
        final var leftSamples = new ArrayList<Sample2D>();
        efm.setLeftSamples(leftSamples);

        // check correctness
        assertSame(leftSamples, efm.getLeftSamples());
    }

    @Test
    void testGetSetRightSamples() {
        final var efm = new EstimatedFundamentalMatrix();

        // check default value
        assertNull(efm.getRightSamples());

        // set new value
        final var rightSamples = new ArrayList<Sample2D>();
        efm.setRightSamples(rightSamples);

        // check correctness
        assertSame(rightSamples, efm.getRightSamples());
    }

    @Test
    void testSerializeDeserialize() throws InvalidPairOfCamerasException, WrongSizeException, IOException,
            ClassNotFoundException, NotAvailableException {
        final var efm1 = new EstimatedFundamentalMatrix();

        // set new values
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
        final var center2 = new InhomogeneousPoint3D(
                center1.getInhomX() + cameraSeparation,
                center1.getInhomY() + cameraSeparation,
                center1.getInhomZ() + cameraSeparation);

        final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
        final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

        final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
        final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

        final var camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
        final var camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

        final var fundMatrix = new FundamentalMatrix(camera1, camera2);

        efm1.setId("id");
        efm1.setFundamentalMatrix(fundMatrix);
        efm1.setQualityScore(0.2);
        final var cov = new Matrix(9, 9);
        efm1.setCovariance(cov);
        efm1.setViewId1(1);
        efm1.setViewId2(2);
        final var inliers = new BitSet();
        efm1.setInliers(inliers);
        final var leftSamples = new ArrayList<Sample2D>();
        efm1.setLeftSamples(leftSamples);
        final var rightSamples = new ArrayList<Sample2D>();
        efm1.setRightSamples(rightSamples);

        // check
        assertEquals("id", efm1.getId());
        assertSame(fundMatrix, efm1.getFundamentalMatrix());
        assertEquals(0.2, efm1.getQualityScore(), 0.0);
        assertSame(cov, efm1.getCovariance());
        assertEquals(1, efm1.getViewId1());
        assertEquals(2, efm1.getViewId2());
        assertSame(inliers, efm1.getInliers());
        assertSame(leftSamples, efm1.getLeftSamples());
        assertSame(rightSamples, efm1.getRightSamples());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(efm1);
        final var efm2 = SerializationHelper.<EstimatedFundamentalMatrix>deserialize(bytes);

        // check
        assertEquals(efm1.getId(), efm2.getId());
        assertEquals(efm1.getFundamentalMatrix().getInternalMatrix(), efm2.getFundamentalMatrix().getInternalMatrix());
        assertEquals(efm1.getQualityScore(), efm2.getQualityScore(), 0.0);
        assertEquals(efm1.getCovariance(), efm2.getCovariance());
        assertEquals(efm1.getViewId1(), efm2.getViewId1());
        assertEquals(efm1.getViewId2(), efm2.getViewId2());
        assertEquals(efm1.getInliers(), efm2.getInliers());
        assertNotSame(efm1.getInliers(), efm2.getInliers());
        assertEquals(efm1.getLeftSamples(), efm2.getLeftSamples());
        assertNotSame(efm1.getLeftSamples(), efm2.getLeftSamples());
        assertEquals(efm1.getRightSamples(), efm2.getRightSamples());
        assertNotSame(efm1.getRightSamples(), efm2.getRightSamples());
    }
}
