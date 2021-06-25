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
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class EstimatedFundamentalMatrixTest {

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
    public void testConstructor() {
        final EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();

        // check default values
        assertNull(efm.getId());
        assertNull(efm.getFundamentalMatrix());
        assertEquals(efm.getQualityScore(),
                EstimatedFundamentalMatrix.DEFAULT_QUALITY_SCORE, 0.0);
        assertNull(efm.getCovariance());
        assertEquals(efm.getViewId1(), 0);
        assertEquals(efm.getViewId2(), 0);
        assertNull(efm.getInliers());
        assertNull(efm.getLeftSamples());
        assertNull(efm.getRightSamples());
    }

    @Test
    public void testGetSetId() {
        final EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();

        // check default value
        assertNull(efm.getId());

        // set new value
        efm.setId("id");

        // check correctness
        assertEquals(efm.getId(), "id");
    }

    @Test
    public void testGetSetFundamentalMatrix() {
        final EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();

        // check default value
        assertNull(efm.getFundamentalMatrix());

        // set new value
        final FundamentalMatrix f = new FundamentalMatrix();
        efm.setFundamentalMatrix(f);

        // check correctness
        assertSame(efm.getFundamentalMatrix(), f);
    }

    @Test
    public void testGetSetQualityScore() {
        final EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();

        // check default value
        assertEquals(efm.getQualityScore(),
                EstimatedFundamentalMatrix.DEFAULT_QUALITY_SCORE, 0.0);

        // set new value
        efm.setQualityScore(5.0);

        // check correctness
        assertEquals(efm.getQualityScore(), 5.0, 0.0);
    }

    @Test
    public void testGetSetCovariance() throws WrongSizeException {
        final EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();

        // check default value
        assertNull(efm.getCovariance());

        // set new value
        final Matrix cov = new Matrix(9, 9);
        efm.setCovariance(cov);

        // check correctness
        assertSame(efm.getCovariance(), cov);
    }

    @Test
    public void testGetSetViewId1() {
        final EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();

        // check default value
        assertEquals(efm.getViewId1(), 0);

        // set new value
        efm.setViewId1(5);

        // check correctness
        assertEquals(efm.getViewId1(), 5);
    }

    @Test
    public void testGetSetViewId2() {
        final EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();

        // check default value
        assertEquals(efm.getViewId2(), 0);

        // set new value
        efm.setViewId2(10);

        // check correctness
        assertEquals(efm.getViewId2(), 10);
    }

    @Test
    public void testGetSetInliers() {
        final EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();

        // check default value
        assertNull(efm.getInliers());

        // set new value
        final BitSet inliers = new BitSet();
        efm.setInliers(inliers);

        // check correctness
        assertSame(efm.getInliers(), inliers);
    }

    @Test
    public void testGetSetLeftSamples() {
        final EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();

        // check default value
        assertNull(efm.getLeftSamples());

        // set new value
        final List<Sample2D> leftSamples = new ArrayList<>();
        efm.setLeftSamples(leftSamples);

        // check correctness
        assertSame(efm.getLeftSamples(), leftSamples);
    }

    @Test
    public void testGetSetRightSamples() {
        final EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();

        // check default value
        assertNull(efm.getRightSamples());

        // set new value
        final List<Sample2D> rightSamples = new ArrayList<>();
        efm.setRightSamples(rightSamples);

        // check correctness
        assertSame(efm.getRightSamples(), rightSamples);
    }

    @Test
    public void testSerializeDeserialize() throws InvalidPairOfCamerasException, WrongSizeException, IOException, ClassNotFoundException, NotAvailableException {
        final EstimatedFundamentalMatrix efm1 = new EstimatedFundamentalMatrix();

        // set new values
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

        efm1.setId("id");
        efm1.setFundamentalMatrix(fundMatrix);
        efm1.setQualityScore(0.2);
        final Matrix cov = new Matrix(9, 9);
        efm1.setCovariance(cov);
        efm1.setViewId1(1);
        efm1.setViewId2(2);
        final BitSet inliers = new BitSet();
        efm1.setInliers(inliers);
        final List<Sample2D> leftSamples = new ArrayList<>();
        efm1.setLeftSamples(leftSamples);
        final List<Sample2D> rightSamples = new ArrayList<>();
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
        final byte[] bytes = SerializationHelper.serialize(efm1);
        final EstimatedFundamentalMatrix efm2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(efm1.getId(), efm2.getId());
        assertEquals(efm1.getFundamentalMatrix().getInternalMatrix(),
                efm2.getFundamentalMatrix().getInternalMatrix());
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
