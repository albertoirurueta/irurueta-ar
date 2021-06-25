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
import com.irurueta.algebra.NotReadyException;
import com.irurueta.algebra.RankDeficientMatrixException;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.ar.SerializationHelper;
import com.irurueta.geometry.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.Arrays;
import java.util.Random;

import static org.junit.Assert.*;

public class EssentialMatrixTest {

    private static final int ESSENTIAL_MATRIX_ROWS = 3;
    private static final int ESSENTIAL_MATRIX_COLS = 3;

    private static final double ABSOLUTE_ERROR = 1e-6;

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
    public void testEmptyConstructor() {
        final EssentialMatrix essentialMatrix = new EssentialMatrix();

        assertFalse(essentialMatrix.isInternalMatrixAvailable());
        try {
            essentialMatrix.getInternalMatrix();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }
    }

    @Test
    public void testConstructorWithInternalMatrix() throws WrongSizeException,
            NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException,
            InvalidEssentialMatrixException, NotAvailableException {
        Matrix internalMatrix = Matrix.createWithUniformRandomValues(
                ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(
                internalMatrix);
        decomposer.decompose();

        Matrix u = decomposer.getU();
        Matrix w = decomposer.getW();
        Matrix v = decomposer.getV();
        Matrix transV = v.transposeAndReturnNew();

        // Set last singular value to zero to enforce rank 2, and set equal
        // singular values for non-zero ones
        w.setElementAt(0, 0, 1.0);
        w.setElementAt(1, 1, 1.0);
        w.setElementAt(2, 2, 0.0);

        internalMatrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(
                transV));

        EssentialMatrix essentialMatrix = new EssentialMatrix(internalMatrix);
        assertTrue(essentialMatrix.isInternalMatrixAvailable());
        assertEquals(essentialMatrix.getInternalMatrix(), internalMatrix);

        // Force IllegalArgumentException
        essentialMatrix = null;
        try {
            essentialMatrix = new EssentialMatrix(internalMatrix, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(essentialMatrix);

        // Force InvalidEssentialMatrixException
        internalMatrix = Matrix.createWithUniformRandomValues(
                ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        decomposer = new SingularValueDecomposer(internalMatrix);
        decomposer.decompose();

        u = decomposer.getU();
        w = decomposer.getW();
        v = decomposer.getV();
        transV = v.transposeAndReturnNew();

        // set non equal singular values
        w.setElementAt(0, 0, 1.0);
        w.setElementAt(1, 1, 2.0);
        w.setElementAt(2, 2, 3.0);

        internalMatrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(
                transV));

        essentialMatrix = null;
        try {
            essentialMatrix = new EssentialMatrix(internalMatrix);
            fail("InvalidEssentialMatrixException expected but not thrown");
        } catch (final InvalidEssentialMatrixException ignore) {
        }
        assertNull(essentialMatrix);
    }

    @Test
    public void testConstructorWithTwoPinholeCameras()
            throws InvalidPairOfCamerasException, WrongSizeException,
            RankDeficientMatrixException, DecomposerException,
            com.irurueta.geometry.estimators.NotReadyException,
            NotAvailableException {
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

        final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS,
                MAX_SKEWNESS);
        final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS,
                MAX_SKEWNESS);

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

        final Point3D cameraCenter1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Point3D cameraCenter2 = new InhomogeneousPoint3D(
                cameraCenter1.getInhomX() + cameraSeparation,
                cameraCenter1.getInhomY() + cameraSeparation,
                cameraCenter1.getInhomZ() + cameraSeparation);

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
                cameraCenter1);
        final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                cameraCenter2);

        // estimate essential matrix using provided cameras
        EssentialMatrix essentialMatrix = new EssentialMatrix(camera1,
                camera2);

        // now normalize cameras by their intrinsic parameters and compute
        // their fundamental matrix
        final Matrix cam1InternalMatrix = camera1.getInternalMatrix();
        final Matrix cam1IntrinsicParameters = intrinsic1.getInternalMatrix();
        final Matrix inverseCam1IntrinsicParameters = Utils.inverse(
                cam1IntrinsicParameters);
        final Matrix newCam1InternalMatrix = inverseCam1IntrinsicParameters.
                multiplyAndReturnNew(cam1InternalMatrix);
        camera1.setInternalMatrix(newCam1InternalMatrix);

        final Matrix cam2InternalMatrix = camera2.getInternalMatrix();
        final Matrix cam2IntrinsicParameters = intrinsic2.getInternalMatrix();
        final Matrix inverseCam2IntrinsicParameters = Utils.inverse(
                cam2IntrinsicParameters);
        final Matrix newCam2InternalMatrix = inverseCam2IntrinsicParameters.
                multiplyAndReturnNew(cam2InternalMatrix);
        camera2.setInternalMatrix(newCam2InternalMatrix);

        // normalize cameras to increase accuracy
        camera1.normalize();
        camera2.normalize();

        final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1,
                camera2);

        // check equality up to scale
        fundamentalMatrix.normalize();
        essentialMatrix.normalize();

        final Matrix fInternal = fundamentalMatrix.getInternalMatrix();
        final Matrix eInternal = essentialMatrix.getInternalMatrix();
        double previousScale = fInternal.getElementAtIndex(0) /
                eInternal.getElementAtIndex(0);
        double currentScale = 0.0;
        for (int i = 1; i < ESSENTIAL_MATRIX_ROWS * ESSENTIAL_MATRIX_COLS; i++) {
            currentScale = fInternal.getElementAtIndex(i) /
                    eInternal.getElementAtIndex(i);
            assertEquals(previousScale - currentScale, 0.0, ABSOLUTE_ERROR);
            previousScale = currentScale;
        }
        assertEquals(previousScale - currentScale, 0.0, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        essentialMatrix = null;
        try {
            essentialMatrix = new EssentialMatrix(camera1, camera2, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(essentialMatrix);
    }

    @Test
    public void testConstructorWithTranslationAndRotation()
            throws InvalidRotationAndTranslationException,
            NotAvailableException, NotReadyException, LockedException,
            DecomposerException, com.irurueta.algebra.NotAvailableException,
            WrongSizeException, InvalidRotationMatrixException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final HomogeneousPoint2D translation = new HomogeneousPoint2D(
                    cameraSeparation, cameraSeparation, cameraSeparation);

            final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                    betaEuler, gammaEuler);

            EssentialMatrix essentialMatrix = new EssentialMatrix(rotation,
                    translation);

            final Matrix internalEssentialMatrix =
                    essentialMatrix.getInternalMatrix();

            final SingularValueDecomposer singularValueDecomposer =
                    new SingularValueDecomposer(internalEssentialMatrix);
            singularValueDecomposer.decompose();
            final Matrix u = singularValueDecomposer.getU();

            final double scaleX = u.getElementAt(0, 2) / translation.getHomX();
            final double scaleY = u.getElementAt(1, 2) / translation.getHomY();
            final double scaleW = u.getElementAt(2, 2) / translation.getHomW();

            assertEquals(scaleX - scaleY, 0.0, ABSOLUTE_ERROR);
            assertEquals(scaleY - scaleW, 0.0, ABSOLUTE_ERROR);
            assertEquals(scaleW - scaleX, 0.0, ABSOLUTE_ERROR);

            final Matrix w = new Matrix(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS);
            w.setElementAt(0, 1, -1.0);
            w.setElementAt(1, 0, 1.0);
            w.setElementAt(2, 2, 1.0);

            final Matrix transW = w.transposeAndReturnNew();
            final Matrix v = singularValueDecomposer.getV();
            final Matrix transV = v.transposeAndReturnNew();

            // First possible rotation
            final Matrix rotation1Matrix = u.multiplyAndReturnNew(
                    w.multiplyAndReturnNew(transV));
            final MatrixRotation3D rotation1 = new MatrixRotation3D(rotation1Matrix);
            // second possible rotation
            final Matrix rotation2Matrix = u.multiplyAndReturnNew(transW.
                    multiplyAndReturnNew(transV));
            final MatrixRotation3D rotation2 = new MatrixRotation3D(rotation2Matrix);

            boolean valid = (Math.abs(Math.abs(rotation1.getAlphaEulerAngle()) -
                    Math.abs(rotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation2.getAlphaEulerAngle()) -
                            Math.abs(rotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation1.getBetaEulerAngle()) -
                    Math.abs(rotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation2.getBetaEulerAngle()) -
                            Math.abs(rotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation1.getGammaEulerAngle()) -
                    Math.abs(rotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation2.getGammaEulerAngle()) -
                            Math.abs(rotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR);

            if (valid) {
                numValid++;
            }

            // Force IllegalArgumentException
            essentialMatrix = null;
            try {
                essentialMatrix = new EssentialMatrix(rotation, translation,
                        -1.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
            assertNull(essentialMatrix);
        }

        assertTrue(numValid > TIMES / 4);
    }

    @Test
    public void testConstructorWithRotationAndCameraCenter()
            throws InvalidRotationAndTranslationException,
            InvalidEssentialMatrixException, NotAvailableException,
            WrongSizeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                    betaEuler, gammaEuler);
            final Point3D center = new InhomogeneousPoint3D(cameraSeparation,
                    cameraSeparation, cameraSeparation);

            // build essential matrix using provided rotation and center
            EssentialMatrix essentialMatrix = new EssentialMatrix(rotation,
                    center);

            // compute possible rotations and translations
            essentialMatrix.computePossibleRotationAndTranslations();

            // check correctness
            final Point2D firstEstimatedTranslation =
                    essentialMatrix.getFirstPossibleTranslation();
            final Point2D secondEstimatedTranslation =
                    essentialMatrix.getSecondPossibleTranslation();
            final MatrixRotation3D firstEstimatedCameraRotation =
                    (MatrixRotation3D) essentialMatrix.getFirstPossibleRotation();
            final MatrixRotation3D secondEstimatedCameraRotation =
                    (MatrixRotation3D) essentialMatrix.getSecondPossibleRotation();

            boolean valid = (Math.abs(Math.abs(rotation.getAlphaEulerAngle()) -
                    Math.abs(firstEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getAlphaEulerAngle()) -
                            Math.abs(secondEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getBetaEulerAngle()) -
                    Math.abs(firstEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getBetaEulerAngle()) -
                            Math.abs(secondEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getGammaEulerAngle()) -
                    Math.abs(firstEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getGammaEulerAngle()) -
                            Math.abs(secondEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR);

            if (valid) {
                numValid++;
            }

            // translation term is equal to t=-R*C, hence we can obtain camera
            // centers as: C = -inv(R)*t = -R'*t
            final Matrix rotationMatrix1 =
                    firstEstimatedCameraRotation.getInternalMatrix();
            final Matrix rotationMatrix2 =
                    secondEstimatedCameraRotation.getInternalMatrix();
            final Matrix transRotationMatrix1 = rotationMatrix1.
                    transposeAndReturnNew();
            final Matrix transRotationMatrix2 = rotationMatrix2.
                    transposeAndReturnNew();
            final Matrix translationMatrix1 = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix1.setElementAt(0, 0,
                    firstEstimatedTranslation.getHomX());
            translationMatrix1.setElementAt(1, 0,
                    firstEstimatedTranslation.getHomY());
            translationMatrix1.setElementAt(2, 0,
                    firstEstimatedTranslation.getHomW());
            final Matrix translationMatrix2 = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix2.setElementAt(0, 0,
                    secondEstimatedTranslation.getHomX());
            translationMatrix2.setElementAt(1, 0,
                    secondEstimatedTranslation.getHomY());
            translationMatrix2.setElementAt(2, 0,
                    secondEstimatedTranslation.getHomW());

            final Matrix centerMatrix1 = transRotationMatrix1.multiplyAndReturnNew(
                    translationMatrix1).multiplyByScalarAndReturnNew(-1.0);
            final Matrix centerMatrix2 = transRotationMatrix1.multiplyAndReturnNew(
                    translationMatrix2).multiplyByScalarAndReturnNew(-1.0);
            final Matrix centerMatrix3 = transRotationMatrix2.multiplyAndReturnNew(
                    translationMatrix1).multiplyByScalarAndReturnNew(-1.0);
            final Matrix centerMatrix4 = transRotationMatrix2.multiplyAndReturnNew(
                    translationMatrix2).multiplyByScalarAndReturnNew(-1.0);

            double scaleX = centerMatrix1.getElementAt(0, 0) / center.getInhomX();
            double scaleY = centerMatrix1.getElementAt(1, 0) / center.getInhomY();
            double scaleZ = centerMatrix1.getElementAt(2, 0) / center.getInhomZ();

            final boolean valid1 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix2.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix2.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix2.getElementAt(2, 0) / center.getInhomZ();

            final boolean valid2 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix3.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix3.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix3.getElementAt(2, 0) / center.getInhomZ();

            final boolean valid3 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix4.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix4.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix4.getElementAt(2, 0) / center.getInhomZ();

            final boolean valid4 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            assertTrue(valid1 || valid2 || valid3 || valid4);

            // Force IllegalArgumentException
            essentialMatrix = null;
            try {
                essentialMatrix = new EssentialMatrix(rotation, center, -1.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
            assertNull(essentialMatrix);
        }
        assertTrue(numValid > TIMES / 4);
    }

    @Test
    public void testConstructorWithFundamentalMatrixAndIntrinsicParameters()
            throws WrongSizeException, NotReadyException, LockedException,
            DecomposerException, com.irurueta.algebra.NotAvailableException,
            InvalidFundamentalMatrixException,
            InvalidPairOfIntrinsicParametersException, NotAvailableException {

        final Matrix a = Matrix.createWithUniformRandomValues(
                ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        decomposer.decompose();
        final Matrix u = decomposer.getU();
        final Matrix w = decomposer.getW();
        final Matrix v = decomposer.getV();
        final Matrix transV = v.transposeAndReturnNew();

        // Set last singular value to zero to enforce rank 2
        w.setElementAt(2, 2, 0.0);
        final Matrix fundamentalInternalMatrix = u.multiplyAndReturnNew(
                w.multiplyAndReturnNew(transV));

        // Creating the intrinsic parameters
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double horizontalFocalLength1 = randomizer.nextDouble(
                MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double verticalFocalLength1 = randomizer.nextDouble(
                MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double horizontalFocalLength2 = randomizer.nextDouble(
                MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double verticalFocalLength2 = randomizer.nextDouble(
                MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS,
                MAX_SKEWNESS);
        final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS,
                MAX_SKEWNESS);

        final double horizontalPrincipalPoint1 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint1 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final PinholeCameraIntrinsicParameters intrinsic1 =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                        verticalFocalLength1, horizontalPrincipalPoint1,
                        verticalPrincipalPoint1, skewness1);
        final PinholeCameraIntrinsicParameters intrinsic2 =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                        verticalFocalLength2, horizontalPrincipalPoint2,
                        verticalPrincipalPoint2, skewness2);

        final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                fundamentalInternalMatrix);

        final EssentialMatrix essentialMatrix = new EssentialMatrix(
                fundamentalMatrix, intrinsic1, intrinsic2);

        final Matrix kLeftMatrix = intrinsic1.getInternalMatrix();
        final double normK1 = Utils.normF(kLeftMatrix);
        final Matrix k1 = kLeftMatrix.multiplyByScalarAndReturnNew(1.0 / normK1);

        final Matrix kRightMatrix = intrinsic2.getInternalMatrix();
        final double normK2 = Utils.normF(kRightMatrix);
        final Matrix k2 = kRightMatrix.multiplyByScalarAndReturnNew(1.0 / normK2);
        final Matrix transK2 = k2.transposeAndReturnNew();

        final double normFund = Utils.normF(fundamentalInternalMatrix);
        final Matrix tempFundMatrix = fundamentalInternalMatrix.
                multiplyByScalarAndReturnNew(1.0 / normFund);

        final Matrix estimatedEssentialMatrix = transK2.multiplyAndReturnNew(
                tempFundMatrix.multiplyAndReturnNew(k1));

        final double normEssential = Utils.normF(estimatedEssentialMatrix);
        estimatedEssentialMatrix.multiplyByScalarAndReturnNew(
                1.0 / normEssential);

        final Matrix eInternal2 = essentialMatrix.getInternalMatrix();
        final double firstScale = eInternal2.getElementAtIndex(0) /
                estimatedEssentialMatrix.getElementAtIndex(0);
        double previousScale = firstScale, currentScale = 0.0;
        for (int i = 1; i < ESSENTIAL_MATRIX_ROWS * ESSENTIAL_MATRIX_COLS; i++) {
            currentScale = eInternal2.getElementAtIndex(i) /
                    estimatedEssentialMatrix.getElementAtIndex(i);
            assertEquals(previousScale - currentScale, 0.0, ABSOLUTE_ERROR);
            previousScale = currentScale;
        }
        assertEquals(currentScale - firstScale, 0.0, ABSOLUTE_ERROR);
    }

    @Test
    public void testSetInternalMatrix() throws WrongSizeException,
            NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException,
            InvalidEssentialMatrixException, NotAvailableException {

        EssentialMatrix essentialMatrix = new EssentialMatrix();

        assertFalse(essentialMatrix.isInternalMatrixAvailable());

        // Force NotAvailableException
        try {
            essentialMatrix.getInternalMatrix();
            fail("NotAvailableException expected but not thrown");
        } catch (final NotAvailableException ignore) {
        }

        Matrix internalMatrix = Matrix.createWithUniformRandomValues(
                ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(
                internalMatrix);
        decomposer.decompose();

        Matrix u = decomposer.getU();
        Matrix w = decomposer.getW();
        Matrix v = decomposer.getV();
        Matrix transV = v.transposeAndReturnNew();

        // Set last singular value to zero to enforce rank 2
        w.setElementAt(0, 0, 1.0);
        w.setElementAt(1, 1, 1.0);
        w.setElementAt(2, 2, 0.0);

        internalMatrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(
                transV));

        essentialMatrix.setInternalMatrix(internalMatrix);

        assertTrue(essentialMatrix.isInternalMatrixAvailable());
        assertEquals(essentialMatrix.getInternalMatrix(), internalMatrix);

        // Force IllegalArgumentException
        try {
            essentialMatrix.setInternalMatrix(internalMatrix, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force InvalidEssentialMatrixException by setting wrong rank
        essentialMatrix = new EssentialMatrix();
        internalMatrix = Matrix.createWithUniformRandomValues(
                ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        decomposer = new SingularValueDecomposer(internalMatrix);
        decomposer.decompose();

        u = decomposer.getU();
        w = decomposer.getW();
        v = decomposer.getV();
        transV = v.transposeAndReturnNew();

        // Enforce wrong rank
        w.setElementAt(0, 0, 1.0);
        w.setElementAt(1, 1, 2.0);
        w.setElementAt(2, 2, 3.0);

        internalMatrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(
                transV));

        try {
            essentialMatrix.setInternalMatrix(internalMatrix);
            fail("InvalidFundamentalMatrixException expected but not " +
                    "thrown");
        } catch (final InvalidEssentialMatrixException ignore) {
        }

        // Force InvalidEssentialMatrixException by setting wrong singular
        // values
        essentialMatrix = new EssentialMatrix();
        internalMatrix = Matrix.createWithUniformRandomValues(
                ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        decomposer = new SingularValueDecomposer(internalMatrix);
        decomposer.decompose();

        u = decomposer.getU();
        w = decomposer.getW();
        v = decomposer.getV();
        transV = v.transposeAndReturnNew();

        // Enforce wrong rank
        w.setElementAt(0, 0, 1.5);
        w.setElementAt(1, 1, 1.0);
        w.setElementAt(2, 2, 0.0);

        internalMatrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(
                transV));

        try {
            essentialMatrix.setInternalMatrix(internalMatrix);
            fail("InvalidFundamentalMatrixException expected but not " +
                    "thrown");
        } catch (final InvalidEssentialMatrixException ignore) {
        }
    }

    @Test
    public void testSetFromPairOfCameras() throws InvalidPairOfCamerasException,
            WrongSizeException, RankDeficientMatrixException,
            DecomposerException,
            com.irurueta.geometry.estimators.NotReadyException,
            NotAvailableException {

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

        final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS,
                MAX_SKEWNESS);
        final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS,
                MAX_SKEWNESS);

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

        final Point3D center1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D center2 = new InhomogeneousPoint3D(
                center1.getInhomX() + cameraSeparation,
                center1.getInhomY() + cameraSeparation,
                center1.getInhomZ() + cameraSeparation);

        final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                betaEuler1, gammaEuler1);
        final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                betaEuler2, gammaEuler2);

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

        final EssentialMatrix essentialMatrix = new EssentialMatrix();

        // set from pair of cameras
        essentialMatrix.setFromPairOfCameras(camera1, camera2);

        // check correctness
        final Matrix cam1InternalMatrix = camera1.getInternalMatrix();
        final Matrix cam1IntrinsicParameters = intrinsic1.getInternalMatrix();
        final Matrix inverseCam1IntrinsicParameters = Utils.inverse(
                cam1IntrinsicParameters);
        final Matrix newCam1InternalMatrix = inverseCam1IntrinsicParameters.
                multiplyAndReturnNew(cam1InternalMatrix);

        camera1.setInternalMatrix(newCam1InternalMatrix);

        final Matrix cam2InternalMatrix = camera2.getInternalMatrix();
        final Matrix cam2IntrinsicParameters = intrinsic2.getInternalMatrix();
        final Matrix inverseCam2IntrinsicParameters = Utils.inverse(
                cam2IntrinsicParameters);
        final Matrix newCam2InternalMatrix = inverseCam2IntrinsicParameters.
                multiplyAndReturnNew(cam2InternalMatrix);

        camera2.setInternalMatrix(newCam2InternalMatrix);

        final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1,
                camera2);

        // check equality up to scale
        fundamentalMatrix.normalize();
        essentialMatrix.normalize();

        final Matrix fInternal = fundamentalMatrix.getInternalMatrix();
        final Matrix eInternal = essentialMatrix.getInternalMatrix();
        final double firstScale = fInternal.getElementAtIndex(0) /
                eInternal.getElementAtIndex(0);
        double previousScale = firstScale, currentScale = 0.0;
        for (int i = 0; i < ESSENTIAL_MATRIX_ROWS * ESSENTIAL_MATRIX_COLS; i++) {
            currentScale = fInternal.getElementAtIndex(i) /
                    eInternal.getElementAtIndex(i);
            assertEquals(previousScale - currentScale, 0.0, ABSOLUTE_ERROR);
            previousScale = currentScale;
        }
        assertEquals(currentScale - firstScale, 0.0, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            essentialMatrix.setFromPairOfCameras(camera1, camera2, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSetFromPairOfCamerasAndIntrinsicsNotAvailable() throws InvalidPairOfCamerasException,
            WrongSizeException, RankDeficientMatrixException,
            DecomposerException,
            com.irurueta.geometry.estimators.NotReadyException,
            NotAvailableException {

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

        final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS,
                MAX_SKEWNESS);
        final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS,
                MAX_SKEWNESS);

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

        final Point3D center1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D center2 = new InhomogeneousPoint3D(
                center1.getInhomX() + cameraSeparation,
                center1.getInhomY() + cameraSeparation,
                center1.getInhomZ() + cameraSeparation);

        final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                betaEuler1, gammaEuler1);
        final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                betaEuler2, gammaEuler2);

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

        final PinholeCamera camera1b = new PinholeCamera(camera1.getInternalMatrix());
        final PinholeCamera camera2b = new PinholeCamera(camera2.getInternalMatrix());

        final EssentialMatrix essentialMatrix = new EssentialMatrix();

        // set from pair of cameras
        essentialMatrix.setFromPairOfCameras(camera1b, camera2b);

        // check correctness
        final Matrix cam1InternalMatrix = camera1.getInternalMatrix();
        final Matrix cam1IntrinsicParameters = intrinsic1.getInternalMatrix();
        final Matrix inverseCam1IntrinsicParameters = Utils.inverse(
                cam1IntrinsicParameters);
        final Matrix newCam1InternalMatrix = inverseCam1IntrinsicParameters.
                multiplyAndReturnNew(cam1InternalMatrix);

        camera1.setInternalMatrix(newCam1InternalMatrix);

        final Matrix cam2InternalMatrix = camera2.getInternalMatrix();
        final Matrix cam2IntrinsicParameters = intrinsic2.getInternalMatrix();
        final Matrix inverseCam2IntrinsicParameters = Utils.inverse(
                cam2IntrinsicParameters);
        final Matrix newCam2InternalMatrix = inverseCam2IntrinsicParameters.
                multiplyAndReturnNew(cam2InternalMatrix);

        camera2.setInternalMatrix(newCam2InternalMatrix);

        final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1,
                camera2);

        // check equality up to scale
        fundamentalMatrix.normalize();
        essentialMatrix.normalize();

        final Matrix fInternal = fundamentalMatrix.getInternalMatrix();
        final Matrix eInternal = essentialMatrix.getInternalMatrix();
        final double firstScale = fInternal.getElementAtIndex(0) /
                eInternal.getElementAtIndex(0);
        double previousScale = firstScale, currentScale = 0.0;
        for (int i = 0; i < ESSENTIAL_MATRIX_ROWS * ESSENTIAL_MATRIX_COLS; i++) {
            currentScale = fInternal.getElementAtIndex(i) /
                    eInternal.getElementAtIndex(i);
            assertEquals(previousScale - currentScale, 0.0, ABSOLUTE_ERROR);
            previousScale = currentScale;
        }
        assertEquals(currentScale - firstScale, 0.0, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            essentialMatrix.setFromPairOfCameras(camera1, camera2, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSetFromPairOfCamerasAndCameraSignFixed() throws InvalidPairOfCamerasException,
            WrongSizeException, RankDeficientMatrixException,
            DecomposerException,
            com.irurueta.geometry.estimators.NotReadyException,
            NotAvailableException, CameraException {

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

        final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS,
                MAX_SKEWNESS);
        final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS,
                MAX_SKEWNESS);

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

        final Point3D center1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final Point3D center2 = new InhomogeneousPoint3D(
                center1.getInhomX() + cameraSeparation,
                center1.getInhomY() + cameraSeparation,
                center1.getInhomZ() + cameraSeparation);

        final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                betaEuler1, gammaEuler1);
        final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                betaEuler2, gammaEuler2);

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
        camera1.fixCameraSign();
        camera2.fixCameraSign();

        final EssentialMatrix essentialMatrix = new EssentialMatrix();

        // set from pair of cameras
        essentialMatrix.setFromPairOfCameras(camera1, camera2);

        // check correctness
        final Matrix cam1InternalMatrix = camera1.getInternalMatrix();
        final Matrix cam1IntrinsicParameters = intrinsic1.getInternalMatrix();
        final Matrix inverseCam1IntrinsicParameters = Utils.inverse(
                cam1IntrinsicParameters);
        final Matrix newCam1InternalMatrix = inverseCam1IntrinsicParameters.
                multiplyAndReturnNew(cam1InternalMatrix);

        camera1.setInternalMatrix(newCam1InternalMatrix);

        final Matrix cam2InternalMatrix = camera2.getInternalMatrix();
        final Matrix cam2IntrinsicParameters = intrinsic2.getInternalMatrix();
        final Matrix inverseCam2IntrinsicParameters = Utils.inverse(
                cam2IntrinsicParameters);
        final Matrix newCam2InternalMatrix = inverseCam2IntrinsicParameters.
                multiplyAndReturnNew(cam2InternalMatrix);

        camera2.setInternalMatrix(newCam2InternalMatrix);

        final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1,
                camera2);

        // check equality up to scale
        fundamentalMatrix.normalize();
        essentialMatrix.normalize();

        final Matrix fInternal = fundamentalMatrix.getInternalMatrix();
        final Matrix eInternal = essentialMatrix.getInternalMatrix();
        final double firstScale = fInternal.getElementAtIndex(0) /
                eInternal.getElementAtIndex(0);
        double previousScale = firstScale, currentScale = 0.0;
        for (int i = 0; i < ESSENTIAL_MATRIX_ROWS * ESSENTIAL_MATRIX_COLS; i++) {
            currentScale = fInternal.getElementAtIndex(i) /
                    eInternal.getElementAtIndex(i);
            assertEquals(previousScale - currentScale, 0.0, ABSOLUTE_ERROR);
            previousScale = currentScale;
        }
        assertEquals(currentScale - firstScale, 0.0, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            essentialMatrix.setFromPairOfCameras(camera1, camera2, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test(expected = InvalidPairOfCamerasException.class)
    public void testSetFromPairOfCamerasWhenNumericallyUnstable() throws InvalidPairOfCamerasException,
            WrongSizeException {

        final Matrix matrix1 = new Matrix(PinholeCamera.PINHOLE_CAMERA_MATRIX_ROWS,
                PinholeCamera.PINHOLE_CAMERA_MATRIX_COLS);
        final Matrix matrix2 = new Matrix(PinholeCamera.PINHOLE_CAMERA_MATRIX_ROWS,
                PinholeCamera.PINHOLE_CAMERA_MATRIX_COLS);
        Arrays.fill(matrix1.getBuffer(), Double.NaN);
        Arrays.fill(matrix2.getBuffer(), Double.NaN);

        final PinholeCamera camera1 = new PinholeCamera(matrix1);
        final PinholeCamera camera2 = new PinholeCamera(matrix2);

        final EssentialMatrix essentialMatrix = new EssentialMatrix();

        // set from pair of cameras
        essentialMatrix.setFromPairOfCameras(camera1, camera2);
    }

    @Test
    public void testSetFromRotationAndTranslation() throws
            InvalidRotationAndTranslationException, NotAvailableException,
            NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException, WrongSizeException,
            InvalidRotationMatrixException {

        int numValid = 0;
        for (int t = 0; t < 2 * TIMES; t++) {

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final Point2D translation = new HomogeneousPoint2D(cameraSeparation,
                    cameraSeparation, cameraSeparation);

            final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                    betaEuler, gammaEuler);

            final EssentialMatrix essentialMatrix = new EssentialMatrix();

            // set from rotation and translation
            essentialMatrix.setFromRotationAndTranslation(rotation,
                    translation);

            // check correctness
            final Matrix internalEssentialMatrix = essentialMatrix.
                    getInternalMatrix();

            final SingularValueDecomposer singularValueDecomposer =
                    new SingularValueDecomposer(internalEssentialMatrix);
            singularValueDecomposer.decompose();

            final Matrix u = singularValueDecomposer.getU();

            final double scaleX = u.getElementAt(0, 2) / translation.getHomX();
            final double scaleY = u.getElementAt(1, 2) / translation.getHomY();
            final double scaleW = u.getElementAt(2, 2) / translation.getHomW();

            assertEquals(scaleX - scaleY, 0.0, ABSOLUTE_ERROR);
            assertEquals(scaleY - scaleW, 0.0, ABSOLUTE_ERROR);
            assertEquals(scaleW - scaleX, 0.0, ABSOLUTE_ERROR);

            final Matrix w = new Matrix(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS);
            w.setElementAt(0, 1, -1.0);
            w.setElementAt(1, 0, 1.0);
            w.setElementAt(2, 2, 1.0);
            final Matrix transW = w.transposeAndReturnNew();
            final Matrix v = singularValueDecomposer.getV();
            final Matrix transV = v.transposeAndReturnNew();

            // First possible rotation
            final Matrix rotation1Matrix = u.multiplyAndReturnNew(
                    w.multiplyAndReturnNew(transV));
            final MatrixRotation3D rotation1 = new MatrixRotation3D(rotation1Matrix);
            // second possible rotation
            final Matrix rotation2Matrix = u.multiplyAndReturnNew(
                    transW.multiplyAndReturnNew(transV));
            final MatrixRotation3D rotation2 = new MatrixRotation3D(rotation2Matrix);

            boolean valid = (Math.abs(Math.abs(rotation1.getAlphaEulerAngle()) -
                    Math.abs(rotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation2.getAlphaEulerAngle()) -
                            Math.abs(rotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation1.getBetaEulerAngle()) -
                    Math.abs(rotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation2.getBetaEulerAngle()) -
                            Math.abs(rotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation1.getGammaEulerAngle()) -
                    Math.abs(rotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation2.getGammaEulerAngle()) -
                            Math.abs(rotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR);

            if (valid) {
                numValid++;
            }

            // Force IllegalArgumentException
            try {
                essentialMatrix.setFromRotationAndTranslation(rotation,
                        translation, -1.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }

            if (numValid > 2 * TIMES / 4) {
                break;
            }
        }

        assertTrue(numValid > 2 * TIMES / 4);
    }

    @Test(expected = InvalidRotationAndTranslationException.class)
    public void testSetFromRotationAndTranslationInvalid() throws
            InvalidRotationAndTranslationException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final Point2D translation = new InhomogeneousPoint2D(Double.NaN,
                Double.NaN);

        final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                betaEuler, gammaEuler);

        final EssentialMatrix essentialMatrix = new EssentialMatrix();

        // set from rotation and translation
        essentialMatrix.setFromRotationAndTranslation(rotation,
                translation);
    }

    @Test
    public void testSetFromFundamentalMatrixAndIntrinsics() throws
            WrongSizeException, NotReadyException, LockedException,
            DecomposerException, com.irurueta.algebra.NotAvailableException,
            InvalidFundamentalMatrixException,
            InvalidPairOfIntrinsicParametersException, NotAvailableException {

        final Matrix a = Matrix.createWithUniformRandomValues(
                ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        decomposer.decompose();
        final Matrix u = decomposer.getU();
        final Matrix w = decomposer.getW();
        final Matrix v = decomposer.getV();

        final Matrix transV = v.transposeAndReturnNew();

        // set last singular value to zero to enforce rank 2
        w.setElementAt(2, 2, 0.0);
        final Matrix fundamentalInternalMatrix = u.multiplyAndReturnNew(
                w.multiplyAndReturnNew(transV));

        // create intrinsic parameters
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double horizontalFocalLength1 = randomizer.nextDouble(
                MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double verticalFocalLength1 = randomizer.nextDouble(
                MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double horizontalFocalLength2 = randomizer.nextDouble(
                MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double verticalFocalLength2 = randomizer.nextDouble(
                MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS,
                MAX_SKEWNESS);
        final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS,
                MAX_SKEWNESS);

        final double horizontalPrincipalPoint1 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint1 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final PinholeCameraIntrinsicParameters intrinsic1 =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                        verticalFocalLength1, horizontalPrincipalPoint1,
                        verticalPrincipalPoint1, skewness1);
        final PinholeCameraIntrinsicParameters intrinsic2 =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                        verticalFocalLength2, horizontalPrincipalPoint2,
                        verticalPrincipalPoint2, skewness2);

        final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                fundamentalInternalMatrix);

        final EssentialMatrix essentialMatrix = new EssentialMatrix();

        // set from fundamental matrix and intrinsics
        essentialMatrix.setFromFundamentalMatrixAndIntrinsics(
                fundamentalMatrix, intrinsic1, intrinsic2);

        final Matrix kLeftMatrix = intrinsic1.getInternalMatrix();
        final double normK1 = Utils.normF(kLeftMatrix);
        final Matrix K1 = kLeftMatrix.multiplyByScalarAndReturnNew(1.0 / normK1);

        final Matrix kRightMatrix = intrinsic2.getInternalMatrix();
        final double normK2 = Utils.normF(kRightMatrix);
        final Matrix K2 = kRightMatrix.multiplyByScalarAndReturnNew(1.0 / normK2);

        final Matrix transK2 = K2.transposeAndReturnNew();

        final double normFund = Utils.normF(fundamentalInternalMatrix);
        final Matrix tempFundMatrix = fundamentalInternalMatrix.
                multiplyByScalarAndReturnNew(1.0 / normFund);

        final Matrix estimatedEssentialMatrix =
                transK2.multiplyAndReturnNew(
                        tempFundMatrix.multiplyAndReturnNew(K1));

        final double normEssential = Utils.normF(estimatedEssentialMatrix);
        estimatedEssentialMatrix.multiplyByScalar(1.0 / normEssential);

        final Matrix eInternal2 = essentialMatrix.getInternalMatrix();
        final double firstScale = eInternal2.getElementAtIndex(0) /
                estimatedEssentialMatrix.getElementAtIndex(0);
        double previousScale = firstScale, currentScale = 0.0;
        for (int i = 1; i < ESSENTIAL_MATRIX_ROWS * ESSENTIAL_MATRIX_COLS; i++) {
            currentScale = eInternal2.getElementAtIndex(i) /
                    estimatedEssentialMatrix.getElementAtIndex(i);
            assertEquals(previousScale - currentScale, 0.0, ABSOLUTE_ERROR);
            previousScale = currentScale;
        }
        assertEquals(currentScale - firstScale, 0.0, ABSOLUTE_ERROR);
    }

    @Test(expected = InvalidPairOfIntrinsicParametersException.class)
    public void testSetFromFundamentalMatrixAndIntrinsicsInvalid() throws
            InvalidPairOfIntrinsicParametersException {

        // create intrinsic parameters
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double horizontalFocalLength1 = randomizer.nextDouble(
                MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double verticalFocalLength1 = randomizer.nextDouble(
                MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double horizontalFocalLength2 = randomizer.nextDouble(
                MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double verticalFocalLength2 = randomizer.nextDouble(
                MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS,
                MAX_SKEWNESS);
        final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS,
                MAX_SKEWNESS);

        final double horizontalPrincipalPoint1 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint1 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final PinholeCameraIntrinsicParameters intrinsic1 =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                        verticalFocalLength1, horizontalPrincipalPoint1,
                        verticalPrincipalPoint1, skewness1);
        final PinholeCameraIntrinsicParameters intrinsic2 =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                        verticalFocalLength2, horizontalPrincipalPoint2,
                        verticalPrincipalPoint2, skewness2);

        final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();

        final EssentialMatrix essentialMatrix = new EssentialMatrix();

        // set from fundamental matrix and intrinsics
        essentialMatrix.setFromFundamentalMatrixAndIntrinsics(
                fundamentalMatrix, intrinsic1, intrinsic2);
    }

    @Test
    public void testToFundamentalMatrix() throws
            EpipolarException,
            com.irurueta.geometry.estimators.NotReadyException,
            NotAvailableException, WrongSizeException, NotReadyException,
            LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix a = Matrix.createWithUniformRandomValues(
                    ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            decomposer.decompose();
            final Matrix u = decomposer.getU();
            final Matrix w = decomposer.getW();
            final Matrix v = decomposer.getV();

            final Matrix transV = v.transposeAndReturnNew();

            // set last singular value to zero to enforce rank 2
            w.setElementAt(2, 2, 0.0);
            final Matrix fundamentalInternalMatrix = u.multiplyAndReturnNew(
                    w.multiplyAndReturnNew(transV));

            // create intrinsic parameters
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS,
                    MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS,
                    MAX_SKEWNESS);

            final double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final FundamentalMatrix fundamentalMatrix1 = new FundamentalMatrix(
                    fundamentalInternalMatrix);

            final EssentialMatrix essential = new EssentialMatrix(fundamentalMatrix1,
                    intrinsic1, intrinsic2);

            final FundamentalMatrix fundamentalMatrix2 =
                    essential.toFundamentalMatrix(intrinsic1, intrinsic2);
            fundamentalMatrix2.normalize();

            final boolean condition = fundamentalMatrix1.getInternalMatrix().equals(
                    fundamentalMatrix2.getInternalMatrix(), ABSOLUTE_ERROR);
            if (!condition) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test(expected = EpipolarException.class)
    public void testToFundamentalMatrixInvalid() throws
            EpipolarException, WrongSizeException, NotReadyException,
            LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException {

        final Matrix a = Matrix.createWithUniformRandomValues(
                ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
        decomposer.decompose();
        final Matrix u = decomposer.getU();
        final Matrix w = decomposer.getW();
        final Matrix v = decomposer.getV();

        final Matrix transV = v.transposeAndReturnNew();

        // set last singular value to zero to enforce rank 2
        w.setElementAt(2, 2, 0.0);
        final Matrix fundamentalInternalMatrix = u.multiplyAndReturnNew(
                w.multiplyAndReturnNew(transV));

        final PinholeCameraIntrinsicParameters intrinsic1 =
                new PinholeCameraIntrinsicParameters(Double.NaN,
                        Double.NaN, Double.NaN,
                        Double.NaN, Double.NaN);
        final PinholeCameraIntrinsicParameters intrinsic2 =
                new PinholeCameraIntrinsicParameters(Double.NaN,
                        Double.NaN, Double.NaN,
                        Double.NaN, Double.NaN);

        final FundamentalMatrix fundamentalMatrix1 = new FundamentalMatrix(
                fundamentalInternalMatrix);

        final EssentialMatrix essential = new EssentialMatrix(fundamentalMatrix1,
                intrinsic1, intrinsic2);

        essential.toFundamentalMatrix(intrinsic1, intrinsic2);
    }

    @Test
    public void testSetFromRotationAndCameraCenter()
            throws InvalidRotationAndTranslationException,
            InvalidEssentialMatrixException, NotAvailableException, WrongSizeException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                    betaEuler, gammaEuler);
            final Point3D center = new InhomogeneousPoint3D(cameraSeparation,
                    cameraSeparation, cameraSeparation);

            final EssentialMatrix essentialMatrix = new EssentialMatrix();

            // set from rotation and camera center
            essentialMatrix.setFromRotationAndCameraCenter(rotation, center);

            // compute possible rotations and translations
            essentialMatrix.computePossibleRotationAndTranslations();

            // check correctness
            final Point2D firstEstimatedTranslation =
                    essentialMatrix.getFirstPossibleTranslation();
            final Point2D secondEstimatedTranslation =
                    essentialMatrix.getSecondPossibleTranslation();
            final MatrixRotation3D firstEstimatedCameraRotation =
                    (MatrixRotation3D) essentialMatrix.getFirstPossibleRotation();
            final MatrixRotation3D secondEstimatedCameraRotation =
                    (MatrixRotation3D) essentialMatrix.getSecondPossibleRotation();

            boolean valid = (Math.abs(Math.abs(rotation.getAlphaEulerAngle()) -
                    Math.abs(firstEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getAlphaEulerAngle()) -
                            Math.abs(secondEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getBetaEulerAngle()) -
                    Math.abs(firstEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getBetaEulerAngle()) -
                            Math.abs(secondEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getGammaEulerAngle()) -
                    Math.abs(firstEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getGammaEulerAngle()) -
                            Math.abs(secondEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR);

            if (valid) {
                numValid++;
            }

            // translation term is equal to t=-R*C, hence we can obtain camera
            // centers as: C = -inv(R)*t = -R'*t
            final Matrix rotationMatrix1 =
                    firstEstimatedCameraRotation.getInternalMatrix();
            final Matrix rotationMatrix2 =
                    secondEstimatedCameraRotation.getInternalMatrix();
            final Matrix transRotationMatrix1 = rotationMatrix1.
                    transposeAndReturnNew();
            final Matrix transRotationMatrix2 = rotationMatrix2.
                    transposeAndReturnNew();
            final Matrix translationMatrix1 = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix1.setElementAt(0, 0,
                    firstEstimatedTranslation.getHomX());
            translationMatrix1.setElementAt(1, 0,
                    firstEstimatedTranslation.getHomY());
            translationMatrix1.setElementAt(2, 0,
                    firstEstimatedTranslation.getHomW());
            final Matrix translationMatrix2 = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix2.setElementAt(0, 0,
                    secondEstimatedTranslation.getHomX());
            translationMatrix2.setElementAt(1, 0,
                    secondEstimatedTranslation.getHomY());
            translationMatrix2.setElementAt(2, 0,
                    secondEstimatedTranslation.getHomW());

            final Matrix centerMatrix1 = transRotationMatrix1.multiplyAndReturnNew(
                    translationMatrix1).multiplyByScalarAndReturnNew(-1.0);
            final Matrix centerMatrix2 = transRotationMatrix1.multiplyAndReturnNew(
                    translationMatrix2).multiplyByScalarAndReturnNew(-1.0);
            final Matrix centerMatrix3 = transRotationMatrix2.multiplyAndReturnNew(
                    translationMatrix1).multiplyByScalarAndReturnNew(-1.0);
            final Matrix centerMatrix4 = transRotationMatrix2.multiplyAndReturnNew(
                    translationMatrix2).multiplyByScalarAndReturnNew(-1.0);

            double scaleX = centerMatrix1.getElementAt(0, 0) / center.getInhomX();
            double scaleY = centerMatrix1.getElementAt(1, 0) / center.getInhomY();
            double scaleZ = centerMatrix1.getElementAt(2, 0) / center.getInhomZ();

            final boolean valid1 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix2.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix2.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix2.getElementAt(2, 0) / center.getInhomZ();

            final boolean valid2 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix3.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix3.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix3.getElementAt(2, 0) / center.getInhomZ();

            final boolean valid3 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix4.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix4.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix4.getElementAt(2, 0) / center.getInhomZ();

            final boolean valid4 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            assertTrue(valid1 || valid2 || valid3 || valid4);

            // Force IllegalArgumentException
            try {
                essentialMatrix.setFromRotationAndCameraCenter(rotation, center,
                        -1.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
        }
        assertTrue(numValid > TIMES / 4);
    }

    @Test(expected = InvalidRotationAndTranslationException.class)
    public void testSetFromRotationAndCameraCenterInvalid()
            throws InvalidRotationAndTranslationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                betaEuler, gammaEuler);
        final Point3D center = new InhomogeneousPoint3D(Double.NaN,
                Double.NaN, Double.NaN);

        final EssentialMatrix essentialMatrix = new EssentialMatrix();

        // set from rotation and camera center
        essentialMatrix.setFromRotationAndCameraCenter(rotation, center);
    }

    @Test
    public void testComputePossibleRotationsAndTranslations()
            throws InvalidRotationAndTranslationException,
            InvalidEssentialMatrixException, NotAvailableException,
            InvalidPairOfIntrinsicParametersException, WrongSizeException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            // Create rotation and translation
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES);
            final double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES);
            final double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES);
            final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler,
                    betaEuler, gammaEuler);
            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
            final Point2D translation = new HomogeneousPoint2D(cameraSeparation,
                    cameraSeparation, cameraSeparation);

            EssentialMatrix essentialMatrix = new EssentialMatrix(rotation,
                    translation);

            // test NotAvailableExceptions
            assertFalse(essentialMatrix.
                    arePossibleRotationsAndTranslationsAvailable());
            try {
                essentialMatrix.getFirstPossibleRotation();
                fail("NotAvailableException expected but not thrown");
            } catch (final NotAvailableException ignore) {
            }
            try {
                essentialMatrix.getSecondPossibleRotation();
                fail("NotAvailableException expected but not thrown");
            } catch (final NotAvailableException ignore) {
            }
            try {
                essentialMatrix.getFirstPossibleTranslation();
                fail("NotAvailableException expected but not thrown");
            } catch (final NotAvailableException ignore) {
            }
            try {
                essentialMatrix.getSecondPossibleTranslation();
                fail("NotAvailableException expected but not thrown");
            } catch (final NotAvailableException ignore) {
            }

            // compute possible rotations and translations
            essentialMatrix.computePossibleRotationAndTranslations();

            // check correctness
            Point2D firstEstimatedTranslation =
                    essentialMatrix.getFirstPossibleTranslation();
            Point2D secondEstimatedTranslation =
                    essentialMatrix.getSecondPossibleTranslation();
            MatrixRotation3D firstEstimatedCameraRotation =
                    (MatrixRotation3D) essentialMatrix.
                            getFirstPossibleRotation();
            MatrixRotation3D secondEstimatedCameraRotation =
                    (MatrixRotation3D) essentialMatrix.
                            getSecondPossibleRotation();

            assertTrue(translation.equals(firstEstimatedTranslation) &&
                    translation.equals(secondEstimatedTranslation));

            boolean valid = (Math.abs(Math.abs(rotation.getAlphaEulerAngle()) -
                    Math.abs(firstEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getAlphaEulerAngle()) -
                            Math.abs(secondEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getBetaEulerAngle()) -
                    Math.abs(firstEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getBetaEulerAngle()) -
                            Math.abs(secondEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getGammaEulerAngle()) -
                    Math.abs(firstEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getGammaEulerAngle()) -
                            Math.abs(secondEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR);

            if (valid) {
                numValid++;
            }

            // testing again from a pair of cameras
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

            final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS,
                    MAX_SKEWNESS);
            final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS,
                    MAX_SKEWNESS);

            final double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            final double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            final PinholeCameraIntrinsicParameters intrinsic1 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                            verticalFocalLength1, horizontalPrincipalPoint1,
                            verticalPrincipalPoint1, skewness1);
            final PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                            verticalFocalLength2, horizontalPrincipalPoint2,
                            verticalPrincipalPoint2, skewness2);

            final Point3D cameraCenter1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D cameraCenter2 = new InhomogeneousPoint3D(cameraSeparation,
                    cameraSeparation, cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1,
                    cameraCenter1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    cameraCenter2);

            // compute their respective fundamental matrix
            final FundamentalMatrix fundamentalMatrix;
            try {
                fundamentalMatrix = new FundamentalMatrix(camera1, camera2);
            } catch (final InvalidPairOfCamerasException e) {
                continue;
            }

            // obtain essential matrix from fundamental matrix and intrinsic
            // parameters of both cameras
            essentialMatrix = new EssentialMatrix(fundamentalMatrix, intrinsic1,
                    intrinsic2);

            // compute rotation and translation
            essentialMatrix.computePossibleRotationAndTranslations();

            // check that rotation is correct
            firstEstimatedTranslation =
                    essentialMatrix.getFirstPossibleTranslation();
            secondEstimatedTranslation =
                    essentialMatrix.getSecondPossibleTranslation();
            firstEstimatedCameraRotation = (MatrixRotation3D) essentialMatrix.
                    getFirstPossibleRotation();
            secondEstimatedCameraRotation = (MatrixRotation3D) essentialMatrix.
                    getSecondPossibleRotation();

            // compute 4 possible camera centers for second camera and check that
            // the translation term is equal to -R*C where R is rotation and C is
            // camera center using inhomogeneous coordinates
            final Matrix rotationMatrix1 = firstEstimatedCameraRotation.
                    getInternalMatrix();
            final Matrix rotationMatrix2 = secondEstimatedCameraRotation.
                    getInternalMatrix();
            final Matrix transRotationMatrix1 = rotationMatrix1.
                    transposeAndReturnNew();
            final Matrix transRotationMatrix2 = rotationMatrix2.
                    transposeAndReturnNew();
            final Matrix translationMatrix1 = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix1.setElementAt(0, 0,
                    firstEstimatedTranslation.getHomX());
            translationMatrix1.setElementAt(1, 0,
                    firstEstimatedTranslation.getHomY());
            translationMatrix1.setElementAt(2, 0,
                    firstEstimatedTranslation.getHomW());
            final Matrix translationMatrix2 = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix2.setElementAt(0, 0,
                    secondEstimatedTranslation.getHomX());
            translationMatrix2.setElementAt(1, 0,
                    secondEstimatedTranslation.getHomY());
            translationMatrix2.setElementAt(2, 0,
                    secondEstimatedTranslation.getHomW());

            final Matrix centerMatrix1 = transRotationMatrix1.multiplyAndReturnNew(
                    translationMatrix1).multiplyByScalarAndReturnNew(-1.0);
            final Matrix centerMatrix2 = transRotationMatrix1.multiplyAndReturnNew(
                    translationMatrix2).multiplyByScalarAndReturnNew(-1.0);
            final Matrix centerMatrix3 = transRotationMatrix2.multiplyAndReturnNew(
                    translationMatrix1).multiplyByScalarAndReturnNew(-1.0);
            final Matrix centerMatrix4 = transRotationMatrix2.multiplyAndReturnNew(
                    translationMatrix2).multiplyByScalarAndReturnNew(-1.0);

            double scaleX = centerMatrix1.getElementAt(0, 0) / cameraCenter2.getInhomX();
            double scaleY = centerMatrix1.getElementAt(1, 0) / cameraCenter2.getInhomY();
            double scaleZ = centerMatrix1.getElementAt(2, 0) / cameraCenter2.getInhomZ();

            final boolean valid1 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix2.getElementAt(0, 0) / cameraCenter2.getInhomX();
            scaleY = centerMatrix2.getElementAt(1, 0) / cameraCenter2.getInhomY();
            scaleZ = centerMatrix2.getElementAt(2, 0) / cameraCenter2.getInhomZ();

            final boolean valid2 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix3.getElementAt(0, 0) / cameraCenter2.getInhomX();
            scaleY = centerMatrix3.getElementAt(1, 0) / cameraCenter2.getInhomY();
            scaleZ = centerMatrix3.getElementAt(2, 0) / cameraCenter2.getInhomZ();

            final boolean valid3 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix4.getElementAt(0, 0) / cameraCenter2.getInhomX();
            scaleY = centerMatrix4.getElementAt(1, 0) / cameraCenter2.getInhomY();
            scaleZ = centerMatrix4.getElementAt(2, 0) / cameraCenter2.getInhomZ();

            final boolean valid4 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            assertTrue(valid1 || valid2 || valid3 || valid4);
        }
        assertTrue(numValid > TIMES / 4);

        // Force InvalidEssentialMatrixException
        final EssentialMatrix essentialMatrix = new EssentialMatrix();
        try {
            essentialMatrix.computePossibleRotationAndTranslations();
            fail("InvalidEssentialMatrixException expected but not thrown");
        } catch (final InvalidEssentialMatrixException ignore) {
        }
    }

    @Test
    public void testIsValidInternalMatrix() throws WrongSizeException,
            NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException {

        // testing invalid essential matrix with rank different of 2
        Matrix internalMatrix = Matrix.createWithUniformRandomValues(
                ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        SingularValueDecomposer decomposer = new SingularValueDecomposer(
                internalMatrix);
        decomposer.decompose();

        Matrix u = decomposer.getU();
        Matrix w = decomposer.getW();
        Matrix v = decomposer.getV();
        Matrix transV = v.transposeAndReturnNew();

        // set all singular values to non-zero to enforce rank 3
        w.setElementAt(0, 0, 1.0);
        w.setElementAt(1, 1, 1.0);
        w.setElementAt(2, 2, 3.0);

        internalMatrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(
                transV));

        assertFalse(EssentialMatrix.isValidInternalMatrix(internalMatrix));

        // testing invalid essential matrix with more than 3 columns and rows
        internalMatrix = Matrix.createWithUniformRandomValues(
                ESSENTIAL_MATRIX_ROWS + 1, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        assertFalse(EssentialMatrix.isValidInternalMatrix(internalMatrix));

        internalMatrix = Matrix.createWithUniformRandomValues(
                ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS + 1,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        assertFalse(EssentialMatrix.isValidInternalMatrix(internalMatrix));

        // testing invalid essential matrix with different singular values
        internalMatrix = Matrix.createWithUniformRandomValues(
                ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        decomposer = new SingularValueDecomposer(internalMatrix);
        decomposer.decompose();

        u = decomposer.getU();
        w = decomposer.getW();
        v = decomposer.getV();
        transV = v.transposeAndReturnNew();

        // set last singular value to zero to enforce rank 2, but set
        // different non-zero singular values
        w.setElementAt(0, 0, 1.0);
        w.setElementAt(1, 1, 1.5);
        w.setElementAt(2, 2, 0.0);

        internalMatrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(
                transV));

        assertFalse(EssentialMatrix.isValidInternalMatrix(internalMatrix));
        assertFalse(EssentialMatrix.isValidInternalMatrix(internalMatrix,
                0.2));

        // setting a large enough threshold makes it valid
        assertTrue(EssentialMatrix.isValidInternalMatrix(internalMatrix,
                0.5 + ABSOLUTE_ERROR));

        // testing a valid essential matrix
        internalMatrix = Matrix.createWithUniformRandomValues(
                ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        decomposer = new SingularValueDecomposer(internalMatrix);
        decomposer.decompose();

        u = decomposer.getU();
        w = decomposer.getW();
        v = decomposer.getV();
        transV = v.transposeAndReturnNew();

        // set last singular value to zero to enforce rank 2
        w.setElementAt(0, 0, 1.0);
        w.setElementAt(1, 1, 1.0);
        w.setElementAt(2, 2, 0.0);

        internalMatrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(
                transV));

        assertTrue(EssentialMatrix.isValidInternalMatrix(internalMatrix));

        // Force IllegalArgumentException
        try {
            EssentialMatrix.isValidInternalMatrix(internalMatrix, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Test for matrix with numerical instabilities
        internalMatrix = new Matrix(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS);
        Arrays.fill(internalMatrix.getBuffer(), Double.NaN);

        assertFalse(EssentialMatrix.isValidInternalMatrix(internalMatrix));
    }

    @Test
    public void testSerializeDeserialize() throws InvalidPairOfCamerasException,
            IOException, ClassNotFoundException, InvalidEssentialMatrixException,
            NotAvailableException {
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

        final double skewness1 = randomizer.nextDouble(MIN_SKEWNESS,
                MAX_SKEWNESS);
        final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS,
                MAX_SKEWNESS);

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

        final Point3D cameraCenter1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Point3D cameraCenter2 = new InhomogeneousPoint3D(
                cameraCenter1.getInhomX() + cameraSeparation,
                cameraCenter1.getInhomY() + cameraSeparation,
                cameraCenter1.getInhomZ() + cameraSeparation);

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
                cameraCenter1);
        final PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                cameraCenter2);

        // estimate essential matrix using provided cameras
        final EssentialMatrix essentialMatrix1 = new EssentialMatrix(camera1,
                camera2);
        essentialMatrix1.computePossibleRotationAndTranslations();

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(essentialMatrix1);
        final EssentialMatrix essentialMatrix2 =
                SerializationHelper.deserialize(bytes);

        // check
        assertEquals(essentialMatrix1.getInternalMatrix(), essentialMatrix2.getInternalMatrix());
        assertEquals(essentialMatrix1.getFirstPossibleRotation(),
                essentialMatrix2.getFirstPossibleRotation());
        assertEquals(essentialMatrix1.getSecondPossibleRotation(),
                essentialMatrix2.getSecondPossibleRotation());
        assertEquals(essentialMatrix1.getFirstPossibleTranslation(),
                essentialMatrix2.getFirstPossibleTranslation());
        assertEquals(essentialMatrix1.getSecondPossibleTranslation(),
                essentialMatrix2.getSecondPossibleTranslation());
        assertEquals(essentialMatrix1.arePossibleRotationsAndTranslationsAvailable(),
                essentialMatrix2.arePossibleRotationsAndTranslationsAvailable());
    }
}
