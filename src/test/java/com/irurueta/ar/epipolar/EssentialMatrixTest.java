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
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;

class EssentialMatrixTest {

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
    void testEmptyConstructor() {
        final var essentialMatrix = new EssentialMatrix();

        assertFalse(essentialMatrix.isInternalMatrixAvailable());
        assertThrows(NotAvailableException.class, essentialMatrix::getInternalMatrix);
    }

    @Test
    void testConstructorWithInternalMatrix() throws WrongSizeException, NotReadyException, LockedException,
            DecomposerException, com.irurueta.algebra.NotAvailableException, InvalidEssentialMatrixException,
            NotAvailableException {
        final var internalMatrix1 = Matrix.createWithUniformRandomValues(
                ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var decomposer = new SingularValueDecomposer(internalMatrix1);
        decomposer.decompose();

        var u = decomposer.getU();
        var w = decomposer.getW();
        var v = decomposer.getV();
        var transV = v.transposeAndReturnNew();

        // Set last singular value to zero to enforce rank 2, and set equal
        // singular values for non-zero ones
        w.setElementAt(0, 0, 1.0);
        w.setElementAt(1, 1, 1.0);
        w.setElementAt(2, 2, 0.0);

        final var internalMatrix2 = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));

        EssentialMatrix essentialMatrix = new EssentialMatrix(internalMatrix2);
        assertTrue(essentialMatrix.isInternalMatrixAvailable());
        assertEquals(essentialMatrix.getInternalMatrix(), internalMatrix2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new EssentialMatrix(internalMatrix2,
                -1.0));

        // Force InvalidEssentialMatrixException
        final var internalMatrix3 = Matrix.createWithUniformRandomValues(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        decomposer = new SingularValueDecomposer(internalMatrix3);
        decomposer.decompose();

        u = decomposer.getU();
        w = decomposer.getW();
        v = decomposer.getV();
        transV = v.transposeAndReturnNew();

        // set non equal singular values
        w.setElementAt(0, 0, 1.0);
        w.setElementAt(1, 1, 2.0);
        w.setElementAt(2, 2, 3.0);

        final var internalMatrix4 = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));
        assertThrows(InvalidEssentialMatrixException.class, () -> new EssentialMatrix(internalMatrix4));
    }

    @Test
    void testConstructorWithTwoPinholeCameras() throws InvalidPairOfCamerasException, WrongSizeException,
            RankDeficientMatrixException, DecomposerException, com.irurueta.geometry.estimators.NotReadyException,
            NotAvailableException {
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

        final var cameraCenter1 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var cameraCenter2 = new InhomogeneousPoint3D(cameraCenter1.getInhomX() + cameraSeparation,
                cameraCenter1.getInhomY() + cameraSeparation, cameraCenter1.getInhomZ() + cameraSeparation);

        final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
        final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

        final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
        final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

        final var camera1 = new PinholeCamera(intrinsic1, rotation1, cameraCenter1);
        final var camera2 = new PinholeCamera(intrinsic2, rotation2, cameraCenter2);

        // estimate essential matrix using provided cameras
        var essentialMatrix = new EssentialMatrix(camera1, camera2);

        // now normalize cameras by their intrinsic parameters and compute
        // their fundamental matrix
        final var cam1InternalMatrix = camera1.getInternalMatrix();
        final var cam1IntrinsicParameters = intrinsic1.getInternalMatrix();
        final var inverseCam1IntrinsicParameters = Utils.inverse(cam1IntrinsicParameters);
        final var newCam1InternalMatrix = inverseCam1IntrinsicParameters.multiplyAndReturnNew(cam1InternalMatrix);
        camera1.setInternalMatrix(newCam1InternalMatrix);

        final var cam2InternalMatrix = camera2.getInternalMatrix();
        final var cam2IntrinsicParameters = intrinsic2.getInternalMatrix();
        final var inverseCam2IntrinsicParameters = Utils.inverse(cam2IntrinsicParameters);
        final var newCam2InternalMatrix = inverseCam2IntrinsicParameters.multiplyAndReturnNew(cam2InternalMatrix);
        camera2.setInternalMatrix(newCam2InternalMatrix);

        // normalize cameras to increase accuracy
        camera1.normalize();
        camera2.normalize();

        final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

        // check equality up to scale
        fundamentalMatrix.normalize();
        essentialMatrix.normalize();

        final var fInternal = fundamentalMatrix.getInternalMatrix();
        final var eInternal = essentialMatrix.getInternalMatrix();
        var previousScale = fInternal.getElementAtIndex(0) / eInternal.getElementAtIndex(0);
        var currentScale = 0.0;
        for (var i = 1; i < ESSENTIAL_MATRIX_ROWS * ESSENTIAL_MATRIX_COLS; i++) {
            currentScale = fInternal.getElementAtIndex(i) / eInternal.getElementAtIndex(i);
            assertEquals(0.0, previousScale - currentScale, ABSOLUTE_ERROR);
            previousScale = currentScale;
        }
        assertEquals(0.0, previousScale - currentScale, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new EssentialMatrix(camera1, camera2, -1.0));
    }

    @Test
    void testConstructorWithTranslationAndRotation() throws InvalidRotationAndTranslationException,
            NotAvailableException, NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException, WrongSizeException, InvalidRotationMatrixException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var translation = new HomogeneousPoint2D(cameraSeparation, cameraSeparation, cameraSeparation);

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            var essentialMatrix = new EssentialMatrix(rotation, translation);

            final var internalEssentialMatrix = essentialMatrix.getInternalMatrix();

            final var singularValueDecomposer = new SingularValueDecomposer(internalEssentialMatrix);
            singularValueDecomposer.decompose();
            final var u = singularValueDecomposer.getU();

            final var scaleX = u.getElementAt(0, 2) / translation.getHomX();
            final var scaleY = u.getElementAt(1, 2) / translation.getHomY();
            final var scaleW = u.getElementAt(2, 2) / translation.getHomW();

            assertEquals(0.0, scaleX - scaleY, ABSOLUTE_ERROR);
            assertEquals(0.0, scaleY - scaleW, ABSOLUTE_ERROR);
            assertEquals(0.0, scaleW - scaleX, ABSOLUTE_ERROR);

            final var w = new Matrix(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS);
            w.setElementAt(0, 1, -1.0);
            w.setElementAt(1, 0, 1.0);
            w.setElementAt(2, 2, 1.0);

            final var transW = w.transposeAndReturnNew();
            final var v = singularValueDecomposer.getV();
            final var transV = v.transposeAndReturnNew();

            // First possible rotation
            final var rotation1Matrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));
            final var rotation1 = new MatrixRotation3D(rotation1Matrix);
            // second possible rotation
            final var rotation2Matrix = u.multiplyAndReturnNew(transW.multiplyAndReturnNew(transV));
            final var rotation2 = new MatrixRotation3D(rotation2Matrix);

            var valid = (Math.abs(Math.abs(rotation1.getAlphaEulerAngle())
                    - Math.abs(rotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR
                    || Math.abs(Math.abs(rotation2.getAlphaEulerAngle())
                    - Math.abs(rotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation1.getBetaEulerAngle())
                    - Math.abs(rotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR
                    || Math.abs(Math.abs(rotation2.getBetaEulerAngle())
                    - Math.abs(rotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation1.getGammaEulerAngle())
                    - Math.abs(rotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR
                    || Math.abs(Math.abs(rotation2.getGammaEulerAngle())
                    - Math.abs(rotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR);

            if (valid) {
                numValid++;
            }

            // Force IllegalArgumentException
            assertThrows(IllegalArgumentException.class, () -> new EssentialMatrix(rotation, translation,
                    -1.0));
        }

        assertTrue(numValid > TIMES / 4);
    }

    @Test
    void testConstructorWithRotationAndCameraCenter() throws InvalidRotationAndTranslationException,
            InvalidEssentialMatrixException, NotAvailableException, WrongSizeException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
            final var center = new InhomogeneousPoint3D(cameraSeparation, cameraSeparation, cameraSeparation);

            // build essential matrix using provided rotation and center
            var essentialMatrix = new EssentialMatrix(rotation, center);

            // compute possible rotations and translations
            essentialMatrix.computePossibleRotationAndTranslations();

            // check correctness
            final var firstEstimatedTranslation = essentialMatrix.getFirstPossibleTranslation();
            final var secondEstimatedTranslation = essentialMatrix.getSecondPossibleTranslation();
            final var firstEstimatedCameraRotation = (MatrixRotation3D) essentialMatrix.getFirstPossibleRotation();
            final var secondEstimatedCameraRotation = (MatrixRotation3D) essentialMatrix.getSecondPossibleRotation();

            var valid = (Math.abs(Math.abs(rotation.getAlphaEulerAngle())
                    - Math.abs(firstEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR
                    || Math.abs(Math.abs(rotation.getAlphaEulerAngle())
                    - Math.abs(secondEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getBetaEulerAngle())
                    - Math.abs(firstEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR
                    || Math.abs(Math.abs(rotation.getBetaEulerAngle())
                    - Math.abs(secondEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getGammaEulerAngle())
                    - Math.abs(firstEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR
                    || Math.abs(Math.abs(rotation.getGammaEulerAngle())
                    - Math.abs(secondEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR);

            if (valid) {
                numValid++;
            }

            // translation term is equal to t=-R*C, hence we can obtain camera
            // centers as: C = -inv(R)*t = -R'*t
            final var rotationMatrix1 = firstEstimatedCameraRotation.getInternalMatrix();
            final var rotationMatrix2 = secondEstimatedCameraRotation.getInternalMatrix();
            final var transRotationMatrix1 = rotationMatrix1.transposeAndReturnNew();
            final var transRotationMatrix2 = rotationMatrix2.transposeAndReturnNew();
            final var translationMatrix1 = new Matrix(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix1.setElementAt(0, 0, firstEstimatedTranslation.getHomX());
            translationMatrix1.setElementAt(1, 0, firstEstimatedTranslation.getHomY());
            translationMatrix1.setElementAt(2, 0, firstEstimatedTranslation.getHomW());
            final var translationMatrix2 = new Matrix(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix2.setElementAt(0, 0, secondEstimatedTranslation.getHomX());
            translationMatrix2.setElementAt(1, 0, secondEstimatedTranslation.getHomY());
            translationMatrix2.setElementAt(2, 0, secondEstimatedTranslation.getHomW());

            final var centerMatrix1 = transRotationMatrix1.multiplyAndReturnNew(
                    translationMatrix1).multiplyByScalarAndReturnNew(-1.0);
            final var centerMatrix2 = transRotationMatrix1.multiplyAndReturnNew(
                    translationMatrix2).multiplyByScalarAndReturnNew(-1.0);
            final var centerMatrix3 = transRotationMatrix2.multiplyAndReturnNew(
                    translationMatrix1).multiplyByScalarAndReturnNew(-1.0);
            final var centerMatrix4 = transRotationMatrix2.multiplyAndReturnNew(
                    translationMatrix2).multiplyByScalarAndReturnNew(-1.0);

            var scaleX = centerMatrix1.getElementAt(0, 0) / center.getInhomX();
            var scaleY = centerMatrix1.getElementAt(1, 0) / center.getInhomY();
            var scaleZ = centerMatrix1.getElementAt(2, 0) / center.getInhomZ();

            final var valid1 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR)
                    && (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) && (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix2.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix2.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix2.getElementAt(2, 0) / center.getInhomZ();

            final var valid2 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR)
                    && (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) && (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix3.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix3.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix3.getElementAt(2, 0) / center.getInhomZ();

            final var valid3 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR)
                    && (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) && (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix4.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix4.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix4.getElementAt(2, 0) / center.getInhomZ();

            final var valid4 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR)
                    && (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) && (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            assertTrue(valid1 || valid2 || valid3 || valid4);

            // Force IllegalArgumentException
            assertThrows(IllegalArgumentException.class,
                    () -> new EssentialMatrix(rotation, center, -1.0));
        }
        assertTrue(numValid > TIMES / 4);
    }

    @Test
    void testConstructorWithFundamentalMatrixAndIntrinsicParameters() throws WrongSizeException, NotReadyException,
            LockedException, DecomposerException, com.irurueta.algebra.NotAvailableException,
            InvalidFundamentalMatrixException, InvalidPairOfIntrinsicParametersException, NotAvailableException {

        final var a = Matrix.createWithUniformRandomValues(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var decomposer = new SingularValueDecomposer(a);
        decomposer.decompose();
        final var u = decomposer.getU();
        final var w = decomposer.getW();
        final var v = decomposer.getV();
        final var transV = v.transposeAndReturnNew();

        // Set last singular value to zero to enforce rank 2
        w.setElementAt(2, 2, 0.0);
        final var fundamentalInternalMatrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));

        // Creating the intrinsic parameters
        final var randomizer = new UniformRandomizer();
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

        final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
        final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

        final var fundamentalMatrix = new FundamentalMatrix(fundamentalInternalMatrix);

        final var essentialMatrix = new EssentialMatrix(fundamentalMatrix, intrinsic1, intrinsic2);

        final var kLeftMatrix = intrinsic1.getInternalMatrix();
        final var normK1 = Utils.normF(kLeftMatrix);
        final var k1 = kLeftMatrix.multiplyByScalarAndReturnNew(1.0 / normK1);

        final var kRightMatrix = intrinsic2.getInternalMatrix();
        final var normK2 = Utils.normF(kRightMatrix);
        final var k2 = kRightMatrix.multiplyByScalarAndReturnNew(1.0 / normK2);
        final var transK2 = k2.transposeAndReturnNew();

        final var normFund = Utils.normF(fundamentalInternalMatrix);
        final var tempFundMatrix = fundamentalInternalMatrix.multiplyByScalarAndReturnNew(1.0 / normFund);

        final var estimatedEssentialMatrix = transK2.multiplyAndReturnNew(tempFundMatrix.multiplyAndReturnNew(k1));

        final var normEssential = Utils.normF(estimatedEssentialMatrix);
        estimatedEssentialMatrix.multiplyByScalarAndReturnNew(1.0 / normEssential);

        final var eInternal2 = essentialMatrix.getInternalMatrix();
        final var firstScale = eInternal2.getElementAtIndex(0) / estimatedEssentialMatrix.getElementAtIndex(0);
        var previousScale = firstScale;
        var currentScale = 0.0;
        for (var i = 1; i < ESSENTIAL_MATRIX_ROWS * ESSENTIAL_MATRIX_COLS; i++) {
            currentScale = eInternal2.getElementAtIndex(i) / estimatedEssentialMatrix.getElementAtIndex(i);
            assertEquals(0.0, previousScale - currentScale, ABSOLUTE_ERROR);
            previousScale = currentScale;
        }
        assertEquals(0.0, currentScale - firstScale, ABSOLUTE_ERROR);
    }

    @Test
    void testSetInternalMatrix() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException, InvalidEssentialMatrixException, NotAvailableException {

        final var essentialMatrix1 = new EssentialMatrix();

        assertFalse(essentialMatrix1.isInternalMatrixAvailable());

        // Force NotAvailableException
        assertThrows(NotAvailableException.class, essentialMatrix1::getInternalMatrix);

        final var internalMatrix1 = Matrix.createWithUniformRandomValues(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var decomposer = new SingularValueDecomposer(internalMatrix1);
        decomposer.decompose();

        var u = decomposer.getU();
        var w = decomposer.getW();
        var v = decomposer.getV();
        var transV = v.transposeAndReturnNew();

        // Set last singular value to zero to enforce rank 2
        w.setElementAt(0, 0, 1.0);
        w.setElementAt(1, 1, 1.0);
        w.setElementAt(2, 2, 0.0);

        final var internalMatrix2 = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));

        essentialMatrix1.setInternalMatrix(internalMatrix2);

        assertTrue(essentialMatrix1.isInternalMatrixAvailable());
        assertEquals(internalMatrix2, essentialMatrix1.getInternalMatrix());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> essentialMatrix1.setInternalMatrix(internalMatrix2,
                -1.0));

        // Force InvalidEssentialMatrixException by setting wrong rank
        final var essentialMatrix2 = new EssentialMatrix();
        final var internalMatrix3 = Matrix.createWithUniformRandomValues(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        decomposer = new SingularValueDecomposer(internalMatrix3);
        decomposer.decompose();

        u = decomposer.getU();
        w = decomposer.getW();
        v = decomposer.getV();
        transV = v.transposeAndReturnNew();

        // Enforce wrong rank
        w.setElementAt(0, 0, 1.0);
        w.setElementAt(1, 1, 2.0);
        w.setElementAt(2, 2, 3.0);

        final var internalMatrix4 = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));
        assertThrows(InvalidEssentialMatrixException.class, () -> essentialMatrix2.setInternalMatrix(internalMatrix4));

        // Force InvalidEssentialMatrixException by setting wrong singular
        // values
        final var essentialMatrix3 = new EssentialMatrix();
        final var internalMatrix5 = Matrix.createWithUniformRandomValues(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        decomposer = new SingularValueDecomposer(internalMatrix5);
        decomposer.decompose();

        u = decomposer.getU();
        w = decomposer.getW();
        v = decomposer.getV();
        transV = v.transposeAndReturnNew();

        // Enforce wrong rank
        w.setElementAt(0, 0, 1.5);
        w.setElementAt(1, 1, 1.0);
        w.setElementAt(2, 2, 0.0);

        final var internalMatrix6 = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));
        assertThrows(InvalidEssentialMatrixException.class, () -> essentialMatrix3.setInternalMatrix(internalMatrix6));
    }

    @Test
    void testSetFromPairOfCameras() throws InvalidPairOfCamerasException, WrongSizeException,
            RankDeficientMatrixException, DecomposerException, com.irurueta.geometry.estimators.NotReadyException,
            NotAvailableException {

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
        final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

        final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
        final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

        final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
        final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

        final var camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
        final var camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

        final var essentialMatrix = new EssentialMatrix();

        // set from a pair of cameras
        essentialMatrix.setFromPairOfCameras(camera1, camera2);

        // check correctness
        final var cam1InternalMatrix = camera1.getInternalMatrix();
        final var cam1IntrinsicParameters = intrinsic1.getInternalMatrix();
        final var inverseCam1IntrinsicParameters = Utils.inverse(cam1IntrinsicParameters);
        final var newCam1InternalMatrix = inverseCam1IntrinsicParameters.multiplyAndReturnNew(cam1InternalMatrix);

        camera1.setInternalMatrix(newCam1InternalMatrix);

        final var cam2InternalMatrix = camera2.getInternalMatrix();
        final var cam2IntrinsicParameters = intrinsic2.getInternalMatrix();
        final var inverseCam2IntrinsicParameters = Utils.inverse(cam2IntrinsicParameters);
        final var newCam2InternalMatrix = inverseCam2IntrinsicParameters.multiplyAndReturnNew(cam2InternalMatrix);

        camera2.setInternalMatrix(newCam2InternalMatrix);

        final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

        // check equality up to scale
        fundamentalMatrix.normalize();
        essentialMatrix.normalize();

        final var fInternal = fundamentalMatrix.getInternalMatrix();
        final var eInternal = essentialMatrix.getInternalMatrix();
        final var firstScale = fInternal.getElementAtIndex(0) / eInternal.getElementAtIndex(0);
        var previousScale = firstScale;
        var currentScale = 0.0;
        for (var i = 0; i < ESSENTIAL_MATRIX_ROWS * ESSENTIAL_MATRIX_COLS; i++) {
            currentScale = fInternal.getElementAtIndex(i) / eInternal.getElementAtIndex(i);
            assertEquals(0.0, previousScale - currentScale, ABSOLUTE_ERROR);
            previousScale = currentScale;
        }
        assertEquals(0.0, currentScale - firstScale, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> essentialMatrix.setFromPairOfCameras(camera1, camera2, -1.0));
    }

    @Test
    void testSetFromPairOfCamerasAndIntrinsicsNotAvailable() throws InvalidPairOfCamerasException, WrongSizeException,
            RankDeficientMatrixException, DecomposerException, com.irurueta.geometry.estimators.NotReadyException,
            NotAvailableException {

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
        final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

        final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
        final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

        final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
        final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

        final var camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
        final var camera2 = new PinholeCamera(intrinsic2, rotation2, center2);

        final var camera1b = new PinholeCamera(camera1.getInternalMatrix());
        final var camera2b = new PinholeCamera(camera2.getInternalMatrix());

        final var essentialMatrix = new EssentialMatrix();

        // set from a pair of cameras
        essentialMatrix.setFromPairOfCameras(camera1b, camera2b);

        // check correctness
        final var cam1InternalMatrix = camera1.getInternalMatrix();
        final var cam1IntrinsicParameters = intrinsic1.getInternalMatrix();
        final var inverseCam1IntrinsicParameters = Utils.inverse(cam1IntrinsicParameters);
        final var newCam1InternalMatrix = inverseCam1IntrinsicParameters.multiplyAndReturnNew(cam1InternalMatrix);

        camera1.setInternalMatrix(newCam1InternalMatrix);

        final var cam2InternalMatrix = camera2.getInternalMatrix();
        final var cam2IntrinsicParameters = intrinsic2.getInternalMatrix();
        final var inverseCam2IntrinsicParameters = Utils.inverse(cam2IntrinsicParameters);
        final var newCam2InternalMatrix = inverseCam2IntrinsicParameters.multiplyAndReturnNew(cam2InternalMatrix);

        camera2.setInternalMatrix(newCam2InternalMatrix);

        final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

        // check equality up to scale
        fundamentalMatrix.normalize();
        essentialMatrix.normalize();

        final var fInternal = fundamentalMatrix.getInternalMatrix();
        final var eInternal = essentialMatrix.getInternalMatrix();
        final var firstScale = fInternal.getElementAtIndex(0) / eInternal.getElementAtIndex(0);
        var previousScale = firstScale;
        var currentScale = 0.0;
        for (var i = 0; i < ESSENTIAL_MATRIX_ROWS * ESSENTIAL_MATRIX_COLS; i++) {
            currentScale = fInternal.getElementAtIndex(i) / eInternal.getElementAtIndex(i);
            assertEquals(0.0, previousScale - currentScale, ABSOLUTE_ERROR);
            previousScale = currentScale;
        }
        assertEquals(0.0, currentScale - firstScale, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> essentialMatrix.setFromPairOfCameras(camera1, camera2, -1.0));
    }

    @Test
    void testSetFromPairOfCamerasAndCameraSignFixed() throws InvalidPairOfCamerasException, WrongSizeException,
            RankDeficientMatrixException, DecomposerException, com.irurueta.geometry.estimators.NotReadyException,
            NotAvailableException, CameraException {

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
        final var center2 = new InhomogeneousPoint3D(center1.getInhomX() + cameraSeparation,
                center1.getInhomY() + cameraSeparation, center1.getInhomZ() + cameraSeparation);

        final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
        final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

        final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
        final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

        final var camera1 = new PinholeCamera(intrinsic1, rotation1, center1);
        final var camera2 = new PinholeCamera(intrinsic2, rotation2, center2);
        camera1.fixCameraSign();
        camera2.fixCameraSign();

        final var essentialMatrix = new EssentialMatrix();

        // set from a pair of cameras
        essentialMatrix.setFromPairOfCameras(camera1, camera2);

        // check correctness
        final var cam1InternalMatrix = camera1.getInternalMatrix();
        final var cam1IntrinsicParameters = intrinsic1.getInternalMatrix();
        final var inverseCam1IntrinsicParameters = Utils.inverse(cam1IntrinsicParameters);
        final var newCam1InternalMatrix = inverseCam1IntrinsicParameters.multiplyAndReturnNew(cam1InternalMatrix);

        camera1.setInternalMatrix(newCam1InternalMatrix);

        final var cam2InternalMatrix = camera2.getInternalMatrix();
        final var cam2IntrinsicParameters = intrinsic2.getInternalMatrix();
        final var inverseCam2IntrinsicParameters = Utils.inverse(cam2IntrinsicParameters);
        final var newCam2InternalMatrix = inverseCam2IntrinsicParameters.multiplyAndReturnNew(cam2InternalMatrix);

        camera2.setInternalMatrix(newCam2InternalMatrix);

        final var fundamentalMatrix = new FundamentalMatrix(camera1, camera2);

        // check equality up to scale
        fundamentalMatrix.normalize();
        essentialMatrix.normalize();

        final var fInternal = fundamentalMatrix.getInternalMatrix();
        final var eInternal = essentialMatrix.getInternalMatrix();
        final var firstScale = fInternal.getElementAtIndex(0) / eInternal.getElementAtIndex(0);
        var previousScale = firstScale;
        var currentScale = 0.0;
        for (var i = 0; i < ESSENTIAL_MATRIX_ROWS * ESSENTIAL_MATRIX_COLS; i++) {
            currentScale = fInternal.getElementAtIndex(i) / eInternal.getElementAtIndex(i);
            assertEquals(0.0, previousScale - currentScale, ABSOLUTE_ERROR);
            previousScale = currentScale;
        }
        assertEquals(0.0, currentScale - firstScale, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> essentialMatrix.setFromPairOfCameras(camera1, camera2, -1.0));
    }

    @Test
    void testSetFromPairOfCamerasWhenNumericallyUnstable() throws WrongSizeException {

        final var matrix1 = new Matrix(PinholeCamera.PINHOLE_CAMERA_MATRIX_ROWS,
                PinholeCamera.PINHOLE_CAMERA_MATRIX_COLS);
        final var matrix2 = new Matrix(PinholeCamera.PINHOLE_CAMERA_MATRIX_ROWS,
                PinholeCamera.PINHOLE_CAMERA_MATRIX_COLS);
        Arrays.fill(matrix1.getBuffer(), Double.NaN);
        Arrays.fill(matrix2.getBuffer(), Double.NaN);

        final var camera1 = new PinholeCamera(matrix1);
        final var camera2 = new PinholeCamera(matrix2);

        final var essentialMatrix = new EssentialMatrix();

        // set from a pair of cameras
        assertThrows(InvalidPairOfCamerasException.class, () -> essentialMatrix.setFromPairOfCameras(camera1, camera2));
    }

    @Test
    void testSetFromRotationAndTranslation() throws InvalidRotationAndTranslationException, NotAvailableException,
            NotReadyException, LockedException, DecomposerException, com.irurueta.algebra.NotAvailableException,
            WrongSizeException, InvalidRotationMatrixException {

        var numValid = 0;
        for (var t = 0; t < 2 * TIMES; t++) {

            final var randomizer = new UniformRandomizer();

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var translation = new HomogeneousPoint2D(cameraSeparation, cameraSeparation, cameraSeparation);

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

            final var essentialMatrix = new EssentialMatrix();

            // set from rotation and translation
            essentialMatrix.setFromRotationAndTranslation(rotation, translation);

            // check correctness
            final var internalEssentialMatrix = essentialMatrix.getInternalMatrix();

            final var singularValueDecomposer = new SingularValueDecomposer(internalEssentialMatrix);
            singularValueDecomposer.decompose();

            final var u = singularValueDecomposer.getU();

            final var scaleX = u.getElementAt(0, 2) / translation.getHomX();
            final var scaleY = u.getElementAt(1, 2) / translation.getHomY();
            final var scaleW = u.getElementAt(2, 2) / translation.getHomW();

            assertEquals(0.0, scaleX - scaleY, ABSOLUTE_ERROR);
            assertEquals(0.0, scaleY - scaleW, ABSOLUTE_ERROR);
            assertEquals(0.0, scaleW - scaleX, ABSOLUTE_ERROR);

            final var w = new Matrix(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS);
            w.setElementAt(0, 1, -1.0);
            w.setElementAt(1, 0, 1.0);
            w.setElementAt(2, 2, 1.0);
            final var transW = w.transposeAndReturnNew();
            final var v = singularValueDecomposer.getV();
            final var transV = v.transposeAndReturnNew();

            // First possible rotation
            final var rotation1Matrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));
            final var rotation1 = new MatrixRotation3D(rotation1Matrix);
            // second possible rotation
            final var rotation2Matrix = u.multiplyAndReturnNew(transW.multiplyAndReturnNew(transV));
            final var rotation2 = new MatrixRotation3D(rotation2Matrix);

            var valid = (Math.abs(Math.abs(rotation1.getAlphaEulerAngle())
                    - Math.abs(rotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR
                    || Math.abs(Math.abs(rotation2.getAlphaEulerAngle())
                    - Math.abs(rotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation1.getBetaEulerAngle())
                    - Math.abs(rotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR
                    || Math.abs(Math.abs(rotation2.getBetaEulerAngle())
                    - Math.abs(rotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation1.getGammaEulerAngle())
                    - Math.abs(rotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR
                    || Math.abs(Math.abs(rotation2.getGammaEulerAngle())
                    - Math.abs(rotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR);

            if (valid) {
                numValid++;
            }

            // Force IllegalArgumentException
            assertThrows(IllegalArgumentException.class, () -> essentialMatrix.setFromRotationAndTranslation(rotation,
                    translation, -1.0));

            if (numValid > 2 * TIMES / 4) {
                break;
            }
        }

        assertTrue(numValid > 2 * TIMES / 4);
    }

    @Test
    void testSetFromRotationAndTranslationInvalid() {

        final var randomizer = new UniformRandomizer();

        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var translation = new InhomogeneousPoint2D(Double.NaN, Double.NaN);

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

        final var essentialMatrix = new EssentialMatrix();

        // set from rotation and translation
        assertThrows(InvalidRotationAndTranslationException.class,
                () -> essentialMatrix.setFromRotationAndTranslation(rotation, translation));
    }

    @Test
    void testSetFromFundamentalMatrixAndIntrinsics() throws WrongSizeException, NotReadyException, LockedException,
            DecomposerException, com.irurueta.algebra.NotAvailableException, InvalidFundamentalMatrixException,
            InvalidPairOfIntrinsicParametersException, NotAvailableException {

        final var a = Matrix.createWithUniformRandomValues(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);
        decomposer.decompose();
        final var u = decomposer.getU();
        final var w = decomposer.getW();
        final var v = decomposer.getV();

        final var transV = v.transposeAndReturnNew();

        // set last singular value to zero to enforce rank 2
        w.setElementAt(2, 2, 0.0);
        final var fundamentalInternalMatrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));

        // create intrinsic parameters
        final var randomizer = new UniformRandomizer();
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

        final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
        final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

        final var fundamentalMatrix = new FundamentalMatrix(fundamentalInternalMatrix);

        final var essentialMatrix = new EssentialMatrix();

        // set from fundamental matrix and intrinsics
        essentialMatrix.setFromFundamentalMatrixAndIntrinsics(fundamentalMatrix, intrinsic1, intrinsic2);

        final var kLeftMatrix = intrinsic1.getInternalMatrix();
        final var normK1 = Utils.normF(kLeftMatrix);
        final var k1 = kLeftMatrix.multiplyByScalarAndReturnNew(1.0 / normK1);

        final var kRightMatrix = intrinsic2.getInternalMatrix();
        final var normK2 = Utils.normF(kRightMatrix);
        final var k2 = kRightMatrix.multiplyByScalarAndReturnNew(1.0 / normK2);

        final var transK2 = k2.transposeAndReturnNew();

        final var normFund = Utils.normF(fundamentalInternalMatrix);
        final var tempFundMatrix = fundamentalInternalMatrix.multiplyByScalarAndReturnNew(1.0 / normFund);

        final var estimatedEssentialMatrix = transK2.multiplyAndReturnNew(tempFundMatrix.multiplyAndReturnNew(k1));

        final var normEssential = Utils.normF(estimatedEssentialMatrix);
        estimatedEssentialMatrix.multiplyByScalar(1.0 / normEssential);

        final var eInternal2 = essentialMatrix.getInternalMatrix();
        final var firstScale = eInternal2.getElementAtIndex(0) / estimatedEssentialMatrix.getElementAtIndex(0);
        var previousScale = firstScale;
        var currentScale = 0.0;
        for (var i = 1; i < ESSENTIAL_MATRIX_ROWS * ESSENTIAL_MATRIX_COLS; i++) {
            currentScale = eInternal2.getElementAtIndex(i) / estimatedEssentialMatrix.getElementAtIndex(i);
            assertEquals(0.0, previousScale - currentScale, ABSOLUTE_ERROR);
            previousScale = currentScale;
        }
        assertEquals(0.0, currentScale - firstScale, ABSOLUTE_ERROR);
    }

    @Test
    void testSetFromFundamentalMatrixAndIntrinsicsInvalid() {

        // create intrinsic parameters
        final var randomizer = new UniformRandomizer();
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

        final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
        final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

        final var fundamentalMatrix = new FundamentalMatrix();

        final var essentialMatrix = new EssentialMatrix();

        // set from fundamental matrix and intrinsics
        assertThrows(InvalidPairOfIntrinsicParametersException.class,
                () -> essentialMatrix.setFromFundamentalMatrixAndIntrinsics(fundamentalMatrix, intrinsic1, intrinsic2));
    }

    @Test
    void testToFundamentalMatrix() throws EpipolarException, com.irurueta.geometry.estimators.NotReadyException,
            NotAvailableException, WrongSizeException, NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var a = Matrix.createWithUniformRandomValues(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var decomposer = new SingularValueDecomposer(a);
            decomposer.decompose();
            final var u = decomposer.getU();
            final var w = decomposer.getW();
            final var v = decomposer.getV();

            final var transV = v.transposeAndReturnNew();

            // set last singular value to zero to enforce rank 2
            w.setElementAt(2, 2, 0.0);
            final var fundamentalInternalMatrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));

            // create intrinsic parameters
            final var randomizer = new UniformRandomizer();
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

            final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                    horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
            final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                    horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

            final var fundamentalMatrix1 = new FundamentalMatrix(fundamentalInternalMatrix);

            final var essential = new EssentialMatrix(fundamentalMatrix1, intrinsic1, intrinsic2);

            final var fundamentalMatrix2 = essential.toFundamentalMatrix(intrinsic1, intrinsic2);
            fundamentalMatrix2.normalize();

            final var condition = fundamentalMatrix1.getInternalMatrix().equals(fundamentalMatrix2.getInternalMatrix(),
                    ABSOLUTE_ERROR);
            if (!condition) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testToFundamentalMatrixInvalid() throws EpipolarException, WrongSizeException, NotReadyException,
            LockedException, DecomposerException, com.irurueta.algebra.NotAvailableException {

        final var a = Matrix.createWithUniformRandomValues(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);
        decomposer.decompose();
        final var u = decomposer.getU();
        final var w = decomposer.getW();
        final var v = decomposer.getV();

        final var transV = v.transposeAndReturnNew();

        // set last singular value to zero to enforce rank 2
        w.setElementAt(2, 2, 0.0);
        final var fundamentalInternalMatrix = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));

        final var intrinsic1 = new PinholeCameraIntrinsicParameters(Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                Double.NaN);
        final var intrinsic2 = new PinholeCameraIntrinsicParameters(Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                Double.NaN);

        final var fundamentalMatrix1 = new FundamentalMatrix(fundamentalInternalMatrix);

        final var essential = new EssentialMatrix(fundamentalMatrix1, intrinsic1, intrinsic2);

        assertThrows(EpipolarException.class, () -> essential.toFundamentalMatrix(intrinsic1, intrinsic2));
    }

    @Test
    void testSetFromRotationAndCameraCenter() throws InvalidRotationAndTranslationException,
            InvalidEssentialMatrixException, NotAvailableException, WrongSizeException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
            final var center = new InhomogeneousPoint3D(cameraSeparation, cameraSeparation, cameraSeparation);

            final var essentialMatrix = new EssentialMatrix();

            // set from rotation and camera center
            essentialMatrix.setFromRotationAndCameraCenter(rotation, center);

            // compute possible rotations and translations
            essentialMatrix.computePossibleRotationAndTranslations();

            // check correctness
            final var firstEstimatedTranslation = essentialMatrix.getFirstPossibleTranslation();
            final var secondEstimatedTranslation = essentialMatrix.getSecondPossibleTranslation();
            final var firstEstimatedCameraRotation = (MatrixRotation3D) essentialMatrix.getFirstPossibleRotation();
            final var secondEstimatedCameraRotation = (MatrixRotation3D) essentialMatrix.getSecondPossibleRotation();

            var valid = (Math.abs(Math.abs(rotation.getAlphaEulerAngle())
                    - Math.abs(firstEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR
                    || Math.abs(Math.abs(rotation.getAlphaEulerAngle())
                    - Math.abs(secondEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getBetaEulerAngle())
                    - Math.abs(firstEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR
                    || Math.abs(Math.abs(rotation.getBetaEulerAngle())
                    - Math.abs(secondEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getGammaEulerAngle())
                    - Math.abs(firstEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR
                    || Math.abs(Math.abs(rotation.getGammaEulerAngle())
                    - Math.abs(secondEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR);

            if (valid) {
                numValid++;
            }

            // translation term is equal to t=-R*C, hence we can obtain camera
            // centers as: C = -inv(R)*t = -R'*t
            final var rotationMatrix1 = firstEstimatedCameraRotation.getInternalMatrix();
            final var rotationMatrix2 = secondEstimatedCameraRotation.getInternalMatrix();
            final var transRotationMatrix1 = rotationMatrix1.transposeAndReturnNew();
            final var transRotationMatrix2 = rotationMatrix2.transposeAndReturnNew();
            final var translationMatrix1 = new Matrix(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix1.setElementAt(0, 0, firstEstimatedTranslation.getHomX());
            translationMatrix1.setElementAt(1, 0, firstEstimatedTranslation.getHomY());
            translationMatrix1.setElementAt(2, 0, firstEstimatedTranslation.getHomW());
            final var translationMatrix2 = new Matrix(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix2.setElementAt(0, 0, secondEstimatedTranslation.getHomX());
            translationMatrix2.setElementAt(1, 0, secondEstimatedTranslation.getHomY());
            translationMatrix2.setElementAt(2, 0, secondEstimatedTranslation.getHomW());

            final var centerMatrix1 = transRotationMatrix1.multiplyAndReturnNew(translationMatrix1)
                    .multiplyByScalarAndReturnNew(-1.0);
            final var centerMatrix2 = transRotationMatrix1.multiplyAndReturnNew(translationMatrix2)
                    .multiplyByScalarAndReturnNew(-1.0);
            final var centerMatrix3 = transRotationMatrix2.multiplyAndReturnNew(translationMatrix1)
                    .multiplyByScalarAndReturnNew(-1.0);
            final var centerMatrix4 = transRotationMatrix2.multiplyAndReturnNew(translationMatrix2)
                    .multiplyByScalarAndReturnNew(-1.0);

            var scaleX = centerMatrix1.getElementAt(0, 0) / center.getInhomX();
            var scaleY = centerMatrix1.getElementAt(1, 0) / center.getInhomY();
            var scaleZ = centerMatrix1.getElementAt(2, 0) / center.getInhomZ();

            final var valid1 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR)
                    && (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) && (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix2.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix2.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix2.getElementAt(2, 0) / center.getInhomZ();

            final var valid2 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR)
                    && (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) && (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix3.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix3.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix3.getElementAt(2, 0) / center.getInhomZ();

            final var valid3 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR)
                    && (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) && (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix4.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix4.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix4.getElementAt(2, 0) / center.getInhomZ();

            final var valid4 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR)
                    && (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) && (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            assertTrue(valid1 || valid2 || valid3 || valid4);

            // Force IllegalArgumentException
            assertThrows(IllegalArgumentException.class,
                    () -> essentialMatrix.setFromRotationAndCameraCenter(rotation, center, -1.0));
        }
        assertTrue(numValid > TIMES / 4);
    }

    @Test
    void testSetFromRotationAndCameraCenterInvalid() {
        final var randomizer = new UniformRandomizer();
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
        final var center = new InhomogeneousPoint3D(Double.NaN, Double.NaN, Double.NaN);

        final var essentialMatrix = new EssentialMatrix();

        // set from rotation and camera center
        assertThrows(InvalidRotationAndTranslationException.class,
                () -> essentialMatrix.setFromRotationAndCameraCenter(rotation, center));
    }

    @Test
    void testComputePossibleRotationsAndTranslations() throws InvalidRotationAndTranslationException,
            InvalidEssentialMatrixException, NotAvailableException, InvalidPairOfIntrinsicParametersException,
            WrongSizeException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            // Create rotation and translation
            final var randomizer = new UniformRandomizer();
            final var alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
            final var betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
            final var gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
            final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
            final var cameraSeparation = randomizer.nextDouble(MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
            final var translation = new HomogeneousPoint2D(cameraSeparation, cameraSeparation, cameraSeparation);

            var essentialMatrix = new EssentialMatrix(rotation, translation);

            // test NotAvailableExceptions
            assertFalse(essentialMatrix.arePossibleRotationsAndTranslationsAvailable());
            assertThrows(NotAvailableException.class, essentialMatrix::getFirstPossibleRotation);
            assertThrows(NotAvailableException.class, essentialMatrix::getSecondPossibleRotation);
            assertThrows(NotAvailableException.class, essentialMatrix::getFirstPossibleTranslation);
            assertThrows(NotAvailableException.class, essentialMatrix::getSecondPossibleTranslation);

            // compute possible rotations and translations
            essentialMatrix.computePossibleRotationAndTranslations();

            // check correctness
            var firstEstimatedTranslation = essentialMatrix.getFirstPossibleTranslation();
            var secondEstimatedTranslation = essentialMatrix.getSecondPossibleTranslation();
            var firstEstimatedCameraRotation = (MatrixRotation3D) essentialMatrix.getFirstPossibleRotation();
            var secondEstimatedCameraRotation = (MatrixRotation3D) essentialMatrix.getSecondPossibleRotation();

            assertTrue(translation.equals(firstEstimatedTranslation)
                    && translation.equals(secondEstimatedTranslation));

            var valid = (Math.abs(Math.abs(rotation.getAlphaEulerAngle())
                    - Math.abs(firstEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR
                    || Math.abs(Math.abs(rotation.getAlphaEulerAngle())
                    - Math.abs(secondEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getBetaEulerAngle())
                    - Math.abs(firstEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR
                    || Math.abs(Math.abs(rotation.getBetaEulerAngle())
                    - Math.abs(secondEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getGammaEulerAngle())
                    - Math.abs(firstEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR
                    || Math.abs(Math.abs(rotation.getGammaEulerAngle())
                    - Math.abs(secondEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR);

            if (valid) {
                numValid++;
            }

            // testing again from a pair of cameras
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

            final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                    horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
            final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                    horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

            final var cameraCenter1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final var cameraCenter2 = new InhomogeneousPoint3D(cameraSeparation, cameraSeparation, cameraSeparation);

            final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
            final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

            final var camera1 = new PinholeCamera(intrinsic1, rotation1, cameraCenter1);
            final var camera2 = new PinholeCamera(intrinsic2, rotation2, cameraCenter2);

            // compute their respective fundamental matrix
            final FundamentalMatrix fundamentalMatrix;
            try {
                fundamentalMatrix = new FundamentalMatrix(camera1, camera2);
            } catch (final InvalidPairOfCamerasException e) {
                continue;
            }

            // obtain essential matrix from fundamental matrix and intrinsic
            // parameters of both cameras
            essentialMatrix = new EssentialMatrix(fundamentalMatrix, intrinsic1, intrinsic2);

            // compute rotation and translation
            essentialMatrix.computePossibleRotationAndTranslations();

            // check that rotation is correct
            firstEstimatedTranslation = essentialMatrix.getFirstPossibleTranslation();
            secondEstimatedTranslation = essentialMatrix.getSecondPossibleTranslation();
            firstEstimatedCameraRotation = (MatrixRotation3D) essentialMatrix.getFirstPossibleRotation();
            secondEstimatedCameraRotation = (MatrixRotation3D) essentialMatrix.getSecondPossibleRotation();

            // compute 4 possible camera centers for second camera and check that
            // the translation term is equal to -R*C where R is rotation and C is
            // camera center using inhomogeneous coordinates
            final var rotationMatrix1 = firstEstimatedCameraRotation.getInternalMatrix();
            final var rotationMatrix2 = secondEstimatedCameraRotation.getInternalMatrix();
            final var transRotationMatrix1 = rotationMatrix1.transposeAndReturnNew();
            final var transRotationMatrix2 = rotationMatrix2.transposeAndReturnNew();
            final var translationMatrix1 = new Matrix(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix1.setElementAt(0, 0, firstEstimatedTranslation.getHomX());
            translationMatrix1.setElementAt(1, 0, firstEstimatedTranslation.getHomY());
            translationMatrix1.setElementAt(2, 0, firstEstimatedTranslation.getHomW());
            final var translationMatrix2 = new Matrix(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix2.setElementAt(0, 0, secondEstimatedTranslation.getHomX());
            translationMatrix2.setElementAt(1, 0, secondEstimatedTranslation.getHomY());
            translationMatrix2.setElementAt(2, 0, secondEstimatedTranslation.getHomW());

            final var centerMatrix1 = transRotationMatrix1.multiplyAndReturnNew(translationMatrix1)
                    .multiplyByScalarAndReturnNew(-1.0);
            final var centerMatrix2 = transRotationMatrix1.multiplyAndReturnNew(translationMatrix2)
                    .multiplyByScalarAndReturnNew(-1.0);
            final var centerMatrix3 = transRotationMatrix2.multiplyAndReturnNew(translationMatrix1)
                    .multiplyByScalarAndReturnNew(-1.0);
            final var centerMatrix4 = transRotationMatrix2.multiplyAndReturnNew(translationMatrix2)
                    .multiplyByScalarAndReturnNew(-1.0);

            var scaleX = centerMatrix1.getElementAt(0, 0) / cameraCenter2.getInhomX();
            var scaleY = centerMatrix1.getElementAt(1, 0) / cameraCenter2.getInhomY();
            var scaleZ = centerMatrix1.getElementAt(2, 0) / cameraCenter2.getInhomZ();

            final var valid1 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR)
                    && (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) && (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix2.getElementAt(0, 0) / cameraCenter2.getInhomX();
            scaleY = centerMatrix2.getElementAt(1, 0) / cameraCenter2.getInhomY();
            scaleZ = centerMatrix2.getElementAt(2, 0) / cameraCenter2.getInhomZ();

            final var valid2 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR)
                    && (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) && (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix3.getElementAt(0, 0) / cameraCenter2.getInhomX();
            scaleY = centerMatrix3.getElementAt(1, 0) / cameraCenter2.getInhomY();
            scaleZ = centerMatrix3.getElementAt(2, 0) / cameraCenter2.getInhomZ();

            final var valid3 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR)
                    && (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) && (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix4.getElementAt(0, 0) / cameraCenter2.getInhomX();
            scaleY = centerMatrix4.getElementAt(1, 0) / cameraCenter2.getInhomY();
            scaleZ = centerMatrix4.getElementAt(2, 0) / cameraCenter2.getInhomZ();

            final var valid4 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR)
                    && (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) && (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            assertTrue(valid1 || valid2 || valid3 || valid4);
        }
        assertTrue(numValid > TIMES / 4);

        // Force InvalidEssentialMatrixException
        final var essentialMatrix = new EssentialMatrix();
        assertThrows(InvalidEssentialMatrixException.class, essentialMatrix::computePossibleRotationAndTranslations);
    }

    @Test
    void testIsValidInternalMatrix() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            com.irurueta.algebra.NotAvailableException {

        // testing invalid essential matrix with rank different of 2
        final var internalMatrix1 = Matrix.createWithUniformRandomValues(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var decomposer = new SingularValueDecomposer(internalMatrix1);
        decomposer.decompose();

        var u = decomposer.getU();
        var w = decomposer.getW();
        var v = decomposer.getV();
        var transV = v.transposeAndReturnNew();

        // set all singular values to non-zero to enforce rank 3
        w.setElementAt(0, 0, 1.0);
        w.setElementAt(1, 1, 1.0);
        w.setElementAt(2, 2, 3.0);

        final var internalMatrix2 = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));

        assertFalse(EssentialMatrix.isValidInternalMatrix(internalMatrix2));

        // testing invalid essential matrix with more than 3 columns and rows
        final var internalMatrix3 = Matrix.createWithUniformRandomValues(ESSENTIAL_MATRIX_ROWS + 1,
                ESSENTIAL_MATRIX_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        assertFalse(EssentialMatrix.isValidInternalMatrix(internalMatrix3));

        final var internalMatrix4 = Matrix.createWithUniformRandomValues(ESSENTIAL_MATRIX_ROWS,
                ESSENTIAL_MATRIX_COLS + 1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        assertFalse(EssentialMatrix.isValidInternalMatrix(internalMatrix4));

        // testing invalid essential matrix with different singular values
        final var internalMatrix5 = Matrix.createWithUniformRandomValues(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        decomposer = new SingularValueDecomposer(internalMatrix5);
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

        final var internalMatrix6 = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));

        assertFalse(EssentialMatrix.isValidInternalMatrix(internalMatrix6));
        assertFalse(EssentialMatrix.isValidInternalMatrix(internalMatrix6, 0.2));

        // setting a large enough threshold makes it valid
        assertTrue(EssentialMatrix.isValidInternalMatrix(internalMatrix6, 0.5 + ABSOLUTE_ERROR));

        // testing a valid essential matrix
        final var internalMatrix7 = Matrix.createWithUniformRandomValues(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        decomposer = new SingularValueDecomposer(internalMatrix7);
        decomposer.decompose();

        u = decomposer.getU();
        w = decomposer.getW();
        v = decomposer.getV();
        transV = v.transposeAndReturnNew();

        // set last singular value to zero to enforce rank 2
        w.setElementAt(0, 0, 1.0);
        w.setElementAt(1, 1, 1.0);
        w.setElementAt(2, 2, 0.0);

        final var internalMatrix8 = u.multiplyAndReturnNew(w.multiplyAndReturnNew(transV));

        assertTrue(EssentialMatrix.isValidInternalMatrix(internalMatrix8));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> EssentialMatrix.isValidInternalMatrix(internalMatrix8,
                -1.0));

        // Test for matrix with numerical instabilities
        final var internalMatrix9 = new Matrix(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS);
        Arrays.fill(internalMatrix9.getBuffer(), Double.NaN);

        assertFalse(EssentialMatrix.isValidInternalMatrix(internalMatrix9));
    }

    @Test
    void testSerializeDeserialize() throws InvalidPairOfCamerasException, IOException, ClassNotFoundException,
            InvalidEssentialMatrixException, NotAvailableException {
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

        final var cameraCenter1 = new InhomogeneousPoint3D(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var cameraCenter2 = new InhomogeneousPoint3D(cameraCenter1.getInhomX() + cameraSeparation,
                cameraCenter1.getInhomY() + cameraSeparation, cameraCenter1.getInhomZ() + cameraSeparation);

        final var rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, gammaEuler1);
        final var rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2, gammaEuler2);

        final var intrinsic1 = new PinholeCameraIntrinsicParameters(horizontalFocalLength1, verticalFocalLength1,
                horizontalPrincipalPoint1, verticalPrincipalPoint1, skewness1);
        final var intrinsic2 = new PinholeCameraIntrinsicParameters(horizontalFocalLength2, verticalFocalLength2,
                horizontalPrincipalPoint2, verticalPrincipalPoint2, skewness2);

        final var camera1 = new PinholeCamera(intrinsic1, rotation1, cameraCenter1);
        final var camera2 = new PinholeCamera(intrinsic2, rotation2, cameraCenter2);

        // estimate essential matrix using provided cameras
        final var essentialMatrix1 = new EssentialMatrix(camera1, camera2);
        essentialMatrix1.computePossibleRotationAndTranslations();

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(essentialMatrix1);
        final var essentialMatrix2 = SerializationHelper.<EssentialMatrix>deserialize(bytes);

        // check
        assertEquals(essentialMatrix1.getInternalMatrix(), essentialMatrix2.getInternalMatrix());
        assertEquals(essentialMatrix1.getFirstPossibleRotation(), essentialMatrix2.getFirstPossibleRotation());
        assertEquals(essentialMatrix1.getSecondPossibleRotation(), essentialMatrix2.getSecondPossibleRotation());
        assertEquals(essentialMatrix1.getFirstPossibleTranslation(), essentialMatrix2.getFirstPossibleTranslation());
        assertEquals(essentialMatrix1.getSecondPossibleTranslation(), essentialMatrix2.getSecondPossibleTranslation());
        assertEquals(essentialMatrix1.arePossibleRotationsAndTranslationsAvailable(),
                essentialMatrix2.arePossibleRotationsAndTranslationsAvailable());
    }
}
