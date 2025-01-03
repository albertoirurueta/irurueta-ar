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
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.*;

import java.io.Serializable;

/**
 * The essential matrix defines the relation between two views in a similar way
 * that the fundamental matrix does, but taking into account the intrinsic
 * parameters of the cameras associated to both views. That ways the relation
 * between their extrinsic parameters (rotation and translation) can be precisely
 * obtained.
 */
public class EssentialMatrix extends FundamentalMatrix implements Serializable {

    /**
     * Default threshold to determine that the two non-zero singular values are
     * equal.
     */
    public static final double DEFAULT_SINGULAR_VALUES_THRESHOLD = 1e-8;

    private Rotation3D rotation1;
    private Rotation3D rotation2;

    private Point2D translation1;
    private Point2D translation2;

    private boolean possibleRotationsAndTranslationsAvailable;

    /**
     * Constructor.
     */
    public EssentialMatrix() {
        super();
    }

    /**
     * Constructor.
     *
     * @param internalMatrix          matrix to be set internally.
     * @param singularValuesThreshold threshold to determine that both singular
     *                                values are equal.
     * @throws InvalidEssentialMatrixException if provided matrix is not 3x3,
     *                                         does not have rank 2 or its two non-zero singular values
     *                                         are not equal up to provided threshold.
     * @throws IllegalArgumentException        if provided threshold is negative.
     */
    public EssentialMatrix(final Matrix internalMatrix, final double singularValuesThreshold)
            throws InvalidEssentialMatrixException {
        super();
        setInternalMatrix(internalMatrix, singularValuesThreshold);
    }

    /**
     * Constructor.
     *
     * @param internalMatrix matrix to be set internally.
     * @throws InvalidEssentialMatrixException if provided matrix is not 3x3,
     *                                         does not have rank 2 or its two non-zero singular values
     *                                         are not equal.
     */
    public EssentialMatrix(final Matrix internalMatrix) throws InvalidEssentialMatrixException {
        this(internalMatrix, DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }

    /**
     * Constructor from a pair of cameras.
     *
     * @param leftCamera              camera corresponding to left view.
     * @param rightCamera             camera corresponding to right view.
     * @param singularValuesThreshold threshold to determine that both singular
     *                                values of generated essential matrix are equal.
     * @throws InvalidPairOfCamerasException if provided cameras do not span a
     *                                       valid epipolar geometry (i.e. they are set in a degenerate
     *                                       configuration).
     * @throws IllegalArgumentException      if provided threshold is negative.
     */
    public EssentialMatrix(
            final PinholeCamera leftCamera, final PinholeCamera rightCamera, final double singularValuesThreshold)
            throws InvalidPairOfCamerasException {
        super();
        setFromPairOfCameras(leftCamera, rightCamera, singularValuesThreshold);
    }

    /**
     * Constructor from a pair of cameras.
     *
     * @param leftCamera  camera corresponding to left view.
     * @param rightCamera camera corresponding to right view.
     * @throws InvalidPairOfCamerasException if provided cameras do not span a
     *                                       valid epipolar geometry (i.e. they are set in a degenerate
     *                                       configuration).
     */
    public EssentialMatrix(
            final PinholeCamera leftCamera, final PinholeCamera rightCamera) throws InvalidPairOfCamerasException {
        this(leftCamera, rightCamera, DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }

    /**
     * Constructor from rotation and translation of the image of world origin
     * relative to left view camera, which is assumed to be located at origin
     * of coordinates with no rotation.
     *
     * @param rotation                rotation of right camera relative to left camera.
     * @param translation             translation of the image of world origin on right
     *                                camera relative to left camera.
     * @param singularValuesThreshold threshold to determine that both singular
     *                                values of generated essential matrix are equal.
     * @throws InvalidRotationAndTranslationException if provided rotation and
     *                                                translation yield a degenerate epipolar geometry.
     * @throws IllegalArgumentException               if provided threshold is negative.
     */
    public EssentialMatrix(
            final Rotation3D rotation, final Point2D translation, final double singularValuesThreshold)
            throws InvalidRotationAndTranslationException {
        super();
        setFromRotationAndTranslation(rotation, translation, singularValuesThreshold);
    }

    /**
     * Constructor from rotation and translation of the image of world origin
     * relative to left view camera, which is assumed to be located at origin
     * of coordinates with no rotation.
     *
     * @param rotation    rotation of right camera relative to left camera.
     * @param translation translation of the image of world origin on right
     *                    camera relative to left camera.
     * @throws InvalidRotationAndTranslationException if provided rotation and
     *                                                translation yield a degenerate epipolar geometry.
     */
    public EssentialMatrix(
            final Rotation3D rotation, final Point2D translation) throws InvalidRotationAndTranslationException {
        this(rotation, translation, DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }

    /**
     * Constructor from rotation and translation of the camera center relative
     * to left view camera, which is assumed to be located at origin of
     * coordinates with no rotation.
     *
     * @param rotation                rotation of right camera relative to left camera.
     * @param cameraCenter            camera center of right camera relative to left camera.
     * @param singularValuesThreshold threshold to determine that both singular
     *                                values of generated essential matrix are equal.
     * @throws InvalidRotationAndTranslationException if provided rotation and
     *                                                translation yield a degenerate epipolar geometry.
     * @throws IllegalArgumentException               if provided threshold is negative.
     */
    public EssentialMatrix(
            final Rotation3D rotation, final Point3D cameraCenter, final double singularValuesThreshold)
            throws InvalidRotationAndTranslationException {
        setFromRotationAndCameraCenter(rotation, cameraCenter, singularValuesThreshold);
    }

    /**
     * Constructor from rotation and translation of the camera center relative
     * to left view camera, which is assumed to be located at origin of
     * coordinates with no rotation.
     *
     * @param rotation     rotation of right camera relative to left camera.
     * @param cameraCenter camera center of right camera relative to left camera.
     * @throws InvalidRotationAndTranslationException if provided rotation and
     *                                                translation yield a degenerate epipolar geometry.
     */
    public EssentialMatrix(
            final Rotation3D rotation, final Point3D cameraCenter) throws InvalidRotationAndTranslationException {
        this(rotation, cameraCenter, DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }

    /**
     * Constructor from fundamental matrix and intrinsic camera parameters.
     *
     * @param fundamentalMatrix        a fundamental matrix.
     * @param leftIntrinsicParameters  intrinsic camera parameters of left view.
     * @param rightIntrinsicParameters intrinsic camera parameters of right view.
     * @throws InvalidPairOfIntrinsicParametersException if provided intrinsic
     *                                                   parameters generate an invalid essential matrix.
     */
    public EssentialMatrix(
            final FundamentalMatrix fundamentalMatrix,
            final PinholeCameraIntrinsicParameters leftIntrinsicParameters,
            final PinholeCameraIntrinsicParameters rightIntrinsicParameters)
            throws InvalidPairOfIntrinsicParametersException {
        setFromFundamentalMatrixAndIntrinsics(fundamentalMatrix, leftIntrinsicParameters, rightIntrinsicParameters);
    }

    /**
     * Sets internal matrix associated to this instance.
     * This method makes a copy of provided matrix.
     *
     * @param internalMatrix matrix to be assigned to this instance.
     * @throws InvalidEssentialMatrixException if provided matrix is not 3x3,
     *                                         does not have rank 2 or its two non-zero singular values
     *                                         are not equal.
     */
    @Override
    public final void setInternalMatrix(final Matrix internalMatrix) throws InvalidEssentialMatrixException {
        setInternalMatrix(internalMatrix, DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }

    /**
     * Sets internal matrix associated to this instance.
     * This method makes a copy of provided matrix.
     *
     * @param internalMatrix          matrix to be assigned to this instance.
     * @param singularValuesThreshold threshold to determine that both
     *                                singular values are equal.
     * @throws IllegalArgumentException        if provided threshold is negative.
     * @throws InvalidEssentialMatrixException if provided matrix is not 3x3,
     *                                         does not have rank 2 or its two non-zero singular values
     *                                         are not equal up to provided threshold.
     */
    public final void setInternalMatrix(
            final Matrix internalMatrix, final double singularValuesThreshold) throws InvalidEssentialMatrixException {
        if (!isValidInternalMatrix(internalMatrix, singularValuesThreshold)) {
            throw new InvalidEssentialMatrixException();
        }

        // because provided matrix is valid, we proceed to setting it
        this.internalMatrix = new Matrix(internalMatrix);
        normalized = false;
        leftEpipole = rightEpipole = null;
    }

    /**
     * Returns a boolean indicating whether provided matrix is a valid essential
     * matrix (i.e. has size 3x3, rank 2 and two non-zero and equal singular
     * values).
     *
     * @param internalMatrix matrix to be checked.
     * @return true if provided matrix is a valid essential matrix, false
     * otherwise.
     */
    public static boolean isValidInternalMatrix(final Matrix internalMatrix) {
        return isValidInternalMatrix(internalMatrix, DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }

    /**
     * Returns a boolean indicating whether provided matrix is a valid
     * essential matrix (i.e. has size 3x3, rank 2 and his two non-zero singular
     * values are equal up to provided threshold).
     *
     * @param internalMatrix          matrix to be checked.
     * @param singularValuesThreshold threshold to determine that both singular
     *                                values are equal.
     * @return true if provided matrix is a valid essential matrix, false
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isValidInternalMatrix(final Matrix internalMatrix, final double singularValuesThreshold) {
        if (singularValuesThreshold < 0) {
            throw new IllegalArgumentException();
        }

        if (internalMatrix.getColumns() != FUNDAMENTAL_MATRIX_COLS
                || internalMatrix.getRows() != FUNDAMENTAL_MATRIX_ROWS) {
            return false;
        }

        try {
            final var decomposer = new SingularValueDecomposer(internalMatrix);

            decomposer.decompose();

            final var rankEssential = decomposer.getRank();

            if (rankEssential != FUNDAMENTAL_MATRIX_RANK) {
                return false;
            }

            final var singularValues = decomposer.getSingularValues();

            return (Math.abs(singularValues[0] - singularValues[1]) <= singularValuesThreshold);
        } catch (final AlgebraException e) {
            return false;
        }
    }

    /**
     * Sets essential matrix from provided a pair of cameras.
     *
     * @param leftCamera  camera corresponding to left view.
     * @param rightCamera camera corresponding to right view.
     * @throws InvalidPairOfCamerasException if provided cameras do not span a
     *                                       valid epipolar geometry (i.e. they are set in a degenerate
     *                                       configuration).
     */
    @Override
    public void setFromPairOfCameras(final PinholeCamera leftCamera, final PinholeCamera rightCamera)
            throws InvalidPairOfCamerasException {
        setFromPairOfCameras(leftCamera, rightCamera, DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }

    /**
     * Sets essential matrix from provided a pair of cameras.
     *
     * @param leftCamera              camera corresponding to left view.
     * @param rightCamera             camera corresponding to right view.
     * @param singularValuesThreshold threshold to determine that both singular
     *                                values of generated essential matrix are equal.
     * @throws InvalidPairOfCamerasException if provided cameras do not span a
     *                                       valid epipolar geometry (i.e. they are set in a degenerate
     *                                       configuration).
     * @throws IllegalArgumentException      if provided threshold is negative.
     */
    public final void setFromPairOfCameras(
            final PinholeCamera leftCamera, final PinholeCamera rightCamera, final double singularValuesThreshold)
            throws InvalidPairOfCamerasException {

        if (singularValuesThreshold < 0) {
            throw new IllegalArgumentException();
        }

        try {
            // normalize cameras to increase accuracy of results and fix their signs
            // if needed
            leftCamera.normalize();
            rightCamera.normalize();

            if (!leftCamera.isCameraSignFixed()) {
                leftCamera.fixCameraSign();
            }
            if (!rightCamera.isCameraSignFixed()) {
                rightCamera.fixCameraSign();
            }

            // Obtain intrinsic parameters of cameras to obtain normalized pinhole
            // cameras where intrinsic parameters have been removed P1' = inv(K) * P1
            if (!leftCamera.areIntrinsicParametersAvailable()) {
                leftCamera.decompose(true, false);
            }
            if (!rightCamera.areIntrinsicParametersAvailable()) {
                rightCamera.decompose(true, false);
            }

            final var leftIntrinsics = leftCamera.getIntrinsicParameters();
            final var rightIntrinsics = rightCamera.getIntrinsicParameters();

            final var leftIntrinsicsMatrix = leftIntrinsics.getInternalMatrix();
            final var rightIntrinsicsMatrix = rightIntrinsics.getInternalMatrix();

            // get left and right internal matrices of cameras
            final var leftCameraInternalMatrix = leftCamera.getInternalMatrix();
            final var rightCameraInternalMatrix = rightCamera.getInternalMatrix();

            // normalize internal camera matrices using inverse intrinsic matrices
            final var invLeftIntrinsicsMatrix = Utils.inverse(leftIntrinsicsMatrix);
            final var invRightIntrinsicsMatrix = Utils.inverse(rightIntrinsicsMatrix);

            // normalize cameras
            // P1' = inv(K1) * P1
            invLeftIntrinsicsMatrix.multiply(leftCameraInternalMatrix);

            // P2' = inv(K2) * P2
            invRightIntrinsicsMatrix.multiply(rightCameraInternalMatrix);

            // instantiate normalized left camera to project right camera center
            // and obtain left eipole
            final var normLeftCamera = new PinholeCamera(invLeftIntrinsicsMatrix);

            // instantiate normalized right camera to decompose it and obtain its
            // center
            final var normRightCamera = new PinholeCamera(invRightIntrinsicsMatrix);

            normRightCamera.decompose(false, true);

            final var rightCameraCenter = normRightCamera.getCameraCenter();
            final var normLeftEpipole = normLeftCamera.project(rightCameraCenter);
            // to increase accuracy
            normLeftEpipole.normalize();

            // compute skew matrix of left epipole
            final var skewNormLeftEpipoleMatrix = Utils.skewMatrix(new double[]{
                    normLeftEpipole.getHomX(), normLeftEpipole.getHomY(), normLeftEpipole.getHomW()});

            // compute transposed of internal normalized left pinhole camera
            final var transNormLeftCameraMatrix = invLeftIntrinsicsMatrix.transposeAndReturnNew();

            // compute transposed of internal normalized right pinhole camera
            final var transNormRightCameraMatrix = invRightIntrinsicsMatrix.transposeAndReturnNew();

            // compute pseudo-inverse of transposed normalized right pinhole camera
            final var pseudoTransNormRightCameraMatrix = Utils.pseudoInverse(transNormRightCameraMatrix);

            // obtain essential matrix as: inv(P2norm') * P1norm' * skew(e1)
            transNormLeftCameraMatrix.multiply(skewNormLeftEpipoleMatrix);
            pseudoTransNormRightCameraMatrix.multiply(transNormLeftCameraMatrix);

            setInternalMatrix(pseudoTransNormRightCameraMatrix, singularValuesThreshold);
        } catch (final GeometryException | AlgebraException e) {
            throw new InvalidPairOfCamerasException(e);
        }
    }

    /**
     * Sets essential matrix from provided rotation and translation of the image
     * of world origin relative to left view camera, which is assumed to be
     * located at origin of coordinates with no rotation.
     *
     * @param rotation    rotation of right camera relative to left camera.
     * @param translation translation of the image of world origin on right
     *                    camera relative to left camera.
     * @throws InvalidRotationAndTranslationException if provided rotation and
     *                                                translation yield a degenerate epipolar geometry.
     */
    public void setFromRotationAndTranslation(
            final Rotation3D rotation, final Point2D translation) throws InvalidRotationAndTranslationException {
        setFromRotationAndTranslation(rotation, translation, DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }

    /**
     * Sets essential matrix from provided rotation and translation of the image
     * of world origin relative to left view camera, which is assumed to be
     * located at origin of coordinates with no rotation.
     *
     * @param rotation                rotation of right camera relative to left camera.
     * @param translation             translation of the image of world origin on right
     *                                camera relative to left camera.
     * @param singularValuesThreshold threshold to determine that both singular
     *                                values of generated essential matrix are equal.
     * @throws InvalidRotationAndTranslationException if provided rotation and
     *                                                translation yield a degenerate epipolar geometry.
     * @throws IllegalArgumentException               if provided threshold is negative.
     */
    public final void setFromRotationAndTranslation(
            final Rotation3D rotation, final Point2D translation, final double singularValuesThreshold)
            throws InvalidRotationAndTranslationException {

        if (singularValuesThreshold < 0) {
            throw new IllegalArgumentException();
        }

        try {
            // to increase accuracy
            translation.normalize();
            final var translationArray = new double[]{
                    translation.getHomX(), translation.getHomY(), translation.getHomW()
            };

            final var skewTranslationMatrix = Utils.skewMatrix(translationArray);

            final var rotationMatrix = rotation.asInhomogeneousMatrix();

            // obtain essential matrix as: skew(translation) * rotation
            skewTranslationMatrix.multiply(rotationMatrix);

            setInternalMatrix(skewTranslationMatrix, singularValuesThreshold);
        } catch (final AlgebraException | InvalidEssentialMatrixException e) {
            throw new InvalidRotationAndTranslationException(e);
        }
    }

    /**
     * Sets essential matrix from provided rotation and translation of the
     * camera center relative to left view camera, which is assumed to be
     * located at origin of coordinates with no rotation.
     *
     * @param rotation     rotation of right camera relative to left camera.
     * @param cameraCenter camera center of right camera relative to left camera.
     * @throws InvalidRotationAndTranslationException if provided rotation and
     *                                                camera center yield a degenerate epipolar geometry.
     */
    public void setFromRotationAndCameraCenter(
            final Rotation3D rotation, final Point3D cameraCenter) throws InvalidRotationAndTranslationException {
        setFromRotationAndCameraCenter(rotation, cameraCenter, DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }

    /**
     * Sets essential matrix from provided rotation and translation of the
     * camera center relative to left view camera, which is assumed to be
     * located at origin of coordinates with no rotation.
     *
     * @param rotation                rotation of right camera relative to left camera.
     * @param cameraCenter            camera center of right camera relative to left camera.
     * @param singularValuesThreshold threshold to determine that both singular
     *                                values of generated essential matrix are equal.
     * @throws InvalidRotationAndTranslationException if provided rotation and
     *                                                camera center yield a degenerate epipolar geometry.
     * @throws IllegalArgumentException               if provided threshold is negative.
     */
    public final void setFromRotationAndCameraCenter(
            final Rotation3D rotation, final Point3D cameraCenter, final double singularValuesThreshold)
            throws InvalidRotationAndTranslationException {

        if (singularValuesThreshold < 0) {
            throw new IllegalArgumentException();
        }

        try {
            var rotationMatrix = rotation.asInhomogeneousMatrix();

            // to increase accuracy
            cameraCenter.normalize();
            final var inhomCenterMatrix = new Matrix(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 1);
            inhomCenterMatrix.setElementAtIndex(0, cameraCenter.getInhomX());
            inhomCenterMatrix.setElementAtIndex(1, cameraCenter.getInhomY());
            inhomCenterMatrix.setElementAtIndex(2, cameraCenter.getInhomZ());

            // translationMatrix = -rotationMatrix * inhomCenterMatrix
            rotationMatrix.multiplyByScalar(-1.0);
            rotationMatrix.multiply(inhomCenterMatrix);
            final var translationMatrix = rotationMatrix;

            // essentialMatrix = skew(translationMatrix) * rotationMatrix
            final var skewTranslationMatrix = Utils.skewMatrix(translationMatrix);
            rotationMatrix = rotation.asInhomogeneousMatrix();
            skewTranslationMatrix.multiply(rotationMatrix);

            setInternalMatrix(skewTranslationMatrix, singularValuesThreshold);

        } catch (final AlgebraException | InvalidEssentialMatrixException e) {
            throw new InvalidRotationAndTranslationException(e);
        }
    }

    /**
     * Sets essential matrix from provided fundamental matrix and intrinsic
     * camera parameters.
     *
     * @param fundamentalMatrix        a fundamental matrix.
     * @param leftIntrinsicParameters  intrinsic camera parameters of left view.
     * @param rightIntrinsicParameters intrinsic camera parameters of right view.
     * @throws InvalidPairOfIntrinsicParametersException if provided intrinsic
     *                                                   parameters generate an invalid essential matrix.
     */
    public final void setFromFundamentalMatrixAndIntrinsics(
            final FundamentalMatrix fundamentalMatrix, final PinholeCameraIntrinsicParameters leftIntrinsicParameters,
            final PinholeCameraIntrinsicParameters rightIntrinsicParameters)
            throws InvalidPairOfIntrinsicParametersException {

        try {
            final var k1 = leftIntrinsicParameters.getInternalMatrix();
            final var normK1 = Utils.normF(k1);
            k1.multiplyByScalar(1.0 / normK1);

            final var k2 = rightIntrinsicParameters.getInternalMatrix();
            final var normK2 = Utils.normF(k2);
            k2.multiplyByScalar(1.0 / normK2);

            // to increase accuracy
            fundamentalMatrix.normalize();
            final var fundMatrix = fundamentalMatrix.getInternalMatrix();

            k2.transpose();

            // E = K2' * F * K1
            fundMatrix.multiply(k1);
            k2.multiply(fundMatrix);

            final var normEssential = Utils.normF(k2);
            k2.multiplyByScalar(1.0 / normEssential);

            internalMatrix = k2;
            normalized = false;
            leftEpipole = rightEpipole = null;
        } catch (final AlgebraException | GeometryException e) {
            throw new InvalidPairOfIntrinsicParametersException(e);
        }
    }

    /**
     * Converts this essential matrix into a fundamental matrix by applying
     * provided intrinsic parameters on left and right views.
     * The essential matrix only contains information about rotation and
     * translation relating two views, while fundamental matrix also contains
     * information about the intrinsic parameters in both views.
     * NOTE: although essential matrix is a subclass of fundamental matrix, it
     * does not behave like a fundamental matrix.
     *
     * @param leftIntrinsicParameters  intrinsic parameters in left view.
     * @param rightIntrinsicParameters intrinsic parameters in right view.
     * @return a fundamental matrix.
     * @throws EpipolarException if something fails.
     */
    public FundamentalMatrix toFundamentalMatrix(
            final PinholeCameraIntrinsicParameters leftIntrinsicParameters,
            final PinholeCameraIntrinsicParameters rightIntrinsicParameters) throws EpipolarException {
        try {
            normalize();

            final var essentialMatrix = getInternalMatrix();

            final var k1 = leftIntrinsicParameters.getInternalMatrix();
            final var invK1 = Utils.inverse(k1);
            final var normInvK1 = Utils.normF(invK1);
            invK1.multiplyByScalar(1.0 / normInvK1);

            final var k2 = rightIntrinsicParameters.getInternalMatrix();
            final var invK2 = Utils.inverse(k2);
            final var normInvK2 = Utils.normF(invK2);
            invK2.multiplyByScalar(1.0 / normInvK2);
            invK2.transpose();

            essentialMatrix.multiply(invK1);
            invK2.multiply(essentialMatrix);

            return new FundamentalMatrix(invK2);

        } catch (final AlgebraException | GeometryException e) {
            throw new EpipolarException(e);
        }
    }

    /**
     * Computes all possible camera rotations and translations that can generate
     * this essential matrix.
     *
     * @throws InvalidEssentialMatrixException if essential matrix contains
     *                                         numerically unstable values.
     */
    public void computePossibleRotationAndTranslations() throws InvalidEssentialMatrixException {
        try {
            final var decomposer = new SingularValueDecomposer(internalMatrix);

            decomposer.decompose();

            final var u = decomposer.getU();
            final var v = decomposer.getV();

            v.transpose();

            translation1 = new HomogeneousPoint2D(u.getElementAt(0, 2),
                    u.getElementAt(1, 2), u.getElementAt(2, 2));
            translation2 = new HomogeneousPoint2D(-u.getElementAt(0, 2),
                    -u.getElementAt(1, 2), -u.getElementAt(2, 2));

            // W is a skew-symmetric matrix that can be used to obtain two possible
            // rotations
            final var w = new Matrix(FUNDAMENTAL_MATRIX_ROWS, FUNDAMENTAL_MATRIX_COLS);
            w.setElementAt(0, 1, -1.0);
            w.setElementAt(1, 0, 1.0);
            w.setElementAt(2, 2, 1.0);

            final var transW = w.transposeAndReturnNew();

            // R1 = U * W * V'
            w.multiply(v);
            final var rotationMatrix1 = u.multiplyAndReturnNew(w);

            // R2 = U * W' * V'
            transW.multiply(v);
            final var rotationMatrix2 = u.multiplyAndReturnNew(transW);

            rotation1 = new MatrixRotation3D(rotationMatrix1);
            rotation2 = new MatrixRotation3D(rotationMatrix2);

            possibleRotationsAndTranslationsAvailable = true;
        } catch (final AlgebraException | InvalidRotationMatrixException e) {
            throw new InvalidEssentialMatrixException(e);
        }
    }

    /**
     * Indicates whether possible camera rotations and translations that can
     * generate this essential matrix have already been computed or not.
     *
     * @return true if possible camera rotations and translations have been
     * computed, false otherwise.
     */
    public boolean arePossibleRotationsAndTranslationsAvailable() {
        return possibleRotationsAndTranslationsAvailable;
    }

    /**
     * Gets first possible rotation that can generate this essential matrix.
     *
     * @return first possible rotation.
     * @throws NotAvailableException if possible rotation has not yet been
     *                               computed.
     */
    public Rotation3D getFirstPossibleRotation() throws NotAvailableException {
        if (!arePossibleRotationsAndTranslationsAvailable()) {
            throw new NotAvailableException();
        }

        return rotation1;
    }

    /**
     * Gets second possible rotation that can generate this essential matrix.
     *
     * @return second possible rotation.
     * @throws NotAvailableException if possible rotation has not yet been
     *                               computed.
     */
    public Rotation3D getSecondPossibleRotation() throws NotAvailableException {
        if (!arePossibleRotationsAndTranslationsAvailable()) {
            throw new NotAvailableException();
        }

        return rotation2;
    }

    /**
     * Gets first possible translation that can generate this essential matrix.
     *
     * @return first possible translation.
     * @throws NotAvailableException if possible translation has not yet been
     *                               computed.
     */
    public Point2D getFirstPossibleTranslation() throws NotAvailableException {
        if (!arePossibleRotationsAndTranslationsAvailable()) {
            throw new NotAvailableException();
        }

        return translation1;
    }

    /**
     * Gets second possible translation that can generate this essential matrix.
     *
     * @return second possible translation.
     * @throws NotAvailableException if possible translation has not yet been
     *                               computed.
     */
    public Point2D getSecondPossibleTranslation() throws NotAvailableException {
        if (!arePossibleRotationsAndTranslationsAvailable()) {
            throw new NotAvailableException();
        }

        return translation2;
    }
}
