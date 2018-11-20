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
 * between their extrinsic parameters (rotation and translation) can be prcisely
 * obtained.
 */
@SuppressWarnings("WeakerAccess")
public class EssentialMatrix extends FundamentalMatrix implements Serializable {
    
    /**
     * Default threshold to determine that the two non-zero singular values are
     * equal.
     */
    public static final double DEFAULT_SINGULAR_VALUES_THRESHOLD = 1e-8;
    
    private Rotation3D mRotation1;
    private Rotation3D mRotation2;
    
    private Point2D mTranslation1;
    private Point2D mTranslation2;
    
    private boolean mPossibleRotationsAndTranslationsAvailable;
    
    /**
     * Constructor.
     */
    public EssentialMatrix() {
        super();
    }
    
    /**
     * Constructor.
     * @param internalMatrix matrix to be set internally.
     * @param singularValuesThreshold threshold to determine that both singular
     * values are equal.
     * @throws InvalidEssentialMatrixException if provided matrix is not 3x3,
     * does not have rank 2 or its two non-zero singular values are not equal up
     * to provided threshold.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public EssentialMatrix(Matrix internalMatrix, 
            double singularValuesThreshold) 
            throws InvalidEssentialMatrixException, IllegalArgumentException {
        super();
        setInternalMatrix(internalMatrix, singularValuesThreshold);
    }
    
    /**
     * Constructor.
     * @param internalMatrix matrix to be set internally.
     * @throws InvalidEssentialMatrixException  if provided matrix is not 3x3,
     * does not have rank 2 or its two non-zero singular values are not equal.
     */
    public EssentialMatrix(Matrix internalMatrix) 
            throws InvalidEssentialMatrixException {
        this(internalMatrix, DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }
    
    /**
     * Constructor from pair of cameras.
     * @param leftCamera camera corresponding to left view.
     * @param rightCamera camera corresponding to right view.
     * @param singularValuesThreshold threshold to determine that both singular
     * values of generated essential matrix are equal.
     * @throws InvalidPairOfCamerasException if provided cameras do not span a 
     * valid epipolar geometry (i.e. they are set in a degenerate configuration).
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public EssentialMatrix(PinholeCamera leftCamera, PinholeCamera rightCamera,
            double singularValuesThreshold) 
            throws InvalidPairOfCamerasException, IllegalArgumentException {
        super();
        setFromPairOfCameras(leftCamera, rightCamera, singularValuesThreshold);
    }
    
    /**
     * Constructor from pair of cameras.
     * @param leftCamera camera corresponding to left view.
     * @param rightCamera camera corresponding to right view.
     * @throws InvalidPairOfCamerasException if provided cameras do not span a
     * valid epipolar geometry (i.e. they are set in a degenerate configuration).
     */
    public EssentialMatrix(PinholeCamera leftCamera, PinholeCamera rightCamera)
            throws InvalidPairOfCamerasException {
        this(leftCamera, rightCamera, DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }
    
    /**
     * Constructor from rotation and translation of the image of world origin
     * relative to left view camera, which is assumed to be located at origin
     * of coordinates with no rotation.
     * @param rotation rotation of right camera relative to left camera.
     * @param translation translation of the image of world origin on right
     * camera relative to left camera.
     * @param singularValuesThreshold threshold to determine that both singular
     * values of generated essential matrix are equal.
     * @throws InvalidRotationAndTranslationException if provided rotation and
     * translation yield a degenerate epipolar geometry.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public EssentialMatrix(Rotation3D rotation, Point2D translation, 
            double singularValuesThreshold) 
            throws InvalidRotationAndTranslationException, 
            IllegalArgumentException {
        super();
        setFromRotationAndTranslation(rotation, translation, 
                singularValuesThreshold);
    }
    
    /**
     * Constructor from rotation and translation of the image of world origin
     * relative to left view camera, which is assumed to be located at origin
     * of coordinates with no rotation.
     * @param rotation rotation of right camera relative to left camera.
     * @param translation translation of the image of world origin on right
     * camera relative to left camera.
     * @throws InvalidRotationAndTranslationException if provided rotation and
     * translation yield a degenerate epipolar geometry.
     */
    public EssentialMatrix(Rotation3D rotation, Point2D translation)
            throws InvalidRotationAndTranslationException {
        this(rotation, translation, DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }
    
    /**
     * Constructor from rotation and translation of the camera center relative
     * to left view camera, which is assumed to be located at origin of 
     * coordinates with no rotation.
     * @param rotation rotation of right camera relative to left camera.
     * @param cameraCenter camera center of right camera relative to left camera.
     * @param singularValuesThreshold threshold to determine that both singular
     * values of generated essential matrix are equal.
     * @throws InvalidRotationAndTranslationException if provided rotation and
     * translation yield a degenerate epipolar geometry.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public EssentialMatrix(Rotation3D rotation, Point3D cameraCenter, 
            double singularValuesThreshold)
            throws InvalidRotationAndTranslationException, 
            IllegalArgumentException {
        setFromRotationAndCameraCenter(rotation, cameraCenter, 
                singularValuesThreshold);
    }
    
    /**
     * Constructor from rotation and translation of the camera center relative
     * to left view camera, which is assumed to be located at origin of 
     * coordinates with no rotation.
     * @param rotation rotation of right camera relative to left camera.
     * @param cameraCenter camera center of right camera relative to left camera.
     * @throws InvalidRotationAndTranslationException if provided rotation and
     * translation yield a degenerate epipolar geometry.
     */    
    public EssentialMatrix(Rotation3D rotation, Point3D cameraCenter)
            throws InvalidRotationAndTranslationException {
        this(rotation, cameraCenter, DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }
    
    /**
     * Constructor from fundamental matrix and intrinsic camera parameters.
     * @param fundamentalMatrix a fundamental matrix.
     * @param leftIntrinsicParameters intrinsic camera parameters of left view.
     * @param rightIntrinsicParameters intrinsic camera parameters of right view.
     * @throws InvalidPairOfIntrinsicParametersException  if provided intrinsic
     * parameters generate an invalid essential matrix.
     */
    public EssentialMatrix(FundamentalMatrix fundamentalMatrix, 
            PinholeCameraIntrinsicParameters leftIntrinsicParameters,
            PinholeCameraIntrinsicParameters rightIntrinsicParameters)
            throws InvalidPairOfIntrinsicParametersException {
        setFromFundamentalMatrixAndIntrinsics(fundamentalMatrix, 
                leftIntrinsicParameters, rightIntrinsicParameters);
    }
            
    /**
     * Sets internal matrix associated to this instance.
     * This method makes a copy of provided matrix.
     * @param internalMatrix matrix to be assigned to this instance.
     * @throws InvalidEssentialMatrixException if provided matrix is not 3x3,
     * does not have rank 2 or its two non-zero singular values are not equal.
     */
    @Override
    public final void setInternalMatrix(Matrix internalMatrix) 
            throws InvalidEssentialMatrixException {
        setInternalMatrix(internalMatrix, DEFAULT_SINGULAR_VALUES_THRESHOLD);
    } 
    
    /**
     * Sets internal matrix associated to this instance.
     * This method makes a copy of provided matrix.
     * @param internalMatrix matrix to be assigned to this instance.
     * @param singularValuesThreshold threshold to determine that both
     * singular values are equal.
     * @throws IllegalArgumentException if provided threshold is negative.
     * @throws InvalidEssentialMatrixException if provided matrix is not 3x3,
     * does not have rank 2 or its two non-zero singular values are not equal up
     * to provided threshold.
     */
    public final void setInternalMatrix(Matrix internalMatrix, 
            double singularValuesThreshold) throws IllegalArgumentException,
            InvalidEssentialMatrixException {
        if (!isValidInternalMatrix(internalMatrix, singularValuesThreshold)) {
            throw new InvalidEssentialMatrixException();
        }
        
        //because provided matrix is valid, we proceed to setting it
        
        mInternalMatrix = internalMatrix.clone();
        mNormalized = false;
        mLeftEpipole = mRightEpipole = null;        
    }
    
    /**
     * Returns a boolean indicating whether provided matrix is a valid essential
     * matrix (i.e. has size 3x3, rank 2 and two non-zero and equal singular
     * values).
     * @param internalMatrix matrix to be checked.
     * @return true if provided matrix is a valid essential matrix, false
     * otherwise.
     */
    public static boolean isValidInternalMatrix(Matrix internalMatrix) {
        return isValidInternalMatrix(internalMatrix, 
                DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }
    
    /**
     * Returns a boolean indicating whether provided matrix is a valid 
     * essential matrix (i.e. has size 3x3, rank 2 and his two non-zero singular
     * values are equal up to provided threshold).
     * @param internalMatrix matrix to be checked.
     * @param singularValuesThreshold threshold to determine that both singular
     * values are equal.
     * @return true if provided matrix is a valid essential matrix, false 
     * otherwise.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public static boolean isValidInternalMatrix(Matrix internalMatrix, 
            double singularValuesThreshold) throws IllegalArgumentException {
        if (singularValuesThreshold < 0) {
            throw new IllegalArgumentException();
        }
        
        if (internalMatrix.getColumns() != FUNDAMENTAL_MATRIX_COLS ||
                internalMatrix.getRows() != FUNDAMENTAL_MATRIX_ROWS) {
            return false;
        }
        
        try {
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    internalMatrix);

            decomposer.decompose();

            double rankEssential = decomposer.getRank();

            if (rankEssential != FUNDAMENTAL_MATRIX_RANK) {
                return false;
            }

            double[] singularValues = decomposer.getSingularValues();

            return (Math.abs(singularValues[0] - singularValues[1]) <= singularValuesThreshold);
        } catch (AlgebraException e) {
            return false;
        }
    }    
    
    /**
     * Sets essential matrix from provided pair of cameras.
     * @param leftCamera camera corresponding to left view.
     * @param rightCamera camera corresponding to right view.
     * @throws InvalidPairOfCamerasException if provided cameras do not span a 
     * valid epipolar geometry (i.e. they are set in a degenerate 
     * configuration).
     */
    @Override
    public void setFromPairOfCameras(PinholeCamera leftCamera,
            PinholeCamera rightCamera) throws InvalidPairOfCamerasException {
        setFromPairOfCameras(leftCamera, rightCamera, 
                DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }   
    
    /**
     * Sets essential matrix from provided pair of cameras.
     * @param leftCamera camera corresponding to left view.
     * @param rightCamera camera corresponding to right view.
     * @param singularValuesThreshold threshold to determine that both singular
     * values of generated essential matrix are equal.
     * @throws InvalidPairOfCamerasException if provided cameras do not span a
     * valid epipolar geometry (i.e. they are set in a degenerate configuration).
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public final void setFromPairOfCameras(PinholeCamera leftCamera,
            PinholeCamera rightCamera, double singularValuesThreshold)
            throws InvalidPairOfCamerasException, IllegalArgumentException {
        
        if (singularValuesThreshold < 0) {
            throw new IllegalArgumentException();
        }
        
        try {
            //normalize cameras to increase accuracy of results and fix their signs
            //if needed
            leftCamera.normalize();
            rightCamera.normalize();

            if (!leftCamera.isCameraSignFixed()) {
                leftCamera.fixCameraSign();
            }
            if (!rightCamera.isCameraSignFixed()) {
                rightCamera.fixCameraSign();
            }

            //Obtain intrinsic parameters of cameras to obtain normalized pinhole
            //cameras where intrinsic parameters have been removed P1' = inv(K) * P1
            if (!leftCamera.areIntrinsicParametersAvailable()) {
                leftCamera.decompose(true, false);
            }
            if (!rightCamera.areIntrinsicParametersAvailable()) {
                rightCamera.decompose(true, false);
            }

            PinholeCameraIntrinsicParameters leftIntrinsics =
                    leftCamera.getIntrinsicParameters();
            PinholeCameraIntrinsicParameters rightIntrinsics =
                    rightCamera.getIntrinsicParameters();

            Matrix leftIntrinsicsMatrix = leftIntrinsics.getInternalMatrix();
            Matrix rightIntrinsicsMatrix = rightIntrinsics.getInternalMatrix();

            //get left and right internal matrices of cameras
            Matrix leftCameraInternalMatrix = leftCamera.getInternalMatrix();
            Matrix rightCameraInternalMatrix = rightCamera.getInternalMatrix();

            //normalize internal camera matrices using inverse intrinsic matrices
            Matrix invLeftIntrinsicsMatrix = Utils.inverse(
                    leftIntrinsicsMatrix);
            Matrix invRightIntrinsicsMatrix = Utils.inverse(
                    rightIntrinsicsMatrix);

            //normalize cameras
            //P1' = inv(K1) * P1
            invLeftIntrinsicsMatrix.multiply(leftCameraInternalMatrix);
            //noinspection all
            Matrix normLeftCameraMatrix = invLeftIntrinsicsMatrix;

            //P2' = inv(K2) * P2
            invRightIntrinsicsMatrix.multiply(rightCameraInternalMatrix);
            //noinspection all
            Matrix normRightCameraMatrix = invRightIntrinsicsMatrix;

            //instantiate normalized left camera to project right camera center
            //and obtain left eipole
            PinholeCamera normLeftCamera = new PinholeCamera(normLeftCameraMatrix);

            //instantiate normalized right camera to decompose it and obtain its
            //center
            PinholeCamera normRightCamera = new PinholeCamera(
                    normRightCameraMatrix);

            normRightCamera.decompose(false, true);

            Point3D rightCameraCenter = normRightCamera.getCameraCenter();
            Point2D normLeftEpipole = normLeftCamera.project(rightCameraCenter);
            normLeftEpipole.normalize(); //to increase accuracy

            //compute skew matrix of left epipole
            Matrix skewNormLeftEpipoleMatrix = Utils.skewMatrix(new double[]{
                normLeftEpipole.getHomX(), normLeftEpipole.getHomY(),
                normLeftEpipole.getHomW() });

            //compute transposed of internal normalized left pinhole camera
            Matrix transNormLeftCameraMatrix = 
                    normLeftCameraMatrix.transposeAndReturnNew();

            //compute transposed of internal normalized right pinhole camera
            Matrix transNormRightCameraMatrix =
                    normRightCameraMatrix.transposeAndReturnNew();

            //compute pseudo-inverse of transposed normalized right pinhole camera
            Matrix pseudoTransNormRightCameraMatrix = Utils.pseudoInverse(
                    transNormRightCameraMatrix);

            //obtain essential matrix as: inv(P2norm') * P1norm' * skew(e1)
            transNormLeftCameraMatrix.multiply(skewNormLeftEpipoleMatrix);
            pseudoTransNormRightCameraMatrix.multiply(transNormLeftCameraMatrix);
            //noinspection all
            Matrix essentialMatrix = pseudoTransNormRightCameraMatrix;

            setInternalMatrix(essentialMatrix, singularValuesThreshold);
        } catch (GeometryException | AlgebraException  e) {
            throw new InvalidPairOfCamerasException(e);
        }
    }
    
    /**
     * Sets essential matrix from provided rotation and translation of the image
     * of world origin relative to left view camera, which is assumed to be 
     * located at origin of coordinates with no rotation.
     * @param rotation rotation of right camera relative to left camera.
     * @param translation translation of the image of world origin on right
     * camera relative to left camera.
     * @throws InvalidRotationAndTranslationException if provided rotation and
     * translation yield a degenerate epipolar geometry.
     */
    public void setFromRotationAndTranslation(Rotation3D rotation, 
            Point2D translation) throws InvalidRotationAndTranslationException {
        setFromRotationAndTranslation(rotation, translation, 
                DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }
    
    /**
     * Sets essential matrix from provided rotation and translation of the image 
     * of world origin relative to left view camera, which is assumed to be
     * located at origin of coordinates with no rotation.
     * @param rotation rotation of right camera relative to left camera.
     * @param translation translation of the image of world origin on right
     * camera relative to left camera.
     * @param singularValuesThreshold threshold to determine that both singular
     * values of generated essential matrix are equal.
     * @throws InvalidRotationAndTranslationException if provided rotation and
     * translation yield a degenerate epipolar geometry.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public final void setFromRotationAndTranslation(Rotation3D rotation,
            Point2D translation, double singularValuesThreshold)
            throws InvalidRotationAndTranslationException, 
            IllegalArgumentException {
        
        if (singularValuesThreshold < 0) {
            throw new IllegalArgumentException();
        }
        
        try {
            translation.normalize(); //to increase accuracy
            double[] translationArray = new double[] {
                translation.getHomX(), translation.getHomY(), translation.getHomW()
            };

            Matrix skewTranslationMatrix = Utils.skewMatrix(translationArray);

            Matrix rotationMatrix = rotation.asInhomogeneousMatrix();

            //obtain essential matrix as: skew(translation) * rotation
            skewTranslationMatrix.multiply(rotationMatrix);

            setInternalMatrix(skewTranslationMatrix, singularValuesThreshold);
        } catch (AlgebraException | InvalidEssentialMatrixException e) {
            throw new InvalidRotationAndTranslationException(e);
        }
    }
    
    /**
     * Sets essential matrix from provided rotation and translation of the
     * camera center relative to left view camera, which is assumed to be
     * located at origin of coordinates with no rotation.
     * @param rotation rotation of right camera relative to left camera.
     * @param cameraCenter camera center of right camera relative to left camera.
     * @throws InvalidRotationAndTranslationException if provided rotation and
     * camera center yield a degenerate epipolar geometry.
     */
    public void setFromRotationAndCameraCenter(Rotation3D rotation, 
            Point3D cameraCenter) throws 
            InvalidRotationAndTranslationException {
        setFromRotationAndCameraCenter(rotation, cameraCenter, 
                DEFAULT_SINGULAR_VALUES_THRESHOLD);
    }    

    /**
     * Sets essential matrix from provided rotation and translation of the
     * camera center relative to left view camera, which is assumed to be
     * located at origin of coordinates with no rotation.
     * @param rotation rotation of right camera relative to left camera.
     * @param cameraCenter camera center of right camera relative to left camera.
     * @param singularValuesThreshold threshold to determine that both singular
     * values of generated essential matrix are equal.
     * @throws InvalidRotationAndTranslationException if provided rotation and
     * camera center yield a degenerate epipolar geometry.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public final void setFromRotationAndCameraCenter(Rotation3D rotation, 
            Point3D cameraCenter, double singularValuesThreshold) throws 
            InvalidRotationAndTranslationException, IllegalArgumentException {
        
        if (singularValuesThreshold < 0) {
            throw new IllegalArgumentException();
        }
        
        try {
            Matrix rotationMatrix = rotation.asInhomogeneousMatrix();
        
            cameraCenter.normalize(); //to increase accuracy
            Matrix inhomCenterMatrix = new Matrix(
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 1);
            inhomCenterMatrix.setElementAtIndex(0, cameraCenter.getInhomX());
            inhomCenterMatrix.setElementAtIndex(1, cameraCenter.getInhomY());
            inhomCenterMatrix.setElementAtIndex(2, cameraCenter.getInhomZ());
        
            //translationMatrix = -rotationMatrix * inhomCenterMatrix
            rotationMatrix.multiplyByScalar(-1.0);
            rotationMatrix.multiply(inhomCenterMatrix);
            Matrix translationMatrix = rotationMatrix;
            
            //essentialMatrix = skew(translationMatrix) * rotationMatrix
            Matrix skewTranslationMatrix = Utils.skewMatrix(translationMatrix);
            rotationMatrix = rotation.asInhomogeneousMatrix();
            skewTranslationMatrix.multiply(rotationMatrix);

            setInternalMatrix(skewTranslationMatrix, singularValuesThreshold);
            
        } catch (AlgebraException | InvalidEssentialMatrixException e) {
            throw new InvalidRotationAndTranslationException(e);
        }
    }    
    
    /**
     * Sets essential matrix from provided fundamental matrix and intrinsic 
     * camera parameters.
     * @param fundamentalMatrix a fundamental matrix.
     * @param leftInstrinsicParameters intrinsic camera parameters of left view.
     * @param rightIntrinsicParameters intrinsic camera parameters of right view.
     * @throws InvalidPairOfIntrinsicParametersException if provided intrinsic
     * parameters generate an invalid essential matrix.
     */
    public final void setFromFundamentalMatrixAndIntrinsics(
            FundamentalMatrix fundamentalMatrix, 
            PinholeCameraIntrinsicParameters leftInstrinsicParameters,
            PinholeCameraIntrinsicParameters rightIntrinsicParameters)
            throws InvalidPairOfIntrinsicParametersException {
        
        try {
            Matrix k1 = leftInstrinsicParameters.getInternalMatrix();
            double normK1 = Utils.normF(k1);
            k1.multiplyByScalar(1.0 / normK1);

            Matrix k2 = rightIntrinsicParameters.getInternalMatrix();
            double normK2 = Utils.normF(k2);
            k2.multiplyByScalar(1.0 / normK2);

            fundamentalMatrix.normalize(); //to increase accuracy
            Matrix fundMatrix = fundamentalMatrix.getInternalMatrix();

            k2.transpose();
            //noinspection all
            Matrix transK2 = k2;

            //E = K2' * F * K1
            fundMatrix.multiply(k1);
            transK2.multiply(fundMatrix);
            //noinspection all
            Matrix essentialMatrix = transK2;

            double normEssential = Utils.normF(essentialMatrix);
            essentialMatrix.multiplyByScalar(1.0 / normEssential);

            mInternalMatrix = essentialMatrix;
            mNormalized = false;
            mLeftEpipole = mRightEpipole = null;   
        } catch (AlgebraException | GeometryException e) {
            throw new InvalidPairOfIntrinsicParametersException(e);
        }
    }
    
    /**
     * Converts this essential matrix into a fundamental matrix by applying
     * provided intrinsic parameters on left and right views.
     * The essential matrix only contains information abour rotation and
     * translation relating two views, while fundamental matrix also contains
     * information about the intrinsic parameters in both views.
     * NOTE: although essential matrix is a subclass of fundamental matrix, it
     * does not behave like a fundamental matrix.
     * @param leftIntrinsicParameters intrinsic parameters in left view.
     * @param rightIntrinsicParameters intrinsic parameters in right view.
     * @return a fundamental matrix.
     * @throws EpipolarException if something fails.
     */
    public FundamentalMatrix toFundamentalMatrix(
            PinholeCameraIntrinsicParameters leftIntrinsicParameters,
            PinholeCameraIntrinsicParameters rightIntrinsicParameters) 
            throws EpipolarException {
        try {
            normalize();
        
            Matrix essentialMatrix = getInternalMatrix();
        
            Matrix k1 = leftIntrinsicParameters.getInternalMatrix();
            Matrix invK1 = Utils.inverse(k1);
            double normInvK1 = Utils.normF(invK1);
            invK1.multiplyByScalar(1.0 / normInvK1);
        
            Matrix k2 = rightIntrinsicParameters.getInternalMatrix();
            Matrix invK2 = Utils.inverse(k2);
            double normInvK2 = Utils.normF(invK2);
            invK2.multiplyByScalar(1.0 / normInvK2);
            invK2.transpose();
            //noinspection all
            Matrix transInvK2 = invK2;
            
            essentialMatrix.multiply(invK1);
            transInvK2.multiply(essentialMatrix);
            //noinspection all
            Matrix fundMatrix = transInvK2;
            
            return new FundamentalMatrix(fundMatrix);
            
        } catch(Exception e) {
            throw new EpipolarException(e);
        }
    }
    
    /**
     * Computes all possible camera rotations and translations that can generate
     * this essential matrix.
     * @throws InvalidEssentialMatrixException if essential matrix contains
     * numerically unstable values.
     */
    public void computePossibleRotationAndTranslations() 
            throws InvalidEssentialMatrixException {
        
        try {
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    mInternalMatrix);

            decomposer.decompose();

            Matrix U = decomposer.getU();
            Matrix V = decomposer.getV();

            V.transpose();
            //noinspection all
            Matrix transV = V;

            mTranslation1 = new HomogeneousPoint2D(U.getElementAt(0, 2),
                U.getElementAt(1, 2), U.getElementAt(2, 2));
            mTranslation2 = new HomogeneousPoint2D(-U.getElementAt(0, 2),
                -U.getElementAt(1, 2), -U.getElementAt(2, 2));

            //W is a skew-symmetric matrix that can be used to obtain two possible
            //rotations
            Matrix W = new Matrix(FUNDAMENTAL_MATRIX_ROWS, FUNDAMENTAL_MATRIX_COLS);
            W.setElementAt(0, 1, -1.0);
            W.setElementAt(1, 0, 1.0);
            W.setElementAt(2, 2, 1.0);
            
            Matrix transW = W.transposeAndReturnNew();
            
            //R1 = U * W * V'
            W.multiply(transV);
            Matrix rotationMatrix1 = U.multiplyAndReturnNew(W);
            
            //R2 = U * W' * V'
            transW.multiply(transV);
            Matrix rotationMatrix2 = U.multiplyAndReturnNew(transW);
            
            mRotation1 = new MatrixRotation3D(rotationMatrix1);
            mRotation2 = new MatrixRotation3D(rotationMatrix2);
            
            mPossibleRotationsAndTranslationsAvailable = true;
        } catch(AlgebraException | InvalidRotationMatrixException e) {
            throw new InvalidEssentialMatrixException(e);
        }
    }
    
    /**
     * Indicates whether possible camera rotations and translations that can 
     * generate this essential matrix have already been computed or not.
     * @return true if possible camera rotations and translations have been
     * computed, false otherwise.
     */
    public boolean arePossibleRotationsAndTranslationsAvailable() {
        return mPossibleRotationsAndTranslationsAvailable;
    }
    
    /**
     * Gets first possible rotation that can generate this essential matrix.
     * @return first possible rotation.
     * @throws NotAvailableException if possible rotation has not yet been
     * computed.
     */
    public Rotation3D getFirstPossibleRotation() throws NotAvailableException {
        if (!arePossibleRotationsAndTranslationsAvailable()) {
            throw new NotAvailableException();
        }
                    
        return mRotation1;
    }
    
    /**
     * Gets second possible rotation that can generate this essential matrix.
     * @return second possible rotation.
     * @throws NotAvailableException if possible rotation has not yet been
     * computed.
     */
    public Rotation3D getSecondPossibleRotation() throws NotAvailableException {
        if (!arePossibleRotationsAndTranslationsAvailable()) {
            throw new NotAvailableException();
        }
        
        return mRotation2;
    }
    
    /**
     * Gets first possible translation that can generate this essential matrix.
     * @return first possible translation.
     * @throws NotAvailableException if possible translation has not yet been
     * computed.
     */
    public Point2D getFirstPossibleTranslation() throws NotAvailableException {
        if (!arePossibleRotationsAndTranslationsAvailable()) {
            throw new NotAvailableException();
        }
        
        return mTranslation1;
    }
    
    /**
     * Gets second possible translation that can generate this essential matrix.
     * @return second possible translation.
     * @throws NotAvailableException if possible translation has not yet been
     * computed.
     */
    public Point2D getSecondPossibleTranslation() throws NotAvailableException {
        if (!arePossibleRotationsAndTranslationsAvailable()) {
            throw new NotAvailableException();
        }
        
        return mTranslation2;
    }  
    
}
