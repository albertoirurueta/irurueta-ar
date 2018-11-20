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

import com.irurueta.algebra.*;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.*;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.estimators.NotReadyException;

import java.io.Serializable;

/**
 * The fundamental matrix describes the epipolar geometry for a pair of cameras.
 * Epipoles are the projections of the opposite camera against the other.
 * By means of point correspondences it is possible to estimate the fundamental
 * matrix, which can later be used to estimate the associated pair of cameras.
 */
@SuppressWarnings("WeakerAccess")
public class FundamentalMatrix implements Serializable {
    /**
     * Number of rows of fundamental matrix.
     */
    public static final int FUNDAMENTAL_MATRIX_ROWS = 3;
    
    /**
     * Number of columns of fundamental matrix.
     */
    public static final int FUNDAMENTAL_MATRIX_COLS = 3;
    
    /**
     * Rank of fundamental matrix.
     */
    public static final int FUNDAMENTAL_MATRIX_RANK = 2;
    
    /**
     * Contains the internal representation of the fundamental matrix, which is
     * a 3x3 matrix having rank 2 defined up to scale.
     */
    protected Matrix mInternalMatrix;

    /**
     * Indicates whether fundamental matrix has been normalized. Normalization
     * can be used to increase the accuracy of estimations, since fundamental
     * matrix is defined up to scale.
     */
    protected boolean mNormalized;
    
    /**
     * Epipole for left view. Corresponds to the projection of the center of the
     * right camera on the left view.
     */
    protected Point2D mLeftEpipole;
    
    /**
     * Epipole for right view. Corresponds to the projection of the center of 
     * the left camera on the right view.
     */
    protected Point2D mRightEpipole;
        
    /**
     * Constructor.
     */
    public FundamentalMatrix() { }
    
    /**
     * Constructor.
     * @param internalMatrix matrix to be set internally.
     * @throws InvalidFundamentalMatrixException if provided matrix is not 3x3
     * or does not have rank 2.
     */
    public FundamentalMatrix(Matrix internalMatrix) 
            throws InvalidFundamentalMatrixException {
        internalSetInternalMatrix(internalMatrix);
    }
    
    /**
     * Constructor from pair of cameras.
     * @param leftCamera camera corresponding to left view.
     * @param rightCamera camera corresponding to right view.
     * @throws InvalidPairOfCamerasException if provided cameras do not span a 
     * valid epipolar geometry (i.e. they are planed in a degenerate 
     * configuration).
     */
    public FundamentalMatrix(PinholeCamera leftCamera, 
            PinholeCamera rightCamera) throws InvalidPairOfCamerasException {
        internalSetFromPairOfCameras(leftCamera, rightCamera);
    }
    
    /**
     * Constructor from an homography and right epipole.
     * @param homography 2D homography.
     * @param rightEpipole right epipole.
     * @throws InvalidFundamentalMatrixException if resulting fundamental matrix
     * is invalid, typically because of numerical instabilities.
     */
    public FundamentalMatrix(Transformation2D homography, Point2D rightEpipole)
            throws InvalidFundamentalMatrixException {
        internalSetFromHomography(homography, rightEpipole);
    }
    
    /**
     * Returns a copy of the internal matrix assigned to this instance.
     * @return copy of the internal matrix.
     * @throws NotAvailableException if internal matrix has not yet been 
     * provided.
     */
    public Matrix getInternalMatrix() throws NotAvailableException {
        if (!isInternalMatrixAvailable()) {
            throw new NotAvailableException();
        }
        return mInternalMatrix.clone();
    }
    
    /**
     * Sets internal matrix associated to this instance.
     * This method makes a copy of provided matrix.
     * @param internalMatrix matrix to be assigned to this instance.
     * @throws InvalidFundamentalMatrixException if provided matrix is not 3x3
     * or does not have rank 2.
     */
    public void setInternalMatrix(Matrix internalMatrix) 
            throws InvalidFundamentalMatrixException {
        internalSetInternalMatrix(internalMatrix);
    }
    
    /**
     * Method used internally to set the internal matrix associated to this 
     * instance.
     * This method makes a copy of provided matrix.
     * @param internalMatrix matrix to be assigned to this instance.
     * @throws InvalidFundamentalMatrixException if provided matrix is not 3x3
     * or does not have rank 2.
     */
    private void internalSetInternalMatrix(Matrix internalMatrix)
            throws InvalidFundamentalMatrixException {
        if (!isValidInternalMatrix(internalMatrix)) {
            throw new InvalidFundamentalMatrixException();
        }
        
        //because provided matrix is valid, we proceed to setting it
        
        mInternalMatrix = internalMatrix.clone();
        mNormalized = false;
        mLeftEpipole = mRightEpipole = null;        
    }
    
    /**
     * Returns a boolean indicating whether provided matrix is a valid 
     * fundamental matrix (i.e. has size 3x3 and rank 2).
     * @param internalMatrix matrix to be checked.
     * @return true if provided matrix is a valid fundamental matrix, false 
     * otherwise.
     */
    public static boolean isValidInternalMatrix(Matrix internalMatrix) {
        if (internalMatrix == null) {
            return false;
        }

        if (internalMatrix.getColumns() != FUNDAMENTAL_MATRIX_COLS ||
                internalMatrix.getRows() != FUNDAMENTAL_MATRIX_ROWS) {
            return false;
        }
        
        try {
            if (Utils.rank(internalMatrix) != FUNDAMENTAL_MATRIX_RANK) {
                return false;
            }
        } catch (DecomposerException e) {
            //an exception might be raised if matrix is not numerically
            //valid (i.e. contains infinity or nan values)
            return false;
        }
        
        return true;
    }
    
    /**
     * Sets fundamental matrix from provided pair of cameras.
     * @param leftCamera camera corresponding to left view.
     * @param rightCamera camera corresponding to right view.
     * @throws InvalidPairOfCamerasException if provided cameras do not span a 
     * valid epipolar geometry (i.e. they are planed in a degenerate 
     * configuration).
     */
    public void setFromPairOfCameras(PinholeCamera leftCamera,
            PinholeCamera rightCamera) throws InvalidPairOfCamerasException {
        internalSetFromPairOfCameras(leftCamera, rightCamera);
    }
    
    /**
     * Internal method to set fundamental matrix from provided pair of cameras.
     * @param leftCamera camera corresponding to left view.
     * @param rightCamera camera corresponding to right view.
     * @throws InvalidPairOfCamerasException if provided cameras do not span a
     * valid epipolar geometry (i.e. they are planed in a degenerate 
     * configuration).
     */
    private void internalSetFromPairOfCameras(PinholeCamera leftCamera,
            PinholeCamera rightCamera) throws InvalidPairOfCamerasException {
        try {
            //normalize cameras to increase accuracy of results and fix their 
            //signs if needed
            leftCamera.normalize();
            rightCamera.normalize();

            if (!leftCamera.isCameraSignFixed()) {
                leftCamera.fixCameraSign();
            }
            if (!rightCamera.isCameraSignFixed()) {
                rightCamera.fixCameraSign();
            }

            //left epipole consists on the projection of right camera center 
            //(C') using left camera P
            if (!rightCamera.isCameraCenterAvailable()) {
                //if camera center is not available, we need to decompose such
                //camera
                rightCamera.decompose(false, true);
            }

            Point3D rightCameraCenter = rightCamera.getCameraCenter();
            Point2D leftEpipole = leftCamera.project(rightCameraCenter);
            leftEpipole.normalize(); //normalize to increase accuracy

            //compute skew matrix of left epipole
            Matrix skewLeftEpipoleMatrix = Utils.skewMatrix(
                    leftEpipole.asArray());
            skewLeftEpipoleMatrix.transpose();
            //noinspection all
            Matrix transSkewLeftEpipoleMatrix = skewLeftEpipoleMatrix;

            //compute transposed of internal left pinhole camera
            Matrix transLeftCameraMatrix = leftCamera.getInternalMatrix().
                    transposeAndReturnNew();

            //compute transposed of internal right pinhole camera
            Matrix transRightCameraMatrix = rightCamera.getInternalMatrix().
                    transposeAndReturnNew();

            //compute pseudo-inverse of transposed right pinhole camera
            Matrix pseudoTransRightCameraMatrix = Utils.pseudoInverse(
                    transRightCameraMatrix);

            //compute pseudoTransRightCameraMatrix * transLeftCameraMatrix *
            //transSkewLeftEpipoleMatrix
            transLeftCameraMatrix.multiply(transSkewLeftEpipoleMatrix);
            pseudoTransRightCameraMatrix.multiply(transLeftCameraMatrix);
            //noinspection all
            Matrix fundamentalMatrix = pseudoTransRightCameraMatrix;

            //test that resulting matrix is 3x3 and rank 2, otherwise provided
            //cameras span a degenerate epipolar geometry and are not valid
            if (!isValidInternalMatrix(fundamentalMatrix)) {
                throw new InvalidPairOfCamerasException();
            }

            mInternalMatrix = fundamentalMatrix;
            mNormalized = false;
            mLeftEpipole = mRightEpipole = null;
        } catch (InvalidPairOfCamerasException e) {
            throw e;
        } catch (AlgebraException | GeometryException e) {
            throw new InvalidPairOfCamerasException(e);
        }
    }
    
    /**
     * Sets fundamental matrix from provided 2D homography and right epipole.
     * @param homography 2D homography.
     * @param rightEpipole right epipole.
     * @throws InvalidFundamentalMatrixException if resulting fundamental matrix
     * is invalid, typically because of numerical instabilities.
     */
    public void setFromHomography(Transformation2D homography, 
            Point2D rightEpipole) throws InvalidFundamentalMatrixException {
        internalSetFromHomography(homography, rightEpipole);
    }
    
    /**
     * Internal method to sets fundamental matrix from provided 2D homography
     * and right epipole.
     * @param homography 2D homography.
     * @param rightEpipole right epipole.
     * @throws InvalidFundamentalMatrixException if resulting fundamental matrix
     * is invalid, typically because of numerical instabilities.
     */
    private void internalSetFromHomography(Transformation2D homography, 
            Point2D rightEpipole) throws InvalidFundamentalMatrixException {
        
        rightEpipole.normalize();
        
        Matrix f = null;
        try {
            f = Utils.skewMatrix(new double[] {
                rightEpipole.getHomX(),
                rightEpipole.getHomY(),
                rightEpipole.getHomW()
            });
            
            f.multiply(homography.asMatrix());
        } catch (WrongSizeException ignore) { /* never happens */ }
        

        //test that resulting matrix is 3x3 and rank 2, otherwise provided
        //matrix is numerically unstable and not valid
        if (!isValidInternalMatrix(f)) {
            throw new InvalidFundamentalMatrixException();
        }

        mInternalMatrix = f;
    }
    
    /**
     * Indicates whether this instance has its internal matrix set.
     * @return true if internal matrix has been set, false otherwise.
     */
    public boolean isInternalMatrixAvailable() {
        return mInternalMatrix != null;
    }    
    
    /**
     * Returns epipolar line on left view corresponding to point on right view.
     * @param rightPoint a point on the right view.
     * @return epipolar line on left view.
     * @throws NotReadyException if internal matrix has not yet been set.
     */
    public Line2D getLeftEpipolarLine(Point2D rightPoint) 
            throws NotReadyException {
        Line2D line = new Line2D();
        leftEpipolarLine(rightPoint, line);
        return line;
    }
    
    /**
     * Computes epipolar line on left view corresponding to point on right view.
     * @param rightPoint a point on the right view.
     * @param result line instance where result will be stored.
     * @throws NotReadyException if internal matrix has not yet been set.
     */
    public void leftEpipolarLine(Point2D rightPoint, Line2D result) 
            throws NotReadyException {
        
        if (!isInternalMatrixAvailable()) {
            throw new NotReadyException();
        }
        
        //make sure that point is homogeneous
        if (rightPoint.getType() != CoordinatesType.HOMOGENEOUS_COORDINATES) {
            rightPoint = new HomogeneousPoint2D(rightPoint);
        }
        
        //normalize to increase accuracy
        rightPoint.normalize();
        normalize();
        
        //compute transposed fundamental matrix
        Matrix transFundMatrix = mInternalMatrix.transposeAndReturnNew();
        
        //compute left epipolar line as the product of transposed fundamental
        //matrix with homogeneous right 2D point 
        double[] rightPointArray = rightPoint.asArray();
        Matrix rightPointMatrix = Matrix.newFromArray(rightPointArray, true);
        
        try {
            Matrix leftEpipolarLineMatrix = transFundMatrix.
                    multiplyAndReturnNew(rightPointMatrix);
        
            result.setParameters(leftEpipolarLineMatrix.getBuffer());
        } catch (WrongSizeException ignore) { 
            //this exception will never occur
        }
        
        result.normalize(); //normalize line to increase accuracy
    }
    
    /**
     * Returns epipolar line on right view corresponding to point on left view.
     * @param leftPoint a point on the left view.
     * @return epipolar line on right view.
     * @throws NotReadyException if internal matrix has not yet been set.
     */
    public Line2D getRightEpipolarLine(Point2D leftPoint) 
            throws NotReadyException {
        Line2D line = new Line2D();
        rightEpipolarLine(leftPoint, line);
        return line;
    }
    
    /**
     * Computes epipolar line on right view corresponding to point on left view.
     * @param leftPoint a point on the left view.
     * @param result line instance where result will be stored.
     * @throws NotReadyException if internal matrix has not yet been set.
     */
    public void rightEpipolarLine(Point2D leftPoint, Line2D result)
            throws NotReadyException {
        
        if (!isInternalMatrixAvailable()) {
            throw new NotReadyException();
        }
        
        //make sure that point is homogeneous
        if (leftPoint.getType() != CoordinatesType.HOMOGENEOUS_COORDINATES) {
            leftPoint = new HomogeneousPoint2D(leftPoint);
        }
        
        //normalize to increase accuracy        
        leftPoint.normalize();
        normalize();
        
        //compute right epipolar line as the product of fundamental matrix with
        //homogeneous left 2D point
        double[] leftPointArray = leftPoint.asArray();
        Matrix leftPointMatrix = Matrix.newFromArray(leftPointArray, true);
        try {
            Matrix rightEpipolarPointMatrix = 
                    mInternalMatrix.multiplyAndReturnNew(leftPointMatrix);
            
            result.setParameters(rightEpipolarPointMatrix.getBuffer());
        } catch (WrongSizeException ignore) { 
            //this exception will never occur
        } 
        
        result.normalize(); //normalize line to increase accuracy        
    }
    
    /**
     * Returns left epipole, which corresponds to the center of right camera
     * projected on left view.
     * @return left epipole.
     * @throws NotAvailableException if epipoles haven't been computed.
     */
    public Point2D getLeftEpipole() throws NotAvailableException {
        if (!areEpipolesAvailable()) {
            throw new NotAvailableException();
        }
        
        return mLeftEpipole;
    }
    
    /**
     * Returns right epipole, which corresponds to the center of left camera
     * projected on right view.
     * @return right epipole.
     * @throws NotAvailableException if epipoles haven't been computed.
     */
    public Point2D getRightEpipole() throws NotAvailableException {
        if (!areEpipolesAvailable()) {
            throw new NotAvailableException();
        }
        
        return mRightEpipole;
    }
    
    /**
     * Normalizes the internal representation of this instance.
     * Normalization is done to increase accuracy of computations with this
     * instance.
     * @throws NotReadyException if internal matrix has not already been set.
     */
    public void normalize() throws NotReadyException {
        if (!mNormalized) {
            if (!isInternalMatrixAvailable()) {
                throw new NotReadyException();
            }
            
            double norm = Utils.normF(mInternalMatrix);
            
            mInternalMatrix.multiplyByScalar(1.0 / norm);
            
            mNormalized = true;
        }
    }
    
    /**
     * Indicates whether this instance is currently normalized or not.
     * @return true if this instance is normalized, false otherwise.
     */
    public boolean isNormalized() {
        return mNormalized;
    }
    
    /**
     * Computes the left and right epipoles of this instance.
     * @throws NotReadyException if an internal matrix has not yet been 
     * provided.
     * @throws InvalidFundamentalMatrixException if internal matrix is 
     * numerically unstable and epipoles couldn't be computed.
     */
    public void computeEpipoles() throws NotReadyException, 
            InvalidFundamentalMatrixException {
        if (!isInternalMatrixAvailable()) {
            throw new NotReadyException();
        }
        
        normalize(); //to increase accuracy
        
        //compute SVD of internal fundamental matrix with singular values 
        //ordered from largest to smallest. Since fundamental matrix has rank 2,
        //last singular value is zero
        SingularValueDecomposer decomposer = new SingularValueDecomposer(
                mInternalMatrix); //F = U*S*V'
        
        try {
            decomposer.decompose();

            Matrix U = decomposer.getU();
            Matrix V = decomposer.getV();
            double[] array = new double[
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH];

            //left epipole is the last column of V (right null-space)        
            V.getSubmatrixAsArray(0, 2, 2, 2, array);
            mLeftEpipole = new HomogeneousPoint2D(array);

            //right epipole is the last column of U (left null-space)
            U.getSubmatrixAsArray(0, 2, 2, 2, array);
            mRightEpipole = new HomogeneousPoint2D(array);
        } catch (AlgebraException e) {
            throw new InvalidFundamentalMatrixException(e);
        }
    }
    
    /**
     * Indicates whether epipoles have been computed and are available for
     * retrieval.
     * @return true if epipoles are available, false otherwise.
     */
    @SuppressWarnings("all")
    public boolean areEpipolesAvailable() {
        return mLeftEpipole != null && mRightEpipole != null;
    }
    
    /**
     * Generates a pair of cameras in any arbitrary projective space which
     * produce this fundamental matrix.
     * This method can be used to obtain a pair of cameras related by this
     * fundamental matrix in order to initialize geometry and get an initial set
     * of cameras.
     * However, because cameras are in any arbitrary projective space, they need
     * to be transformed into a metric space using a Dual Absolute Quadric 
     * estimator.
     * @param leftCamera instance where left camera will be stored.
     * @param rightCamera instance where right camera will be stored.
     * @param referencePlaneDirectorVectorX x coordinate of reference plane 
     * director vector. This can be any arbitrary value, however typically the
     * reference plane is assumed to be the plane at infinity, hence the value
     * typically is zero.
     * @param referencePlaneDirectorVectorY y coordinate of reference plane
     * director vector. This can be any arbitrary value, however typically the
     * reference plane is assumed to be the plane at infinity, hence the value
     * typically is zero.
     * @param referencePlaneDirectorVectorZ z coordinate of reference plane
     * director vector. This can be any arbitrary value, however typically the
     * reference plane is assumed to be the plane at infinity, hence the value
     * typically is zero.
     * @param scaleFactor scale factor defining the length of the baseline in
     * a metric stratum. This can be any value, since cameras are obtained in
     * an arbitrary projective stratum. However, even if the stratum was metric,
     * cameras can only be defined up to scale. A typical value is a scale 
     * factor of one.
     * @throws InvalidFundamentalMatrixException if internal matrix is 
     * numerically unstable and epipoles couldn't be computed.
     * @throws NotReadyException if an internal matrix has not yet been 
     * provided.
     */
    public void generateCamerasInArbitraryProjectiveSpace(
            PinholeCamera leftCamera, PinholeCamera rightCamera, 
            double referencePlaneDirectorVectorX, 
            double referencePlaneDirectorVectorY,
            double referencePlaneDirectorVectorZ,
            double scaleFactor) throws InvalidFundamentalMatrixException, 
            NotReadyException {
        
        normalize();
        
        try {
            if (!areEpipolesAvailable()) {
                computeEpipoles();
            }
            Point2D rightEpipole = getRightEpipole();
            rightEpipole.normalize();
            
            Matrix e2 = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            e2.setElementAtIndex(0, rightEpipole.getHomX());
            e2.setElementAtIndex(1, rightEpipole.getHomY());
            e2.setElementAtIndex(2, rightEpipole.getHomW());
            
            Matrix tmp = Utils.skewMatrix(e2);
            tmp.multiply(mInternalMatrix);
            
            if (referencePlaneDirectorVectorX != 0.0 || 
                    referencePlaneDirectorVectorY != 0.0 ||
                    referencePlaneDirectorVectorZ != 0.0) {
                Matrix tmp2 = new Matrix(
                        1, Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
                tmp2.setElementAtIndex(0, referencePlaneDirectorVectorX);
                tmp2.setElementAtIndex(1, referencePlaneDirectorVectorY);
                tmp2.setElementAtIndex(2, referencePlaneDirectorVectorZ);
                Matrix tmp3 = e2.multiplyAndReturnNew(tmp2);
                
                tmp.add(tmp3);
            }
            
            e2.multiplyByScalar(scaleFactor);
            
            Matrix leftCameraMatrix = Matrix.identity(
                    PinholeCamera.PINHOLE_CAMERA_MATRIX_ROWS, 
                    PinholeCamera.PINHOLE_CAMERA_MATRIX_COLS);
            
            Matrix rightCameraMatrix = new Matrix(
                    PinholeCamera.PINHOLE_CAMERA_MATRIX_ROWS, 
                    PinholeCamera.PINHOLE_CAMERA_MATRIX_COLS);
            rightCameraMatrix.setSubmatrix(0, 0, 2, 2, tmp);
            rightCameraMatrix.setSubmatrix(0, 3, 2, 3, e2);
            
            leftCamera.setInternalMatrix(leftCameraMatrix);
            rightCamera.setInternalMatrix(rightCameraMatrix);
            
        } catch (NotAvailableException | WrongSizeException ignore) { }
    }
    
    /**
     * Generates a pair of cameras in any arbitrary projective space which
     * produce this fundamental matrix.
     * This method can be used to obtain a pair of cameras related by this
     * fundamental matrix in order to initialize geometry and get an initial set
     * of cameras.
     * However, because cameras are in any arbitrary projective space, they need
     * to be transformed into a metric space using a Dual Absolute Quadric 
     * estimator.
     * This method assumes that the reference plane is the plane at infinity and
     * that scale factor is one.
     * @param leftCamera instance where left camera will be stored.
     * @param rightCamera instance where right camera will be stored.
     * @throws InvalidFundamentalMatrixException if internal matrix is 
     * numerically unstable and epipoles couldn't be computed.
     * @throws NotReadyException if an internal matrix has not yet been 
     * provided.
     */
    public void generateCamerasInArbitraryProjectiveSpace(
            PinholeCamera leftCamera, PinholeCamera rightCamera) 
            throws InvalidFundamentalMatrixException, NotReadyException {
        generateCamerasInArbitraryProjectiveSpace(leftCamera, rightCamera, 
                0.0, 0.0, 0.0, 1.0);
    }
}
