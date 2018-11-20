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
package com.irurueta.ar.calibration.estimators;

import com.irurueta.algebra.*;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.*;

/**
 * Estimates the camera pose for a given homography and pinhole camera intrinsic
 * parameters.
 * 
 * This class is used for camera calibration but can also be used for
 * virtual reality applications
 * This class assumes that the homography is estimated as the result of
 * projecting 3D points located in a plane.
 * 
 * This class is based on technique described at:
 * Zhengyou Zhang. A Flexible New Technique for Camera Calibration. Technical 
 * Report. MSR-TR-98-71. December 2, 1998
 */
public class CameraPoseEstimator {
    
    /**
     * Estimated camera rotation.
     */
    private Rotation3D mRotation;
    
    /**
     * Estimated camera center.
     */
    private Point3D mCameraCenter;
    
    /**
     * Estimated camera.
     */
    private PinholeCamera mCamera;
    
    /**
     * Constructor.
     */
    public CameraPoseEstimator() { }
    
    /**
     * Estimates camera posed based on provided intrinsic parameters and
     * 2D homography.
     * @param intrinsic pinhole camera intrinsic parameters.
     * @param homography a 2D homography.
     * @throws AlgebraException if provided data is numerically unstable.
     * @throws GeometryException if a proper camera rotation cannot be found.
     */
    public void estimate(PinholeCameraIntrinsicParameters intrinsic,
            Transformation2D homography) throws AlgebraException, 
            GeometryException {
        mRotation = new MatrixRotation3D();
        mCameraCenter = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        mCamera = new PinholeCamera();
        estimate(intrinsic, homography, mRotation, mCameraCenter, mCamera);
    }
    
    /**
     * Returns estimated camera rotation.
     * @return estimated camera rotation.
     */
    public Rotation3D getRotation() {
        return mRotation;
    }
    
    /**
     * Returns estimated camera center.
     * @return estimated camera center.
     */
    public Point3D getCameraCenter() {
        return mCameraCenter;
    }
    
    /**
     * Returns estimated camera.
     * @return estimated camera.
     */
    public PinholeCamera getCamera() {
        return mCamera;
    }
            
    /**
     * Estimates camera pose based on provided intrinsic parameters and
     * 2D homography.
     * @param intrinsic pinhole camera intrinsic parameters.
     * @param homography a 2D homography.
     * @param estimatedRotation instance where estimated rotation will be stored.
     * @param estimatedCameraCenter instance where estimated camera center will
     * be stored.
     * @param estimatedCamera instance where estimated camera will be stored.
     * @throws AlgebraException if provided data is numerically unstable.
     * @throws GeometryException if a proper camera rotation cannot be found.
     */
    public static void estimate(
            PinholeCameraIntrinsicParameters intrinsic, 
            Transformation2D homography, Rotation3D estimatedRotation,
            Point3D estimatedCameraCenter, PinholeCamera estimatedCamera)
            throws AlgebraException, GeometryException {
        
        //inverse of intrinsic parameters matrix

        //what follows next is equivalent to Utils.inverse(
        //intrinsic.getInternalMatrix());
        Matrix intrinsicInvMatrix = intrinsic.getInverseInternalMatrix();
                
        if(homography instanceof ProjectiveTransformation2D){
            ((ProjectiveTransformation2D)homography).normalize();
        }
        Matrix homographyMatrix = homography.asMatrix();

        Matrix h1 = homographyMatrix.getSubmatrix(0, 0, 2, 0);
        Matrix h2 = homographyMatrix.getSubmatrix(0, 1, 2, 1);
        Matrix h3 = homographyMatrix.getSubmatrix(0, 2, 2, 2);

        Matrix matR1 = intrinsicInvMatrix.multiplyAndReturnNew(h1);
        Matrix matR2 = intrinsicInvMatrix.multiplyAndReturnNew(h2);
        Matrix matT = intrinsicInvMatrix.multiplyAndReturnNew(h3);

        //because rotation matrices are orthonormal, we find norm of 1st or
        //2nd column (both should be equal, except for rounding errors) 
        //to normalize columns 1 and 2.
        //Because norms might not be equal, we use average or norms of matR1 and 
        //matR2
        double norm1 = Utils.normF(matR1);
        double norm2 = Utils.normF(matR2);
        double invNorm = 2.0/(norm1 + norm2);
        matR1.multiplyByScalar(invNorm);
        matR2.multiplyByScalar(invNorm);

        //also normalize translation term, since pinhole camera is defined
        //up to scale
        matT.multiplyByScalar(invNorm);

        double[] r1 = matR1.getBuffer();
        double[] r2 = matR2.getBuffer();

        //3rd column of rotation must be orthogonal to 1st and 2nd columns and
        //also have norm 1, because rotations are orthonormal
        double[] r3 = Utils.crossProduct(r1, r2);
        ArrayUtils.normalize(r3);
        
        Matrix rot = new Matrix(3, 3);
        rot.setSubmatrix(0, 0, 2, 0, r1);
        rot.setSubmatrix(0, 1, 2, 1, r2);
        rot.setSubmatrix(0, 2, 2, 2, r3);

        //because of noise in data during the estimation of both the homography
        //and the intrinsic parameters, it might happen that r1 and r2 are not 
        //perfectly othogonal, for that reason we approximate matrix rot for
        //the closest orthonormal matrix to ensure that it is a valid rotation
        //matrix. This is done by obtaining the SVD decomposition and setting
        //all singular values to one, so that R = U*S*V' = U*V', S = I
        SingularValueDecomposer decomposer = 
                new SingularValueDecomposer(rot);
        decomposer.decompose();
        Matrix U = decomposer.getU();
        Matrix V = decomposer.getV();
        V.transpose();

        //U and V are orthonormal, hence U*V' is also orthonormal and can be
        //considered a rotation (up to sign)
        U.multiply(V, rot);
        
        //we need to ensure that rotation has determinant equal to 1, otherwise
        //we change sign
        double det = Utils.det(rot);
        if (det < 0.0) {
            rot.multiplyByScalar(-1.0);
        }

        estimatedRotation.fromMatrix(rot);

        //camera center

        //t = K^-1*h3 = -R*C --> C=-R^-1*t=-R'*t
        //p4 = -K*R*C = K*t = K*K^-1*h3 = h3 --> C= -(K*R)^-1*h3
        Matrix invRot = rot.transposeAndReturnNew();
        invRot.multiply(matT);
        invRot.multiplyByScalar(-1.0);

        double[] centerBuffer = invRot.getBuffer();
        estimatedCameraCenter.setInhomogeneousCoordinates(
                centerBuffer[0], centerBuffer[1], centerBuffer[2]);
        
        //check that origin of coordinates (which is typically one of the
        //pattern points) is located in front of the camera, otherwise
        //fix camera        
        estimatedCamera.setIntrinsicAndExtrinsicParameters(intrinsic, 
                estimatedRotation, estimatedCameraCenter);        
    }    
}
