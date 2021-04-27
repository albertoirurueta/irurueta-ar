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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.GeometryException;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.Transformation2D;

/**
 * Estimates the camera pose for a given homography and pinhole camera intrinsic
 * parameters.
 * <p>
 * This class is used for camera calibration but can also be used for
 * virtual reality applications
 * This class assumes that the homography is estimated as the result of
 * projecting 3D points located in a plane.
 * <p>
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
     * Estimates camera posed based on provided intrinsic parameters and
     * 2D homography.
     *
     * @param intrinsic  pinhole camera intrinsic parameters.
     * @param homography a 2D homography.
     * @throws AlgebraException  if provided data is numerically unstable.
     * @throws GeometryException if a proper camera rotation cannot be found.
     */
    public void estimate(
            final PinholeCameraIntrinsicParameters intrinsic,
            final Transformation2D homography) throws AlgebraException, GeometryException {
        mRotation = new MatrixRotation3D();
        mCameraCenter = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        mCamera = new PinholeCamera();
        estimate(intrinsic, homography, mRotation, mCameraCenter, mCamera);
    }

    /**
     * Returns estimated camera rotation.
     *
     * @return estimated camera rotation.
     */
    public Rotation3D getRotation() {
        return mRotation;
    }

    /**
     * Returns estimated camera center.
     *
     * @return estimated camera center.
     */
    public Point3D getCameraCenter() {
        return mCameraCenter;
    }

    /**
     * Returns estimated camera.
     *
     * @return estimated camera.
     */
    public PinholeCamera getCamera() {
        return mCamera;
    }

    /**
     * Estimates camera pose based on provided intrinsic parameters and
     * 2D homography.
     *
     * @param intrinsic             pinhole camera intrinsic parameters.
     * @param homography            a 2D homography.
     * @param estimatedRotation     instance where estimated rotation will be stored.
     * @param estimatedCameraCenter instance where estimated camera center will
     *                              be stored.
     * @param estimatedCamera       instance where estimated camera will be stored.
     * @throws AlgebraException  if provided data is numerically unstable.
     * @throws GeometryException if a proper camera rotation cannot be found.
     */
    public static void estimate(
            final PinholeCameraIntrinsicParameters intrinsic,
            final Transformation2D homography,
            final Rotation3D estimatedRotation,
            final Point3D estimatedCameraCenter,
            final PinholeCamera estimatedCamera)
            throws AlgebraException, GeometryException {

        // inverse of intrinsic parameters matrix

        // what follows next is equivalent to Utils.inverse(intrinsic.getInternalMatrix())
        final Matrix intrinsicInvMatrix = intrinsic.getInverseInternalMatrix();

        if (homography instanceof ProjectiveTransformation2D) {
            ((ProjectiveTransformation2D) homography).normalize();
        }
        final Matrix homographyMatrix = homography.asMatrix();

        final Matrix h1 = homographyMatrix.getSubmatrix(
                0, 0, 2, 0);
        final Matrix h2 = homographyMatrix.getSubmatrix(
                0, 1, 2, 1);
        final Matrix h3 = homographyMatrix.getSubmatrix(
                0, 2, 2, 2);

        final Matrix matR1 = intrinsicInvMatrix.multiplyAndReturnNew(h1);
        final Matrix matR2 = intrinsicInvMatrix.multiplyAndReturnNew(h2);
        final Matrix matT = intrinsicInvMatrix.multiplyAndReturnNew(h3);

        // because rotation matrices are orthonormal, we find norm of 1st or
        // 2nd column (both should be equal, except for rounding errors)
        // to normalize columns 1 and 2.
        // Because norms might not be equal, we use average or norms of matR1 and
        // matR2
        final double norm1 = Utils.normF(matR1);
        final double norm2 = Utils.normF(matR2);
        final double invNorm = 2.0 / (norm1 + norm2);
        matR1.multiplyByScalar(invNorm);
        matR2.multiplyByScalar(invNorm);

        // also normalize translation term, since pinhole camera is defined
        // up to scale
        matT.multiplyByScalar(invNorm);

        final double[] r1 = matR1.getBuffer();
        final double[] r2 = matR2.getBuffer();

        // 3rd column of rotation must be orthogonal to 1st and 2nd columns and
        // also have norm 1, because rotations are orthonormal
        final double[] r3 = Utils.crossProduct(r1, r2);
        ArrayUtils.normalize(r3);

        final Matrix rot = new Matrix(3, 3);
        rot.setSubmatrix(0, 0, 2, 0, r1);
        rot.setSubmatrix(0, 1, 2, 1, r2);
        rot.setSubmatrix(0, 2, 2, 2, r3);

        // because of noise in data during the estimation of both the homography
        // and the intrinsic parameters, it might happen that r1 and r2 are not
        // perfectly orthogonal, for that reason we approximate matrix rot for
        // the closest orthonormal matrix to ensure that it is a valid rotation
        // matrix. This is done by obtaining the SVD decomposition and setting
        // all singular values to one, so that R = U*S*V' = U*V', S = I
        final SingularValueDecomposer decomposer =
                new SingularValueDecomposer(rot);
        decomposer.decompose();
        final Matrix u = decomposer.getU();
        final Matrix v = decomposer.getV();
        v.transpose();

        // U and V are orthonormal, hence U*V' is also orthonormal and can be
        // considered a rotation (up to sign)
        u.multiply(v, rot);

        // we need to ensure that rotation has determinant equal to 1, otherwise
        // we change sign
        final double det = Utils.det(rot);
        if (det < 0.0) {
            rot.multiplyByScalar(-1.0);
        }

        estimatedRotation.fromMatrix(rot);

        // camera center

        // t = K^-1*h3 = -R*C --> C=-R^-1*t=-R'*t
        // p4 = -K*R*C = K*t = K*K^-1*h3 = h3 --> C= -(K*R)^-1*h3
        final Matrix invRot = rot.transposeAndReturnNew();
        invRot.multiply(matT);
        invRot.multiplyByScalar(-1.0);

        final double[] centerBuffer = invRot.getBuffer();
        estimatedCameraCenter.setInhomogeneousCoordinates(
                centerBuffer[0], centerBuffer[1], centerBuffer[2]);

        // check that origin of coordinates (which is typically one of the
        // pattern points) is located in front of the camera, otherwise
        // fix camera
        estimatedCamera.setIntrinsicAndExtrinsicParameters(intrinsic,
                estimatedRotation, estimatedCameraCenter);
    }
}
