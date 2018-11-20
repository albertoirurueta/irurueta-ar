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
package com.irurueta.ar.calibration;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.CholeskyDecomposer;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.*;

import java.io.Serializable;

/**
 * The dual image of the absolute conic (DIAC), is the projection of the
 * dual absolute quadric using a given pinhole camera.
 * In an ideal metric stratum, the dual absolute quadric is equal to the 
 * identity matrix, except for the bottom-right element, which is zero. In those
 * cases the dual image of the absolute conic only depends on the pinhole camera
 * intrinsic parameters, and for that reason the DIAC is typically used for
 * camera calibration purposes.
 */
@SuppressWarnings("WeakerAccess")
public class DualImageOfAbsoluteConic extends DualConic implements Serializable {
    
    /**
     * Constructor.
     * When working on a metric stratum, the DIAC is directly related by the
     * pinhole camera intrinsic parameters as C^-1=K*K'
     * @param k pinhole camera intrinsic parameters.
     */
    public DualImageOfAbsoluteConic(PinholeCameraIntrinsicParameters k) {
        super();
        setFromPinholeCameraIntrinsicParameters(k);
    }
    
    /**
     * Constructor.
     * When not working on a metric stratum, the DIAC can be obtained as the
     * projection of provided dual absolute quadric using provided camera
     * @param camera a pinhole camera.
     * @param dualAbsoluteQuadric the dual absolute quadric.
     */
    public DualImageOfAbsoluteConic(PinholeCamera camera, 
            DualQuadric dualAbsoluteQuadric) {
        super();
        setFromCameraAndDualAbsoluteQuadric(camera, dualAbsoluteQuadric);
    }
    
    /**
     * Constructor of this class. This constructor accepts every parameter
     * describing a dual conic (parameters a, b, c, d, e, f).
     * @param a Parameter A of the conic.
     * @param b Parameter B of the conic.
     * @param c Parameter C of the conic.
     * @param d Parameter D of the conic.
     * @param e Parameter E of the conic.
     * @param f Parameter F of the conic.
     */
    public DualImageOfAbsoluteConic(double a, double b, double c, double d, 
            double e, double f) {
        super(a, b, c, d, e, f);
    }
    
    /**
     * This method sets the matrix used to describe a dual conic.
     * This matrix must be 3x3 and symmetric.
     * @param m 3x3 Matrix describing the conic.
     * @throws IllegalArgumentException Raised when the size of the matrix is 
     * not 3x3.
     * @throws NonSymmetricMatrixException Raised when the conic matrix is not 
     * symmetric.
     */
    public DualImageOfAbsoluteConic(Matrix m) throws IllegalArgumentException, 
            NonSymmetricMatrixException {
        super(m);
    }    
    
    /**
     * Constructor without arguments.
     */
    protected DualImageOfAbsoluteConic() {
        super();
    }
    
    /**
     * Computes the conic corresponding to this dual conic.
     * @return A new conic instance of this dual conic.
     * @throws ConicNotAvailableException Raised if the rank of the dual conic 
     * matrix is not complete due to wrong parameters or numerical instability.
     */
    @Override
    public Conic getConic() throws ConicNotAvailableException {
        ImageOfAbsoluteConic c = new ImageOfAbsoluteConic();
        conic(c);
        return c;
    }
    
    
    /**
     * Sets DIAC parameters from pinhole camera intrinsic parameters when we
     * are working in a matric stratum, which is equal to C^-1=K*K'.
     * @param k pinhole camera intrinsic parameters.
     */
    public final void setFromPinholeCameraIntrinsicParameters(
            PinholeCameraIntrinsicParameters k) {
        Matrix kMatrix = k.getInternalMatrix();
        try {
            setParameters(kMatrix.multiplyAndReturnNew(
                    kMatrix.transposeAndReturnNew()));
        } catch (WrongSizeException | NonSymmetricMatrixException ignore) { }
    }
    
    /**
     * Sets DIAC parameters by projecting provided dual absolute quadric using
     * provided pinhole camera. This method can be used when not working in a
     * metric stratum. The projection is equal to C^-1=P*Q^-1*P'.
     * @param camera a pinhole camera.
     * @param dualAbsoluteQuadric the dual absolute quadric.
     */
    public final void setFromCameraAndDualAbsoluteQuadric(PinholeCamera camera,
            DualQuadric dualAbsoluteQuadric) {
        camera.project(dualAbsoluteQuadric, this);
    }
    
    /**
     * Assuming that we are working in a metric stratum this method obtains the
     * internal parameters of a pinhole camera by means of cholesky 
     * decomposition.
     * To avoid numerical inestabilities it is preferred to obtain the
     * ImageOfAbsoluteConic corresponding to this instance (using #getConic() 
     * method), and from there using 
     * ImageOfAbsoluteConic#getIntrinsicParameters() method, since it is more
     * reliable than using cholesky decomposition.
     * @return the internal parameters of a pinhole camera.
     * @throws InvalidPinholeCameraIntrinsicParametersException if pinhole
     * camera intrinsic parameters cannot be obtained from this conic instance.
     */
    public PinholeCameraIntrinsicParameters getIntrinsicParameters() 
            throws InvalidPinholeCameraIntrinsicParametersException {
        try {
            normalize();
            //the DIAC is a dual conic, and hence it is symmetric, hence:
            //C^-1=K*K'=(K*K')'=(C^-1)'
            //C=(K*K')^-1 = K'^-1*K^-1 = (K^-1)'*(K^-1), where K^-1 is still 
            //upper triangular
            Matrix m = asMatrix();
            Matrix invM = com.irurueta.algebra.Utils.inverse(m);
            CholeskyDecomposer decomposer = new CholeskyDecomposer(invM);
            decomposer.decompose();            
            Matrix inverseInternalParamsMatrix = decomposer.getR();
            Matrix internalParamsMatrix = com.irurueta.algebra.Utils.inverse(
                    inverseInternalParamsMatrix);
            return new PinholeCameraIntrinsicParameters(internalParamsMatrix);
        } catch (AlgebraException e) {
            throw new InvalidPinholeCameraIntrinsicParametersException(e);
        }
    }

}
