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
import com.irurueta.geometry.*;

import java.io.Serializable;

/**
 * The image of the absolute conic, is the projection of the absolute
 * quadric using a given pinhole camera.
 * Because the absolute quadric cannot be computed from the dual absolute 
 * quadric because the latter is degenerate, then this instance assumes
 * that we always work on an ideal metric stratum.
 * In such cases, the image of the absolute conic (IAC), is directly related
 * to the inverse of pinhole camera intrinsic parameters.
 */
@SuppressWarnings("WeakerAccess")
public class ImageOfAbsoluteConic extends Conic implements Serializable {
    
    /**
     * Constructor.
     * When working on a metric stratum, the IAC is related by the pinhole
     * camera intrinsic parameters as C=(K^-1)'*(K^-1).
     * @param k pinhole camera intrinsic parameters.
     * @throws InvalidPinholeCameraIntrinsicParametersException if intrinsic
     * parameters cannot be inverted (i.e. this might happen when values are
     * incorrectly set such as when focal length is zero, etc). Typically this
     * will never be thrown.
     */
    public ImageOfAbsoluteConic(PinholeCameraIntrinsicParameters k) 
            throws InvalidPinholeCameraIntrinsicParametersException {
        super();
        setFromPinholeCameraIntrinsicParameters(k);
    }
    
    /**
     * Constructor of this class. This constructor accepts every parameter
     * describing a conic (parameters a, b, c, d, e, f).
     * @param a Parameter A of the conic.
     * @param b Parameter B of the conic.
     * @param c Parameter C of the conic.
     * @param d Parameter D of the conic.
     * @param e Parameter E of the conic.
     * @param f Parameter F of the conic.
     */
    public ImageOfAbsoluteConic(double a, double b, double c, double d, 
            double e, double f) {
        super(a, b, c, d, e, f);
    }    
    
    /**
     * This method sets the matrix used to describe a conic.
     * This matrix must be 3x3 and symmetric.
     * @param m 3x3 Matrix describing the conic.
     * @throws IllegalArgumentException Raised when the size of the matrix is 
     * not 3x3.
     * @throws NonSymmetricMatrixException Raised when the conic matrix is not 
     * symmetric.
     */
    public ImageOfAbsoluteConic(Matrix m) throws IllegalArgumentException, 
            NonSymmetricMatrixException {
        super(m);
    }    
    
    /**
     * Constructor without arguments.
     */
    protected ImageOfAbsoluteConic() {
        super();
    }
    
    /**
     * Computes the dual conic of this conic.
     * The dual conic is equal to the inverse of the conic matrix.
     * @return A new DualConic corresponding to the dual conic of this instance.
     * @throws DualConicNotAvailableException Raised if the dual conic does not 
     * exist because this conic instance is degenerate (its inverse cannot be 
     * computed).
     */
    @Override
    public DualConic getDualConic() throws DualConicNotAvailableException {
        DualImageOfAbsoluteConic dualConic = new DualImageOfAbsoluteConic();
        dualConic(dualConic);
        return dualConic;
    }
    
    /**
     * Sets IAC parameters from pinhole camera intrinsic parameters when we
     * are working in a metric stratum, which is equal to C=(K^-1)'*(K^-1).
     * @param k pinhole camera intrinsic parameters.
     * @throws InvalidPinholeCameraIntrinsicParametersException if intrinsic
     * parameters cannot be inverted (i.e. this might happen when values are
     * incorrectly set such as when focal length is zero, etc). Typically this
     * will never be thrown.
     */
    public final void setFromPinholeCameraIntrinsicParameters(
            PinholeCameraIntrinsicParameters k) 
            throws InvalidPinholeCameraIntrinsicParametersException{
        Matrix kMatrix = k.getInternalMatrix();
        try {
            Matrix invKMatrix = com.irurueta.algebra.Utils.inverse(kMatrix);
            setParameters(invKMatrix.transposeAndReturnNew().
                    multiplyAndReturnNew(invKMatrix));
        } catch (AlgebraException e) {
            throw new InvalidPinholeCameraIntrinsicParametersException(e);
        } catch (NonSymmetricMatrixException ignore) { } //this will never happen
    }
    
    /**
     * Assuming that we are working in a metric stratum this method obtains the
     * internal parameters of a pinhole camera analitically from IAC.
     * This method should be preferred over cholesky decomposition as it is
     * more numerically stable.
     * @return the internal parameters of a pinhole camera.
     * @throws InvalidPinholeCameraIntrinsicParametersException if pinhole
     * camera intrinsic parameters cannot be obtained from this conic instance.
     */    
    public PinholeCameraIntrinsicParameters getIntrinsicParameters() 
            throws InvalidPinholeCameraIntrinsicParametersException {
        try {
            normalize();
            
            //A conic is defined as [A  B   D]
            //                      [B  C   E]
            //                      [D  E   F]
            double b11 = getA();
            double b12 = getB();
            double b22 = getC();
            double b13 = getD();
            double b23 = getE();
            double b33 = getF();
            
            //alpha = horizontal focal distance
            //beta = vertical focal distance
            //gamma = skewness
            //u0 = horizontal principal point
            //v0 = vertical principal point
            //lambda = scale
            double v0 = (b12 * b13 - b11 * b23) /
                    (b11 * b22 - b12 * b12);
            if (Double.isNaN(v0) || Double.isInfinite(v0)) {
                throw new InvalidPinholeCameraIntrinsicParametersException();
            }
            
            //lambda is an arbitrary scale, because conics are defined up to 
            //scale and we are obtaining intrinsic parameters from IAC (Image
            //of Absolute Conic)
            double lambda = b33 - (b13 * b13 + v0 * (b12 * b13 - b11 * b23)) / 
                    b11;
            //fix sign of lambda so that squared roots are always positive
            lambda *= Math.signum(lambda) * Math.signum(b11);
            
            if (Double.isNaN(lambda) || Double.isInfinite(lambda)) {
                throw new InvalidPinholeCameraIntrinsicParametersException();
            }
            
            double alpha = Math.sqrt(lambda / b11);
            if (Double.isNaN(alpha) || Double.isInfinite(alpha)) {
                throw new InvalidPinholeCameraIntrinsicParametersException();
            }
            
            double beta = Math.sqrt(lambda * b11 / (b11 * b22 - b12 * b12));
            if (Double.isNaN(beta) || Double.isInfinite(beta)) {
                throw new InvalidPinholeCameraIntrinsicParametersException();
            }
            
            double gamma = -b12 * alpha * alpha * beta / lambda;
            if (Double.isNaN(gamma) || Double.isInfinite(gamma)) {
                throw new InvalidPinholeCameraIntrinsicParametersException();
            }
            
            double u0 = gamma * v0 / beta - b13 * alpha * alpha / lambda;
            if (Double.isNaN(u0) || Double.isInfinite(u0)) {
                throw new InvalidPinholeCameraIntrinsicParametersException();
            }
            
            return new PinholeCameraIntrinsicParameters(alpha, beta, u0, v0, 
                    gamma);
        } catch (ArithmeticException e) {
            throw new InvalidPinholeCameraIntrinsicParametersException(e);
        }
    }
    
    /**
     * Assuming that we are working in a metric stratum this method obtains the
     * internal parameters of a pinhole camera by means of cholesky 
     * decomposition.
     * @return the internal parameters of a pinhole camera.
     * @throws InvalidPinholeCameraIntrinsicParametersException if pinhole
     * camera intrinsic parameters cannot be obtained from this conic instance.
     */        
    public PinholeCameraIntrinsicParameters getIntrinsicParametersCholesky()
            throws InvalidPinholeCameraIntrinsicParametersException {
        try {
            normalize();
        
            Matrix m = asMatrix();
            CholeskyDecomposer decomposer = new CholeskyDecomposer(m);
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
