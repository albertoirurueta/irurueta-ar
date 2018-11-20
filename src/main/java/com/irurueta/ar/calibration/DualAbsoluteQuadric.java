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
package com.irurueta.ar.calibration;

import com.irurueta.algebra.*;
import com.irurueta.geometry.DualQuadric;
import com.irurueta.geometry.NonSymmetricMatrixException;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.numerical.NumericalException;
import com.irurueta.numerical.roots.FirstDegreePolynomialRootsEstimator;
import com.irurueta.numerical.roots.SecondDegreePolynomialRootsEstimator;

import java.io.Serializable;

/**
 * The dual absolute quadric is the dual quadric tangent to the plane at 
 * infinity.
 * The absolute quadric (which is its inverse) contains all planes located
 * at infinity (x,y,z,0), hence the absolute quadric fulfills 
 * (x,y,z,0)'*Q*(x,y,z,0).
 * Consequently, the dual absolute quadric fulfills P*Q^-1*P, where P is the
 * plane at infinity (0,0,0,1) in the metric stratum.
 * This means that in the metric stratus the dual absolute quadric is equal
 * (up to scale) to:
 *      [1  0   0   0]
 * Q =  [0  1   0   0]
 *      [0  0   1   0]
 *      [0  0   0   0]
 */
@SuppressWarnings("WeakerAccess")
public class DualAbsoluteQuadric extends DualQuadric implements Serializable {

    /**
     * Constructor.
     * Initializes the Dual Absolute Quadric assuming metric stratum, where it
     * is equal to the identity except for the last element which is zero.
     */
    public DualAbsoluteQuadric() {
        super(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
    
    /**
     * Constructor of this class. This constructor accepts every parameter
     * describing a dual quadric (parameters a, b, c, d, e, f, g, h, i, j).
     * @param a Parameter A of the quadric.
     * @param b Parameter B of the quadric.
     * @param c Parameter C of the quadric.
     * @param d Parameter D of the quadric.
     * @param e Parameter E of the quadric.
     * @param f Parameter F of the quadric.
     * @param g Parameter G of the quadric.
     * @param h Parameter H of the quadric.
     * @param i Parameter I of the quadric.
     * @param j Parameter J of the quadric.
     */
    public DualAbsoluteQuadric(double a, double b, double c, double d, double e, 
            double f, double g, double h, double i, double j) {
        super(a, b, c, d, e, f, g, h, i, j);
    }   
    
    /**
     * Constructor from provided dual image of absolute conic and plane at
     * infinity on an arbitrary projective stratum.
     * @param diac dual image of absolute conic in an arbitrary projective 
     * stratum.
     * @param planeAtInfinity plane at infinity in an arbitrary projective 
     * stratum.
     */
    public DualAbsoluteQuadric(DualImageOfAbsoluteConic diac, 
            Plane planeAtInfinity) {
        super();
        setDualImageOfAbsoluteConicAndPlaneAtInfinity(diac, planeAtInfinity);
    }
    
    /**
     * Constructor using provided dual image of absolute conic on an arbitrary
     * affine stratum while keeping the plane at infinity typically used in
     * a metric stratum (0,0,0,1).
     * @param diac dual image of absolute conic.
     */
    public DualAbsoluteQuadric(DualImageOfAbsoluteConic diac) {
        super();
        
        diac.normalize();
        
        double a = diac.getA();
        double b = diac.getC();
        double c = diac.getF();
        double d = diac.getB();
        double e = diac.getE();
        double f = diac.getD();        
        setParameters(a, b, c, d, e, f, 0.0, 0.0, 0.0, 0.0);
    }
    
    /**
     * Constructor using provided plane at infinity while using the unitary
     * dual image of absolute conic (the identity).
     * @param planeAtInfinity plane at infinity.
     */
    public DualAbsoluteQuadric(Plane planeAtInfinity) {
        super();
        
        double planeA = planeAtInfinity.getA();
        double planeB = planeAtInfinity.getB();
        double planeC = planeAtInfinity.getC();
        double planeD = planeAtInfinity.getD();
        
        //normalize plane components so that last one is the unit
        planeA /= planeD;
        planeB /= planeD;
        planeC /= planeD;
        
        double g = -planeA;        
        double h = -planeB;        
        double i = -planeC;
        double j = planeA * planeA + planeB * planeB + planeC * planeC;
        
        setParameters(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, g, h, i, j);        
    }
    
    /**
     * Constructor of the Dual Absolute Quadric in an arbitrary projective 
     * stratum using a transformation from metric stratum to such projective
     * space.
     * @param metricToProjectiveTransformation transformation from metric to
     * a projective space.
     * @throws InvalidTransformationException if provided transformation is 
     * numerically unstable.
     */
    public DualAbsoluteQuadric(
            ProjectiveTransformation3D metricToProjectiveTransformation) 
            throws InvalidTransformationException {
        setMetricToProjectiveTransformation(metricToProjectiveTransformation);
    }
    
    /**
     * Sets this dual absolute quadric from provided metric to projective space
     * transformation.
     * @param metricToProjectiveTransformation transformation from metric to a
     * projective space.
     * @throws InvalidTransformationException if provided transformation is 
     * numerically unstable.
     */
    public final void setMetricToProjectiveTransformation(
            ProjectiveTransformation3D metricToProjectiveTransformation) 
            throws InvalidTransformationException {
        metricToProjectiveTransformation.normalize();
        
        try {
            //transformation
            Matrix t = metricToProjectiveTransformation.asMatrix();
        
            //identity except for the last element. This is the DAQ in metric 
            //stratum
            Matrix i = Matrix.identity(BASEQUADRIC_MATRIX_ROW_SIZE, 
                    BASEQUADRIC_MATRIX_COLUMN_SIZE);
            i.setElementAt(3, 3, 0.0);
        
            //transformation transposed
            Matrix tt = t.transposeAndReturnNew();
        
            //make product t * i * tt
            t.multiply(i);
            t.multiply(tt);
        
            setParameters(t);
        } catch (NonSymmetricMatrixException e) {
            throw new InvalidTransformationException(e);
        } catch (WrongSizeException ignore) {
            /* never thrown */
        }        
    }
    
    /**
     * Obtains the metric to projective stratum transformation defining this
     * Dual Absolute Quadric.
     * @return a new metric to projective space transformation.
     * @throws InvalidTransformationException if transformation cannot be 
     * determined because dual absolute quadric is numerically unstable.
     */
    public ProjectiveTransformation3D getMetricToProjectiveTransformation() 
            throws InvalidTransformationException {
        ProjectiveTransformation3D result = new ProjectiveTransformation3D();
        getMetricToProjectiveTransformation(result);
        return result;
    }
    
    /**
     * Obtains the metric to projective stratum transformation defining this
     * Dual Absolute Quadric.
     * @param result instance where metric to projective space transformation 
     * will be stored.
     * @throws InvalidTransformationException if transformation cannot be 
     * determined because dual absolute quadric is numerically unstable.
     */
    public void getMetricToProjectiveTransformation(
            ProjectiveTransformation3D result) 
            throws InvalidTransformationException {
        //DAQ can be decomposed as DAQ = H * I * H^t, where
        //I =   [1  0   0   0]
        //      [0  1   0   0]
        //      [0  0   1   0]
        //      [0  0   0   0]
        //Hence I can be seen as the eigen values of DAQ's eigen decomposition
        //and H are the eigenvectors.
        //From this decomposition, it can be seen that:
        //- DAQ is singular (has rank 3)
        //- DAQ is symmetric positive definite (or negative depending on scale 
        //sign, but all eigen values have the same sign).
        //We know that symmetric positive definite matrices have square roots
        //such as:
        //DAQ = M*M^t
        //Hence the SVD of the square root is equal to:
        //M = U*S*V^t
        //And consequently DAQ is:
        //DAQ = (U*S*V^t)*(U*S*V^t)^t = U*S*V^t*V*S*U^t = U*S*S*U^t
        //where S is diagonal and contains the singular values of M
        //and U are the singular vectors of M
        //Since S is diagonal, then S*S contains the squared singular vlaues on
        //its diagonal and matrix S*S can be seen as the eigen values of DAQ,
        //while U are the eigen vectors of DAQ
        //Since we don't care about scale, we can normalize the eigen values of 
        //DAQ, and transformation H is equal to matrix U (up to scale)
        
        try {
            Matrix daqMatrix = asMatrix();
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    daqMatrix);
            decomposer.decompose();
            //since daq matrix will always be symmetric U = V
            Matrix u = decomposer.getU();
            //we need to undo the effect of possible different singular values
            //because we want H * I * H^t, so that middle matrix is the identity
            //except for the last element which is zero.
            //For that reason we multiply each column of U by the square root 
            //of each singular value
            double[] w = decomposer.getSingularValues();
            for (int i = 0; i < BASEQUADRIC_MATRIX_COLUMN_SIZE - 1; i++) {
                double scalar = Math.sqrt(w[i]);
                for (int j = 0; j < BASEQUADRIC_MATRIX_ROW_SIZE; j++) {
                    u.setElementAt(j, i, scalar * u.getElementAt(j, i));
                }
            }
            result.setT(u);
        } catch (AlgebraException e) {
            throw new InvalidTransformationException(e);
        }
    }
    
    /**
     * Sets this dual absolute quadric from provided dual image of absolute 
     * conic and plane at infinity on an arbitrary projective stratum.
     * @param diac dual image of absolute conic to be set.
     * @param planeAtInfinity plane at infinity to be set.
     */
    public final void setDualImageOfAbsoluteConicAndPlaneAtInfinity(
            DualImageOfAbsoluteConic diac, Plane planeAtInfinity) {
        
        double planeA = planeAtInfinity.getA();
        double planeB = planeAtInfinity.getB();
        double planeC = planeAtInfinity.getC();
        double planeD = planeAtInfinity.getD();
        
        //normalize plane components so that last one is the unit
        planeA /= planeD;
        planeB /= planeD;
        planeC /= planeD;
        
        double a = diac.getA();
        double b = diac.getC();
        double c = diac.getF();
        double d = diac.getB();
        double e = diac.getE();
        double f = diac.getD();

        double g = -(diac.getA() * planeA + diac.getB() * planeB +
                diac.getD() * planeC);
        
        double h = -(diac.getB() * planeA + diac.getC() * planeB +
                diac.getE() * planeC);
        
        double i = -(diac.getD() * planeA + diac.getE() * planeB +
                diac.getF() * planeC);
        
        double j = -(planeA * g + planeB * h + planeC * i);
        
        setParameters(a, b, c, d, e, f, g, h, i, j);        
    }
    
    /**
     * Gets dual image of absolute conic associated to this dual absolute 
     * quadric in an arbitrary projective stratum.
     * @return dual image of absolute conic associated to this dual absolute 
     * quadric.
     */
    public DualImageOfAbsoluteConic getDualImageOfAbsoluteConic() {
        DualImageOfAbsoluteConic result = new DualImageOfAbsoluteConic();
        getDualImageOfAbsoluteConic(result);
        return result;
    }
    
    /**
     * Gets dual iamge of absolute conic associated to this dual absolute 
     * quadric in an arbitrary projective stratum and stores the result into 
     * provided instance.
     * @param result instance where dual image of absolute conic will be stored.
     */
    public void getDualImageOfAbsoluteConic(DualImageOfAbsoluteConic result) {
        double a = getA();
        double b = getD();
        double c = getB();
        double d = getF();
        double e = getE();
        double f = getC();
        result.setParameters(a, b, c, d, e, f);
    }
    
    /**
     * Sets dual image of absolute conic while keeping current plane at infinity
     * in an arbitrary projective stratum.
     * @param diac dual image of absolute conic to be set.
     * @throws InvalidPlaneAtInfinityException if plane at infinity to be 
     * preserved when setting DIAC cannot be determined.
     */
    public final void setDualImageOfAbsoluteConic(DualImageOfAbsoluteConic diac) 
            throws InvalidPlaneAtInfinityException {
        Plane planeAtInfinity = getPlaneAtInfinity();
        setDualImageOfAbsoluteConicAndPlaneAtInfinity(diac, planeAtInfinity);
    }
    
    /**
     * Gets plane at infinity associated to this dual absolute quadric in an
     * arbitrary projective stratum.
     * @return plane at infinity associated to this dual absolute quadric.
     * @throws InvalidPlaneAtInfinityException if plane at infinity cannot be
     * determined.
     */
    public Plane getPlaneAtInfinity() throws InvalidPlaneAtInfinityException {
        Plane result = new Plane();
        getPlaneAtInfinity(result);
        return result;
    }
    
    /**
     * Gets plane at infinity associated to this dual absolute quadric in an
     * arbitrary projective stratum.
     * @param result instance where plane at infinity will be stored.
     * @throws InvalidPlaneAtInfinityException if plane at infinity cannot be
     * determined.
     */
    public void getPlaneAtInfinity(Plane result) 
            throws InvalidPlaneAtInfinityException {
        try {
            DualImageOfAbsoluteConic diac = getDualImageOfAbsoluteConic();
            Matrix m = diac.asMatrix();
            Utils.inverse(m, m);
            
            double g = getG();
            double h = getH();
            double i = getI();
            
            double planeA = - m.getElementAt(0, 0) * g 
                    - m.getElementAt(0, 1) * h
                    - m.getElementAt(0, 2) * i;
            
            double planeB = - m.getElementAt(1, 0) * g
                    - m.getElementAt(1, 1) * h
                    - m.getElementAt(1, 2) * i;
            
            double planeC = - m.getElementAt(2, 0) * g
                    - m.getElementAt(2, 1) * h
                    - m.getElementAt(2, 2) * i;
            
            //plane at infinity must be tangent to dual absolute quadric, hence
            //P^T*Q^-1*P = 0
            //[planeA planeB planeC planeD]*[a  d   f   g][planeA]
            //                              [d  b   e   h][planeB] = 0
            //                              [f  e   c   i][planeC]
            //                              [g  h   i   j][planeD]
            
            //[planeA planeB planeC planeD]*[a*planeA + d*planeB + f*planeC + g*planeD]
            //                              [d*planeA + b*planeB + e*planeC + h*planeD] = 0
            //                              [f*planeA + e*planeB + c*planeC + i*planeD]
            //                              [g*planeA + h*planeB + i*planeC + j*planeD]
            
            //planeA*(a*planeA + d*planeB + f*planeC) + 
            //planeB*(d*planeA + b*planeB + e*planeC) + 
            //planeC*(f*planeA + e*planeB + c*planeC) + 
            //2*planeD*(g*planeA + h*planeB + i*planeC) + j*planeD*planeD = 0
            
            //hence we create the 2nd degree polynomial shown above and solve 
            //planeD value
            double a = getA();
            double b = getB();
            double c = getC();
            double d = getD();
            double e = getE();
            double f = getF();
            double j = getJ();
            double[] polyParams = new double[]{
                planeA*(a*planeA + d*planeB + f*planeC) + 
                    planeB*(d*planeA + b*planeB + e*planeC) + 
                    planeC*(f*planeA + e*planeB + c*planeC),
                2.0*(g*planeA + h*planeB + i*planeC),
                j
            };
            
            double planeD;
            if (SecondDegreePolynomialRootsEstimator.isSecondDegree(polyParams)) {
                //second degree
                SecondDegreePolynomialRootsEstimator estimator = 
                    new SecondDegreePolynomialRootsEstimator(polyParams);
            
            
                boolean hasDoubleRoot = estimator.hasDoubleRoot();
                
                //a double REAL root (same root happening twice) must be present 
                //because only one plane at infinity should be defined
                if (!hasDoubleRoot) {
                    throw new InvalidPlaneAtInfinityException(
                            "more than one possible solution");
                }
            
                estimator.estimate();
                
                Complex[] roots = estimator.getRoots();
                planeD = roots[0].getReal();
            } else {
                //polynomial is not second degree, attempt to solve as 1 degree
                if (FirstDegreePolynomialRootsEstimator.isFirstDegree(polyParams)) {
                    //first degree
                    FirstDegreePolynomialRootsEstimator estimatorFirst =
                            new FirstDegreePolynomialRootsEstimator(
                                    new double[]{polyParams[0], polyParams[1]});
                    
                    estimatorFirst.estimate();
                    
                    Complex[] roots = estimatorFirst.getRoots();
                    planeD = roots[0].getReal();
                } else {
                    //degenerate polynomial (i.e. metric stratum)
                    planeD = 1.0;            
                }
            }
            result.setParameters(planeA, planeB, planeC, planeD);
                
        } catch (AlgebraException | NumericalException e) {
            throw new InvalidPlaneAtInfinityException(e);
        }
    }
    
    /**
     * Sets provided plane at infinity at an arbitrary projective stratum while
     * keeping current dual image of absolute conic.
     * @param planeAtInfinity plane at infinity to be set.
     */
    public final void setPlaneAtInfinity(Plane planeAtInfinity) {
        DualImageOfAbsoluteConic diac = getDualImageOfAbsoluteConic();
        setDualImageOfAbsoluteConicAndPlaneAtInfinity(diac, planeAtInfinity);
    }
}
