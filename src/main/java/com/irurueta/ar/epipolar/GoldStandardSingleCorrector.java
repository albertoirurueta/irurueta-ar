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
import com.irurueta.algebra.Complex;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.NumericalException;
import com.irurueta.numerical.roots.LaguerrePolynomialRootsEstimator;

/**
 * Fixes a single matched pair of points so that they perfectly follow a given
 * epipolar geometry using the Gold Standard method, which is capable to 
 * completely remove errors assuming their gaussianity.
 * When matching points typically the matching precission is about 1 pixel, 
 * however this makes that matched points under a given epipolar geometry (i.e.
 * fundamental or essential matrix), do not lie perfectly on the corresponding
 * epipolar plane or epipolar lines.
 * The consequence is that triangularization of these matches will fail or 
 * produce innaccurate results. 
 * By fixing matched points using a corrector following a given epipolar 
 * geometry, this effect is alleviated.
 * This corrector uses the Gold Standard method, which is more expensive to 
 * compute than the Sampson approximation, but is capable to remove larger 
 * errors assuming their gaussianity. Contrary to the Sampson corrector, the
 * Gold Standard method might fail in some situations, while in those cases
 * probably the Sampson corrector produces wrong results without failing.
 */
public class GoldStandardSingleCorrector extends SingleCorrector {
        
    /**
     * A tiny value.
     */
    private static final double EPS = 1e-6;
    
    /**
     * Constructor.
     */
    public GoldStandardSingleCorrector() {
        super();
    }
    
    /**
     * Constructor.
     * @param fundamentalMatrix fundamental matrix defining the epipolar
     * geometry.
     */
    public GoldStandardSingleCorrector(FundamentalMatrix fundamentalMatrix) {
        super(fundamentalMatrix);
    }
    
    /**
     * Constructor.
     * @param leftPoint matched point on left view to be corrected.
     * @param rightPoint matched point on right view to be corrected.
     */
    public GoldStandardSingleCorrector(Point2D leftPoint, Point2D rightPoint) {
        super(leftPoint, rightPoint);
    }
    
    /**
     * Constructor.
     * @param leftPoint matched point on left view to be corrected.
     * @param rightPoint matched point on right view to be corrected.
     * @param fundamentalMatrix fundamental matrix defining an epipolar geometry.
     */
    public GoldStandardSingleCorrector(Point2D leftPoint, Point2D rightPoint,
            FundamentalMatrix fundamentalMatrix) {
        super(leftPoint, rightPoint, fundamentalMatrix);
    }
    
    /**
     * Corrects the pair of provided matched points to be corrected.
     * @throws NotReadyException if this instance is not ready (either points or
     * fundamntal matrix has not been provided yet).
     * @throws CorrectionException if correction fails.
     */
    @Override
    public void correct() throws NotReadyException, CorrectionException {
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        mLeftCorrectedPoint = Point2D.create(
                CoordinatesType.HOMOGENEOUS_COORDINATES);
        mRightCorrectedPoint = Point2D.create(
                CoordinatesType.HOMOGENEOUS_COORDINATES);        
        correct(mLeftPoint, mRightPoint, mFundamentalMatrix, 
                mLeftCorrectedPoint, mRightCorrectedPoint);
    }
    
    /**
     * Gets type of correction being used.
     * @return type of correction.
     */
    @Override
    public CorrectorType getType() {
        return CorrectorType.GOLD_STANDARD;
    }    
    
    /**
     * Corrects the pair of provided matched points to be corrected using
     * provided fundamental matrix and stores the corrected points into provided
     * instances.
     * @param leftPoint point on left view to be corrected.
     * @param rightPoint point on right view to be corrected.
     * @param fundamentalMatrix fundamental matrix defining the epipolar
     * geometry.
     * @param correctedLeftPoint point on left view after correction.
     * @param correctedRightPoint point on right view after correction.
     * @throws NotReadyException if provided fundamental matrix is not ready.
     * @throws CorrectionException if correction fails.
     */
    public static void correct(Point2D leftPoint, Point2D rightPoint,
            FundamentalMatrix fundamentalMatrix, Point2D correctedLeftPoint,
            Point2D correctedRightPoint) throws NotReadyException, 
            CorrectionException {
        
        
        //normalize to increase accuracy
        leftPoint.normalize();
        rightPoint.normalize();
        fundamentalMatrix.normalize();
        
        try {
            //create transformations to move left and right points to the origin 
            //(0,0,1)
            Matrix leftTranslationMatrix = Matrix.identity(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
            //add terms to move left point to origin
            leftTranslationMatrix.setElementAt(0, 2, -leftPoint.getInhomX());
            leftTranslationMatrix.setElementAt(1, 2, -leftPoint.getInhomY());
            
            //create inverse transformation
            Matrix invLeftTranslationMatrix = Matrix.identity(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
            //add terms to obtain inverse transformation
            invLeftTranslationMatrix.setElementAt(0, 2, leftPoint.getInhomX());
            invLeftTranslationMatrix.setElementAt(1, 2, leftPoint.getInhomY());
        
            Matrix rightTranslationMatrix = Matrix.identity(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
            //add terms to move right point to origin
            rightTranslationMatrix.setElementAt(0, 2, -rightPoint.getInhomX());
            rightTranslationMatrix.setElementAt(1, 2, -rightPoint.getInhomY());
        
            //create inverse and transposed transformation
            Matrix invTransRightTranslationMatrix = Matrix.identity(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
            //add terms to obtain inverse transformation
            invTransRightTranslationMatrix.setElementAt(2, 0, 
                    rightPoint.getInhomX());
            invTransRightTranslationMatrix.setElementAt(2, 1, 
                    rightPoint.getInhomY());
            
            //because points have been transformed using left and right 
            //translations (T1 and T2 respectively), then fundamental matrix 
            //becomes (T2^-1)'*F*T1^-1
            Matrix fundInternalMatrix = fundamentalMatrix.getInternalMatrix();
            fundInternalMatrix.multiply(invLeftTranslationMatrix);
            invTransRightTranslationMatrix.multiply(fundInternalMatrix);
            Matrix transformedFundInternalMatrix = 
                    invTransRightTranslationMatrix;
            
            //compute transformations to rotate epipoles of transformed 
            //fundamental matrix so that they are located at e1 = (1,0,f1) and 
            //e2 = (1,0,f2)
            FundamentalMatrix transformedFundamentalMatrix = 
                    new FundamentalMatrix(transformedFundInternalMatrix);
            transformedFundamentalMatrix.computeEpipoles();
            Point2D leftEpipole = transformedFundamentalMatrix.
                    getLeftEpipole();
            Point2D rightEpipole = transformedFundamentalMatrix.
                    getRightEpipole();
            
            //normalize so that leftEpipole.x^2 + leftEpipole.y^2 = 1 and
            //rightEpipole.x^2 + rightEpipole.y^2 = 1, that way transformations
            //become rotations
            double normLeftEpipole = Math.pow(leftEpipole.getHomX(), 2.0) +
                    Math.pow(leftEpipole.getHomY(), 2.0);
            double normRightEpipole = Math.pow(rightEpipole.getHomX(), 2.0) +
                    Math.pow(rightEpipole.getHomY(), 2.0);
            
            leftEpipole.setHomogeneousCoordinates(
                    leftEpipole.getHomX() / normLeftEpipole, 
                    leftEpipole.getHomY() / normLeftEpipole, 
                    leftEpipole.getHomW() / normLeftEpipole);
            rightEpipole.setHomogeneousCoordinates(
                    rightEpipole.getHomX() / normRightEpipole, 
                    rightEpipole.getHomY() / normRightEpipole, 
                    rightEpipole.getHomW() / normRightEpipole);
            
            Matrix leftRotationMatrix = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH,
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
            leftRotationMatrix.setElementAt(0, 0, leftEpipole.getHomX());
            leftRotationMatrix.setElementAt(1, 0, -leftEpipole.getHomY());
            leftRotationMatrix.setElementAt(0, 1, leftEpipole.getHomY());
            leftRotationMatrix.setElementAt(1, 1, leftEpipole.getHomX());
            leftRotationMatrix.setElementAt(2, 2, 1.0);
            
            //the inverse of a rotation is its transpose
            Matrix invLeftRotationMatrix = 
                    leftRotationMatrix.transposeAndReturnNew();
            
            Matrix rightRotationMatrix = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH,
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
            rightRotationMatrix.setElementAt(0, 0, rightEpipole.getHomX());
            rightRotationMatrix.setElementAt(1, 0, -rightEpipole.getHomY());
            rightRotationMatrix.setElementAt(0, 1, rightEpipole.getHomY());
            rightRotationMatrix.setElementAt(1, 1, rightEpipole.getHomX());
            rightRotationMatrix.setElementAt(2, 2, 1.0);
            
            //again the inverse of rotation is its transpose
            Matrix invRightRotationMatrix = 
                    rightRotationMatrix.transposeAndReturnNew();
                        
            //and so the fundamental matrix now becomes: R2*(T2^-1)'*F*T1^-1*R1',
            //where the middle matrices correspond to previous transformation
            fundInternalMatrix = transformedFundInternalMatrix;
            fundInternalMatrix.multiply(invLeftRotationMatrix);
            rightRotationMatrix.multiply(fundInternalMatrix);
            transformedFundInternalMatrix = rightRotationMatrix;
            
            //where F is a transformed fundamental matrix that has the following 
            //form:
            //[f1*f2*d  -f2*c   -f2*d   ]
            //[-f1*b    a       b       ]
            //[-f1*d    c       d       ]
            
            //hence:
            double a = transformedFundInternalMatrix.getElementAt(1, 1);
            double b = transformedFundInternalMatrix.getElementAt(1, 2);
            double c = transformedFundInternalMatrix.getElementAt(2, 1);
            double d = transformedFundInternalMatrix.getElementAt(2, 2);
            double f1, f2;
            if (Math.abs(b) > Math.abs(d)) {
                f1 = -transformedFundInternalMatrix.getElementAt(1, 0) / b;
            } else {
                f1 = -transformedFundInternalMatrix.getElementAt(2, 0) / d;
            }
            if (Math.abs(c) > Math.abs(d)) {
                f2 = -transformedFundInternalMatrix.getElementAt(0, 1) / c;
            } else {
                f2 = -transformedFundInternalMatrix.getElementAt(0, 2) / d;
            }       
            
            //Hence the polynomial of degree 6 to solve corresponding to the derivative of s(t) is:
            //g(t) = A*t^6 + B*t^5 + C*t^4 + D*t^3 + E*t^2 + F*t + G
            //where:
            double A = -(Math.pow(a, 2.0)*d - a*b*c)*c*Math.pow(f1, 4.0);
            double B = Math.pow((Math.pow(a, 2.0) + Math.pow(c, 2.0)*Math.pow(f2, 2.0)), 2.0) -
                    ((Math.pow(a, 2)*d - a*b*c)*d + (a*b*d - Math.pow(b, 2.0)*c)*c)*Math.pow(f1, 4.0);
            double C = (4.0*(Math.pow(a, 2.0) + Math.pow(c, 2.0) * Math.pow(f2, 2.0))*(a*b + c*d*Math.pow(f2, 2.0)) -
                    (2.0*(Math.pow(a, 2.0)*d - a*b*c)*c*Math.pow(f1, 2.0) + (a*b*d - Math.pow(b, 2.0)*c)*d*Math.pow(f1, 4.0)));
            double D = 2.0*((Math.pow(a, 2.0) + Math.pow(c, 2.0)*Math.pow(f2, 2.0))*(Math.pow(b, 2.0) + Math.pow(d, 2.0)*Math.pow(f2, 2.0)) +
                    2.0*Math.pow(a*b + c*d*Math.pow(f2, 2.0), 2.0) -
                    ((Math.pow(a, 2.0)*d - a*b*c)*d + (a*b*d - Math.pow(b, 2.0)*c)*c)*Math.pow(f1, 2.0));
            double E = (4.0*(a*b + c*d*Math.pow(f2, 2.0))*(Math.pow(b, 2.0) + Math.pow(d, 2.0)*Math.pow(f2, 2.0)) - 
                    ((Math.pow(a, 2.0)*d - a*b*c)*c + 2.0*(a*b*d - Math.pow(b, 2.0)*c)*d*Math.pow(f1, 2.0)));
            double F = (Math.pow(Math.pow(b, 2.0) + Math.pow(d, 2.0)*Math.pow(f2, 2.0), 2.0) -
                    ((Math.pow(a, 2.0)*d - a*b*c)*d + (a*b*d - Math.pow(b, 2.0)*c)*c));
            double G = -(a*b*d - Math.pow(b, 2.0)*c)*d;
            
            Complex complexA = new Complex(A);
            Complex complexB = new Complex(B);
            Complex complexC = new Complex(C);
            Complex complexD = new Complex(D);
            Complex complexE = new Complex(E);
            Complex complexF = new Complex(F);
            Complex complexG = new Complex(G);
            Complex [] polyParams = new Complex[]{ complexG, complexF, complexE, 
                complexD, complexC, complexB, complexA };
            LaguerrePolynomialRootsEstimator rootEstimator = 
                    new LaguerrePolynomialRootsEstimator(polyParams);
            rootEstimator.estimate();
            Complex[] roots = rootEstimator.getRoots();
            
            //evaluate polynomial s(t) on each root of its derivative to obtain
            //the global minima (we discard non real roots)
            double minimum = Double.MAX_VALUE;
            double value;
            Complex bestRoot = null;
            for (Complex root : roots) {
                //skip solutions having an imaginary part
                if (Math.abs(root.getImaginary()) > EPS) {
                    continue;
                }
                
                value = s(root.getReal(), a, b, c, d, f1, f2);
                if (value < minimum) {
                    minimum = value;
                    bestRoot = root;
                }
            }
            
            if (bestRoot == null) {
                throw new CorrectionException();
            }
            
            double tmin = bestRoot.getReal();
            
            //and so, the transformed left point is (-tmin^2*f1, tmin, 1 + (tmin*f1)^2)
            Point2D transformedLeftPoint = new HomogeneousPoint2D(
                    -Math.pow(tmin,2.0)*f1, tmin, 1.0 + Math.pow(tmin*f1, 2.0));
            //and the right point is (f2*(c*tmin+d)^2, -(a*tmin+b)*(c*tmin+d), (a*t+b)^2 + f2^2*(c*t+d)^2)
            Point2D transformedRightPoint = new HomogeneousPoint2D(
                    f2*Math.pow(c*tmin + d, 2.0), 
                    -(a*tmin + b)*(c*tmin + d),
                    Math.pow(a*tmin + b, 2.0) + Math.pow(f2, 2.0)*Math.pow(c*tmin + d, 2.0));
            
            //undo translation and rotation transformations
            //correctedLeftPoint = T1^-1*R1'*transformedLeftPoint
            //correctedRightPoint = T2^-1*R2'*transformedRightPoint
            
            //inverse left transformation
            invLeftTranslationMatrix = Matrix.identity(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
            //add terms to obtain inverse transformation
            invLeftTranslationMatrix.setElementAt(0, 2, leftPoint.getInhomX());
            invLeftTranslationMatrix.setElementAt(1, 2, leftPoint.getInhomY());
        
            //inverse right translation transformation
            Matrix invRightTranslationMatrix = Matrix.identity(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
            //add terms to obtain inverse transformation
            invRightTranslationMatrix.setElementAt(0, 2, rightPoint.getInhomX());
            invRightTranslationMatrix.setElementAt(1, 2, rightPoint.getInhomY());            
            
            invLeftTranslationMatrix.multiply(invLeftRotationMatrix);
            Matrix invLeftTransformationMatrix = invLeftTranslationMatrix;
            invRightTranslationMatrix.multiply(invRightRotationMatrix);
            //noinspection all
            Matrix invRightTransformationMatrix = invRightTranslationMatrix;
            
            ProjectiveTransformation2D invLeftTransformation = 
                    new ProjectiveTransformation2D(
                    invLeftTransformationMatrix);
            ProjectiveTransformation2D invRightTransformation =
                    new ProjectiveTransformation2D(
                    invRightTransformationMatrix);
            
            invLeftTransformation.transform(transformedLeftPoint, 
                    correctedLeftPoint);
            invRightTransformation.transform(transformedRightPoint, 
                    correctedRightPoint);
            
        } catch (AlgebraException | GeometryException | NumericalException e) {
            throw new CorrectionException(e);
        }
    }
    
    /**
     * Evaluates polynomial to be minimized in order to find best corrected 
     * points.
     * @param t polynomial variable.
     * @param a a value for transformed fundamental matrix.
     * @param b b value for transformed fundamental matrix.
     * @param c c value for transformed fundamental matrix.
     * @param d d value for transformed fundamental matrix.
     * @param f1 f1 value for transformed fundamental matrix.
     * @param f2 f2 value for transformed fundamental matrix.
     * @return evaluated polynomial value.
     */
    private static double s(double t, double a, double b, double c, double d, 
            double f1, double f2) {
        return Math.pow(t, 2.0) / (1 + Math.pow(t*f1,2.0)) + 
                Math.pow(c*t + d, 2.0) / (Math.pow(a*t + b, 2.0) + 
                Math.pow(f2, 2.0)*Math.pow(c*t+d, 2.0));
    }
}
