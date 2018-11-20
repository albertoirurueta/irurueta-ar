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
package com.irurueta.ar.epipolar.estimators;

import com.irurueta.algebra.*;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidFundamentalMatrixException;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NormalizerException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.Point2DNormalizer;
import com.irurueta.numerical.NumericalException;
import com.irurueta.numerical.roots.FirstDegreePolynomialRootsEstimator;
import com.irurueta.numerical.roots.SecondDegreePolynomialRootsEstimator;
import com.irurueta.numerical.roots.ThirdDegreePolynomialRootsEstimator;

import java.util.ArrayList;
import java.util.List;

/**
 * Non-robust fundamental matrix estimator that uses 7 matched 2D points on
 * left and right views.
 * Although this algorithm requires one less point than the 8 points algorithm,
 * it might return up to three different fundamental matrices that must be 
 * later checked (i.e. using a robust algorithm to maximize inliers) so that
 * the correct solution is picked.
 * If the correct solution is found, it typically obtains results with higher
 * accuracy than the 8 points algorithm, because rank-2 is not approximated
 * using SVD, and rather it is accurately enforced.
 */
@SuppressWarnings("WeakerAccess")
public class SevenPointsFundamentalMatrixEstimator extends 
        FundamentalMatrixEstimator {
    
    /**
     * Constant indicating that by default an LMSE solution is not allowed.
     */
    public static final boolean DEFAULT_ALLOW_LMSE_SOLUTION = false;
    
    /**
     * Minimum number of matched 2D points to start the estimation.
     */
    public static final int MIN_REQUIRED_POINTS = 7;
    
    /**
     * Indicates if by default provided point correspondences are normalized to
     * increase the accuracy of the estimation.
     */
    public static final boolean DEFAULT_NORMALIZE_POINT_CORRESPONDENCES = true;        
    
    /**
     * Tiniest value closest to zero.
     */
    public static final double EPS = Double.MIN_VALUE;
    
    /**
     * Indicates whether an LMSE (Least Mean Square Error) solution is allowed
     * or not. When an LMSE solution is allowed, more than 7 matched points can
     * be used for fundamental matrix estimation. If LMSE solution is not 
     * allowed then only the 7 former matched points will be taken into account.
     */
    private boolean mAllowLMSESolution;
    
    /**
     * Indicates whether provided matched 2D points must be normalized to 
     * increase the accuracy of the estimation.
     */
    private boolean mNormalizePoints;
    
    /**
     * Constructor.
     */
    public SevenPointsFundamentalMatrixEstimator() {
        super();
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
        mNormalizePoints = DEFAULT_NORMALIZE_POINT_CORRESPONDENCES;
    }
    
    /**
     * Constructor with matched 2D points.
     * @param leftPoints 2D points on left view.
     * @param rightPoints 2D points on right view.
     * @throws IllegalArgumentException if provided list of points do not
     * have the same length.
     */
    public SevenPointsFundamentalMatrixEstimator(List<Point2D> leftPoints, 
            List<Point2D> rightPoints) throws IllegalArgumentException {
        super(leftPoints, rightPoints);
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
        mNormalizePoints = DEFAULT_NORMALIZE_POINT_CORRESPONDENCES;
    }

    /**
     * Returns boolean indicating whether an LMSE (Least Mean Square Error) 
     * solution is allowed or not. When an LMSE solution is allowed, more than 8
     * matched points can be used for fundamental matrix estimation. If LMSE 
     * solution is not allowed then only the 7 former matched points will be 
     * taken into account.
     * @return true if an LMSE solution is allowed, false otherwise.
     */
    public boolean isLMSESolutionAllowed() {
        return mAllowLMSESolution;
    }
    
    /**
     * Sets boolean indicating whether an LMSE (LEast Mean Square Error) 
     * solution is allowed or not. When an LMSE solution is allowed, more than 8 
     * matched points can be used for fundamental matrix estimation. If LMSE 
     * solution is not allowed then only the 7 former matched points will be 
     * taken into account.
     * @param allowed true if an LMSE solution is allowed, false otherwise.
     * @throws LockedException if this instance is locked because an estimation
     * is in progress.
     */
    public void setLMSESolutionAllowed(boolean allowed) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mAllowLMSESolution = allowed;
    }

    /**
     * Indicates whether provided matched 2D points must be normalized to 
     * increase the accuracy of the estimation.
     * @return true if points must be normalized, false otherwise.
     */
    public boolean arePointsNormalized() {
        return mNormalizePoints;
    }
    
    /**
     * Sets boolean indicating whether provided matched 2D points must be
     * normalized to increase the accuracy of the estimation.
     * @param normalizePoints true if points must be normalized, false 
     * otherwise.
     * @throws LockedException if this instance is locked because an estimation
     * is in progress.
     */
    public void setPointsNormalized(boolean normalizePoints) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mNormalizePoints = normalizePoints;
    }
    
    /**
     * Returns boolean indicating whether estimator is ready to start the 
     * fundamental matrix estimation.
     * This is true when the required minimum number of matched points is 
     * provided to obtain a solution and both left and right views have the
     * same number of matched points.
     * @return true if estimator is ready to start the fundamental matrix
     * estimation, false otherwise.
     */    
    @Override
    public boolean isReady() {
        return mLeftPoints != null && mRightPoints != null &&
                mLeftPoints.size() == mRightPoints.size() &&
                mLeftPoints.size() >= MIN_REQUIRED_POINTS;
    }
    
    /**
     * Estimates all possible fundamental matrices found using provided points.
     * Because the algorithm uses 7 points and enforces rank 2 by solving a
     * third degree polynomial, the algorithm might return 1 real solution
     * (and 2 imaginary ones which are discarded), 3 real solutions, or 1 real
     * solution with triple multiplicity.
     * @return all posible fundamental matrices found using provided points.
     * @throws LockedException if estimator is locked doing an estimation.
     * @throws NotReadyException if estimator is not ready because required 
     * input points have not already been provided.
     * @throws FundamentalMatrixEstimatorException if configuration of provided
     * 2D points is degenerate and fundamental matrix estimation fails.
     */
    public List<FundamentalMatrix> estimateAll() throws LockedException,
            NotReadyException, FundamentalMatrixEstimatorException {
        List<FundamentalMatrix> result = new ArrayList<>();
        estimateAll(result);
        return result;
    }
    
    /**
     * Estimates all possible fundamental matrices found using provided points
     * and adds the result to provided result list.
     * Because the algorithm uses 7 points and enforces rank 2 by solving a
     * third degree polynomial, the algorithm might return 1 real solution
     * (and 2 imaginary ones which are discarded), 3 real solutions, or 1 real
     * solution with triple multiplicity.
     * @param result list where results will be stored.
     * @throws LockedException if estimator is locked doing an estimation.
     * @throws NotReadyException if estimator is not ready because required 
     * input points have not already been provided.
     * @throws FundamentalMatrixEstimatorException if configuration of provided
     * 2D points is degenerate and fundamental matrix estimation fails.
     */
    public void estimateAll(List<FundamentalMatrix> result) 
            throws LockedException, NotReadyException, 
            FundamentalMatrixEstimatorException {
        
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        mLocked = true;
        
        result.clear();
                
        if (mListener != null) {
            mListener.onEstimateStart(this);
        }
        
        int nPoints = mLeftPoints.size();
                
        try {
            ProjectiveTransformation2D leftNormalization = null, 
                    rightNormalization = null; 
            List<Point2D> leftPoints, rightPoints;
            if (mNormalizePoints) {
                //normalize points on left view
                Point2DNormalizer normalizer = new Point2DNormalizer(
                        mLeftPoints);
                normalizer.compute();                
                
                leftNormalization = normalizer.getTransformation();
                
                //normalize points on right view
                normalizer.setPoints(mRightPoints);
                normalizer.compute();
                
                rightNormalization = normalizer.getTransformation();
                
                //normalize to increase accuracy
                leftNormalization.normalize();
                rightNormalization.normalize();
                
                leftPoints = leftNormalization.transformPointsAndReturnNew(
                        mLeftPoints);
                rightPoints = rightNormalization.transformPointsAndReturnNew(
                        mRightPoints);
            } else {
                leftPoints = mLeftPoints;
                rightPoints = mRightPoints;
            }
            
            Matrix a;
            if (isLMSESolutionAllowed()) {
                a = new Matrix(nPoints, 9);
            } else {
                a = new Matrix(MIN_REQUIRED_POINTS, 9);
            }
            
            Point2D leftPoint, rightPoint;
            double homLeftX, homLeftY, homLeftW, homRightX, homRightY, 
                    homRightW;
            double value0, value1, value2, value3, value4, value5, value6, 
                    value7, value8, rowNorm;
            for (int i = 0; i < nPoints; i++) {
                leftPoint = leftPoints.get(i);
                rightPoint = rightPoints.get(i);
                
                //normalize points to increase accuracy
                leftPoint.normalize();
                rightPoint.normalize();
                                
                homLeftX = leftPoint.getHomX();
                homLeftY = leftPoint.getHomY();
                homLeftW = leftPoint.getHomW();
                
                homRightX = rightPoint.getHomX();
                homRightY = rightPoint.getHomY();
                homRightW = rightPoint.getHomW();
                
                //set a row values
                value0 = homLeftX * homRightX;
                value1 = homLeftY * homRightX;
                value2 = homLeftW * homRightX;
			
                value3 = homLeftX * homRightY;
                value4 = homLeftY * homRightY;
                value5 = homLeftW * homRightY;
			
                value6 = homLeftX * homRightW;
                value7 = homLeftY * homRightW;
                value8 = homLeftW * homRightW;
                
                //normalize row to increase accuracy
                rowNorm = Math.sqrt(Math.pow(value0, 2.0) +
                        Math.pow(value1, 2.0) + Math.pow(value2, 2.0) +
                        Math.pow(value3, 2.0) + Math.pow(value4, 2.0) +
                        Math.pow(value5, 2.0) + Math.pow(value6, 2.0) +
                        Math.pow(value7, 2.0) + Math.pow(value8, 2.0));
                
                a.setElementAt(i, 0, value0 / rowNorm);
                a.setElementAt(i, 1, value1 / rowNorm);
                a.setElementAt(i, 2, value2 / rowNorm);
                a.setElementAt(i, 3, value3 / rowNorm);
                a.setElementAt(i, 4, value4 / rowNorm);
                a.setElementAt(i, 5, value5 / rowNorm);
                a.setElementAt(i, 6, value6 / rowNorm);
                a.setElementAt(i, 7, value7 / rowNorm);
                a.setElementAt(i, 8, value8 / rowNorm);

                if(!isLMSESolutionAllowed() && i == (MIN_REQUIRED_POINTS - 1))
                    break;
            }
            
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            
            decomposer.decompose();
            
            //having 7 points for 9 variables means that rank can be as high as
            //7, and nullity must be 2, so that the final fundamental matrix
            //can be found as a linear combination of the null-space.
            //If nullity is bigger than 2, then geometry is degenerate, usually
            //due to colinearities or coplanarities on projected image points.
            //In this case we throw an exception
            if (decomposer.getNullity() > 2) {
                throw new FundamentalMatrixEstimatorException();
            }
            
            Matrix V = decomposer.getV();
            
            //The last two column vectors of V contain the "base" matrices
            //to be used for the retrieval of the true fundamental matrix, since
            //the fundamental matrix we are looking for will be a linear 
            //combination of such matrices after reshaping the vectors into 3x3 
            //matrices
            Matrix fundMatrix1 = new Matrix(
                    FundamentalMatrix.FUNDAMENTAL_MATRIX_ROWS,
                    FundamentalMatrix.FUNDAMENTAL_MATRIX_COLS);
            fundMatrix1.setElementAt(0, 0, V.getElementAt(0, 8));
            fundMatrix1.setElementAt(0, 1, V.getElementAt(1, 8));
            fundMatrix1.setElementAt(0, 2, V.getElementAt(2, 8));
            fundMatrix1.setElementAt(1, 0, V.getElementAt(3, 8));
            fundMatrix1.setElementAt(1, 1, V.getElementAt(4, 8));
            fundMatrix1.setElementAt(1, 2, V.getElementAt(5, 8));
            fundMatrix1.setElementAt(2, 0, V.getElementAt(6, 8));
            fundMatrix1.setElementAt(2, 1, V.getElementAt(7, 8));
            fundMatrix1.setElementAt(2, 2, V.getElementAt(8, 8));

            Matrix fundMatrix2 = new Matrix(
                    FundamentalMatrix.FUNDAMENTAL_MATRIX_ROWS,
                    FundamentalMatrix.FUNDAMENTAL_MATRIX_COLS);
            fundMatrix2.setElementAt(0, 0, V.getElementAt(0, 7));
            fundMatrix2.setElementAt(0, 1, V.getElementAt(1, 7));
            fundMatrix2.setElementAt(0, 2, V.getElementAt(2, 7));
            fundMatrix2.setElementAt(1, 0, V.getElementAt(3, 7));
            fundMatrix2.setElementAt(1, 1, V.getElementAt(4, 7));
            fundMatrix2.setElementAt(1, 2, V.getElementAt(5, 7));
            fundMatrix2.setElementAt(2, 0, V.getElementAt(6, 7));
            fundMatrix2.setElementAt(2, 1, V.getElementAt(7, 7));
            fundMatrix2.setElementAt(2, 2, V.getElementAt(8, 7));
            
            if (mNormalizePoints && leftNormalization != null) {
                //denormalize linear combination of fundamental matrices
                //fundMatrix1 and fundMatrix2
                Matrix transposedRightTransformationMatrix =
                        rightNormalization.asMatrix().transposeAndReturnNew();
                Matrix leftTransformationMatrix = leftNormalization.asMatrix();
                
                //compute fundMatrix1 = transposedRightTransformationMatrix *
                //fundMatrix1 * leftTransformationMatrix;
                fundMatrix1.multiply(leftTransformationMatrix);
                fundMatrix1 = transposedRightTransformationMatrix.
                        multiplyAndReturnNew(fundMatrix1);
                
                //normalize by frobenius norm to increase accuracy after point
                //denormalization
                double norm = Utils.normF(fundMatrix1);
                fundMatrix1.multiplyByScalar(1.0 / norm);
                
                //compute fundMatrix2 = transposedRightTransformationMatrix *
                //fundMatrix2 * leftTransformationMatrix;
                fundMatrix2.multiply(leftTransformationMatrix);
                transposedRightTransformationMatrix.multiply(fundMatrix2);
                fundMatrix2 = transposedRightTransformationMatrix;

                //normalize by frobenius norm to increase accuracy after point
                //denormalization
                norm = Utils.normF(fundMatrix2);
                fundMatrix2.multiplyByScalar(1.0 / norm);                
            }
            
            //because fundMatrix1, and fundMatrix2 have been obtained as 
            //columns of V, then its frobenius norm will be 1 because SVD 
            //already returns normalized singular vectors, and there is no need 
            //to normalize by frobenius norm if points are NOT normalized
            
            //The last thing we need to do is to enforce rank 2 on fundamental
            //matrix, since we know it always is a rank 2 matrix. For that 
            //reason we know that det(F) = 0.
            //Since the fundamental matrix F is a linear combination of the
            //two matrices F1, F2 we have found then: F = b * F1 + (1.0 - b) *F2
            //where b will range from 0 to 1. 
            //Hence: det(b * F1 + (1.0 - b) * F2) = 0
            //This produces a third degree polynomial as follows:

            //coefficients of polynomial: a*x^3 + b*x^2 + c*x + d
            double aPoly = 0.0, bPoly = 0.0, cPoly = 0.0, dPoly = 0.0;
            double[] params = new double[4];
            
            computeParams(fundMatrix1.getElementAt(0, 0), 
                    fundMatrix2.getElementAt(0, 0),
                    fundMatrix1.getElementAt(1, 1), 
                    fundMatrix2.getElementAt(1, 1),
                    fundMatrix1.getElementAt(2, 2), 
                    fundMatrix2.getElementAt(2, 2), params);
            
            aPoly += params[0];
            bPoly += params[1];
            cPoly += params[2];
            dPoly += params[3];
            
            computeParams(fundMatrix1.getElementAt(2, 1), 
                    fundMatrix2.getElementAt(2, 1),
                    fundMatrix1.getElementAt(1, 0),
                    fundMatrix2.getElementAt(1, 0),
                    fundMatrix1.getElementAt(0, 2),
                    fundMatrix2.getElementAt(0, 2), params);

            aPoly += params[0];
            bPoly += params[1];
            cPoly += params[2];
            dPoly += params[3];
            
            computeParams(fundMatrix1.getElementAt(2, 0),
                    fundMatrix2.getElementAt(2, 0),
                    fundMatrix1.getElementAt(0, 1),
                    fundMatrix2.getElementAt(0, 1),
                    fundMatrix1.getElementAt(1, 2),
                    fundMatrix2.getElementAt(1, 2), params);
            
            aPoly += params[0];
            bPoly += params[1];
            cPoly += params[2];
            dPoly += params[3];
            
            computeParams(fundMatrix1.getElementAt(2, 0), 
                    fundMatrix2.getElementAt(2, 0),
                    fundMatrix1.getElementAt(1, 1),
                    fundMatrix2.getElementAt(1, 1),
                    fundMatrix1.getElementAt(0, 2),
                    fundMatrix2.getElementAt(0, 2), params);
            
            aPoly -= params[0];
            bPoly -= params[1];
            cPoly -= params[2];
            dPoly -= params[3];
            
            computeParams(fundMatrix1.getElementAt(1, 2),
                    fundMatrix2.getElementAt(1, 2),
                    fundMatrix1.getElementAt(2, 1),
                    fundMatrix2.getElementAt(2, 1),
                    fundMatrix1.getElementAt(0, 0),
                    fundMatrix2.getElementAt(0, 0), params);
            
            aPoly -= params[0];
            bPoly -= params[1];
            cPoly -= params[2];
            dPoly -= params[3];
            
            computeParams(fundMatrix1.getElementAt(1, 0),
                    fundMatrix2.getElementAt(1, 0),
                    fundMatrix1.getElementAt(0, 1),
                    fundMatrix2.getElementAt(0, 1),
                    fundMatrix1.getElementAt(2, 2),
                    fundMatrix2.getElementAt(2, 2), params);
            
            aPoly -= params[0];
            bPoly -= params[1];
            cPoly -= params[2];
            dPoly -= params[3];

            //normalize polynomial coefficients to increase accuracy
            double coeffNorm = Math.sqrt(Math.pow(aPoly, 2.0) +
                    Math.pow(bPoly, 2.0) + Math.pow(cPoly, 2.0) +
                    Math.pow(dPoly, 2.0));
            aPoly /= coeffNorm;
            bPoly /= coeffNorm;
            cPoly /= coeffNorm;
            dPoly /= coeffNorm;
            
            //store polynomial coefficients into array and find its roots to
            //enforce det(F) = 0. The solution must be unique and real!
            params[0] = dPoly;
            params[1] = cPoly;
            params[2] = bPoly;
            params[3] = aPoly;
            
            double beta1 = 0.0, beta2 = 0.0, beta3 = 0.0;
            boolean beta1Available = false, beta2Available= false,
                    beta3Available = false;
            Complex[] roots;
            Complex root1, root2, root3;
            
            if (ThirdDegreePolynomialRootsEstimator.isThirdDegree(params)) {
                //solve third degree polynomial
                ThirdDegreePolynomialRootsEstimator estimator =
                        new ThirdDegreePolynomialRootsEstimator(params);
                estimator.estimate();
                roots = estimator.getRoots();
                root1 = roots[0];
                root2 = roots[1];
                root3 = roots[2];
                                
                if (Math.abs(root1.getImaginary()) <= EPS) {
                    //1st root is real, so we keep it
                    beta1 = root1.getReal();
                    beta1Available = true;
                }
                if (Math.abs(root2.getImaginary()) <= EPS) {
                    //2nd root is real, so we keep it
                    beta2 = root2.getReal();
                    beta2Available = true;
                }
                if (Math.abs(root3.getImaginary()) <= EPS) {
                    //3rd root is real, so we keep it
                    beta3 = root3.getReal();
                    beta3Available = true;
                }
            } else if (SecondDegreePolynomialRootsEstimator.isSecondDegree(
                    params)) {
                //solve second degree polynomial
                SecondDegreePolynomialRootsEstimator estimator =
                        new SecondDegreePolynomialRootsEstimator(params);
                estimator.estimate();
                roots = estimator.getRoots();
                root1 = roots[0];
                root2 = roots[1];
                if (Math.abs(root1.getImaginary()) <= EPS) {
                    //1st root is real, so we keep it
                    beta1 = root1.getReal();
                    beta1Available = true;
                }
                if (Math.abs(root2.getImaginary()) <= EPS) {
                    //2nd root is real, so we keep it
                    beta2 = root2.getReal();
                    beta2Available = true;
                }
            } else if (FirstDegreePolynomialRootsEstimator.isFirstDegree(
                    params)) {
                //solve first degree polynomial
                FirstDegreePolynomialRootsEstimator estimator =
                        new FirstDegreePolynomialRootsEstimator(params);
                estimator.estimate();
                roots = estimator.getRoots();
                root1 = roots[0]; //this is the only solution and is real 
                                //becuase coefficients are real
                beta1 = root1.getReal();
                beta1Available = true;
            } else {
                //invalid polynomial degree
                throw new FundamentalMatrixEstimatorException();
            }
            
            if (!beta1Available && !beta2Available && !beta3Available) {
                //No solution was found
                throw new FundamentalMatrixEstimatorException();
            }
            
            //Once the polynomial is solved we compute the linear combination
            //F = b * F1 + (1 - b) * F2 which has rank 2 using all available 
            //solutions
            Matrix fundMatrix;
            FundamentalMatrix F;
            result.clear(); //clear previous values
            if (beta1Available) {
                fundMatrix = fundMatrix1.multiplyByScalarAndReturnNew(beta1).
                        addAndReturnNew(fundMatrix2.
                                multiplyByScalarAndReturnNew(1.0 - beta1));
                if (enforceRank2(fundMatrix, decomposer)) {
                    throw new FundamentalMatrixEstimatorException();
                }
                
                F = new FundamentalMatrix(fundMatrix);
                result.add(F);
            }
            if (beta2Available) {
                fundMatrix = fundMatrix1.multiplyByScalarAndReturnNew(beta2).
                        addAndReturnNew(fundMatrix2.multiplyByScalarAndReturnNew(
                        1.0 - beta2));
                if (enforceRank2(fundMatrix, decomposer)) {
                    throw new FundamentalMatrixEstimatorException();
                }
                
                F = new FundamentalMatrix(fundMatrix);
                result.add(F);                
            }
            if (beta3Available) {
                fundMatrix = fundMatrix1.multiplyByScalarAndReturnNew(beta3).
                        addAndReturnNew(fundMatrix2.multiplyByScalarAndReturnNew(
                        1.0 - beta3));
                if (enforceRank2(fundMatrix, decomposer)) {
                    throw new FundamentalMatrixEstimatorException();
                }
                
                F = new FundamentalMatrix(fundMatrix);
                result.add(F);                
            }
            
            if (result.isEmpty()) {
                //no valid solution was found
                throw new FundamentalMatrixEstimatorException();
            }
                        
            if (mListener != null) {
                mListener.onEstimateEnd(this, result.get(0));
            }
            
        } catch (InvalidFundamentalMatrixException | AlgebraException |
                NumericalException | NormalizerException e) {
            throw new FundamentalMatrixEstimatorException(e);
        } finally {
            mLocked = false;
        }        
    }
    
    /**
     * Estimates a fundamental matrix using provided lists of matched points on
     * left and right views.
     * This method returns a solution only if one possible solution exists.
     * If more than one solution is available, this method will fail.
     * Because this algorithm might return more than one solution, it is highly
     * encouraged to use estimateAll method instead.
     * @return a fundamental matrix.
     * @throws LockedException if estimator is locked doing an estimation.
     * @throws NotReadyException if estimator is not ready because required 
     * input points have not already been provided.
     * @throws FundamentalMatrixEstimatorException if configuration of provided
     * 2D points is degenerate and fundamental matrix estimation fails or more
     * than one solution exists.
     */    
    @Override
    public FundamentalMatrix estimate() throws LockedException, 
            NotReadyException, FundamentalMatrixEstimatorException {
        List<FundamentalMatrix> list = estimateAll();
        if (list.size() > 1) {
            throw new FundamentalMatrixEstimatorException();
        }
        
        return list.get(0);
    }

    /**
     * Returns method of non-robust fundamental matrix estimator.
     * @return method of fundamental matrix estimator.
     */    
    @Override
    public FundamentalMatrixEstimatorMethod getMethod() {
        return FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM;
    }    
    
    /**
     * Returns minimum number of matched pair of points required to start
     * the estimation. This implementation requires a minimum of 7 points.
     * @return minimum number of matched pair of points required to start
     * the estimation. Always returns 7.
     */
    @Override
    public int getMinRequiredPoints() {
        return MIN_REQUIRED_POINTS;
    }
        
    /**
     * Enforces rank 2 into provided matrix.
     * This method modifies provided matrix.
     * @param matrix matrix to be enforced to have rank 2.
     * @param decomposer an SVD decomposer.
     * @return false if rank was successfully enforced, true otherwise.
     * @throws AlgebraException if an error occurs during SVD decomposition
     * because of numerical instabilities.
     */
    private boolean enforceRank2(Matrix matrix, 
            SingularValueDecomposer decomposer) throws AlgebraException {
        
        decomposer.setInputMatrix(matrix);
        decomposer.decompose();
        
        int rank = decomposer.getRank();
        if (rank > FundamentalMatrix.FUNDAMENTAL_MATRIX_RANK) {
            //rank needs to be reduced
            Matrix U = decomposer.getU();
            Matrix W = decomposer.getW();
            Matrix V = decomposer.getV();
                
            //transpose V
            V.transpose();

            //set last singular value to zero to enforce rank 2
            W.setElementAt(2, 2, 0.0);
                
            //compute matrix = U * W * V'
            W.multiply(V);
            U.multiply(W);
            matrix.copyFrom(U);
            return false;
        } else {
            //if rank is 2, rank is ok, otherwise rank is lower than fundamental 
            //matrix rank (rank 1) and estimation has failed because of 
            //coplanarities
            return rank != FundamentalMatrix.FUNDAMENTAL_MATRIX_RANK;
        }                
    }
    
    /**
     * Computes parameters of third degree polynomial to obtain possible 
     * solutions.
     * @param a 1st param.
     * @param b 2nd param.
     * @param c 3rd param.
     * @param d 4th param.
     * @param e 5th param.
     * @param f 6th param.
     * @param out array of length 4 where partial parameters of third degree
     * polynomial are stored.
     */
    private void computeParams(double a, double b, double c, double d, double e, 
            double f, double[] out) {
        
        double ace = a*c*e;
        double ade = a*d*e;
        double bce = b*c*e;
        double bde = b*d*e;
        double acf = a*c*f;
        double adf = a*d*f;
        double bcf = b*c*f;
        double bdf = b*d*f;
        
        out[0] = ace - ade - bce + bde - acf + adf + bcf - bdf;
        out[1] = ade + bce - 2.0*bde + acf - 2.0*adf - 2.0*bcf + 3.0*bdf;
        out[2] = bde + adf + bcf -3.0*bdf;
        out[3] = bdf;        
    }
}
