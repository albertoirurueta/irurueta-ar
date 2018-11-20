/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.Utils;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidFundamentalMatrixException;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NormalizerException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.Point2DNormalizer;

import java.util.List;

/**
 * Non-robust fundamental matrix estimator for Affine camera projection model.
 * This implementation uses 4 matched 2D points on left and right views.
 */
public class AffineFundamentalMatrixEstimator extends 
        FundamentalMatrixEstimator {

    /**
     * Constant indicating that by default an LMSE solution is not allowed.
     */
    public static final boolean DEFAULT_ALLOW_LMSE_SOLUTION = false;
    
    /**
     * Minimum number of matched 2D points to start the estimation.
     */
    public static final int MIN_REQUIRED_POINTS = 4;
    
    /**
     * Indicates if by default provided point correspondences are normalized to
     * increase the accuracy of the estimation.
     */
    public static final boolean DEFAULT_NORMALIZE_POINT_CORRESPONDENCES = true;
    
    /**
     * Indicates whether an LMSE (Least Mean Square Error) solution is allowed
     * or not.
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
    public AffineFundamentalMatrixEstimator() {
        super();
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
        mNormalizePoints = DEFAULT_NORMALIZE_POINT_CORRESPONDENCES;
    }
    
    /**
     * Constructor with matched 2D points.
     * @param leftPoints 2D points on left view.
     * @param rightPoints 2D points on right view.
     * @throws IllegalArgumentException  if provided list of points do not 
     * have the same length.
     */
    public AffineFundamentalMatrixEstimator(List<Point2D> leftPoints,
            List<Point2D> rightPoints) throws IllegalArgumentException {
        super(leftPoints, rightPoints);
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
        mNormalizePoints = DEFAULT_NORMALIZE_POINT_CORRESPONDENCES;        
    }
    
    /**
     * Returns boolean indicating whether an LMSE (Least Mean Square Error) 
     * solution is allowed or not. When an LMSE solution is allowed, more than 8
     * matched points can be used for fundamental matrix estimation. If LMSE 
     * solution is not allowed then only the 4 former matched points will be 
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
     * solution is not allowed then only the 4 former matched points will be 
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
     * Estimates a fundamental matrix using provided lists of matched points on
     * left and right views.
     * @return a fundamental matrix.
     * @throws LockedException if estimator is locked doing an estimation.
     * @throws NotReadyException if estimator is not ready because required 
     * input points have not already been provided.
     * @throws FundamentalMatrixEstimatorException if configuration of provided
     * 2D points is degenerate and fundamental matrix estimation fails.
     */        
    @Override
    public FundamentalMatrix estimate() throws LockedException, 
            NotReadyException, FundamentalMatrixEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        mLocked = true;
                
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
                a = new Matrix(nPoints, 5);
            } else {
                a = new Matrix(MIN_REQUIRED_POINTS, 5);
            }
            
            Point2D leftPoint, rightPoint;
            double homLeftX, homLeftY, homLeftW, homRightX, homRightY, 
                    homRightW;
            double value0, value1, value2, value3, value4, rowNorm;
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
                value0 = homLeftW * homRightX; //homLeftX * homRightX;
                value1 = homLeftW * homRightY; //homLeftY * homRightX;
                value2 = homLeftX * homRightW;
                value3 = homLeftY * homRightW;
                value4 = homLeftW * homRightW;
                                
                //normalize row to increase accuracy
                rowNorm = Math.sqrt(Math.pow(value0, 2.0) +
                        Math.pow(value1, 2.0) + Math.pow(value2, 2.0) +
                        Math.pow(value3, 2.0) + Math.pow(value4, 2.0));
                
                a.setElementAt(i, 0, value0 / rowNorm);
                a.setElementAt(i, 1, value1 / rowNorm);
                a.setElementAt(i, 2, value2 / rowNorm);
                a.setElementAt(i, 3, value3 / rowNorm);
                a.setElementAt(i, 4, value4 / rowNorm);                   

                if (!isLMSESolutionAllowed() && 
                        i == (MIN_REQUIRED_POINTS - 1)) {
                    break;
                }
            }
            
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            
            decomposer.decompose();
            
            //if nullity of provided a matrix is not of dimension 1 (number of
            //dimensions of null-spance), then epipolar geometry is degenerate
            //because there is more than one possible solution (up to scale).
            //This is typically due to clinearities or coplanarities on 
            //projected 2D points. In this case we throw an exception
            if (decomposer.getNullity() > 1) {
                throw new FundamentalMatrixEstimatorException();
            }
            
            Matrix V = decomposer.getV();
            
            //The fundamental matrix is contained in vector form on the last
            //column of V, we reshape such vector into a 3x3 matrix
            Matrix fundMatrix = new Matrix(
                    FundamentalMatrix.FUNDAMENTAL_MATRIX_ROWS,
                    FundamentalMatrix.FUNDAMENTAL_MATRIX_COLS);
            fundMatrix.setElementAt(0, 2, V.getElementAt(0, 4));
            fundMatrix.setElementAt(1, 2, V.getElementAt(1, 4));
            fundMatrix.setElementAt(2, 0, V.getElementAt(2, 4));
            fundMatrix.setElementAt(2, 1, V.getElementAt(3, 4));
            fundMatrix.setElementAt(2, 2, V.getElementAt(4, 4));
            
            if (mNormalizePoints && leftNormalization != null) {
                //denormalize fundMatrix
                Matrix transposedRightTransformationMatrix =
                        rightNormalization.asMatrix().transposeAndReturnNew();
                Matrix leftTransformationMatrix = leftNormalization.asMatrix();
                
                //compute fundMatrix = transposedRightTransformationMatrix *
                //fundMatrix * leftTransformationMatrix;
                fundMatrix.multiply(leftTransformationMatrix);
                transposedRightTransformationMatrix.multiply(fundMatrix);
                fundMatrix = transposedRightTransformationMatrix;
                
                //normalize by frobenius norm to increase accuracy after point
                //denormalization
                double norm = Utils.normF(fundMatrix);
                fundMatrix.multiplyByScalar(1.0 / norm);
            }
            
            //enforce rank 2
            decomposer.setInputMatrix(fundMatrix);
            
            decomposer.decompose();
            
            //if rank is not already correct, then we enforce it
            int rank = decomposer.getRank();
            if (rank > FundamentalMatrix.FUNDAMENTAL_MATRIX_RANK) {
                //rank needs to be reduced
                Matrix U = decomposer.getU();
                Matrix W = decomposer.getW();
                V = decomposer.getV();
                
                //transpose V
                V.transpose();
                Matrix transV = V;
                
                //set last singular value to zero to enforce rank 2
                W.setElementAt(2, 2, 0.0);
                
                //compute fundMatrix = U * W * V'
                W.multiply(transV);
                U.multiply(W);
                fundMatrix = U;
            } else if (rank < FundamentalMatrix.FUNDAMENTAL_MATRIX_RANK) {
                //rank is 1, which is lower than required fundamental matrix 
                //rank (rank 2)
                throw new FundamentalMatrixEstimatorException();
            }
            
            FundamentalMatrix result = new FundamentalMatrix(fundMatrix);
            
            if (mListener != null) {
                mListener.onEstimateEnd(this, result);
            }
            
            return result;

        } catch (InvalidFundamentalMatrixException | AlgebraException | NormalizerException e) {
            throw new FundamentalMatrixEstimatorException(e);
        } finally {
            mLocked = false;
        }
    }

    /**
     * Returns method of non-robust fundamental matrix estimator.
     * @return method of fundamental matrix estimator.
     */        
    @Override
    public FundamentalMatrixEstimatorMethod getMethod() {
        return FundamentalMatrixEstimatorMethod.AFFINE_ALGORITHM;
    }

    /**
     * Returns minimum number of matched pair of points required to start
     * the estimation. This implementation requires a minimum of 4 points
     * @return minimum number of matched pair of points required to start
     * the estimation. Always returns 4.
     */    
    @Override
    public int getMinRequiredPoints() {
        return MIN_REQUIRED_POINTS;
    }    
}
